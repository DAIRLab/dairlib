#include "systems/controllers/mpc_balance_controller.h"
//#include "systems/controllers/admmsolve.h"
#include <ctime>
using namespace std;

namespace dairlib{
namespace systems{


// function used in this file for solving QCQP
double fn(Eigen::VectorXd lambda, Eigen::VectorXd q, Eigen::VectorXd z, double nu){
	double fval = 0;
	int varnum = lambda.rows();
	for (int i = 0; i < varnum; i++){
		fval += lambda(i)*(nu*q(i)-2*z(i))*(nu*q(i)-2*z(i)) / (4*(1+nu*lambda(i))*(1+nu*lambda(i))) 
				- q(i)*(nu*q(i)-2*z(i)) / (2*(1+nu*lambda(i)));
	}
	return fval;
}	;

// function used in this file for solving QCQP
double DoBisection(double lb, double ub, Eigen::VectorXd lambda, Eigen::VectorXd q, Eigen::VectorXd z){
	double epsilon = 1e-6;

	while (1){
        
		double nu = (lb + ub) / 2;
		double tmp = fn(lambda, q, z, nu);
		if (abs( tmp ) < epsilon){
			return nu;
		}
		else{
			if ( tmp > 0 ){
				lb = nu;
			}
			else{
				ub = nu;
			}
		}
	}

};

MpcBalanceController::MpcBalanceController(int num_positions, int num_velocities,
                                   int num_inputs,RigidBodyTree<double>& tree,VectorXd x_des,VectorXd u_des, VectorXd lambda_des) {

    output_input_port_ = this->DeclareVectorInputPort(
        OutputVector<double>(num_positions, num_velocities,
                           num_inputs)).get_index();

    this->DeclareVectorOutputPort(TimestampedVector<double>(num_inputs),
        &MpcBalanceController::CalcControl);

    //x_des_ are the desired positon and velocity
    num_inputs_ = num_inputs;
    num_states_ = num_positions + num_velocities;
    x_des_ = x_des;
    u_des_ = u_des;
    lambda_des_ = lambda_des;
    cout << endl;

    const VectorXd q = x_des.head(num_positions);
    const VectorXd v = x_des.tail(num_velocities); 

    KinematicsCache<double> cache = tree.doKinematics(q, v, true);

    //get the dynamics equations and linearize it(or linearized constraint directly)(should include n(qk),D(qk),M*(qk),C(qk,qkdot),delta_V)
    VectorXd phi_total; //distance from object to environment
    Matrix3Xd normal_total, xA_total, xB_total; // contacts between bodyA and bodyB
    vector<int> idxA_total, idxB_total; //

    // This (const cast) is an ugly way of doing it. Change it later if a better method is available. Return a boolean 
    // Collision detect (double template as AutoDiff doesnt work)
    cout << const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
        cache, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);
    cout << endl;

    const int num_total_contacts = normal_total.cols();
    // 4 contacts for Cassie (2 in each toe)
    const int num_contacts = 4;
    dlim_ = MatrixXd::Zero(num_contacts,1);
    num_contacts_nor_ = num_contacts;
    num_contacts_tan_ = num_contacts*4;
    // Getting the indices of the world and toes
    const int world_ind = tree.FindBodyIndex("world");
    const int toe_left_ind = tree.FindBodyIndex("toe_left");
    const int toe_right_ind = tree.FindBodyIndex("toe_right");

    // find contact between the toes and world frame 
    vector<int> contact_ind(num_contacts);
    int k=0;

    // which body indicates the world
    
    for (int i=0; i<num_total_contacts; i++) {
        int ind_a = idxA_total.at(i);
        //cout << "a is " << tree.getBodyOrFrameName(ind_a) << endl;
        int ind_b = idxB_total.at(i);
        //cout << "b is " << tree.getBodyOrFrameName(ind_b) << endl;
        if ((ind_a == world_ind && ind_b == toe_left_ind) ||
            (ind_a == world_ind && ind_b == toe_right_ind) ||
            (ind_a == toe_left_ind && ind_b == world_ind) ||
            (ind_a == toe_right_ind && ind_b == world_ind)) {
                contact_ind.at(k) = i;
                k++;
        }
    }

    Matrix3Xd normal = Matrix3Xd::Zero(normal_total.rows(), num_contacts);
     for (int i=0; i<num_contacts; i++) {
       normal.col(i) = normal_total.col(contact_ind.at(i));
       dlim_(i,0) = phi_total(contact_ind.at(i));
      }
     
    // Map an array data to a matrix. What is the difference between normal and normal_map
    const Map<Matrix3Xd> normal_map(normal.data(), normal_total.rows(), num_contacts);
    
    // create tangent vector
    // TODO: more tangent vector to create a more accurate friction cone
    vector<Map<Matrix3Xd>> tangents;
    Matrix3Xd tmp_mat1 = Matrix3Xd::Zero(3, 4);
    Map<Matrix3Xd> tmp_map1(tmp_mat1.data(), 3, 4);
    Matrix3Xd tmp_mat2 = Matrix3Xd::Zero(3, 4);
    Map<Matrix3Xd> tmp_map2(tmp_mat2.data(), 3, 4);
    tangents.push_back(tmp_map1);
    tangents.push_back(tmp_map2);

    tree.surfaceTangents(normal_map, tangents);         
    // Computing the position Jacobian
    vector<MatrixXd> Jd(num_contacts);
    for (int i=0; i<num_contacts; i++) {
        auto tmp_JA = tree.transformPointsJacobian(cache,
                                                xA_total.col(contact_ind.at(i)), 
                                                idxA_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
        // transform the points from toe to the world frame
        auto tmp_JB = tree.transformPointsJacobian(cache,
                                                xB_total.col(contact_ind.at(i)), 
                                                idxB_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
        Jd.at(i) = tmp_JA - tmp_JB;
    }

    // Computing the jacobians(tangent and normal) for each contact point
    MatrixXd J(num_contacts, tree.get_num_positions());
    MatrixXd D(num_contacts*4, tree.get_num_positions());
    for (int i=0; i<num_contacts; i++) {
        MatrixXd J_pt(1, tree.get_num_positions());
        MatrixXd D_pt(4, tree.get_num_positions());
        auto normal_pt = normal.col(contact_ind.at(i));
        auto tangent1_pt = tangents.at(0).col(contact_ind.at(i));
        auto tangent2_pt = tangents.at(1).col(contact_ind.at(i));
        auto tangent3_pt = -tangents.at(0).col(contact_ind.at(i));
        auto tangent4_pt = -tangents.at(1).col(contact_ind.at(i));
        J_pt.row(0) = normal_pt.transpose()*Jd.at(i);

        D_pt.row(0) = tangent1_pt.transpose()*Jd.at(i);
        D_pt.row(1) = tangent2_pt.transpose()*Jd.at(i);
        D_pt.row(2) = tangent3_pt.transpose()*Jd.at(i);
        D_pt.row(3) = tangent4_pt.transpose()*Jd.at(i);
        
        J.block(i, 0, 1, tree.get_num_positions()) =  J_pt;  
        D.block(i*4, 0, 4, tree.get_num_positions()) =  D_pt;       
    }
    
//TODO: check if it should be -J or J, after I change the sign of J, it becomes feasible
    //J = -J;

    dlim_ =  dlim_ - J * q;
    //cout << "J: " << J <<endl;
    //cout << "D: " << D <<endl;
    // TODO:add force variables to autodiff and constraint
    const int num_normalforce = num_contacts;
    const int num_tangentforce = num_contacts*4;
    const int num_treeforce = 2;
    VectorXd x(num_positions + num_velocities + num_inputs + num_normalforce + num_tangentforce + num_treeforce);
    VectorXd q1 = q;
    VectorXd v1 = v;
    // use zero: treat this as a constant     dont use zero:
    VectorXd u = u_des;
    VectorXd cn = VectorXd::Zero(num_normalforce);
    for(int i = 0;i<num_normalforce; i++){
        cn(i) = dt_*lambda_des(2 + i*3);
    }
    VectorXd betan = VectorXd::Zero(num_tangentforce);
    VectorXd cn_tree = dt_*lambda_des.head(2);

    x << q1, v1, u, cn, betan, cn_tree ;
    
    drake::AutoDiffVecXd x_autodiff = drake::math::initializeAutoDiff(x);  
    drake::AutoDiffVecXd q_autodiff = x_autodiff.head(num_positions);
    drake::AutoDiffVecXd v_autodiff = x_autodiff.segment(num_positions,num_velocities);
    drake::AutoDiffVecXd input_autodiff = x_autodiff.segment(num_positions+num_velocities,num_inputs);
    drake::AutoDiffVecXd normal_autodiff = x_autodiff.segment(num_positions+num_velocities+num_inputs,num_normalforce);
    drake::AutoDiffVecXd tangent_autodiff = x_autodiff.segment(num_positions+num_velocities+num_inputs+num_normalforce,num_tangentforce);
    drake::AutoDiffVecXd tree_force_autodiff = x_autodiff.tail(num_treeforce);
    // get kinematics cache for autodiff
    KinematicsCache<drake::AutoDiffXd> cache_autodiff = tree.doKinematics(q_autodiff, v_autodiff, true);
    auto J_tree_auto = tree.positionConstraintsJacobian(cache_autodiff);
    MatrixXd J_tree = tree.positionConstraintsJacobian(cache);

    // M*vdot  = C + B*u + J^T*f + D(q)^T*fb + J_tree^T*lambda.
    // linearize M,C,B,J,D: how to handle linearization of a matrix?
    // Then linearize this vector function: how to find a equibibrium(input and contact force)，solve linear function
    const typename RigidBodyTree<drake::AutoDiffXd>::BodyToWrenchMap no_external_wrenches_auto;
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    drake::AutoDiffVecXd f = tree.massMatrix(cache_autodiff).inverse()*(-dt_*tree.dynamicsBiasTerm(cache_autodiff, no_external_wrenches_auto) + dt_*tree.B*input_autodiff)
        + tree.massMatrix(cache_autodiff).inverse()*(J.transpose()*normal_autodiff + D.transpose()*tangent_autodiff + J_tree_auto.transpose()*tree_force_autodiff);

    VectorXd fcons = tree.massMatrix(cache).inverse()*(-dt_*tree.dynamicsBiasTerm(cache, no_external_wrenches) + dt_*tree.B*u)
        + tree.massMatrix(cache).inverse()*(J.transpose()*cn + D.transpose()*betan + J_tree.transpose()*cn_tree);


    //TODO : some eigenvalues of massMatrix are really small,may cause problem for the following optimization problem

    // still need to add constant term
    // linearize it
    auto GradientMatrix = drake::math::autoDiffToGradientMatrix(f);
    const_ = (fcons - GradientMatrix*x);


    A_ = GradientMatrix.block(0,0,num_positions,num_positions+num_positions);
    B_ = GradientMatrix.block(0,num_positions+num_positions,num_positions,num_inputs);    
    n_ = J.transpose();
    D_ = D.transpose();
    invMn_ = GradientMatrix.block(0,num_positions+num_positions+num_inputs,num_positions,num_normalforce);
    invMD_ = GradientMatrix.block(0,num_positions+num_positions+num_inputs+num_normalforce,num_positions,num_tangentforce);
    invMJtree_ = GradientMatrix.block(0,num_positions+num_positions+num_inputs+num_normalforce+num_tangentforce,num_positions,num_treeforce);
    
    //other part(bound constraint on variable), limit of torque and so on
    qmax_ = VectorXd::Zero(num_states_);
    qmin_ = VectorXd::Zero(num_states_);
    qmax_.segment(0,num_states_/2) = tree.joint_limit_max;
    qmin_.segment(0,num_states_/2) = tree.joint_limit_min;

    qmax_.segment(num_states_/2,num_states_/2) = 100 * VectorXd::Ones(num_states_/2);
    qmin_.segment(num_states_/2,num_states_/2) = -100 * VectorXd::Ones(num_states_/2);

    umax_ = 500 * VectorXd::Ones(num_inputs_);
    umin_ = -500 * VectorXd::Ones(num_inputs_);

    cmax_ = 500000 *  VectorXd::Ones(num_contacts_nor_*2+num_contacts_tan_);
    cmin_ = 0 *  VectorXd::Ones(num_contacts_nor_*2+num_contacts_tan_);
}


void MpcBalanceController::CalcControl(const Context<double>& context,
                                  TimestampedVector<double>* control) const {
    const OutputVector<double>* output = (OutputVector<double>*)
        this->EvalVectorInput(context, output_input_port_);
    // formulate the large mpc matrix, assume now we have 
    MatrixXd dynamic_matrix[14];
    int N = 6;
    int rho = 100;
    VectorXd current_state = output->GetState();
    get_optimization_matrix(dynamic_matrix,N,current_state);
    //admm_solve(dynamic_matrix,N,current_state,rho);
    // call admmsolver to optimize problem to get u
    //VectorXd u = admm_solve(dynamic_matrix,N,current_state,rho);
    //cout << u << endl;
    VectorXd u = u_des_;
    //send u as command
    control->SetDataVector(u);
    control->set_timestamp(output->get_timestamp());
}

VectorXd MpcBalanceController::admm_solve(MatrixXd dynamic_matrix[13],int N,VectorXd x0,int rho) const{

    //TODO: check all the variables and matrixs are valid:
    //TODO: modify rho and other ADMM tricks in the future
    
    // get relative matrix
    MatrixXd Amat = dynamic_matrix[0];
    MatrixXd Bmat = dynamic_matrix[1];
    MatrixXd Cmat = dynamic_matrix[2];
    MatrixXd con = dynamic_matrix[3];
    MatrixXd dlim = dynamic_matrix[4];
    MatrixXd D = dynamic_matrix[5];
    // qcqp constraint
    MatrixXd H1 = dynamic_matrix[6];
    MatrixXd h1 = dynamic_matrix[7];
    MatrixXd H2 = dynamic_matrix[8];
    MatrixXd H3 = dynamic_matrix[9];

    // linear constraint constraint
    MatrixXd Ae = dynamic_matrix[10];
    MatrixXd be = dynamic_matrix[11];
    MatrixXd Ain = dynamic_matrix[12];
    MatrixXd bin = dynamic_matrix[13];   

    MatrixXd Q = 10*MatrixXd::Identity(num_states_,num_states_);
    MatrixXd R = 1*MatrixXd::Identity(num_inputs_,num_inputs_);

	int varnum = num_states_ + num_inputs_ + 2* num_contacts_nor_ + num_contacts_tan_ + 2;		
    // eigenvalue decomposition of H1
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(H1);
    //if (eigensolver.info != Eigen::Success) abort();
    MatrixXd H1egvec = eigensolver.eigenvectors();
    VectorXd H1egval = eigensolver.eigenvalues();
    MatrixXd H1egvaldiag = H1egval.asDiagonal();
    VectorXd h1hat = H1egvec.transpose()*h1;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver2(H2);
    //	if (eigensolver.info != Eigen::Success) abort();
    MatrixXd H2egvec = eigensolver2.eigenvectors();
    VectorXd H2egval = eigensolver2.eigenvalues();
    MatrixXd H2egvaldiag = H2egval.asDiagonal();
    VectorXd h2hat = VectorXd::Zero(num_states_/2 + num_contacts_tan_ + num_contacts_nor_);


    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver3(H3);
    //	if (eigensolver.info != Eigen::Success) abort();
    MatrixXd H3egvec = eigensolver3.eigenvectors();
    VectorXd H3egval = eigensolver3.eigenvalues();
    MatrixXd H3egvaldiag = H3egval.asDiagonal();
    VectorXd h3hat = VectorXd::Zero(num_contacts_nor_*2 + num_contacts_tan_);

    // initialize dual vector
	VectorXd zk = VectorXd::Zero(varnum*N); 
    VectorXd zk1 = VectorXd::Zero(varnum*N);
    VectorXd zk2 = VectorXd::Zero(varnum*N);
	VectorXd muk = VectorXd::Zero(varnum*N);	
    VectorXd muk1 = VectorXd::Zero(varnum*N);
    VectorXd muk2 = VectorXd::Zero(varnum*N);
    VectorXd temp_x_mu = VectorXd::Zero(num_states_/2 + num_contacts_nor_);
    VectorXd temp_x_mu1 = VectorXd::Zero(num_states_/2 + num_contacts_tan_ + num_contacts_nor_);
    VectorXd temp_x_mu2 = VectorXd::Zero(num_contacts_nor_*2 + num_contacts_tan_);
    VectorXd local_zk = VectorXd::Zero(num_states_/2 + num_contacts_nor_);
    VectorXd local_zk1 = VectorXd::Zero(num_states_/2 + num_contacts_tan_ + num_contacts_nor_);
    VectorXd local_zk2 = VectorXd::Zero(num_contacts_nor_*2 + num_contacts_tan_);

    VectorXd xmax = VectorXd::Zero(varnum*N);
    VectorXd xmin = VectorXd::Zero(varnum*N);

    const int dimA = num_states_;
    const int dimB = num_inputs_;
    const int dimC = num_contacts_nor_*2 + num_contacts_tan_;

    // limit value
    for(int i = 0; i < N; i++){
        xmax.segment(i*dimA,dimA) = qmax_;
        xmax.segment(N*dimA+i*dimC,dimC) = cmax_;
        xmax.segment(N*dimA+N*dimC+i*dimB,dimB) = umax_;
        xmax.segment(N*(dimA+dimB+dimC)+i*2,2) = 5000*VectorXd::Ones(2);

        xmin.segment(i*dimA,dimA) = qmin_;
        xmin.segment(N*dimA+i*dimC,dimC) = cmin_;
        xmin.segment(N*dimA+N*dimC+i*dimB,dimB) = umin_;
        xmin.segment(N*(dimA+dimB+dimC)+i*2,2) = -5000*VectorXd::Ones(2);
    }

    double *varmax = xmax.data();
	double *varmin = xmin.data();
	
	// preallocate variables and predefine GRB constraints to save time. 
	VectorXd xvarop = VectorXd::Zero(varnum*N);
	//Eigen::VectorXd lastz = zk;
	double objval = 0;
	int iter = 0;
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);
	// add variables    

	GRBVar *xvar = model.addVars(varmin, varmax, NULL, NULL, NULL, varnum*N);
	int innum = N*(num_contacts_nor_*2 + num_contacts_tan_);
    int eqnum = N*num_states_;


    for (int i = 0; i < innum+eqnum; i++){
		GRBLinExpr linconstr = 0;
		if (i < innum){
			for (int j = 0; j < varnum*N; j++){
                if(abs(Ain(i,j))>1e-7)
				    linconstr += Ain(i,j) * xvar[j];
			}		
			model.addConstr(linconstr, '>', bin(i));
			}
		else{            
            
			int k = i - innum;
			for (int j = 0; j < varnum*N; j++){
                if(abs(Ae(k,j))>1e-7)
				    linconstr += Ae(k,j) * xvar[j];
			}
			model.addConstr(linconstr, '=', be(k));
            
		}
	}
	// solve ADMM
    
	while (iter <= 10){
		++iter;
		// step 1: find x(k+1) 
		try{
			// set objective 
			GRBQuadExpr obj = 0;
			for (int i = 0; i < varnum*N; ++i){
				double qi =  - rho * ( *(zk.data()+i)+*(muk.data()+i)) - rho * ( *(zk1.data()+i) + *(muk1.data()+i)) - rho * ( *(zk2.data()+i) + *(muk2.data()+i));
				obj += qi * xvar[i];
			}

            // add quadratic cost on x
            for(int i = 0;i<num_states_;i++){
                for(int j=0;j<N;j++){
                    obj += Q(i,i) * (xvar[j*num_states_+i] - x_des_(i)) * (xvar[j*num_states_+i] - x_des_(i));
                } 
            }

            // add quadratic cost on input
            for(int i=0;i<num_inputs_;i++){
                for(int j=0;j<N;j++){
                    obj += R(i,i) * (xvar[N*(dimA+dimC)+j*dimB+i] - u_des_(i)) * (xvar[N*(dimA+dimC)+j*dimB+i] - u_des_(i)) ;
                }
            }

            // from admm, add additional quadratic term on all diagonal matrix
            for(int i=0;i<varnum*N;i++){
                    obj += 1.5 * rho * xvar[i] * xvar[i];
            }

			//model.set(GRB_DoubleParam_BarConvTol, 1e-1);
			//model.set(GRB_DoubleParam_FeasibilityTol, 1e-6);
            model.set(GRB_IntParam_DualReductions, 0);
			model.setObjective(obj, GRB_MINIMIZE);
			model.set(GRB_IntParam_OutputFlag, 0);
			model.reset();
            
            
			model.optimize();
			
			if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL){
				// save the optimal solution
				objval = model.get(GRB_DoubleAttr_ObjVal);
				for (int i = 0; i < varnum*N; ++i){
					xvarop(i) = xvar[i].get(GRB_DoubleAttr_X);
				}
			}
			else if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE){
				cout << "Problem infeasible at phase 1." << endl;
                delete[] xvar;
				//return;
				return Eigen::VectorXd::Random(dimB);
			}
			else{
				cout << "Problem unbounded at phase 1." << endl;
                delete[] xvar;
				//return; 
				return Eigen::VectorXd::Random(dimB);
			} 
		}catch(GRBException e){
            delete[] xvar;
			cout << "Error code = " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
		}
        
        // variables not involved in quadratic constraint should be the same as the xvarop
        zk = xvarop - muk;
        zk1 = xvarop - muk1;
        zk2 = xvarop - muk2;

        // update z at each step since z of different time step are independent
        for(int i = 0; i<N ; i++){
            // evaluate feasibility of xvarop
            VectorXd feasi_x_H1 = VectorXd::Zero(num_states_/2 + num_contacts_nor_);
            VectorXd feasi_x_H2 = VectorXd::Zero(num_states_/2 + num_contacts_tan_ + num_contacts_nor_);
            VectorXd feasi_x_H3 = VectorXd::Zero(num_contacts_nor_*2 + num_contacts_tan_);
            feasi_x_H1.segment(0,num_states_/2) = xvarop.segment(i*dimA,num_states_/2);
            feasi_x_H1.segment(num_states_/2,num_contacts_nor_) = xvarop.segment(N*dimA + i*dimC,num_contacts_nor_);
            
            feasi_x_H2.segment(0,num_states_/2) = xvarop.segment(i*dimA+num_states_/2,num_states_/2);
            feasi_x_H2.segment(num_states_/2,num_contacts_nor_+num_contacts_tan_) = xvarop.segment(N*dimA+i*dimC+num_contacts_nor_,num_contacts_nor_+num_contacts_tan_);
            
            feasi_x_H3 = xvarop.segment(N*dimA + i*dimC,dimC);

            //cout << "feasibility of first condition " << endl;
            //cout << feasi_x_H1.transpose() * H1 * feasi_x_H1 + feasi_x_H1.transpose()*h1<< endl;
            //cout << "feasibility of second condition " << endl;
            //cout << feasi_x_H2.transpose() * H2 * feasi_x_H2 << endl;
            //cout << "feasibility of third condition " << endl;
            //cout << feasi_x_H3.transpose() * H3 * feasi_x_H3 << endl;
            // get tempvec for each variable
             temp_x_mu.segment(0,num_states_/2)  = xvarop.segment(i*dimA,num_states_/2) - muk.segment(i*dimA,num_states_/2);
             temp_x_mu.segment(num_states_/2,num_contacts_nor_) = xvarop.segment(N*dimA + i*dimC,num_contacts_nor_) - muk.segment(N*dimA + i*dimC,num_contacts_nor_);

             temp_x_mu1.segment(0,num_states_/2) = xvarop.segment(i*dimA+num_states_/2,num_states_/2) - muk1.segment(i*dimA+num_states_/2,num_states_/2);
             temp_x_mu1.segment(num_states_/2,num_contacts_nor_+num_contacts_tan_) = xvarop.segment(N*dimA+i*dimC+num_contacts_nor_,num_contacts_nor_+num_contacts_tan_) 
             - muk1.segment(N*dimA+i*dimC+num_contacts_nor_,num_contacts_nor_+num_contacts_tan_) ;

             temp_x_mu2 = xvarop.segment(N*dimA + i*dimC,dimC) - muk2.segment(N*dimA + i*dimC,dimC);

             // check if they already satisfy the conditions
             VectorXd result  = temp_x_mu.transpose()*H1*temp_x_mu+ temp_x_mu.transpose()*h1;
             VectorXd result1 = temp_x_mu1.transpose()*H2*temp_x_mu1;
             VectorXd result2 = temp_x_mu2.transpose()*H3*temp_x_mu2;
         
             //solve first qcqp
            if(abs(result(0)) < 1e-8){
            //if(result(0) < 1e-8){
                local_zk = temp_x_mu;
            }
            else{
                temp_x_mu = H1egvec.transpose()*temp_x_mu;
			    double eigmax = H1egval.maxCoeff();
			    double eigmin = H1egval.minCoeff();
		    	double lb, ub;
		    	lb = - 1 / eigmax;
			    ub = - 1 / eigmin;
		    	double nu = DoBisection(lb, ub, H1egval, h1hat,temp_x_mu);
                MatrixXd tmpmat2 = MatrixXd::Identity(num_states_/2+num_contacts_nor_,num_states_/2+num_contacts_nor_) + nu*H1egvaldiag;
		    	//	tmpmat = Eigen::MatrixXd::Identity(varnum, varnum) + nu*eigenval.asDiagonal();
		    	VectorXd temp = nu*h1hat-2*temp_x_mu;
		    	temp = tmpmat2.inverse() * temp;
		    	local_zk = -0.5 *H1egvec * temp;   
            }

            //solve second qcqp
            if(abs(result1(0)) < 1e-8){
            //if(result1(0) < 1e-8){
                local_zk1 = temp_x_mu1;
            }
            else{
                temp_x_mu1 = H2egvec.transpose()*temp_x_mu1;
			    double eigmax = H2egval.maxCoeff();
			    double eigmin = H2egval.minCoeff();
		    	double lb, ub;
		    	lb = - 1 / eigmax;
			    ub = - 1 / eigmin;         
		    	double nu = DoBisection(lb, ub, H2egval, h2hat, temp_x_mu1);
                MatrixXd tmpmat2 = MatrixXd::Identity(num_states_/2+num_contacts_nor_+num_contacts_tan_,num_states_/2+num_contacts_nor_+num_contacts_tan_) + nu*H2egvaldiag;
		    	//	tmpmat = Eigen::MatrixXd::Identity(varnum, varnum) + nu*eigenval.asDiagonal();
		    	VectorXd temp = nu*h2hat-2*temp_x_mu1;
		    	temp = tmpmat2.inverse() * temp;
		    	local_zk1 = -0.5 *H2egvec * temp;    
            }
            //solve third qcqp
            if(abs(result2(0)) < 1e-8){
            //if(result2(0) < 1e-8){
                 local_zk2 = temp_x_mu2;
            }
            else{
                temp_x_mu2 = H3egvec.transpose()*temp_x_mu2;
			    double eigmax = H3egval.maxCoeff();
			    double eigmin = H3egval.minCoeff();
		    	double lb, ub;
		    	lb = - 1 / eigmax;
			    ub = - 1 / eigmin;   
		    	double nu = DoBisection(lb, ub, H3egval, h3hat, temp_x_mu2);
                MatrixXd tmpmat2 = MatrixXd::Identity(dimC,dimC) + nu*H3egvaldiag;
		    	//	tmpmat = Eigen::MatrixXd::Identity(varnum, varnum) + nu*eigenval.asDiagonal();
		    	VectorXd temp = nu*h3hat-2*temp_x_mu2;
		    	temp = tmpmat2.inverse() * temp;
		    	local_zk2 = -0.5 *H3egvec * temp;   
            }
            // update zk,zk1,zk2
            zk.segment(i*dimA,num_states_/2) = local_zk.segment(0,num_states_/2);
            zk.segment(N*dimA + i*dimC,num_contacts_nor_) = local_zk.segment(num_states_/2,num_contacts_nor_);

            zk1.segment(i*dimA+num_states_/2,num_states_/2) = local_zk1.segment(0,num_states_/2);
            zk1.segment(N*dimA+i*dimC+num_contacts_nor_,num_contacts_nor_+num_contacts_tan_) = local_zk1.segment(num_states_/2,num_contacts_nor_+num_contacts_tan_);

            zk2.segment(N*dimA+i*dimC,dimC) = local_zk2;
        }
    
		// step 3: update muk,muk1,muk2
		muk = muk + zk - xvarop;
        muk1 = muk1 + zk1 - xvarop;
        muk2 = muk2 + zk2 - xvarop;
        		
		//cout << "iter = " << iter << ", objval = " << objval << endl;

	}
    delete[] xvar;
    cout << "admm solved" << endl;
	return xvarop.segment(N*(dimA+dimC),dimB);
    
}



void MpcBalanceController::get_optimization_matrix(MatrixXd dynamic_matrix[14],int N,VectorXd x0) const{

    const int dimA = num_states_;
    const int dimB = num_inputs_;
    const int dimC = num_contacts_nor_*2 + num_contacts_tan_;
    const int varnum = num_states_ + num_inputs_ + 2*num_contacts_nor_ + num_contacts_tan_ + 2;
    
    MatrixXd mat = MatrixXd::Identity(num_states_,num_states_);
    mat.block(0,num_states_/2,num_states_/2,num_states_/2) = -dt_ * MatrixXd::Identity(num_states_/2,num_states_/2);
    mat = mat.inverse();

    MatrixXd Amat = MatrixXd::Identity(num_states_,num_states_);
    Amat.block(num_states_/2,0,num_states_/2,num_states_) = Amat.block(num_states_/2,0,num_states_/2,num_states_) +  A_;
    Amat = mat*Amat;

    MatrixXd Bmat = MatrixXd::Zero(num_states_,num_inputs_);
    Bmat.block(num_states_/2,0,num_states_/2,num_inputs_) = Bmat.block(num_states_/2,0,num_states_/2,num_inputs_) + B_;
    Bmat = mat*Bmat;

    MatrixXd Cmat = MatrixXd::Zero(num_states_,num_contacts_nor_*2 + num_contacts_tan_);
    Cmat.block(num_states_/2,0,num_states_/2,num_contacts_nor_) = invMn_;
    Cmat.block(num_states_/2,num_contacts_nor_,num_states_/2,num_contacts_tan_) = invMD_;
    Cmat = mat*Cmat;

    MatrixXd J_tree_mat = MatrixXd::Zero(num_states_ , 2);
    J_tree_mat.block(num_states_/2,0,num_states_/2,2) = invMJtree_;
    J_tree_mat = mat*J_tree_mat;

    MatrixXd con = MatrixXd::Zero(num_states_,1);
    con.block(num_states_/2,0,num_states_/2,1) = const_;
    con = mat*con;

    MatrixXd Q = 10*MatrixXd::Identity(num_states_,num_states_);
    MatrixXd R = 0.1*MatrixXd::Identity(num_inputs_,num_inputs_);


    MatrixXd dlim = MatrixXd::Zero(num_contacts_nor_*2+num_contacts_tan_,1);
    dlim.block(0,0,num_contacts_nor_,1) = dlim_;
    
    MatrixXd temp = MatrixXd::Zero(num_contacts_tan_,num_contacts_nor_);
    for(int i = 0;i<num_contacts_nor_;i++){
        temp.block(i*num_contacts_nor_,i,num_contacts_nor_,1) = MatrixXd::Ones(num_contacts_nor_,1);
    }
 
    MatrixXd D = MatrixXd::Zero(num_contacts_nor_*2+num_contacts_tan_,num_states_+num_contacts_nor_*2+num_contacts_tan_);
    
    D.block(0,0,num_contacts_nor_,num_states_/2) = n_.transpose();
    D.block(num_contacts_nor_,num_states_/2,num_contacts_tan_,num_states_/2) = D_.transpose();
    D.block(num_contacts_nor_,num_states_+num_contacts_nor_+num_contacts_tan_,num_contacts_tan_,num_contacts_nor_) = temp;
    D.block(num_contacts_nor_+num_contacts_tan_,num_states_,num_contacts_nor_,num_contacts_nor_) = 0.7*MatrixXd::Identity(num_contacts_nor_,num_contacts_nor_);
    D.block(num_contacts_nor_+num_contacts_tan_,num_states_+num_contacts_nor_,num_contacts_nor_,num_contacts_tan_) = -temp.transpose();

    
    MatrixXd H1 = MatrixXd::Zero(num_states_/2 + num_contacts_nor_ , num_states_/2 + num_contacts_nor_);
    MatrixXd h1 = MatrixXd::Zero(num_states_/2 + num_contacts_nor_ , 1);
    H1.block(0 , num_states_/2 , num_states_/2 , num_contacts_nor_) = 0.5*n_;
    H1.block(num_states_/2 , 0 , num_contacts_nor_ , num_states_/2) = 0.5*n_.transpose();
    h1.block(num_states_/2 , 0 , num_contacts_nor_ , 1) = dlim_;

    MatrixXd H2 = MatrixXd::Zero(num_states_/2+num_contacts_nor_+num_contacts_tan_,num_states_/2+num_contacts_nor_+num_contacts_tan_);
    H2.block(0,num_states_/2,num_states_/2,num_contacts_tan_) = 0.5*D_;
    H2.block(num_states_/2,0,num_contacts_tan_,num_states_/2) = 0.5*D_.transpose();
    H2.block(num_states_/2,num_states_/2+num_contacts_tan_,num_contacts_tan_,num_contacts_nor_) = 0.5*temp;
    H2.block(num_states_/2+num_contacts_tan_,num_states_/2,num_contacts_nor_,num_contacts_tan_) = 0.5*temp.transpose();

    MatrixXd H3 = MatrixXd::Zero(2*num_contacts_nor_ + num_contacts_tan_ , 2*num_contacts_nor_ + num_contacts_tan_);
    H3.block(0 , num_contacts_nor_ + num_contacts_tan_ , num_contacts_nor_ , num_contacts_nor_) = 0.35*MatrixXd::Identity(num_contacts_nor_,num_contacts_nor_);
    H3.block(num_contacts_nor_ + num_contacts_tan_ , 0 , num_contacts_nor_ , num_contacts_nor_) = 0.35*MatrixXd::Identity(num_contacts_nor_,num_contacts_nor_);
    H3.block(num_contacts_nor_ , num_contacts_nor_ + num_contacts_tan_ , num_contacts_tan_ , num_contacts_nor_) = -temp/2;
    H3.block(num_contacts_nor_ + num_contacts_tan_,num_contacts_nor_,num_contacts_nor_,num_contacts_tan_) = -temp.transpose()/2;

    MatrixXd Ae = MatrixXd::Zero(N*num_states_,N*varnum);
    MatrixXd be = MatrixXd::Zero(N*num_states_,1);
    MatrixXd Ain = MatrixXd::Zero(N*(num_contacts_nor_*2+num_contacts_tan_),N*varnum);
    MatrixXd bin = MatrixXd::Zero(N*(num_contacts_nor_*2+num_contacts_tan_),1);
    MatrixXd tempA = -MatrixXd::Identity(N*dimA,N*dimA); Ae.block(0,0,N*dimA,N*dimA) = tempA;
    for(int i = 0 ; i<N; i++){
        if(i==0){   
            be.block(0,0,dimA,1) = -Amat*x0-con;
        }
        else{
            be.block(i*dimA,0,dimA,1) = -con;
            Ae.block(i*dimA,(i-1)*dimA,dimA,dimA) = Amat;    
        } 
        Ae.block(i*dimA,N*dimA+i*dimC,dimA,dimC) = Cmat;
        Ae.block(i*dimA,N*dimA+N*dimC+i*dimB,dimA,dimB) = Bmat;
        Ae.block(i*dimA,N*(dimA+dimB+dimC)+i*2,dimA,2) = J_tree_mat;
        Ain.block(i*dimC,i*dimA,dimC,dimA) = D.block(0,0,dimC,dimA);
        Ain.block(i*dimC,N*dimA+i*dimC,dimC,dimC) = D.block(0,dimA,dimC,dimC);
        bin.block(i*dimC,0,dimC,1) = dlim;
    }

    
    dynamic_matrix[0] = Amat;
    dynamic_matrix[1] = Bmat;
    dynamic_matrix[2] = Cmat;
    dynamic_matrix[3] = con;
    dynamic_matrix[4] = dlim;
    dynamic_matrix[5] = D;

    //qcqp constraint
    dynamic_matrix[6] = H1;
    dynamic_matrix[7] = h1;
    dynamic_matrix[8] = H2;
    dynamic_matrix[9] = H3;

    // linear constraint matrix
    dynamic_matrix[10] = Ae;
    dynamic_matrix[11] = be;
    dynamic_matrix[12] = Ain;
    dynamic_matrix[13] = -bin;

}


}
}