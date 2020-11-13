import numpy as np
import matplotlib.pyplot as plt
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable


def main():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile("/home/yangwill/Documents/research/dairlib/examples/five_link_biped/five_link_biped.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    world = plant.world_frame()

    l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
    r_contact_frame = plant.GetBodyByName("right_foot").body_frame()

    print("nq: ")
    print(plant.num_positions())
    print("nv: ")
    print(plant.num_velocities())

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nu = plant.num_actuators()

    TXZ = np.array([[1, 0, 0], [0, 0, 1]]);

    v_and_error_1 = np.genfromtxt('V_5e-5.csv', delimiter=',')
    v_and_error_2 = np.genfromtxt('V_5e-5_no_inputs.csv', delimiter=',')
    # v_and_error_1 = np.genfromtxt('V_1e-4_no_inputs.csv', delimiter=',')
    # v_and_error_2 = np.genfromtxt('V_1e-4.csv', delimiter=',')
    # v_and_error_2 = np.genfromtxt('V.csv', delimiter=',')
    inputs_1 = np.genfromtxt('inputs_5e-5.csv', delimiter=',')
    inputs_2 = np.genfromtxt('inputs_5e-5_no_inputs.csv', delimiter=',')
    # inputs_1 = np.genfromtxt('inputs_1e-4_no_inputs.csv', delimiter=',')
    # inputs_2 = np.genfromtxt('inputs_1e-4.csv', delimiter=',')
    # inputs_2 = np.genfromtxt('inputs.csv', delimiter=',')


    #load in values
    t_1 = v_and_error_1[:,0]
    t_input_1 = inputs_1[:,0]
    V_1 = v_and_error_1[:,1:4]
    x_actual_1 = v_and_error_1[:,4:4+14]
    x_des0_1 = v_and_error_1[:,18:18+14]
    x_des1_1 = v_and_error_1[:,32:32+14]
    x_des2_1 = v_and_error_1[:,46:46+14]
    x_des_all_modes_1 = v_and_error_1[:,60:60+14]
    lambd_1 = -v_and_error_1[:,[74, 76, 77, 79]]
    t_2 = v_and_error_2[:,0]
    t_input_2 = inputs_2[:,0]
    V_2 = v_and_error_2[:,1:4]
    x_actual_2 = v_and_error_2[:,4:4+14]
    x_des0_2 = v_and_error_2[:,18:18+14]
    x_des1_2 = v_and_error_2[:,32:32+14]
    x_des2_2 = v_and_error_2[:,46:46+14]
    x_des_all_modes_2 = v_and_error_2[:,60:60+14]
    lambd_2 = -v_and_error_2[:,[74, 76, 77, 79]]

    x_idx = 0
    z_idx = 1
    roty_idx = 2
    lhip_idx = 3
    rhip_idx = 4
    lknee_idx = 5
    rknee_idx = 6
    # The indices are fixed to the datafiles
    # t_pre_idx_1 = np.where(t_1 == .213)[0][0]
    # t_pre_idx_1 = np.where(t_1 == .213)[0][0]
    # t_post_idx_1 = np.where(t_1 == .21725)[0][0]
    t_pre_idx_1 = np.where(t_1 == .21275)[0][0]
    t_post_idx_1 = np.where(t_1 == .22)[0][0]
    t_pre_idx_2 = np.where(t_2 == .21275)[0][0]
    t_post_idx_2 = np.where(t_2 == .22)[0][0]
    # Saved indices
    # t_pre_idx_1 = 984
    # t_post_idx_1 = 1000
    # t_pre_idx_2 = 869
    # t_post_idx_2 = 895

    # print(t_2[t_pre_idx_2])
    # print(t_2[t_post_idx_2])
    t_pre_idx_input_1 = np.where(t_input_1 == t_1[t_pre_idx_1])[0][0]
    t_post_idx_input_1 = np.where(t_input_1 == t_1[t_post_idx_1])[0][0]
    t_pre_idx_input_2 = np.where(t_input_2 == t_2[t_pre_idx_2])[0][0]
    t_post_idx_input_2 = np.where(t_input_2 == t_2[t_post_idx_2])[0][0]
    num_dt_1 = t_post_idx_1 - t_pre_idx_1
    num_dt_2 = t_post_idx_2 - t_pre_idx_2
    num_input_dt_1 = t_post_idx_input_1 - t_pre_idx_input_1
    num_input_dt_2 = t_post_idx_input_2 - t_pre_idx_input_2
    v_pre_1 = x_actual_1[t_pre_idx_1, 7:15]
    v_post_1 = x_actual_1[t_post_idx_1, 7:15]
    v_pre_2 = x_actual_2[t_pre_idx_2, 7:15]
    v_post_2 = x_actual_2[t_post_idx_2, 7:15]

    # plt.plot(t_1, x_actual_1[:,7:15])
    # plt.show()
    # print((x_actual_2[:,rknee_idx][1:] - x_actual_2[:,rknee_idx][:-1]).shape)
    # print(t_pre_1)
    # print(np.where(t_2 == 0.22))
    # print(t_1[t_pre_1])
    # t_pre_idx = t_1.index(:,rkee)



    context = plant.CreateDefaultContext();
    plant.SetPositionsAndVelocities(context, x_actual_1[t_pre_idx_1,:])
    M_pre = plant.CalcMassMatrixViaInverseDynamics(context)
    # pt_on_body = plant.CalcPointsPositions(context, r_contact_frame, np.zeros(3), world);
    pt_on_body = np.zeros(3)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_contact_frame, pt_on_body, world, world)
    M_pre_inv = np.linalg.inv(M_pre)
    C_pre = plant.CalcBiasTerm(context)
    v_rfoot_pre = J_r * x_actual_1[t_pre_idx_1, 7:15]
    v_rfoot_post = J_r * x_actual_1[t_post_idx_1, 7:15]
    plant.SetPositionsAndVelocities(context, x_actual_1[t_post_idx_1,:])
    M_post = plant.CalcMassMatrixViaInverseDynamics(context)
    C_post = plant.CalcBiasTerm(context)

    C_1 = np.zeros((num_dt_1, nq))
    C_2 = np.zeros((num_dt_2, nq))
    g_1 = np.zeros((num_dt_1, nq))
    g_2 = np.zeros((num_dt_2, nq))
    B_U_1 = np.zeros((num_input_dt_1, nq))
    B_U_2 = np.zeros((num_input_dt_2, nq))
    # lambda_1_est = np.zeros((num_dt_1, 4))
    net_bias_impulse_1 = np.zeros(nq)
    net_bias_impulse_2 = np.zeros(nq)
    net_grav_impulse_1 = np.zeros(nq)
    net_grav_impulse_2 = np.zeros(nq)
    net_input_impulse_1 = np.zeros(nq)
    net_input_impulse_2 = np.zeros(nq)
    net_Lambda_1 = np.zeros(4)
    net_Lambda_2 = np.zeros(4)

    for i in range(num_dt_1):
        plant.SetPositionsAndVelocities(context, x_actual_1[i + t_pre_idx_1,:])
        C_1[i,:] = plant.CalcBiasTerm(context)
        g_1[i,:] = plant.CalcGravityGeneralizedForces(context)
    for i in range(num_dt_2):
        plant.SetPositionsAndVelocities(context, x_actual_2[i + t_pre_idx_2,:])
        C_2[i,:] = plant.CalcBiasTerm(context)
        g_2[i,:] = plant.CalcGravityGeneralizedForces(context)
    for i in range(num_input_dt_1):
        plant.SetPositionsAndVelocities(context, x_actual_1[t_pre_idx_1,:])
        B = plant.MakeActuationMatrix()
        B_U_1[i,:] = B@inputs_1[i+t_pre_idx_input_1, 1:5]
    for i in range(num_input_dt_2):
        plant.SetPositionsAndVelocities(context, x_actual_2[t_pre_idx_2,:])
        B = plant.MakeActuationMatrix()
        B_U_2[i,:] = B@inputs_2[i+t_pre_idx_input_2, 1:5]
    for state in range(nq):
        net_bias_impulse_1[state] = np.trapz(C_1[:,state], t_1[t_pre_idx_1:t_post_idx_1])
        net_bias_impulse_2[state] = np.trapz(C_2[:,state], t_2[t_pre_idx_2:t_post_idx_2])
        net_grav_impulse_1[state] = np.trapz(g_1[:,state], t_1[t_pre_idx_1:t_post_idx_1])
        net_grav_impulse_2[state] = np.trapz(g_2[:,state], t_2[t_pre_idx_2:t_post_idx_2])
        net_input_impulse_1[state] = np.trapz(B_U_1[:,state], t_input_1[t_pre_idx_input_1:t_post_idx_input_1])
        net_input_impulse_2[state] = np.trapz(B_U_2[:,state], t_input_2[t_pre_idx_input_2:t_post_idx_input_2])
    for force in range(4):
        net_Lambda_1[force] = np.trapz(lambd_1[t_pre_idx_1:t_post_idx_1,force], t_1[t_pre_idx_1:t_post_idx_1])
        net_Lambda_2[force] = np.trapz(lambd_2[t_pre_idx_2:t_post_idx_2,force], t_2[t_pre_idx_2:t_post_idx_2])
    lambda_1_pred = np.linalg.pinv(M_pre_inv@J_r.T)@(v_post_1 - v_pre_1)
    lambda_2_pred = np.linalg.pinv(M_pre_inv@J_r.T)@(v_post_2 - v_pre_2)
    vel_delta_1 = M_pre_inv @ net_bias_impulse_1 + M_pre_inv @ net_grav_impulse_1 + M_pre_inv @ net_input_impulse_1
    vel_delta_2 = M_pre_inv @ net_bias_impulse_2 + M_pre_inv @ net_grav_impulse_2 + M_pre_inv @ net_input_impulse_2
    vel_delta_minus_lambda_1 = (v_post_1 - v_pre_1) - vel_delta_1
    vel_delta_minus_lambda_2 = (v_post_2 - v_pre_2) - vel_delta_2
    lambda_1_pred_other = np.linalg.pinv(M_pre_inv @ J_r.T) @ vel_delta_minus_lambda_1
    lambda_2_pred_other = np.linalg.pinv(M_pre_inv @ J_r.T) @ vel_delta_minus_lambda_2
    # print(J_r @ M_pre_inv @ net_bias_impulse_1)
    # print(J_r @ M_pre_inv @ net_grav_impulse_1)
    # print(J_r @ M_pre_inv @ net_input_impulse_1)
    # foot_vel_delta = J_r @ M_pre_inv @ net_bias_impulse_1 + J_r @ M_pre_inv @ net_grav_impulse_1 + J_r @ M_pre_inv @ net_input_impulse_1
    # print(vel_delta_minus_lambda)
    # print(lambda_1_pred)
    # print(lambda_1_pred_other)
    # print(net_Lambda)
    # print(M_pre_inv @ J_r.T @ lambda_1_pred)
    # print(M_pre_inv @ J_r.T @ lambda_1_pred_other)
    # print(v_post_1 - v_post_2)

    # print("Predicted Lambdas: ")
    # print("Lambda from samping: ")
    # print(net_Lambda_1)
    # print("Lambda from pinv assuming only contact forces")
    # print(lambda_1_pred)
    # print("Lambda from pinv including all forces")
    # print(lambda_1_pred_other)
    # print(lambda_2_pred)
    # print(lambda_2_pred_other)

    # print(v_post_1 - v_pre_1)
    # print(M_pre_inv @ J_r.T @ lambda_1_pred_other)
    # print(v_post_2 - v_pre_2)
    # print(M_pre_inv @ J_r.T @ lambda_2_pred_other)
    # print("Only Lambda from pinv")
    print("Predicted post_impact feet vel: ")
    # print(J_r @ (v_pre_1 + M_pre_inv @ J_r.T @ lambda_1_pred))
    print("Actual: ")
    print(J_r @ (v_post_1 - v_pre_1))
    print(J_r @ (v_post_2 - v_pre_1))
    print("All terms using pinv for lambda")
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_1_pred_other + net_bias_impulse_1 + net_grav_impulse_1 + net_input_impulse_1)))
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_2_pred_other + net_bias_impulse_2 + net_grav_impulse_2 + net_input_impulse_2)))
    print("All terms: ")
    print(J_r @ (M_pre_inv @ (J_l.T @ net_Lambda_1[0:2] + J_r.T @ net_Lambda_1[2:4] + net_bias_impulse_1 + net_grav_impulse_1 + net_input_impulse_1)))
    print(J_r @ (M_pre_inv @ (J_l.T @ net_Lambda_2[0:2] + J_r.T @ net_Lambda_2[2:4] + net_bias_impulse_2 + net_grav_impulse_2 + net_input_impulse_2)))
    print("Just pinv for lambda")
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_1_pred_other)))
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_2_pred_other)))
    print("Just pinv for lambda (bad)")
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_1_pred)))
    print(J_r @ (M_pre_inv @ (J_r.T @ lambda_2_pred)))
    print("Only lambda from sampled contact forces")
    print(J_r @ (M_pre_inv @ (J_l.T @ net_Lambda_1[0:2] + J_r.T @ net_Lambda_1[2:4])))
    print(J_r @ (M_pre_inv @ (J_l.T @ net_Lambda_2[0:2] + J_r.T @ net_Lambda_2[2:4])))
    print("Input contribution")
    print(J_r @ (M_pre_inv @ (net_input_impulse_1)))
    print(J_r @ (M_pre_inv @ (net_input_impulse_2)))
    print("Coriolis contribution")
    print(J_r @ (M_pre_inv @ (net_bias_impulse_1)))
    print(J_r @ (M_pre_inv @ (net_bias_impulse_2)))
    print("Grav contribution")
    print(J_r @ (M_pre_inv @ (net_grav_impulse_1)))
    print(J_r @ (M_pre_inv @ (net_grav_impulse_2)))
    print()
    print("Lambda from g")
    print(M_pre_inv @ J_r.T @ np.linalg.inv(J_r @ M_pre_inv @ J_r.T) @  J_r @ (M_pre_inv @ net_grav_impulse_1))
    print(M_pre_inv @ J_r.T @ np.linalg.inv(J_r @ M_pre_inv @ J_r.T) @  J_r @ (M_pre_inv @ net_grav_impulse_2))
    print(v_post_1 - v_post_2)
    # print(M_pre_inv @ net_grav_impulse_1)

    print("Predicted post-impact generalized vel 5e-5")
    print("All terms: ")
    print(v_pre_1 + M_pre_inv @ (J_r.T @ net_Lambda_1[2:4] + net_bias_impulse_1 + net_grav_impulse_1 + net_input_impulse_1))
    print(v_pre_2 + M_pre_inv @ (J_r.T @ net_Lambda_2[2:4] + net_bias_impulse_2 + net_grav_impulse_2 + net_input_impulse_2))
    # print(v_pre_1 + M_pre_inv @ (J_r.T @ lambda_1_pred_other + net_bias_impulse_1 + net_grav_impulse_1 + net_input_impulse_1))

    print("Actual: ")
    print(v_post_2)
    print("Input contribution")
    print((M_pre_inv @ (net_input_impulse_1)))
    print((M_pre_inv @ (net_input_impulse_2)))
    print("Coriolis contribution")
    print((M_pre_inv @ (net_bias_impulse_1)))
    print((M_pre_inv @ (net_bias_impulse_2)))
    print("Grav contribution")
    print((M_pre_inv @ (net_grav_impulse_1)))
    print((M_pre_inv @ (net_grav_impulse_2)))
    # print(v_pre_1 + M_pre_inv @ (J_r.T @ lambda_1_pred))
    # print(v_pre_1 + M_pre_inv @ (J_r.T @ net_Lambda_1[2:4] + net_grav_impulse_1  + net_bias_impulse_1))
    # print(v_pre_1 + M_pre_inv @ (J_r.T @ net_Lambda_1[2:4]))

    print("diff")

    print(net_Lambda_1[2:4])
    print(net_Lambda_1[2:4] - lambda_1_pred_other)
    print(net_Lambda_2[2:4] - lambda_2_pred_other)


    ## Double checking rank of matrices
    print("Rank Checking")
    print("J: ")
    print(np.linalg.matrix_rank(J_r))
    print("J M_inv J.T: ")
    print(np.linalg.matrix_rank(J_r @ M_pre_inv @ J_r.T))
    # print(J_r @ M_pre @ J_r.T)
    # print(J_r)
    # print(np.linalg.matrix_rank(M_pre_inv @ J_r.T))
    # print(np.linalg.matrix_rank(J_r))
    # [u, s, vh] = np.linalg.svd(J_r)

    ## Mismatch between equations - clearly lambda can't explain the vel change
    print("Mismatch between predicted and actual post impact feet velocities")
    # print(J_r @ M_pre_inv @ J_r.T @ net_Lambda)
    # print(J_r @ (v_post_1 - v_pre_1))
    print(J_r @ (v_post_1 - v_post_2))
    print(J_r @ M_pre_inv @ J_r.T @ (lambda_1_pred - lambda_2_pred))
    print(v_post_1 - v_post_2)
    # print(J_r @ M_pre_inv @ J_r.T @ (lambda_1_pred_other - lambda_2_pred_other))

    ## Any plotting
    # plt.plot(t_1, x_actual_1[:,10:15])
    # plt.plot(t_2, x_actual_2[:,10:15])
    # plt.plot(t_1, x_des_all_modes_1[:,10:15])
    # plt.plot(t_1, x_actual_1[:,7:15] @ J_r.T)
    # plt.plot(t_2, x_actual_2[:,7:15] @ J_r.T)
    # print(lambd_1[t_pre_idx_1 + 2])
    # plt.plot(t_1[t_pre_idx_1:t_post_idx_1], lambd_1[t_pre_idx_1: t_post_idx_1], '.')
    # plt.plot(t_2[t_pre_idx_2:t_post_idx_2], lambd_2[t_pre_idx_2: t_post_idx_2], '.')
    # plt.plot(t_1, lambd_1, '.')
    # plt.plot(t_2, lambd_2, '.')
    # plt.legend(['r_t', 'r_n', 'l_t', 'l_n'])
    # plt.plot(t_2, x_actual_2[:,11:15])
    # plt.plot(inputs_1[:,0], inputs_1[:,1:5], 'b')
    plt.plot(inputs_1[:,0], inputs_1[:,[2,4]], 'b')
    # plt.plot(inputs_2[:,0], inputs_2[:,1:5], 'r')
    plt.plot(inputs_2[:,0], inputs_2[:,[2,4]], 'r')
    # plt.plot(t_1, x_actual_1[:,0:8])
    # plt.plot(t_2, x_actual_2[:,0:8])

    # print(J_r @ (v_post_1 - v_pre_1) - J_r @ M_pre_inv @ J_r.T @ lambda_1_pred)
    # print(foot_vel_delta)
    # print(np.linalg.pinv(M_pre_inv@J_r.T)@(v_post_1 - v_pre_1))

    plt.show()
    diagram = builder.Build()
    simulator = Simulator(diagram)


if __name__ == "__main__":
    main()
