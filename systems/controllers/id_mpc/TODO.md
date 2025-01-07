# Features needed for complete hybrid MPC via SQP

## Defining the nonlinear optimization problem

- Dynamics Constraints (Done)
- Contact Constraints (Done)
- Define references (TODO)
- Define costs (TODO)


## Per Solve Actions
- Update initial state constraint
- Update trajectory timing, discretization, and contact states
- Group and linearize the nonlinear equality constraints
- Form a Gauss-Newton approximation of the cost hessian
- Solve the QP
- Line Search 

## Timeline Object 
The timeline is an object that holds the time and active contacts
for each knot point, as well as a vector of KnotPointState objects, 
which serve as caches for computing mutlibody quantities based on different 
values of the decision variables.

## IDMPC Object 
The IDMPC object is responsible for holding the state of the entire NLP 
representation of the MPC. In addition to holding the NLP formulation as a 
mathematical program, it should be able to construct a QP formulation for a 
given set of decision variables to make the SQP approximation about.

### Owns

- Nonlinear Equality Constraints
- Nonlinear Inequality Constraints
- Nonlinear Least-Squares Costs
- Timeline object holding the structural problem data
