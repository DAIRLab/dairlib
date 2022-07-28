import matplotlib.pyplot as plt
import numpy as np
def main():
    K = 1; C=1; m1=1; m3=1; m2=1; dt = 1e-2
    u_dim = 2; n_dim = 6
    A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[-K/m1,K/m1,0,-C/m1,0,0],[K/m2,-2*K/m2,K/m2,0,-C/m2,0],[0,K/m3,-K/m3,0,0,-C/m3]])
    B = np.array([[0,0],[0,0],[0,0],[1/m1,0],[0,0],[0,1/m3]])
    t = np.arange(1000) * dt
    u = np.array([0,0])

    controllability_matrix = np.zeros((6,6))
    for i in range(n_dim):
        controllability_matrix[:,i] = np.linalg.matrix_power(A,i) @ B[:,1]
    state = np.zeros(6)
    state[0] = 0.1
    x1 = [state[0]]
    x2 = [state[1]]
    x3 = [state[2]]
    x1dot = [state[3]]
    x2dot = [state[4]]
    x3dot = [state[5]]
    v = A @ state + B @ u
    x1ddot = [v[3]]
    x2ddot = [v[4]]
    x3ddot = [v[5]]
    
    K = np.zeros((u_dim,n_dim))
    K[1,:] = np.ones(n_dim) * 0

    state_desired = np.zeros(6)

    for i in range(1, t.shape[0]):
        state = state + dt * v
        v = A @ state + B @ (u + K @ (state_desired - state))
        x1.append(state[0])
        x2.append(state[1])
        x3.append(state[2])
        x1dot.append(state[3])
        x2dot.append(state[4])
        x3dot.append(state[5])
        x1ddot.append(v[3])
        x2ddot.append(v[4])
        x3ddot.append(v[5])

    x1 = np.array(x1); x2 = np.array(x2); x3 = np.array(x3); 
    x1dot = np.array(x1dot); x2dot = np.array(x2dot); x3dot = np.array(x3dot)
    x1ddot = np.array(x1ddot); x2ddot = np.array(x2ddot); x3ddot = np.array(x3ddot)

    plt.figure()
    plt.plot(t, x1, label="x1")
    plt.plot(t, x2, label="x2")
    plt.plot(t, x3, label="x3")
    plt.legend()
    plt.figure()
    plt.plot(t, x1ddot, label="x1ddot")
    plt.plot(t, x2ddot, label="x2ddot")
    plt.plot(t, x3ddot, label="x3ddot")
    plt.legend()
    plt.figure()
    plt.plot(t, x1dot+x2dot+x1ddot+x2ddot+x3ddot, label="sum up vdot")
    plt.plot(t, -x3dot, label="-x3dot")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()