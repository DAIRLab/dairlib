import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
def main():
    K1 = 100; C1=0.1; K2=100; C2=0.1 
    m1=1; m2=10; m3=1; dt = 1e-2
    u_dim = 2; n_dim = 6
    A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[-K1/m1,K1/m1,0,-C1/m1,C1/m1,0],[K1/m2,-(K1+K2)/m2,K2/m2,C1/m1,-(C1+C2)/m2,C2/m2],[0,K2/m3,-K2/m3,0,C2/m3,-C2/m3]])
    B = np.array([[0,0],[0,0],[0,0],[1/m1,0],[0,0],[0,1/m3]])
    t = np.arange(200) * dt
    u = [np.array([0,0])]

    controllability_matrix = np.zeros((6,6))
    for i in range(n_dim):
        controllability_matrix[:,i] = np.linalg.matrix_power(A,i) @ B[:,1]
    state = np.zeros(6)
    state[0] = 0.01
    x1 = [state[0]]
    x2 = [state[1]]
    x3 = [state[2]]
    x1dot = [state[3]]
    x2dot = [state[4]]
    x3dot = [state[5]]
    v = A @ state + B @ u[-1]
    x1ddot = [v[3]]
    x2ddot = [v[4]]
    x3ddot = [v[5]]
    
    K = -scipy.signal.place_poles(A, B, np.array([-5,-6,-7,-8,-9,-10])).gain_matrix

    state_desired = np.zeros(6)

    for i in range(1, t.shape[0]):
        state = state + dt * v
        u.append(K @ (state-state_desired))
        v = A @ state + B @ u[-1]
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
    u = np.array(u)

    plt.figure()
    plt.plot(t, x1, label="x1")
    plt.plot(t, x2, label="x2")
    plt.plot(t, x3, label="x3")
    plt.legend()
    plt.figure()
    plt.plot(t, x1dot, label="x1dot")
    plt.plot(t, x2dot, label="x2dot")
    plt.plot(t, x3dot, label="x3dot")
    plt.legend()
    plt.figure()
    plt.plot(t, x1ddot, label="x1ddot")
    plt.plot(t, x2ddot, label="x2ddot")
    plt.plot(t, x3ddot, label="x3ddot")
    plt.legend()
    plt.figure()
    plt.plot(t, u[:,0], label="u1")
    plt.plot(t, u[:,1], label="u2")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()