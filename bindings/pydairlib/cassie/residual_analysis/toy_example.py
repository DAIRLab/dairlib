import matplotlib.pyplot as plt
import numpy as np
def main():
    K = 1e2; C=5; m1=1; m3=1; m2=100; dt = 1e-2
    u_dim = 2; n_dim = 6
    A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[-K/m1,K/m1,0,-C/m1,0,0],[K/m2,-2*K/m2,K/m2,0,-C/m2,0],[0,K/m3,-K/m3,0,0,-C/m3]])
    B = np.array([[0,0],[0,0],[0,0],[1/m1,0],[0,0],[0,1/m3]])
    t = np.arange(100) * dt
    u = np.array([10,0])

    state = np.zeros(6)
    state[0] = 0.1
    x1 = [state[0]]
    x2 = [state[1]]
    x3 = [state[2]]
    v = A @ state + B @ u
    x1ddot = [v[3]]
    x2ddot = [v[4]]
    x3ddot = [v[5]]

    for i in range(1, t.shape[0]):
        state = state + dt * v
        v = A @ state + B @ u
        x1.append(state[0])
        x2.append(state[1])
        x3.append(state[2])
        x1ddot.append(v[3])
        x2ddot.append(v[4])
        x3ddot.append(v[5])

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
    plt.show()

if __name__ == "__main__":
    main()