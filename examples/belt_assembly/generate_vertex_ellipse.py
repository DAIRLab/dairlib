import numpy as np
import matplotlib.pyplot as plt

def ellipse_points(a, b, N):
    """
    Generate N points on an ellipse centered at the origin.
    a: semi-major axis length
    b: semi-minor axis length
    N: number of points
    Returns:
        x, y: numpy arrays of shape (N,)
    """
    theta = np.linspace(0, 2 * np.pi, N, endpoint=False)
    x = a * np.cos(theta)
    y = b * np.sin(theta)
    return x, y

# Example usage
if __name__ == "__main__":
    a, b = 0.12, 0.056   # semi-major and semi-minor axes
    N = 40       # number of points

    x, y = ellipse_points(a, b, N)
    for i in range(N):
        print(f"{x[i]} {y[i]} 0.")

    # Plot
    plt.figure(figsize=(6, 6))
    plt.plot(x, y, 'b-', label=f"Ellipse (a={a}, b={b})")
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.title('Ellipse Points')
    plt.show()
