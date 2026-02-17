import numpy as np
import matplotlib.pyplot as plt

L1 = 75
L2 = 80

def solve_ik(x, y):
    r = np.hypot(x, y)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None

    phi = np.arctan2(y, x)
    print(f"Phi: {np.degrees(phi)}")
    beta = np.arccos((L1**2 + r**2 - L2**2) / (2*L1*r))
    print(f"Beta: {np.degrees(beta)}")
    alpha = np.arccos((L1**2 + L2**2 - r**2) / (2*L1*L2))
    print(f"Alpha: {np.degrees(alpha)}")


    theta1 = phi - beta
    theta2 = np.pi - alpha
    return theta1, theta2



def draw_leg(x, y):
    sol = solve_ik(x, y)
    if sol is None:
        print("Unreachable")
        return

    t1, t2 = sol
    print(np.degrees(t1), np.degrees(t2))
    print(np.degrees(t1) + 90, np.degrees(t2))

    hip = np.array([0, 0])
    knee = hip + L1 * np.array([np.cos(t1), np.sin(t1)])
    foot = knee + L2 * np.array([np.cos(t1 + t2), np.sin(t1 + t2)])

    plt.figure(figsize=(5,5))
    plt.plot([hip[0], knee[0]], [hip[1], knee[1]], 'o-', linewidth=4)
    plt.plot([knee[0], foot[0]], [knee[1], foot[1]], 'o-', linewidth=4)
    plt.plot(x, y, 'rx', markersize=12)
    plt.axhline(0); plt.axvline(0)
    plt.gca().set_aspect('equal')
    plt.grid()
    plt.title(f"x={x}, y={y}")
    plt.show()

draw_leg(0, -120)

