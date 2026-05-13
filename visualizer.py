import numpy as np

def calculate_kinematics(q1, q2, q3, L1=40, L2=40):
    """
    L1 = Thigh length (cm), L2 = Shin length (cm)
    q1 = Hip Abduction (Side-to-side)
    q2 = Hip Flexion (Up-and-down)
    q3 = Knee Flexion
    """
    # Convert to Radians
    r1, r2, r3 = np.deg2rad([q1, q2, q3])

    # Knee Position
    kx = L1 * np.cos(r2) * np.cos(r1)
    ky = L1 * np.cos(r2) * np.sin(r1)
    kz = L1 * np.sin(r2)

    # Foot Position (End Effector)
    fx = kx + L2 * np.cos(r2 + r3) * np.cos(r1)
    fy = ky + L2 * np.cos(r2 + r3) * np.sin(r1)
    fz = kz + L2 * np.sin(r2 + r3)

    return (round(kx, 2), round(ky, 2), round(kz, 2)), (round(fx, 2), round(fy, 2), round(fz, 2))

def print_exercise(name, q1, q2, q3):
    knee, foot = calculate_kinematics(q1, q2, q3)
    print(f"\n--- EXERCISE: {name} ---")
    print(f"Angles: Hip_Side={q1}°, Hip_Up={q2}°, Knee={q3}°")
    print(f"KNEE Position: X={knee[0]}cm, Y={knee[1]}cm, Z={knee[2]}cm")
    print(f"FOOT Position: X={foot[0]}cm, Y={foot[1]}cm, Z={foot[2]}cm")

# Run some test cases
print_exercise("HOME (Straight Leg)", 0, 0, 0)
print_exercise("HIP FLEXION (Leg Up)", 0, 45, 0)
print_exercise("KNEE BEND (CPM)", 0, 30, -60)
print_exercise("SIDE MOVE (Abduction)", 20, 0, 0)