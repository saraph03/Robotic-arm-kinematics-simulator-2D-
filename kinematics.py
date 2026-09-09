"""
2-Link Robotic Arm Kinematics Simulator
-----------------------------------------
A small project to build intuition for forward and inverse kinematics,
the math used to control robotic arms (including surgical/medical robots).

Author: Sara Phondge 
"""

import numpy as np
import matplotlib.pyplot as plt


def forward_kinematics(theta1, theta2, l1=1.0, l2=1.0):
    """
    Calculate the (x, y) position of the elbow and hand of a 2-link arm.

    Parameters
    ----------
    theta1 : float
        Angle of the first joint (shoulder), in degrees, measured
        counter-clockwise from the positive x-axis.
    theta2 : float
        Angle of the second joint (elbow), in degrees, measured relative
        to the first link (not relative to the ground).
    l1, l2 : float
        Lengths of the two arm segments.

    Returns
    -------
    elbow : (float, float)
        (x, y) position of the elbow joint.
    hand : (float, float)
        (x, y) position of the end of the arm (the "hand").
    """
    t1 = np.radians(theta1)
    t2 = np.radians(theta2)

    elbow_x = l1 * np.cos(t1)
    elbow_y = l1 * np.sin(t1)

    # The second segment's true angle in space is t1 + t2, since t2 is
    # defined relative to the first segment, not the x-axis.
    hand_x = elbow_x + l2 * np.cos(t1 + t2)
    hand_y = elbow_y + l2 * np.sin(t1 + t2)

    return (elbow_x, elbow_y), (hand_x, hand_y)


def plot_arm(theta1, theta2, l1=1.0, l2=1.0):
    """Draw the 2-link arm at the given joint angles."""
    elbow, hand = forward_kinematics(theta1, theta2, l1, l2)
    origin = (0, 0)

    xs = [origin[0], elbow[0], hand[0]]
    ys = [origin[1], elbow[1], hand[1]]

    plt.figure(figsize=(5, 5))
    plt.plot(xs, ys, '-o', linewidth=3, markersize=8, color='steelblue')
    plt.plot(hand[0], hand[1], 'ro', markersize=12, label='Hand')

    plt.xlim(-(l1 + l2) - 0.5, (l1 + l2) + 0.5)
    plt.ylim(-(l1 + l2) - 0.5, (l1 + l2) + 0.5)
    plt.gca().set_aspect('equal')
    plt.grid(True)
    plt.legend()
    plt.title(f"Arm: theta1={theta1}°, theta2={theta2}°")
    plt.show()
    
def check_reachable(x, y, l1=1.0, l2=1.0):
    """
    Calculate the distance from the origin to the target point, and check
    whether the arm can actually reach it.
    """
    d = np.sqrt(x**2 + y**2)

    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    if d > max_reach:
        print(f"Target is too far away (distance={d:.2f}, max reach={max_reach:.2f})")
        return d, False
    elif d < min_reach:
        print(f"Target is too close (distance={d:.2f}, min reach={min_reach:.2f})")
        return d, False
    else:
        return d, True
    
def inverse_kinematics_theta2(x, y, l1=1.0, l2=1.0):
    """
    Solve for theta2 (the elbow angle) given a target (x, y) position.
    """
    d, reachable = check_reachable(x, y, l1, l2)
    if not reachable:
        return None  # no valid solution exists

    # Law of cosines, rearranged to solve for cos(theta2)
    cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)

    # Safety clamp: floating point rounding can push this slightly outside
    # [-1, 1], which would break arccos. Clip it back into the valid range.
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2_rad = np.arccos(cos_theta2)
    theta2_deg = np.degrees(theta2_rad)

    return theta2_deg

def inverse_kinematics(x, y, l1=1.0, l2=1.0):
    """
    Full inverse kinematics: solve for both theta1 and theta2 given a
    target (x, y) position.
    """
    theta2_deg = inverse_kinematics_theta2(x, y, l1, l2)
    if theta2_deg is None:
        return None, None  # unreachable target

    theta2_rad = np.radians(theta2_deg)

    # Piece 1: straight-line angle to the target, ignoring the elbow bend
    angle_to_target = np.arctan2(y, x)

    # Piece 2: offset caused by the elbow bending
    offset = np.arctan2(l2 * np.sin(theta2_rad), l1 + l2 * np.cos(theta2_rad))

    # Combine: theta1 = angle_to_target - offset
    theta1_rad = angle_to_target - offset
    theta1_deg = np.degrees(theta1_rad)

    return theta1_deg, theta2_deg

def plot_arm_to_target(x, y, l1=1.0, l2=1.0):
    """
    Solve inverse kinematics for a target (x, y) and plot the resulting
    arm position, with the target marked separately so you can visually
    confirm the hand reaches it.
    """
    theta1, theta2 = inverse_kinematics(x, y, l1, l2)

    if theta1 is None:
        print(f"Target ({x}, {y}) is unreachable — not plotting.")
        return

    elbow, hand = forward_kinematics(theta1, theta2, l1, l2)
    origin = (0, 0)

    xs = [origin[0], elbow[0], hand[0]]
    ys = [origin[1], elbow[1], hand[1]]

    plt.figure(figsize=(5, 5))
    plt.plot(xs, ys, '-o', linewidth=3, markersize=8, color='steelblue', label='Arm')
    plt.plot(x, y, 'x', markersize=14, markeredgewidth=3, color='green', label='Target')
    plt.plot(hand[0], hand[1], 'ro', markersize=8, label='Hand (reached)')

    plt.xlim(-(l1 + l2) - 0.5, (l1 + l2) + 0.5)
    plt.ylim(-(l1 + l2) - 0.5, (l1 + l2) + 0.5)
    plt.gca().set_aspect('equal')
    plt.grid(True)
    plt.legend()
    plt.title(f"IK result: theta1={theta1:.1f}°, theta2={theta2:.1f}°")
    plt.show()



if __name__ == "__main__":
    # Quick manual test
    elbow, hand = forward_kinematics(theta1=45, theta2=30)
    print(f"Elbow position: {elbow}")
    print(f"Hand position: {hand}")

    plot_arm(theta1=45, theta2=30)
    
    d, reachable = check_reachable(x=0.5, y=1.5)
    print(f"Distance to target: {d:.3f}")
    print(f"Reachable: {reachable}")
    
    theta2 = inverse_kinematics_theta2(x=0.5, y=1.5)
    print(f"theta2: {theta2:.2f}°")
    
    theta1, theta2 = inverse_kinematics(x=0.5, y=1.5)
    print(f"theta1: {theta1:.2f}°")
    print(f"theta2: {theta2:.2f}°")
    
    elbow, hand = forward_kinematics(theta1, theta2)
    print(f"Hand position: {hand}")  # should be very close to (0.5, 1.5)
    
    plot_arm_to_target(x=0.5, y=1.5)
