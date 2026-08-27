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


if __name__ == "__main__":
    # Quick manual test
    elbow, hand = forward_kinematics(theta1=45, theta2=30)
    print(f"Elbow position: {elbow}")
    print(f"Hand position: {hand}")

    plot_arm(theta1=45, theta2=30)
