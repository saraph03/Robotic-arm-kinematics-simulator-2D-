# 2-Link Robotic Arm Kinematics Simulator

A small project to build hands-on intuition for **forward kinematics** — the math
used to figure out where a robotic arm's end effector (the "hand") ends up,
given its joint angles. This is foundational to how real robotic arms are
controlled, including surgical/medical robots.

## Why this project

I'm interested in medical robotics but wanted to strengthen my coding skills,
specifically the gap between understanding concepts (geometry, trigonometry)
and translating them into working code. This project uses a 2D robotic arm
as a simple, visual way to practice that translation.

## What it does

Given two joint angles (`theta1`, `theta2`) and two link lengths (`l1`, `l2`),
the simulator:

1. Calculates the (x, y) position of the elbow and the hand using trigonometry
2. Plots the arm as connected line segments
3. Provides interactive sliders to move the arm in real time and build intuition

## How it works (the math)

The core idea: **any length pointing in a direction can be broken into x and y
components using cosine and sine.**

- Elbow position: the end of the first segment (length `l1`), given its angle
  `theta1` from the x-axis:
  ```
  elbow_x = l1 * cos(theta1)
  elbow_y = l1 * sin(theta1)
  ```

- Hand position: the end of the second segment (length `l2`), which starts
  from the elbow. Since `theta2` is defined *relative to the first segment*,
  the second segment's true angle in space is `theta1 + theta2`:
  ```
  hand_x = elbow_x + l2 * cos(theta1 + theta2)
  hand_y = elbow_y + l2 * sin(theta1 + theta2)
  ```

## Setup

```bash
pip install -r requirements.txt
```

Run the script directly:
```bash
python kinematics.py
```

Or open it in a Jupyter/Colab notebook to use the interactive sliders:
```python
from kinematics import plot_arm
from ipywidgets import interact

interact(plot_arm,
         theta1=(0, 180, 5),
         theta2=(-150, 150, 5),
         l1=(0.5, 2.0, 0.1),
         l2=(0.5, 2.0, 0.1))
```

## Progress log

- [x] **Forward kinematics** — calculate elbow/hand position from joint angles
- [x] **Static visualization** — draw the arm with matplotlib
- [x] **Interactive sliders** — move the arm live with ipywidgets
- [ ] **Inverse kinematics** — given a target (x, y), solve for the joint angles
      needed to reach it (the harder, more practically useful direction —
      this is closer to how a surgeon or operator would specify "move the
      instrument tip here")
- [ ] Add joint angle constraints (realistic robots can't bend infinitely)
- [ ] Extend to a 3-link arm

## Notes / things I learned

- Angles here are measured counter-clockwise from the positive x-axis (standard
  math convention) — cos gives the x-component, sin gives the y-component.
- `theta2` is relative to the first link, not the ground, which is why the
  hand's true angle in space is `theta1 + theta2`, not just `theta2`.
- Hit a real environment/dependency debugging detour: a matplotlib/IPython
  version mismatch (`RcParams` error) and a missing `ipywidgets` renderer in
  VS Code. Fixed by upgrading matplotlib and installing the Jupyter widget
  renderer extension. Good reminder that environment setup is its own skill,
  separate from the actual coding logic.

