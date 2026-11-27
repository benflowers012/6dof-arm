# Idea (parked): drag the arm on screen

Status: **parked, not started.** Saved 2026-11-26 to revisit later.

## What
A window on the laptop showing a drawing of the arm. Grab a part with the mouse and
drag it, and the real arm follows.

## Design sketched so far
- Lives inside `laptop/arm.py` (only one program can hold the Uno's serial port), started
  with `python arm.py --gui`. It uses the pygame window that's already a dependency.
  The controller, recording, replay and the flip keep working alongside it.
- **Side view** (shoulder, elbow, wrist):
  - Drag the **elbow** point: the shoulder swings to point at the mouse.
  - Drag the **wrist** point: shoulder and elbow solve 2-link inverse kinematics, and
    the claw keeps its current tilt.
  - Drag the **claw tip**: the same, with the claw tilt held.
- **Top view** dial: drag to turn the base.
- Claw twist and grip stay on the controller, with arrow keys as a backup when no
  controller is connected.
- Unreachable targets, or targets past a joint limit, stop at the closest reachable point.
- When a point can be reached with the elbow bent either way, the code picks the
  solution closest to the current angles, so the arm never suddenly flips over.
- This adds inverse kinematics, which the original spec listed as a non-goal. The
  inverse kinematics would apply to this window only.

## Needed before building
- Three measurements from the real arm, in cm: shoulder pivot → elbow pivot,
  elbow pivot → wrist pivot, wrist pivot → claw tip.
- Per joint: which servo angle points the segment straight up or straight out, and
  which way increasing the angle turns it. These become calibration constants.
