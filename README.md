# Quadruped IK

Small 2024 VEX quadruped project focused on inverse kinematics for a 3-DOF leg.

This repo is a mix of:
- VEXcode experiments for driving real motors on a single assembled leg
- Python-side IK / FK testing
- simple path smoothing experiments
- CAD, drawings, and build photos

## Start Here

- `IK testing.py`
  Main math sandbox. Contains:
  - inverse kinematics `IK(pos)`
  - forward kinematics `FK(angles)`
  - joint limiting with `Check_MotorAngles(...)`
  - simple path interpolation for test motion

- `Crawler 1 leg test.v5python`
  VEXcode project for a one-leg hardware test on a V5 brain.
  Configures:
  - `Pelvis` motor on port 1
  - `Hip` motor on port 2
  - `Knee` motor on port 3

- `3-dof-kinematic solution.py`
  Earlier standalone 2D/3-link IK derivation.

- `pygame path smoothing.py`
  Quick visual tool for drawing control points and generating a smoothed spline path.

## Leg Model

The leg is modeled with three segments:

- `Coxa = 5.197`
- `Femur = 7.974`
- `Tibia = 12.226`

The code treats the pelvis joint as rotation in the `x/z` plane, then solves the hip and knee in a 2D plane using Inverse kinematics.

## Repo Layout

- `Quadruped CAD/` CAD screenshots and printable/cuttable tibia parts
- `Quadruped pictures/` build photos
- `Drawings/` notebook/sketch images
- `VQW_Standup_test` very early motor write/stand-up stub

## Notes
- This is prototype code
- `IK testing.py` is the most complete file in the repo.
- Some files are partial or unfinished.

