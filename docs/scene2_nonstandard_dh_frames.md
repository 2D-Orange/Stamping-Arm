# scene2 non-standard DH frame standard

This document defines the frame standard used by the scene2 RRRP URDF and
kinematics scripts. The purpose is to keep joint centers and joint axes tied to
the robot model instead of to SolidWorks mesh bounding boxes.

## Model coordinates

At the home configuration:

| Axis | Meaning |
| --- | --- |
| X | Radial outward from the base yaw axis |
| Y | Horizontal lateral direction |
| Z | Vertical upward direction |

SolidWorks scene2 STL meshes are exported in assembly coordinates. The URDF
writer maps them into the model frame with:

```text
CAD X = model Y
CAD Y = model Z
CAD Z = model X
```

Joint origins and axes below are not CAD mesh centers. They are the theoretical
RRRP joint centers.

## Geometry

| Parameter | Value |
| --- | --- |
| L1 | 0.150 m |
| L2 | 0.250 m |
| L3 | 0.300 m |
| L_axis | 0.110 m |
| H_stamp | 0.010 m |
| q4 range | 0 to 0.120 m |

## Home-frame points

| Point | Model coordinate |
| --- | --- |
| Base reference | `[0, 0, 0]` |
| Shoulder / base yaw axis point | `[0, 0, L1]` |
| Elbow | `[L2, 0, L1]` |
| Wrist | `[L2 + L3, 0, L1]` |
| Stamp prismatic axis | `[L2 + L3 + L_axis, 0, L1]` |
| Stamp face at q4 = 0 | `[L2 + L3 + L_axis, 0, L1 - H_stamp]` |

## URDF joints

| Joint | Parent -> child | Origin in parent frame | Axis in parent frame | Model mapping |
| --- | --- | --- | --- | --- |
| `base_yaw` | `base_link -> yaw_link` | `[0, 0, L1]` | `[0, 0, 1]` | `q1` |
| `shoulder_pitch` | `yaw_link -> upper_arm_link` | `[0, 0, 0]` | `[0, -1, 0]` | `q2` |
| `elbow_pitch` | `upper_arm_link -> forearm_link` | `[L2, 0, 0]` | `[0, -1, 0]` | `q3` |
| `wrist_level` | `forearm_link -> hand_link` | `[L3, 0, 0]` | `[0, 1, 0]` | `q2 + q3` |
| `stamp_prismatic` | `hand_link -> stamp_link` | `[L_axis, 0, -H_stamp]` | `[0, 0, -1]` | `q4` |

`wrist_level` is a dependent URDF joint used to keep the stamp axis vertical in
the simplified RRRP model. It is not an extra independent model coordinate.

## Transform chain

The kinematic model uses:

```text
T01 = Tz(L1) * Rz(q1)
T12 = Ry(-q2) * Tx(L2)
T23 = Ry(-q3) * Tx(L3)
T3a = Ry(q2 + q3) * Tx(L_axis)
Ta4 = Tz(-(H_stamp + q4))
```

The corresponding URDF visualization maps model coordinates to joints as:

```text
base_yaw        = q1
shoulder_pitch  = q2
elbow_pitch     = q3
wrist_level     = q2 + q3
stamp_prismatic = q4
```

## New SolidWorks exporter URDF mapping

The newly exported package in `models/urdf/*/urdf/*.urdf` keeps the SolidWorks
exporter joint names:

| Exported joint | Model mapping |
| --- | --- |
| `joint_1` | `q1` |
| `joint_2` | `-q2` |
| `joint_3` | `-q3` |
| `joint_4` | `q2 + q3` |
| `joint_5` | `q4` |

`joint_2` and `joint_3` are negated because the exported pitch axes are
`[0, 0, -1]` in the rotated SolidWorks joint frames. `joint_4` is still the
dependent wrist-leveling joint used to keep the stamp axis vertical.

When the visual authoring tool is used, edit `origin xyz` and `axis xyz` in the
parent link DH frame shown in the table above. The mesh transform is separate
and should not be used as the basis for joint center placement.
