# scene2 RRRP stamping arm

This folder contains the simplified scene2 stamping arm model and the
SolidWorks-exported URDF visualization pipeline.

Kinematic structure:

```text
base yaw R + upper arm pitch R + forearm pitch R + stamp prismatic P
```

Frame convention:

```text
Model X: radial outward at q1 = 0
Model Y: horizontal lateral
Model Z: vertical upward
```

The simplified FK/IK model uses this DH-aligned convention. The default modeled
visualization now imports the newest SolidWorks package under
`scene2/urdf/*/urdf/*.urdf` and creates a MATLAB-friendly
`scene2_solidworks_import.urdf` copy beside the exported meshes.

Geometry parameters:

| Parameter | Meaning | Value |
| --- | --- | --- |
| L1 | Base height | 0.15 m |
| L2 | Upper arm length | 0.25 m |
| L3 | Forearm length | 0.30 m |
| L_axis | Wrist joint axis to stamp prismatic axis horizontal offset | 0.11 m |
| H_stamp | Stamp face vertical offset when q4 = 0 | 0.01 m |
| q4 | Downward stamp extension | 0 to 0.12 m |

Run in MATLAB:

```matlab
cd('C:\Users\2D_Orange\Desktop\qrs3\stamp_arm 6.3\scene2')
codeDir = dir('01_*');
addpath(fullfile(pwd, codeDir(1).name))
run_scene2_pipeline
```

Default pipeline order:

```matlab
run_kinematics_solution                  % elbow-up IK and FK check
run_trajectory_planning                  % joint-space quintic trajectory
run_joint_pd_tracking_simulation         % joint-space PD tracking simulation
run_joint_pd_tracking_animation          % PD reference-vs-actual video
run_rrrp_kinematics_animation            % primitive, no-CAD animation
run_scene2_urdf_kinematics_visualization % SolidWorks URDF animation
```

Individual stages:

```matlab
run_kinematics_solution
run_trajectory_planning
run_joint_pd_tracking_simulation
run_joint_pd_tracking_animation
run_rrrp_kinematics_animation
run_scene2_urdf_kinematics_visualization
```

Outputs are written to the existing `02_*` and `03_*` folders. The animation is
`scene2_urdf_kinematics.*`, with start/middle/end snapshots saved beside the CSV
and MAT trajectory files. `scene2_solidworks_urdf_validation.csv` records the
stamp-face consistency check between the simplified FK and imported URDF.
`run_rrrp_kinematics_animation` remains the primitive geometry simulation that
does not use the SolidWorks URDF.

The joint-space PD tracking stage rebuilds IK and the quintic trajectory from
`stamp_robot_params.m` each time it runs. To move the ink box, paper stamping
point, safe height, or add/delete intermediate work points, edit
`params.taskNames` and `params.taskTargets` in that file. Keep the two lists the
same length. Each target row is:

```text
[x, y, z, press]
```

where `x y z` is the desired stamp-face position in meters, and `press` is `0`
for lifted, `1` for `params.q4_press`, or a direct q4 extension in meters.
The PD stage saves `joint_pd_tracking_samples.csv`,
`joint_pd_tracking_summary.csv`, `joint_pd_tracking_simulation.mat`, and the
`fig_*pd_tracking*.png` plots in `02_*`.
It also appends a configurable final hold segment, `params.pdSettlingTime`
seconds, to let the PD response settle at the home pose before the final error
is reported.
The PD animation stage reads `joint_pd_tracking_simulation.mat` and writes
`scene2_joint_pd_tracking.*` into `03_*`, with blue showing the reference motion
and red showing the actual PD response.

For the new SolidWorks exporter URDF, the visualization maps the simplified
model joints to URDF joints as:

```text
joint_1 = q1
joint_2 = -q2
joint_3 = -q3
joint_4 = q2 + q3
joint_5 = q4
```

The signs on `joint_2` and `joint_3` come from the exported joint-axis
directions. `joint_4` is the dependent wrist-leveling joint that keeps
`L_axis` horizontal and the stamp axis vertical.

Legacy/custom DH URDF tools are still available as separate utilities:
`run_scene2_urdf_joint_authoring` and `run_regenerate_scene2_urdf`. They edit
and regenerate the older `exported/stamp_arm.urdf` path, not the default
SolidWorks package under `scene2/urdf`.

See `docs/scene2_nonstandard_dh_frames.md` for the exact joint-frame standard.
