# stamp_arm export

Generated from stamp_arm 6.3/stamp_arm.SLDASM using SOLIDWORKS COM automation.

Files:
- stamp_arm.stp: full assembly STEP export.
- components_step/*.stp: per-part STEP exports.
- meshes/*.stl: assembly-position STL meshes for the URDF.
- stamp_arm_fixed.urdf: fixed-pose visualization URDF using meshes already exported in assembly coordinates.
- simscape_multibody_xml_status.txt: Simscape Multibody XML status.

Notes:
- The URDF is fixed-pose visualization, not a recovered robot kinematic model.
- Mesh scale is set to 0.001 because SolidWorks STL exports are commonly millimeter-based.
- For a true Simscape Multibody XML, install/register MathWorks Simscape Multibody Link for SOLIDWORKS and export through that add-in.
