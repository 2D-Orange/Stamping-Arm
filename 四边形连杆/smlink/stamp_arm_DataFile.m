% Simscape(TM) Multibody(TM) version: 23.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(23).translation = [0.0 0.0 0.0];
smiData.RigidTransform(23).angle = 0.0;
smiData.RigidTransform(23).axis = [0.0 0.0 0.0];
smiData.RigidTransform(23).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 75 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[base-1:-:link1_column-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-2.4726887204451486e-12 49.999999999998259 -2.7380600056183385e-14];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931962;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962595 -0.5773502691896254 0.57735026918962595];
smiData.RigidTransform(2).ID = "F[base-1:-:link1_column-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-12.500000000000011 91.22498999199199 -25.000000000000004];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = "B[link1_column-1:-:link2_upper_arm-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [6.8212102632969618e-13 7.1054273576010019e-13 -3.5527136788005009e-15];  % mm
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [0 0 0];
smiData.RigidTransform(4).ID = "F[link1_column-1:-:link2_upper_arm-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [12.500000000000011 60.000000000000007 -25.000000000000004];  % mm
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = "B[link1_column-1:-:link2_upper_arm-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [5.1159076974727213e-13 5.4001247917767614e-13 -1.7763568394002505e-14];  % mm
smiData.RigidTransform(6).angle = 0;  % rad
smiData.RigidTransform(6).axis = [0 0 0];
smiData.RigidTransform(6).ID = "F[link1_column-1:-:link2_upper_arm-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [10.000000000000009 -9.9999999999999947 7.5000000000000071];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(7).ID = "B[stamp_slider-1:-:stamp_head-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [5.0000000000000266 61.537489991991052 4.9999999999999973];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [-1.2213621905924389e-16 0.70710678118654757 0.70710678118654757];
smiData.RigidTransform(8).ID = "F[stamp_slider-1:-:stamp_head-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [44.999999999999986 31.224989991991968 15.000000000000002];  % mm
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = "B[link3-1:-:link3_forearm-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [1.1756503524278465e-12 -3.4106051316484859e-13 27.499999999999947];  % mm
smiData.RigidTransform(10).angle = 3.7673693750117816e-16;  % rad
smiData.RigidTransform(10).axis = [2.2204460492503121e-16 1 4.1826202224057641e-32];
smiData.RigidTransform(10).ID = "F[link3-1:-:link3_forearm-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [19.999999999999989 0 15.000000000000002];  % mm
smiData.RigidTransform(11).angle = 0;  % rad
smiData.RigidTransform(11).axis = [0 0 0];
smiData.RigidTransform(11).ID = "B[link3-1:-:link3_forearm-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [1.2959716420098391e-12 0 27.499999999999961];  % mm
smiData.RigidTransform(12).angle = 6.1798973242395742e-16;  % rad
smiData.RigidTransform(12).axis = [0 1 0];
smiData.RigidTransform(12).ID = "F[link3-1:-:link3_forearm-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [0 250 0];  % mm
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = "B[link2_upper_arm-2:-:link3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [-25.00000000000199 31.224989991990242 -12.500000000000027];  % mm
smiData.RigidTransform(14).angle = 0;  % rad
smiData.RigidTransform(14).axis = [0 0 0];
smiData.RigidTransform(14).ID = "F[link2_upper_arm-2:-:link3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [0 250 0];  % mm
smiData.RigidTransform(15).angle = 0;  % rad
smiData.RigidTransform(15).axis = [0 0 0];
smiData.RigidTransform(15).ID = "B[link2_upper_arm-3:-:link3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-1.8189894035458565e-12 -1.5347723092418164e-12 -12.500000000000012];  % mm
smiData.RigidTransform(16).angle = 0;  % rad
smiData.RigidTransform(16).axis = [0 0 0];
smiData.RigidTransform(16).ID = "F[link2_upper_arm-3:-:link3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [0 300 0];  % mm
smiData.RigidTransform(17).angle = 0;  % rad
smiData.RigidTransform(17).axis = [0 0 0];
smiData.RigidTransform(17).ID = "B[link3_forearm-2:-:stamp_slider-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [-5.8546856234986198 -4.6874999999998117 37.499999999999957];  % mm
smiData.RigidTransform(18).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(18).axis = [1 -1.3833724075474375e-47 -6.1431711790071143e-17];
smiData.RigidTransform(18).ID = "F[link3_forearm-2:-:stamp_slider-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [0 300.00000000000011 0];  % mm
smiData.RigidTransform(19).angle = 0;  % rad
smiData.RigidTransform(19).axis = [0 0 0];
smiData.RigidTransform(19).ID = "B[link3_forearm-3:-:stamp_slider-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [-30.854685623498529 26.537489991991983 37.499999999999986];  % mm
smiData.RigidTransform(20).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(20).axis = [1 3.1483077026210886e-47 5.9194685671318487e-17];
smiData.RigidTransform(20).ID = "F[link3_forearm-3:-:stamp_slider-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [0 30 0];  % mm
smiData.RigidTransform(21).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(21).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(21).ID = "B[table-1:-:base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [197.34814788625451 1.7598822404954602e-12 32.460876849923793];  % mm
smiData.RigidTransform(22).angle = 2.1909401290968149;  % rad
smiData.RigidTransform(22).axis = [-0.51468473313786267 -0.60625886611034652 -0.60625886611034729];
smiData.RigidTransform(22).ID = "F[table-1:-:base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [0 0 0];  % mm
smiData.RigidTransform(23).angle = 0;  % rad
smiData.RigidTransform(23).axis = [0 0 0];
smiData.RigidTransform(23).ID = "RootGround[table-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(8).mass = 0.0;
smiData.Solid(8).CoM = [0.0 0.0 0.0];
smiData.Solid(8).MoI = [0.0 0.0 0.0];
smiData.Solid(8).PoI = [0.0 0.0 0.0];
smiData.Solid(8).color = [0.0 0.0 0.0];
smiData.Solid(8).opacity = 0.0;
smiData.Solid(8).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.11780972450961727;  % kg
smiData.Solid(1).CoM = [0 15.499999999999998 0];  % mm
smiData.Solid(1).MoI = [69.085585947848074 81.28870991163592 69.085585947848074];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(1).color = [1 1 1];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "base*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.20861587385212332;  % kg
smiData.Solid(2).CoM = [0 150 25.000000000000007];  % mm
smiData.Solid(2).MoI = [1423.3940211273334 50.114984454556513 1381.1588892313835];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0 1 1];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "link3_forearm*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 18.800000000000107;  % kg
smiData.Solid(3).CoM = [5.7815196188445912e-13 -161.59574468085231 0];  % mm
smiData.Solid(3).MoI = [2149645.4609929058 1861333.3333333433 2149645.4609929039];  % kg*mm^2
smiData.Solid(3).PoI = [5.8456086547108403e-10 1.4094628242311518e-09 3.0935805871039082e-09];  % kg*mm^2
smiData.Solid(3).color = [1 1 1];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "table*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.15057042348344837;  % kg
smiData.Solid(4).CoM = [0 50.142639714668313 0];  % mm
smiData.Solid(4).MoI = [172.33194743725716 45.500009876776851 181.06394622675984];  % kg*mm^2
smiData.Solid(4).PoI = [0 0 -0.7663765559941933];  % kg*mm^2
smiData.Solid(4).color = [1 0.47058823529411764 1];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "link1_column*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.1504908738521234;  % kg
smiData.Solid(5).CoM = [0 127.80332613667025 24.999999999999996];  % mm
smiData.Solid(5).MoI = [644.24286018007672 40.138421954556506 609.80460328412664];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [0 1 1];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "link2_upper_arm*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.068279385756511879;  % kg
smiData.Solid(6).CoM = [10.000000000000002 19.677774129821785 12.154918616492148];  % mm
smiData.Solid(6).MoI = [13.665170852387815 30.081633602826951 36.729169832340979];  % kg*mm^2
smiData.Solid(6).PoI = [-0.095785882962142146 0 0];  % kg*mm^2
smiData.Solid(6).color = [1 1 0];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "link3*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.021280470994068101;  % kg
smiData.Solid(7).CoM = [-9.3483843511923084 6.5925970948909729 12.5];  % mm
smiData.Solid(7).MoI = [4.0740189542748881 7.457470182933136 9.2760565889423816];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 3.430523876194858];  % kg*mm^2
smiData.Solid(7).color = [1 1 0];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "stamp_slider*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 0.017621127501618747;  % kg
smiData.Solid(8).CoM = [0 15.430020721825484 0];  % mm
smiData.Solid(8).MoI = [13.995319510332333 1.6065684820187047 13.995319510332333];  % kg*mm^2
smiData.Solid(8).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(8).color = [1 0.54509803921568623 0];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = "stamp_head*:*默认";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the CylindricalJoint structure array by filling in null values.
smiData.CylindricalJoint(1).Rz.Pos = 0.0;
smiData.CylindricalJoint(1).Pz.Pos = 0.0;
smiData.CylindricalJoint(1).ID = "";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.CylindricalJoint(1).Rz.Pos = 0;  % deg
smiData.CylindricalJoint(1).Pz.Pos = 0;  % mm
smiData.CylindricalJoint(1).ID = "[link2_upper_arm-3:-:link3-1]";


%Initialize the PrismaticJoint structure array by filling in null values.
smiData.PrismaticJoint(1).Pz.Pos = 0.0;
smiData.PrismaticJoint(1).ID = "";

smiData.PrismaticJoint(1).Pz.Pos = 0;  % m
smiData.PrismaticJoint(1).ID = "[stamp_slider-1:-:stamp_head-1]";


%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(8).Rz.Pos = 0.0;
smiData.RevoluteJoint(8).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 9.3406763092406866;  % deg
smiData.RevoluteJoint(1).ID = "[base-1:-:link1_column-1]";

smiData.RevoluteJoint(2).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(2).ID = "[link1_column-1:-:link2_upper_arm-2]";

smiData.RevoluteJoint(3).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(3).ID = "[link1_column-1:-:link2_upper_arm-3]";

smiData.RevoluteJoint(4).Rz.Pos = -179.99999999999994;  % deg
smiData.RevoluteJoint(4).ID = "[link3-1:-:link3_forearm-2]";

smiData.RevoluteJoint(5).Rz.Pos = 179.99999999999997;  % deg
smiData.RevoluteJoint(5).ID = "[link3-1:-:link3_forearm-3]";

smiData.RevoluteJoint(6).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(6).ID = "[link2_upper_arm-2:-:link3-1]";

smiData.RevoluteJoint(7).Rz.Pos = 179.9999999999998;  % deg
smiData.RevoluteJoint(7).ID = "[link3_forearm-2:-:stamp_slider-1]";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.RevoluteJoint(8).Rz.Pos = 179.99999999999983;  % deg
smiData.RevoluteJoint(8).ID = "[link3_forearm-3:-:stamp_slider-1]";

