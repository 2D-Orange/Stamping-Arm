# 盖章机械臂仿真项目

本项目用于复现 RRRP 结构盖章机械臂的运动学求解、关节空间轨迹规划、PD 控制跟踪仿真，以及基于 SolidWorks 导出 URDF 的三维动画可视化。

## 目录结构

```text
01_程序/        MATLAB 源程序和公共工具函数
02_仿真结果/    自动生成的 CSV、MAT、PNG 结果
03_视频/        自动生成的动画视频
models/         SolidWorks 零件、装配体和导出的 URDF/STL
docs/           说明文档和课程材料
run_project.m   推荐的一键复现入口
```

其中，其他电脑复现时必须保留：

```text
01_程序/*.m
models/urdf/装配体1.SLDASM/urdf/*.urdf
models/urdf/装配体1.SLDASM/meshes/*.STL
```

`02_仿真结果/` 和 `03_视频/` 是生成物，缺失时程序会重新生成。

## 环境要求

建议使用 MATLAB R2022b 或更新版本。必须安装：

- MATLAB 基础环境
- Robotics System Toolbox

程序会用到 `importrobot`、`show`、`getTransform`、`VideoWriter`、`exportgraphics`、`readtable`、`writetable` 等函数。

## 一键复现

在 MATLAB 中进入项目根目录，运行：

```matlab
run_project
```

`run_project.m` 会自动：

1. 添加 `01_程序/` 到 MATLAB 路径；
2. 执行环境自检；
3. 运行完整仿真流水线。

也可以手动运行：

```matlab
cd('你的项目根目录')
addpath(fullfile(pwd, '01_程序'))
check_project_environment(pwd)
run_scene2_pipeline
```

## 仿真流水线

完整流水线位于 `01_程序/run_scene2_pipeline.m`，执行顺序为：

```matlab
run_kinematics_solution                  % 逆运动学求解和正运动学校验
run_trajectory_planning                  % 五次多项式关节空间轨迹规划
run_joint_pd_tracking_simulation         % 关节空间 PD 控制跟踪仿真
run_joint_pd_tracking_animation          % 简化模型 PD 跟踪动画
run_joint_pd_tracking_urdf_animation     % SolidWorks URDF 的 PD 跟踪动画
run_rrrp_kinematics_animation            % 简化 RRRP 运动学动画
run_scene2_urdf_kinematics_visualization % SolidWorks URDF 轨迹动画
```

## 关键输出

运行完成后，主要结果会写入：

```text
02_仿真结果/kinematics_solution.csv
02_仿真结果/trajectory_planning.mat
02_仿真结果/trajectory_samples.csv
02_仿真结果/joint_pd_tracking_simulation.mat
02_仿真结果/joint_pd_tracking_summary.csv
02_仿真结果/scene2_solidworks_urdf_validation.csv
03_视频/scene2_joint_pd_tracking.*
03_视频/scene2_joint_pd_tracking_urdf.*
03_视频/scene2_rrrp_kinematics.*
03_视频/scene2_urdf_kinematics.*
```

视频优先输出 MP4。如果当前电脑的 MATLAB 不支持 MPEG-4 编码，程序会自动退回到 Motion JPEG AVI。

## 预期校验结果

正常情况下可以看到：

- FK/IK 盖章面位置误差在 `1e-15 m` 量级；
- 轨迹段边界速度和加速度连续性误差接近 0；
- URDF 与简化模型的末端位置校验误差在 `1e-6 m` 量级；
- PD 跟踪末端最大误差通常为数毫米量级，具体数值取决于 `stamp_robot_params.m` 中的任务点和 PD 参数。

## 修改任务点

盖章位置、抬起高度、按压量和任务顺序集中在：

```text
01_程序/stamp_robot_params.m
```

主要修改：

```matlab
params.taskNames
params.taskTargets
```

`taskTargets` 每一行格式为：

```text
[x, y, z, press]
```

其中 `x y z` 是盖章工作面的目标位置，单位为米；`press` 为 0 表示抬起，为 1 表示使用 `params.q4_press`，也可以直接填写 q4 伸长量。

## URDF 与模型文件

程序默认搜索：

```text
models/urdf/*/urdf/*.urdf
```

并要求同级包目录下存在：

```text
meshes/base_link.STL
meshes/link_1.STL
meshes/link_2.STL
meshes/link_3.STL
meshes/link_4.STL
meshes/link_5.STL
```

导入 MATLAB 前，脚本会生成 MATLAB 友好的 URDF 副本：

```text
models/urdf/装配体1.SLDASM/scene2_solidworks_import.urdf
models/urdf/装配体1.SLDASM/scene2_solidworks_pd_tracking.urdf
```

这两个文件是生成物，可以由脚本重新生成。

## 常见问题

如果提示找不到 `importrobot`，说明没有安装 Robotics System Toolbox。

如果提示找不到 URDF 或 STL，请确认 `models/urdf/装配体1.SLDASM/` 目录完整复制。

如果只生成 AVI 而不是 MP4，说明当前 MATLAB 或操作系统没有可用的 MPEG-4 写入器，不影响仿真结果。
