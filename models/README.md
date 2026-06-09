# 模型与 URDF 说明

本目录保存盖章机械臂的 SolidWorks 模型、导出的 URDF 包，以及 MATLAB 可视化所需的 STL 网格。

## 当前使用的模型

默认仿真使用 `models/urdf/装配体1.SLDASM/` 下的 SolidWorks 导出包：

```text
models/urdf/装配体1.SLDASM/urdf/装配体1.SLDASM.urdf
models/urdf/装配体1.SLDASM/meshes/*.STL
```

MATLAB 脚本会读取上面的原始 URDF，并自动生成两个导入副本：

```text
scene2_solidworks_import.urdf
scene2_solidworks_pd_tracking.urdf
```

这两个副本用于放宽部分关节限位、修正 MATLAB 导入所需的网格路径、设置显示颜色，并对盖章末端的导入坐标做统一处理。它们是生成物，可以删除后由脚本重新生成。

## 机械臂结构

简化模型采用：

```text
底座偏航 R + 大臂俯仰 R + 小臂俯仰 R + 印章伸缩 P
```

坐标约定：

```text
模型 X：q1 = 0 时径向向外
模型 Y：水平侧向
模型 Z：竖直向上
```

## 几何参数

主要几何参数在 `01_程序/stamp_robot_params.m` 中维护：

| 参数 | 含义 | 当前值 |
| --- | --- | --- |
| `L1` | 底座高度 | `0.15 m` |
| `L2` | 大臂长度 | `0.25 m` |
| `L3` | 小臂长度 | `0.30 m` |
| `L_axis` | 腕部关节轴到印章伸缩轴的水平偏移 | `0.11 m` |
| `H_stamp` | q4 为 0 时印章工作面偏移 | `0.01 m` |
| `q4` | 印章向下伸长量 | `0 到 0.12 m` |

## URDF 关节映射

SolidWorks 导出的 URDF 与简化运动学模型之间使用如下映射：

```text
joint_1 = q1
joint_2 = -q2
joint_3 = -q3
joint_4 = q2 + q3
joint_5 = q4
```

`joint_2` 和 `joint_3` 前的负号来自 SolidWorks 导出关节轴方向。`joint_4` 是保持腕部水平的从动关节，使印章伸缩轴保持竖直。

## MATLAB 运行

推荐在项目根目录运行：

```matlab
run_project
```

如果只想运行与模型相关的动画，可以先运行前置轨迹或 PD 仿真，再单独运行：

```matlab
run_trajectory_planning
run_scene2_urdf_kinematics_visualization

run_joint_pd_tracking_simulation
run_joint_pd_tracking_urdf_animation
```

## 旧工具

`run_scene2_urdf_joint_authoring` 和 `run_regenerate_scene2_urdf` 是早期自定义 URDF 工具，默认写入旧的 `exported/stamp_arm.urdf` 路径。当前主流水线不依赖它们。
