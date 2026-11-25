# 手眼标定（ChArUco + D405 + Nova5）

## 适用场景

本项目是 **eye-in-hand（眼在手上）**：D405 相机装在机械臂末端，
标定板固定在工作台上。机械臂走到不同位姿，每拍一张图记录一次末端位姿，
共 20 组。

标定结果：

```
X = 相机坐标系 -> 机械臂末端坐标系
```

拿到 `X` 后，目标在相机系下的坐标 `p_cam` 先换算到末端系，再用
机械臂当前基座->末端变换 `FK` 换算到基座系：

```
p_gripper = X · p_cam
p_base    = FK · p_gripper
```

> 如果以后改回相机固定俯拍（eye-to-hand），标定命令加 `--mode eye-to-hand`，
> 此时 `X` 直接是 相机->基座 变换。

## 标定板参数（默认已写入脚本）

| 参数 | 值 |
|---|---|
| 内角点 | 5 列 × 7 行 |
| 棋盘格边长 | 32 mm |
| ArUco 码边长 | 24 mm |
| 字典 | DICT_4X4_250（若实体板是其他变体，改 `BOARD_DICT`） |

> 注意：OpenCV 的 `CharucoBoard` 参数是"格子数"，5×7 内角点对应
> **4×6 个格子**，脚本已自动处理。

## 数据准备

```
calib/handeye/
├── images/           # 20 张标定图（未旋转原始图，jpg/png 均可）
└── poses.txt         # 每张图对应的机械臂末端位姿，与 images 按文件名排序一一对应
```

`poses.txt` 每行一个位姿（默认 mm + 度，Rz@Ry@Rx）：

```
# x, y, z, rx, ry, rz
12.34, 56.78, 100.0, 15.0, 20.0, 30.0
...
```

如果记录的是四元数，用 7 列 `x,y,z,qw,qx,qy,qz`，
标定时加 `--pose-format quat`。

## 使用步骤

### 1. 导出 D405 出厂内参（推荐，免去内参自标定）

```bash
cd src
python3 dump_intrinsics.py
```

生成 `calib/d405_intrinsics.json`。

### 2. 检测 + 标注

```bash
python3 charuco_handeye.py detect
```

检查 `calib/handeye/out/overview.jpg`：

- 每张图应检出 ≥10 个角点（20 张全部 OK 为佳）；
- 角点编号与坐标轴方向正确；
- 若有几张图角点少/失败，重新拍或删掉对应位姿对。

### 3. 手眼标定

```bash
python3 charuco_handeye.py calibrate \
    --mode eye-in-hand \
    --intrinsics ../calib/d405_intrinsics.json
```

结果保存到 `calib/handeye/out/handeye_result.json`。

## 结果判读

脚本会打印两类残差：

| 指标 | 说明 | 参考值 |
|---|---|---|
| AX=XB 残差 | 每个位姿下 `A·X − X·B` 的旋转/平移差 | 旋转 <1°，平移 <5mm |
| C=AXB 一致性 | `A·X·B` 恒定矩阵的散度 | 同上 |

如果残差巨大，按顺序排查：

1. `poses.txt` 单位/格式（mm 还是 m、度还是弧度）；
2. 旋转顺序是否 Rz@Ry@Rx（若机械臂文档不同，改 `pose_to_matrix`）；
3. 模式是否选对（相机在末端 = eye-in-hand，相机固定 = eye-to-hand）；
4. 图片是否与位姿错位（排序不一致）；
5. 标定时末端姿态变化是否足够大（要绕多个不同轴旋转）。

## 备选：leo038/hand_eye_calibrate

这个仓库的思路（采集图 + 机械臂位姿 → `cv2.calibrateHandEye`）正确，
但直接用它有两个问题：

1. 它只支持**普通棋盘格**（`findChessboardCorners`），不支持 ChArUco；
2. 它写死 eye-in-hand，且 OpenCV 5.0 已移除 `cv2.calibrateHandEye`，
   本机无法直接运行。

本项目脚本已针对这两点重写：ChArUco 检测、内参自标定、AX=XB 求解全部自带。
