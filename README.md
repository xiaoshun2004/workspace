# Nova5 纸杯视觉抓取（v0）

> 最短路径跑通「检测 → 侧抓 → 放置」的完整流程，跑通后再逐步优化。

## 一句话

Dobot Nova 5 + 乐白 LMG-90 夹爪 + Intel RealSense D405 固定俯拍，
视觉定位纸杯（星巴克 Short 杯），侧抓并放到固定位置。

## 目录结构

```
nova5-cup-grasp/
├── README.md          # 本文件：项目概览
├── docs/
│   ├── requirements.md      # 需求文档
│   ├── layout-v0.svg/png    # 布局示意图
│   ├── hand-eye-calib.md    # ChArUco 手眼标定流程
│   └── hardware-wiring-lmg90.md  # 航插直连参考（与 v0 无关）
├── calib/             # 标定数据（calib.json）
└── src/
    ├── config.py      # 所有可调参数
    ├── nova5.py       # Nova5 TCP/IP V3 驱动
    ├── camera.py      # D405 相机封装
    ├── dump_intrinsics.py  # 导出 D405 出厂内参 JSON
    ├── charuco_handeye.py  # ChArUco 检测标注 + 手眼标定(AX=XB)
    ├── detect.py      # YOLO cup / HSV 检测
    ├── calibration.py # 单应标定
    ├── calibrate_gui.py # 交互式标定工具
    ├── gripper.py     # LMG-90 夹爪封装
    ├── manual_grasp.py # 手动抓取模式(人手持夹爪, 调夹爪参数)
    └── pick_cup.py    # v0 主流程
```

## 与旧工程的关系

- `文档/lebai/bread_grasp`：旧的面包抓取工程（已废弃，仅作参考素材，
  其中的 `nova5.py` TCP 驱动、`gripper_cli.py`、`calibrate_gui.py` 思路可复用）。
- `DexHand-021S-sdk`：灵巧手 SDK，第二阶段再用。

## v0 运行顺序（按验收门槛）

在 `src/` 目录下执行：

1. 相机出流：`python3 camera.py`
2. 检测：`python3 detect.py --method yolo`（按 h 切换 hsv 兜底）
3. 标定：`python3 calibrate_gui.py`（残差 < 3mm，保存到 calib/calib.json）
4. dry-run：`python3 pick_cup.py --dry-run`（只打印坐标，不动臂）

## 手眼标定（ChArUco 板）

相机装在机械臂末端（eye-in-hand），标定板固定在工作台，拍 20 张图并记录末端位姿：

```bash
cd src
python3 dump_intrinsics.py                     # 1. 导出 D405 出厂内参
python3 charuco_handeye.py detect              # 2. 检测 + 标注, 检查 overview.jpg
python3 charuco_handeye.py calibrate --mode eye-in-hand \
    --intrinsics ../calib/d405_intrinsics.json # 3. 手眼标定
```

详细说明见 `docs/hand-eye-calib.md`。
5. 单次抓放：`python3 pick_cup.py`（先固定杯子位置调参）
6. 循环：`python3 pick_cup.py --loop`

机械臂接入前可先跑 `python3 manual_grasp.py`：手持夹爪对准画面红圈，
夹爪覆盖杯中心即自动合爪（默认开启，2.5s 后自动张开；按 a 切换手动），
把 config.py 里的 `GRIPPER_GRASP` 调准（当前实测值 72，不控力度）。
默认深度带触发（相机俯拍看不到杯心的夹爪，杯壁会挡），盯杯外环形带：
夹爪在抓取高度(深度比桌面近 15~70mm)靠近杯侧即合爪。现场可调：
`--min-px 20 --frames 2`（更灵敏）、`--cup-r 120`（杯更大时）、
`--band-close 10 --band-far 80`（夹爪拿得更高时）。

首次调试前务必修改 `src/config.py`：`GRASP_R`（夹爪水平姿态）、
`GRIPPER_GRASP`/`GRIPPER_FORCE`（开合量与力度）需要真机示教/实测；
`CAM_ROTATE` 控制画面旋转（相机侧装时设为 "cw90"）。

## 当前状态

- [x] 需求定稿（见 `docs/requirements.md`）
- [x] v0 代码骨架（未上真机）
- [ ] 验收门槛 1：YOLO 俯视稳定检测
- [ ] 验收门槛 2：单应标定残差 < 3mm，TCP 误差 < 5mm
- [ ] 验收门槛 3：固定位置完整侧抓
- [ ] 验收门槛 4：随机 5 位置连续抓放 ≥ 4/5
