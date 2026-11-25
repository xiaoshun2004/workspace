# workspace

个人工作仓库 —— 机器人夹爪与视觉、智能硬件控制、3D 模型库的合集。

原先分散在多个独立仓库，现统一为 monorepo：每个项目是一个自包含的子目录，各自保留独立的 README 与依赖声明。

## 目录结构

```
workspace/
├── projects/                 代码项目
│   ├── nova5-cup-grasp/        Nova5 纸杯视觉抓取
│   ├── nova5-calib-manual/     Nova5 眼在手上标定工具
│   ├── lebai-gripper-sdk/      乐白夹爪 RS485/Modbus 独立控制
│   ├── pc-lebai-upper/         乐白 LMG-90 夹爪 PC 上位机
│   └── tasmota-plug-controller/ Tasmota 智能插头控制
└── models/                   3D 模型库
    ├── 夹爪与转接座/            转接座、法兰、末端执行器
    ├── 马达/                  舵机与马达安装结构件
    ├── 冷凝管/                 实验用冷凝管
    ├── 打印机配件/              Bambu / 拓竹周边改装件
    ├── 汽车模型/                造型展示模型
    ├── 手办与装饰/              手办、头盔、装饰件
    ├── 日用与收纳/              家居日用与收纳
    ├── 通用零件/               标准件与基础结构
    └── 待确认模型/              来源或用途待确认
```

## 项目一览

### Nova5 纸杯视觉抓取 — `projects/nova5-cup-grasp/`

视觉检测纸杯并完成侧抓放置的完整流程。目标是先跑通「检测 → 侧抓 → 放置」最短路径，再逐步优化。

### Nova5 眼在手上标定 — `projects/nova5-calib-manual/`

PySide6 桌面应用，用于 Dobot Nova-5 腕部相机标定。把标定流程编成「项目」存下来，之后可以像实例一样反复重放。

依赖有版本下限要求（`opencv-python>=4.8,<5`），原因写在 `requirements.txt` 里：4.7 以下没有 `cv2.aruco.CharucoDetector`，4.8 在 Python 绑定下 `matchImagePoints` 会返回错误的物点导致内参解算被污染，5.0 以上又不暴露 `cv2.calibrateHandEye`。

### 乐白夹爪 SDK Demo — `projects/lebai-gripper-sdk/`

通过 RS485 / Modbus RTU 串口直接控制乐白独立夹爪，不依赖机械臂主机。含命令行工具和键盘交互抓取 demo。

### 乐白 LMG-90 上位机 — `projects/pc-lebai-upper/`

PC 端上位机，经 DOBOT Nova 控制乐白 LMG-90 夹爪。通讯层只用 Python 标准库 socket，无第三方依赖。

联调经验总结见目录内 `乐白夹爪Nova5联调经验总结.md`。

### Tasmota 智能插头控制 — `projects/tasmota-plug-controller/`

命令行 + 网页 + 桌面 GUI 三种方式控制 Tasmota 智能插头，纯 Python 标准库零依赖。

核心结论：Tasmota 插头就是局域网里的一个小 HTTP 服务器，浏览器、curl、MQTT 都能控制。Ubuntu 完全够用，Windows 虚拟机没有优势。

### 3D 模型库 — `models/`

91 个模型文件，命名格式 `模型内容_YYYYMMDD.扩展名`（日期优先取原文件名中已有的）。

格式说明：`.stp/.step` 是 CAD 源文件，`.3mf` 是含模型与参数的切片工程文件，`.stl` 是网格模型，`.scad` 是 OpenSCAD 参数化源码。

完整对照表见 `models/_manifest.csv`。

## 克隆

```bash
git clone git@github.com:xiaoshun2004/workspace.git
```

只取代码部分（模型文件占 256 MB）：

```bash
git clone --filter=blob:none --sparse git@github.com:xiaoshun2004/workspace.git
cd workspace && git sparse-checkout set projects
```

## 说明

- 所有内容为个人设计与学习用途，版权归原作者所有。
- 纯打印指令类切片文件（只有 gcode、无模型几何）体积极大且可由模型重新生成，未纳入本仓库。
- 如需正式开源，请在根目录添加 LICENSE 文件。
