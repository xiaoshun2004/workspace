"""v0 全局配置 —— 所有可调参数集中在这里。"""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CALIB_DIR = PROJECT_ROOT / "calib"
CALIB_PATH = CALIB_DIR / "calib.json"

# ---------- 机械臂 ----------
ARM_IP = "192.168.5.1"
SPEED_LINEAR = 100          # 直线速度 mm/s（首次调试建议 50~100）
SPEED_JOINT = 80            # 关节速度 %

# ---------- 相机 ----------
CAM_W, CAM_H, CAM_FPS = 1280, 720, 30
CAM_ROTATE = "cw90"         # 画面旋转: "" 不转 / "cw90" 顺时针90 / "ccw90" 逆时针90 / "180"

# ---------- 检测 ----------
DETECT_METHOD = "yolo"      # yolo / hsv
YOLO_WEIGHTS = "/home/sam/yolo11n.pt"
YOLO_CONF = 0.25
YOLO_CLASSES = [41]         # COCO 类别 'cup' 的 id 是 41
HSV_LOWER = [0, 0, 180]     # 浅色纸杯 HSV 兜底（现场调）
HSV_UPPER = [180, 80, 255]
MIN_AREA = 1500

# ---------- 机器人坐标 (mm) ----------
# Z 为机器人坐标系下的绝对高度；Z=0 即桌面高度，标定时确认
Z_SAFE = 130                # 安全高度（杯顶 110mm + 余量）
Z_GRASP = 40                # 抓取高度 = 桌面 + 40mm
Z_PLACE = 5                 # 放置高度
APPROACH_BACK = 80          # 预抓取点沿 +X 退后距离
GRASP_R = [0, 0, 90]        # TODO 示教：夹爪水平时的姿态 [rx,ry,rz]（度）
HOME_R = [0, 0, 0]          # 夹爪竖直（默认姿态）
PLACE = (150, 150)          # 放置点 (x, y) mm
WORK_X_MIN, WORK_X_MAX = -250, 250
WORK_Y_MIN, WORK_Y_MAX = -250, 250

# ---------- 夹爪 LMG-90（0-100 对应行程 0-90mm） ----------
GRIPPER_OPEN = 100
GRIPPER_GRASP = 72          # 实测：夹住 72mm 杯身的位置（自锁到位，不控力度）
GRIPPER_FORCE = None        # 不控制力度（None = 跳过 set_force）
