# lebai-gripper-demo

乐白（lebai）独立夹爪控制 Demo —— Python / RS485 / Modbus RTU

通过串口直接控制乐白独立夹爪（无需连接机械臂主机），包含命令行工具和键盘交互抓取 Demo。

## 硬件

- 乐白独立夹爪 **LMG-90**（RS485 / Modbus RTU）
  - 夹持力范围 **10–35 N**，力度参数 `force`（0–100）大致线性映射到此范围
  - 力控逻辑：闭合过程中抓到物体即停止，判断准则为设置的力度
  - 规格：[LMG-90 产品页](http://www.lebai.ltd/products/lmg-90/) ｜ [夹爪帮助文档](http://help.lebai.ltd/sdk/claw.html)
- USB 转串口适配器（WCH CH340/CH341），Linux 下通常为 `/dev/ttyACM0`

> 串口权限：确保当前用户属于 `dialout` 组，否则无法访问设备：
> ```bash
> sudo usermod -aG dialout $USER   # 之后需重新登录
> ```

## 安装

```bash
pip install pylebai==2.0.4
```

## 文件说明

| 文件 | 说明 |
|---|---|
| `gripper_cli.py` | 命令行控制工具（open / close / force / speed / calibrate / status），不带参数时进入交互模式 |
| `gripper_demo.py` | 键盘交互抓取 Demo，适合演示"抓取面包模型"等场景 |
| `gripper_control.py` | 最简单的顺序控制示例（校准→开→闭→读状态） |
| `docs/lebai-sdk-manual.md` | SDK 完整参考手册（从官方文档合并生成，见下方"参考"） |

## 用法

### CLI 工具

```bash
python3 gripper_cli.py status        # 查看状态（已校准/位置/力道/速度）
python3 gripper_cli.py open 80       # 张开到 80%
python3 gripper_cli.py close         # 闭合到 0%
python3 gripper_cli.py force 50      # 设置夹持力道 50%
python3 gripper_cli.py calibrate     # 校准夹爪
python3 gripper_cli.py               # 交互模式：输入 open / close / status ...
```

串口不在默认路径时用 `--port /dev/ttyACM0` 指定。

### 键盘交互 Demo（抓取面包模型）

```bash
python3 gripper_demo.py
```

| 按键 | 功能 |
|---|---|
| `g` | 抓取（闭合夹爪） |
| `r` | 释放（张开夹爪） |
| `↑` / `↓` | 夹持力度 +10 / -10 |
| `←` / `→` | 开合速度 -10 / +10 |
| `h` | 半开（50%） |
| `q` | 退出 |

> 建议运行 `gripper_demo.py` 前先用 `gripper_cli.py status` 确认通信正常。

## 参考

SDK 参考手册源自乐白官方文档：
<https://lebai-robotics.github.io/lebai-sdk/> （Doxygen 生成，v2.0.4）

## 许可证

MIT，详见 [LICENSE](LICENSE)。
