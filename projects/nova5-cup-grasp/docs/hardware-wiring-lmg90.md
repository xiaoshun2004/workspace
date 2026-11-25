# Nova5 + 乐白 LMG-90 接线可行性（航插直连）

日期：2026-08-06
状态：结论已基于一手来源核实，Nova5 实物接线前需实测

## 一句话结论

**电气和协议上可行**（两端都是 24V + RS485/Modbus RTU），但**没有现成的"两头航插"
成品线可直接对插**——两端航插型号不同，需要定制一根跳线，且 Dobot 工具口 24V
供电电流余量要实测。

## 两端规格（一手来源）

### 乐白 LMG-90（[官方产品/协议页](http://www.lebai.ltd/products/lmg-90/)）

- 供电：24V DC
- 通讯：RS485，115200 8N1，Modbus RTU，从站地址 1
- 内置控制器，重量 0.46kg，行程 0–90mm，夹持力 10–35N
- 关键寄存器：40000 幅度（0–100）、40001 力度（0–100）、40005 当前位置、
  40007 指令执行完成标志
- 上电 5 秒后自动执行"找行程"（张合一次），上电前确保夹爪周围无障碍

### Dobot Nova 5 / Nova 系列（[产品页](https://www.dobot.cn/products/nova-series/nova5.html?lang=cn) +
[Nova 2S 硬件手册 Tool I/O 章节](https://www.manualslib.com/manual/4097176/Dobot-Nova-2s.html?page=37)）

- Nova 5 支持 RS485 / Modbus RTU 通讯（485 接口 × 1）
- 法兰末端有 Tool I/O 航插（Nova 系列手册：Tool I/O interface）
- Nova 2S 手册引脚/线缆定义（Nova 系列同款，线缆型号 Lutronic FP-222460）：
  - 白 = 485A
  - 棕 = 485B
  - 绿 = DI_2（PNP 输入）
  - 其余针脚含 24V 输出 / GND 等（以实物手册为准）
- 工具口 24V 输出：最大 2A（持续时间不超过 1 秒）

## 直连要处理的三个点

### 1. 物理接口：需要定制跳线，不是拿来就插

乐白机械臂法兰口是乐白自己的航插，Nova5 法兰口是 Dobot 的航插（FP-222460 规格）。
两头型号不同，**乐白机械臂那根原装线插不到 Nova5 上**。正确做法是定制一根线：

- Nova5 端：对接法兰 Tool I/O 航插，取 485A / 485B / 24V / GND
- LMG-90 端：对接夹爪自带的 4pin 航插
- 中间按上述定义一一对应跳接

### 2. 供电电流：先试工具口，异常就外接电源

- Dobot 工具口 24V 标称最大 2A（短时），LMG-90 官方未公布峰值电流
- 实测方法：夹爪全速开合时观察 Nova5 是否报警/电压跌落
- 不稳妥就：24V 由独立电源供电，只把 485A/485B 接到 Nova5（共地 GND 必须接）

### 3. 主站配置：协议要配成一致

- Nova5 侧把 485 口配为 **Modbus RTU 主站**：115200、8N1、从站地址 1
- 用 Nova5 的 Modbus 指令读写 LMG-90 寄存器：
  - 写 40000 = 开合幅度（0–100）
  - 写 40001 = 夹持力度（0–100）
  - 读 40005 / 40007 = 位置 / 执行完成
- 控制方式与乐白机械臂的 `set_claw`（JSON-RPC）不同，代码需按 Modbus 重写

## 对 v0 的建议

**v0 继续用现在的方案**：LMG-90 独立 24V 供电 + USB 转 RS485 接电脑，
`gripper_cli.py` 直接可用，零硬件风险。

**直连法兰口作为后续优化**：省一根串口线和独立电源，但需要：
1. 定制航插跳线
2. 实测工具口电流
3. Nova5 侧 Modbus 主站配置 + 抓取逻辑改造

## 来源

- 乐白 LMG-90 官方产品/通讯协议页：http://www.lebai.ltd/products/lmg-90/
- 乐白夹爪帮助文档（机械臂端 set_claw）：http://help.lebai.ltd/sdk/claw.html
- Dobot Nova 5 产品页（RS485/Modbus RTU）：https://www.dobot.cn/products/nova-series/nova5.html?lang=cn
- Dobot Nova 2S 硬件手册 Tool I/O（485A/485B/24V 引脚）：https://www.manualslib.com/manual/4097176/Dobot-Nova-2s.html?page=37
- Dobot Nova 系列用户手册（Tool I/O interface，V1.3 2023-07-19）：https://www.manualslib.com/manual/3456857/Dobot-Nova-Series.html?page=35

> 待补：Nova 5 手册的 Tool I/O 完整引脚表（公开资料暂缺，以实物手册为准）；
> LMG-90 额定/峰值电流（官方页未给出，需实测）。
