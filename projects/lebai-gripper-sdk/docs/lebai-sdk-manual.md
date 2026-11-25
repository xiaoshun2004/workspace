# lebai-sdk 手册目录

- [lebai](#lebai)
- [python应用文档](#python应用文档)
- [Java平台应用文档](#java平台应用文档)
- [.net平台应用文档](#net平台应用文档)
- [FAQ](#faq)
- [Develop](#develop)
- [ChangeLog](#changelog)
- [lebai Namespace Reference](#lebai-namespace-reference)
- [lebai::l_master::Gripper Class Reference](#lebailmastergripper-class-reference)
- [lebai::l_master::LuaRobot Class Reference](#lebailmasterluarobot-class-reference)
- [lebai::l_master::Robot Class Reference](#lebailmasterrobot-class-reference)
- [lebai::zeroconf::ControllerInfo Class Reference](#lebaizeroconfcontrollerinfo-class-reference)
- [lebai::zeroconf::Discovery Class Reference](#lebaizeroconfdiscovery-class-reference)
- [夹爪.](#夹爪)
- [机器人配置.](#机器人配置)
- [文件系统.](#文件系统)
- [通用输入输出](#通用输入输出)
- [灯板.](#灯板)
- [modbus.](#modbus)
- [机械臂运动.](#机械臂运动)
- [机器人.](#机器人)
- [场景.](#场景)
- [serial.](#serial)
- [信号量.](#信号量)
- [启动停止.](#启动停止)
- [机械臂状态.](#机械臂状态)
- [STORAGE](#storage)
- [lebai::l_master::BackupInfoData Struct Reference](#lebailmasterbackupinfodata-struct-reference)
- [lebai::l_master::BackupOptionsData Struct Reference](#lebailmasterbackupoptionsdata-struct-reference)
- [lebai::l_master::ButtonIndexData Struct Reference](#lebailmasterbuttonindexdata-struct-reference)
- [lebai::l_master::ButtonStatusData Struct Reference](#lebailmasterbuttonstatusdata-struct-reference)
- [lebai::l_master::CartesianLimitConfig Struct Reference](#lebailmastercartesianlimitconfig-struct-reference)
- [lebai::l_master::CheckUpgradeData Struct Reference](#lebailmastercheckupgradedata-struct-reference)
- [lebai::l_master::ClawData Struct Reference](#lebailmasterclawdata-struct-reference)
- [lebai::l_master::CollisionDetectorConfig Struct Reference](#lebailmastercollisiondetectorconfig-struct-reference)
- [lebai::l_master::CommandStdoutData Struct Reference](#lebailmastercommandstdoutdata-struct-reference)
- [lebai::l_master::DeviceInfoData Struct Reference](#lebailmasterdeviceinfodata-struct-reference)
- [lebai::l_master::DhParamData Struct Reference](#lebailmasterdhparamdata-struct-reference)
- [lebai::l_master::DirData Struct Reference](#lebailmasterdirdata-struct-reference)
- [lebai::l_master::DiscoveredRobotData Struct Reference](#lebailmasterdiscoveredrobotdata-struct-reference)
- [lebai::l_master::ExtraServoParamData Struct Reference](#lebailmasterextraservoparamdata-struct-reference)
- [lebai::l_master::FrameData Struct Reference](#lebailmasterframedata-struct-reference)
- [lebai::l_master::HardwareInfoData Struct Reference](#lebailmasterhardwareinfodata-struct-reference)
- [lebai::l_master::HttpRequestData Struct Reference](#lebailmasterhttprequestdata-struct-reference)
- [lebai::l_master::HttpResponseData Struct Reference](#lebailmasterhttpresponsedata-struct-reference)
- [lebai::l_master::JointLimitConfig Struct Reference](#lebailmasterjointlimitconfig-struct-reference)
- [lebai::l_master::JointMotionData Struct Reference](#lebailmasterjointmotiondata-struct-reference)
- [lebai::l_master::JointMoveData Struct Reference](#lebailmasterjointmovedata-struct-reference)
- [lebai::l_master::KinematicsForwardResp Struct Reference](#lebailmasterkinematicsforwardresp-struct-reference)
- [lebai::l_master::KinematicsInverseResp Struct Reference](#lebailmasterkinematicsinverseresp-struct-reference)
- [lebai::l_master::LedStyleData Struct Reference](#lebailmasterledstyledata-struct-reference)
- [lebai::l_master::MessageData Struct Reference](#lebailmastermessagedata-struct-reference)
- [lebai::l_master::ModbusData Struct Reference](#lebailmastermodbusdata-struct-reference)
- [lebai::l_master::ModbusRegisterData Struct Reference](#lebailmastermodbusregisterdata-struct-reference)
- [lebai::l_master::OtaStateData Struct Reference](#lebailmasterotastatedata-struct-reference)
- [lebai::l_master::PhysicalData Struct Reference](#lebailmasterphysicaldata-struct-reference)
- [lebai::l_master::PluginInfoData Struct Reference](#lebailmasterplugininfodata-struct-reference)
- [lebai::l_master::PluginStoreInfoData Struct Reference](#lebailmasterpluginstoreinfodata-struct-reference)
- [lebai::l_master::PoseData Struct Reference](#lebailmasterposedata-struct-reference)
- [lebai::l_master::PvatPointData Struct Reference](#lebailmasterpvatpointdata-struct-reference)
- [lebai::l_master::RobotInfoData Struct Reference](#lebailmasterrobotinfodata-struct-reference)
- [lebai::l_master::ServoParamData Struct Reference](#lebailmasterservoparamdata-struct-reference)
- [lebai::l_master::ShortcutData Struct Reference](#lebailmastershortcutdata-struct-reference)
- [lebai::l_master::SoftwareInfoData Struct Reference](#lebailmastersoftwareinfodata-struct-reference)
- [lebai::l_master::SoftwareItemInfoData Struct Reference](#lebailmastersoftwareiteminfodata-struct-reference)
- [lebai::l_master::StorageItem Struct Reference](#lebailmasterstorageitem-struct-reference)
- [lebai::l_master::StructureData Struct Reference](#lebailmasterstructuredata-struct-reference)
- [lebai::l_master::SystemInfoData Struct Reference](#lebailmastersysteminfodata-struct-reference)
- [lebai::l_master::TaskData Struct Reference](#lebailmastertaskdata-struct-reference)
- [lebai::l_master::TaskStdoutData Struct Reference](#lebailmastertaskstdoutdata-struct-reference)
- [lebai::l_master::TrajectoryData Struct Reference](#lebailmastertrajectorydata-struct-reference)
- [lebai::l_master::TriggerConditionData Struct Reference](#lebailmastertriggerconditiondata-struct-reference)
- [lebai::l_master::TriggerData Struct Reference](#lebailmastertriggerdata-struct-reference)
- [lebai::l_master::WrenchData Struct Reference](#lebailmasterwrenchdata-struct-reference)

---

# lebai

#### Table of Contents

- 概述
- SDK 组成
- 如何开始
- ChangeLog
- Python
- .net
- Java
- Develop
- FAQ \*

## 概述

Lebai机械臂All in one开发包。开发包通过C++实现和机械臂的通讯,提供和机械臂控制器交互的接口,并且通过[SWIG (https://www.swig.org/ "SWIG")生成其它各种语言的接口库.

您可以通过项目的README文档来了解如何构建和安装本开发包.

您也可以基于本开发包创建使用其它语言的开发包.

## SDK 组成

SDK 的公共接口主要分为四部分:

- Discovery: 用于在局域网内发现 Lebai 控制器和设备.
- Robot: 主要的机器人控制器接口, 包括运动、 IO、任务、配置和系统 RPC 调用.
- LuaRobot: 面向 Lua 风格控制器通信的兼容接口.
- Gripper: 直接 RS485 夹爪接口, 独立于 Robot 的 JSON-RPC 主控制链路.

## 如何开始

查阅 README 文档,了解如何构建和安装本开发包.

## ChangeLog

查阅 ChangeLog.

## Python

查看 python应用文档 来获取使用python开发相关的内容.

## .net

查看 .net平台应用文档 来获取在.net中使用C#开发相关的内容.

## Java

查看 Java平台应用文档 来获取使用Java开发相关的内容.

## Develop

如果需要开发sdk，添加新的功能，可以参考 Develop.

## FAQ \*

参考 FAQ.


---

# python应用文档

## 通过pypi安装

pip3 install pylebai

## 通过源码构建安装

### 编译python包

您可以参考项目的README文档来安装项目的依赖，然后进行构建编译。 如果所有的依赖都满足，您可以用下面的命令来构建python语言的支持：

cmake -S. -Bbuild -DBUILD\_PYTHON=ON

cmake --build build --target python\_package

### 安装

构建完成后，在`build/python/dist/`目录下会生成python的whl包，可以直接使用pip进行安装。

pip3 install build/python/dist/pylebai-xxx.whl

您可以通过如下命令来查看安装的包的信息。

pip3 show pylebai

您可以通过如下命令来卸载。

sudo pip3 uninstall pylebai

### 运行

安装完成后可以直接使用lebai包。

from pylebai import Robot

robot = Robot("172.17.0.5", True)

robot.movej([1.0, -1.0471975511965976, 1.3962634015954636, -0.17453292519943295, -1.0471975511965976, 0.0], 1.0, 1.0, 0.0, 0.0)

推荐应用层使用 `pylebai.Robot`。它是 Python 风格 facade，接受普通 `list`/`dict` 参数，并将常用 vector 返回值转换为 Python `list`。 `pylebai.l_master` 仍保留 SWIG 生成的低层接口，主要用于兼容和内部桥接。

## API

### Robot

启动停止
运动
状态数据
控制箱IO
法兰盘IO
拓展IO
夹爪
灯板
信号量
程序控制
机器人学
设备发现

#### 启动停止

#启动机器人(上电)

start\_sys()

#停止机器人(下电)

stop\_sys()

#关闭机器人电源(关机)

powerdown()

#停止运动(但不下电)

stop()

#紧急停止(急停)

estop()

#进入示教模式

teach\_mode()

#退出示教模式

end\_teach\_mode()

#暂停运动

pause()

#恢复运动

resume()

#### 运动

#关节运动

#p 关节位置或坐标位置，类型为dict

#a 主轴的关节加速度(rad/s.pow(2))

#v 主轴的关节速度(rad/s)

#t 运动时间(s).当t>0时，速度参数v和加速度a无效

#r 交融半径(m).0表示关闭平滑运动;1表示启用平滑运动(最大交融半径为1m)

#return 整型数字代表此运动的id,若为0则启动运动失败

movej(p,a,v,t,r)

#p 坐标位置，或者关节位置(将通过运动学正解转为坐标位置)

#a 工具空间加速度 (m/s^2^)

#v 工具空间速度 (m/s).注意这里的速度和加速度指的是在工具空间上的描述。

#其他参数同movej

#return 整型数字代表此运动的id,若为0则启动运动失败

movel(p, a, v, t, r)

#p 关节位置，或者坐标位置(将通过运动学反解转为关节位置)

#v 每个关节的速度 (rad/s).如该值为数字，则表示所有关节速度相同。

#a 每个关节的加速度 (rad/s2).如该值为数字，则表示所有关节加速度相同。

#t 运动时间 (s)

move\_pvat(p, v, a, t)

#等待运动结束

#id 指定运动的id.如该值为0，则表示等待所有运动完成

wait\_move(id)

#### 状态数据

#获取机器人当前状态

#return 整形状态码,代表机器人当前的状态

## 状态码 | 状态 | 说明

## ------ | ------------- | -------------

## -1 | 控制系统故障 | 机器人软件控制系统异常

## 0 | 硬件通讯故障 | 机器人硬件通讯故障

## 1 | 已急停 | 机器人处于急停状态，请确认安全性

## 2 | 初始化中 | 机器人初始化中

## 4 | 初始化完成 | 机器人电源已开启

## 5 | 空闲 | 机器人处于空闲状态

## 6 | 暂停 | 机器人处于暂停中状态

## 7 | 运行中 | 机器人运行中

## 8 | 更新中 | 机器人系统更新中

## 9 | 启动中 | 机器人初始化完成到空闲的启动过程中

## 10 | 正在停止 | 机器人空闲状态转到停止状态

## 11 | 示教中 | 机器人处于示教模式中

## 12 | 已停止 | 机器人处于停止状态，非急停状态

get\_robot\_mode()

#获取反馈关节位置

#return 包含"j1"-"j6"的字典，代表每个关节的位置

get\_actual\_joint\_positions()

#获取规划关节位置

#return 包含"j1"-"j6"的字典，代表每个关节的位置

get\_target\_joint\_positions()

#获取反馈关节速度

#return 包含6个元素的列表，代表每个关节的速度

get\_actual\_joint\_speed()

#获取规划关节速度

#return 包含6个元素的列表，代表每个关节的速度

get\_target\_joint\_speed()

#获取反馈工具位置

#return 包含6个元素的切片(slice)，代表tcp位置

get\_actual\_tcp\_pose()

#获取规划工具位置

#return 包含6个元素的切片(slice)，代表tcp位置

get\_target\_tcp\_pose()

#获取关节温度

#joint 关节编号

#return 指定关节的温度

get\_joint\_temp(joint)

#获取关节反馈力矩

#return 一个包含6个元素的列表，代表每个关节的力矩

get\_actual\_joint\_torques()

#获取关节规划力矩

#return 一个包含6个元素的列表，代表每个关节的力矩

get\_target\_joint\_torques()

#### 控制箱IO

#设置数字输出

#pin 端口.从0开始

#value 待设置的值，可以是1/0，也可以是True/False

set\_do(pin,value)

#获取数字输入

#pin 端口.从0开始

#return True/False

get\_di(pin)

#设置模拟输出

#pin 端口库.从0开始

#value 待设置的值，可以是1/0，也可以是True/False

set\_ao(pin,value)

#获取模拟输入

#pin 端口.从0开始

#return True/False

get\_ai(pin)

#### 法兰盘IO

#设置数字输出

#pin 端口.从0开始

#value 待设置的值，可以是1/0，也可以是True/False

set\_flange\_do(pin,value)

#获取数字输入

#pin 端口.从0开始

#return True/False

get\_flange\_di(pin)

#### 拓展IO

#设置数字输出

#pin 端口.从0开始

#value 待设置的值，可以是1/0，也可以是True/False

set\_extra\_do(pin,value)

#获取数字输入

#pin 端口.从0开始

#return True/False

get\_extra\_di(pin)

#设置模拟输出

#pin 端口库.从0开始

#value 待设置的值，可以是1/0，也可以是True/False

set\_extra\_ao(pin,value)

#获取模拟输入

#pin 端口.从0开始

#return True/False

get\_extra\_ai(pin)

#### 夹爪

#设置夹爪参数

#force 力度(0-100)

#amplitude 张合幅度(0-100)

set\_claw(force,amplitude)

#获取夹爪参数

#return 一个元组tuple,第一个数为夹爪力度，第二个数为幅度，第三个数为夹爪是否稳定

get\_claw()

#### 灯板

#设置led灯状态

#mode 亮灯模式.0：不变；1：关闭；2：常亮；3：呼吸；4：均分旋转；5：同色旋转；6：闪烁

#speed 速度.1：快速；2：正常；3：慢速

#color 最多包含 4 个 0 ~ 15 之间的整数

set\_led(mode,speed,color)

#设置声音

#voice 声音列表

#volume 音量.0：静音；1：低；2：正常；3：高

set\_voice(voice,volume)

#开关风扇

#status 是否开启风扇.1为关闭，2为开启

set\_fan(status)

#### 信号量

#设置信号量

#index 信号量下标(0-255)

#value 待设置的信号量值(32位有符号整形)

set\_signal(index,value)

#获取信号量

#index 信号量下标(0-255)

#return 设置的信号量值(32位有符号整形)

get\_signal(index)

#增加指定下标的信号量值

#index 信号量下标(0-255)

#value 待设置的信号量值(32位有符号整形)

add\_signal(index,value)

#### 程序控制

#调用指定场景

#id 所调用的场景的id

#return 执行此场景的taskid

scene(id)

#查询任务列表

#return 包含多个taskid的列表

load\_task\_list()

#暂停任务

#id 指定暂停的任务的taskid

#time 暂停的时间

#wait 是否等待恢复

pause\_task(id,time,wait)

#恢复任务

#id 要恢复的暂停任务的taskid

resume\_task(id)

#取消任务

#id 要取消的队列中的任务的taskid

cancel\_task(id)

#### 机器人学

#通过正向运动学将关节角度转成笛卡尔位置和姿态

#joints 关节角度.包含"j1"-"j6"的字典

#return 返回包含一个bool值和string-double键值对的字典，bool表示正解是否成功

kinematics\_forward(joints)

#通过逆向运动学反解，将笛卡尔位置和姿态转成关节角度

#vector 包含6个元素的列表，代表笛卡尔坐标

#joints 包含6个元素的列表，代表关节初始位置

kinematics\_inverse(vector,joints)

#将a和b转为4x4的齐次矩阵A和B，然后将C=AB转为位姿返回

#a 位姿a

#b 位姿b

#return 返回相乘后的位姿

pose\_times(a,b)

#将a转为4x4的齐次矩阵A,返回A的逆的位姿表示

#a 位姿a

#return 逆矩阵的位姿

pose\_inverse(a)

example:

#原始关节角度

jpose0={'j1':-0.2, 'j2':2.7, 'j3':-4.0, 'j4':-2.3, 'j5':1.6, 'j6':-1.3}

fk\_resp = robot.kinematics\_forward(jpose0)

#经正向转化后的笛卡尔位姿，ok=True代表转化成功

#{1=0.36081962328886585,2=-0.19076037719926695,3=0.3537290579173756,4=-0.7527842092971923,5=-1.036487456216255,6=-2.623180719146089,ok=true}

print(fk\_resp)

ik\_resp = robot.kinematics\_inverse(fk\_resp.pose)

#将笛卡尔位姿反向转化为关节角，ok=True代表转化成功

#{j1=-0.20000195232787238,j2=-1.613927211382733,j3=-1.4217944465440524,j4=-3.7058747067503983,j5=-1.6000033762300137,j6=1.841591302572852,ok=true}

print(ik\_resp)

#经过两次转化后的关节角与原始关节角是相同的，且精度更高

#{1=0.3608198951635695,2=-0.19076058061714882,3=0.35372925635352953,4=-0.7527992994328961,5=-1.0364868722925928,6=-2.6231651290324502,ok=true}

print(robot.kinematics\_forward(ik\_resp.joint\_positions))

#### 设备发现

#返回当前局域网内最多16台可发现的设备，并返回控制器信息（包含信息：hostname、ip\_address、mac\_address、model、ds\_version、rc\_version、id

resolve()

example:

#从lebai导入设备发现的模块

from pylebai import Robot, zeroconf

#实例化一个Discovery类用来存储设备发现的信息

d = zeroconf.Discovery()

#搜索局域网内设备，返回一个最多16个元素的列表

controllers = d.resolve()

#根据局域网内发现的设备，建立连接

robot = Robot(controllers[0].ip\_address)


---

# Java平台应用文档

## 安装

Java 构件发布到 Maven Central 后，可以在应用项目中依赖主包：

<dependency>

<groupId>io.github.liufang-robot</groupId>

<artifactId>lebai-java</artifactId>

<version>2.0.4</version>

</dependency>

Java 源码包名仍为 `org.sdk.lebai`，用于保持现有 SWIG API 命名空间兼容。

应用代码推荐使用 `org.sdk.lebai.Robot`。它是 SWIG 绑定外层的 Java 封装： 常见向量参数使用基本类型数组传入，常见向量结果返回基本类型数组，会自动加载原生库， 并实现 `AutoCloseable`。

import org.sdk.lebai.Robot;

try (Robot robot = new Robot("172.17.0.5", true)) {

robot.startSys();

robot.moveJ(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.5, 0.5, 0.0, 0.0);

robot.waitMove(0);

}

较底层的 SWIG 生成机器人类在 v2.x 中是 `org.sdk.lebai.NativeRobot`。 当某个方法尚未由 `Robot` 封装时，可以通过 `Robot.getNative()` 调用底层对象。

## 夹爪

直接 RS485 夹爪接口由 SWIG 生成的 `org.sdk.lebai.Gripper` 类提供。 它独立于主 `Robot` JSON-RPC 连接，需要传入本机串口设备路径。

import org.sdk.lebai.Gripper;

import org.sdk.lebai.Loader;

Loader.loadNativeLibraries();

Gripper gripper = new Gripper("/dev/serial/by-id/usb-1a86\_USB\_Serial-if00-port0");

try {

gripper.set\_position(50);

} finally {

gripper.delete();

}

当前维护的 Java 示例是 `examples/gripper/GripperExample.java`，运行方式为：

GripperExample <serial\_port> [position]

`position` 是可选参数，默认值为 `50`，取值范围为 `0-100`。 Linux 上优先使用 `/dev/serial/by-id/` 下的稳定路径，而不是 `/dev/ttyUSB0`， 因为 USB 串口设备重连后可能重新编号。

## 本地构建

cmake -S . -B build-java \

-DBUILD\_JAVA=ON \

-DBUILD\_DOCUMENTATION=OFF \

-DBUILD\_EXAMPLES=OFF \

-DBUILD\_TESTING=OFF \

-DSKIP\_GPG=ON

cmake --build build-java --target java\_package

构建会生成：

- 原生运行时 jar：`build-java/java/lebai-<platform-id>/target/*.jar`
- 主 Java jar：`build-java/java/lebai-java/target/*.jar`

## Maven Central 发布

发布工作流为 `.github/workflows/java_release.yml`。

它会为以下平台构建原生运行时 jar：

- `lebai-linux-x86-64`
- `lebai-linux-aarch64`
- `lebai-win32-x86-64`

随后构建聚合包 `lebai-java`，并声明对这些原生 jar 的运行时依赖。 所有 jar 会上传到名为 `lebai-java-jars` 的 GitHub Actions 构件， 并包含在顶层 GitHub Release 中。

如果配置了以下 GitHub 仓库 secrets，工作流还会将签名后的 Maven 部署上传到 Maven Central Portal：

- `MAVEN_CENTRAL_USERNAME`
- `MAVEN_CENTRAL_PASSWORD`
- `MAVEN_GPG_PRIVATE_KEY`
- `MAVEN_GPG_PASSPHRASE`

Maven server id 为 `central`。

与 `MAVEN_GPG_PRIVATE_KEY` 对应的公钥必须在发布前公开。 Maven Central 会使用公钥服务器校验签名，支持 `keyserver.ubuntu.com`、 `keys.openpgp.org` 和 `pgp.mit.edu`。发布任务会先导入配置的私钥、读取指纹， 然后确认能从这些服务器之一获取对应公钥，再开始上传到 Maven Central。

Central Portal 插件只在 `JAVA_ENABLE_MAVEN_CENTRAL=ON` 的部署步骤中启用。 普通本地 Java 包构建不会加载 Central Portal 插件。

`central.autoPublish` 默认值为 `true`，所以发布 CI 会通过 Central Portal 自动发布已校验的部署。CI 完成后，在 `https://central.sonatype.com/` 查看发布运行和已发布构件。


---

# .net平台应用文档

\*\*.NET 包目标支持 Linux x64、Linux arm64 和 Windows x64；具体平台可用性以 CI 产物和发布包内容为准。\*\*

[Nuget包地址 (https://www.nuget.org/packages/lebai/)

## 通过 NuGet 安装

dotnet add package lebai

`lebai` 包包含托管 C# 程序集，并在 NuGet 的 `runtimes/<rid>/native` 目录中携带对应平台的原生库。为了兼容历史构建，仓库仍可能生成 `lebai.runtime.<rid>` 这类运行时包，但普通用户不应该需要手动安装所有 运行时包。

Windows 的 C# 原生库按静态 MSVC runtime (`/MT`) 构建，因此用户不应该只 为了加载 `lebai-native.dll` 而额外安装 Visual C++ Redistributable。

## 示例工程运行

您可以自行创建.net工程，并且在CSharp中使用lebai包进行开发。

一个简单的示例步骤如下：

1. 创建工程

## 创建

mkdir lebai-sdk-dotnet-example

cd lebai-sdk-dotnet-example

dotnet new console

1. 添加lebai packages

dotnet add package lebai

1. 编写代码

在程序中添加如下代码

{c#}

using System;

using lebai;

## in Main

Robot robot = new Robot("xxx.xxx.xxx.xxx", true);

robot.StopSys();

Console.WriteLine($"stop...");

robot.StartSys();

Console.WriteLine($"start...");

double[] joints = {

0.0,

-60.0 / 180.0 \* Math.PI,

80.0 / 180.0 \* Math.PI,

-10.0 / 180.0 \* Math.PI,

-60.0 / 180.0 \* Math.PI,

0.0

};

robot.MoveJ(joints, 1.0, 1.0, 0.0, 0.0);

1. 编译运行

dotnet build

## run your console application.

## API 风格

推荐应用层使用 `lebai.Robot`。它是面向 C# 的轻量 facade，公开 `double[]`、`uint[]`、`int[]`、`string[]` 等普通 C# 类型，并在内部转换到 SWIG 生成的底层类型。

`lebai.l_master` 命名空间下仍包含 SWIG 生成的低层接口，例如 `DoubleVector` 和 `StrVector`。这些类型主要用于兼容和内部桥接，新代码不应优先直接使用。


---

# FAQ

## 如何在CMake项目中使用lebai c++包开发。

1. 通过deb包或者源码安装。
2. 创建您的CMake项目，在CMakeLists.txt中添加如下内容：

   ``` find\_package(lebai REQUIRED) target\_link\_libraries(your\_target\_name lebai::lebai-cpp) ```

## 机械臂界面上坐标系统中，Rz、Ry、Rx是如何定义的。

乐白机械臂界面的Rx,Ry,Rz表示机械臂姿态，是由EulerZYX确定的。

界面显示的Rz，Ry，Rx为角度，将其转换为弧度后，它和旋转矩阵的关系如下：

\[ RotationMatrix = \begin{bmatrix} \cos(Rz) & -\sin(Rz) & 0 \\ \sin(Rz) & \cos(Rz) & 0\\ 0 & 0 & 1 \end{bmatrix}\begin{bmatrix} \cos(Ry) & 0 & \sin(Ry)\\ 0 & 1 & 0 \\ -\sin(Ry) & 0 & \cos(Ry) \end{bmatrix}\begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos(Rx) & -\sin(Rx) \\ 0 & \sin(Rx) & \cos(Rx) \end{bmatrix} \]

## 夹爪的轴位置关系

`gripper`中一共有6个关节，按照左右对称分布。

- gripper\_r\_joint1
- gripper\_r\_joint2
- gripper\_r\_joint\_finger
- gripper\_l\_joint1
- gripper\_l\_joint2
- gripper\_l\_joint\_finger

夹爪的张开角度`amplitude`由一个[0,100]的整数表示，0表示完全闭合，100表示完全打开。

夹爪的机械结构为类似于挖掘机的结构，因此关节角度需要满足以下关系：

\[ angle = \frac{60.0}{180.0} \times \pi \times \frac{amplitude}{100.0} = \frac{\pi \times amplitude}{300} \]

\[ \begin{aligned} gripper\\_r\\_joint1 &= angle \\ gripper\\_r\\_joint2 &= -angle \\ gripper\\_r\\_joint\\_finger &= -angle \\ gripper\\_l\\_joint1 &= angle \\ gripper\\_l\\_joint2 &= -angle \\ gripper\\_l\\_joint\\_finger &= -angle \end{aligned} \]


---

# Develop

## RPC Source Of Truth

Lebai RPC definitions are documented at <https://lebai-robotics.github.io/lebai-proto/>.

Use that site for service methods, requests, and responses. Controller wire method names are snake\_case mappings, for example proto `StartTeachMode` maps to wire RPC `start_teach_mode`; see `docs/rpc-protocol.md` before adding or renaming a method.

## Add Or Change A Robot RPC

Use the migrated SDK2-style path:

1. Check the proto docs and confirm the actual wire RPC name.
2. Add or update `nlohmann/json` DTOs in `sdk/src/protos_json`.
3. Add or update the public declaration in `sdk/include/lebai/robot.hh`.
4. Convert public parameters to DTOs in `sdk/src/robot.cc`.
5. Add or update the internal call in `sdk/src/robot_impl.hh` and `sdk/src/robot_impl.cc`.
6. Review SWIG bindings under `sdk/python`, `sdk/dotnet`, and `sdk/java` if the public API changed.
7. Add focused tests in `sdk/test`.

Do not reintroduce the old handwritten DTO layer. New robot RPC request and response types belong under `sdk/src/protos_json`.

## DTO Example

Prefer small aggregate structs with `NLOHMANN_DEFINE_TYPE_INTRUSIVE` when the JSON shape is direct:

namespace protos\_json::motion\_proto {

struct MotionIndex {

unsigned int id{};

NLOHMANN\_DEFINE\_TYPE\_INTRUSIVE(MotionIndex, id)

};

} // namespace protos\_json::motion\_proto

Use custom `from_json`/`to_json` only when the controller shape differs from the C++ field names or has optional fields.

## Implementation Example

Public wrapper in `sdk/src/robot.cc`:

int Robot::movej(const std::vector<double>& joint\_positions, double a,

double v, double t, double r) {

protos\_json::motion\_proto::MoveRequest req;

req.param.acc = a;

req.param.velocity = v;

req.param.time = t;

req.param.radius = r;

req.pose.kind = 1;

req.pose.joint.joint = joint\_positions;

const auto resp = impl\_->move\_joint(req);

return static\_cast<int>(resp.id);

}

Internal RPC call in `sdk/src/robot_impl.cc`:

protos\_json::motion\_proto::MotionIndex Robot::RobotImpl::move\_joint(

const protos\_json::motion\_proto::MoveRequest& req) {

return rpc\_client\_->Call<protos\_json::motion\_proto::MotionIndex>(

"move\_joint", {req});

}

## Verification

At minimum, build the C++ test targets and run the DTO test:

cmake --build build-work --target test\_json\_protos test\_robot

ctest --test-dir build-work -R TestJsonProtos --output-on-failure

When a simulator is running, use `127.0.0.1:3030` through `TEST_ROBOT_IP=127.0.0.1` and run the focused `test_robot` case for the changed API.


---

# ChangeLog

## 2.0.4

破坏性变更：移除无需鉴权的 `Robot::box_test()` 重载。调用方现在必须显式 传入质量平台的 `time` 和 `auth` 参数。

DIO 模式按照控制器协议序列化为 `INPUT` 和 `OUTPUT`，并要求内部请求 DTO 必须包含全部四个力控参数。在发送请求前校验 `move_pvat` 各向量的长度。

支持阻塞 JSON-RPC 调用并发执行，并根据方法类型设置超时时间。为 Lua 的 域名解析、连接和响应操作设置边界，支持控制器的两种响应结束符，并保留一次 读取中合并返回的后续响应。隔离每次 mDNS 发现操作的响应状态，并正确关联 PTR、SRV、TXT、A 和 AAAA 记录。

修正夹爪自动标定的禁用操作，安装公开的 Lua 头文件，强制校验 Python 构建 依赖版本，并要求所有 SDK 构建配置都包含 C++ 核心库。

## 2.0.3

简化各语言夹爪示例，使其只设置一个位置，并支持通过可选命令行参数传入位置。

## 2.0.2

添加 C++ 机器人夹爪 JSON-RPC 手动验证示例。 修正多处 C++ JSON-RPC DTO 与 `lebai-proto` 不一致的字段名和响应字段。 将力控参数 API 对齐 `SetForceModeParamRequest` 的 `mass`、 `force_threshold`、`torque_threshold` 字段。 Breaking: `set_force_mode_param` 签名由 `(damping, gain, max_vel)` 调整为 `(damping, mass, force_threshold, torque_threshold)`。

## 2.0.1

调整 C++ 示例和测试二进制输出目录，避免示例目标名和测试二进制混在 `build/bin` 下；机器人示例移除依赖外部插件商店的默认调用。

## 2.0.0

扩展 C++ RPC API 覆盖，补齐备份、法兰、硬件升级、订阅、DH、轨迹、 电机参数、质量平台和虚拟 IP 相关的 snake\_case RPC 接口，并继续使用 `json-rpc-cxx` + `nlohmann/json` DTO 迁移模式。

更新 SDK 版本号，作为 SDK2 风格 JSON-RPC API 迁移版本。

## 1.4.4

调整版本号，避免发布 Python 包时与已存在版本冲突

## 1.4.3

修正`speedl`和`speedj`接口的错误

添加 safety 相关接口，支持碰撞检测、碰撞力矩误差、安全限位和笛卡尔空间限位配置

## 1.4.2

支持 .net framework 4.8。

## 1.4.1

添加夹爪关节角度与张开角度之间的关系说明。

## 1.4.0

优化构建系统, 支持 Python 3.12

## 1.3.7

优化构建系统

调整 gripper 接口命名

## 1.3.5

优化 Windows 平台 python 包的构建支持

## 1.3.4

添加独立的 gripper 接口

## 1.3.3

添加 init\_claw 接口 load\_task\_list -> get\_task\_list 修正 修正 get\_motion\_state 接口

## 1.3.2

添加 get\_kin\_data 升级 dotnet sdk 为 8.0

## 1.3.1

get\_robot\_mode -> get\_robot\_state 添加 get\_phy\_data wait\_task 返回 string

## 1.2.2

添加`c#`语言和`java`语言捕获异常的处理。

## 1.2.1

添加`win32`流水线编译构建。

## autotoc\_md42

1.2.0 添加存储项的接口。

## 1.1.32

添加 wait\_task 接口

## 1.1.31

python 镜像升级

## 1.1.30

修正 ci 中的错误

## 1.1.29

修正 move\_pvat 不能正常工作的问题。

## 1.1.28

修正 c++安装包 CMake Config 的错误，添加使用文档。

## 1.1.27

调整构建。

## 1.1.25

优化代码格式和构建。

## 1.1.24

暂时移除 windows 上 python3.12 生成。

## 1.1.23

修正 pypi 生成的问题。

## 1.1.22

调整 python 构建 token。

## 1.1.19

调整 python 构建 token。

## 1.1.19

调整 python 构建 token。

## 1.1.18

尝试调整 python 包名字。

## 1.1.17

修正 python 不能捕获异常的问题。

## 1.1.16

添加接口`get_claw_data`返回数据结构而不是 tuple。

## 1.1.15

完善文档，python 支持更多版本，python 生成 aarch64 的包。

## 1.1.14

修正 IK 的 bug

## 1.1.13

独立的 FAQ 文件

## 1.1.12

修正 pypi 持续集成的若干问题。

## 1.1.10

添加设置串口波特率和校验位的接口

## 1.1.9

添加接口连接抛出异常的处理

## 1.1.8

speedl 添加参考坐标系

修正.net 包描述问题

修正注释和文档的一些 bug

## 1.1.7

README 添加多语言支持

## 1.1.1

添加 speedj 和 speedl 的实现和文档

## 1.1.0

添加 LuaRobot 对象，可以向机械臂发送 lua 指令

## 1.0.21

添加 is\_network\_connected 方法。

## 1.0.20

修正 windows 平台编译 WebSocket 的问题。

## 1.0.17

添加 python2 的构建选项 修正旋转部分数据表示 x 和 z 反了的问题 完善文档

## 1.0.14

文档更新，持续集成更新，修正程序的错误

## 1.0.11

文档更新，持续集成更新

## 1.0.10

支持 dotnet 的编译生成

## 1.0.9

修正文档错误

修正 rpc 的逻辑错误，该逻辑错误导致 rpc 调用可能卡死的问题。

## 1.0.8

添加关节跟随运动的接口 towardj

添加圆弧运动的接口 movec

调整位置数据结构，关节位置修改为数组，坐标位置修改为键值

修正多线程访问的 bug.

## 1.0.3 – 2022-11-17

修正多线程访问的 bug.

## 1.0.2 – 2022-11-06

修改字典数据在 python 中的的 print 显示。

修改 ik 计算的传入数据结构，关节位置数据初值修改为字典。

添加编译选项 `TEST_ROBOT_IP` ，该设置用来设置单元测试连接的测试机器人的 IP 地址，默认为 `127.0.0.1`。

## 1.0.1 – 2022-10-28

修改 get\_xxx\_joint\_positions 返回值为字典类型数据。

添加和完善若干数据类型的 `__repr__` 函数，使得这些数据类型可以直接打印。

添加 `set_do` , `set_flange_do`, `set_extra_do` 值参数为 int 类型的接口。

## 1.0.2 – 2022-11-06

修正字典类型数据打印的格式问题。

添加测试框架，采用 l-master 容器进行测试。


---

# lebai Namespace Reference

### Detailed Description

Copyright 2022 lebai.ltd

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

```
http://www.apache.org/licenses/LICENSE-2.0
```

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

Copyright 2022-2025 lebai.ltd

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

```
http://www.apache.org/licenses/LICENSE-2.0
```

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

Copyright 2022-2023 lebai.ltd

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

```
http://www.apache.org/licenses/LICENSE-2.0
```

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.


---

# lebai::l_master::Gripper Class Reference

独立的夹爪的主要接口对象，通过本对象的方法与夹爪进行数据交互.

`#include <gripper.hh>`

| Public Member Functions | |
|  | Gripper (const std::string &port\_name) |
|  | 构造Gripper对象. |

| virtual | ~Gripper () |
|  | 析构Robot对象. |

| void | set\_position (unsigned int position) |
|  | 设置夹爪幅度. 范围为0-100. |

| unsigned int | get\_current\_position () const |
|  | 获取夹爪当前位置. |

| void | set\_force (unsigned int force) |
|  | 设置夹爪夹持力道. 范围为0-100. |

| unsigned int | get\_current\_force () const |
|  | 获取夹爪当前力道. |

| void | set\_velocity (unsigned int velocity, bool persistent) |
|  | 设置夹爪开合速度. 范围为0-100. |

| unsigned int | get\_current\_velocity (bool persistent) const |
|  | 获取夹爪当前开合速度. |

| void | do\_calibration () |
|  | 执行夹爪校准动作. |

| bool | is\_calibrated () const |
|  | 获取夹爪是否已校准完成状态. |

| void | turn\_on\_auto\_calibration () |
|  | 打开自动校准功能. |

| void | turn\_off\_auto\_calibration () |
|  | 关闭自动校准功能. |

| Protected Attributes | |
| std::unique\_ptr< GripperImpl > | impl\_ |

### Detailed Description

独立的夹爪的主要接口对象，通过本对象的方法与夹爪进行数据交互.

### Member Function Documentation

### ◆do\_calibration()

| void lebai::l\_master::Gripper::do\_calibration | ( |  | ) |  |

执行夹爪校准动作.

Note
:   请求的寄存器地址为40007,0x9C47。

### ◆get\_current\_force()

| unsigned int lebai::l\_master::Gripper::get\_current\_force | ( |  | ) | const |

获取夹爪当前力道.

Note
:   请求的寄存器地址为40006,0x9C46。

Returns
:   当前力道.

### ◆get\_current\_position()

| unsigned int lebai::l\_master::Gripper::get\_current\_position | ( |  | ) | const |

获取夹爪当前位置.

Note
:   请求的寄存器地址为40005,0x9C45。

Returns
:   当前位置.

### ◆get\_current\_velocity()

| unsigned int lebai::l\_master::Gripper::get\_current\_velocity | ( | bool | **persistent** | ) | const |

获取夹爪当前开合速度.

Note
:   当 persistent=false 时，请求寄存器地址为40010,0x9C4A；当 persistent=true 时，请求寄存器地址为40011,0x9C4B。

Returns
:   当前速度.

### ◆is\_calibrated()

| bool lebai::l\_master::Gripper::is\_calibrated | ( |  | ) | const |

获取夹爪是否已校准完成状态.

Note
:   请求的寄存器地址为40008,0x9C48。

Returns
:   true 已校准完成，false 未校准.

### ◆set\_force()

| void lebai::l\_master::Gripper::set\_force | ( | unsigned int | **force** | ) |  |

设置夹爪夹持力道. 范围为0-100.

> **硬件规格补充（SDK 文档未写）**：LMG-90 夹爪的物理夹持力范围为 **10–35 N**，
> `force`（0–100）大致线性映射到此范围，**10 N 即最小档位**。力控逻辑：闭合过程中
> 抓到物体即停止闭合，判断准则为设置的力度大小。
> 参考：[LMG-90 产品页](http://www.lebai.ltd/products/lmg-90/) ｜
> [夹爪帮助文档](http://help.lebai.ltd/sdk/claw.html)

Note
:   请求的寄存器地址为40001,0x9C41。

Parameters
:   |  |  |
    | --- | --- |
    | force | 力道. |

### ◆set\_position()

| void lebai::l\_master::Gripper::set\_position | ( | unsigned int | **position** | ) |  |

设置夹爪幅度. 范围为0-100.

Note
:   请求的寄存器地址为40000,0x9C40。

Parameters
:   |  |  |
    | --- | --- |
    | Position | 位置. |

### ◆set\_velocity()

| void lebai::l\_master::Gripper::set\_velocity | ( | unsigned int | **velocity**, |
|  |  | bool | **persistent** |
|  | ) |  |  |

设置夹爪开合速度. 范围为0-100.

Note
:   当 persistent=false 时，请求寄存器地址为40010,0x9C4A；当 persistent=true 时，请求寄存器地址为40011,0x9C4B。

Parameters
:   |  |  |
    | --- | --- |
    | velocity | 速度. |
    | persistent | 是否写入掉电保持寄存器。 |

### ◆turn\_off\_auto\_calibration()

| void lebai::l\_master::Gripper::turn\_off\_auto\_calibration | ( |  | ) |  |

关闭自动校准功能.

Note
:   请求的寄存器地址为40090,0x9C9A。

### ◆turn\_on\_auto\_calibration()

| void lebai::l\_master::Gripper::turn\_on\_auto\_calibration | ( |  | ) |  |

打开自动校准功能.

Note
:   请求的寄存器地址为40090,0x9C9A。

### Member Data Documentation

### ◆impl\_

| |  | | --- | | std::unique\_ptr<GripperImpl> lebai::l\_master::Gripper::impl\_ | | protected |

内部实现数据结构，用户无需关注.

---

The documentation for this class was generated from the following file:

- sdk/include/lebai/gripper.hh


---

# lebai::l_master::LuaRobot Class Reference

机械臂的Lua接口，通过本对象向机械臂发送lua指令.

`#include <lua_robot.hh>`

| Public Member Functions | |
|  | LuaRobot (std::string ip) |
|  | 构造LuaRobot对象.可以通过该对象向机械臂发送lua指令，并且获取返回值。 |

| virtual | ~LuaRobot () |
|  | 析构LuaRobot对象. |

| void | send (const std::string &lua\_code) |
|  | 向机械臂发送lua指令. |

| void | send (const std::vector< std::string > &lua\_codes) |
|  | 向机械臂发送多行lua指令.这些指令会按照顺序执行 |

| std::string | call (const std::string &lua\_code) |
|  | 向机械臂发送lua指令，并且获取机械臂的返回数据. |

| Protected Attributes | |
| std::unique\_ptr< LuaRobotImpl > | impl\_ |

### Detailed Description

机械臂的Lua接口，通过本对象向机械臂发送lua指令.

### Constructor & Destructor Documentation

### ◆LuaRobot()

| |  |  |  |  |  |  | | --- | --- | --- | --- | --- | --- | | lebai::l\_master::LuaRobot::LuaRobot | ( | std::string | **ip** | ) |  | | explicit |

构造LuaRobot对象.可以通过该对象向机械臂发送lua指令，并且获取返回值。

可以参考Lua指令说明了解所有和机械臂相关的lua指令

Parameters
:   |  |  |
    | --- | --- |
    | ip | 机械臂IP地址. |

### Member Function Documentation

### ◆call()

| std::string lebai::l\_master::LuaRobot::call | ( | const std::string & | **lua\_code** | ) |  |

向机械臂发送lua指令，并且获取机械臂的返回数据.

示例代码:

```
auto ret = lua_robot.call("get_robot_mode()");
```

Note
:   该函数会发送lua指令给机械臂，并且读取机械臂的返回数据.

Parameters
:   |  |  |
    | --- | --- |
    | lua\_code | lua指令. |

Returns
:   std::string: 机械臂返回的数据.

### ◆send() [1/2]

| void lebai::l\_master::LuaRobot::send | ( | const std::string & | **lua\_code** | ) |  |

向机械臂发送lua指令.

示例代码:

```
lua_robot.send("start_sys()");
```

Note
:   该函数只会发送lua指令给机械臂，不会读取机械臂的返回数据.

Parameters
:   |  |  |
    | --- | --- |
    | lua\_code | lua指令. |

### ◆send() [2/2]

| void lebai::l\_master::LuaRobot::send | ( | const std::vector< std::string > & | **lua\_codes** | ) |  |

向机械臂发送多行lua指令.这些指令会按照顺序执行

示例代码:

```
std::vector<std::string> codes;
codes.push_back("start_sys()");
codes.push_back("movej({j1 = 1.0, j2 = 0, j3 = 0, j4 = 0, j5 = 0, j6 =
```

0}, 1.2, 0.2, 0, 0)"); codes.push\_back("movej({j1 = 0.0, j2 = 0, j3 = 0, j4 = 0, j5 = 0, j6 = 0}, 1.2, 0.2, 0, 0)"); codes.push\_back("sync()");
codes.push\_back("stop\_sys()"); lua\_robot.send(codes);

Note
:   该函数只会发送lua指令给机械臂，不会读取机械臂的返回数据.

Parameters
:   |  |  |
    | --- | --- |
    | lua\_codes | 多个（行）lua指令. |

### Member Data Documentation

### ◆impl\_

| |  | | --- | | std::unique\_ptr<LuaRobotImpl> lebai::l\_master::LuaRobot::impl\_ | | protected |

内部实现数据结构，用户无需关注.

---

The documentation for this class was generated from the following file:

- sdk/include/lebai/lua\_robot.hh


---

# lebai::l_master::Robot Class Reference

机械臂的主要接口对象，通过本对象的方法与机械臂进行数据交互.

`#include <robot.hh>`

| Public Member Functions | |
|  | Robot (std::string ip, bool simulator=false) |
|  | 构造Robot对象. |

| virtual | ~Robot () |
|  | 析构Robot对象. |

| std::tuple< int, std::string > | call (const std::string &method, const std::string &params) |
|  | 用JSON格式字符串调用机械臂的接口. |

| std::string | hello (const std::string &data) |
|  | 测试 JSON-RPC 连接. |

| void | set\_auto (int name, bool value) |
|  | 设置自动配置项，name 取值与 SDK2 保持一致: 1, 2, 3. |

| bool | get\_auto (int name) |
|  | 获取自动配置项，name 取值与 SDK2 保持一致: 1, 2, 3. |

| bool | is\_network\_connected () |
|  | 返回是否和机械臂的网络连接正常，如果网络连接异常，调用和机械臂交互的接口会抛出异常std::runtime\_error。 |

| void | start\_sys () |
|  | 启动机械臂（机械臂上使能）. |

| void | stop\_sys () |
|  | 停止机械臂（机械臂下使能）. |

| void | powerdown () |
|  | 关闭机器人电源（关机）. |

| void | stop () |
|  | 停止运动（但不下电）. |

| void | estop () |
|  | 紧急停止（急停）. |

| void | start\_teach\_mode () |
|  | 进入示教模式. |

| void | **teach\_mode** () |

| void | end\_teach\_mode () |
|  | 退出示教模式. |

| void | pause\_move () |
|  | 暂停运动. |

| void | **pause** () |

| void | resume\_move () |
|  | 恢复运动. |

| void | **resume** () |

| void | reboot () |
|  | 重新启动机箱 |

| int | move\_joint (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂关节移动 |

| int | **movej** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| int | move\_joint (const CartesianPose &cart\_pose, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂关节移动 |

| int | **movej** (const CartesianPose &cart\_pose, double a, double v, double t, double r) |

| int | move\_linear (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂直线移动 |

| int | **movel** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| int | move\_linear (const CartesianPose &cart\_pose, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂直线移动 |

| int | **movel** (const CartesianPose &cart\_pose, double a, double v, double t, double r) |

| int | move\_circular (const std::vector< double > &joint\_via, const std::vector< double > &joint, double rad, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂圆弧运动. |

| int | **movec** (const std::vector< double > &joint\_via, const std::vector< double > &joint, double rad, double a, double v, double t, double r) |

| int | move\_circular (const CartesianPose &cart\_via, const CartesianPose &cart, double rad, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂圆弧运动 |

| int | **movec** (const CartesianPose &cart\_via, const CartesianPose &cart, double rad, double a, double v, double t, double r) |

| int | speed\_joint (double a, const std::vector< double > &v, double t=0.0) |
|  | 通过关节速度矢量发送机械臂关节匀速运动 |

| int | **speedj** (double a, const std::vector< double > &v, double t=0.0) |

| int | speed\_linear (double a, const CartesianPose &v, double t=0.0, const CartesianPose &reference={{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}}) |
|  | 通过坐标速度矢量发送机械臂关节匀速运动 |

| int | **speedl** (double a, const CartesianPose &v, double t=0.0, const CartesianPose &reference={{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}}) |

| int | toward\_joint (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂关节自由移动. |

| int | **towardj** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| void | move\_pvat (std::vector< double > p, std::vector< double > v, std::vector< double > a, double t) |
|  | 伺服运动PVAT |

| void | wait\_move (unsigned int id) |
|  | 等待运动完成. |

| void | wait\_move () |
|  | 等待所有运动完成. |

| unsigned int | get\_running\_motion () |
|  | 查询当前正在运动的MotionId(无运动时返回上次MotionId). |

| std::string | get\_motion\_state (unsigned int id) |
|  | 查询指定MotionId的运动状态. |

| void | stop\_move () |
|  | 停止所有运动. |

| void | skip\_move () |
|  | 跳过当前运动. |

| int | get\_robot\_state () |
|  | 获取机器人状态码 |

| int | get\_estop\_reason () |
|  | 查看急停原因 |

| SystemInfoData | get\_system\_info () |
|  | 获取控制器系统信息 |

| RobotInfoData | get\_robot\_info () |
|  | 获取机器人设备信息 |

| HardwareInfoData | get\_hardware\_info () |
|  | 获取控制器硬件信息 |

| SoftwareInfoData | get\_software\_info () |
|  | 获取控制器软件信息 |

| HttpResponseData | http (HttpRequestData request) |
|  | 通过控制器发起HTTP请求. |

| void | **clean** (const BackupOptionsData &option) |

| void | **backup** (const std::string &file, const BackupOptionsData &option) |

| BackupInfoData | **get\_backup\_info** (const std::string &file) |

| void | **restore** (const std::string &file, const BackupOptionsData &option) |

| void | **set\_virtual\_ip** (const std::string &ifname, const std::string &ip) |

| std::vector< std::string > | get\_box\_devices (const std::string &prefix) |
|  | 获取控制箱dev设备列表 |

| std::vector< DirData > | get\_dirs () |
|  | 获取数据库目录列表 |

| void | create\_dir (DirData dir) |
|  | 创建数据库目录. |

| void | update\_dir (std::string from, std::string to) |
|  | 重命名数据库目录. |

| std::vector< ShortcutData > | get\_short\_poses () |
|  | 获取所有快捷路点 |

| void | set\_short\_pose (ShortcutData shortcut\_data) |
|  | 设置快捷路点 |

| ShortcutData | get\_short\_pose (unsigned int id) |
|  | 获取单个快捷路点 |

| std::vector< ShortcutData > | get\_short\_tasks () |
|  | 获取所有快捷任务 |

| void | set\_short\_task (ShortcutData shortcut\_data) |
|  | 设置快捷任务 |

| ShortcutData | get\_short\_task (unsigned int id) |
|  | 获取单个快捷任务 |

| std::vector< TriggerData > | get\_triggers () |
|  | 获取条件任务触发器列表 |

| void | set\_trigger (TriggerData trigger) |
|  | 设置条件任务触发器 |

| std::map< std::string, LedStyleData > | get\_led\_styles () |
|  | 获取声光交互样式集 |

| void | set\_led\_styles (std::map< std::string, LedStyleData > styles) |
|  | 设置声光交互样式集 |

| LedStyleData | load\_led\_style (std::string name, std::string dir="") |
|  | 查询声光样式. |

| void | save\_led\_style (std::string name, LedStyleData style, std::string dir="") |
|  | 保存声光样式. |

| void | set\_led\_style (std::string state, LedStyleData style) |
|  | 设置机器人状态对应的声光交互样式. |

| std::vector< std::string > | load\_led\_style\_list (std::string dir="") |
|  | 查询声光样式列表. |

| std::vector< ServoParamData > | get\_servo\_params () |
|  | 获取伺服参数 |

| void | **set\_servo\_params** (const std::vector< ServoParamData > &params) |

| void | **find\_zero** () |

| void | **set\_zero** (const std::vector< double > &pose, const std::vector< bool > &valids) |

| void | **set\_extra\_servo\_params** (const std::vector< ExtraServoParamData > &params, const std::vector< bool > &valids) |

| void | **reset\_extra\_servo\_params** (const std::vector< bool > &valids) |

| void | **set\_flange\_baud\_rate** (unsigned int baud\_rate) |

| WrenchData | get\_tcp\_force () |
|  | 获取末端受力 |

| void | set\_tcp\_force (const WrenchData &wrench) |
|  | 设置末端受力 |

| void | set\_force\_mode\_sensor (const std::string &sensor, unsigned int address) |
|  | 设置力控传感器. |

| void | set\_force\_mode\_param (double damping, double mass, double force\_threshold, double torque\_threshold) |
|  | 设置力控参数. |

| void | start\_force\_mode (const CartesianPose &limit, const WrenchData &wrench) |
|  | 开始力控模式. |

| void | end\_force\_mode () |
|  | 结束力控模式. |

| std::vector< PluginInfoData > | load\_plugins () |
|  | 查询已安装插件列表 |

| std::vector< PluginStoreInfoData > | get\_plugin\_store () |
|  | 查询插件商店列表. |

| PluginInfoData | load\_plugin (const std::string &name) |
|  | 查询已安装插件 |

| CommandStdoutData | run\_plugin\_cmd (const std::string &name, const std::vector< std::string > &params=std::vector< std::string >()) |
|  | 执行插件命令. |

| CommandStdoutData | enable\_plugin (const std::string &name) |
|  | 启用插件. |

| CommandStdoutData | disable\_plugin (const std::string &name) |
|  | 禁用插件. |

| void | restart\_plugin\_daemon (const std::string &name) |
|  | 重启插件守护进程. |

| std::vector< DiscoveredRobotData > | discover\_robots () |
|  | 通过控制器RPC发现机器人设备. |

| CommandStdoutData | get\_plugin\_daemon\_stdout (const std::string &name) |
|  | 获取插件守护进程输出. |

| std::vector< MessageData > | get\_messages () |
|  | 获取控制器消息列表 |

| OtaStateData | get\_ota\_state () |
|  | 获取OTA升级状态 |

| CheckUpgradeData | check\_upgrade () |
|  | 查询是否需要系统升级 |

| CommandStdoutData | get\_upgrade\_stdout () |
|  | 获取系统升级输出 |

| void | **sub\_buttons\_status** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **sub\_kin\_data** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **sub\_message** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **sub\_robot\_state** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **sub\_phy\_data** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **sub\_task\_stdout** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **start\_ota** (const std::string &address, const std::string &partition, const std::string &file) |

| void | **switch\_partition** (const std::string &address, const std::string &partition) |

| void | **start\_upgrade** () |

| int | box\_test (const std::string &time, const std::string &auth) |
|  | Run the quality-platform box test. |

| std::string | **init\_robot** (const std::string &time, const std::string &auth, const RobotInfoData &info) |

| PhysicalData | get\_phy\_data () |
|  | 获取机械臂物理数据 |

| JointMotionData | get\_kin\_data () |
|  | 获取机械臂关节运动数据 |

| bool | is\_disconnected () |
|  | 是否已与手臂断开连接 |

| bool | is\_down () |
|  | 手臂是否已下电 |

| std::vector< double > | get\_actual\_joint\_positions () |
|  | 获取机械臂关节当前反馈位置 |

| std::vector< double > | get\_target\_joint\_positions () |
|  | 获取机械臂关节当前控制位置 |

| std::vector< double > | get\_actual\_joint\_speed () |
|  | 获取机械臂关节当前反馈速度 |

| std::vector< double > | get\_target\_joint\_speed () |
|  | 获取机械臂关节当前控制力矩 |

| CartesianPose | get\_actual\_tcp\_pose () |
|  | 获取机械臂末端在机械臂基坐标系下的实际位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |

| CartesianPose | get\_target\_tcp\_pose () |
|  | 获取机械臂末端在机械臂基坐标系下的控制位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |

| double | get\_joint\_temp (unsigned int joint\_index) |
|  | 获取单个关节温度 |

| std::vector< double > | get\_actual\_joint\_torques () |
|  | 获取机械臂关节当前反馈力矩 |

| std::vector< double > | get\_target\_joint\_torques () |
|  | 获取机械臂关节当前控制力矩 |

| void | set\_do (std::string device, unsigned int pin, unsigned int value) |
|  | 设置数字输出  |

| void | set\_dos (std::string device, unsigned int pin, std::vector< unsigned int > values) |
|  | 设置多个数字输出 |

| unsigned int | get\_do (std::string device, unsigned int pin) |
|  | 获取数字输出 |

| std::vector< unsigned int > | get\_dos (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个数字输出 |

| unsigned int | get\_di (std::string device, unsigned int pin) |
|  | 获取数字输入 |

| std::vector< unsigned int > | get\_dis (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个数字输入 |

| void | set\_ao (std::string device, unsigned int pin, double value) |
|  | 设置模拟输出 |

| void | set\_aos (std::string device, unsigned int pin, std::vector< double > values) |
|  | 设置多个模拟输出 |

| double | get\_ao (std::string device, unsigned int pin) |
|  | 获取模拟输出 |

| std::vector< double > | get\_aos (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个模拟输出 |

| double | get\_ai (std::string device, unsigned int pin) |
|  | 获取模拟输入 |

| std::vector< double > | get\_ais (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个模拟输入 |

| void | set\_dio\_mode (std::string device, unsigned int pin, bool value) |
|  | 设置数字端口模式 |

| bool | get\_dio\_mode (std::string device, unsigned int pin) |
|  | 获取单个数字端口模式 |

| std::vector< bool > | get\_dios\_mode (std::string device, unsigned int pin, unsigned int count) |
|  | 获取数字端口模式 |

| void | enable\_button (std::string device, unsigned int pin) |
|  | 启用按钮输入. |

| void | disable\_button (std::string device, unsigned int pin) |
|  | 禁用按钮输入. |

| void | init\_claw (bool force) |
|  | 初始化夹爪 |

| void | set\_claw (double force, double amplitude) |
|  | 设置夹爪力度（力控）和幅度（位控）.如果在闭合过程中抓取到物体，则不再继续闭合以避免夹坏物体，判断的准则为这里设置的力的大小. |

| void | set\_claw\_ao (unsigned int address, double value) |
|  | 设置夹爪指定模拟输出数据. |

| ClawData | get\_claw () |
|  | 获取夹爪当前数据 |

| ClawData | **get\_claw\_data** () |

| double | get\_claw\_ai (unsigned int address) |
|  | 读取夹爪指定模拟输入数据. |

| void | wait\_claw\_ai (unsigned int address, double value, std::string relation="EQ") |
|  | 等待夹爪模拟输入满足指定关系. |

| void | set\_led (unsigned int mode, unsigned int speed, const std::vector< unsigned int > &color) |
|  | 设置LED灯状态. |

| void | set\_voice (unsigned int voice, unsigned int volume) |
|  | 设置声音 |

| void | set\_fan (unsigned int status) |
|  | 开关风扇 |

| void | set\_signal (unsigned int index, int value) |
|  | 设置信号量 |

| void | set\_signals (unsigned int index, std::vector< int > values) |
|  | 设置多个连续信号量 |

| int | get\_signal (unsigned int index) |
|  | 获取信号量 |

| std::vector< int > | get\_signals (unsigned int index, unsigned int len) |
|  | 获取多个连续信号量 |

| void | wait\_signal (unsigned int index, int value, std::string relation="EQ") |
|  | 等待信号量满足指定关系. |

| void | add\_signal (unsigned int index, int value) |
|  | 增加指定下标的信号量值，该操作是原子的. |

| unsigned int | start\_task (const std::string &name, const std::vector< std::string > &params, const std::string &dir, bool is\_parallel, unsigned int loop\_to) |
|  | 调用场景 |

| unsigned int | start\_task (const std::string &name) |
|  | 调用场景 |

| std::vector< unsigned int > | load\_task\_list () |
|  | 查询任务列表 |

| std::vector< unsigned int > | **get\_task\_list** () |

| std::vector< TaskData > | load\_running\_tasks () |
|  | 查询运行中的任务列表. |

| TaskStdoutData | get\_task\_stdout (unsigned int id) |
|  | 获取任务输出. |

| std::string | wait\_task (unsigned int id) |
|  | 等待任务完成 |

| void | pause\_task (unsigned int id, unsigned long time, bool wait) |
|  | 暂停任务与运动 |

| void | pause\_task (unsigned int id) |
|  | 暂停任务与运动 |

| void | resume\_task (unsigned int id) |
|  | 恢复任务与运动 |

| void | cancel\_task (unsigned int id) |
|  | 取消任务与运动. |

| unsigned int | exec\_hook (unsigned int id) |
|  | 根据已设置的Hook执行对应场景 |

| std::string | load\_task () |
|  | 获取任务状态. |

| std::string | **get\_task\_state** () |

| std::string | load\_task (unsigned int id) |
|  | 获取任务状态. |

| std::string | **get\_task\_state** (unsigned int id) |

| KinematicsForwardResp | get\_forward\_kin (const std::vector< double > &joint\_positions) |
|  | 根据机械臂关节位置计算机器人末端位姿（位置的运动学正解）. |

| KinematicsForwardResp | **kinematics\_forward** (const std::vector< double > &joint\_positions) |

| KinematicsInverseResp | get\_inverse\_kin (const CartesianPose &pose, const std::vector< double > &joint\_init\_positions={}) |
|  | 根据机械臂的末端位姿计算关节位置（位置的运动学逆解）. |

| KinematicsInverseResp | **kinematics\_inverse** (const CartesianPose &pose, const std::vector< double > &joint\_init\_positions={}) |

| double | measure\_manipulation (const std::vector< double > &joint\_positions) |
|  | 估算关节位置的灵活性. |

| CartesianPose | get\_pose\_trans (const CartesianPose &a, const CartesianPose &b) |
|  | 位姿变换乘法（等价于对应的齐次坐标矩阵乘法） |

| CartesianPose | **pose\_times** (const CartesianPose &a, const CartesianPose &b) |

| CartesianPose | get\_pose\_add (const CartesianPose &pose, const CartesianPose &delta) |
|  | 位姿加. |

| CartesianPose | calc\_frame (const CartesianPose &o, const CartesianPose &x, const CartesianPose &xy) |
|  | 三点采样计算原点特征. |

| CartesianPose | calc\_tcp (const std::vector< CartesianPose > &poses) |
|  | 三点采样计算工具中心点. |

| CartesianPose | get\_pose\_inverse (const CartesianPose &in) |
|  | 位姿变换的逆（等价于对应的齐次坐标矩的逆） |

| CartesianPose | **pose\_inverse** (const CartesianPose &in) |

| void | save\_file (const std::string &dir, const std::string &name, bool is\_dir, const std::string &data) |
|  | 保存文件（以字节形式）. |

| void | rename\_file (const std::string &from\_dir, const std::string &from\_name, const std::string &to\_dir, const std::string &to\_name) |
|  | 重命名文件 |

| void | download\_file (const std::string &dir, const std::string &name, const std::string &url) |
|  | 通过URL下载文件到控制器. |

| std::tuple< bool, std::string > | load\_file (const std::string &dir, const std::string &name) |
|  | 查询文件 |

| std::vector< std::tuple< bool, std::string > > | load\_file\_list (const std::string &dir, const std::string &prefix, const std::string &suffix) |
|  | 查询文件列表. |

| void | zip (const std::string &from\_dir, std::vector< std::string > files, const std::string &to\_dir, const std::string &name) |
|  | 将文件从文件系统中压缩到zip文件. |

| void | unzip (const std::string &from\_dir, const std::string &name, std::vector< std::string > files, const std::string &to\_dir) |
|  | 将zip文件解压到文件系统. |

| std::vector< std::tuple< bool, std::string > > | load\_zip\_list (const std::string &zip, const std::string &dir, const std::string &prefix, const std::string &suffix) |
|  | 查询zip内文件列表. |

| void | set\_tcp (std::array< double, 6 > tcp) |
|  | 设置工具中心点(TCP)坐标，坐标值相对于工具坐标系. |

| std::array< double, 6 > | get\_tcp () |
|  | 获取当前机器人工具中心点设置. |

| std::vector< DhParamData > | get\_dh () |
|  | 获取当前DH参数. |

| void | **set\_dh** (const std::vector< DhParamData > &params) |

| void | set\_kin\_factor (int factor) |
|  | 设置速度因子. |

| void | **set\_velocity\_factor** (int factor) |

| int | get\_kin\_factor () |
|  | 获取当前的速度因子. |

| int | **get\_velocity\_factor** () |

| void | set\_payload (double mass, std::map< std::string, double > cog) |
|  | 设置机器人末端负载. |

| void | set\_payload\_mass (double mass) |
|  | 设置机器人末端负载质量. |

| void | set\_payload\_cog (std::map< std::string, double > cog) |
|  | 设置机器人末端负载重心. |

| void | save\_payload (std::string name, std::map< std::string, double > payload, std::string dir="") |
|  | 保存末端负载. |

| std::map< std::string, double > | get\_payload () |
|  | 获取末端负载设置. |

| std::map< std::string, double > | load\_payload (std::string name, std::string dir="") |
|  | 从资源库加载末端负载. |

| std::vector< std::string > | load\_payload\_list (std::string dir="") |
|  | 查询负载列表. |

| void | set\_gravity (std::map< std::string, double > gravity) |
|  | 设置机器人重力加速度方向. |

| std::map< std::string, double > | get\_gravity () |
|  | 获取机器人重力加速度的方向. |

| void | enable\_collision\_detector () |
|  | 使能碰撞检测. |

| void | disable\_collision\_detector () |
|  | 临时禁用碰撞检测. |

| void | set\_collision\_torque\_diff (const std::vector< double > &diffs) |
|  | 设置碰撞监测误差. |

| std::vector< double > | get\_collision\_torque\_diff () |
|  | 获取碰撞监测误差. |

| void | set\_collision\_detector (const CollisionDetectorConfig &config) |
|  | 设置碰撞检测参数. |

| CollisionDetectorConfig | get\_collision\_detector () |
|  | 获取碰撞检测参数. |

| void | enable\_limit () |
|  | 使能安全限位检测. |

| void | disable\_limit () |
|  | 临时禁用安全限位检测. |

| void | set\_joints\_limit (const std::vector< JointLimitConfig > &joints) |
|  | 设置关节限位. |

| std::vector< JointLimitConfig > | get\_joints\_limit () |
|  | 获取关节限位. |

| void | set\_cart\_limit (const CartesianLimitConfig &limit) |
|  | 设置笛卡尔空间限位. |

| CartesianLimitConfig | get\_cart\_limit () |
|  | 获取笛卡尔空间限位. |

| CartesianPose | load\_tcp (std::string name, std::string dir="") |
|  | 从资源库加载tcp. |

| void | save\_tcp (std::string name, CartesianPose tcp, std::string dir="") |
|  | 保存tcp. |

| std::vector< std::string > | load\_tcp\_list (std::string dir="") |
|  | 查询工具中心点列表. |

| std::vector< std::string > | load\_trajectory\_list (std::string dir="") |
|  | 查询轨迹列表. |

| TrajectoryData | load\_trajectory (std::string name, std::string dir="") |
|  | 查询轨迹. |

| void | save\_trajectory (std::string name, TrajectoryData trajectory, std::string dir="") |
|  | 保存轨迹. |

| unsigned int | **move\_trajectory** (std::string name, std::string dir="") |

| void | start\_record\_trajectory (std::string kind, double duration) |
|  | 开始记录轨迹. |

| void | end\_record\_trajectory (std::string name, std::string dir="") |
|  | 结束记录轨迹并保存. |

| std::vector< std::string > | load\_pose\_list (std::string dir="") |
|  | 查询路点列表. |

| PoseData | load\_pose (std::string name, std::string dir="") |
|  | 查询路点. |

| void | save\_pose (std::string name, PoseData pose, std::string dir="") |
|  | 保存路点. |

| std::vector< std::string > | load\_frame\_list (std::string dir="") |
|  | 查询特征列表. |

| FrameData | load\_frame (std::string name, std::string dir="") |
|  | 查询特征. |

| void | save\_frame (std::string name, FrameData frame, std::string dir="") |
|  | 保存特征. |

| std::vector< std::string > | load\_structure\_list (std::string dir="") |
|  | 查询机器人结构列表. |

| StructureData | load\_structure (std::string name, std::string dir="") |
|  | 查询机器人结构. |

| void | save\_structure (std::string name, StructureData structure, std::string dir="") |
|  | 保存机器人结构. |

| std::vector< std::string > | load\_modbus\_list (std::string dir="") |
|  | 查询Modbus配置列表. |

| ModbusData | load\_modbus (std::string name, std::string dir="") |
|  | 查询Modbus设备. |

| void | save\_modbus (std::string name, ModbusData modbus) |
|  | 保存Modbus设备. |

| void | set\_modbus\_timeout (std::string device, unsigned int timeout) |
|  | 设置Modbus设备超时时间. |

| void | set\_modbus\_retry (std::string device, unsigned int retry) |
|  | 设置Modbus设备重试次数. |

| void | disconnect\_modbus (std::string device) |
|  | 断开Modbus设备连接. |

| std::vector< std::string > | load\_modbus\_register\_list (std::string device) |
|  | 查询Modbus寄存器列表. |

| ModbusRegisterData | load\_modbus\_register (std::string device, std::string name) |
|  | 查询Modbus寄存器. |

| void | save\_modbus\_register (std::string device, std::string name, ModbusRegisterData reg) |
|  | 保存Modbus寄存器. |

| void | write\_single\_coil (std::string device, std::string addr, bool value) |
|  | 写单个线圈. |

| void | write\_multiple\_coils (std::string device, std::string addr, std::vector< bool > values) |
|  | 写多个线圈 |

| void | **wirte\_multiple\_coils** (std::string device, std::string addr, std::vector< bool > values) |

| std::vector< bool > | read\_coils (std::string device, std::string addr, unsigned int num) |
|  | 读线圈 |

| std::vector< bool > | read\_discrete\_inputs (std::string device, std::string addr, unsigned int num) |
|  | 读离散输入 |

| void | write\_single\_register (std::string device, std::string addr, unsigned int value) |
|  | 写单个寄存器 |

| void | write\_multiple\_registers (std::string device, std::string addr, std::vector< unsigned int > values) |
|  | 写多个寄存器 |

| std::vector< unsigned int > | read\_holding\_registers (std::string device, std::string addr, unsigned int num) |
|  | 读保持寄存器 |

| std::vector< unsigned int > | read\_input\_registers (std::string device, std::string addr, unsigned int num) |
|  | 读输入寄存器 |

| void | set\_serial\_baud\_rate (std::string device, unsigned int baud\_rate) |
|  | 设置串口波特率. |

| void | set\_serial\_timeout (std::string device, unsigned int timeout) |
|  | 设置串口超时时间. |

| void | set\_serial\_parity (std::string device, unsigned int parity) |
|  | 设置串口校验位. |

| void | write\_serial (std::string device, std::vector< unsigned int > data) |
|  | 串口发送数据. |

| std::vector< unsigned int > | read\_serial (std::string device, unsigned int len) |
|  | 串口读取数据. |

| void | clear\_serial (std::string device) |
|  | 清除串口收发缓存. |

| void | set\_item (StorageItem item) |
|  | 设置存储项. |

| StorageItem | get\_item (std::string name) |
|  | 获取存储项. |

| std::vector< StorageItem > | get\_items (std::string prefix) |
|  | 获取存储项列表. |

| Protected Attributes | |
| std::unique\_ptr< RobotImpl > | impl\_ |

### Detailed Description

机械臂的主要接口对象，通过本对象的方法与机械臂进行数据交互.

Note
:   使用该数据结构和机械臂交互要求机械臂软件版本>=3.1.5。
:   接口的每一个函数都可能抛出异常std::runtime\_error, 这是因为网络连接丢失或者调用的逻辑错误等。

### Constructor & Destructor Documentation

### ◆Robot()

| |  |  |  |  | | --- | --- | --- | --- | | lebai::l\_master::Robot::Robot | ( | std::string | **ip**, | |  |  | bool | **simulator** = `false` | |  | ) |  |  | | explicit |

构造Robot对象.

Note
:   当你尝试创建Robot对象时，会根据传入的ip参数尝试去连接机械臂，但是可能会连接失败，当连接不成功时，对象依然会创建。

Parameters
:   |  |  |
    | --- | --- |
    | ip | 机械臂IP地址. |
    | simulator | 用于表示机械臂是否为仿真机械臂(docker仿真或控制器运行在仿真模式下)的macs标志.True表示仿真模式，False表示实物机械臂. |

### Member Function Documentation

### ◆call()

| std::tuple<int, std::string> lebai::l\_master::Robot::call | ( | const std::string & | **method**, |
|  |  | const std::string & | **params** |
|  | ) |  |  |

用JSON格式字符串调用机械臂的接口.

示例代码:

```
std::string movej_req = "{\"param\":{\"v\":0.1},\"pose\":{\"joint\":{\"delta\":"{\"joint\":[-1.0,0.0,0.0,0.0,0.0,0.0]}}}}";
resp = robot.call("movej", movej_req);
std::cout << "resp: " << std::get<0>(resp) << ", " << std::get<1>(resp) << std::endl;
```

Note
:   SDK采用JSONRPC协议和机械臂控制进行通讯. 您可以自行生成JSON数据，并通过本接口调用相应的JSONRPC，并获取返回的结果和JSON数据.

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | method | 方法名字 |
    | [in] | params | 请求JSONRPC的JSON格式字符串. |

Returns
:   返回一个元组，第一个元素是返回码，第二个元素是数据字符串. 如果返回码为0，表示调用成功，第二个元素是JSONRPC的返回数据. 如果返回码为非0，表示调用失败，第二个元素是错误信息.

### ◆is\_network\_connected()

| bool lebai::l\_master::Robot::is\_network\_connected | ( |  | ) |  |

返回是否和机械臂的网络连接正常，如果网络连接异常，调用和机械臂交互的接口会抛出异常std::runtime\_error。

Note
:   不建议使用，直接catch接口调用获取网络异常。

Returns
:   true 表示网络连接正常，false表示网络连接异常.

### Member Data Documentation

### ◆impl\_

| |  | | --- | | std::unique\_ptr<RobotImpl> lebai::l\_master::Robot::impl\_ | | protected |

内部实现数据结构，用户无需关注.

---

The documentation for this class was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::zeroconf::ControllerInfo Class Reference

Lebai机械臂控制器的信息数据结构.

`#include <discovery.hh>`

| Public Member Functions | |
| std::string | str () |
|  | 将控制器信息转换为字符串. |

| Public Attributes | |
| std::string | hostname |

| std::string | ip\_address |

| std::string | mac\_address |

| std::string | model |

| std::string | ds\_version |

| std::string | rc\_version |

| std::string | id |

### Detailed Description

Lebai机械臂控制器的信息数据结构.

### Member Function Documentation

### ◆str()

| std::string lebai::zeroconf::ControllerInfo::str | ( |  | ) |  |

将控制器信息转换为字符串.

Returns
:   字符串.

### Member Data Documentation

### ◆ds\_version

| std::string lebai::zeroconf::ControllerInfo::ds\_version |

ds软件版本.

### ◆hostname

| std::string lebai::zeroconf::ControllerInfo::hostname |

主机名字.

### ◆id

| std::string lebai::zeroconf::ControllerInfo::id |

机械臂产品ID.

### ◆ip\_address

| std::string lebai::zeroconf::ControllerInfo::ip\_address |

IP地址.

### ◆mac\_address

| std::string lebai::zeroconf::ControllerInfo::mac\_address |

MAC地址.

### ◆model

| std::string lebai::zeroconf::ControllerInfo::model |

机械臂模型.

### ◆rc\_version

| std::string lebai::zeroconf::ControllerInfo::rc\_version |

rc软件版本.

---

The documentation for this class was generated from the following file:

- sdk/include/lebai/discovery.hh


---

# lebai::zeroconf::Discovery Class Reference

自动发现局域网内lebai机械臂.

`#include <discovery.hh>`

| Public Member Functions | |
|  | Discovery () |
|  | 构造Discovery对象. |

| virtual | ~Discovery () |
|  | 析构Discovery对象. |

| std::vector< ControllerInfo > | resolve () |
|  | 获取局域网内lebai机械臂的信息数据,可以用来自动发现局域网内的所有机械臂控制器. |

| Protected Attributes | |
| std::unique\_ptr< DiscoveryImpl > | impl\_ |

### Detailed Description

自动发现局域网内lebai机械臂.

### Member Function Documentation

### ◆resolve()

| std::vector<ControllerInfo> lebai::zeroconf::Discovery::resolve | ( |  | ) |  |

获取局域网内lebai机械臂的信息数据,可以用来自动发现局域网内的所有机械臂控制器.

Returns
:   std::vector<std::string> 所有的机械臂的信息数据.

### Member Data Documentation

### ◆impl\_

| |  | | --- | | std::unique\_ptr<DiscoveryImpl> lebai::zeroconf::Discovery::impl\_ | | protected |

内部实现数据结构，用户无需关注.

---

The documentation for this class was generated from the following file:

- sdk/include/lebai/discovery.hh


---

# 夹爪.

夹爪相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::init\_claw (bool force) |
|  | 初始化夹爪 |

| void | lebai::l\_master::Robot::set\_claw (double force, double amplitude) |
|  | 设置夹爪力度（力控）和幅度（位控）.如果在闭合过程中抓取到物体，则不再继续闭合以避免夹坏物体，判断的准则为这里设置的力的大小. |

| void | lebai::l\_master::Robot::set\_claw\_ao (unsigned int address, double value) |
|  | 设置夹爪指定模拟输出数据. |

| ClawData | lebai::l\_master::Robot::get\_claw () |
|  | 获取夹爪当前数据 |

| ClawData | **lebai::l\_master::Robot::get\_claw\_data** () |

| double | lebai::l\_master::Robot::get\_claw\_ai (unsigned int address) |
|  | 读取夹爪指定模拟输入数据. |

| void | lebai::l\_master::Robot::wait\_claw\_ai (unsigned int address, double value, std::string relation="EQ") |
|  | 等待夹爪模拟输入满足指定关系. |

### Detailed Description

夹爪相关的接口.

### Function Documentation

### ◆get\_claw()

| ClawData lebai::l\_master::Robot::get\_claw | ( |  | ) |  |

获取夹爪当前数据

Returns
:   ClawData 数据

### ◆get\_claw\_ai()

| double lebai::l\_master::Robot::get\_claw\_ai | ( | unsigned int | **address** | ) |  |

读取夹爪指定模拟输入数据.

Parameters
:   |  |  |
    | --- | --- |
    | address | 夹爪地址. |

### ◆init\_claw()

| void lebai::l\_master::Robot::init\_claw | ( | bool | **force** | ) |  |

初始化夹爪

Parameters
:   |  |  |
    | --- | --- |
    | force | 是否强制初始化 |

### ◆set\_claw()

| void lebai::l\_master::Robot::set\_claw | ( | double | **force**, |
|  |  | double | **amplitude** |
|  | ) |  |  |

设置夹爪力度（力控）和幅度（位控）.如果在闭合过程中抓取到物体，则不再继续闭合以避免夹坏物体，判断的准则为这里设置的力的大小.

Parameters
:   |  |  |
    | --- | --- |
    | force | 力度（0-100） |
    | amplitude | 张合幅度（0-100） |

### ◆set\_claw\_ao()

| void lebai::l\_master::Robot::set\_claw\_ao | ( | unsigned int | **address**, |
|  |  | double | **value** |
|  | ) |  |  |

设置夹爪指定模拟输出数据.

Parameters
:   |  |  |
    | --- | --- |
    | address | 夹爪地址. |
    | value | 输出值. |

### ◆wait\_claw\_ai()

| void lebai::l\_master::Robot::wait\_claw\_ai | ( | unsigned int | **address**, |
|  |  | double | **value**, |
|  |  | std::string | **relation** = `"EQ"` |
|  | ) |  |  |

等待夹爪模拟输入满足指定关系.

Parameters
:   |  |  |
    | --- | --- |
    | address | 夹爪地址. |
    | value | 待比较的模拟输入值. |
    | relation | 比较关系，如 GTE. |


---

# 机器人配置.

机器人配置参数相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::set\_tcp (std::array< double, 6 > tcp) |
|  | 设置工具中心点(TCP)坐标，坐标值相对于工具坐标系. |

| std::array< double, 6 > | lebai::l\_master::Robot::get\_tcp () |
|  | 获取当前机器人工具中心点设置. |

| std::vector< DhParamData > | lebai::l\_master::Robot::get\_dh () |
|  | 获取当前DH参数. |

| void | **lebai::l\_master::Robot::set\_dh** (const std::vector< DhParamData > &params) |

| void | lebai::l\_master::Robot::set\_kin\_factor (int factor) |
|  | 设置速度因子. |

| void | **lebai::l\_master::Robot::set\_velocity\_factor** (int factor) |

| int | lebai::l\_master::Robot::get\_kin\_factor () |
|  | 获取当前的速度因子. |

| int | **lebai::l\_master::Robot::get\_velocity\_factor** () |

| void | lebai::l\_master::Robot::set\_payload (double mass, std::map< std::string, double > cog) |
|  | 设置机器人末端负载. |

| void | lebai::l\_master::Robot::set\_payload\_mass (double mass) |
|  | 设置机器人末端负载质量. |

| void | lebai::l\_master::Robot::set\_payload\_cog (std::map< std::string, double > cog) |
|  | 设置机器人末端负载重心. |

| void | lebai::l\_master::Robot::save\_payload (std::string name, std::map< std::string, double > payload, std::string dir="") |
|  | 保存末端负载. |

| std::map< std::string, double > | lebai::l\_master::Robot::get\_payload () |
|  | 获取末端负载设置. |

| std::map< std::string, double > | lebai::l\_master::Robot::load\_payload (std::string name, std::string dir="") |
|  | 从资源库加载末端负载. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_payload\_list (std::string dir="") |
|  | 查询负载列表. |

| void | lebai::l\_master::Robot::set\_gravity (std::map< std::string, double > gravity) |
|  | 设置机器人重力加速度方向. |

| std::map< std::string, double > | lebai::l\_master::Robot::get\_gravity () |
|  | 获取机器人重力加速度的方向. |

| void | lebai::l\_master::Robot::enable\_collision\_detector () |
|  | 使能碰撞检测. |

| void | lebai::l\_master::Robot::disable\_collision\_detector () |
|  | 临时禁用碰撞检测. |

| void | lebai::l\_master::Robot::set\_collision\_torque\_diff (const std::vector< double > &diffs) |
|  | 设置碰撞监测误差. |

| std::vector< double > | lebai::l\_master::Robot::get\_collision\_torque\_diff () |
|  | 获取碰撞监测误差. |

| void | lebai::l\_master::Robot::set\_collision\_detector (const CollisionDetectorConfig &config) |
|  | 设置碰撞检测参数. |

| CollisionDetectorConfig | lebai::l\_master::Robot::get\_collision\_detector () |
|  | 获取碰撞检测参数. |

| void | lebai::l\_master::Robot::enable\_limit () |
|  | 使能安全限位检测. |

| void | lebai::l\_master::Robot::disable\_limit () |
|  | 临时禁用安全限位检测. |

| void | lebai::l\_master::Robot::set\_joints\_limit (const std::vector< JointLimitConfig > &joints) |
|  | 设置关节限位. |

| std::vector< JointLimitConfig > | lebai::l\_master::Robot::get\_joints\_limit () |
|  | 获取关节限位. |

| void | lebai::l\_master::Robot::set\_cart\_limit (const CartesianLimitConfig &limit) |
|  | 设置笛卡尔空间限位. |

| CartesianLimitConfig | lebai::l\_master::Robot::get\_cart\_limit () |
|  | 获取笛卡尔空间限位. |

| CartesianPose | lebai::l\_master::Robot::load\_tcp (std::string name, std::string dir="") |
|  | 从资源库加载tcp. |

| void | lebai::l\_master::Robot::save\_tcp (std::string name, CartesianPose tcp, std::string dir="") |
|  | 保存tcp. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_tcp\_list (std::string dir="") |
|  | 查询工具中心点列表. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_trajectory\_list (std::string dir="") |
|  | 查询轨迹列表. |

| TrajectoryData | lebai::l\_master::Robot::load\_trajectory (std::string name, std::string dir="") |
|  | 查询轨迹. |

| void | lebai::l\_master::Robot::save\_trajectory (std::string name, TrajectoryData trajectory, std::string dir="") |
|  | 保存轨迹. |

| unsigned int | **lebai::l\_master::Robot::move\_trajectory** (std::string name, std::string dir="") |

| void | lebai::l\_master::Robot::start\_record\_trajectory (std::string kind, double duration) |
|  | 开始记录轨迹. |

| void | lebai::l\_master::Robot::end\_record\_trajectory (std::string name, std::string dir="") |
|  | 结束记录轨迹并保存. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_pose\_list (std::string dir="") |
|  | 查询路点列表. |

| PoseData | lebai::l\_master::Robot::load\_pose (std::string name, std::string dir="") |
|  | 查询路点. |

| void | lebai::l\_master::Robot::save\_pose (std::string name, PoseData pose, std::string dir="") |
|  | 保存路点. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_frame\_list (std::string dir="") |
|  | 查询特征列表. |

| FrameData | lebai::l\_master::Robot::load\_frame (std::string name, std::string dir="") |
|  | 查询特征. |

| void | lebai::l\_master::Robot::save\_frame (std::string name, FrameData frame, std::string dir="") |
|  | 保存特征. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_structure\_list (std::string dir="") |
|  | 查询机器人结构列表. |

| StructureData | lebai::l\_master::Robot::load\_structure (std::string name, std::string dir="") |
|  | 查询机器人结构. |

| void | lebai::l\_master::Robot::save\_structure (std::string name, StructureData structure, std::string dir="") |
|  | 保存机器人结构. |

### Detailed Description

机器人配置参数相关的接口.

### Function Documentation

### ◆end\_record\_trajectory()

| void lebai::l\_master::Robot::end\_record\_trajectory | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

结束记录轨迹并保存.

Parameters
:   |  |  |
    | --- | --- |
    | name | 轨迹名称. |
    | dir | 轨迹目录. |

### ◆get\_cart\_limit()

| CartesianLimitConfig lebai::l\_master::Robot::get\_cart\_limit | ( |  | ) |  |

获取笛卡尔空间限位.

Returns
:   笛卡尔空间限位配置.

### ◆get\_collision\_detector()

| CollisionDetectorConfig lebai::l\_master::Robot::get\_collision\_detector | ( |  | ) |  |

获取碰撞检测参数.

Returns
:   碰撞检测配置.

### ◆get\_collision\_torque\_diff()

| std::vector<double> lebai::l\_master::Robot::get\_collision\_torque\_diff | ( |  | ) |  |

获取碰撞监测误差.

Returns
:   各关节的碰撞监测误差.

### ◆get\_dh()

| std::vector<DhParamData> lebai::l\_master::Robot::get\_dh | ( |  | ) |  |

获取当前DH参数.

Returns
:   DH参数列表.

### ◆get\_gravity()

| std::map<std::string, double> lebai::l\_master::Robot::get\_gravity | ( |  | ) |  |

获取机器人重力加速度的方向.

Returns
:   相对于机器人基座标的重力方向.

### ◆get\_joints\_limit()

| std::vector<JointLimitConfig> lebai::l\_master::Robot::get\_joints\_limit | ( |  | ) |  |

获取关节限位.

Returns
:   各关节的限位配置.

### ◆get\_kin\_factor()

| int lebai::l\_master::Robot::get\_kin\_factor | ( |  | ) |  |

获取当前的速度因子.

Returns
:   速度因子百分比.

### ◆get\_payload()

| std::map<std::string, double> lebai::l\_master::Robot::get\_payload | ( |  | ) |  |

获取末端负载设置.

Returns
:   由负载质量mass和负载偏移组成的元组.

### ◆get\_tcp()

| std::array<double, 6> lebai::l\_master::Robot::get\_tcp | ( |  | ) |  |

获取当前机器人工具中心点设置.

Returns
:   当前机器人的工具中心点参数，为六元组.

### ◆load\_frame()

| FrameData lebai::l\_master::Robot::load\_frame | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询特征.

Parameters
:   |  |  |
    | --- | --- |
    | name | 特征名称. |
    | dir | 特征目录. |

### ◆load\_frame\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_frame\_list | ( | std::string | **dir** = `""` | ) |  |

查询特征列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 特征目录. |

### ◆load\_payload()

| std::map<std::string, double> lebai::l\_master::Robot::load\_payload | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

从资源库加载末端负载.

Parameters
:   |  |  |
    | --- | --- |
    | name | 负载名称. |
    | dir | 负载目录. |

### ◆load\_payload\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_payload\_list | ( | std::string | **dir** = `""` | ) |  |

查询负载列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 负载目录. |

### ◆load\_pose()

| PoseData lebai::l\_master::Robot::load\_pose | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询路点.

Parameters
:   |  |  |
    | --- | --- |
    | name | 路点名称. |
    | dir | 路点目录. |

### ◆load\_pose\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_pose\_list | ( | std::string | **dir** = `""` | ) |  |

查询路点列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 路点目录. |

### ◆load\_structure()

| StructureData lebai::l\_master::Robot::load\_structure | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询机器人结构.

Parameters
:   |  |  |
    | --- | --- |
    | name | 机器人结构名称. |
    | dir | 机器人结构目录. |

### ◆load\_structure\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_structure\_list | ( | std::string | **dir** = `""` | ) |  |

查询机器人结构列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 机器人结构目录. |

### ◆load\_tcp()

| CartesianPose lebai::l\_master::Robot::load\_tcp | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

从资源库加载tcp.

Parameters
:   |  |  |
    | --- | --- |
    | name | 点位名称. |
    | dir | 点位目录. |

### ◆load\_tcp\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_tcp\_list | ( | std::string | **dir** = `""` | ) |  |

查询工具中心点列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 工具中心点目录. |

### ◆load\_trajectory()

| TrajectoryData lebai::l\_master::Robot::load\_trajectory | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询轨迹.

Parameters
:   |  |  |
    | --- | --- |
    | name | 轨迹名称. |
    | dir | 轨迹目录. |

### ◆load\_trajectory\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_trajectory\_list | ( | std::string | **dir** = `""` | ) |  |

查询轨迹列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 轨迹目录. |

### ◆save\_frame()

| void lebai::l\_master::Robot::save\_frame | ( | std::string | **name**, |
|  |  | FrameData | **frame**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存特征.

Parameters
:   |  |  |
    | --- | --- |
    | name | 特征名称. |
    | frame | 特征数据. |
    | dir | 特征目录. |

### ◆save\_payload()

| void lebai::l\_master::Robot::save\_payload | ( | std::string | **name**, |
|  |  | std::map< std::string, double > | **payload**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存末端负载.

Parameters
:   |  |  |
    | --- | --- |
    | name | 负载名称. |
    | payload | 负载数据，包含mass,x,y,z. |
    | dir | 负载目录. |

### ◆save\_pose()

| void lebai::l\_master::Robot::save\_pose | ( | std::string | **name**, |
|  |  | PoseData | **pose**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存路点.

Parameters
:   |  |  |
    | --- | --- |
    | name | 路点名称. |
    | pose | 路点数据. |
    | dir | 路点目录. |

### ◆save\_structure()

| void lebai::l\_master::Robot::save\_structure | ( | std::string | **name**, |
|  |  | StructureData | **structure**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存机器人结构.

Parameters
:   |  |  |
    | --- | --- |
    | name | 机器人结构名称. |
    | structure | 机器人结构数据. |
    | dir | 机器人结构目录. |

### ◆save\_tcp()

| void lebai::l\_master::Robot::save\_tcp | ( | std::string | **name**, |
|  |  | CartesianPose | **tcp**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存tcp.

Parameters
:   |  |  |
    | --- | --- |
    | name | tcp名称. |
    | tcp | tcp数据，包含x,y,z,rx,ry,rz. |
    | dir | tcp目录. |

### ◆save\_trajectory()

| void lebai::l\_master::Robot::save\_trajectory | ( | std::string | **name**, |
|  |  | TrajectoryData | **trajectory**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存轨迹.

Parameters
:   |  |  |
    | --- | --- |
    | name | 轨迹名称. |
    | trajectory | 轨迹数据. |
    | dir | 轨迹目录. |

### ◆set\_cart\_limit()

| void lebai::l\_master::Robot::set\_cart\_limit | ( | const CartesianLimitConfig & | **limit** | ) |  |

设置笛卡尔空间限位.

Parameters
:   |  |  |
    | --- | --- |
    | limit | 笛卡尔空间限位配置. |

### ◆set\_collision\_detector()

| void lebai::l\_master::Robot::set\_collision\_detector | ( | const CollisionDetectorConfig & | **config** | ) |  |

设置碰撞检测参数.

Parameters
:   |  |  |
    | --- | --- |
    | config | 碰撞检测配置. |

### ◆set\_collision\_torque\_diff()

| void lebai::l\_master::Robot::set\_collision\_torque\_diff | ( | const std::vector< double > & | **diffs** | ) |  |

设置碰撞监测误差.

Parameters
:   |  |  |
    | --- | --- |
    | diffs | 各关节的碰撞监测误差. |

### ◆set\_gravity()

| void lebai::l\_master::Robot::set\_gravity | ( | std::map< std::string, double > | **gravity** | ) |  |

设置机器人重力加速度方向.

Parameters
:   |  |  |
    | --- | --- |
    | gravity | 相对于机器人基座标的重力方向. |

### ◆set\_joints\_limit()

| void lebai::l\_master::Robot::set\_joints\_limit | ( | const std::vector< JointLimitConfig > & | **joints** | ) |  |

设置关节限位.

Parameters
:   |  |  |
    | --- | --- |
    | joints | 各关节的限位配置. |

### ◆set\_kin\_factor()

| void lebai::l\_master::Robot::set\_kin\_factor | ( | int | **factor** | ) |  |

设置速度因子.

Parameters
:   |  |  |
    | --- | --- |
    | factor | 速度因子百分比，范围0-100. |

### ◆set\_payload()

| void lebai::l\_master::Robot::set\_payload | ( | double | **mass**, |
|  |  | std::map< std::string, double > | **cog** |
|  | ) |  |  |

设置机器人末端负载.

Parameters
:   |  |  |
    | --- | --- |
    | mass | 末端负载的质量(kg). |
    | cog | 质心相对于TCP坐标系的偏移. |

### ◆set\_payload\_cog()

| void lebai::l\_master::Robot::set\_payload\_cog | ( | std::map< std::string, double > | **cog** | ) |  |

设置机器人末端负载重心.

Parameters
:   |  |  |
    | --- | --- |
    | cog | 质心相对于TCP坐标系的偏移. |

### ◆set\_payload\_mass()

| void lebai::l\_master::Robot::set\_payload\_mass | ( | double | **mass** | ) |  |

设置机器人末端负载质量.

Parameters
:   |  |  |
    | --- | --- |
    | mass | 末端负载的质量(kg). |

### ◆set\_tcp()

| void lebai::l\_master::Robot::set\_tcp | ( | std::array< double, 6 > | **tcp** | ) |  |

设置工具中心点(TCP)坐标，坐标值相对于工具坐标系.

Parameters
:   |  |  |
    | --- | --- |
    | tcp | 参数为六元组，表示一个空间位置变换. |

### ◆start\_record\_trajectory()

| void lebai::l\_master::Robot::start\_record\_trajectory | ( | std::string | **kind**, |
|  |  | double | **duration** |
|  | ) |  |  |

开始记录轨迹.

Parameters
:   |  |  |
    | --- | --- |
    | kind | 轨迹类型. |
    | duration | 采样周期. |


---

# 文件系统.

文件系统相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::save\_file (const std::string &dir, const std::string &name, bool is\_dir, const std::string &data) |
|  | 保存文件（以字节形式）. |

| void | lebai::l\_master::Robot::rename\_file (const std::string &from\_dir, const std::string &from\_name, const std::string &to\_dir, const std::string &to\_name) |
|  | 重命名文件 |

| void | lebai::l\_master::Robot::download\_file (const std::string &dir, const std::string &name, const std::string &url) |
|  | 通过URL下载文件到控制器. |

| std::tuple< bool, std::string > | lebai::l\_master::Robot::load\_file (const std::string &dir, const std::string &name) |
|  | 查询文件 |

| std::vector< std::tuple< bool, std::string > > | lebai::l\_master::Robot::load\_file\_list (const std::string &dir, const std::string &prefix, const std::string &suffix) |
|  | 查询文件列表. |

| void | lebai::l\_master::Robot::zip (const std::string &from\_dir, std::vector< std::string > files, const std::string &to\_dir, const std::string &name) |
|  | 将文件从文件系统中压缩到zip文件. |

| void | lebai::l\_master::Robot::unzip (const std::string &from\_dir, const std::string &name, std::vector< std::string > files, const std::string &to\_dir) |
|  | 将zip文件解压到文件系统. |

| std::vector< std::tuple< bool, std::string > > | lebai::l\_master::Robot::load\_zip\_list (const std::string &zip, const std::string &dir, const std::string &prefix, const std::string &suffix) |
|  | 查询zip内文件列表. |

### Detailed Description

文件系统相关的接口.

### Function Documentation

### ◆download\_file()

| void lebai::l\_master::Robot::download\_file | ( | const std::string & | **dir**, |
|  |  | const std::string & | **name**, |
|  |  | const std::string & | **url** |
|  | ) |  |  |

通过URL下载文件到控制器.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 保存的文件路径. |
    | name | 保存的文件名. |
    | url | 下载URL. |

### ◆load\_file()

| std::tuple<bool, std::string> lebai::l\_master::Robot::load\_file | ( | const std::string & | **dir**, |
|  |  | const std::string & | **name** |
|  | ) |  |  |

查询文件

Parameters
:   |  |  |
    | --- | --- |
    | dir | 文件的目录 |
    | name | 文件名 |

Returns
:   文件的具体内容

### ◆load\_file\_list()

| std::vector<std::tuple<bool, std::string> > lebai::l\_master::Robot::load\_file\_list | ( | const std::string & | **dir**, |
|  |  | const std::string & | **prefix**, |
|  |  | const std::string & | **suffix** |
|  | ) |  |  |

查询文件列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 文件的目录. |
    | prefix | 前缀. |
    | suffix | 后缀. |

Returns
:   文件列表.

### ◆rename\_file()

| void lebai::l\_master::Robot::rename\_file | ( | const std::string & | **from\_dir**, |
|  |  | const std::string & | **from\_name**, |
|  |  | const std::string & | **to\_dir**, |
|  |  | const std::string & | **to\_name** |
|  | ) |  |  |

重命名文件

Parameters
:   |  |  |
    | --- | --- |
    | from\_dir | 源文件所在的文件夹. |
    | from\_name | 源文件名称. |
    | to\_dir | 目标文件文件夹. |
    | to\_name | 目标文件文件名. |

### ◆save\_file()

| void lebai::l\_master::Robot::save\_file | ( | const std::string & | **dir**, |
|  |  | const std::string & | **name**, |
|  |  | bool | **is\_dir**, |
|  |  | const std::string & | **data** |
|  | ) |  |  |

保存文件（以字节形式）.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 保存的文件路径. |
    | name | 保存的文件名. |
    | is\_dir | 要保存的文件是否为文件夹. |
    | data | 文件字节. |

### ◆unzip()

| void lebai::l\_master::Robot::unzip | ( | const std::string & | **from\_dir**, |
|  |  | const std::string & | **name**, |
|  |  | std::vector< std::string > | **files**, |
|  |  | const std::string & | **to\_dir** |
|  | ) |  |  |

将zip文件解压到文件系统.

Parameters
:   |  |  |
    | --- | --- |
    | from\_dir | zip文件的路径. |
    | name | zip文件的名称. |
    | files | zip文件内的文件名. |
    | to\_dir | 解压到的路径. |

### ◆zip()

| void lebai::l\_master::Robot::zip | ( | const std::string & | **from\_dir**, |
|  |  | std::vector< std::string > | **files**, |
|  |  | const std::string & | **to\_dir**, |
|  |  | const std::string & | **name** |
|  | ) |  |  |

将文件从文件系统中压缩到zip文件.

Parameters
:   |  |  |
    | --- | --- |
    | from\_dir | 源文件的目录. |
    | files | 源文件的文件名. |
    | to\_dir | 压缩后文件的路径. |
    | name | 压缩后文件的名称. |


---

# 通用输入输出

通用输入输出相关的接口

| Functions | |
| void | lebai::l\_master::Robot::set\_do (std::string device, unsigned int pin, unsigned int value) |
|  | 设置数字输出  |

| void | lebai::l\_master::Robot::set\_dos (std::string device, unsigned int pin, std::vector< unsigned int > values) |
|  | 设置多个数字输出 |

| unsigned int | lebai::l\_master::Robot::get\_do (std::string device, unsigned int pin) |
|  | 获取数字输出 |

| std::vector< unsigned int > | lebai::l\_master::Robot::get\_dos (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个数字输出 |

| unsigned int | lebai::l\_master::Robot::get\_di (std::string device, unsigned int pin) |
|  | 获取数字输入 |

| std::vector< unsigned int > | lebai::l\_master::Robot::get\_dis (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个数字输入 |

| void | lebai::l\_master::Robot::set\_ao (std::string device, unsigned int pin, double value) |
|  | 设置模拟输出 |

| void | lebai::l\_master::Robot::set\_aos (std::string device, unsigned int pin, std::vector< double > values) |
|  | 设置多个模拟输出 |

| double | lebai::l\_master::Robot::get\_ao (std::string device, unsigned int pin) |
|  | 获取模拟输出 |

| std::vector< double > | lebai::l\_master::Robot::get\_aos (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个模拟输出 |

| double | lebai::l\_master::Robot::get\_ai (std::string device, unsigned int pin) |
|  | 获取模拟输入 |

| std::vector< double > | lebai::l\_master::Robot::get\_ais (std::string device, unsigned int pin, unsigned int num) |
|  | 获取多个模拟输入 |

| void | lebai::l\_master::Robot::set\_dio\_mode (std::string device, unsigned int pin, bool value) |
|  | 设置数字端口模式 |

| bool | lebai::l\_master::Robot::get\_dio\_mode (std::string device, unsigned int pin) |
|  | 获取单个数字端口模式 |

| std::vector< bool > | lebai::l\_master::Robot::get\_dios\_mode (std::string device, unsigned int pin, unsigned int count) |
|  | 获取数字端口模式 |

| void | lebai::l\_master::Robot::enable\_button (std::string device, unsigned int pin) |
|  | 启用按钮输入. |

| void | lebai::l\_master::Robot::disable\_button (std::string device, unsigned int pin) |
|  | 禁用按钮输入. |

### Detailed Description

通用输入输出相关的接口

### Function Documentation

### ◆disable\_button()

| void lebai::l\_master::Robot::disable\_button | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

禁用按钮输入.

Parameters
:   |  |  |
    | --- | --- |
    | device | 按钮设备名称. |
    | pin | 按钮编号. |

### ◆enable\_button()

| void lebai::l\_master::Robot::enable\_button | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

启用按钮输入.

Parameters
:   |  |  |
    | --- | --- |
    | device | 按钮设备名称. |
    | pin | 按钮编号. |

### ◆get\_ai()

| double lebai::l\_master::Robot::get\_ai | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

获取模拟输入

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口，从 0 开始 |

Returns
:   返回模拟输入数值

### ◆get\_ais()

| std::vector<double> lebai::l\_master::Robot::get\_ais | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

获取多个模拟输入

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 起始模拟输入端口，从 0 开始 |
    | num | 连续的模拟输入个数 |

Returns
:   返回多个模拟输入数值

### ◆get\_ao()

| double lebai::l\_master::Robot::get\_ao | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

获取模拟输出

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口，从 0 开始 |

Returns
:   返回模拟输入数值

### ◆get\_aos()

| std::vector<double> lebai::l\_master::Robot::get\_aos | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

获取多个模拟输出

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 起始模拟输出端口，从 0 开始 |
    | num | 连续的模拟输出个数 |

Returns
:   返回模拟输出数值

### ◆get\_di()

| unsigned int lebai::l\_master::Robot::get\_di | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

获取数字输入

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口，从 0 开始 |

Returns
:   返回输入数值

### ◆get\_dio\_mode()

| bool lebai::l\_master::Robot::get\_dio\_mode | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

获取单个数字端口模式

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME. |
    | pin | 端口号，从 0 开始 |

Returns
:   false为输入模式，true为输出模式

### ◆get\_dios\_mode()

| std::vector<bool> lebai::l\_master::Robot::get\_dios\_mode | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **count** |
|  | ) |  |  |

获取数字端口模式

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口号，从 0 开始 |
    | count | 查询的连续端口数 |

Returns
:   从pin开始的连续count个端口的当前模式

### ◆get\_dis()

| std::vector<unsigned int> lebai::l\_master::Robot::get\_dis | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

获取多个数字输入

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 起始数字输入端口，从 0 开始 |
    | num | 连续的数字输入个数 |

Returns
:   返回多个数字输入

### ◆get\_do()

| unsigned int lebai::l\_master::Robot::get\_do | ( | std::string | **device**, |
|  |  | unsigned int | **pin** |
|  | ) |  |  |

获取数字输出

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口，从 0 开始 |

Returns
:   返回数字输出数值

### ◆get\_dos()

| std::vector<unsigned int> lebai::l\_master::Robot::get\_dos | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

获取多个数字输出

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 起始数字输出端口，从 0 开始 |
    | num | 连续的数字输出个数 |

Returns
:   返回多个数字输出数值

### ◆set\_ao()

| void lebai::l\_master::Robot::set\_ao | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | double | **value** |
|  | ) |  |  |

设置模拟输出

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 模拟输出端口，从 0 开始 |
    | value | 待设置的模拟输出值 |

### ◆set\_dio\_mode()

| void lebai::l\_master::Robot::set\_dio\_mode | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | bool | **value** |
|  | ) |  |  |

设置数字端口模式

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名字，查看 DEVICENAME ，详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B> |
    | pin | 端口号，从 0 开始 |
    | value | 设置的值，false为输入模式，true为输出模式 |

Returns
:   返回是否成功

### ◆set\_do()

| void lebai::l\_master::Robot::set\_do | ( | std::string | **device**, |
|  |  | unsigned int | **pin**, |
|  |  | unsigned int | **value** |
|  | ) |  |  |

设置数字输出

Parameters
:   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
    | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
    | device | 设备名字，  | ID | 设备名字 | 说明 | | --- | --- | --- | | 0 | ROBOT | 机箱IO | | 1 | FLANGE | 法兰IO | | 2 | EXTRA | 拓展IO | | 11 | SHOULDER | 肩部按钮DI | | 12 | FLANGE\_BTN | 法兰按钮DI | |

详细信息: <https://help.lebai.ltd/sdk/io.html#io-%E8%AE%BE%E5%A4%87%E7%B1%BB%E5%9E%8B>

Parameters
:   |  |  |
    | --- | --- |
    | pin | 端口，从 0 开始 |
    | value | 待设置的值 |


---

# 灯板.

灯板相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::set\_led (unsigned int mode, unsigned int speed, const std::vector< unsigned int > &color) |
|  | 设置LED灯状态. |

| void | lebai::l\_master::Robot::set\_voice (unsigned int voice, unsigned int volume) |
|  | 设置声音 |

| void | lebai::l\_master::Robot::set\_fan (unsigned int status) |
|  | 开关风扇 |

### Detailed Description

灯板相关的接口.

### Function Documentation

### ◆set\_fan()

| void lebai::l\_master::Robot::set\_fan | ( | unsigned int | **status** | ) |  |

开关风扇

Parameters
:   |  |  |
    | --- | --- |
    | status | 状态.1：关闭；2：开启 |

### ◆set\_led()

| void lebai::l\_master::Robot::set\_led | ( | unsigned int | **mode**, |
|  |  | unsigned int | **speed**, |
|  |  | const std::vector< unsigned int > & | **color** |
|  | ) |  |  |

设置LED灯状态.

Parameters
:   |  |  |
    | --- | --- |
    | mode | 亮灯模式.0：不变；1：关闭；2：常亮；3：呼吸；4：均分旋转；5：同色旋转；6：闪烁 |
    | speed | 速度.1：快速；2：正常；3：慢速 |
    | color | 最多包含 4 个 0 ~ 15 之间的整数 |

### ◆set\_voice()

| void lebai::l\_master::Robot::set\_voice | ( | unsigned int | **voice**, |
|  |  | unsigned int | **volume** |
|  | ) |  |  |

设置声音

Parameters
:   |  |  |
    | --- | --- |
    | voice | 声音列表 |
    | volume | 音量.0：静音；1：低；2：正常；3：高 |


---

# modbus.

modbus相关的接口.

| Functions | |
| std::vector< std::string > | lebai::l\_master::Robot::load\_modbus\_list (std::string dir="") |
|  | 查询Modbus配置列表. |

| ModbusData | lebai::l\_master::Robot::load\_modbus (std::string name, std::string dir="") |
|  | 查询Modbus设备. |

| void | lebai::l\_master::Robot::save\_modbus (std::string name, ModbusData modbus) |
|  | 保存Modbus设备. |

| void | lebai::l\_master::Robot::set\_modbus\_timeout (std::string device, unsigned int timeout) |
|  | 设置Modbus设备超时时间. |

| void | lebai::l\_master::Robot::set\_modbus\_retry (std::string device, unsigned int retry) |
|  | 设置Modbus设备重试次数. |

| void | lebai::l\_master::Robot::disconnect\_modbus (std::string device) |
|  | 断开Modbus设备连接. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_modbus\_register\_list (std::string device) |
|  | 查询Modbus寄存器列表. |

| ModbusRegisterData | lebai::l\_master::Robot::load\_modbus\_register (std::string device, std::string name) |
|  | 查询Modbus寄存器. |

| void | lebai::l\_master::Robot::save\_modbus\_register (std::string device, std::string name, ModbusRegisterData reg) |
|  | 保存Modbus寄存器. |

| void | lebai::l\_master::Robot::write\_single\_coil (std::string device, std::string addr, bool value) |
|  | 写单个线圈. |

| void | lebai::l\_master::Robot::write\_multiple\_coils (std::string device, std::string addr, std::vector< bool > values) |
|  | 写多个线圈 |

| void | **lebai::l\_master::Robot::wirte\_multiple\_coils** (std::string device, std::string addr, std::vector< bool > values) |

| std::vector< bool > | lebai::l\_master::Robot::read\_coils (std::string device, std::string addr, unsigned int num) |
|  | 读线圈 |

| std::vector< bool > | lebai::l\_master::Robot::read\_discrete\_inputs (std::string device, std::string addr, unsigned int num) |
|  | 读离散输入 |

| void | lebai::l\_master::Robot::write\_single\_register (std::string device, std::string addr, unsigned int value) |
|  | 写单个寄存器 |

| void | lebai::l\_master::Robot::write\_multiple\_registers (std::string device, std::string addr, std::vector< unsigned int > values) |
|  | 写多个寄存器 |

| std::vector< unsigned int > | lebai::l\_master::Robot::read\_holding\_registers (std::string device, std::string addr, unsigned int num) |
|  | 读保持寄存器 |

| std::vector< unsigned int > | lebai::l\_master::Robot::read\_input\_registers (std::string device, std::string addr, unsigned int num) |
|  | 读输入寄存器 |

### Detailed Description

modbus相关的接口.

### Function Documentation

### ◆disconnect\_modbus()

| void lebai::l\_master::Robot::disconnect\_modbus | ( | std::string | **device** | ) |  |

断开Modbus设备连接.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |

### ◆load\_modbus()

| ModbusData lebai::l\_master::Robot::load\_modbus | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询Modbus设备.

Parameters
:   |  |  |
    | --- | --- |
    | name | Modbus设备名称. |
    | dir | Modbus配置目录. |

### ◆load\_modbus\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_modbus\_list | ( | std::string | **dir** = `""` | ) |  |

查询Modbus配置列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | Modbus配置目录. |

### ◆load\_modbus\_register()

| ModbusRegisterData lebai::l\_master::Robot::load\_modbus\_register | ( | std::string | **device**, |
|  |  | std::string | **name** |
|  | ) |  |  |

查询Modbus寄存器.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |
    | name | 寄存器名称. |

### ◆load\_modbus\_register\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_modbus\_register\_list | ( | std::string | **device** | ) |  |

查询Modbus寄存器列表.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |

### ◆read\_coils()

| std::vector<bool> lebai::l\_master::Robot::read\_coils | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

读线圈

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | num | 连续数量. |

### ◆read\_discrete\_inputs()

| std::vector<bool> lebai::l\_master::Robot::read\_discrete\_inputs | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

读离散输入

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | num | 连续数量. |

### ◆read\_holding\_registers()

| std::vector<unsigned int> lebai::l\_master::Robot::read\_holding\_registers | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

读保持寄存器

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称 |
    | addr | 寄存器地址 |
    | num | 连续数量 |

### ◆read\_input\_registers()

| std::vector<unsigned int> lebai::l\_master::Robot::read\_input\_registers | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | unsigned int | **num** |
|  | ) |  |  |

读输入寄存器

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称 |
    | addr | 寄存器地址 |
    | num | 连续数量 |

### ◆save\_modbus()

| void lebai::l\_master::Robot::save\_modbus | ( | std::string | **name**, |
|  |  | ModbusData | **modbus** |
|  | ) |  |  |

保存Modbus设备.

Parameters
:   |  |  |
    | --- | --- |
    | name | Modbus设备名称. |
    | modbus | Modbus设备数据. |

### ◆save\_modbus\_register()

| void lebai::l\_master::Robot::save\_modbus\_register | ( | std::string | **device**, |
|  |  | std::string | **name**, |
|  |  | ModbusRegisterData | **reg** |
|  | ) |  |  |

保存Modbus寄存器.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |
    | name | 寄存器名称. |
    | reg | Modbus寄存器数据. |

### ◆set\_modbus\_retry()

| void lebai::l\_master::Robot::set\_modbus\_retry | ( | std::string | **device**, |
|  |  | unsigned int | **retry** |
|  | ) |  |  |

设置Modbus设备重试次数.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |
    | retry | 重试次数. |

### ◆set\_modbus\_timeout()

| void lebai::l\_master::Robot::set\_modbus\_timeout | ( | std::string | **device**, |
|  |  | unsigned int | **timeout** |
|  | ) |  |  |

设置Modbus设备超时时间.

Parameters
:   |  |  |
    | --- | --- |
    | device | Modbus设备名称. |
    | timeout | 超时时间，单位毫秒. |

### ◆write\_multiple\_coils()

| void lebai::l\_master::Robot::write\_multiple\_coils | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | std::vector< bool > | **values** |
|  | ) |  |  |

写多个线圈

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | values | 待设置的值. |

### ◆write\_multiple\_registers()

| void lebai::l\_master::Robot::write\_multiple\_registers | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | std::vector< unsigned int > | **values** |
|  | ) |  |  |

写多个寄存器

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | values | 待设置的值. |

### ◆write\_single\_coil()

| void lebai::l\_master::Robot::write\_single\_coil | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | bool | **value** |
|  | ) |  |  |

写单个线圈.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | value | 待设置的值. |

### ◆write\_single\_register()

| void lebai::l\_master::Robot::write\_single\_register | ( | std::string | **device**, |
|  |  | std::string | **addr**, |
|  |  | unsigned int | **value** |
|  | ) |  |  |

写单个寄存器

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | addr | 寄存器地址. |
    | value | 待设置的值. |


---

# 机械臂运动.

机械臂运动相关的接口.

| Functions | |
| int | lebai::l\_master::Robot::move\_joint (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂关节移动 |

| int | **lebai::l\_master::Robot::movej** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::move\_joint (const CartesianPose &cart\_pose, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂关节移动 |

| int | **lebai::l\_master::Robot::movej** (const CartesianPose &cart\_pose, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::move\_linear (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂直线移动 |

| int | **lebai::l\_master::Robot::movel** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::move\_linear (const CartesianPose &cart\_pose, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂直线移动 |

| int | **lebai::l\_master::Robot::movel** (const CartesianPose &cart\_pose, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::move\_circular (const std::vector< double > &joint\_via, const std::vector< double > &joint, double rad, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂圆弧运动. |

| int | **lebai::l\_master::Robot::movec** (const std::vector< double > &joint\_via, const std::vector< double > &joint, double rad, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::move\_circular (const CartesianPose &cart\_via, const CartesianPose &cart, double rad, double a, double v, double t, double r) |
|  | 通过坐标位置发送机械臂圆弧运动 |

| int | **lebai::l\_master::Robot::movec** (const CartesianPose &cart\_via, const CartesianPose &cart, double rad, double a, double v, double t, double r) |

| int | lebai::l\_master::Robot::speed\_joint (double a, const std::vector< double > &v, double t=0.0) |
|  | 通过关节速度矢量发送机械臂关节匀速运动 |

| int | **lebai::l\_master::Robot::speedj** (double a, const std::vector< double > &v, double t=0.0) |

| int | lebai::l\_master::Robot::speed\_linear (double a, const CartesianPose &v, double t=0.0, const CartesianPose &reference={{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}}) |
|  | 通过坐标速度矢量发送机械臂关节匀速运动 |

| int | **lebai::l\_master::Robot::speedl** (double a, const CartesianPose &v, double t=0.0, const CartesianPose &reference={{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}}) |

| int | lebai::l\_master::Robot::toward\_joint (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |
|  | 通过关节位置发送机械臂关节自由移动. |

| int | **lebai::l\_master::Robot::towardj** (const std::vector< double > &joint\_positions, double a, double v, double t, double r) |

| void | lebai::l\_master::Robot::move\_pvat (std::vector< double > p, std::vector< double > v, std::vector< double > a, double t) |
|  | 伺服运动PVAT |

| void | lebai::l\_master::Robot::wait\_move (unsigned int id) |
|  | 等待运动完成. |

| void | lebai::l\_master::Robot::wait\_move () |
|  | 等待所有运动完成. |

| unsigned int | lebai::l\_master::Robot::get\_running\_motion () |
|  | 查询当前正在运动的MotionId(无运动时返回上次MotionId). |

| std::string | lebai::l\_master::Robot::get\_motion\_state (unsigned int id) |
|  | 查询指定MotionId的运动状态. |

| void | lebai::l\_master::Robot::stop\_move () |
|  | 停止所有运动. |

| void | lebai::l\_master::Robot::skip\_move () |
|  | 跳过当前运动. |

### Detailed Description

机械臂运动相关的接口.

### Function Documentation

### ◆get\_motion\_state()

| std::string lebai::l\_master::Robot::get\_motion\_state | ( | unsigned int | **id** | ) |  |

查询指定MotionId的运动状态.

Returns
:   id 指定的运动id.

### ◆move\_circular() [1/2]

| int lebai::l\_master::Robot::move\_circular | ( | const CartesianPose & | **cart\_via**, |
|  |  | const CartesianPose & | **cart**, |
|  |  | double | **rad**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过坐标位置发送机械臂圆弧运动

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | cart\_via | 圆弧上途径点坐标位置，应当包括键为x,y,z,rz,ry,rx的值.为圆上三点中的一点. |
    | [in] | cart | 圆弧目标点坐标位置，应当包括键为x,y,z,rz,ry,rx的值.如果编程rad不为零，则为圆上三点中的一点. |
    | [in] | rad | 圆弧角度值，单位为弧度，如果为零，则走到目标点，正负值用来确定圆弧方向. |
    | [in] | a | 加速度. |
    | [in] | v | 速度. |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功.
:   <=0 发送失败.

### ◆move\_circular() [2/2]

| int lebai::l\_master::Robot::move\_circular | ( | const std::vector< double > & | **joint\_via**, |
|  |  | const std::vector< double > & | **joint**, |
|  |  | double | **rad**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过关节位置发送机械臂圆弧运动.

示例代码

```
robot.movec({3.0/ 180.0 * M_PI, -48.0/ 180.0 * M_PI, 78.0/ 180.0 *
```

M\_PI, 9.0/ 180.0 \* M\_PI, -67.0/ 180.0 \* M\_PI, -3.0/ 180.0 \* M\_PI}, {-28/ 180.0 \* M\_PI, -59.0/ 180.0 \* M\_PI, 96.0/ 180.0 \* M\_PI, -2.0/ 180.0 \* M\_PI, -92.0/ 180.0 \* M\_PI, 16.0/ 180.0 \* M\_PI}, 0.0, 1.0, 0.5, 0.0, 0.0);

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | joint\_via | 圆弧上途径点关节位置，为关节的角度值构成的数组.为圆上三点中的一点. |
    | [in] | joint | 圆弧目标点关节位置，为关节的角度值构成的数组.如果编程rad不为零，则为圆上三点中的一点. |
    | [in] | rad | 圆弧角度值，单位为弧度，如果为零，则走到目标点，正负值用来确定圆弧方向. |
    | [in] | a | 加速度. |
    | [in] | v | 速度. |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功.
:   <=0 发送失败.

### ◆move\_joint() [1/2]

| int lebai::l\_master::Robot::move\_joint | ( | const CartesianPose & | **cart\_pose**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过坐标位置发送机械臂关节移动

示例代码:

```
robot.movej({”x",-0.296},{"y",-0.295},{"z",0.285},{"rz",60.0 / 180.0 *
```

M\_PI},{"ry",-5.0 / 180.0 \* M\_PI},{"rx", 81.0 / 180.0 \* M\_PI}}, 3.0, 1.0, 0.0, 0.0);

Note
:   该接口为异步接口，仅向控制器内部的运动缓冲区写入一个关节移动即返回，不会等待运动结束.

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | cart\_pose | 目标位置在机器人基座标系下的坐标数据(目前不支持在其它坐标系下的坐标数据)，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |
    | [in] | a | 加速度. |
    | [in] | v | 速度. |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功
:   <=0 发送失败

### ◆move\_joint() [2/2]

| int lebai::l\_master::Robot::move\_joint | ( | const std::vector< double > & | **joint\_positions**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过关节位置发送机械臂关节移动

示例代码:

```
std::vector<double> joint_positions = {0.0, -60.0 / 180.0 * M_PI, 80.0
```

/ 180.0 \* M\_PI, -10.0 / 180.0 \* M\_PI, -60.0 / 180.0 \* M\_PI, 0.0}; robot.movej(joint\_positions, 3.0, 1.0, 0.0, 0.0);

Note
:   该接口为异步接口，仅向控制器内部的运动缓冲区写入一个关节移动即返回，不会等待运动结束.

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | joint\_positions | 目标位置的关节数据,为关节的角度值构成的数组. |
    | [in] | a | 加速度 |
    | [in] | v | 速度 |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功
:   <=0 发送失败

### ◆move\_linear() [1/2]

| int lebai::l\_master::Robot::move\_linear | ( | const CartesianPose & | **cart\_pose**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过坐标位置发送机械臂直线移动

示例代码:

```
robot.movel({”x",-0.296},{"y",-0.295},{"z",0.285},{"rz",60.0 / 180.0 *
```

M\_PI},{"ry",-5.0 / 180.0 \* M\_PI},{"rx", 81.0 / 180.0 \* M\_PI}}, 3.0, 1.0, 0.0, 0.0);

Note
:   该接口为异步接口，仅向控制器内部的运动缓冲区写入一个关节移动即返回，不会等待运动结束.

Parameters
:   |  |  |
    | --- | --- |
    | cart\_pose | 目标位置在机器人基座标系下的坐标数据(目前不支持在其它坐标系下的坐标数据)，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |
    | a | 加速度. |
    | v | 速度. |
    | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功.
:   <=0 发送失败.

### ◆move\_linear() [2/2]

| int lebai::l\_master::Robot::move\_linear | ( | const std::vector< double > & | **joint\_positions**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过关节位置发送机械臂直线移动

示例代码:

```
std::vector<double> joint_positions = {0.0, -60.0 / 180.0 * M_PI, 80.0
```

/ 180.0 \* M\_PI, -10.0 / 180.0 \* M\_PI, -60.0 / 180.0 \* M\_PI, 0.0}; robot.movel(joint\_positions, 3.0, 1.0, 0.0, 0.0);

Note
:   该接口为异步接口，仅向控制器内部的运动缓冲区写入一个关节移动即返回，不会等待运动结束.

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | joint\_positions | 目标位置的关节数据,为关节的角度值构成的数组. |
    | [in] | a | 加速度. |
    | [in] | v | 速度. |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功
:   <=0 发送失败

### ◆move\_pvat()

| void lebai::l\_master::Robot::move\_pvat | ( | std::vector< double > | **p**, |
|  |  | std::vector< double > | **v**, |
|  |  | std::vector< double > | **a**, |
|  |  | double | **t** |
|  | ) |  |  |

伺服运动PVAT

Parameters
:   |  |  |
    | --- | --- |
    | p | 关节位置，或者坐标位置（将通过运动学反解转为关节位置）. |
    | v | 每个关节的速度 (rad/s)。如该值为数字，则表示所有关节速度相同. |
    | a | 每个关节的加速度 (rad/s2)。如该值为数字，则表示所有关节加速度相同. |
    | t | 运动时间 (s) |

### ◆speed\_joint()

| int lebai::l\_master::Robot::speed\_joint | ( | double | **a**, |
|  |  | const std::vector< double > & | **v**, |
|  |  | double | **t** = `0.0` |
|  | ) |  |  |

通过关节速度矢量发送机械臂关节匀速运动

示例代码:

```
robot.speedj( 1.0, {0.5,0.0,0.0,0.0,0.0,0.0}, 0.0);
```

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | a | 加速度. |
    | [in] | v | 速度矢量 |
    | [in] | t | 运动时间，默认t = 0，一直运动到限位. |

Returns
:   >0 发送成功.返回运动号
:   <=0 发送失败.

### ◆speed\_linear()

| int lebai::l\_master::Robot::speed\_linear | ( | double | **a**, |
|  |  | const CartesianPose & | **v**, |
|  |  | double | **t** = `0.0`, |
|  |  | const CartesianPose & | **reference** = `{{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}}` |
|  | ) |  |  |

通过坐标速度矢量发送机械臂关节匀速运动

示例代码:

```
robot.speedl( 1.0, {{"x", 0.0}, {"y", 0.0}, {"z", 0.1}, {"rx", 0.0},
```

{"ry", 0.0}, {"rz", 0.0}}, 0.0);

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | a | 加速度. |
    | [in] | v | 速度矢量 |
    | [in] | t | 运动时间，默认t = 0，一直运动到限位. |
    | [in] | reference | 参考坐标系，默认为零. |

Returns
:   >0 发送成功.返回运动号
:   <=0 发送失败.

### ◆toward\_joint()

| int lebai::l\_master::Robot::toward\_joint | ( | const std::vector< double > & | **joint\_positions**, |
|  |  | double | **a**, |
|  |  | double | **v**, |
|  |  | double | **t**, |
|  |  | double | **r** |
|  | ) |  |  |

通过关节位置发送机械臂关节自由移动.

示例代码:

```
std::vector<double> joint_positions = {0.0, -60.0 / 180.0 * M_PI, 80.0
```

/ 180.0 \* M\_PI, -10.0 / 180.0 \* M\_PI, -60.0 / 180.0 \* M\_PI, 0.0}; robot.toawrdj(joint\_positions, 3.0, 1.0, 0.0, 0.0);

Note
:   该接口为异步接口，仅向控制器内部的运动缓冲区写入一个关节自由移动即返回，不会等待运动结束.

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | joint\_positions | 目标位置的关节数据,为关节的角度值构成的数组. |
    | [in] | a | 加速度. |
    | [in] | v | 速度. |
    | [in] | t | 时间参数，如果设置时间不为零，则按照时间计算出速度，而不使用速度参数. |
    | [in] | r | 交融半径，设置为0，则无交融半径. |

Returns
:   >0 发送成功.返回运动号
:   <=0 发送失败.

### ◆wait\_move()

| void lebai::l\_master::Robot::wait\_move | ( | unsigned int | **id** | ) |  |

等待运动完成.

Parameters
:   |  |  |
    | --- | --- |
    | id | 指定运动的id(0为等待全部任务). |


---

# 机器人.

机器人计算相关的接口.

| Functions | |
| KinematicsForwardResp | lebai::l\_master::Robot::get\_forward\_kin (const std::vector< double > &joint\_positions) |
|  | 根据机械臂关节位置计算机器人末端位姿（位置的运动学正解）. |

| KinematicsForwardResp | **lebai::l\_master::Robot::kinematics\_forward** (const std::vector< double > &joint\_positions) |

| KinematicsInverseResp | lebai::l\_master::Robot::get\_inverse\_kin (const CartesianPose &pose, const std::vector< double > &joint\_init\_positions={}) |
|  | 根据机械臂的末端位姿计算关节位置（位置的运动学逆解）. |

| KinematicsInverseResp | **lebai::l\_master::Robot::kinematics\_inverse** (const CartesianPose &pose, const std::vector< double > &joint\_init\_positions={}) |

| double | lebai::l\_master::Robot::measure\_manipulation (const std::vector< double > &joint\_positions) |
|  | 估算关节位置的灵活性. |

| CartesianPose | lebai::l\_master::Robot::get\_pose\_trans (const CartesianPose &a, const CartesianPose &b) |
|  | 位姿变换乘法（等价于对应的齐次坐标矩阵乘法） |

| CartesianPose | **lebai::l\_master::Robot::pose\_times** (const CartesianPose &a, const CartesianPose &b) |

| CartesianPose | lebai::l\_master::Robot::get\_pose\_add (const CartesianPose &pose, const CartesianPose &delta) |
|  | 位姿加. |

| CartesianPose | lebai::l\_master::Robot::calc\_frame (const CartesianPose &o, const CartesianPose &x, const CartesianPose &xy) |
|  | 三点采样计算原点特征. |

| CartesianPose | lebai::l\_master::Robot::calc\_tcp (const std::vector< CartesianPose > &poses) |
|  | 三点采样计算工具中心点. |

| CartesianPose | lebai::l\_master::Robot::get\_pose\_inverse (const CartesianPose &in) |
|  | 位姿变换的逆（等价于对应的齐次坐标矩的逆） |

| CartesianPose | **lebai::l\_master::Robot::pose\_inverse** (const CartesianPose &in) |

### Detailed Description

机器人计算相关的接口.

### Function Documentation

### ◆calc\_frame()

| CartesianPose lebai::l\_master::Robot::calc\_frame | ( | const CartesianPose & | **o**, |
|  |  | const CartesianPose & | **x**, |
|  |  | const CartesianPose & | **xy** |
|  | ) |  |  |

三点采样计算原点特征.

Parameters
:   |  |  |
    | --- | --- |
    | o | 原点. |
    | x | X轴点. |
    | xy | XY平面点. |

### ◆calc\_tcp()

| CartesianPose lebai::l\_master::Robot::calc\_tcp | ( | const std::vector< CartesianPose > & | **poses** | ) |  |

三点采样计算工具中心点.

Parameters
:   |  |  |
    | --- | --- |
    | poses | 采样位姿列表，至少3个. |

### ◆get\_forward\_kin()

| KinematicsForwardResp lebai::l\_master::Robot::get\_forward\_kin | ( | const std::vector< double > & | **joint\_positions** | ) |  |

根据机械臂关节位置计算机器人末端位姿（位置的运动学正解）.

Parameters
:   |  |  |
    | --- | --- |
    | joint\_positions | 机械臂关节位置的数组. |

Returns
:   返回计算结果 KinematicsForwardResp.

### ◆get\_inverse\_kin()

| KinematicsInverseResp lebai::l\_master::Robot::get\_inverse\_kin | ( | const CartesianPose & | **pose**, |
|  |  | const std::vector< double > & | **joint\_init\_positions** = `{}` |
|  | ) |  |  |

根据机械臂的末端位姿计算关节位置（位置的运动学逆解）.

Parameters
:   |  |  |
    | --- | --- |
    | pose | 机械臂末端位姿，应当包括键为x,y,z,rz,ry,rx的值. |
    | joint\_init\_positions | 机械臂关节初始位置, 以数组形式传入. |

Returns
:   返回计算结果 KinematicsInverseResp.

### ◆get\_pose\_add()

| CartesianPose lebai::l\_master::Robot::get\_pose\_add | ( | const CartesianPose & | **pose**, |
|  |  | const CartesianPose & | **delta** |
|  | ) |  |  |

位姿加.

Parameters
:   |  |  |
    | --- | --- |
    | pose | 基准位姿. |
    | delta | 增量位姿. |

### ◆get\_pose\_inverse()

| CartesianPose lebai::l\_master::Robot::get\_pose\_inverse | ( | const CartesianPose & | **in** | ) |  |

位姿变换的逆（等价于对应的齐次坐标矩的逆）

Parameters
:   |  |  |
    | --- | --- |
    | in | 位姿，应当包括键为x,y,z,rz,ry,rx的值. |

Returns
:   CartesianPose 返回位姿变换的逆，应当包括键为x,y,z,rz,ry,rx的值.

### ◆get\_pose\_trans()

| CartesianPose lebai::l\_master::Robot::get\_pose\_trans | ( | const CartesianPose & | **a**, |
|  |  | const CartesianPose & | **b** |
|  | ) |  |  |

位姿变换乘法（等价于对应的齐次坐标矩阵乘法）

Parameters
:   |  |  |  |
    | --- | --- | --- |
    | [in] | a | 位姿，应当包括键为x,y,z,rz,ry,rx的值. |
    | [in] | b | 位姿，应当包括键为x,y,z,rz,ry,rx的值. |

Returns
:   CartesianPose 返回的位姿，应当包括键为x,y,z,rz,ry,rx的值.

### ◆measure\_manipulation()

| double lebai::l\_master::Robot::measure\_manipulation | ( | const std::vector< double > & | **joint\_positions** | ) |  |

估算关节位置的灵活性.

Parameters
:   |  |  |
    | --- | --- |
    | joint\_positions | 机械臂关节位置数组. |

Returns
:   灵活性估算值.


---

# 场景.

场景相关的接口.

| Functions | |
| unsigned int | lebai::l\_master::Robot::start\_task (const std::string &name, const std::vector< std::string > &params, const std::string &dir, bool is\_parallel, unsigned int loop\_to) |
|  | 调用场景 |

| unsigned int | lebai::l\_master::Robot::start\_task (const std::string &name) |
|  | 调用场景 |

| std::vector< unsigned int > | lebai::l\_master::Robot::load\_task\_list () |
|  | 查询任务列表 |

| std::vector< unsigned int > | **lebai::l\_master::Robot::get\_task\_list** () |

| std::vector< TaskData > | lebai::l\_master::Robot::load\_running\_tasks () |
|  | 查询运行中的任务列表. |

| TaskStdoutData | lebai::l\_master::Robot::get\_task\_stdout (unsigned int id) |
|  | 获取任务输出. |

| std::string | lebai::l\_master::Robot::wait\_task (unsigned int id) |
|  | 等待任务完成 |

| void | lebai::l\_master::Robot::pause\_task (unsigned int id, unsigned long time, bool wait) |
|  | 暂停任务与运动 |

| void | lebai::l\_master::Robot::pause\_task (unsigned int id) |
|  | 暂停任务与运动 |

| void | lebai::l\_master::Robot::resume\_task (unsigned int id) |
|  | 恢复任务与运动 |

| void | lebai::l\_master::Robot::cancel\_task (unsigned int id) |
|  | 取消任务与运动. |

| unsigned int | lebai::l\_master::Robot::exec\_hook (unsigned int id) |
|  | 根据已设置的Hook执行对应场景 |

| std::string | lebai::l\_master::Robot::load\_task () |
|  | 获取任务状态. |

| std::string | **lebai::l\_master::Robot::get\_task\_state** () |

| std::string | lebai::l\_master::Robot::load\_task (unsigned int id) |
|  | 获取任务状态. |

| std::string | **lebai::l\_master::Robot::get\_task\_state** (unsigned int id) |

### Detailed Description

场景相关的接口.

### Function Documentation

### ◆cancel\_task()

| void lebai::l\_master::Robot::cancel\_task | ( | unsigned int | **id** | ) |  |

取消任务与运动.

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID. |

### ◆exec\_hook()

| unsigned int lebai::l\_master::Robot::exec\_hook | ( | unsigned int | **id** | ) |  |

根据已设置的Hook执行对应场景

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID. |

### ◆get\_task\_stdout()

| TaskStdoutData lebai::l\_master::Robot::get\_task\_stdout | ( | unsigned int | **id** | ) |  |

获取任务输出.

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务ID. |

### ◆load\_task()

| std::string lebai::l\_master::Robot::load\_task | ( | unsigned int | **id** | ) |  |

获取任务状态.

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID. |

### ◆pause\_task() [1/2]

| void lebai::l\_master::Robot::pause\_task | ( | unsigned int | **id** | ) |  |

暂停任务与运动

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID |

### ◆pause\_task() [2/2]

| void lebai::l\_master::Robot::pause\_task | ( | unsigned int | **id**, |
|  |  | unsigned long | **time**, |
|  |  | bool | **wait** |
|  | ) |  |  |

暂停任务与运动

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID |
    | time | 暂停的时间 |
    | wait | 是否等待 |

### ◆resume\_task()

| void lebai::l\_master::Robot::resume\_task | ( | unsigned int | **id** | ) |  |

恢复任务与运动

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID |

### ◆start\_task() [1/2]

| unsigned int lebai::l\_master::Robot::start\_task | ( | const std::string & | **name** | ) |  |

调用场景

Parameters
:   |  |  |
    | --- | --- |
    | name | 调用场景的名字 |

### ◆start\_task() [2/2]

| unsigned int lebai::l\_master::Robot::start\_task | ( | const std::string & | **name**, |
|  |  | const std::vector< std::string > & | **params**, |
|  |  | const std::string & | **dir**, |
|  |  | bool | **is\_parallel**, |
|  |  | unsigned int | **loop\_to** |
|  | ) |  |  |

调用场景

Parameters
:   |  |  |
    | --- | --- |
    | name | 调用场景的名字 |
    | params | 其他参数 |
    | dir | 调用场景所在的文件夹名 |
    | is\_parallel | 是否并行 |
    | loop\_to | 循环次数（默认0永久循环） |

### ◆wait\_task()

| std::string lebai::l\_master::Robot::wait\_task | ( | unsigned int | **id** | ) |  |

等待任务完成

Parameters
:   |  |  |
    | --- | --- |
    | id | 任务的ID |

Returns
:   返回任务是否成功


---

# serial.

串口相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::set\_serial\_baud\_rate (std::string device, unsigned int baud\_rate) |
|  | 设置串口波特率. |

| void | lebai::l\_master::Robot::set\_serial\_timeout (std::string device, unsigned int timeout) |
|  | 设置串口超时时间. |

| void | lebai::l\_master::Robot::set\_serial\_parity (std::string device, unsigned int parity) |
|  | 设置串口校验位. |

| void | lebai::l\_master::Robot::write\_serial (std::string device, std::vector< unsigned int > data) |
|  | 串口发送数据. |

| std::vector< unsigned int > | lebai::l\_master::Robot::read\_serial (std::string device, unsigned int len) |
|  | 串口读取数据. |

| void | lebai::l\_master::Robot::clear\_serial (std::string device) |
|  | 清除串口收发缓存. |

### Detailed Description

串口相关的接口.

### Function Documentation

### ◆clear\_serial()

| void lebai::l\_master::Robot::clear\_serial | ( | std::string | **device** | ) |  |

清除串口收发缓存.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |

### ◆read\_serial()

| std::vector<unsigned int> lebai::l\_master::Robot::read\_serial | ( | std::string | **device**, |
|  |  | unsigned int | **len** |
|  | ) |  |  |

串口读取数据.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | len | 单次接收的最大缓冲长度. |

### ◆set\_serial\_baud\_rate()

| void lebai::l\_master::Robot::set\_serial\_baud\_rate | ( | std::string | **device**, |
|  |  | unsigned int | **baud\_rate** |
|  | ) |  |  |

设置串口波特率.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | baud\_rate | 波特率. |

### ◆set\_serial\_parity()

| void lebai::l\_master::Robot::set\_serial\_parity | ( | std::string | **device**, |
|  |  | unsigned int | **parity** |
|  | ) |  |  |

设置串口校验位.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | parity | 校验位. 0:无校验; 1:奇校验; 2:偶校验. |

### ◆set\_serial\_timeout()

| void lebai::l\_master::Robot::set\_serial\_timeout | ( | std::string | **device**, |
|  |  | unsigned int | **timeout** |
|  | ) |  |  |

设置串口超时时间.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | timeout | 超时时间，单位毫秒. |

### ◆write\_serial()

| void lebai::l\_master::Robot::write\_serial | ( | std::string | **device**, |
|  |  | std::vector< unsigned int > | **data** |
|  | ) |  |  |

串口发送数据.

Parameters
:   |  |  |
    | --- | --- |
    | device | 设备名称. |
    | data | u8数组. |


---

# 信号量.

信号量相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::set\_signal (unsigned int index, int value) |
|  | 设置信号量 |

| void | lebai::l\_master::Robot::set\_signals (unsigned int index, std::vector< int > values) |
|  | 设置多个连续信号量 |

| int | lebai::l\_master::Robot::get\_signal (unsigned int index) |
|  | 获取信号量 |

| std::vector< int > | lebai::l\_master::Robot::get\_signals (unsigned int index, unsigned int len) |
|  | 获取多个连续信号量 |

| void | lebai::l\_master::Robot::wait\_signal (unsigned int index, int value, std::string relation="EQ") |
|  | 等待信号量满足指定关系. |

| void | lebai::l\_master::Robot::add\_signal (unsigned int index, int value) |
|  | 增加指定下标的信号量值，该操作是原子的. |

### Detailed Description

信号量相关的接口.

### Function Documentation

### ◆add\_signal()

| void lebai::l\_master::Robot::add\_signal | ( | unsigned int | **index**, |
|  |  | int | **value** |
|  | ) |  |  |

增加指定下标的信号量值，该操作是原子的.

Parameters
:   |  |  |
    | --- | --- |
    | index | 信号量下标（取值范围0~255） |
    | value | 待增加的信号量值 |

### ◆get\_signal()

| int lebai::l\_master::Robot::get\_signal | ( | unsigned int | **index** | ) |  |

获取信号量

Parameters
:   |  |  |
    | --- | --- |
    | index | 信号量下标（取值范围0~255) |

Returns
:   返回对应的信号量

### ◆set\_signal()

| void lebai::l\_master::Robot::set\_signal | ( | unsigned int | **index**, |
|  |  | int | **value** |
|  | ) |  |  |

设置信号量

Parameters
:   |  |  |
    | --- | --- |
    | index | 信号量下标（取值范围0~255) |
    | value | 待设置的信号量(32位有符号整数) |

### ◆wait\_signal()

| void lebai::l\_master::Robot::wait\_signal | ( | unsigned int | **index**, |
|  |  | int | **value**, |
|  |  | std::string | **relation** = `"EQ"` |
|  | ) |  |  |

等待信号量满足指定关系.

Parameters
:   |  |  |
    | --- | --- |
    | index | 信号量下标（取值范围0~255）. |
    | value | 待比较的信号量值. |
    | relation | 比较关系，如 EQ. |


---

# 启动停止.

启动停止相关的接口.

| Functions | |
| void | lebai::l\_master::Robot::start\_sys () |
|  | 启动机械臂（机械臂上使能）. |

| void | lebai::l\_master::Robot::stop\_sys () |
|  | 停止机械臂（机械臂下使能）. |

| void | lebai::l\_master::Robot::powerdown () |
|  | 关闭机器人电源（关机）. |

| void | lebai::l\_master::Robot::stop () |
|  | 停止运动（但不下电）. |

| void | lebai::l\_master::Robot::estop () |
|  | 紧急停止（急停）. |

| void | lebai::l\_master::Robot::start\_teach\_mode () |
|  | 进入示教模式. |

| void | **lebai::l\_master::Robot::teach\_mode** () |

| void | lebai::l\_master::Robot::end\_teach\_mode () |
|  | 退出示教模式. |

| void | lebai::l\_master::Robot::pause\_move () |
|  | 暂停运动. |

| void | **lebai::l\_master::Robot::pause** () |

| void | lebai::l\_master::Robot::resume\_move () |
|  | 恢复运动. |

| void | **lebai::l\_master::Robot::resume** () |

| void | lebai::l\_master::Robot::reboot () |
|  | 重新启动机箱 |

### Detailed Description

启动停止相关的接口.


---

# 机械臂状态.

机械臂状态相关的接口.

| Functions | |
| int | lebai::l\_master::Robot::get\_robot\_state () |
|  | 获取机器人状态码 |

| int | lebai::l\_master::Robot::get\_estop\_reason () |
|  | 查看急停原因 |

| SystemInfoData | lebai::l\_master::Robot::get\_system\_info () |
|  | 获取控制器系统信息 |

| RobotInfoData | lebai::l\_master::Robot::get\_robot\_info () |
|  | 获取机器人设备信息 |

| HardwareInfoData | lebai::l\_master::Robot::get\_hardware\_info () |
|  | 获取控制器硬件信息 |

| SoftwareInfoData | lebai::l\_master::Robot::get\_software\_info () |
|  | 获取控制器软件信息 |

| HttpResponseData | lebai::l\_master::Robot::http (HttpRequestData request) |
|  | 通过控制器发起HTTP请求. |

| void | **lebai::l\_master::Robot::clean** (const BackupOptionsData &option) |

| void | **lebai::l\_master::Robot::backup** (const std::string &file, const BackupOptionsData &option) |

| BackupInfoData | **lebai::l\_master::Robot::get\_backup\_info** (const std::string &file) |

| void | **lebai::l\_master::Robot::restore** (const std::string &file, const BackupOptionsData &option) |

| void | **lebai::l\_master::Robot::set\_virtual\_ip** (const std::string &ifname, const std::string &ip) |

| std::vector< std::string > | lebai::l\_master::Robot::get\_box\_devices (const std::string &prefix) |
|  | 获取控制箱dev设备列表 |

| std::vector< DirData > | lebai::l\_master::Robot::get\_dirs () |
|  | 获取数据库目录列表 |

| void | lebai::l\_master::Robot::create\_dir (DirData dir) |
|  | 创建数据库目录. |

| void | lebai::l\_master::Robot::update\_dir (std::string from, std::string to) |
|  | 重命名数据库目录. |

| std::vector< ShortcutData > | lebai::l\_master::Robot::get\_short\_poses () |
|  | 获取所有快捷路点 |

| void | lebai::l\_master::Robot::set\_short\_pose (ShortcutData shortcut\_data) |
|  | 设置快捷路点 |

| ShortcutData | lebai::l\_master::Robot::get\_short\_pose (unsigned int id) |
|  | 获取单个快捷路点 |

| std::vector< ShortcutData > | lebai::l\_master::Robot::get\_short\_tasks () |
|  | 获取所有快捷任务 |

| void | lebai::l\_master::Robot::set\_short\_task (ShortcutData shortcut\_data) |
|  | 设置快捷任务 |

| ShortcutData | lebai::l\_master::Robot::get\_short\_task (unsigned int id) |
|  | 获取单个快捷任务 |

| std::vector< TriggerData > | lebai::l\_master::Robot::get\_triggers () |
|  | 获取条件任务触发器列表 |

| void | lebai::l\_master::Robot::set\_trigger (TriggerData trigger) |
|  | 设置条件任务触发器 |

| std::map< std::string, LedStyleData > | lebai::l\_master::Robot::get\_led\_styles () |
|  | 获取声光交互样式集 |

| void | lebai::l\_master::Robot::set\_led\_styles (std::map< std::string, LedStyleData > styles) |
|  | 设置声光交互样式集 |

| LedStyleData | lebai::l\_master::Robot::load\_led\_style (std::string name, std::string dir="") |
|  | 查询声光样式. |

| void | lebai::l\_master::Robot::save\_led\_style (std::string name, LedStyleData style, std::string dir="") |
|  | 保存声光样式. |

| void | lebai::l\_master::Robot::set\_led\_style (std::string state, LedStyleData style) |
|  | 设置机器人状态对应的声光交互样式. |

| std::vector< std::string > | lebai::l\_master::Robot::load\_led\_style\_list (std::string dir="") |
|  | 查询声光样式列表. |

| std::vector< ServoParamData > | lebai::l\_master::Robot::get\_servo\_params () |
|  | 获取伺服参数 |

| void | **lebai::l\_master::Robot::set\_servo\_params** (const std::vector< ServoParamData > &params) |

| void | **lebai::l\_master::Robot::find\_zero** () |

| void | **lebai::l\_master::Robot::set\_zero** (const std::vector< double > &pose, const std::vector< bool > &valids) |

| void | **lebai::l\_master::Robot::set\_extra\_servo\_params** (const std::vector< ExtraServoParamData > &params, const std::vector< bool > &valids) |

| void | **lebai::l\_master::Robot::reset\_extra\_servo\_params** (const std::vector< bool > &valids) |

| void | **lebai::l\_master::Robot::set\_flange\_baud\_rate** (unsigned int baud\_rate) |

| WrenchData | lebai::l\_master::Robot::get\_tcp\_force () |
|  | 获取末端受力 |

| void | lebai::l\_master::Robot::set\_tcp\_force (const WrenchData &wrench) |
|  | 设置末端受力 |

| void | lebai::l\_master::Robot::set\_force\_mode\_sensor (const std::string &sensor, unsigned int address) |
|  | 设置力控传感器. |

| void | lebai::l\_master::Robot::set\_force\_mode\_param (double damping, double mass, double force\_threshold, double torque\_threshold) |
|  | 设置力控参数. |

| void | lebai::l\_master::Robot::start\_force\_mode (const CartesianPose &limit, const WrenchData &wrench) |
|  | 开始力控模式. |

| void | lebai::l\_master::Robot::end\_force\_mode () |
|  | 结束力控模式. |

| std::vector< PluginInfoData > | lebai::l\_master::Robot::load\_plugins () |
|  | 查询已安装插件列表 |

| std::vector< PluginStoreInfoData > | lebai::l\_master::Robot::get\_plugin\_store () |
|  | 查询插件商店列表. |

| PluginInfoData | lebai::l\_master::Robot::load\_plugin (const std::string &name) |
|  | 查询已安装插件 |

| CommandStdoutData | lebai::l\_master::Robot::run\_plugin\_cmd (const std::string &name, const std::vector< std::string > &params=std::vector< std::string >()) |
|  | 执行插件命令. |

| CommandStdoutData | lebai::l\_master::Robot::enable\_plugin (const std::string &name) |
|  | 启用插件. |

| CommandStdoutData | lebai::l\_master::Robot::disable\_plugin (const std::string &name) |
|  | 禁用插件. |

| void | lebai::l\_master::Robot::restart\_plugin\_daemon (const std::string &name) |
|  | 重启插件守护进程. |

| std::vector< DiscoveredRobotData > | lebai::l\_master::Robot::discover\_robots () |
|  | 通过控制器RPC发现机器人设备. |

| CommandStdoutData | lebai::l\_master::Robot::get\_plugin\_daemon\_stdout (const std::string &name) |
|  | 获取插件守护进程输出. |

| std::vector< MessageData > | lebai::l\_master::Robot::get\_messages () |
|  | 获取控制器消息列表 |

| OtaStateData | lebai::l\_master::Robot::get\_ota\_state () |
|  | 获取OTA升级状态 |

| CheckUpgradeData | lebai::l\_master::Robot::check\_upgrade () |
|  | 查询是否需要系统升级 |

| CommandStdoutData | lebai::l\_master::Robot::get\_upgrade\_stdout () |
|  | 获取系统升级输出 |

| void | **lebai::l\_master::Robot::sub\_buttons\_status** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::sub\_kin\_data** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::sub\_message** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::sub\_robot\_state** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::sub\_phy\_data** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::sub\_task\_stdout** (uint64\_t interval\_min, uint64\_t interval\_max) |

| void | **lebai::l\_master::Robot::start\_ota** (const std::string &address, const std::string &partition, const std::string &file) |

| void | **lebai::l\_master::Robot::switch\_partition** (const std::string &address, const std::string &partition) |

| void | **lebai::l\_master::Robot::start\_upgrade** () |

| int | lebai::l\_master::Robot::box\_test (const std::string &time, const std::string &auth) |
|  | Run the quality-platform box test. |

| std::string | **lebai::l\_master::Robot::init\_robot** (const std::string &time, const std::string &auth, const RobotInfoData &info) |

| PhysicalData | lebai::l\_master::Robot::get\_phy\_data () |
|  | 获取机械臂物理数据 |

| JointMotionData | lebai::l\_master::Robot::get\_kin\_data () |
|  | 获取机械臂关节运动数据 |

| bool | lebai::l\_master::Robot::is\_disconnected () |
|  | 是否已与手臂断开连接 |

| bool | lebai::l\_master::Robot::is\_down () |
|  | 手臂是否已下电 |

| std::vector< double > | lebai::l\_master::Robot::get\_actual\_joint\_positions () |
|  | 获取机械臂关节当前反馈位置 |

| std::vector< double > | lebai::l\_master::Robot::get\_target\_joint\_positions () |
|  | 获取机械臂关节当前控制位置 |

| std::vector< double > | lebai::l\_master::Robot::get\_actual\_joint\_speed () |
|  | 获取机械臂关节当前反馈速度 |

| std::vector< double > | lebai::l\_master::Robot::get\_target\_joint\_speed () |
|  | 获取机械臂关节当前控制力矩 |

| CartesianPose | lebai::l\_master::Robot::get\_actual\_tcp\_pose () |
|  | 获取机械臂末端在机械臂基坐标系下的实际位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |

| CartesianPose | lebai::l\_master::Robot::get\_target\_tcp\_pose () |
|  | 获取机械臂末端在机械臂基坐标系下的控制位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值. |

| double | lebai::l\_master::Robot::get\_joint\_temp (unsigned int joint\_index) |
|  | 获取单个关节温度 |

| std::vector< double > | lebai::l\_master::Robot::get\_actual\_joint\_torques () |
|  | 获取机械臂关节当前反馈力矩 |

| std::vector< double > | lebai::l\_master::Robot::get\_target\_joint\_torques () |
|  | 获取机械臂关节当前控制力矩 |

### Detailed Description

机械臂状态相关的接口.

### Function Documentation

### ◆box\_test()

| int lebai::l\_master::Robot::box\_test | ( | const std::string & | **time**, |
|  |  | const std::string & | **auth** |
|  | ) |  |  |

Run the quality-platform box test.

Parameters
:   |  |  |
    | --- | --- |
    | time | Quality-platform authentication time. |
    | auth | Quality-platform authentication value. |

Returns
:   Test status returned by the quality platform.

### ◆check\_upgrade()

| CheckUpgradeData lebai::l\_master::Robot::check\_upgrade | ( |  | ) |  |

查询是否需要系统升级

Returns
:   升级检查结果

### ◆create\_dir()

| void lebai::l\_master::Robot::create\_dir | ( | DirData | **dir** | ) |  |

创建数据库目录.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 目录数据. |

### ◆disable\_plugin()

| CommandStdoutData lebai::l\_master::Robot::disable\_plugin | ( | const std::string & | **name** | ) |  |

禁用插件.

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |

### ◆discover\_robots()

| std::vector<DiscoveredRobotData> lebai::l\_master::Robot::discover\_robots | ( |  | ) |  |

通过控制器RPC发现机器人设备.

Returns
:   设备信息列表.

### ◆enable\_plugin()

| CommandStdoutData lebai::l\_master::Robot::enable\_plugin | ( | const std::string & | **name** | ) |  |

启用插件.

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |

### ◆get\_actual\_joint\_positions()

| std::vector<double> lebai::l\_master::Robot::get\_actual\_joint\_positions | ( |  | ) |  |

获取机械臂关节当前反馈位置

Returns
:   关节反馈位置数组，包括所有关节的角度值.

### ◆get\_actual\_joint\_speed()

| std::vector<double> lebai::l\_master::Robot::get\_actual\_joint\_speed | ( |  | ) |  |

获取机械臂关节当前反馈速度

Returns
:   关节反馈速度数据

### ◆get\_actual\_joint\_torques()

| std::vector<double> lebai::l\_master::Robot::get\_actual\_joint\_torques | ( |  | ) |  |

获取机械臂关节当前反馈力矩

Returns
:   关节当前反馈力矩

### ◆get\_actual\_tcp\_pose()

| CartesianPose lebai::l\_master::Robot::get\_actual\_tcp\_pose | ( |  | ) |  |

获取机械臂末端在机械臂基坐标系下的实际位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值.

Returns
:   CartesianPose

### ◆get\_box\_devices()

| std::vector<std::string> lebai::l\_master::Robot::get\_box\_devices | ( | const std::string & | **prefix** | ) |  |

获取控制箱dev设备列表

Parameters
:   |  |  |
    | --- | --- |
    | prefix | 设备名前缀过滤条件. |

Returns
:   设备名列表

### ◆get\_dirs()

| std::vector<DirData> lebai::l\_master::Robot::get\_dirs | ( |  | ) |  |

获取数据库目录列表

Returns
:   目录列表

### ◆get\_estop\_reason()

| int lebai::l\_master::Robot::get\_estop\_reason | ( |  | ) |  |

查看急停原因

Returns
:   急停原因

### ◆get\_hardware\_info()

| HardwareInfoData lebai::l\_master::Robot::get\_hardware\_info | ( |  | ) |  |

获取控制器硬件信息

Returns
:   控制器硬件信息

### ◆get\_joint\_temp()

| double lebai::l\_master::Robot::get\_joint\_temp | ( | unsigned int | **joint\_index** | ) |  |

获取单个关节温度

Parameters
:   |  |  |
    | --- | --- |
    | joint\_index | 关节索引 |

Returns
:   double 关节当前温度

### ◆get\_kin\_data()

| JointMotionData lebai::l\_master::Robot::get\_kin\_data | ( |  | ) |  |

获取机械臂关节运动数据

Returns
:   机械臂关节运动数据

### ◆get\_led\_styles()

| std::map<std::string, LedStyleData> lebai::l\_master::Robot::get\_led\_styles | ( |  | ) |  |

获取声光交互样式集

Returns
:   状态到声光样式的映射

### ◆get\_messages()

| std::vector<MessageData> lebai::l\_master::Robot::get\_messages | ( |  | ) |  |

获取控制器消息列表

Returns
:   消息列表

### ◆get\_ota\_state()

| OtaStateData lebai::l\_master::Robot::get\_ota\_state | ( |  | ) |  |

获取OTA升级状态

Returns
:   OTA升级状态

### ◆get\_phy\_data()

| PhysicalData lebai::l\_master::Robot::get\_phy\_data | ( |  | ) |  |

获取机械臂物理数据

Returns
:   机械臂物理数据

### ◆get\_plugin\_daemon\_stdout()

| CommandStdoutData lebai::l\_master::Robot::get\_plugin\_daemon\_stdout | ( | const std::string & | **name** | ) |  |

获取插件守护进程输出.

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |

### ◆get\_plugin\_store()

| std::vector<PluginStoreInfoData> lebai::l\_master::Robot::get\_plugin\_store | ( |  | ) |  |

查询插件商店列表.

Returns
:   插件商店信息列表.

### ◆get\_robot\_info()

| RobotInfoData lebai::l\_master::Robot::get\_robot\_info | ( |  | ) |  |

获取机器人设备信息

Returns
:   机器人设备信息

### ◆get\_robot\_state()

| int lebai::l\_master::Robot::get\_robot\_state | ( |  | ) |  |

获取机器人状态码

Returns
:   int 返回状态码.

| 状态码 | 状态 | 说明 |

| -1 | 控制系统故障 | 机器人软件控制系统异常 |
| 0 | 硬件通讯故障 | 机器人硬件通讯故障 |
| 1 | 已急停 | 机器人处于急停状态，请确认安全性 |
| 2 | 初始化中 | 机器人初始化中 |
| 4 | 初始化完成 | 机器人电源已开启 |
| 5 | 空闲 | 机器人处于空闲状态 |
| 6 | 暂停 | 机器人处于暂停中状态 |
| 7 | 运行中 | 机器人运行中 |
| 8 | 更新中 | 机器人系统更新中 |
| 9 | 启动中 | 机器人初始化完成到空闲的启动过程中 |
| 10 | 正在停止 | 机器人空闲状态转到停止状态 |
| 11 | 示教中 | 机器人处于示教模式中 |
| 12 | 已停止 | 机器人处于停止状态，非急停状态 |

Note
:   具体信息: <https://help.lebai.ltd/guide/basic.html#%E6%9C%BA%E5%99%A8%E4%BA%BA%E7%8A%B6%E6%80%81>

### ◆get\_servo\_params()

| std::vector<ServoParamData> lebai::l\_master::Robot::get\_servo\_params | ( |  | ) |  |

获取伺服参数

Returns
:   伺服参数列表

### ◆get\_short\_poses()

| std::vector<ShortcutData> lebai::l\_master::Robot::get\_short\_poses | ( |  | ) |  |

获取所有快捷路点

Returns
:   快捷路点列表

### ◆get\_short\_tasks()

| std::vector<ShortcutData> lebai::l\_master::Robot::get\_short\_tasks | ( |  | ) |  |

获取所有快捷任务

Returns
:   快捷任务列表

### ◆get\_software\_info()

| SoftwareInfoData lebai::l\_master::Robot::get\_software\_info | ( |  | ) |  |

获取控制器软件信息

Returns
:   控制器软件信息

### ◆get\_system\_info()

| SystemInfoData lebai::l\_master::Robot::get\_system\_info | ( |  | ) |  |

获取控制器系统信息

Returns
:   控制器系统信息

### ◆get\_target\_joint\_positions()

| std::vector<double> lebai::l\_master::Robot::get\_target\_joint\_positions | ( |  | ) |  |

获取机械臂关节当前控制位置

Returns
:   关节控制位置数组，包括所有关节的角度值.

### ◆get\_target\_joint\_speed()

| std::vector<double> lebai::l\_master::Robot::get\_target\_joint\_speed | ( |  | ) |  |

获取机械臂关节当前控制力矩

Returns
:   关节控制速度数据

### ◆get\_target\_joint\_torques()

| std::vector<double> lebai::l\_master::Robot::get\_target\_joint\_torques | ( |  | ) |  |

获取机械臂关节当前控制力矩

Returns
:   关节当前控制力矩

### ◆get\_target\_tcp\_pose()

| CartesianPose lebai::l\_master::Robot::get\_target\_tcp\_pose | ( |  | ) |  |

获取机械臂末端在机械臂基坐标系下的控制位姿，CartesianPose = std::map<std::string,double>，应当包括键为x,y,z,rz,ry,rx的值.

Returns
:   CartesianPose

### ◆get\_tcp\_force()

| WrenchData lebai::l\_master::Robot::get\_tcp\_force | ( |  | ) |  |

获取末端受力

Returns
:   末端受力和力矩

### ◆get\_triggers()

| std::vector<TriggerData> lebai::l\_master::Robot::get\_triggers | ( |  | ) |  |

获取条件任务触发器列表

Returns
:   触发器列表

### ◆get\_upgrade\_stdout()

| CommandStdoutData lebai::l\_master::Robot::get\_upgrade\_stdout | ( |  | ) |  |

获取系统升级输出

Returns
:   升级输出

### ◆http()

| HttpResponseData lebai::l\_master::Robot::http | ( | HttpRequestData | **request** | ) |  |

通过控制器发起HTTP请求.

Parameters
:   |  |  |
    | --- | --- |
    | request | HTTP请求. |

Returns
:   HTTP响应.

### ◆is\_disconnected()

| bool lebai::l\_master::Robot::is\_disconnected | ( |  | ) |  |

是否已与手臂断开连接

Returns
:   是否已断开连接

### ◆is\_down()

| bool lebai::l\_master::Robot::is\_down | ( |  | ) |  |

手臂是否已下电

Returns
:   是否已下电

### ◆load\_led\_style()

| LedStyleData lebai::l\_master::Robot::load\_led\_style | ( | std::string | **name**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

查询声光样式.

Parameters
:   |  |  |
    | --- | --- |
    | name | 样式名称. |
    | dir | 样式目录. |

### ◆load\_led\_style\_list()

| std::vector<std::string> lebai::l\_master::Robot::load\_led\_style\_list | ( | std::string | **dir** = `""` | ) |  |

查询声光样式列表.

Parameters
:   |  |  |
    | --- | --- |
    | dir | 声光样式目录. |

### ◆load\_plugin()

| PluginInfoData lebai::l\_master::Robot::load\_plugin | ( | const std::string & | **name** | ) |  |

查询已安装插件

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |

### ◆load\_plugins()

| std::vector<PluginInfoData> lebai::l\_master::Robot::load\_plugins | ( |  | ) |  |

查询已安装插件列表

Returns
:   插件信息列表

### ◆restart\_plugin\_daemon()

| void lebai::l\_master::Robot::restart\_plugin\_daemon | ( | const std::string & | **name** | ) |  |

重启插件守护进程.

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |

### ◆run\_plugin\_cmd()

| CommandStdoutData lebai::l\_master::Robot::run\_plugin\_cmd | ( | const std::string & | **name**, |
|  |  | const std::vector< std::string > & | **params** = `std::vector< std::string >()` |
|  | ) |  |  |

执行插件命令.

Parameters
:   |  |  |
    | --- | --- |
    | name | 插件名称. |
    | params | 命令参数. |

### ◆save\_led\_style()

| void lebai::l\_master::Robot::save\_led\_style | ( | std::string | **name**, |
|  |  | LedStyleData | **style**, |
|  |  | std::string | **dir** = `""` |
|  | ) |  |  |

保存声光样式.

Parameters
:   |  |  |
    | --- | --- |
    | name | 样式名称. |
    | style | 样式数据. |
    | dir | 样式目录. |

### ◆set\_force\_mode\_param()

| void lebai::l\_master::Robot::set\_force\_mode\_param | ( | double | **damping**, |
|  |  | double | **mass**, |
|  |  | double | **force\_threshold**, |
|  |  | double | **torque\_threshold** |
|  | ) |  |  |

设置力控参数.

Parameters
:   |  |  |
    | --- | --- |
    | damping | 阻尼. |
    | mass | 物体重量. |
    | force\_threshold | 启动力阈值. |
    | torque\_threshold | 启动力矩阈值. |

### ◆set\_force\_mode\_sensor()

| void lebai::l\_master::Robot::set\_force\_mode\_sensor | ( | const std::string & | **sensor**, |
|  |  | unsigned int | **address** |
|  | ) |  |  |

设置力控传感器.

Parameters
:   |  |  |
    | --- | --- |
    | sensor | 传感器类型. |
    | address | 传感器地址. |

### ◆set\_led\_style()

| void lebai::l\_master::Robot::set\_led\_style | ( | std::string | **state**, |
|  |  | LedStyleData | **style** |
|  | ) |  |  |

设置机器人状态对应的声光交互样式.

Parameters
:   |  |  |
    | --- | --- |
    | state | 机器人状态名称. |
    | style | 样式数据. |

### ◆set\_led\_styles()

| void lebai::l\_master::Robot::set\_led\_styles | ( | std::map< std::string, LedStyleData > | **styles** | ) |  |

设置声光交互样式集

Parameters
:   |  |  |
    | --- | --- |
    | styles | 状态到声光样式的映射 |

### ◆set\_tcp\_force()

| void lebai::l\_master::Robot::set\_tcp\_force | ( | const WrenchData & | **wrench** | ) |  |

设置末端受力

Parameters
:   |  |  |
    | --- | --- |
    | wrench | 末端受力和力矩 |

### ◆start\_force\_mode()

| void lebai::l\_master::Robot::start\_force\_mode | ( | const CartesianPose & | **limit**, |
|  |  | const WrenchData & | **wrench** |
|  | ) |  |  |

开始力控模式.

Parameters
:   |  |  |
    | --- | --- |
    | limit | 末端限速. |
    | wrench | 控制力. |

### ◆update\_dir()

| void lebai::l\_master::Robot::update\_dir | ( | std::string | **from**, |
|  |  | std::string | **to** |
|  | ) |  |  |

重命名数据库目录.

Parameters
:   |  |  |
    | --- | --- |
    | from | 原目录名. |
    | to | 新目录名. |


---

# STORAGE

| Functions | |
| void | lebai::l\_master::Robot::set\_item (StorageItem item) |
|  | 设置存储项. |

| StorageItem | lebai::l\_master::Robot::get\_item (std::string name) |
|  | 获取存储项. |

| std::vector< StorageItem > | lebai::l\_master::Robot::get\_items (std::string prefix) |
|  | 获取存储项列表. |

### Detailed Description

### Function Documentation

### ◆get\_item()

| StorageItem lebai::l\_master::Robot::get\_item | ( | std::string | **name** | ) |  |

获取存储项.

Parameters
:   |  |  |
    | --- | --- |
    | name | 存储项名称. |

Returns
:   存储项值.

### ◆get\_items()

| std::vector<StorageItem> lebai::l\_master::Robot::get\_items | ( | std::string | **prefix** | ) |  |

获取存储项列表.

Parameters
:   |  |  |
    | --- | --- |
    | prefix | 存储项前缀. |

Returns
:   存储项列表.

### ◆set\_item()

| void lebai::l\_master::Robot::set\_item | ( | StorageItem | **item** | ) |  |

设置存储项.

Parameters
:   |  |  |
    | --- | --- |
    | item | 存储项数据结构. |


---

# lebai::l_master::BackupInfoData Struct Reference

Collaboration diagram for lebai::l\_master::BackupInfoData:

![Collaboration graph](structlebai_1_1l__master_1_1_backup_info_data__coll__graph.png)

[legend]

| Public Attributes | |
| SystemInfoData | **system** |

| RobotInfoData | **robot** |

| HardwareInfoData | **hardware** |

| SoftwareInfoData | **software** |

| int64\_t | **timestamp\_seconds** = 0 |

| int32\_t | **timestamp\_nanos** = 0 |

| BackupOptionsData | **option** |

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::BackupOptionsData Struct Reference

| Public Attributes | |
| bool | **tmp** = false |

| bool | **syslog** = false |

| bool | **arm** = false |

| bool | **config** = false |

| bool | **data** = false |

| bool | **file** = false |

| bool | **docker** = false |

| bool | **ds** = false |

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ButtonIndexData Struct Reference

| Public Attributes | |
| std::string | device |

| uint32\_t | pin = 0 |

### Member Data Documentation

### ◆device

| std::string lebai::l\_master::ButtonIndexData::device |

按键设备.

### ◆pin

| uint32\_t lebai::l\_master::ButtonIndexData::pin = 0 |

按键编号.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ButtonStatusData Struct Reference

| Public Attributes | |
| std::string | state |

| uint32\_t | time = 0 |

### Member Data Documentation

### ◆state

| std::string lebai::l\_master::ButtonStatusData::state |

按键状态.

### ◆time

| uint32\_t lebai::l\_master::ButtonStatusData::time = 0 |

持续时间.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::CartesianLimitConfig Struct Reference

笛卡尔空间限位配置.

`#include <robot.hh>`

| Public Attributes | |
| double | max\_a = 0.0 |

| double | max\_v = 0.0 |

| double | eq\_radius = 0.0 |

### Detailed Description

笛卡尔空间限位配置.

### Member Data Documentation

### ◆eq\_radius

| double lebai::l\_master::CartesianLimitConfig::eq\_radius = 0.0 |

等效半径

### ◆max\_a

| double lebai::l\_master::CartesianLimitConfig::max\_a = 0.0 |

笛卡尔空间最大加速度

### ◆max\_v

| double lebai::l\_master::CartesianLimitConfig::max\_v = 0.0 |

笛卡尔空间最大速度

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::CheckUpgradeData Struct Reference

| Public Attributes | |
| bool | need\_upgrade = false |

| std::string | introduction |

### Member Data Documentation

### ◆introduction

| std::string lebai::l\_master::CheckUpgradeData::introduction |

升级介绍.

### ◆need\_upgrade

| bool lebai::l\_master::CheckUpgradeData::need\_upgrade = false |

是否需要升级.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ClawData Struct Reference

夹爪数据结构.

`#include <robot.hh>`

| Public Attributes | |
| double | force |

| double | amplitude |

| bool | hold\_on |

### Detailed Description

夹爪数据结构.

### Member Data Documentation

### ◆amplitude

| double lebai::l\_master::ClawData::amplitude |

夹爪的开合角度值.

### ◆force

| double lebai::l\_master::ClawData::force |

夹爪的力矩值.

### ◆hold\_on

| bool lebai::l\_master::ClawData::hold\_on |

夹爪是否夹住.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::CollisionDetectorConfig Struct Reference

碰撞检测配置.

`#include <robot.hh>`

| Public Attributes | |
| unsigned int | action = 99 |

| unsigned int | pause\_time = 0 |

| unsigned int | sensitivity = 0 |

### Detailed Description

碰撞检测配置.

### Member Data Documentation

### ◆action

| unsigned int lebai::l\_master::CollisionDetectorConfig::action = 99 |

0:急停; 1:暂停; 2:停止运动; 99:关闭碰撞检测

### ◆pause\_time

| unsigned int lebai::l\_master::CollisionDetectorConfig::pause\_time = 0 |

action为1时的暂停时间，单位秒

### ◆sensitivity

| unsigned int lebai::l\_master::CollisionDetectorConfig::sensitivity = 0 |

灵敏度[0-100]

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::CommandStdoutData Struct Reference

| Public Attributes | |
| bool | done = false |

| std::string | stdout\_text |

| std::string | stderr\_text |

| int | code = 0 |

### Member Data Documentation

### ◆code

| int lebai::l\_master::CommandStdoutData::code = 0 |

返回码.

### ◆done

| bool lebai::l\_master::CommandStdoutData::done = false |

命令是否完成.

### ◆stderr\_text

| std::string lebai::l\_master::CommandStdoutData::stderr\_text |

标准错误.

### ◆stdout\_text

| std::string lebai::l\_master::CommandStdoutData::stdout\_text |

标准输出.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::DeviceInfoData Struct Reference

控制器硬件设备信息.

`#include <robot.hh>`

| Public Attributes | |
| bool | invalid = false |

| std::string | sn |

| std::string | version |

| std::string | partition |

| uint32\_t | di\_num = 0 |

| uint32\_t | do\_num = 0 |

| uint32\_t | dio\_num = 0 |

| uint32\_t | ai\_num = 0 |

| uint32\_t | ao\_num = 0 |

### Detailed Description

控制器硬件设备信息.

### Member Data Documentation

### ◆ai\_num

| uint32\_t lebai::l\_master::DeviceInfoData::ai\_num = 0 |

AI数量.

### ◆ao\_num

| uint32\_t lebai::l\_master::DeviceInfoData::ao\_num = 0 |

AO数量.

### ◆di\_num

| uint32\_t lebai::l\_master::DeviceInfoData::di\_num = 0 |

DI数量.

### ◆dio\_num

| uint32\_t lebai::l\_master::DeviceInfoData::dio\_num = 0 |

DIO数量.

### ◆do\_num

| uint32\_t lebai::l\_master::DeviceInfoData::do\_num = 0 |

DO数量.

### ◆invalid

| bool lebai::l\_master::DeviceInfoData::invalid = false |

设备信息是否无效.

### ◆partition

| std::string lebai::l\_master::DeviceInfoData::partition |

固件分区.

### ◆sn

| std::string lebai::l\_master::DeviceInfoData::sn |

序列号.

### ◆version

| std::string lebai::l\_master::DeviceInfoData::version |

固件版本.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::DhParamData Struct Reference

| Public Attributes | |
| double | a = 0.0 |

| double | alpha = 0.0 |

| double | d = 0.0 |

| double | theta = 0.0 |

### Member Data Documentation

### ◆a

| double lebai::l\_master::DhParamData::a = 0.0 |

DH参数a.

### ◆alpha

| double lebai::l\_master::DhParamData::alpha = 0.0 |

DH参数alpha.

### ◆d

| double lebai::l\_master::DhParamData::d = 0.0 |

DH参数d.

### ◆theta

| double lebai::l\_master::DhParamData::theta = 0.0 |

DH参数theta.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::DirData Struct Reference

| Public Attributes | |
| std::string | name |

| uint32\_t | id = 0 |

### Member Data Documentation

### ◆id

| uint32\_t lebai::l\_master::DirData::id = 0 |

目录内部ID.

### ◆name

| std::string lebai::l\_master::DirData::name |

目录名.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::DiscoveredRobotData Struct Reference

| Public Attributes | |
| std::string | name |

| std::string | mac |

| std::string | ip |

| bool | online = false |

### Member Data Documentation

### ◆ip

| std::string lebai::l\_master::DiscoveredRobotData::ip |

设备IP地址.

### ◆mac

| std::string lebai::l\_master::DiscoveredRobotData::mac |

设备物理唯一标识.

### ◆name

| std::string lebai::l\_master::DiscoveredRobotData::name |

设备名称.

### ◆online

| bool lebai::l\_master::DiscoveredRobotData::online = false |

设备是否在线.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ExtraServoParamData Struct Reference

| Public Attributes | |
| double | **acc\_position\_kp** = 0.0 |

| double | **acc\_speed\_kp** = 0.0 |

| double | **acc\_speed\_it** = 0.0 |

| double | **uni\_position\_kp** = 0.0 |

| double | **uni\_speed\_kp** = 0.0 |

| double | **uni\_speed\_it** = 0.0 |

| double | **dec\_position\_kp** = 0.0 |

| double | **dec\_speed\_kp** = 0.0 |

| double | **dec\_speed\_it** = 0.0 |

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::FrameData Struct Reference

| Public Attributes | |
| std::string | position\_kind |

| CartesianPose | position |

| std::string | rotation\_kind |

| CartesianPose | rotation |

### Member Data Documentation

### ◆position

| CartesianPose lebai::l\_master::FrameData::position |

自定义位置，使用x/y/z键.

### ◆position\_kind

| std::string lebai::l\_master::FrameData::position\_kind |

位置参考系类型.

### ◆rotation

| CartesianPose lebai::l\_master::FrameData::rotation |

自定义姿态，使用rx/ry/rz键.

### ◆rotation\_kind

| std::string lebai::l\_master::FrameData::rotation\_kind |

姿态参考系类型.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::HardwareInfoData Struct Reference

Collaboration diagram for lebai::l\_master::HardwareInfoData:

![Collaboration graph](structlebai_1_1l__master_1_1_hardware_info_data__coll__graph.png)

[legend]

| Public Attributes | |
| DeviceInfoData | comboard |

| std::vector< DeviceInfoData > | joints |

| DeviceInfoData | flange |

| DeviceInfoData | led |

| DeviceInfoData | extra\_io |

### Member Data Documentation

### ◆comboard

| DeviceInfoData lebai::l\_master::HardwareInfoData::comboard |

通信板信息.

### ◆extra\_io

| DeviceInfoData lebai::l\_master::HardwareInfoData::extra\_io |

扩展IO信息.

### ◆flange

| DeviceInfoData lebai::l\_master::HardwareInfoData::flange |

法兰信息.

### ◆joints

| std::vector<DeviceInfoData> lebai::l\_master::HardwareInfoData::joints |

关节信息.

### ◆led

| DeviceInfoData lebai::l\_master::HardwareInfoData::led |

灯板信息.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::HttpRequestData Struct Reference

| Public Attributes | |
| std::string | method |

| std::string | url |

| std::map< std::string, std::string > | headers |

| std::string | body |

### Member Data Documentation

### ◆body

| std::string lebai::l\_master::HttpRequestData::body |

请求体.

### ◆headers

| std::map<std::string, std::string> lebai::l\_master::HttpRequestData::headers |

请求头.

### ◆method

| std::string lebai::l\_master::HttpRequestData::method |

HTTP方法.

### ◆url

| std::string lebai::l\_master::HttpRequestData::url |

请求URL.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::HttpResponseData Struct Reference

| Public Attributes | |
| unsigned int | status = 0 |

| std::map< std::string, std::string > | headers |

| std::string | body |

### Member Data Documentation

### ◆body

| std::string lebai::l\_master::HttpResponseData::body |

响应体.

### ◆headers

| std::map<std::string, std::string> lebai::l\_master::HttpResponseData::headers |

响应头.

### ◆status

| unsigned int lebai::l\_master::HttpResponseData::status = 0 |

HTTP状态码.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::JointLimitConfig Struct Reference

关节限位配置.

`#include <robot.hh>`

| Public Attributes | |
| double | min\_position = 0.0 |

| double | max\_position = 0.0 |

| double | max\_a = 0.0 |

| double | max\_v = 0.0 |

### Detailed Description

关节限位配置.

### Member Data Documentation

### ◆max\_a

| double lebai::l\_master::JointLimitConfig::max\_a = 0.0 |

关节最大加速度

### ◆max\_position

| double lebai::l\_master::JointLimitConfig::max\_position = 0.0 |

关节最大位置

### ◆max\_v

| double lebai::l\_master::JointLimitConfig::max\_v = 0.0 |

关节最大速度

### ◆min\_position

| double lebai::l\_master::JointLimitConfig::min\_position = 0.0 |

关节最小位置

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::JointMotionData Struct Reference

机械臂关节运动数据结构.

`#include <robot.hh>`

| Public Attributes | |
| DoubleVector | actual\_joint\_pose |

| DoubleVector | actual\_joint\_speed |

| DoubleVector | actual\_joint\_acc |

| DoubleVector | actual\_joint\_torque |

| DoubleVector | target\_joint\_pose |

| DoubleVector | target\_joint\_speed |

| DoubleVector | target\_joint\_acc |

| DoubleVector | target\_joint\_torque |

| CartesianPose | actual\_tcp\_pose |

| CartesianPose | target\_tcp\_pose |

| CartesianPose | actual\_flange\_pose |

### Detailed Description

机械臂关节运动数据结构.

### Member Data Documentation

### ◆actual\_flange\_pose

| CartesianPose lebai::l\_master::JointMotionData::actual\_flange\_pose |

法兰实际位置.

### ◆actual\_joint\_acc

| DoubleVector lebai::l\_master::JointMotionData::actual\_joint\_acc |

关节实际加速度.

### ◆actual\_joint\_pose

| DoubleVector lebai::l\_master::JointMotionData::actual\_joint\_pose |

关节实际位置.

### ◆actual\_joint\_speed

| DoubleVector lebai::l\_master::JointMotionData::actual\_joint\_speed |

关节实际速度.

### ◆actual\_joint\_torque

| DoubleVector lebai::l\_master::JointMotionData::actual\_joint\_torque |

关节实际力矩.

### ◆actual\_tcp\_pose

| CartesianPose lebai::l\_master::JointMotionData::actual\_tcp\_pose |

法兰实际位置.

### ◆target\_joint\_acc

| DoubleVector lebai::l\_master::JointMotionData::target\_joint\_acc |

关节目标加速度.

### ◆target\_joint\_pose

| DoubleVector lebai::l\_master::JointMotionData::target\_joint\_pose |

关节目标位置.

### ◆target\_joint\_speed

| DoubleVector lebai::l\_master::JointMotionData::target\_joint\_speed |

关节目标速度.

### ◆target\_joint\_torque

| DoubleVector lebai::l\_master::JointMotionData::target\_joint\_torque |

关节目标力矩.

### ◆target\_tcp\_pose

| CartesianPose lebai::l\_master::JointMotionData::target\_tcp\_pose |

法兰目标位置.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::JointMoveData Struct Reference

| Public Attributes | |
| double | pose = 0.0 |

| double | velocity = 0.0 |

| double | acc = 0.0 |

### Member Data Documentation

### ◆acc

| double lebai::l\_master::JointMoveData::acc = 0.0 |

关节加速度.

### ◆pose

| double lebai::l\_master::JointMoveData::pose = 0.0 |

关节位置.

### ◆velocity

| double lebai::l\_master::JointMoveData::velocity = 0.0 |

关节速度.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::KinematicsForwardResp Struct Reference

运动学正解的返回值数据结构.

`#include <robot.hh>`

| Public Attributes | |
| CartesianPose | pose |

| bool | ok = false |

### Detailed Description

运动学正解的返回值数据结构.

### Member Data Documentation

### ◆ok

| bool lebai::l\_master::KinematicsForwardResp::ok = false |

计算是否成功.

### ◆pose

| CartesianPose lebai::l\_master::KinematicsForwardResp::pose |

笛卡尔坐标位置，map数据，应当包括'x','y','z','rx','ry','rz'的键值.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::KinematicsInverseResp Struct Reference

运动学逆解的返回值数据结构

`#include <robot.hh>`

| Public Attributes | |
| DoubleVector | joint\_positions |

| bool | ok = false |

### Detailed Description

运动学逆解的返回值数据结构

### Member Data Documentation

### ◆joint\_positions

| DoubleVector lebai::l\_master::KinematicsInverseResp::joint\_positions |

机械臂关节位置的map数据，应当包括'j1','j2','j3','j4','j5','j6'六个关节的角度值.

### ◆ok

| bool lebai::l\_master::KinematicsInverseResp::ok = false |

计算是否成功

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::LedStyleData Struct Reference

| Public Attributes | |
| std::string | mode |

| std::string | speed |

| std::vector< std::string > | colors |

| std::string | voice |

| std::string | volume |

### Member Data Documentation

### ◆colors

| std::vector<std::string> lebai::l\_master::LedStyleData::colors |

LED颜色.

### ◆mode

| std::string lebai::l\_master::LedStyleData::mode |

LED工作模式.

### ◆speed

| std::string lebai::l\_master::LedStyleData::speed |

LED运行速度.

### ◆voice

| std::string lebai::l\_master::LedStyleData::voice |

语音ID.

### ◆volume

| std::string lebai::l\_master::LedStyleData::volume |

语音音量.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::MessageData Struct Reference

| Public Attributes | |
| std::string | level |

| std::string | kind |

| std::string | detail |

| std::string | time |

### Member Data Documentation

### ◆detail

| std::string lebai::l\_master::MessageData::detail |

详细信息.

### ◆kind

| std::string lebai::l\_master::MessageData::kind |

消息类型.

### ◆level

| std::string lebai::l\_master::MessageData::level |

消息等级.

### ◆time

| std::string lebai::l\_master::MessageData::time |

消息产生时间.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ModbusData Struct Reference

| Public Attributes | |
| std::string | kind |

| std::string | address |

| unsigned int | port = 0 |

| unsigned int | slave\_id = 0 |

### Member Data Documentation

### ◆address

| std::string lebai::l\_master::ModbusData::address |

IP地址或串口地址.

### ◆kind

| std::string lebai::l\_master::ModbusData::kind |

Modbus设备类型.

### ◆port

| unsigned int lebai::l\_master::ModbusData::port = 0 |

TCP端口号.

### ◆slave\_id

| unsigned int lebai::l\_master::ModbusData::slave\_id = 0 |

Modbus从机地址.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ModbusRegisterData Struct Reference

| Public Attributes | |
| std::string | kind |

| unsigned int | address = 0 |

### Member Data Documentation

### ◆address

| unsigned int lebai::l\_master::ModbusRegisterData::address = 0 |

Modbus内存地址.

### ◆kind

| std::string lebai::l\_master::ModbusRegisterData::kind |

Modbus寄存器类型.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::OtaStateData Struct Reference

| Public Attributes | |
| std::string | address |

| std::string | partition |

| std::string | step |

| uint32\_t | progress = 0 |

### Member Data Documentation

### ◆address

| std::string lebai::l\_master::OtaStateData::address |

硬件地址.

### ◆partition

| std::string lebai::l\_master::OtaStateData::partition |

固件分区.

### ◆progress

| uint32\_t lebai::l\_master::OtaStateData::progress = 0 |

升级进度.

### ◆step

| std::string lebai::l\_master::OtaStateData::step |

升级步骤.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::PhysicalData Struct Reference

机械臂物理数据结构.

`#include <robot.hh>`

| Public Attributes | |
| DoubleVector | joint\_temperature |

| DoubleVector | joint\_voltage |

| double | flange\_voltage |

### Detailed Description

机械臂物理数据结构.

### Member Data Documentation

### ◆flange\_voltage

| double lebai::l\_master::PhysicalData::flange\_voltage |

法兰电压.

### ◆joint\_temperature

| DoubleVector lebai::l\_master::PhysicalData::joint\_temperature |

关节温度.

### ◆joint\_voltage

| DoubleVector lebai::l\_master::PhysicalData::joint\_voltage |

关节电压.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::PluginInfoData Struct Reference

| Public Attributes | |
| std::string | name |

| std::vector< std::string > | boxs |

| std::vector< std::string > | arms |

| std::string | description |

| std::string | homepage |

| bool | auto\_restart = false |

| bool | web = false |

| bool | daemon = false |

| bool | cmd = false |

| bool | enable = false |

### Member Data Documentation

### ◆arms

| std::vector<std::string> lebai::l\_master::PluginInfoData::arms |

适用手臂型号.

### ◆auto\_restart

| bool lebai::l\_master::PluginInfoData::auto\_restart = false |

daemon异常退出后是否自动启动.

### ◆boxs

| std::vector<std::string> lebai::l\_master::PluginInfoData::boxs |

适用机箱型号.

### ◆cmd

| bool lebai::l\_master::PluginInfoData::cmd = false |

是否具备Cmd功能.

### ◆daemon

| bool lebai::l\_master::PluginInfoData::daemon = false |

是否具备Daemon功能.

### ◆description

| std::string lebai::l\_master::PluginInfoData::description |

插件描述.

### ◆enable

| bool lebai::l\_master::PluginInfoData::enable = false |

是否启用.

### ◆homepage

| std::string lebai::l\_master::PluginInfoData::homepage |

项目主页.

### ◆name

| std::string lebai::l\_master::PluginInfoData::name |

插件名称.

### ◆web

| bool lebai::l\_master::PluginInfoData::web = false |

是否具备Web功能.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::PluginStoreInfoData Struct Reference

| Public Attributes | |
| std::string | name |

| std::string | url |

### Member Data Documentation

### ◆name

| std::string lebai::l\_master::PluginStoreInfoData::name |

插件名称.

### ◆url

| std::string lebai::l\_master::PluginStoreInfoData::url |

插件下载地址.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::PoseData Struct Reference

| Public Attributes | |
| std::string | kind |

| CartesianPose | cart |

| std::vector< double > | joint |

### Member Data Documentation

### ◆cart

| CartesianPose lebai::l\_master::PoseData::cart |

笛卡尔位姿.

### ◆joint

| std::vector<double> lebai::l\_master::PoseData::joint |

关节位姿.

### ◆kind

| std::string lebai::l\_master::PoseData::kind |

位姿类型.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::PvatPointData Struct Reference

| Public Attributes | |
| double | duration = 0.0 |

| std::vector< JointMoveData > | joints |

### Member Data Documentation

### ◆duration

| double lebai::l\_master::PvatPointData::duration = 0.0 |

执行时间.

### ◆joints

| std::vector<JointMoveData> lebai::l\_master::PvatPointData::joints |

关节运动数据.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::RobotInfoData Struct Reference

机器人设备信息.

`#include <robot.hh>`

| Public Attributes | |
| std::string | name |

| std::string | mac |

| std::string | box\_model |

| std::string | box\_sn |

| std::string | arm\_model |

| std::string | arm\_sn |

### Detailed Description

机器人设备信息.

### Member Data Documentation

### ◆arm\_model

| std::string lebai::l\_master::RobotInfoData::arm\_model |

手臂型号.

### ◆arm\_sn

| std::string lebai::l\_master::RobotInfoData::arm\_sn |

手臂序列号.

### ◆box\_model

| std::string lebai::l\_master::RobotInfoData::box\_model |

控制箱型号.

### ◆box\_sn

| std::string lebai::l\_master::RobotInfoData::box\_sn |

控制箱序列号.

### ◆mac

| std::string lebai::l\_master::RobotInfoData::mac |

设备物理唯一标识.

### ◆name

| std::string lebai::l\_master::RobotInfoData::name |

设备名称.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ServoParamData Struct Reference

| Public Attributes | |
| double | position\_kp = 0.0 |

| double | speed\_kp = 0.0 |

| double | speed\_it = 0.0 |

| double | torque\_cmd\_filter = 0.0 |

### Member Data Documentation

### ◆position\_kp

| double lebai::l\_master::ServoParamData::position\_kp = 0.0 |

位置环KP.

### ◆speed\_it

| double lebai::l\_master::ServoParamData::speed\_it = 0.0 |

速度环IT.

### ◆speed\_kp

| double lebai::l\_master::ServoParamData::speed\_kp = 0.0 |

速度环KP.

### ◆torque\_cmd\_filter

| double lebai::l\_master::ServoParamData::torque\_cmd\_filter = 0.0 |

力矩命令滤波.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::ShortcutData Struct Reference

| Public Attributes | |
| uint32\_t | id = 0 |

| std::string | dir |

| std::string | name |

### Member Data Documentation

### ◆dir

| std::string lebai::l\_master::ShortcutData::dir |

目标目录.

### ◆id

| uint32\_t lebai::l\_master::ShortcutData::id = 0 |

快捷方式ID.

### ◆name

| std::string lebai::l\_master::ShortcutData::name |

目标名称.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::SoftwareInfoData Struct Reference

| Public Attributes | |
| std::map< std::string, SoftwareItemInfoData > | software |

### Member Data Documentation

### ◆software

| std::map<std::string, SoftwareItemInfoData> lebai::l\_master::SoftwareInfoData::software |

软件信息.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::SoftwareItemInfoData Struct Reference

| Public Attributes | |
| std::string | version |

| std::string | branch |

### Member Data Documentation

### ◆branch

| std::string lebai::l\_master::SoftwareItemInfoData::branch |

软件分支.

### ◆version

| std::string lebai::l\_master::SoftwareItemInfoData::version |

软件版本.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::StorageItem Struct Reference

存储项数据结构.

`#include <robot.hh>`

| Public Attributes | |
| std::string | key |

| std::string | value |

### Detailed Description

存储项数据结构.

### Member Data Documentation

### ◆key

| std::string lebai::l\_master::StorageItem::key |

存储项名称.

### ◆value

| std::string lebai::l\_master::StorageItem::value |

存储项值.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::StructureData Struct Reference

| Public Attributes | |
| bool | active = false |

| std::string | name |

| std::string | kind |

| unsigned int | dof = 0 |

| std::string | dh |

| std::string | dyn |

| std::string | servo |

### Member Data Documentation

### ◆active

| bool lebai::l\_master::StructureData::active = false |

是否激活.

### ◆dh

| std::string lebai::l\_master::StructureData::dh |

DH参数资源名.

### ◆dof

| unsigned int lebai::l\_master::StructureData::dof = 0 |

自由度.

### ◆dyn

| std::string lebai::l\_master::StructureData::dyn |

动力学参数资源名.

### ◆kind

| std::string lebai::l\_master::StructureData::kind |

结构型号.

### ◆name

| std::string lebai::l\_master::StructureData::name |

结构名称.

### ◆servo

| std::string lebai::l\_master::StructureData::servo |

伺服参数资源名.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::SystemInfoData Struct Reference

控制器系统信息数据结构.

`#include <robot.hh>`

| Public Attributes | |
| std::string | name |

| std::string | kernel\_version |

| std::string | os\_version |

| std::string | host\_name |

| uint64\_t | used\_memory = 0 |

| uint64\_t | total\_memory = 0 |

### Detailed Description

控制器系统信息数据结构.

### Member Data Documentation

### ◆host\_name

| std::string lebai::l\_master::SystemInfoData::host\_name |

主机名.

### ◆kernel\_version

| std::string lebai::l\_master::SystemInfoData::kernel\_version |

内核版本.

### ◆name

| std::string lebai::l\_master::SystemInfoData::name |

操作系统名字.

### ◆os\_version

| std::string lebai::l\_master::SystemInfoData::os\_version |

系统版本.

### ◆total\_memory

| uint64\_t lebai::l\_master::SystemInfoData::total\_memory = 0 |

总内存.

### ◆used\_memory

| uint64\_t lebai::l\_master::SystemInfoData::used\_memory = 0 |

已使用内存.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::TaskData Struct Reference

| Public Attributes | |
| unsigned int | id = 0 |

| std::string | state |

| unsigned int | loop\_count = 0 |

| unsigned int | loop\_to = 0 |

| bool | is\_parallel = false |

| bool | is\_simu = false |

| std::string | stdout\_text |

| std::string | kind |

| std::string | dir |

| std::string | name |

| std::vector< std::string > | params |

### Member Data Documentation

### ◆dir

| std::string lebai::l\_master::TaskData::dir |

任务目录.

### ◆id

| unsigned int lebai::l\_master::TaskData::id = 0 |

任务ID.

### ◆is\_parallel

| bool lebai::l\_master::TaskData::is\_parallel = false |

是否并行运行.

### ◆is\_simu

| bool lebai::l\_master::TaskData::is\_simu = false |

是否为仿真模式.

### ◆kind

| std::string lebai::l\_master::TaskData::kind |

任务类型.

### ◆loop\_count

| unsigned int lebai::l\_master::TaskData::loop\_count = 0 |

已完成循环次数.

### ◆loop\_to

| unsigned int lebai::l\_master::TaskData::loop\_to = 0 |

目标循环次数.

### ◆name

| std::string lebai::l\_master::TaskData::name |

任务名称.

### ◆params

| std::vector<std::string> lebai::l\_master::TaskData::params |

任务参数.

### ◆state

| std::string lebai::l\_master::TaskData::state |

任务状态.

### ◆stdout\_text

| std::string lebai::l\_master::TaskData::stdout\_text |

任务输出.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::TaskStdoutData Struct Reference

| Public Attributes | |
| unsigned int | id = 0 |

| bool | done = false |

| std::string | stdout\_text |

### Member Data Documentation

### ◆done

| bool lebai::l\_master::TaskStdoutData::done = false |

任务是否完成.

### ◆id

| unsigned int lebai::l\_master::TaskStdoutData::id = 0 |

任务ID.

### ◆stdout\_text

| std::string lebai::l\_master::TaskStdoutData::stdout\_text |

标准输出.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::TrajectoryData Struct Reference

| Public Attributes | |
| std::string | kind |

| std::vector< PvatPointData > | data |

### Member Data Documentation

### ◆data

| std::vector<PvatPointData> lebai::l\_master::TrajectoryData::data |

轨迹数据.

### ◆kind

| std::string lebai::l\_master::TrajectoryData::kind |

轨迹类型.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::TriggerConditionData Struct Reference

Collaboration diagram for lebai::l\_master::TriggerConditionData:

![Collaboration graph](structlebai_1_1l__master_1_1_trigger_condition_data__coll__graph.png)

[legend]

| Public Attributes | |
| std::vector< ButtonIndexData > | pressed |

| ButtonIndexData | button |

| ButtonStatusData | status |

### Member Data Documentation

### ◆button

| ButtonIndexData lebai::l\_master::TriggerConditionData::button |

条件按键.

### ◆pressed

| std::vector<ButtonIndexData> lebai::l\_master::TriggerConditionData::pressed |

同时按下的按键.

### ◆status

| ButtonStatusData lebai::l\_master::TriggerConditionData::status |

条件状态.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::TriggerData Struct Reference

Collaboration diagram for lebai::l\_master::TriggerData:

![Collaboration graph](structlebai_1_1l__master_1_1_trigger_data__coll__graph.png)

[legend]

| Public Attributes | |
| TriggerConditionData | condition |

| std::string | function |

### Member Data Documentation

### ◆condition

| TriggerConditionData lebai::l\_master::TriggerData::condition |

触发条件.

### ◆function

| std::string lebai::l\_master::TriggerData::function |

触发功能.

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh


---

# lebai::l_master::WrenchData Struct Reference

| Public Attributes | |
| DoubleVector | force |

| DoubleVector | torque |

### Member Data Documentation

### ◆force

| DoubleVector lebai::l\_master::WrenchData::force |

受力大小[x,y,z].

### ◆torque

| DoubleVector lebai::l\_master::WrenchData::torque |

扭矩大小[x,y,z].

---

The documentation for this struct was generated from the following file:

- sdk/include/lebai/robot.hh
