# Unitree Go2 SDK (unitree_sdk2py) 接口概览

基于 [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) 官方仓库整理，便于 PoseAdapter 开发参考。

---

## 1. SportClient（高层运动控制）

**模块路径**：`unitree_sdk2py.go2.sport.sport_client`  
**用法**：需先 `ChannelFactoryInitialize(0, "eth0")` 初始化 DDS，再 `SportClient().Init()`。运动前需在 APP 中开启 sport_mode 服务。

### 1.1 基础姿态与运动

| 方法 | 说明 |
|------|------|
| `StandUp()` | 站立 |
| `StandDown()` | 趴下 |
| `BalanceStand()` | 平衡站立（可配合 Euler 调姿态） |
| `RecoveryStand()` | 恢复站立 |
| `StopMove()` | 停止运动 |
| `Move(vx, vy, vyaw)` | 速度控制（机体坐标系），单位 m/s、rad/s |
| `Euler(roll, pitch, yaw)` | 设置机身欧拉角（弧度） |
| `Damp()` | 阻尼模式 |

### 1.2 步态与速度

| 方法 | 说明 |
|------|------|
| **`ClassicWalk(flag: bool)`** | **经典步态（稀碎步）**，`True` 开启、`False` 关闭 |
| `StaticWalk()` | 静态行走 |
| `TrotRun()` | 小跑 |
| `SpeedLevel(level: int)` | 速度档位（如 -1 慢 / 0 普通 / 1 快） |
| `FreeWalk()` | 自由行走 |
| `FreeBound(flag: bool)` | 自由弹跳 |
| `FreeJump(flag: bool)` | 自由跳跃 |
| `WalkUpright(flag: bool)` | 直立行走 |
| `CrossStep(flag: bool)` | 交叉步 |

### 1.3 其他

| 方法 | 说明 |
|------|------|
| `Sit()` | 坐下 |
| `RiseSit()` | 从坐姿站起 |
| `SwitchJoystick(on: bool)` | 切换摇杆控制权 |
| `Pose(flag: bool)` | 姿态模式 |
| `SwitchAvoidMode()` | 切换避障模式（Sport 内） |
| `AutoRecoverySet(enabled: bool)` | 自动恢复开关 |
| `AutoRecoveryGet()` | 查询自动恢复状态 |
| `Hello()` / `Stretch()` / `Content()` | 打招呼、伸懒腰、满足 |
| `Dance1()` / `Dance2()` | 舞蹈 |
| `Scrape()` / `FrontFlip()` / `FrontJump()` / `FrontPounce()` | 扒地、前空翻、前跳、前扑 |
| `Heart()` / `LeftFlip()` / `BackFlip()` / `HandStand()` | 比心、左翻、后空翻、倒立 |
| `FreeAvoid(flag: bool)` | 自由避障开关 |

**PoseAdapter 与经典步态**：若希望由代码切到经典步态（稀碎步），可在 SportClient 初始化并 BalanceStand 后调用 `sport_client.ClassicWalk(True)`。

---

## 2. ObstaclesAvoidClient（避障开关）

**模块路径**：`unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client`  
**用途**：开关机身避障。PoseAdapter 运动控制需在**关闭避障**下使用。

| 方法 | 说明 |
|------|------|
| `SwitchSet(on: bool)` | 设置避障开关，`False` 关闭避障 |
| `SwitchGet()` | 查询当前避障开关状态 |
| `Move(vx, vy, vyaw)` | 带避障的速度控制（一般不用） |
| `MoveToAbsolutePosition(...)` / `MoveToIncrementPosition(...)` | 绝对/增量位姿移动 |
| `UseRemoteCommandFromApi(isRemoteCommandsFromApi: bool)` | 是否使用 API 下发的遥控指令 |

**示例**：关闭避障  
```python
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
c = ObstaclesAvoidClient()
c.Init()
c.SwitchSet(False)  # 关闭避障
```

---

## 3. 其他 Go2 模块

| 模块 | 路径 | 说明 |
|------|------|------|
| **VideoClient** | `unitree_sdk2py.go2.video.video_client` | 前置相机取流（PoseAdapter 可选） |
| **RobotStateClient** | `unitree_sdk2py.go2.robot_state.robot_state_client` | 机器人状态查询（如服务开关、版本等） |
| **VuiClient** | `unitree_sdk2py.go2.vui.vui_client` | 灯光、音量等 VUI 控制 |

---

## 4. 通用与 DDS

| 模块/用法 | 说明 |
|-----------|------|
| `unitree_sdk2py.core.channel.ChannelFactoryInitialize(0, "eth0")` | 初始化 DDS，第二个参数为网口名 |
| `unitree_sdk2py.core.channel.ChannelSubscriber(topic, type)` | 订阅 DDS 话题 |
| `unitree_sdk2py.idl.default.unitree_go_msg_dds__SportModeState_` | 运动状态消息类型，话题常为 `rt/SportModeState` |

---

## 5. 官方文档与示例

- 运动服务：https://support.unitree.com/home/zh/developer/sports_services  
- 避障：https://support.unitree.com/home/en/developer/ObstaclesAvoidClient  
- 快速开始：https://support.unitree.com/home/zh/developer/Quick_start  
- 示例：SDK 仓库 `example/` 下如 `go2`、`obstacles_avoid`、`wireless_controller`、`vui_client` 等。
