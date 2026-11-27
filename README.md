# Prototype2027-Public-KillerKing

Killer King 的 Driver Station 手动程序为蓝方 `Blue` 和红方 `Red`（`Main` 分组），调参与测试程序在 `Quest` 分组。项目基于 FIRST FTC SDK v12.0，TeamCode 使用 FTCLib 2.1.1 的 CommandOpMode、CommandBase 和 SubsystemBase。

## Gamepad 1

| 操作 | 功能 |
| --- | --- |
| 左摇杆 / 右摇杆 X | 场地坐标驾驶平移 / 自转。启动时驾驶前方是远离本方联盟墙的方向（红方 +X，蓝方 −X），按 Pinpoint 初始朝向换算成相对车身的方向；从自动的停车点开始时，红蓝都是车头朝前 |
| 方块 / X | 将当前车头方向设为驾驶前方，只重置驾驶用的 Control Hub IMU，不改变 Pinpoint 定位 |
| ↑ / ↓ | 手动微调 Intake 高度，每按一次 ±0.1（平时不用动，INIT 时就在固定高度 0.56） |
| ← / → | 飞轮目标转速减 / 加 25 RPM，距离模式下调整插值结果的偏移。手柄 2 的 ← / → 功能相同（见下面 Gamepad 2） |
| RT（右扳机） | 按下打开球挡，等待 100 ms 后送料（Intake 1.0，通道 0.8），松开停止发射送料 |
| LT（左扳机） | 按下关闭球挡，按住吸球；Intake 高度不变（不再有抬起 / 放下） |
| LB（左肩键） | 按住时反转 Intake（1.0）和通道（0.8） |
| 圆圈 / B | 手柄 1 和手柄 2 的圆圈 / B 都可以，功能相同。强制用 Limelight 校准：1 s 内的下一帧有效画面直接覆盖 Pinpoint 的 X、Y，不等阈值和多帧确认；之后在 2 s 内取 5 帧车和云台都静止时的画面，平均出车的朝向，一起校准朝向和 X、Y。按下后保持车不动 |
| 三角 / Y | 自瞄默认开启，按一下关闭并回中，再按开启 |
| 按下右摇杆 | Flower 铲在抬起 / 放下之间切换，INIT 时抬起 |

## Gamepad 2（比赛手动程序 `Red`、`Blue`、`Red After 3Tips`、`Blue After 3Tips`）

| 操作 | 功能 |
| --- | --- |
| ← / → | 和手柄 1 的 ← / → 相同：飞轮目标转速减 / 加 25 RPM。距离模式下是在插值结果上叠加偏移，两个手柄按的累加在同一个偏移上，范围 0–6000 RPM。遥测 `Shooter RPM 微调` 显示当前偏移，每次启动从 0 开始 |
| 圆圈 / B | 和手柄 1 相同，强制 Limelight 校准 |

两个 Limelight Tuning 程序的手柄 2 ← / → 仍是选增益（见下面调参部分）。

按下 Driver Station 的 Start 后，`Blue` 和 `Red` 根据到当前 Hive 目标点的距离自动设置飞轮转速。Intake 电机固定 1.0（`IntakeConstants.INTAKE_MOTOR_POWER`）。通道电机分手动和自动两个值：手动程序（吸球、射球、反转）和自动吸球用 0.8（`FEED_MOTOR_POWER`），两个稳定版自动射球送料时用 0.6（`AUTO_FIRE_FEED_MOTOR_POWER`）。手动程序按住 RT 射球时，Intake 用 0.6（`FIRE_INTAKE_MOTOR_POWER`），通道用 0.8（`FIRE_FEED_MOTOR_POWER`）；吸球和反转还是 1.0 / 0.8。电机优先级依次为 RT、LT、LB。按住 RT 射球时，飞轮转速到目标 70 RPM 以内（最多等 0.3 s）才开始送球，开始后一直送到松开 RT，中途不停（见下面"飞轮控制"）。RT 和 LT 同时按下时优先开挡射击，松开 RT 后仍按住 LT 会关闭球挡。松开按键后，球挡保持上一次位置。

按住 RT 时开挡并送料，球挡打开后等待 100 ms。云台回中和跨越行程缺口时也按此方式发射。

Pinpoint 首次进入 READY 时写入起始位姿。`Red` 和 `Blue` 的起始位姿就是两翻自动（`Red2Tips1.0` / `Blue2Tips1.0`）的停车终点：红方 (13.20, 95.77) in、0.13°，蓝方 (130.80, 48.23) in、180.13°（`PinpointConstants.RED_TELEOP_START_POSE` / `BLUE_TELEOP_START_POSE`）。换算方法：Pedro 的场地中心是 70.75 in，Pinpoint 是 72 in，所以 Pedro 坐标的 X、Y 各加 1.25 in；两套坐标的朝向定义相同，不用换算。停车点的朝向是 07 停车路径终点的反切线方向。改了停车点要同步改这两个常量。`Red After 3Tips` 是三翻 2.0 之后用的红方手动程序，其余和 `Red` 完全一样，只是起始位姿不同：优先用 `Red3Tips2.0` 交来的实测终点（`AutoEndPose`，2 分钟内有效）；没有就用预设终点 (10.75, 89.75) in、270°（`RED_AFTER_3TIPS_START_POSE`，即 Pedro 的 (9.5, 88.5) 加 1.25 in）。`Blue After 3Tips` 是 `Blue3Tips1.0` 之后用的蓝方手动程序，做法相同：优先用 `Blue3Tips1.0` 交来的实测终点，没有就用预设终点 (133.36, 42.58) in、270°（`BLUE_AFTER_3TIPS_START_POSE`，见下面蓝方部分）。INIT 时遥测会显示 `Pinpoint 初始位姿`，两个 After 3Tips 还会显示位姿来源。不跑自动、直接开手动时，要把车摆到这个位置和朝向。两个 Limelight Tuning 和测试程序仍从场地中心 (72, 72) in、180° 开始（`START_POSE`）。Intake 朝红方联盟墙对应 180°。正式开始前云台处于中位 0.50，朝车右（逆时针为正时记为 -90°）。START 后取得有效位姿即开始自瞄。蓝方自瞄目标的 X 为 84 in，红方为 60 in。机器人 Y < 72 in 时选目标 Y = 60 in，其余位置选 Y = 84 in。舵机位置减小时云台逆时针转动，总行程为 320°，相对中位可偏转 ±160°。目标落在 ±160° 之外的 40° 死区时，两块灯常亮红色，手柄 1 持续震动，目标回到行程内就停止；关闭自瞄或没有有效位姿时不提示。定位无有效位姿时，自瞄保持当前舵机位置。

## 硬件配置

| 配置名 | 机构 |
| --- | --- |
| `0` / `1` / `2` / `3` | 左前 / 左后 / 右前 / 右后底盘电机，左侧正向、右侧反向 |
| `00` / `01` | 带编码器的飞轮电机（FORWARD，直接读取编码器速度） / 另一侧飞轮电机 |
| `03` / `02` | Intake 电机 / 通道电机（反向；手动和自动吸球 0.8，自动射球 0.6） |
| `s0` / `s00` | Intake 抬升舵机，所有程序都固定在一个高度 0.56 / 0.44（`IntakeConstants.LIFT_POSITION`，数值越大越高），不再分抬起位和吸球位；手动程序可用 D-pad 微调 |
| `s2` / `s01` | 云台舵机，同向控制 |
| `s5` / `s05` | goBILDA RGB 指示灯，在 Robot Configuration 中配置为 Servo |
| `s1` | 球挡舵机，关闭 0.55，打开 0.80 |
| `s3` / `s03` | Flower 铲舵机，镜像控制，抬起 0.90 / 0.10，放下 0.29 / 0.71，INIT 时抬起 |
| `imu` | Hub Logo 朝右，USB 口朝后 |
| `od` | goBILDA Pinpoint，偏移 (-48, -125) mm，X、Y 编码器均为 FORWARD |
| `limelight` | 可选 Limelight 3A，供 HIVE AprilTag 定位 |

## Pinpoint 方向测试

Driver Station 的 `Pinpoint Direction Test` 以启动时的车头对齐驾驶前方。Intake 朝红方联盟墙启动后，按手柄 1 的 A 标记测量起点，用左摇杆一次只沿一个方向平移。站在蓝方面朝场内时，左移预期场地 ΔY 为负，后移预期场地 ΔX 为正。遥测同时显示场地坐标增量、标记时车体坐标下的前向和左向增量，以及航向变化。

## 横移偏转补偿

Killer King 全速横移时前轮推得比后轮多，起步阶段车头会转约 30°，几乎以右后轮为轴。`DriveConstants.frontStrafeScale` 把每次横移指令中前轮那一份乘以这个系数（< 1 时前轮少推），前进和转向不受影响。手动程序的 `Drive` 和自动用的 Pedro（`pedro/CompensatedMecanum`，AutoTune 的 Foresight 也用它）共用这个值。默认 1.0，即不补偿。

确定系数：在 `Test` 分组运行 `Strafe Compensation Test`，左右两侧都要留出空地。按住手柄 1 的十字键 ← / → 以车身方向全速横移，松开即停；LB / RB 把系数减 / 加 0.02。每次横移结束后，遥测会显示车头转了多少度（+ 为向左转）。找到左右两个方向都接近 0° 的系数，写进 `DriveConstants`。测试结束后系数会恢复为代码里的值。改过系数后要重跑 Foresight，因为它的参数是在这个补偿下测出来的；以后修好机械问题，要把系数改回 1.0，再重跑 Foresight。

## Limelight 视觉定位

先在 Limelight 网页端给下面两个 pipeline 分别上传地图。选择对应的 OpMode 后点 INIT，程序会读取相机当前的 pipeline，并在需要时发送切换请求。看到 `Pipeline P0 ready` 或 `Pipeline P1 ready` 表示相机已经切到对应编号。按 START 后，程序只读取相机结果。若编号与程序不符，遥测会显示 `Wrong pipeline`，程序会忽略该帧。

| 地图文件 | Limelight pipeline | 视觉测试 | 手动驾驶 |
| --- | --- | --- | --- |
| `BIOBUZZ_HiveFlat_Blue.fmap` | 0 | `Vision Fusion Command Test - Blue` | `Blue` |
| `BIOBUZZ_HiveFlat_Red.fmap` | 1 | `Vision Fusion Command Test - Red` | `Red` |

两张地图都在 `TeamCode/src/main/assets/vision/`。按 Killer King 实车测量值填写两套 pipeline 的相机 Forward、Right、Up 和 Roll、Pitch、Yaw。以云台转轴在地面的投影为原点。将云台置于 0.5（发射口朝车右）后，按车头作为 +X 标定相机外参。

在 `VisionLocalizationConstants` 填写转轴相对底盘中心的前向和左向偏移，初值均为 0。视觉校正使用曝光时的云台估算角。云台转动过快或估算角明显落后指令角时会拒绝该帧。可在 `TurretConstants` 按实车带载行程调整估算速度，初值为 300°/s。

`Vision Fusion Command Test` 的蓝、红版本并列显示 Pinpoint 和 Limelight 返回的原始 MT1 位姿。两组位置都显示为英寸，MT1 保留 fmap 的原点和轴向。遥测标出上一帧的年龄。手柄 1 左摇杆平移，右摇杆左右拨动控制底盘自转。

红蓝 Limelight 版以 Pinpoint 为主要位姿来源。Limelight 与当前 Pinpoint 的平面位置相差至少 0.30 m，且至少三张有效画面在扣除里程计运动后相互一致，并持续至少 0.60 s，程序才校准一次。每帧用视觉位置减去拍摄时刻的 Pinpoint 位置，得到要修正的平移量；确认后直接把修正后的位置写进 Pinpoint 设备，航向不变。确认期间，各帧平移量与首帧的差须在 0.12 m 以内。校准只更新 X、Y，车头航向始终取 Pinpoint。相邻有效帧间隔超过 0.20 s 时从头确认。

手动程序中按手柄 1 的圆圈 / B 可强制校准：之后 1 s 内的第一帧有效画面（同样要通过 HIVE 几何、场地边界和云台转速检查）直接把 Pinpoint 的 X、Y 改成 Limelight 的结果，不看 0.30 m 阈值，也不等多帧确认。1 s 内没有有效画面时这次请求作废。

X、Y 改完后接着校准航向。Limelight 算出的车身航向 = 看到的相机朝向 − 云台转角 − 云台回中时相机相对车头的朝向（发射口朝右，即 −90°，`TURRET_CENTER_CAMERA_YAW_DEGREES`）；云台舵机没有反馈，转角是按指令估算的，所以只用车和云台都静止时的画面：云台估算已到达指令，且之前 0.25 s 内云台变化不超过 0.5°、车身转动不超过 1°。凑够 5 帧后取平均，5 帧相差都不超过 2° 才采用，否则重新收集；从按键起 2 s 内凑不齐就只保留 X、Y 的校准。校准时直接改写 Pinpoint 设备本身（`setPosition`）：把车当前的位置和航向换成校准后的值写进去，之后 Pinpoint 自己的读数就是校准后的值，按新的航向继续累加位移，不会越走越偏；程序里不再另外保存修正量。每次改写后清空视觉的历史记录，改写前拍的帧会被丢掉。自动触发的校准仍然只改 X、Y，保留已有的航向校准量。

遥测 `Forced alignment (circle / B)` 依次显示 `Waiting for valid Limelight frame`、`X/Y applied; measuring heading (hold still)`、`Applied X/Y and heading (+x.x°)`（括号里是这次航向改了多少），失败时显示 `Timed out: no valid Limelight frame` 或 `X/Y applied; heading timed out ...`。`Pinpoint rewrites by vision` 显示这次运行中视觉改写了 Pinpoint 几次（按一次圆圈通常是 2 次：先 X/Y，再航向）；每次航向校准也会写进 RobotLog（`KillerKingVision` 标签）。

场地原点是站在红方人站面朝场内时右手边的场地角。X 朝蓝方递增，Y 朝计分侧递增。Limelight 的中心坐标顺时针旋转 90°，再平移 (+72, +72) in。以英寸表示就是 X = 72 + 地图 Y，Y = 72 − 地图 X，航向同时减去 90°。这个 −90° 是两个场地坐标系之间的旋转（地图 0° 指向观众一侧，Pinpoint 0° 指向蓝方），位置和航向一起转；用 Limelight 地图核对过，只有 −90° 能让红方标签落在红方 HIVE 的两个 CELL 上（30–33 在远离观众一侧）。另外，Limelight 把云台当作“车”，报的是相机朝向：云台回中时相机朝车右，所以车身航向还要再加 90°（减去 −90°），只加在航向上，Limelight 网页里的相机朝向保持 0。没加这一步时，视觉航向比 Pinpoint 少 90°，0° 指向计分侧（裁判一侧）。遥测 `Limelight field X/Y/Heading` 显示拍摄时的视觉位姿，其中航向是根据云台角换算的视觉估计值，仅用于对照。`X/Y/Heading` 显示自瞄使用的当前 Pinpoint 位姿。`Last vision discrepancy` 显示最近的位置差和确认帧数。

丢失 Tag 时，Pinpoint 从上一次视觉校准继续积分。首次校准前，Pinpoint 从预设起点积分。里程计掉读时，云台保持当前位置，直到再次取得位姿。

`Blue`（蓝方 P0）、`Red`（红方 P1）和两方的 Tuning 在运行期间持续用有效视觉帧校正 Pinpoint。丢 Tag 后保留已确认的位置偏移，每轮继续用 Pinpoint 更新位置和车头航向。若 Pinpoint 本身不可用，程序可用 200 ms 内的有效视觉位姿临时瞄准。两者都不可用时保持云台角度。Pinpoint 起点只用于初始化里程计，Limelight 根据 Tag 地图解算场地上的绝对位置。云台自瞄开关独立于这一过程。

## Limelight 飞轮调参

在 Driver Station 的 `Quest` 分组选择蓝方 `Ba Limelight Tuning` 或红方 `Ba Red Limelight Tuning`。启动后默认使用视觉校正和固定目标 3375 RPM 的飞轮 PIDF，手柄 2 的 X 可试用距离插值。调参页显示目标及实测 RPM、飞轮实际输出、手动设定功率、增益、送料和球挡状态。每次启动从代码默认值开始。

`到目标 Hive` 显示自瞄位姿计算的平面直线距离，单位为厘米。两块灯用 Pinpoint 的读数计算到当前联盟 Hive 目标点的距离，90–160 cm（含两端）亮绿，区间外或 Pinpoint 不可用时亮红。视觉校准直接改写 Pinpoint 设备，所以灯和自瞄用的是同一个校准后的位置，校准的那一刻灯的距离会跟着跳。云台目标在死区时，不论距离，灯都常亮红色。初始化和停止时熄灯。距离区间在 `HiveLightsConstants` 中配置。

距离模式在 104.0 cm / 3275 RPM、118.4 cm / 3350 RPM、138.4 cm / 3400 RPM 和 170.0 cm / 3600 RPM 之间分段线性插值（2026-11-27 在 KP 0.0014 / KI 0.0007 / KF 0.000174 下实测，当时飞轮稳态能到目标。同一天改成 KI 0 后每个点加过 50 RPM，那只是在补 KI 0 加 KF 0.000145 时飞轮稳态低 80–100 RPM 的差；现在的 PIDF（KF 0.00018）在 12.8 V 左右能到目标，表恢复成实测值，要试射核对；旧增益下的表是 109.4 cm / 3177、125.2 cm / 3277、143.0 cm / 3427）。超出标定范围时使用对应的端点转速。位姿不可用时使用固定转速设定，再叠加手柄设定的偏移。

| 操作 | 功能 |
| --- | --- |
| 手柄 2 ← / → | 循环选择 P、I、D、F |
| 手柄 2 ↑ / ↓ | 增 / 减所选增益，P 步长 0.00010，I 为 0.00001，F 为 0.000002，D 为 0.000001 |
| 手柄 2 A | 切换 PIDF / 手动飞轮功率 |
| 手柄 2 X | 切换固定目标 RPM / 按距离插值 |
| 手柄 2 LB / RB | 减 / 加手动飞轮功率 0.01，范围 0 到 1 |
| 手柄 1 ← / → | 减 / 加目标转速 25 RPM |

## 飞轮控制

飞轮是软件 PIDF：两个飞轮电机都是 `RUN_WITHOUT_ENCODER`，`Shooter.updateControl()` 每个循环读一次电机 `00` 的编码器速度，滤波后算出功率，用 `setPower()` 发给两个电机。没有用 Hub 内置的速度控制。

测到的转速先经过一阶低通滤波，P 和 D 用的都是滤波后的值，自动判断"飞轮到速"也用它。滤波按时间常数算：每一圈让滤波值向原始读数走 dt ÷ (0.03 s + dt)（`FILTER_TIME_CONSTANT_SECONDS`），所以不管一圈多长，滞后都是 30 ms。原来是每圈固定走 0.4：调参工具 20 ms 一圈时正好也是 30 ms，增益就是在这个条件下调的；但自动里一圈 30–40 ms，滞后就变成了 45–60 ms。每次 `startControl()` 之后的第一次读数直接作为滤波值，不从停下前的旧值慢慢追上来。按 2026-11-27 的模拟：滤波能让遥测上的转速抖动从 ±19 降到 ±9 RPM，但飞轮真实转速的抖动不管滤不滤都只有 4–6 RPM（飞轮 1.12 s 的惯性本身就把功率抖动滤掉了）。时间常数到 60–100 ms 以后，连射时每颗球的掉速发现得太晚，同一轮 4 颗球之间的转速差明显变大，所以不要调大。

飞轮只用 PIDF，没有别的控制（2026-11-27 起：去掉了电压补偿、KS 常数项和门控积分）：输出 = KF × 目标 RPM + KP × 误差 + KI × 误差积分 + KD × 转速变化率，限制在 −0.1 到 1.0。积分只在输出还没到上下限时增长。当前 KP 0.0014 / KI 0 / KD 0 / KF 0.00018。

- **KF 0.00018**：调参工具实测飞轮是"稳态 RPM = 6404 × 功率 − 378"（13.3 V 左右），KF 0.000174 在那个电压下正好覆盖 3275–3600 RPM；0.00018 是同样的转速在 12.8 V（比赛电池常见的电压）下要的值。因为没有电压补偿，电池电压会让整轮转速一起平移：按模拟，13.3 V 时约高 15 RPM，12.2 V 时约低 19 RPM，同一轮 4 颗之间不受影响。电池低的时候可以用 ← / → 往上补。
- **KP 0.0014**：调参工具里掉速恢复最好的值。按 2026-11-27 的连射模拟，0.001 恢复太慢，后几颗球的转速取决于球间隔，间隔一变就有高有低；0.002 以上会让第 2–4 颗比第 1 颗高 10–25 RPM。
- **KI 0**：每颗球掉速时积分都会往上涨，转速回来后冲过头，同一轮一颗比一颗高。模拟里 KI 0.0003 / 0.0007 让第 4 颗比第 1 颗高 15 / 30 RPM。KI 能修掉电池造成的平移，但代价是这个。
- **KD 0**：只会放大 ±18 RPM 的测速噪声。
- 遥测 `Shooter 积分` 显示积分项的功率（KI 为 0 时一直是 0）。手动功率、开环功率和调参工具的刹车功率直接发给电机。

**开始送球的条件**（只在手动程序里，`FeedCommand`）：按住 RT 开挡后，滤波转速离目标在 70 RPM 以内（`FEED_START_RPM`，`Shooter.isReadyToFeed()`）才开始转 Intake 和通道，最多等 0.3 s（`FEED_MAX_WAIT_SECONDS`），到时间不管转速照样送，飞轮到不了目标或编码器坏了时也能射。一旦开始送，就一直送到松开 RT，中途不会因为转速停下（2026-11-27 改；之前送的过程中转速差超过 80、后来 120 RPM 会暂停，射两颗停一下，删掉了）。自动程序的送料没有改（开挡前等转速和云台，开挡后固定送 `VOLLEY_MS`）。

**射球记录**（只用来分析，不影响控制）：滤波转速在 0.15 s 内比最高点掉了 60 RPM 以上、并且低于目标 30 RPM 以上，就算打出一颗球；转速从最低点回升 25 RPM 算这颗结束。相隔 0.8 s 以内的算同一轮。遥测 `上一轮出手 误差/掉速 RPM` 显示上一轮每颗球出手前的转速误差（出手前最高的滤波转速 − 目标）和这颗带走的转速，`上一轮出手 球间隔 s` 显示球与球的间隔。每颗球也写进 RobotLog（标签 `Shooter`，自动程序里也有），包括目标、出手前转速、掉速、间隔、电池电压（只在记录时读一次，不参与控制）和积分。如果同一轮几颗球的误差都在 ±20 RPM 以内，但落点还是有高有低，问题就不在飞轮转速，要查球本身、送料或出球结构。

距离插值表是在飞轮能到目标时实测的（见上面）。各自动的射击转速是更早的增益下调的，要试射核对。

## 飞轮自动调参 `Shooter Auto Tuner`（Test 分组）

`tests/ShooterAutoTuner.java` 自动测出飞轮特性并推荐 `ShooterConstants` 的 KP、KF（KI、KD 固定为 0）。只转飞轮，车、Intake、球挡都不动，开始前要取出所有球。大约 2 分钟，随时按 STOP 停止。

INIT 时用手柄 1 设置：十字键 ↑ / ↓ 调循环周期（每次 5 ms，0 为不限速），← / → 调测试转速（每次 100 RPM，默认 3375）。循环周期要设成手动程序的实际循环时间，因为同一组增益在 10 ms 和 40 ms 一圈时表现不同。`Blue`、`Red` 和两个 Tuning 的遥测第一行 `循环时间` 显示运行时的平均循环时间和最近 2 s 内最长的一圈。

分三步：

1. 开环：依次用 0.35–0.85 的固定功率转飞轮，每档等稳定后采样，拟合每单位功率对应的 RPM，算出测试转速需要的功率和 KF；同时从每次换档的阶跃响应拟合时间常数，并统计测速噪声。KF 是测试时电池电压下的值，结果里也会给出按 12.8 V（比赛电池常见的电压，`MATCH_VOLTS`）换算的 KF，供参考。
2. KP：先跑 `ShooterConstants` 里的当前增益作为对照，再用 8 个 KP（KI = KD = 0）逐个测试。每组先测稳态转速抖动，再做 3 次掉速测试：以 `MIN_POWER` 刹车，直到掉 150 RPM（`DIP_RPM`，模拟一颗球带走的能量），然后放开，统计放开后 0.15–0.40 s（下一颗球大约到达的时间）的平均转速误差。得分 = 这个误差 + 稳态抖动，越低越好；过冲超过 40 RPM 的不选。
3. KI：用最好的 KP，从慢到快试 3 个 KI，结果只供参考，推荐值始终是 KI = 0。

掉速的深度从这组增益自己的稳态转速算起，只把掉速过程按 150 RPM 折算；KF 不准造成的稳态偏差按原值计入误差和过冲。

结束后屏幕显示推荐值、测试条件（转速、循环时间、电池电压），以及当前常数和推荐值的误差、抖动、过冲对比；全部结果也写进 RobotLog（标签 `ShooterAutoTuner`）。推荐值要手动复制进 `ShooterConstants`。

2026-11-27 第一次实测（电池 13.42 → 13.09 V，循环 20 ms，3375 RPM）：每单位功率 6404 RPM，3375 RPM 需要 0.586 功率，时间常数 1.12 s，恒定功率下测速噪声 ±18 RPM（遥测上看到的转速抖动大多是这个噪声）。KP 在 0.0014–0.0028 之间得分差不多，任何增益在掉 150 RPM 后 0.15–0.40 s 都还差约 30 RPM，所以连续送球时后面几颗球的转速靠调 PID 解决不了。按这次结果，`ShooterConstants` 从 KP 0.001 / KI 0 / KD 0.0001 / KF 0.0002 改成 KP 0.0014 / KI 0.0007 / KD 0 / KF 0.000174（程序当时选的 KI 0.00281 得分和过冲都不如 0.0007）；同一天后来 KI 改成 0、加过电压补偿，又试过 KS + KV 前馈和门控积分，最后改回纯 PIDF：KP 0.0014 / KI 0 / KD 0 / KF 0.00018（见上面“飞轮控制”）。旧 KF 在满电时会让飞轮比目标高约 70 RPM。距离插值表已在新增益下重新实测；各自动的射击转速还是旧增益下调的，要重新试射。

每一圈的数据（时间、阶段、功率、原始 / 滤波后 RPM、目标 RPM）存在主控的 `/sdcard/FIRST/ShooterAutoTuner/tuner-日期-时间.csv`，以 `#` 开头的行是测试条件、各组结果和最后的报告，中途按 STOP 也会保存已经测完的部分。电脑用 USB 连 Control Hub（或连上机器人 Wi-Fi 后 `adb connect 192.168.43.1:5555`），再用 `adb pull /sdcard/FIRST/ShooterAutoTuner/` 取出来，可以离线重新分析。掉速测试是用刹车模拟的，真实射球时掉多少转速，还要实际射球确认。

## 自动 `Red3Tips1.0`（Auto 分组，稳定版）

`Red3Tips1.0`（`opmodes/autos/Red3TipsAuto.java`）是稳定版：只走固定路线，不用 Limelight，不分支，也不做时间截止。每段路径和每轮射击的开始时间都写进 RobotLog（`Red3Tips1.0` 标签），用来实测耗时。带视觉的试验版见下文 `Red3Tips1.0 Vision (test)`，它暂时还用旧路线（`Red3TipsVisionRoute.java`），稳定版换路线不影响它。

自动使用 Pedro Pathing 3.0.2（`com.pedropathing:revhub` / `tuning`）。首次使用前运行 AutoTune（电脑连上 Control Hub 的 Wi-Fi，浏览器打开 http://192.168.43.1:10158），依次完成 Mecanum、Pinpoint、Foresight 三个流程，把导出的配置粘贴到 `pedro/Constants.java`。`foresightConfig` 为空时，自动只提示调参，不会驱动底盘。Mecanum 和 Pinpoint 的结果（2026-11-26）与现有配置一致：轮子为 goBILDA 4-bar，两个编码器均为 FORWARD；偏移保留实测的 (-48, -125) mm，没有采用 AutoTune 原地旋转估算的 (-68.5, -143.1) mm。Foresight 同一天跑了两次，`Constants` 里是第二次的结果：最高前进 83.7 in/s、最高横移 58.5 in/s（第一次横移时车头会转约 30°，只测到 21.8 in/s，已作废）。底盘有改动或者改了 `frontStrafeScale`，都要重跑 Foresight。

路线在 `opmodes/autos/Red3Tips_1_0.java`，是 2026-11-26 第三次 Pedro Visualizer 导出的原样（只改了类名，并把起点设为 public），起点 (54.37, 9.44) in、270°，车头为 Intake 并贴住观众侧墙。车身按实测 479.5 × 348.2 mm（18.88 × 13.71 in）计算。Garden 点 (6.96, 9.97) 让 Intake 停在离观众侧墙 0.53 in、车侧离红方墙 0.11 in 处，不顶墙。Far flower、Side flower 两个点比车头刚好贴墙的位置再往里 0.40–0.46 in，让 Intake 压进去；Far flower 后来又往里加了 5 cm（y 从 132.52 改到 134.49，两翻同样；曾经铲子到位但球没下来），现在比贴墙位置往里约 2.4 in，车到不了，靠顶住停下算到达，Pedro 会更用力往里压；车顶住停下、离终点不到 1.5 in 且 200 ms 内移动不到 0.2 in 时就算到达，开始计停留时间，Pedro 在停留期间继续往里压（否则 Pedro 可能一直认为没走完，要等 5 s 路径超时）。02 全程朝 270°（倒着开到远端射击点）；03 在导出里的起始朝向是 70°，而 02 到达时是 270°，所以 03 一开始车会先快速转向 70°，再转到 90°。05 去队友预装球时从 90° 转到 270°，06 从 270° 转到 180° 到达近端射击点，07、08 保持 180°，09 停靠时转到 90°。03 和 07 的最后 12 in 限速 35 in/s（原来 20 in/s，Far flower 不稳定，提高了；原来是最后 10 in，Far flower 点加深 5 cm 后改成 12 in，让 Flower 前同一段路仍然是慢速；红蓝四个稳定版相同），让 Intake 慢慢进 Flower：程序在离终点 12 in 处把曲线切成两段（形状不变），后一段带 Pedro 的 `maxVelocityConstraint`，只在这一段生效，走完自动恢复（`FLOWER_APPROACH_INCHES` / `FLOWER_APPROACH_SPEED_IN_PER_S`）。到 Flower 后最多停 1.5 s（`FLOWER_MAX_MS`），然后必须进行下一步，不管 Pedro 有没有走完：从下面三种情况里最先出现的那一刻开始计时——车中心离 Flower 点不到 5 in（Flower 从墙面伸出约 3.8 in，Intake 大约在离终点 4 in 时碰到它，`FLOWER_CONTACT_INCHES`）、Pedro 走完路径、或者在离终点 14 in 以内卡住不动（`FLOWER_STALL_INCHES`；200 ms 内移动不到 0.2 in，从停下的那一刻算起）。这样 Intake 卡在 Flower 上停住时，不会再等 5 s 的路径超时。正常进到底时，从碰到 Flower 算起停留 1.5 s。流程：起点射预装球 → Garden 吸球 → 远端射击 → Far flower 吸球 → 远端射击 → 队友预装球 → 近端射击 → Side flower 吸球 → 近端射击 → 停靠。吸球时关闭球挡，Intake 以 1.0、通道以 0.8 功率运行，吸完停转；Intake 高度全程固定在 0.56，不抬也不放（和手动程序按住 LT 一样）。Garden 和队友预装球两处改成：到点后吸 0.5 s，沿车身正后方后退 5 cm，再开回停下的原位吸 0.5 s（`PICKUP_DWELL_MS` / `PICKUP_BACK_OFF_INCHES`）。车在离吸球点 4 in 以内停住（200 ms 内移动不到 0.2 in）就算到达（`PICKUP_STALL_INCHES`），不用等 5 s 的路径超时（有一次 Garden 被挡在 1.5 in 以外，白等了 5 s）；路径超时的日志会写出车离终点还有多远，整个过程 Intake 一直在转；直接吸常常吸不上。后退和回来都按车停下时的实际位置现算路径。Garden 吸完后，Intake 和通道不停，在 02 开走的头 1 s 里继续转（`GARDEN_EXTRA_INTAKE_MS`，球挡关着），把还在 Intake 口的球吸进去，4 个球更稳；Garden 这一轮的飞轮转速提前到吸球前设定，所以吸球和这 1 s 之间 Intake 不会停顿。队友预装球那段不加这 1 s。两段吸 Flower 的路线（Far flower、Side flower）在整段吸球期间放下 Flower 铲，吸完后先不收：开往射击点时，车离开 Flower 6 in（`FLOWER_SCOOP_CLEAR_INCHES`，从离开 Flower 时车的位置算）才抬起；到射击点还没够 6 in 就在到达时抬起。原来吸完立即抬起，铲子会卡住 Flower。Far flower 例外：铲子在 04 开往射击点时保持放下，04 射球开挡的那一刻收起，一边射一边收（离开 6 in 就收、射完 0.5 s 再收都试过，前者会卡住）。两翻的下一步 05 一开始又放下铲子去近端 Flower。04 全程 Intake 都在转，接上射击前的预转。从 Flower 退出来时 Intake 和通道不停（球挡关着），一直转到铲子抬起才停；如果这时射击前的预转已经开始，就继续转下去。程序中途停止时也会抬起铲子。试验版仍是吸完立即抬起（被时间截止或看门狗中断时也会抬起）。Garden 和队友预装球两段不放铲；射击时等待飞轮转速和云台到位（最长 1.5 s；起点这一轮飞轮从静止起转，最长 3 s），每轮射击前，球挡关着让 Intake（1.0）和通道（0.8）先转至少 1 s（`PRE_FEED_MS`），把卡在 Intake 和通道之间的球顶到球挡前（Garden 吸完后曾有一个球卡在那里，远端第一轮只射出 3 个）；开往射击点时离终点还剩 12 in 就开始转（`PRE_FEED_START_INCHES`），这 1 s 大部分和开车、等飞轮重叠，再早开可能在路上吸到散落的球（G407 最多 4 个）。转满 1 s 且飞轮、云台到位后，电机停、打开球挡，100 ms 后送料 1200 ms，送料时通道电机 0.6（手动是 0.8）。送料时间原来是 0.8 功率下的 900 ms，按 0.8 / 0.6 放长到 1200 ms，保证 4 个球都能送出去；实测 4 个球提前送完的话可以调短 `VOLLEY_MS`。云台使用手动程序的 `TurretAimCommand` 和红方目标，开始时直接打开自瞄（不用切换，因为它的默认值已经改成开），在接近射击点时提前瞄准，停靠前关闭并回中位。

三翻的顺序（A = (60, 60)，开局朝上，里面有 3 个 NECTAR；B = (60, 84)）：

| 射击 | 球 | 打的 CELL | 结果 |
| --- | --- | --- | --- |
| 00 起点 | 预装 4 个（进 3 个即可） | A | 第一翻，B 翻上来 |
| 02 + 04 远端 | Garden 4 个 + Far flower 4 个 | B | 第二翻，A 翻上来 |
| 06 + 08 近端 | 队友预装 4 个 + Side flower 4 个 | A | 第三翻 |

AUTO 最多拿到 20 个球，三翻要 3 + 8 + 8 = 19 个，第二翻、第三翻一个都不能漏。每次都射空再吸球，车上最多 4 个球。

| 射击点 | 位置 (in) | 车身朝向 | 目标 (in) | 云台偏转（相对中位） | 舵机位置 | 到目标距离 (in) |
| --- | --- | --- | --- | --- | --- | --- |
| 00 起点 | (54.37, 9.44) | 270° | (60, 60) | -95.1° | 0.797 | 49.5 |
| 02 远端 | (42.64, 113.28) | 270° | (60, 84) | +117.8° | 0.132 | 34.5 |
| 04 远端 | (37.90, 116.02) | 90° | (60, 84) | -57.9° | 0.681 | 39.3 |
| 06 近端 | (34.98, 37.91) | 180° | (60, 60) | -48.8° | 0.652 | 31.6 |
| 08 近端 | (33.21, 35.28) | 180° | (60, 60) | -47.4° | 0.648 | 34.7 |

以上按云台中位朝车右、±160° 行程（`SERVO_TRAVEL_DEGREES` = 320°）和红方目标 (60, 60/84) in 计算。云台角度由自瞄实时算出，不用手调；表里只是核对是否在行程内。02 改成朝 270° 后云台要转 +117.8°，离极限还有 42°，是余量最小的一个；起点 65°，其余 100° 以上。五轮射击各有一个飞轮转速（`PRELOAD_SHOT_RPM`、`GARDEN_SHOT_RPM`、`FAR_FLOWER_SHOT_RPM`、`PARTNER_SHOT_RPM`、`SIDE_FLOWER_SHOT_RPM`），起点预装球 3225（手动程序默认值 3375 减 150；3375、3350、3325 都打高了），其余暂时都是 3375 RPM，需要在各射击点实测调整。手动程序的距离插值表最近一点是 104.0 cm，而 02/04/06/08 只有 80–100 cm，表外的转速不能直接推算，所以自动不开距离插值。

## 两翻 `Red2Tips1.0`（Auto 分组）

`Red2Tips1.0`（`opmodes/autos/Red2TipsAuto.java`）是留有容错的两翻：不用 Limelight、不分支、不用队友预装球。程序从稳定版三翻复制而来，吸球、Flower 铲（Far flower 在 04 射球时收，近端 Flower 离开 6 in 后才收，收起前 Intake 一直转）、最后 12 in 慢速进 Flower、顶住停下算到达、到 Flower 后最多停 1.5 s、Garden 后退再回来吸球、Garden 吸完后 Intake 多转 1 s、射击前 Intake 和通道先转 1 s、自动射球送料 0.6 和 1200 ms、每轮单独的飞轮转速都和三翻相同；三翻本身没有改动。RobotLog 标签为 `Red2Tips1.0`。

两翻独有（三翻不变，红蓝相同）：每次吸球（Garden、Far flower、近端 Flower）结束、车开始开走的那一刻，Intake 高度从吸球高度 0.56 降到 0.5（`TRAVEL_LIFT_POSITION`），之后开往射击点和射球都在 0.5；下一次吸球开始时回到 0.56。Garden 的后退 5 cm 再回来属于吸球过程，仍是 0.56，回来吸满 0.5 s、真正开走时才降。日志会记一条 `pickup done, intake lowered to 0.50`。

路线在 `opmodes/autos/Red2Tips_1_0.java`，是 2026-11-27 Pedro Visualizer 导出的原样（改了类名，并把起点设为 public），只有两个点改过：Far flower 点往里加深 5 cm（y 132.52 → 134.49），Far flower 射击点 04 往红方联盟墙移 10 cm（x 40.85 → 36.91，05 的起点跟着改）。01 与三翻相同；02、04、06 有自己的射击点（都比三翻的更靠计分侧）；05 去近端（Side）Flower，06 带着这 4 个球回到远端射击点；导出里 06 叫 Flowerloadnear，但射击点在远端。

| 射击 | 球 | 打的 CELL | 结果 |
| --- | --- | --- | --- |
| 00 起点 | 预装 4 个（进 3 个即可） | A (60, 60) | 第一翻，B 翻上来 |
| 02 + 04 + 06 远端 | Garden 4 个 + Far flower 4 个 + 近端 Flower 4 个 | B (60, 84) | 第二翻，12 个球里进 8 个即可，最多可漏 4 个 |

| 射击点 | 位置 (in) | 车身朝向 | 云台偏转（相对中位） | 舵机位置 | 到目标距离 (in) |
| --- | --- | --- | --- | --- | --- |
| 00 起点 | (54.37, 9.44) | 270° | -95.1° | 0.797 | 49.5 |
| 02 远端 | (55.40, 123.36) | 270° | +94.7° | 0.204 | 40.7 |
| 04 远端 | (36.91, 123.08) | 90° | -61.6° | 0.692 | 45.9 |
| 06 远端 | (49.02, 122.85) | 160° | -146.4° | 0.958 | 41.3 |

06 沿路径从 180° 线性转到 160°，到点时云台要转 -146.4°，离 ±160° 极限还有约 13.6°。如果朝 180°，要转 -166.4°，会超出行程落进死区。07 停靠的朝向沿路径反切线方向，从 66° 转到 0°；06 到达时朝 160°，所以 07 一开始车会先快速转约 94°（此时云台已回中）。

四轮射击各有一个飞轮转速：`PRELOAD_SHOT_RPM`、`GARDEN_SHOT_RPM`、`FAR_FLOWER_SHOT_RPM`、`NEAR_FLOWER_SHOT_RPM`。远端三轮用 3375 RPM 时打得过远，现在按手动程序距离插值表（`DISTANCE_RPM_POINTS`）在各自距离的值取整到 25 RPM：02（103 cm）3175、04（116.5 cm，往联盟墙移 10 cm 后）3225、06（105 cm）3175。当时（旧飞轮增益）的表从 109.4 cm 起，更近时手动程序也保持 3177，所以 02、06 用的是表的最低值；表已在新增益下重测，这几个转速还没跟着改，若仍偏远就在实测中再往下调。起点预装球（126 cm）不是远端，用 3225（默认 3375 减 150）。按 Visualizer 的速度估算，最早的路线走路约 14.8 s、停留约 3.1 s、四轮射击约 7.5 s，合计约 25 s；现在的路线总路程 404 in，比最早的 373 in 多约 31 in。实车会慢一些，以 RobotLog 里记录的实测时间为准。

## 三翻 2.0 `Red3Tips2.0`（Auto 分组）

`Red3Tips2.0`（`opmodes/autos/Red3Tips2Auto.java`，路线 `Red3Tips_2_0.java`）是新的三翻，只有红方。不用 Limelight。构建块从两翻复制而来；`Red3Tips1.0` 和两翻都没有改动。RobotLog 标签为 `Red3Tips2.0`。跑完接手动程序 `Red After 3Tips`（见下）。`Red3Tips2.0.pp.json` 是完整路线，由路线代码生成，并和 Pedro 实际生成的路径逐点核对过。

路线是 2026-11-27 第一次实车之后 Visualizer 导出的版本：Visualizer 里拆开的线在代码里合回成一步，另外改了三处，见下文。场地模型：

| 项目 | Pedro 坐标（in） |
| --- | --- |
| 红方停靠区（LOADING ZONE） | x 0–11，y 94.33–117.92（从观众侧数第 5 排地垫，手册图 9-2） |
| 队友预装球 | 停靠区靠起点一侧的短边上，y ≈ 94.3，沿墙排 4 个 |
| 队友车 | 16 × 16 in，贴红方联盟墙，在预装球靠起点一侧：x 0–16，y ≈ 78–94.3 |
| HIVE 支架（红方一侧） | x 46–48.6，y 50–91.4 |

流程。车只在空车时进 Flower，避免同时控制自己的球和 Flower 的球：
1. **00**：起点射预装球（第 1 翻）。
2. **01 Garden**：到点吸 0.35 s，后退 5 cm，回来再吸 0.35 s（两翻是各 0.5 s）。
3. **02**：带着 Garden 的 4 个球开到 Far flower 下方 (46.18, 120.47)，全程朝 270°，在这里把 4 个球射出去（101 cm，3175）。开火的同时放下铲子，此时铲子朝外，不朝 Flower。
4. **03 Far flower**：原地转到 90°（差 15° 以内就走），空车开进 Flower，开着 Intake 吸，到位后停 0.75 s，然后在 Flower 里把这 4 个球射出去（108–118 cm，3225）。8 个球进远端，第 2 翻。
5. **04-05**：退出 Flower（离停下的位置 10 in 时收铲子，Intake 一直开着），离 Flower 11 in 后开始顺时针转 180°，再沿墙从上往下吸队友预装球。到点停 0.5 s，不后退再回来（`PARTNER_DWELL_MS`）。
6. **06**：退开、绕过队友的角，从 HIVE 支架右侧下去，在通道终点 (37.19, 31.26) 射队友预装球。
7. **07 Side flower**：从 06 射击点转到 180°，对准 Flower 轴线开进去吸球（开始时放下铲子）。
8. **08**：沿 Flower 轴线退出，往下到退出终点 (29.99, 36.48)，在这里射 Flower 的球，开火时收铲子。第 3 翻。
9. **09**：往上到 (17.9, 56)，转到 270° 对正（这一段限速 15 in/s），用车尾平推队友，推到两台车都进停靠区就停（车中心 y 88.5），程序结束。不再回到两翻的停车点。

| 射击 | 位置（in），朝向 | 到目标 | 转速 |
| --- | --- | --- | --- |
| 00 | (54.37, 9.44)，270° | 125.7 cm | 3225 |
| 02 Garden 的球 | (46.18, 120.47)，270° | 101.0 cm | 3175 |
| 03 Far flower 的球 | 在 Flower 里，约 (46.5, 124–127)，90° | 108–118 cm（看铲子让车停在哪） | 3225 |
| 06 通道终点 | (37.19, 31.26)，270° | 88.8 cm | 3175 |
| 08 退出终点 | (29.99, 36.48)，180° | 92.4 cm | 3175 |

转速都沿用两翻在差不多距离用的值（103.5、105 cm 用 3175，116.5 cm 用 3225），没有新调。06、08 比试过的最近距离（103.5 cm）还近，先用 3175，要试射。每次开火，日志都会写出离目标的实际距离，按它试射调整。

**相对导出路线的三处修改**：
- **04-05 的 180° 转向提前开始**：原来整个转向在 05 的第一段（24.7 in）里完成，那一段终点离联盟墙只有 8.7 in。模拟里车转不过来，车角撞进墙 1.8–2.9 in。现在离 Flower 11 in、铲子已收起后就开始转，分摊在 32 in 上。这两段限速 30 in/s（`Red3Tips_2_0.TURN_SPEED_IN_PER_S`），车只要每秒能转 170° 以上，离墙就有 0.7 in。上车仍然碰墙的话，改成 25（每秒 150° 也够，多花约 0.25 s）。
- **05 第二段的第一个控制点**：改成沿第一段终点的切线方向。导出里留的是旧拆分点的控制点，车会先往上走 1 in 再下来。
- **06 通道的前两个控制点**：y 从 111、97.90 改成 116、100。导出路线经过队友右上角时间隙为 0，现在是 2.0 in，离 HIVE 支架仍是 2.3 in。

**余量偏小的地方（没改，按导出路线）**：02 离 HIVE 支架 1.65 in（之前是 4.15 in），06 通道离 HIVE 支架 2.3 in。03 原地转身时铲子是放下的，离 Flower 尖 2.3 in；两翻的 03 是 1.8 in，实车没出问题。

**第一次实车**（2026-11-27 07:00，旧路线，有队友）：起点进 3 个，02 进 4 个，04 进 4 个，06 没到位就开火、进 0 个（已修，见下）。06 射完是 23.3 s，剩 6.7 s，跳过了 07、08；27.7 s 停回两翻终点，差 0.32 in、0.1°。队友预装球吸到 4 个，队友被推进停靠区，所有吸球和铲子都正常。

**耗时**：
- **09:00 那次实车**：33.1 s，近端 06 开了 3.7 s，08 开了 2.9 s。
- **这次缩短的时间**：
  - 队友预装球不后退再回来，约 1.0 s
  - 送料 1000 ms，5 轮共约 1.0 s
  - Garden 停留缩短，约 0.3 s
  - 两个 Flower 停留缩短，约 0.5 s
- **模拟结果**：从 25.8 s 降到 23.0 s。之后 06、08 改到正确的射击点，09 的对正段要限速 15 in/s，否则模拟里车角会撞墙（车转不过来），这一项多花约 0.65 s，模拟是 23.8 s。按 09:00 那次实车比模拟慢约 1.3 倍，估计约 30.7 s，还超一点。

**近端两轮射击（06、08）**：09:00 那次是在通道拐进来的终点 (19.66, 46.62) 和往上走的终点 (17.9, 56) 射的，一个没进，球打在 CELL 侧壁上：那两个点在 CELL 侧面，偏开口方向 73° 和 86°。按路线设计，射击点应该是 06 通道终点和 08 退出终点，那次是我理解错了。这两个点在开口前方，偏 38° 和 52°。

**没有时间门槛**：07、08 一定做（2026-11-27 定）。车慢的话，30 s 时可能还没推完，队友和车不一定进停靠区。即使这样，`Red After 3Tips` 也能拿到车停下时的位姿。

和两翻不同的地方：

- **射击前预转**：Intake 和通道从最近一次启动算起转满 1 s 就算预转完成。从 Flower 出来时它们一直没停，所以到点后飞轮、云台到位就射；其余几轮在离射击点 40 in 时开始转（两翻是 12 in），到点时已经转满。
- **射击前确认到位**：开挡前还要求车离射击点不到 1.5 in，并且静止 100 ms（`SHOT_SPOT_INCHES` / `SHOT_STILL_MS`）。到不了，就按原来的 1.5 s 超时照射，日志写出车离射击点多远。在 Flower 里的那一轮不做这个检查，因为车被 Flower 顶住。原因：Pedro 在车投影到路径最后一段 97.5% 处之后，100 ms 就报告"走完"（Foresight 的 `parametricTConstraint` 和 `timeoutConstraint` 默认值），不管车实际在哪。07:00 那次，06 的最后一段是边拐弯边转 90° 的曲线，车还没到位就开火，4 个球都没进。每段路径结束和每次开火时，日志都会写出车的实际位置。
- **Flower**：两个 Flower 到点后都停 0.75 s（`FLOWER_MAX_MS`；三翻 1.0 和两翻是 1.5 s）。整步（含开过去）最多 3 s（`FLOWER_STEP_MAX_MS`）。Far flower 点比两翻深 2.4 in，车会停在离点 9.4–13.3 in 处，所以"停住算到达"的范围放宽到 16 in（`FLOWER_STALL_INCHES`）。
- **铲子**：Far flower 的铲子在 02 开火时放下，退出时离停下的位置 10 in 收起（`FAR_SCOOP_CLEAR_INCHES`）。Side flower 是开着 Intake 退出来，在 08 射击点开挡时收起，那里离停下的位置约 10 in。06 的射击点在 Side flower 正前方，离得太近，铲子改在离 06 射击点 40 in 时（和预转一起）放下。
- **后退再回来**：只有 Garden 做。后退和回来在离目标 0.3 in 时就算到达，不等 Pedro 停稳（`PICKUP_NEAR_INCHES`）。队友预装球吸完后，Intake 也多转 1 s。
- **送料**：每轮送料 1000 ms（`VOLLEY_MS`；两翻是 1200 ms）。第 4 个球留在车里的话，先把它调回去。
- **Intake 高度**：和两翻一样，吸球时 0.56，吸完开走时降到 0.5。
- **没有时间门槛**：两翻和三翻 1.0 也没有。之前 2.0 有过"来不及就跳过 07、08 保停靠"的门槛，已去掉。
- **各步超时**：一段路径最多 4 s，推队友最多 3 s，03 原地转身最多 1.5 s。

09 的做法：
- **对正**：先转到 270°（车尾朝队友）并对正到 x = 9.5，转完之后车尾才碰到队友（车中心 y 67.5）。
- **推**：沿墙直推到车中心 y 88.5 就停，程序结束。这时车尾在 y 97.94，进停靠区 3.6 in（停靠区从 y 94.33 开始）；队友在 y 97.94–113.94，整台在停靠区里，18 in 的队友也放得下（停靠区到 117.92）。
- **推不动**：车在途中停住时，原地结束，这种情况下停不进停靠区。
- **结束交接**：程序结束时（包括 30 s 被场地停下时），把车当时的位姿交给 `Red After 3Tips`。

终点 `Red3Tips_2_0.end` 和 `PinpointConstants.RED_AFTER_3TIPS_START_POSE` 要一起改。

### 手动程序 `Red After 3Tips`（Main 分组）

`opmodes/teleops/KillerKingRedAfter3TipsTeleOp.java`。和 `Red` 完全一样，只是 Pinpoint 的起始位姿不同，跑完三翻 2.0 后不用转回两翻的停车点。

- **起始位姿**：优先用 `Red3Tips2.0` 结束时交来的实测位姿。自动结束时把 Pedro 位姿换算成 Pinpoint 坐标（X、Y 各加 1.25 in，朝向不变）存进 `AutoEndPose`，2 分钟内有效。
- **预设**：没有 2 分钟内的实测位姿（控制器 App 重启过，或者隔太久），就用预设 (10.75, 89.75) in、270°。
- **遥测**：INIT 时显示用的是哪个。
- **驾驶方向**：起始朝向 270°，驾驶前方照常是远离红方墙（+X），程序已按起始朝向换算，不用按方块重置。

**Limelight 和云台要注意**：
- **云台死区**：在终点（车头朝观众侧，车尾贴着队友），手动程序按位置会瞄远端 CELL (60, 84)，这个方向在云台死区里 13°：云台要转 173°，行程只有 ±160°。所以开局云台停在行程边缘，两块灯亮红色，手柄 1 震动。
- **怎么解除**：车逆时针转约 15°，或者开离墙边，就能恢复正常。在原地转会让车角扫到联盟墙，所以自动里没有转。
- **Limelight 能不能看到 HIVE**：Limelight 装在云台上，云台停在边缘时，镜头离远端 CELL 方向约 13°，应该还在视野里，但没有实测过。
- **校准**：位姿偏差超过 30 cm 时，Limelight 会自动改正 X、Y；朝向只在按圆圈 / B 时校准。用的是预设位姿、车实际没停在那里（比如推不动）时，开局按一次圆圈。
- **近端 CELL**：(60, 60) 在行程内（偏转 149°）。

## 蓝方 `Blue3Tips1.0` / `Blue2Tips1.0`（Auto 分组）

红方 `Red3Tips1.0`、`Red2Tips1.0` 定为 1.0 最终版。蓝方两个程序（`opmodes/autos/Blue3TipsAuto.java`、`Blue2TipsAuto.java`）是它们的中心对称复制：程序逻辑完全相同，只换了路线和联盟。路线 `Blue3Tips_1_0.java`、`Blue2Tips_1_0.java` 由红方路线绕场地中心旋转 180° 生成（Pedro 坐标 x' = 141.5 − x，y' = 141.5 − y，朝向 + 180°；控制点的朝向不用，保持 0），路径的写法不变。蓝方不是 Visualizer 导出的，改路线时先改红方，再重新生成蓝方，两边才能保持对称。Far flower 加深 5 cm、两翻 04 往联盟墙移 10 cm 后都已重新生成（蓝方 Far flower 点 (94.97, 7.01)、270°，两翻 04 (104.59, 18.42)、270°，往蓝方联盟墙方向）。用 Pedro 实际采样核对过：每段路径的位置和朝向都与红方旋转 180° 后完全一致，转向方向也相同（包括三翻 05 那段 180° 的转向）。

| | 红方 | 蓝方 |
| --- | --- | --- |
| 起点 | (54.37, 9.44) in、270° | (87.13, 132.06) in、90° |
| 第一翻的 CELL | A (60, 60) | (84, 84) |
| 远端射击的 CELL | B (60, 84) | (84, 60) |
| 自瞄 | `TurretAimCommand` 红方目标 | 蓝方目标（X = 84 in） |
| RobotLog 标签 | `Red3Tips1.0` / `Red2Tips1.0` | `Blue3Tips1.0` / `Blue2Tips1.0` |

云台的目标 Y 仍按车所在的半场选（Y < 72 in 打 Y = 60，其余打 Y = 84），中心对称后正好对上：蓝方起点在 Y > 72，第一翻打 (84, 84)；远端射击点在 Y < 72，打 (84, 60)。各射击点的云台偏转、到目标距离、每轮飞轮转速、Flower 铲、吸球和停留时间都和红方相同。前提是场地本身中心对称，蓝方开局朝上的是 (84, 84) 那个 CELL。试验版没有蓝方。

两边只有一处不对称：`Blue3Tips1.0` 结束时（包括 30 s 被场地停下时）把车当时的位姿交给手动程序 `Blue After 3Tips`，`Red3Tips1.0` 不交（没有接它的手动程序）。

### 手动程序 `Blue After 3Tips`（Main 分组）

`opmodes/teleops/KillerKingBlueAfter3TipsTeleOp.java`。跑完 `Blue3Tips1.0` 用这个，不要用 `Blue`：`Blue` 从两翻的停车点 (130.80, 48.23) in、180.13° 开始，三翻 09 停车的终点是 Pedro (132.11, 41.33)、270°，朝向差了 90°，驾驶方向和自瞄都会错。其余和 `Blue` 完全一样，包括手柄 2 的 ← / → 调转速。

- **起始位姿**：优先用 `Blue3Tips1.0` 结束时交来的实测位姿（换算成 Pinpoint 坐标，X、Y 各加 1.25 in，朝向不变），2 分钟内有效。只有自动真正开始跑过才交；INIT 里就停掉不算。
- **预设**：没有实测位姿时用 (133.36, 42.58) in、270°（`PinpointConstants.BLUE_AFTER_3TIPS_START_POSE`，即 `Blue3Tips_1_0` 的 `point909Park` 加 1.25 in）。重新生成蓝方路线、09 终点变了，要同步改这个常量。
- **驾驶方向**：起始朝向 270°，驾驶前方照常是远离蓝方墙（−X），程序已按起始朝向换算，不用按方块重置。
- **云台**：在预设终点打 (84, 60)，相对中位偏转 −19°，不在死区；到目标约 133 cm，灯亮绿。没有实测过。

## 试验版 `Red3Tips1.0 Vision (test)`（Auto 分组）

试验版在 `opmodes/autos/Red3TipsVisionAuto.java`，路线与稳定版相同，另加 Limelight 翻转检查、远端补射和时间截止。RobotLog 标签为 `Red3TipsVision`，和稳定版分开。所有动态功能只在这个文件里改，稳定版不受影响。

### 04 翻转检查与远端补射

试验版只检查 04 远端这一轮（第二翻）。起点预装球这一轮按必定翻转处理；最后一轮即使没翻也无法补救，所以都不检查。试验版会像手动程序一样启动 Limelight，INIT 时切到 P1。`HiveTipWatcher` 读取 HIVE 倾角，在 04 打开球挡时记下 HIVE 停在哪一侧，之后看到 HIVE 停到另一侧就判定为翻转。它只比较前后两侧，不需要标定倾角正负号对应哪个 CELL。

两条路线接下来都要去 05 吸队友预装球，所以在 05 结束时才读取结果，判断不额外耗时：

| 04 结果 | 后续 |
| --- | --- |
| 翻了 | 照原计划：06 近端 → 07 Side flower → 08 近端 → 09 停靠 |
| 没翻 | 05R 回到 04 的远端射击点，把队友预装球补射进仍然朝上的 B，完成第二翻。剩余时间 ≥ 12.5 s 时走 06R 到近端射击点，再接 07 → 08 → 09；否则 09R 直接从远端停靠 |
| 看不清 | 默认照原计划（`CONTINUE_WHEN_TIP_UNSEEN`） |

遥测显示 `Plan`、`04 tip check`、HIVE 倾角和 Limelight 状态。每段路径开始的时间、每个时间检查点的判断都会写进 RobotLog（`Red3TipsVision` 标签），可据此调整下面的时间阈值。05R、06R 是在运行时从当前位置连到现有点位的直线；提前停靠的路径在场地下半场借用 09 的控制点（先绕开 Side flower 再沿墙上行），上半场是直线。05R 从 270° 转到 90°；06R 全程保持 90°（边走边转到 180° 会扫到 HIVE），由 07 在前 60% 路程转到 180°，正对 Side flower；主路线的 07 本来就从 180° 出发，不用转。上半场的直线停靠保持当前朝向：05 结束时车离红方墙只有 7.7 in、朝 270°，原地转到 90° 会撞墙（原地转需要 11.7 in）。上场前需要在 Pedro Visualizer 里确认不会碰到 HIVE、墙或队友。

### 时间截止（停靠优先）

AUTO PARK 只在 AUTO 计分，TIP 在 AUTO 和 TELEOP 都是 20 分，所以时间不够时先保停靠，来不及的第三翻留到 TELEOP。

| 检查点 | 至少剩余 | 来不及时 |
| --- | --- | --- |
| 05 结束、开始 06 前 | 8.0 s | 从队友预装球位置直接停靠（相距约 2.5 in），车上 4 个球留给 TELEOP |
| 06 射完、开始 07 前（补救路线在 06R 前按 12.5 s 判断） | 10.0 s | 从近端停靠，A 里的 4 个球留给 TELEOP |
| 08 到位、最后一轮射击前 | 5.5 s | 不射，带着 4 个球停靠 |

看门狗：每个循环估算从当前位置去停靠点要多久（1.0 s + 距离 ÷ 35 in/s），再加 1.0 s 余量。剩余时间不够时立即中断当前步骤（路径、吸球或射击），关飞轮、云台回中，从当前位置去停靠。检查点阈值按 Visualizer 速度和 1.5 s 一轮射击估算，并用看门狗同一套停靠估算，按时完成的步骤不会被中途切掉。

按估算，整条三翻路线需要约 28.3–30.3 s（取决于起点飞轮起转多久）。留出停靠余量后，起点这一轮要在 1.5 s 内射完才赶得上 07，否则会跳过 Side flower。实际能不能走完，要看上场的 RobotLog。

---

## NOTICE

This repository contains the public FTC SDK for the BIOBUZZ (2026-2027) competition season.

## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

## Requirements
To use this Android Studio project, you will need Android Studio Narwhal 3 Feature Drop or later.

To program your robot in Blocks or OnBot Java, you do not need Android Studio.

## Getting Started
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html), and then migrate to the [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) or to [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html) afterwards.

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo.  Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the [Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) page for this repository.

* The Releases page also contains prebuilt APKs.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.

# Release Information

## Version 12.0 (20260907-090034)

### Breaking Changes
* The new AprilTag Cluster capability breaks legacy AprilTag OpModes resulting in compile errors for software that uses AprilTagDetection objects in both Android Studio and OnBot Java.
  * Legacy AprilTag OpModes must be updated to check whether the returned AprilTag is a cluster or singleton,
     and cast the returned detection into the correct type to access its elements.  See below:    
     
	 **Old method for AprilTag processing**
    ```
     for (AprilTagDetection detection : currentDetections) {
       // Do single Tag processing here  
     }
    ```

     **New method for AprilTag Singleton/Cluster processing**

    ```
     for (AprilTagDetection detection : currentDetections) {
       if (detection instanceof AprilTagSingleDetection) {
         AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
         // Do single Tag processing here  
       } else {
         AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
         // Do cluster Tag processing here  
       }
     }
     ```
    For more information about how to update your OpModes to fix the breaking change see: https://ftc-docs.firstinspires.org/apriltag-clusters
    
  * About AprilTag clusters:  
    * Clusters are co-planar groups of two or more AprilTags wherein the position of each member tag is defined relative to a common origin
    * This origin may be placed outside the bounds of the tags themselves to provide a more suitable "aiming" target
    * Clusters are resilient to partial occlusion. Full 6DOF pose can be estimated from a cluster even if only a single member tag is visible. Of course, the more tags that are visible, the better and more stable the pose estimate will be
    * All AprilTag samples have been updated to differentiate between standalone tags and clusters
  
### Enhancements
* Adds a tree view for robot configurations [issue 1821](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1821)
* Gamepad indicators on the Driver Station are now colored orange if the respective gamepads are connected to the Android generic gamepad driver instead of the Driver Station's usermode USB driver 
* Updated AprilTag Library for BIOBUZZ. Notably, getCurrentGameTagLibrary() now returns BIOBUZZ tags.
  * In BIOBUZZ, the Origin of each cluster is located in the center of the Cell opening for easy aiming.
  * The Origin X,Y & Z Axes are now displayed by default on the preview image.
  * <B>Unfortunately, since BIOBUZZ AprilTags move, they are not suitable for absolute Field Localization.</B>
* Supports OctoQuad MK2 firmware v3.1.0, which adds diagnostics parameters for the IMU and MCU uptime

### Bug Fixes
* Fixes issue [2078](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/2078) where battery
voltage was not updated on driver station if OpMode did not send any telemetry.

## Version 11.2.1 (20260724-093406)

### Bug Fixes
* Fixes issue [2099](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/2099). Gradle and the AGP are now updated to 9.1 and 8.13.2 respectively.
 
## Version 11.2 (20260707-102819)

### Breaking Changes
* Gradle is upgraded to v9.1 and the Android Gradle Plugin is updated to v8.13.2.  AGP v8.13.2 requires Android Studio Narwhal 3 Feature Drop or later.  Earlier versions of Android Studio will fail to sync the project.  Older versions of Android Studio may prompt the user to downgrade AGP.  Do not do this.  Gradle v9.1 removed support for features that older versions of the AGP use. Updating Gradle fixes a Windows 11 problem some teams may encounter if they have agressive security software installed on their machine.  For more context see [this Gradle issue](https://github.com/gradle/gradle/issues/31438)

### Enhancements
* New type of OpMode is now available. (`@Utility`)
   * Utility opmodes that are not disabled will show up in the Utility menu (requires 11.2 or later DS and RC) 
* TestHardware Utility now available
  * It allows you to test all servos, CR servos, motors, Color sensors, distance sensors, touch sensors, IMUs, webcams, and analog sensors in the config
* TestGamepad Utility now available
  * It allows you to see the results of your two gamepads to make sure it is what you expect and find problems with your gamepads. 
* Adds methods to PwmControl interface to allow you to setPulseWidth and getPulseWidth 
   * Both of these are in microseconds (uSeconds)
   * This is an ADVANCED feature.   There is not a supporting sample.
   * NOTE: You may see a slight difference since the hardware is not accurate to the microsecond
* Adds ability to set UVC camera "quirks" from user code to control compatibility flags used inside the low level UVC driver. Addresses issue [1428](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1428)
* Changes Apriltag Axis Display order on camera preview screen, to reflect updated Z axis direction.
* The Driver Station app init button has a light teal background with the word init if
   * the driver station and robot controller are connected and have the same team number
   * there is at least one gamepad attached
   * the timer is enabled (for an Autonomous OpMode)

### Bug Fixes
* Fixes issue [1949](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1949) overwriting the group with the default group when registering a OpMode with OpModeManager.register(OpModeMeta name, Class<? extends OpMode> clazz)
* Fixes issue mentioned in [1890](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1890) where if 
  for a servo you change the direction or scaleRange and send the same setPosition that was sent 
  before, then it wouldn't update the servo.
* Fixes an issue where Self-Inspect doesn't flag a driver station using -RC in it's name. The message is now:
  * The team numbers in the robot controller and driver station names do not match, or a device name is invalid. Refer to the FTC Competition Manual for device naming rules.

## Version 11.1 (20251231-104637)

### Enhancements

* Gamepad triggers can now be accessed as booleans and have edge detection supported.
* GoBildaPinpointDriver now supports Pinpoint v2 functionality
* Adds webcam calibrations for goBILDA's USB camera.

### Bug Fixes
* Fixes issue [1654](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1654) in GoBildaPinpointDriver that caused error if resolution was set in other than MM
* Fixes issue [1628](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1628) Blocks editor displays incorrect Java code for gamepad edge detection blocks.
* Fixes possible race condition issue [1884](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1884) on Driver Station startup when Driver Station name doesn't match the Robot Controller name.
* Fixes issue [1863](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1863) - Incorrect package paths in samples.
* Fixes an issue where an OnBotJava filename that begins with a lowercase character would fail to properly rename the file if the user tried to rename it so that it begins with an uppercase character.

## Version 11.0 (20250827-105138)

### Enhancements

* OnBotJava now has the concept of a project.  
  A project is a collection of related files.  A project may be chosen by selecting 'Example Project'
  from the 'File type:' dropdown.  Doing so will populate the dropdown to the immediate right with 
  a list of projects to choose from.
  When selecting a project all of the related files appear in the left pane of the workspace 
  underneath a directory with the chosen project name.
  This is useful for example for ConceptExternalHardwareClass which has a dependency upon
  RobotHardware.  This feature simplifies the usage of this Concept example by automatically
  pulling in dependent classes.
* Adds support for AndyMark ToF, IMU, and Color sensors.
* The Driver Station app indicates if WiFi is disabled on the device.
* Adds several features to the Color Processing software:
  * DECODE colors `ARTIFACT_GREEN` and `ARTIFACT_PURPLE`
  * Choice of the order of pre-processing steps Erode and Dilate
  * Best-fit preview shape called `circleFit`, an alternate to the existing `boxFit`
  * Sample OpMode `ConceptVisionColorLocator_Circle`, an alternate to the renamed `ConceptVisionColorLocator_Rectangle`
* The Driver Station app play button has a green background with a white play symbol if
  * the driver station and robot controller are connected and have the same team number
  * there is at least one gamepad attached
  * the timer is enabled (for an Autonomous OpMode)
* Updated AprilTag Library for DECODE. Notably, getCurrentGameTagLibrary() now returns DECODE tags.
  * Since the AprilTags on the Obelisk should not be used for localization, the ConceptAprilTagLocalization samples only use those tags without the name 'Obelisk' in them.
* OctoQuad I2C driver updated to support firmware v3.x 
  * Adds support for odometry localizer on MK2 hardware revision
  * Adds ability to track position for an absolute encoder across multiple rotations
  * Note that some driver APIs have changed; minor updates to user software may be required
  * Requires firmware v3.x. For instructions on updating firmware, see
    https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/OctoQuadDatasheet_Rev_3.0C.pdf


## Version 10.3 (20250625-090416)

### Breaking Changes
* The behavior of setGlobalErrorMsg() is changed.  Note that this is an SDK internal method that is not 
  meant to be used by team software or third party libraries.  Teams or libraries using this method should
  find another means to communicate failure.  The design intent of setGlobalErrorMsg() is to report an 
  error and force the user to restart the robot, which in certain circumstances when used inappropriately
  could cause a robot to continue running while Driver Station controls are disabled.  To prevent this,
  processing of a call to setGlobalErrorMsg() is deferred until the robot is in a known safe state.  This may
  mean that a call to setGlobalErrorMsg() that does not also result in stopping a running OpMode will appear
  as though nothing happened until the robot is stopped, at which point, if clearGlobalErrorMsg() has not 
  been called the message will appear on the Driver Station and a restart will be required.
  Addresses issue [1381](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1381)
* Fixes getLatestResult in Limelight3A so if the Limelight hasn't provided data yet, it still returns an LLResult but valid will be false
  * If you previously used to check and see if this was `null` to see if the Limelight had been contacted, you now need to use `isValid()` on the result.  That is because now it always returns an LLResult even before it talks to the Limelight, but if it doesn't have valid data, the `isValid()` will be `false`.
* Changed all omni samples to use front_left_drive, front_right_drive, back_left_drive, back_right_drive
  * This is only breaking for you if you copy one of the changed samples to your own project and expect to use the same robot configuration as before.

### Known Issues
* The redesigned OnBotJava new file workflow allows the user to use a lowercase letter as the first character of a filename.
  This is a regression from 10.2 which required the first character to be uppercase.  Software will build, but if the user tries
  to rename the file, the rename will fail.

### Enhancements
* Adds a configuration item for a Full Range Servo.  Selecting this item expands the pulse width range from 500us to 2500us.  For comparison, the legacy Servo type defines the pulse width range as 600us to 2400us.
* Improved the OBJ new file creation flow workflow. The new flow allows you to easily use samples, craft new custom OpModes and make new Java classes.
* Added support for gamepad edge detection.
  * A new sample program `ConceptGamepadEdgeDetection` demonstrates its use.
* Adds a blackboard member to the Opmode that maintains state between opmodes (but not between robot resets).  See the ConceptBlackboard sample for how to use it.
* Updated PredominantColorProcessor to also return the predominant color in RGB, HSV and YCrCb color spaces.  Updated ConceptVisionColorSensor sample OpMode to display the getAnalysis() result in all three color spaces.
* Adds support for the GoBilda Pinpoint 
  * Also adds `SensorGoBildaPinpoint` sample to show how to use it
* Added `getArcLength()` and `getCircularity()` to ColorBlobLocatorProcessor.Blob.  Added BY_ARC_LENGTH and BY_CIRCULARITY as additional BlobCriteria.
* Added `filterByCriteria()` and `sortByCriteria()` to ColorBlobLocatorProcessor.Util.
  * The filter and sort methods for specific criteria have been deprecated.
  * The updated sample program `ConceptVisionColorLocator` provides more details on the new syntax.
* Add Help menu item and Help page that is available when connected to the robot controller via Program and Manage. The Help page has links to team resources such as [FTC Documentation](https://ftc-docs.firstinspires.org/), [FTC Discussion Forums](https://ftc-community.firstinspires.org), [Java FTC SDK API Documentation](https://javadoc.io/doc/org.firstinspires.ftc), and [FTC Game Information](https://ftc.game/).
* Self inspection changes:
  * List both the Driver Station Name and Robot Controller Name when inspecting the Driver Station.
  * Report if the team number portion of the device names do not match.
  * -rc is no longer valid as part of a Robot Controller name, must be -RC.
  * Use Robot Controller Name or Driver Station Name labels on the inspection screens instead of WIFI Access Point or WIFI Direct Name.

### Bug Fixes
* Fixes issue [1478](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1478) in AnnotatedHooksClassFilter that ignored exceptions if they occur in one of the SDK app hooks.
* Fix initialize in distance sensor (Rev 2m) to prevent bad data in first call to getDistance.
* Fixes issue [1470](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1470) Scaling a servo range is now irrespective of reverse() being called.  For example, if you set the scale range to [0.0, 0.5] and the servo is reversed, it will be from 0.5 to 0.0, NOT 1.0 to 0.5.
* Fixes issue [1232](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1232), a rare race condition where using the log rapidly along with other telemetry could cause a crash.

## Version 10.2 (20250121-174034)

### Enhancements
* Add ability to upload the pipeline for Limelight3A which allows teams to version control their limelight pipelines.


### Bug Fixes

* Fix an internal bug where if the RUN_TO_POSITION run mode was specified before a target position, recovery would require a power cycle. A side effect of this fix is that a stack trace identifying the location of the error is always produced in the log. Fixes issue [1345](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1345).
* Throws a helpful exception if region of interest is set to null when building a PredominantColorProcessor. Also sets the default RoI to the full frame. Addresses issue [1076](FIRST-Tech-Challenge/FtcRobotController#1076)
* Throws a helpful exception if user tries to construct an ImageRegion with malformed boundaries.  Addresses issue [1078](FIRST-Tech-Challenge/FtcRobotController#1078)

## Version 10.1.1 (20241102-092223)

### Breaking Changes

* Support for Android Studio Ladybug.  Requires Android Studio Ladybug.  

### Known Issues

* Android Studio Ladybug's bundled JDK is version 21.  JDK 21 has deprecated support for Java 1.8, and Ladybug will warn on this deprecation.
  OnBotJava only supports Java 1.8, therefore, in order to ensure that software developed using Android Studio will 
  run within the OnBotJava environment, the targetCompatibility and sourceCompatibility versions for the SDK have been left at VERSION_1_8.
  FIRST has decided that until it can devote the resources to migrating OnBotJava to a newer version of Java, the deprecation is the 
  lesser of two non-optimal situations.

### Enhancements

* Added `toString()` method to Pose2D
* Added `toString()` method to SparkFunOTOS.Pose2D

## Version 10.1 (20240919-122750)

### Enhancements
* Adds new OpenCV-based `VisionProcessor`s (which may be attached to a VisionPortal in either Java or Blocks) to help teams implement color processing via computer vision in the INTO THE DEEP game
  * `ColorBlobLocatorProcessor` implements OpenCV color "blob" detection. A new sample program `ConceptVisionColorLocator` demonstrates its use.
    * A choice is offered between pre-defined color ranges, or creating a custom one in RGB, HSV, or YCrCb color space
    * The ability is provided to restrict detection to a specified Region of Interest on the screen
    * Functions for applying erosion / dilation morphing to the threshold mask are provided
    * Functions for sorting and filtering the returned data are provided
  * `PredominantColorProcessor` allows using a region of the camera as a "long range color sensor" to determine the predominant color of that region. A new sample program `ConceptVisionColorSensor` demonstrates its use.
    * The determined predominant color is selected from a discrete set of color "swatches", similar to the MINDSTORMS NXT color sensor
  * Documentation on this Color Processing feature can be found here: https://ftc-docs.firstinspires.org/color-processing
* Added Blocks sample programs for color sensors: RobotAutoDriveToLine and SensorColor.
* Updated Self-Inspect to identify mismatched RC/DS software versions as a "caution" rather than a "failure."

### Bug Fixes
* Fixes [AngularVelocity conversion regression](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070)

## Version 10.0  (20240828-111152)

### Breaking Changes
* Java classes and Blocks for TensorFlow Object Detection have been removed.
* `AngularVelocity.unit` which was of type `AngleUnit` has been renamed `AngularVelocity.angleUnit` of type `UnnormalizedAngleUnit`

### Enhancements
* Sample for REV Digital Indicator has been added - ConceptRevLED
* Adds support for the [Sparkfun QWIIC LED Stick](https://www.sparkfun.com/products/18354)
  * To connect it directly, you need this [cable](https://www.sparkfun.com/products/25596)
* Adds ConceptLEDStick OpMode
* Adds Blocks for colors black, blue, cyan, dkgray, gray, green, ltgray, magenta, red, white, and yellow.
* Adds an "evaluate but ignore result" Block that executes the connected block and ignores the result. Allows you to call a function and ignore the return value.
* Adds I2C driver for Maxbotix Maxsonar I2CXL sonar rangefinder
* Adds Blocks for setPwmEnable, setPwmDisable, and isPwmEnabled for servos and CR servos.
* In the Blocks editor: a \n in the ExportToBlocks annotation's comment field is displayed as a line break.
* Telemetry has new method setNumDecimalPlaces
* Telemetry now formats doubles and floats (not inside objects, just by themselves)
* Adds support for the Limelight 3A.
* Adds initial support for the REV Servo Hub
  * Both the Robot Controller and Driver Station need to be updated to version 10.0 in order for Servo Hubs to be
    configurable as Servo Hubs. If the app on either device is outdated, the Servo Hub will show up as an Expansion Hub,
    and some functionality will not work as expected. You should wait to create a configuration that includes a Servo Hub
    until both the Driver Station and Robot Controller apps have been updated to version 10.0.
  * Updating the Servo Hub's firmware and changing its address can only be done using the REV Hardware Client at this time
* Adds support for the REV 9-Axis IMU (REV-31-3332)
  * The REV 9-Axis IMU is only supported by the [Universal IMU interface](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)
  * Adds `Rev9AxisImuOrientationOnRobot` Java class.
  * If you mentally substitute this IMU's I2C port for the Control Hub's USB ports, `RevHubOrientationOnRobot` is also compatible with this sensor
  * Adds Blocks for Rev9AxisImuOrientationOnRobot, including RevHubImuOrientationOnRobot.xyzOrientation and  RevHubImuOrientationOnRobot.zyxOrientation.
  * Adds Blocks samples SensorRev9AxisIMUOrthogonal and SensorRev9AxisIMUNonOrthogonal.
* Improves Blocks support for RevHubImuOrientationOnRobot.
  * Adds Blocks for RevHubImuOrientationOnRobot.xyzOrientation and  RevHubImuOrientationOnRobot.zyxOrientation.
  * Adds Blocks samples SensorHubIMUOrthogonal (replaces SensorIMU) and SensorHubIMUNonOrthogonal.
* Updates EasyOpenCV, AprilTag, OpenCV, and `libjpeg-turbo` versions
* Adds Blocks for max and min that take two numbers.
* Adds Blocks OpModes ConceptRevSPARKMini, RobotAutoDriveByEncoder, RobotAutoDriveByGyro, RobotAutoDriveByTime, RobotAutoDriveToAprilTagOmni, and RobotAutoDriveToAprilTagTank.
* Two OpModes with the same name now automatically get renamed with the name followed by a "-" and the class name allowing them to both be on the device.
* Shows the name of the active configuration on the Manage page of the Robot Controller Console
* Updated AprilTag Library for INTO THE DEEP. Notably, `getCurrentGameTagLibrary()` now returns INTO THE DEEP tags.
* Adds Blocks for Telemetry.setMsTransmissionInterval and Telemetry.getMsTransmissionInterval.
* Adds Blocks sample SensorOctoQuad.

### Bug Fixes
* Fixes a bug where the RevBlinkinLedDriver Blocks were under Actuators in the Blocks editor toolbox. They are now Other Devices.
* Fixes a bug where `Exception`s thrown in user code after a stop was requested by the Driver Station would be silently eaten
* Fixed a bug where if you asked for `AngularVelocity` in a unit different than the device reported it in, it would normalize it between -PI and PI for radians, and -180 and 180 for degrees.

## Version 9.2 (20240701-085519)

### Important Notes
* Java classes and Blocks for TensorFlow Object Detection have been deprecated and will be removed in Version 10.0.
* The samples that use TensorFlow Object Detection have been removed.

### Enhancements
* Adds explanatory text to failed items on the inspection activities.  To view the explanatory text tap the red warning icon for a failed item.
* In the Blocks editor: added a new kind of variable set block that sets the variable and also returns the new value.
* Changes the way that camera controls behave for a SwitchableCamera. Now, each method (such as getExposure, getMinExposure, getMaxExposure, setExposure for ExposureControl) acts on the currently active camera.
* Adds support for the REV USB PS4 Compatible Gamepad (REV-31-2983)
* Adds ConceptAprilTagMultiPortal OpMode
* Adds support for OctoQuad Quadrature Encoder & Pulse Width Interface Module
* Adds the ExportAprilTagLibraryToBlocks annotation that indicates that a static method that returns an AprilTagLibrary is exported to the Blocks programming environment. The corresponding block will appear in the Blocks toolbox along with the built-in tag libraries.
* Adds Blocks OpMode ConceptAprilTagOptimizeExposure.
* Adds support for the SparkFun Optical Tracking Odometry sensor.

### Bug Fixes
* Fixes https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/942 where visionPortal.close() can cause an IndexOutOfBoundsError.
* Fixes a bug in the blocks editor where collapsed function blocks show a warning "Collapsed blocks contain warnings." when the Blocks OpMode is reopened.
* Fixes a bug where the blocks editor wouldn't warn you that you have unsaved changes when you try to leave. This bug was introduced due to a behavior change in Chrome 119.
* [Issue #764](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/764) - Get gain control returns a null pointer for a switchable camera
* Fixes a bug where the correct deadzone for certain gamepads was not applied when Advanced Gamepad Features was enabled

## Version 9.1 (20240215-115542)

### Enhancements
* Fixes a problem with Blocks: if the user closes a Block's warning balloon, it will still be closed next time the project is opened in the Blocks editor.
* In the Blocks editor, an alert concerning missing hardware devices is not shown if all the Blocks that use the missing hardware devices are disabled.
* Adds Blocks to support comparing property values CRServo.Direction, DCMotor.Direction, DCMotor.Mode, DCMotor.ZeroPowerBehavior, DigitalChannel.Mode, GyroSensor.HeadingMode, IrSeekerSensor.Mode, and Servo.Direction, to the corresponding enum Block.
* Improves OnBotJava auto-import to correctly import classes when used in certain situations.
* Improves OnBotJava autocomplete to provide better completion options in most cases.
  * This fixes an issue where autocomplete would fail if a method with two or more formal parameters was defined.
* In OnBotJava, code folding support was added to expand and collapse code sections
* In OnBotJava, the copyright header is now automatically collapsed loading new files
* For all Blocks OpMode samples, intro comments have been moved to the RunOpMode comment balloon.
* The Clean up Blocks command in the Blocks editor now positions function Blocks so their comment balloons don't overlap other function Blocks.
* Added Blocks OpMode sample SensorTouch.
* Added Java OpMode sample SensorDigitalTouch.
* Several improvements to VisionPortal
  * Adds option to control whether the stream is automatically started following a `.build()` call on a VisionPortal Builder
  * Adds option to control whether the vision processing statistics overlay is rendered or not
  * VisionPortals now implement the `CameraStreamSource` interface, allowing multiportal users to select which portal is routed to the DS in INIT by calling CameraStreamServer.getInstance().setSource(visionPortal). Can be selected via gamepad, between Camera Stream sessions.
  * Add option to `AprilTagProcessor` to suppress calibration warnings
  * Improves camera calibration warnings
    * If a calibration is scaled, the resolution it was scaled from will be listed
    * If calibrations exist with the wrong aspect ratio, the calibrated resolutions will be listed
  * Fixes race condition which caused app crash when calling `stopStreaming()` immediately followed by `close()` on a VisionPortal
  * Fixes IllegalStateException when calling `stopStreaming()` immediately after building a VisionPortal
  * Added FTC Blocks counterparts to new Java methods:
    * VisionPortal.Builder.setAutoStartStreamOnBuild
    * VisionPortal.Builder.setShowStatsOverlay
    * AprilTagProcessor.Builder.setSuppressCalibrationWarnings
    * CameraStreamServer.setSource​

### Bug Fixes
* Fixes a problem where OnBotJava does not apply font size settings to the editor.
* Updates EasyOpenCV dependency to v1.7.1
  * Fixes inability to use EasyOpenCV CameraFactory in OnBotJava
  * Fixes entire RC app crash when user pipeline throws an exception
  * Fixes entire RC app crash when user user canvas annotator throws an exception
  * Use the modern stacktrace display when handling user exceptions instead of the legacy ESTOP telemetry message

## Version 9.0.1 (20230929-083754)

### Enhancements
* Updates AprilTag samples to include Decimation and additional Comments.  Also corrects misleading tag ID warnings
* Increases maximum size of Blocks inline comments to 140 characters
* Adds Blocks sample BasicOmniOpMode.
* Updated CENTERSTAGE library AprilTag orientation quaternions
    * Thanks [@FromenActual](https://github.com/FromenActual)
* Updated Java Sample ConceptTensorFlowObjectDetection.java to include missing elements needed for custom model support.

### Bug Fixes
* Fixes a problem where after October 1 the Driver Station will report as obsolete on v9.0 and prompt the user to update.

## Version 9.0 (20230830-154348)

### Breaking Changes
* Removes Vuforia
* Fields in `AprilTagDetection` and `AprilTagPose(ftc/raw)` objects are now `final`
* VisionPortal builder method `setCameraMonitorViewId()` has been renamed to `setLiveViewContainerId()` and `enableCameraMonitoring()` has been renamed to `enableLiveView()`

### Enhancements
* Adds support for the DFRobot HuskyLens Vision Sensor.
* Blocks teams can now perform webcam calibration.
    * Added a Block for System.currentTimeMillis (under Utilities/Time)
    * Added a Block for VisionPortal.saveNextFrameRaw (under Vision/VisionPortal)
    * Added a new sample Blocks OpMode called UtilityCameraFrameCapture.
* The RobotDriveByGyro sample has been updated to use the new universal IMU interface.  It now supports both IMU types.
* Removed some error-prone ElapsedTime Blocks from the Blocks editor's toolbox. This is not a
  breaking change: old Blocks OpModes that use these Blocks will still function, both in the
  Blocks editor and at runtime.
* Standardizes on the form "OpMode" for the term OpMode.
    * The preferred way to refer to OpModes that specifically extend `LinearOpMode` (including Blocks OpModes) is "linear OpMode".
    * The preferred way to refer to OpModes that specifically extend `OpMode` directly is "iterative OpMode".
* Overhauls `OpMode` and `LinearOpMode` Javadoc comments to be easier to read and include more detail.
* Makes minor enhancements to Java samples
    * Javadoc comments in samples that could be rendered badly in Android Studio have been converted to standard multi-line comments
    * Consistency between samples has been improved
    * The SensorDigitalTouch sample has been replaced with a new SensorTouch sample that uses the `TouchSensor` interface instead of `DigitalChannel`.
    * The ConceptCompassCalibration, SensorMRCompass, and SensorMRIRSeeker samples have been deleted, as they are not useful for modern FTC competitions.

### Bug Fixes
* Fixes a bug which prevented PlayStation gamepads from being used in bluetooth mode. Bluetooth is NOT legal for competition but may be useful to allow a DS device to be used while charging, or at an outreach event.
* Fixes a bug where a Blocks OpMode's Date Modified value can change to December 31, 1969, if the Control Hub is rebooted while the Blocks OpMode is being edited.
* Fixes the automatic TeleOp preselection feature (was broken in 8.2)
* Fixes a bug where passing an integer number such as 123 to the Telemetry.addData block that takes a number shows up as 123.0 in the telemetry.
* Fixes OnBotJava autocomplete issues:
  * Autocomplete would incorrectly provide values for the current class when autocompleting a local variable
  * `hardwareMap` autocomplete would incorrectly include lambda class entries
* Fixes OnBotJava not automatically importing classes.
* Fixes OnBotJava tabs failing to close when their file is deleted.
* Fixes a project view refresh not happening when a file is renamed in OnBotJava.
* Fixes the "Download" context menu item for external libraries in the OnBotJava interface.
* Fixes issue where Driver Station telemetry would intermittently freeze when set to Monospace mode.
* Fixes performance regression for certain REV Hub operations that was introduced in version 8.2.
* Fixes TagID comparison logic in DriveToTag samples.

## Version 8.2 (20230707-131020)

### Breaking Changes
* Non-linear (iterative) OpModes are no longer allowed to manipulate actuators in their `stop()` method. Attempts to do so will be ignored and logged.
  * When an OpMode attempts to illegally manipulate an actuator, the Robot Controller will print a log message
    including the text `CANCELLED_FOR_SAFETY`.
  * Additionally, LinearOpModes are no longer able to regain the ability to manipulate actuators by removing their
    thread's interrupt or using another thread.
* Removes support for Android version 6.0 (Marshmallow). The minSdkVersion is now 24.
* Increases the Robocol version.
  * This means an 8.2 or later Robot Controller or Driver Station will not be able to communicate with an 8.1 or earlier Driver Station or Robot Controller.
  * If you forget to update both apps at the same time, an error message will be shown explaining which app is older and should be updated.
* FTC_FieldCoordinateSystemDefinition.pdf has been moved.  It is still in the git history, but has been removed from the git snapshot corresponding with the 8.2 tag.  The official version now lives at [Field Coordinate System](https://ftc-docs.firstinspires.org/field-coordinate-system).
* `LynxUsbDevice.addConfiguredModule()` and `LynxUsbDevice.getConfiguredModule()` have been replaced with `LynxUsbDevice.getOrAddModule()`.
* Old Blocks for Vuforia and TensorFlow Object Detection are obsolete and have been removed from the
  Blocks editor's toolbox. Existing Blocks OpModes that contain the old Blocks for Vuforia or
  TensorFlow Object Detection can be opened in the Blocks editor, but running them will not work.

### New features
* Adds new `VisionPortal` API for computer vision
    * **This API may be subject to change for final kickoff release!**
    * Several new samples added.
    * Adds support for detecting AprilTags.
    * `VisionPortal` is the new entry point for both AprilTag and TFOD processing.
    * Vuforia will be removed in a future release.
    * Updated TensorFlow dependencies.
    * Added support for webcam camera controls to blocks.
    * The Blocks editor's toolbox now has a Vision category, directly above the Utilities category.
* Related documentation for associated technologies can be found at
    * [AprilTag Introduction](https://ftc-docs.firstinspires.org/apriltag-intro)
    * [AprilTag SDK Guide](https://ftc-docs.firstinspires.org/apriltag-sdk)
    * [AprilTag Detection Values](https://ftc-docs.firstinspires.org/apriltag-detection-values)
    * [AprilTag Test Images](https://ftc-docs.firstinspires.org/apriltag-test-images)
    * [Camera Calibration](https://ftc-docs.firstinspires.org/camera-calibration)
* Adds Driver Station support for Logitech Dual Action and Sony PS5 DualSense gamepads.
    * This **does not** include support for the Sony PS5 DualSense Edge gamepad.
    * Always refer to Game Manual 1 to determine gamepad legality in competition.
* Adds support for MJPEG payload streaming to UVC driver (external JPEG decompression routine required for use).
* Shows a hint on the Driver Station UI about how to bind a gamepad when buttons are pressed or the sticks are moved on an unbound gamepad.
* Adds option for fullscreening "Camera Stream" on Driver Station.
* OnBotJava source code is automatically saved as a ZIP file on every build with a rolling window of the last 30 builds kept; allows recovering source code from previous builds if code is accidentally deleted or corrupted.
* Adds support for changing the addresses of Expansion Hubs that are not connected directly via USB.
  * The Expansion Hub Address Change screen now has an Apply button that changes the addresses without leaving the screen.
  * Addresses that are assigned to other hubs connected to the same USB connection or Control Hub are no longer able to be selected.
* Increases maximum size of Blocks inline comments to 100 characters
* Saves position of open Blocks comment balloons
* Adds new AprilTag Driving samples:  RobotDriveToAprilTagTank & RobotDriveToAprilTagOmni
* Adds Sample to illustrate optimizing camera exposure for AprilTags: ConceptAprilTagOptimizeExposure

### Bug Fixes
* Corrects inspection screen to report app version using the SDK version defined in the libraries instead of the version specified in `AndroidManifest.xml`. This corrects the case where the app could show matching versions numbers to the user but still state that the versions did not match.
  * If the version specified in `AndroidManifest.xml` does not match the SDK version, an SDK version entry will be displayed on the Manage webpage.
* Fixes no error being displayed when saving a configuration file with duplicate names from the Driver Station.
* Fixes a deadlock in the UVC driver which manifested in https://github.com/OpenFTC/EasyOpenCV/issues/57.
* Fixes a deadlock in the UVC driver that could occur when hot-plugging cameras.
* Fixes UVC driver compatibility with Arducam OV9281 global shutter camera.
* Fixes Emergency Stop condition when an OnBotJava build with duplicate OpMode names occurs.
* Fixes known causes of "Attempted use of a closed LynxModule instance" logspam.
* Fixes the visual identification LED pattern when configuring Expansion Hubs connected via RS-485.

## Version 8.1.1 (20221201-150726)

This is a bug fix only release to address the following four issues.

* [Issue #492](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/492) - Can't create new blocks opmodes.
* [Issue #495](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/495) - Remove the final modifier from the OpMode's Telemetry object.
* [Issue #500](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/500) - Some devices cannot be configured when the Driver Station app has been updated to 8.1
  * Updating either the Robot Controller app or the Driver Station app to 8.1.1 or later will fix this issue.
* The Modern Robotics touch sensor was configurable as a  Digital Device. It can only be used as an Analog Device.

## Version 8.1 (20221121-115119)

### Breaking Changes
* Deprecates the `OpMode` fields `msStuckDetectInit`, `msStuckDetectInitLoop`, `msStuckDetectStart`, `msStuckDetectLoop`, and `msStuckDetectStop`.
    * OpModes no longer have a time limit for `init()`, `init_loop()`, `start()` or `loop()`, so the fields corresponding to those methods are no longer used.
    * `stop()` still has a time limit, but it is now hardcoded to be 1 second, and cannot be changed using `msStuckDetectStop`.
* Deprecates the `OpMode` methods `internalPreInit()`, `internalPostInitLoop()`, and `internalPostLoop()`.
    * Iterative `OpMode`s will continue to call these methods in case they were overridden.
    * These methods will not be called at all for `LinearOpMode`s.
* Deprecates (and stops respecting) `DeviceProperties.xmlTagAliases`.

### Enhancements
* Adds a new `IMU` interface to Blocks and Java that can be used with both the original BNO055 IMU
  included in all older Control Hubs and Expansion Hubs, and the new alternative BHI260AP IMU.
  * You can determine which type of IMU is in your Control Hub by navigating to the Manage page of the web interface.
  * To learn how to use the new `IMU` interface, see https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html. The `SensorIMU` Blocks sample was also updated to use the new `IMU` interface, and the following Java samples were added:
    * `SensorIMUOrthogonal`
      * Use this sample if your REV Hub is mounted so that it is parallel or perpendicular to the
        bottom of your robot.
    * `SensorIMUNonOrthogonal`
      * Use this sample if your REV Hub is mounted to your robot in any other orientation
    * `ConceptExploringIMUOrientations`
      * This OpMode is a tool to help you understand how the orthogonal orientations work, and
        which one applies to your robot.
  * The BHI260AP IMU can only be accessed via the new `IMU` interface. The BNO055 IMU can be
    programmed using the new `IMU` interface, or you can continue to program it using the old `BNO055IMU`
    interface. If you want to be able to quickly switch to a new Control Hub that may contain the
    BHI260AP IMU, you should migrate your code to use the new `IMU` interface.
  * Unlike the old `BNO055IMU` interface, which only worked correctly when the REV Hub was mounted flat
    on your robot, the `IMU` interface allows you to specify the orientation of the REV Hub on your
    robot. It will account for this, and give you your orientation in a Robot Coordinate System,
    instead of a special coordinate system for the REV Hub. As a result, your pitch and yaw will be
    0 when your *robot* is level, instead of when the REV Hub is level, which will result in much
    more reliable orientation angle values for most mounting orientations.
  * Because of the new robot-centric coordinate system, the pitch and roll angles returned by the
    `IMU` interface will be different from the ones returned by the `BNO055IMU` interface. When you are
    migrating your code, pay careful attention to the documentation.
  * If you have calibrated your BNO055, you can provide that calibration data to the new `IMU`
    interface by passing a `BNO055IMUNew.Parameters` instance to `IMU.initialize()`.
  * The `IMU` interface is also suitable for implementation by third-party vendors for IMUs that
    support providing the orientation in the form of a quaternion.
* Iterative `OpMode`s (as opposed to `LinearOpMode`s) now run on a dedicated thread.
    * Cycle times should not be as impacted by everything else going on in the system.
    * Slow `OpMode`s can no longer increase the amount of time it takes to process network commands, and vice versa.
    * The `init()`, `init_loop()`, `start()` and `loop()` methods no longer need to return within a certain time frame.
* BNO055 IMU legacy driver: restores the ability to initialize in one OpMode, and then have another OpMode re-use that
  initialization. This allows you to maintain the 0-yaw position between OpModes, if desired.
* Allows customized versions of device drivers in the FTC SDK to use the same XML tag.
  * Before, if you wanted to customize a device driver, you had to copy it to a new class _and_ give
    it a new XML tag. Giving it a new XML tag meant that to switch which driver was being used, you
    had to modify your configuration file.
  * Now, to use your custom driver, all you have to do is specify your custom driver's class when
    calling `hardwareMap.get()`. To go back to the original driver, specify the original driver
    class. If you specify an interface that is implemented by both the original driver and the
    custom driver, there is no guarantee about which implementation will be returned.

### Bug Fixes
* Fixes accessing the "Manage TensorFlow Lite Models" and "Manage Sounds" links and performing
  Blocks and OnBotJava OpMode downloads from the REV Hardware Client.
* Fixes issue where an I2C device driver would be auto-initialized using the parameters assigned in
  a previous OpMode run.
* Improves Driver Station popup menu placement in the landscape layout.
* Fixes NullPointerException when attempting to get a non-configured BNO055 IMU in a Blocks OpMode on an RC phone.
* Fixes problem with Blocks if a variable is named `orientation`.

## Version 8.0 (20220907-131644)

### Breaking Changes
* Increases the Robocol version.
  * This means an 8.0 or later Robot Controller or Driver Station will not be able to communicate with a 7.2 or earlier Driver Station or Robot Controller.
  * If you forget to update both apps at the same time, an error message will be shown explaining which app is older and should be updated.
* Initializing I2C devices now happens when you retrieve them from the `HardwareMap` for the first time.
  * Previously, all I2C devices would be initialized before the OpMode even began executing,
    whether you were actually going to use them or not. This could result in reduced performance and
    unnecessary warnings.
  * With this change, it is very important for Java users to retrieve all needed devices from the
    `HardwareMap` **during the Init phase of the OpMode**. Namely, declare a variable for each hardware
    device the OpMode will use, and assign a value to each. Do not do this during the Run phase, or your
    OpMode may briefly hang while the devices you are retrieving get initialized.
  * OpModes that do not use all of the I2C devices specified in the configuration file should take
    less time to initialize. OpModes that do use all of the specified I2C devices should take the
    same amount of time as previously.
* Fixes [issue #251](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251) by changing the order in which axis rotation rates are read from the angular velocity vector in the BNO055 IMU driver.
* Deprecates `pitchMode` in `BNO055IMU.Parameters`.
  * Setting `pitchMode` to `PitchMode.WINDOWS` would break the coordinate conventions used by the driver.
* Moves `OpModeManagerImpl` to the `com.qualcomm.robotcore.eventloop.opmode` package.
  * This breaks third party libraries EasyOpenCV (version 1.5.1 and earlier) and FTC Dashboard (version 0.4.4 and earlier).
* Deletes the deprecated `OpMode` method `resetStartTime()` (use `resetRuntime()` instead).
* Deletes the protected `LinearOpMode.LinearOpModeHelper` class (which was not meant for use by OpModes).
* Removes I2C Device (Synchronous) config type (deprecated since 2018)

### Enhancements
* Uncaught exceptions in OpModes no longer require a Restart Robot
  * A blue screen popping up with a stacktrace is not an SDK error; this replaces the red text in the telemetry area.
  * Since the very first SDK release, OpMode crashes have put the robot into "EMERGENCY STOP" state, only showing the first line of the exception, and requiring the user to press "Restart Robot" to continue
  * Exceptions during an OpMode now open a popup window with the same color scheme as the log viewer, containing 15 lines of the exception stacktrace to allow easily tracing down the offending line without needing to connect to view logs over ADB or scroll through large amounts of logs in the log viewer.
  * The exception text in the popup window is both zoomable and scrollable just like a webpage.
  * Pressing the "OK" button in the popup window will return to the main screen of the Driver Station and allow an OpMode to be run again immediately, without the need to perform a "Restart Robot"
* Adds new Java sample to demonstrate using a hardware class to abstract robot actuators, and share them across multiple OpModes.
  * Sample OpMode is [ConceptExternalHardwareClass.java](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptExternalHardwareClass.java)
  * Abstracted hardware class is [RobotHardware.java](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotHardware.java)
* Updates RobotAutoDriveByGyro_Linear Java sample to use REV Control/Expansion hub IMU.
* Updates Vuforia samples to reference PowerPlay assets and have correct names and field locations of image targets.
* Updates TensorFlow samples to reference PowerPlay assets.
* Adds opt-in support for Java 8 language features to the OnBotJava editor.
  * To opt in, open the OnBotJava Settings, and check `Enable beta Java 8 support`.
  * Note that Java 8 code will only compile when the Robot Controller runs Android 7.0 Nougat or later.
  * Please report issues [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues).
* In OnBotJava, clicking on build errors now correctly jumps to the correct location.
* Improves OnBotJava autocomplete behavior, to provide better completion options in most cases.
* Adds a QR code to the Robot Controller Inspection Report when viewed from the Driver Station for scanning by inspectors at competition.
* Improves I2C performance and reliability in some scenarios.

## Version 7.2 (20220723-130006)

### Breaking Changes
* Updates the build tooling.  For Android Studio users, this change requires Android Studio Chipmunk 2021.2.1.
* Removes support for devices that are not competition legal, including Modern Robotics Core Control Modules, the Matrix Controller, and HiTechnic/NXT controllers and sensors.  Support remains for Modern Robotics I2C sensors.

### Enhancements
* Increases the height of the 3-dots Landscape menu touch area on the Driver Station, making it much easier to select.
* Adds `terminateOpModeNow()` method to allow OpModes to cleanly self-exit immediately.
* Adds `opModeInInit()` method to `LinearOpMode` to facilitate init-loops. Similar to `opModeIsActive()` but for the init phase.
* Warns user if they have a Logitech F310 gamepad connected that is set to DirectInput mode.
* Allows SPARKmini motor controllers to react more quickly to speed changes.
* Hides the version number of incorrectly installed sister app (i.e. DS installed on RC device or vice-versa) on inspection screen.
* Adds support for allowing the user to edit the comment for the runOpMode block.
* Adds parameterDefaultValues field to @ExportToBlocks. This provides the ability for a java method with an @ExportToBlocks annotation to specify default values for method parameters when it is shown in the block editor.
* Make LinearOpMode blocks more readable. The opmode name is displayed on the runOpMode block, but not on the other LinearOpMode blocks.
* Added support to TensorFlow Object Detection for using a different frame generator, instead of Vuforia.
  Using Vuforia to pass the camera frame to TFOD is still supported.
* Removes usage of Renderscript.
* Fixes logspam on app startup of repeated stacktraces relating to `"Failed resolution of: Landroid/net/wifi/p2p/WifiP2pManager$DeviceInfoListener"`
* Allows disabling bluetooth radio from inspection screen
* Improves warning messages when I2C devices are not responding
* Adds support for controlling the RGB LED present on PS4/Etpark gamepads from OpModes
* Removes legacy Pushbot references from OpMode samples.  Renames "Pushbot" samples to "Robot".  Motor directions reversed to be compatible with "direct Drive" drive train.


### Bug fixes
* Fixes [issue #316](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/316) (MatrixF.inverted() returned an incorrectly-sized matrix for 1x1 and 2x2 matrixes).
* Self inspect now allows for Driver Station and Robot Controller compatibility between point releases.
* Fixes bug where if the same `RumbleEffect` object instance was queued for multiple gamepads, it
  could happen that both rumble commands would be sent to just one gamepad.
* Fixes bug in Driver Station where on the Driver Hub, if Advanced Gamepad Features was disabled and
  an officially supported gamepad was connected, then opening the Advanced Gamepad Features or
  Gamepad Type Overrides screens would cause the gamepad to be rebound by the custom USB driver even
  though advanced gamepad features was disabled.
* Protects against (unlikely) null pointer exception in Vuforia Localizer.
* Harden OnBotJava and Blocks saves to protect against save issues when disconnecting from Program and Manage
* Fixes issue where the RC app would hang if a REV Hub I2C write failed because the previous I2C
  operation was still in progress. This hang most commonly occurred during REV 2M Distance Sensor initialization
* Removes ConceptWebcam.java sample program.  This sample is not compatible with OnBotJava.
* Fixes bug where using html tags in an @ExportToBlocks comment field prevented the blocks editor from loading.
* Fixes blocks editor so it doesn't ask you to save when you haven't modified anything.
* Fixes uploading a very large blocks project to offline blocks editor.
* Fixes bug that caused blocks for DcMotorEx to be omitted from the blocks editor toolbox.
* Fixes [Blocks Programs Stripped of Blocks (due to using TensorFlow Label block)](https://ftcforum.firstinspires.org/forum/ftc-technology/blocks-programming/87035-blocks-programs-stripped-of-blocks)

## Version 7.1 (20211223-120805)

* Fixes crash when calling `isPwmEnabled()` ([issue #223](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/233)).
* Fixes lint error ([issue #4](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/4)).
* Fixes Driver Station crash when attempting to use DualShock4 v1 gamepad with Advanced Gamepad Features enabled ([issue #173](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/173)).
* Fixes possible (but unlikely) Driver Station crash when connecting gamepads of any type.
* Fixes bug where Driver Station would use generic 20% deadzone for Xbox360 and Logitech F310 gamepads when Advanced Gamepad Features was disabled.
* Added SimpleOmniDrive sample OpMode.
* Adds UVC white balance control API.
* Fixes [issue #259](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/259) Most blocks samples for TensorFlow can't be used for a different model.
    * The blocks previously labeled TensorFlowObjectDetectionFreightFrenzy (from the subcategory named "Optimized for Freight Frenzy") and TensorFlowObjectDetectionCustomModel (from the subcategory named "Custom Model") have been replaced with blocks labeled TensorFlowObjectDetection. Blocks in existing opmodes will be automatically updated to the new blocks when opened in the blocks editor.
* Fixes [issue #260](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/260) Blocks can't call java method that has a VuforiaLocalizer parameter.
    * Blocks now has a block labeled VuforiaFreightFrenzy.getVuforiaLocalizer for this.
* Added a page to manage the TensorFlow Lite models in /sdcard/FIRST/tflitemodels. To get to the TFLite Models page:
    * You can click on the link at the bottom of the Manage page.
    * You can click on the link at the upper-right the Blocks project page.
* Fixes logspam when `isBusy()` is called on a motor not in RTP mode.
* Hides the "RC Password" item on the inspection screen for phone-based Robot Controllers. (It is only applicable for Control Hubs).
* Adds channel 165 to Wi-Fi Direct channel selection menu in the settings screen. (165 was previously available through the web UI, but not locally in the app).

## Version 7.0 (20210915-141025)

### Enhancements and New Features
* Adds support for external libraries to OnBotJava and Blocks.
    * Upload .jar and .aar files in OnBotJava.
      * Known limitation - RobotController device must be running Android 7.0 or greater.
      * Known limitation - .aar files with assets are not supported.
    * External libraries can provide support for hardware devices by using the annotation in the
      com.qualcomm.robotcore.hardware.configuration.annotations package.
    * External libraries can include .so files for native code.
    * External libraries can be used from OnBotJava OpModes.
    * External libraries that use the following annotations can be used from Blocks OpModes.
      * org.firstinspires.ftc.robotcore.external.ExportClassToBlocks
      * org.firstinspires.ftc.robotcore.external.ExportToBlocks
    * External libraries that use the following annotations can add new hardware devices:
      * com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType
      * com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
      * com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType
      * com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
      * com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
      * com.qualcomm.robotcore.hardware.configuration.annotations.ServoType
    * External libraries that use the following annotations can add new functionality to the Robot Controller:
      * org.firstinspires.ftc.ftccommon.external.OnCreate
      * org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
      * org.firstinspires.ftc.ftccommon.external.OnCreateMenu
      * org.firstinspires.ftc.ftccommon.external.OnDestroy
      * org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
* Adds support for REV Robotics Driver Hub.
* Adds fully custom userspace USB gamepad driver to Driver Station (see "Advanced Gamepad Features" menu in DS settings).
    * Allows gamepads to work on devices without native Linux kernel support (e.g. some Romanian Motorola devices).
    * Allows the DS to read the unique serial number of each gamepad, enabling auto-recovery of dropped gamepads even if two gamepads of the same model drop. *(NOTE: unfortunately this does not apply to Etpark gamepads, because they do not have a unique serial)*.
    * Reading the unique serial number also provides the ability to configure the DS to assign gamepads to a certain position by default (so no need to do start+a/b at all).
    * The LED ring on the Xbox360 gamepad and the RGB LED bar on the PS4 gamepad is used to indicate the driver position the gamepad is bound to.
    * The rumble motors on the Xbox360, PS4, and Etpark gamepads can be controlled from OpModes.
    * The 2-point touchpad on the PS4 gamepad can be read from OpModes.
    * The "back" and "guide" buttons on the gamepad can now be safely bound to robot controls (Previously, on many devices, Android would intercept these buttons as home button presses and close the app).
    * Advanced Gamepad features are enabled by default, but may be disabled through the settings menu in order to revert to gamepad support provided natively by Android.
* Improves accuracy of ping measurement.
    * Fixes issue where the ping time showed as being higher than reality when initially connecting to or restarting the robot.
    * To see the full improvement, you must update both the Robot Controller and Driver Station apps.
* Updates samples located at [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples).
    * Added ConceptGamepadRumble and ConceptGamepadTouchpad samples to illustrate the use of these new gampad capabilities.
    * Condensed existing Vuforia samples into just 2 samples (ConceptVuforiaFieldNavigation & ConceptVuforiaFieldNavigationWebcam) showing how to determine the robot's location on the field using Vuforia. These both use the current season's Target images.
    * Added ConceptVuforiaDriveToTargetWebcam to illustrate an easy way to drive directly to any visible Vuforia target.
* Makes many improvements to the warning system and individual warnings.
    * Warnings are now much more spaced out, so that they are easier to read.
    * New warnings were added for conditions that should be resolved before competing.
    * The mismatched apps warning now uses the major and minor app versions, not the version code.
    * The warnings are automatically re-enabled when a Robot Controller app from a new FTC season is installed.
* Adds support for I2C transactions on the Expansion Hub / Control Hub without specifying a register address.
    * See section 3 of the [TI I2C spec](https://www.ti.com/lit/an/slva704/slva704.pdf).
    * Calling these new methods when using Modern Robotics hardware will result in an UnsupportedOperationException.
* Changes VuforiaLocalizer `close()` method to be public.
* Adds support for TensorFlow v2 object detection models.
* Reduces ambiguity of the Self Inspect language and graphics.
* OnBotJava now warns about potentially unintended file overwrites.
* Improves behavior of the Wi-Fi band and channel selector on the Manage webpage.

### Bug fixes
 * Fixes Robot Controller app crash on Android 9+ when a Driver Station connects.
 * Fixes issue where an OpMode was responsible for calling shutdown on the
   TensorFlow TFObjectDetector. Now this is done automatically.
 * Fixes Vuforia initialization blocks to allow user to chose AxesOrder. Updated
   relevant blocks sample opmodes.
 * Fixes [FtcRobotController issue #114](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/114)
   LED blocks and Java class do not work.
 * Fixes match logging for OpModes that contain special characters in their names.
 * Fixes Driver Station OpMode controls becoming unresponsive if the Driver Station was set to the landscape layout and an OnBotJava build was triggered while an OpMode was running.
 * Fixes the Driver Station app closing itself when it is switched away from, or the screen is turned off.
 * Fixes "black swirl of doom" (Infinite "configuring Wi-Fi Direct" message) on older devices.
 * Updates the wiki comment on the OnBotJava intro page.

## Version 6.2 (20210218-074821)

### Enhancements
* Attempts to automatically fix the condition where a Control Hub's internal Expansion Hub is not
  working by re-flashing its firmware
* Makes various improvements to the Wi-Fi Direct pairing screen, especially in landscape mode
* Makes the Robot Controller service no longer be categorically restarted when the main activity is brought to foreground
    * (e.g. the service is no longer restarted simply by viewing the Self Inspect screen and pressing the back button)
    * It is still restarted if the Settings menu or Configure Robot menu is opened


### Bug fixes
* Fixes [FtcRobotController issue #71](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/71)
  Cannot open OpModes in v6.1 Blocks offline editor
* Fixes [FtcRobotController issue #79](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/79)
  6.1 causes a soft reboot on the Motorola E5 Play
* Fixes issue where the Control Hub OS's watchdog would restart the Robot Controller app if
  the Control Hub was not able to communicate with its internal Expansion Hub
* Fixes certain I2C devices not showing up in the appropriate `HardwareMap` fields (such as `hardwareMap.colorSensor`)
* Fixes issue where performing a Wi-Fi factory reset on the Control Hub would not set the Wi-Fi band to 2.4 GHz
* Fixes issue where OnBotJava might fail to create a new file if the option to "Setup Code for Configured Hardware" was selected
* Fixes issue where performing certain operations after an OpMode crashes would temporarily break Control/Expansion Hub communication
* Fixes issue where a Control Hub with a configured USB-connected Expansion Hub would not work if the Expansion Hub was missing at startup
* Fixes potential issues caused by having mismatched Control/Expansion Hub firmware versions
* Fixes [ftc_app issue 673](https://github.com/ftctechnh/ftc_app/issues/673) Latest matchlog is being deleted instead of old ones by RobotLog
* Fixes ConceptVuforiaUltimateGoalNavigationWebcam sample opmode by correctly orienting camera on robot.
* Fixes issue where logcat would be spammed with InterruptedExceptions when stop is requested from the Driver Station (this behavior was accidentally introduced in v5.3). This change has no impact on functionality.
* Fixes issue where the blocks editor fails to load if the name of any TeleOp opmode contains an apostrophe.

## Version 6.1 (20201209-113742)
* Makes the scan button on the configuration screen update the list of Expansion Hubs connected via RS-485
    * Fixes [SkyStone issue #143](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/143)
* Improves web interface compatibility with older browser and Android System WebView versions.
* Fixes issue in UVC driver where some cameras (e.g. certain MS Lifecams) which reported frame intervals as rounded rather than truncated values (e.g. `666667*100ns` instead of `666666*100ns` for 15FPS) would fail to start streaming.
* Adds support in UVC driver for virtual PTZ control
* Adds support in UVC driver for gain (ISO) control
* Adds support in UVC driver for enabling/disable AE priority. This setting provides a means to tell the camera firmware either
    * A) It can undershoot the requested frame rate in order to provide a theoretically better image (i.e. with a longer exposure than the inter-frame period of the selected frame rate allows)
    * B) It *must* meet the inter-frame deadline for the selected frame rate, even if the image may be underexposed as a result
* Adds support for the Control Hub OS 1.1.2 Robot Controller watchdog
    * The Robot Controller app will be restarted if it stops responding for more than 10 seconds
* Adds support for using the Driver Station app on Android 10+
* Introduces an automatic TeleOp preselection feature
    * For details and usage guide, please see [this wiki entry](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Automatically-Loading-a-Driver-Controlled-Op-Mode)
* Shows icon next to OpMode name in the OpMode list dropdown on the Driver Station to indicate the source of the OpMode (i.e. the programming tool used to create it)
* Fixes issue where the Driver Station app would exit after displaying the Configuring Wi-Fi Direct screen
* Fixes Blocks and OnBotJava prompts when accessed via the REV Hardware Client

## Version 6.0 (20200921-085816)

### Important Notes
* Version 6.0 is the version for the Ultimate Goal season.
* Requires Android Studio 4.0.
* Android Studio users need to be connected to the Internet the first time they build the app (in order to download needed packages for the build).
* Version 5.5 was a moderately large off-season, August 2020, drop.  It's worth reviewing those release notes below also.
* Version 5.5 and greater will not work on older Android 4.x and 5.x phones.  Users must upgrade to an approved Android 6.x device or newer.
* The default PIDF values for REV motors have been reverted to the default PID values that were used in the 2018-2019 season
    * This change was made because the 2018-2019 values turned out to work better for many mechanisms
    * This brings the behavior of the REV motors in line with the behavior of all other motors
    * If you prefer the 2019-2020 season's behavior for REV motors, here are the PIDF values that were in place, so that you can manually set them in your OpModes:
      <br>
      **HD Hex motors (all gearboxes):**
      Velocity PIDF values: `P = 1.17`, `I = 0.117`, `F = 11.7`
      Position PIDF values: `P = 5.0`
      **Core Hex motor:**
      Velocity PIDF values: `P = 4.96`, `I = 0.496`, `F = 49.6`
      Position PIDF values: `P = 5.0`

### New features
* Includes TensorFlow inference model and sample OpModes to detect Ultimate Goal Starter Stacks (four rings vs single ring stack).
* Includes Vuforia Ultimate Goal vision targets and sample OpModes.
* Introduces a digital zoom feature for TensorFlow object detection (to detect objects more accurately at greater distances).
* Adds configuration entry for the REV UltraPlanetary HD Hex motor

### Enhancements
* Adds setGain() and getGain() methods to the NormalizedColorSensor interface
    * By setting the gain of a color sensor, you can adjust for different lighting conditions.
      For example, if you detect lower color values than expected, you can increase the gain.
    * The gain value is only applied to the argb() and getNormalizedColors() methods, not to the raw color methods.
      The getNormalizedColors() method is recommended for ease-of-use and clarity, since argb() has to be converted.
    * Updates SensorColor Java sample to demonstrate gain usage
* Merges SensorREVColorDistance Java sample into SensorColor Java sample, which showcases best practices for all color sensors
* Improves retrieving values from the REV Color Sensor V3
    * Updates the normalization calculation of the RGB channels
    * Improves the calculation of the alpha channel (can be used as an overall brightness indicator)
    * Fixes the default sensor resolution, which caused issues with bright environments
    * Adds support for changing the resolution and measuring rate of the Broadcom sensor chip
    * Removes IR readings and calculations not meant for the Broadcom sensor chip

### Bug fixes
* Improves reliability of BNO055IMU IMU initialization to prevent random initialization failures (which manifested as `Problem with 'imu'`).

## Version 5.5 (20200824-090813)

Version 5.5 requires Android Studio 4.0 or later.

### New features
* Adds support for calling custom Java classes from Blocks OpModes (fixes [SkyStone issue #161](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/161)).
    * Classes must be in the org.firstinspires.ftc.teamcode package.
    * To have easy access to the opMode, hardwareMap, telemetry, gamepad1, and gamepad2, classes can
      extends org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.
    * Methods must be public static and have no more than 21 parameters.
    * Methods must be annotated with org.firstinspires.ftc.robotcore.external.ExportToBlocks.
    * Parameters declared as OpMode, LinearOpMode, Telemetry, and HardwareMap are supported and the
      argument is provided automatically, regardless of the order of the parameters. On the block,
      the sockets for those parameters are automatically filled in.
    * Parameters declared as char or java.lang.Character will accept any block that returns text
      and will only use the first character in the text.
    * Parameters declared as boolean or java.lang.Boolean will accept any block that returns boolean.
    * Parameters declared as byte, java.lang.Byte, short, java.lang.Short, int, java.lang.Integer,
      long, or java.lang.Long,  will accept any block that returns a number and will round that
      value to the nearest whole number.
    * Parameters declared as float, java.lang.Float, double, java.lang.Double will accept any
      block that returns a number.
* Adds telemetry API method for setting display format
    * Classic
    * Monospace
    * HTML (certain tags only)
* Adds blocks support for switching cameras.
* Adds Blocks support for TensorFlow Object Detection with a custom model.
* Adds support for uploading a custom TensorFlow Object Detection model in the Manage page, which
  is especially useful for Blocks and OnBotJava users.
* Shows new Control Hub blink codes when the Wi-Fi band is switched using the Control Hub's button (only possible on Control Hub OS 1.1.2)
* Adds new warnings which can be disabled in the Advanced RC Settings
    * Mismatched app versions warning
    * Unnecessary 2.4 GHz Wi-Fi usage warning
    * REV Hub is running outdated firmware (older than version 1.8.2)
* Adds support for Sony PS4 gamepad, and reworks how gamepads work on the Driver Station
    * Removes preference which sets gamepad type based on driver position. Replaced with menu which allows specifying type for gamepads with unknown VID and PID
	* Attempts to auto-detect gamepad type based on USB VID and PID
	* If gamepad VID and PID is not known, use type specified by user for that VID and PID
	* If gamepad VID and PID is not known AND the user has not specified a type for that VID and PID, an educated guess is made about how to map the gamepad
* Driver Station will now attempt to automatically recover from a gamepad disconnecting, and re-assign it to the position it was assigned to when it dropped
    * If only one gamepad is assigned and it drops: it can be recovered
    * If two gamepads are assigned, and have **different** VID/PID signatures, and only one drops: it will be recovered
    * If two gamepads are assigned, and have **different** VID/PID signatures, and BOTH drop: both will be recovered
    * If two gamepads are assigned, and have **the same** VID/PID signatures, and only one drops: it will be recovered
    * If two gamepads are assigned, and have **the same** VID/PID signatures, and BOTH drop: **neither** will be recovered, because of the ambiguity of the gamepads when they re-appear on the USB bus.
    * There is currently one known edge case: if there are **two** gamepads with **the same** VID/PID signature plugged in, **but only one is assigned**, and they BOTH drop, it's a 50-50 chance of which one will be chosen for automatic recovery to the assigned position: it is determined by whichever one is re-enumerated first by the USB bus controller.
* Adds landscape user interface to Driver Station
    * New feature: practice timer with audio cues
    * New feature (Control Hub only): wireless network connection strength indicator (0-5 bars)
    * New feature (Control Hub only): tapping on the ping/channel display will switch to an alternate display showing radio RX dBm and link speed (tap again to switch back)
    * The layout will NOT autorotate. You can switch the layout from the Driver Station's settings menu.
### Breaking changes
* Removes support for Android versions 4.4 through 5.1 (KitKat and Lollipop). The minSdkVersion is now 23.
* Removes the deprecated `LinearOpMode` methods `waitOneFullHardwareCycle()` and `waitForNextHardwareCycle()`
### Enhancements
* Handles RS485 address of Control Hub automatically
    * The Control Hub is automatically given a reserved address
    * Existing configuration files will continue to work
    * All addresses in the range of 1-10 are still available for Expansion Hubs
    * The Control Hub light will now normally be solid green, without blinking to indicate the address
    * The Control Hub will not be shown on the Expansion Hub Address Change settings page
* Improves REV Hub firmware updater
    * The user can now choose between all available firmware update files
    * Version 1.8.2 of the REV Hub firmware is bundled into the Robot Controller app.
    * Text was added to clarify that Expansion Hubs can only be updated via USB.
    * Firmware update speed was reduced to improve reliability
    * Allows REV Hub firmware to be updated directly from the Manage webpage
* Improves log viewer on Robot Controller
    * Horizontal scrolling support (no longer word wrapped)
    * Supports pinch-to-zoom
    * Uses a monospaced font
    * Error messages are highlighted
    * New color scheme
* Attempts to force-stop a runaway/stuck OpMode without restarting the entire app
    * Not all types of runaway conditions are stoppable, but if the user code attempts to talk to hardware during the runaway, the system should be able to capture it.
* Makes various tweaks to the Self Inspect screen
    * Renames "OS version" entry to "Android version"
    * Renames "Wi-Fi Direct Name" to "Wi-Fi Name"
    * Adds Control Hub OS version, when viewing the report of a Control Hub
    * Hides the airplane mode entry, when viewing the report of a Control Hub
    * Removes check for ZTE Speed Channel Changer
    * Shows firmware version for **all** Expansion and Control Hubs
* Reworks network settings portion of Manage page
    * All network settings are now applied with a single click
    * The Wi-Fi Direct channel of phone-based Robot Controllers can now be changed from the Manage page
    * Wi-Fi channels are filtered by band (2.4 vs 5 GHz) and whether they overlap with other channels
    * The current Wi-Fi channel is pre-selected on phone-based Robot Controllers, and Control Hubs running OS 1.1.2 or later.
    * On Control Hubs running OS 1.1.2 or later, you can choose to have the system automatically select a channel on the 5 GHz band
* Improves OnBotJava
    * New light and dark themes replace the old themes (chaos, github, chrome,...)
        * the new default theme is `light` and will be used when you first update to this version
    * OnBotJava now has a tabbed editor
    * Read-only offline mode
* Improves function of "exit" menu item on Robot Controller and Driver Station
    * Now guaranteed to be fully stopped and unloaded from memory
* Shows a warning message if a LinearOpMode exists prematurely due to failure to monitor for the start condition
* Improves error message shown when the Driver Station and Robot Controller are incompatible with each other
* Driver Station OpMode Control Panel now disabled while a Restart Robot is in progress
* Disables advanced settings related to Wi-Fi Direct when the Robot Controller is a Control Hub.
* Tint phone battery icons on Driver Station when low/critical.
* Uses names "Control Hub Portal" and "Control Hub" (when appropriate) in new configuration files
* Improve I2C read performance
    * Very large improvement on Control Hub; up to ~2x faster with small (e.g. 6 byte) reads
    * Not as apparent on Expansion Hubs connected to a phone
* Update/refresh build infrastructure
    * Update to 'androidx' support library from 'com.android.support:appcompat', which is end-of-life
    * Update targetSdkVersion and compileSdkVersion to 28
    * Update Android Studio's Android plugin to latest
    * Fix reported build timestamp in 'About' screen
* Add sample illustrating manual webcam use: ConceptWebcam


### Bug fixes
* Fixes [SkyStone issue #248](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/248)
* Fixes [SkyStone issue #232](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/232) and
  modifies bulk caching semantics to allow for cache-preserving MANUAL/AUTO transitions.
* Improves performance when REV 2M distance sensor is unplugged
* Improves readability of Toast messages on certain devices
* Allows a Driver Station to connect to a Robot Controller after another has disconnected
* Improves generation of fake serial numbers for UVC cameras which do not provide a real serial number
    * Previously some devices would assign such cameras a serial of `0:0` and fail to open and start streaming
	* Fixes [ftc_app issue #638](https://github.com/ftctechnh/ftc_app/issues/638).
* Fixes a slew of bugs with the Vuforia camera monitor including:
    * Fixes bug where preview could be displayed with a wonky aspect ratio
    * Fixes bug where preview could be cut off in landscape
    * Fixes bug where preview got totally messed up when rotating phone
    * Fixes bug where crosshair could drift off target when using webcams
* Fixes issue in UVC driver on some devices ([ftc_app 681](https://github.com/ftctechnh/ftc_app/issues/681)) if streaming was started/stopped multiple times in a row
    * Issue manifested as kernel panic on devices which do not have [this kernel patch](https://lore.kernel.org/patchwork/patch/352400/).
    * On affected devices which **do** have the patch, the issue was manifest as simply a failure to start streaming.
    * The Tech Team believes that the root cause of the issue is a bug in the Linux kernel XHCI driver. A workaround was implemented in the SDK UVC driver.
* Fixes bug in UVC driver where often half the frames from the camera would be dropped (e.g. only 15FPS delivered during a streaming session configured for 30FPS).
* Fixes issue where TensorFlow Object Detection would show results whose confidence was lower than
  the minimum confidence parameter.
* Fixes a potential exploitation issue of [CVE-2019-11358](https://www.cvedetails.com/cve/CVE-2019-11358/) in OnBotJava
* Fixes changing the address of an Expansion Hub with additional Expansion Hubs connected to it
* Preserves the Control Hub's network connection when "Restart Robot" is selected
* Fixes issue where device scans would fail while the Robot was restarting
* Fix RenderScript usage
    * Use androidx.renderscript variant: increased compatibility
    * Use RenderScript in Java mode, not native: simplifies build
* Fixes webcam-frame-to-bitmap conversion problem: alpha channel wasn't being initialized, only R, G, & B
* Fixes possible arithmetic overflow in Deadline
* Fixes deadlock in Vuforia webcam support which could cause 5-second delays when stopping OpMode

## Version 5.4 (20200108-101156)
* Fixes [SkyStone issue #88](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/88)
* Adds an inspection item that notes when a robot controller (Control Hub) is using the factory default password.
* Fixes [SkyStone issue #61](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/61)
* Fixes [SkyStone issue #142](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/142)
* Fixes [ftc_app issue #417](https://github.com/ftctechnh/ftc_app/issues/417) by adding more current and voltage monitoring capabilities for REV Hubs.
* Fixes [a crash sometimes caused by OnBotJava activity](https://ftcforum.firstinspires.org/forum/ftc-technology/76217-onbotjava-crashes-robot-controller)
* Improves OnBotJava autosave functionality [ftc_app #738](https://github.com/ftctechnh/ftc_app/issues/738)
* Fixes system responsiveness issue when an Expansion Hub is disconnected
* Fixes issue where IMU initialization could prevent OpModes from stopping
* Fixes issue where AndroidTextToSpeech.speak() would fail if it was called too early
* Adds telemetry.speak() methods and blocks, which cause the Driver Station (if also updated) to speak text
* Adds and improves Expansion Hub-related warnings
    * Improves Expansion Hub low battery warning
        * Displays the warning immediately after the hub reports it
        * Specifies whether the condition is current or occurred temporarily during an OpMode run
        * Displays which hubs reported low battery
    * Displays warning when hub loses and regains power during an OpMode run
        * Fixes the hub's LED pattern after this condition
    * Displays warning when Expansion Hub is not responding to commands
        * Specifies whether the condition is current or occurred temporarily during an OpMode run
    * Clarifies warning when Expansion Hub is not present at startup
        * Specifies that this condition requires a Robot Restart before the hub can be used.
        * The hub light will now accurately reflect this state
    * Improves logging and reduces log spam during these conditions
* Syncs the Control Hub time and timezone to a connected web browser programming the robot, if a Driver Station is not available.
* Adds bulk read functionality for REV Hubs
  * A bulk caching mode must be set at the Hub level with `LynxModule#setBulkCachingMode()`. This applies to all relevant SDK hardware classes that reference that Hub.
  * The following following Hub bulk caching modes are available:
    * `BulkCachingMode.OFF` (default): All hardware calls operate as usual. Bulk data can read through `LynxModule#getBulkData()` and processed manually.
    * `BulkCachingMode.AUTO`: Applicable hardware calls are served from a bulk read cache that is cleared/refreshed automatically to ensure identical commands don't hit the same cache. The cache can also be cleared manually with `LynxModule#clearBulkCache()`, although this is not recommended.
    * (advanced users) `BulkCachingMode.MANUAL`: Same as `BulkCachingMode.AUTO` except the cache is never cleared automatically. To avoid getting stale data, the cache must be manually cleared at the beginning of each loop body or as the user deems appropriate.
* Removes PIDF Annotation values added in Rev 5.3 (to AndyMark, goBILDA and TETRIX motor configurations).
  * The new motor types will still be available but their Default control behavior will revert back to Rev 5.2
* Adds new `ConceptMotorBulkRead` sample Opmode to demonstrate and compare Motor Bulk-Read modes for reducing I/O latencies.

## Version 5.3 (20191004-112306)
* Fixes external USB/UVC webcam support
* Makes various bugfixes and improvements to Blocks page, including but not limited to:
    * Many visual tweaks
    * Browser zoom and window resize behave better
    * Resizing the Java preview pane works better and more consistently across browsers
    * The Java preview pane consistently gets scrollbars when needed
    * The Java preview pane is hidden by default on phones
    * Internet Explorer 11 should work
    * Large dropdown lists display properly on lower res screens
    * Disabled buttons are now visually identifiable as disabled
    * A warning is shown if a user selects a TFOD sample, but their device is not compatible
    * Warning messages in a Blocks OpMode are now visible by default.
* Adds goBILDA 5201 and 5202 motors to Robot Configurator
* Adds PIDF Annotation values to AndyMark, goBILDA and TETRIX motor configurations.
    This has the effect of causing the RUN_USING_ENCODERS and RUN_TO_POSITION modes to use
    PIDF vs PID closed loop control on these motors.  This should provide more responsive, yet stable, speed control.
    PIDF adds Feedforward control to the basic PID control loop.
    Feedforward is useful when controlling a motor's speed because it "anticipates" how much the control voltage
    must change to achieve a new speed set-point, rather than requiring the integrated error to change sufficiently.
    The PIDF values were chosen to provide responsive, yet stable, speed control on a lightly loaded motor.
    The more heavily a motor is loaded (drag or friction), the more noticable the PIDF improvement will be.
* Fixes startup crash on Android 10
* Fixes [ftc_app issue #712](https://github.com/ftctechnh/ftc_app/issues/712) (thanks to FROGbots-4634)
* Fixes [ftc_app issue #542](https://github.com/ftctechnh/ftc_app/issues/542)
* Allows "A" and lowercase letters when naming device through RC and DS apps.

## Version 5.2 (20190905-083277)
* Fixes extra-wide margins on settings activities, and placement of the new configuration button
* Adds Skystone Vuforia image target data.
   * Includes sample Skystone Vuforia Navigation OpModes (Java).
   * Includes sample Skystone Vuforia Navigation OpModes (Blocks).
* Adds TensorFlow inference model (.tflite) for Skystone game elements.
   * Includes sample Skystone TensorFlow OpModes (Java).
   * Includes sample Skystone TensorFlow OpModes (Blocks).
* Removes older (season-specific) sample OpModes.
* Includes 64-bit support (to comply with [Google Play requirements](https://android-developers.googleblog.com/2019/01/get-your-apps-ready-for-64-bit.html)).
* Protects against Stuck OpModes when a Restart Robot is requested. (Thanks to FROGbots-4634) ([ftc_app issue #709](https://github.com/ftctechnh/ftc_app/issues/709))
* Blocks related changes:
   * Fixes bug with blocks generated code when hardware device name is a java or javascript reserved word.
   * Shows generated java code for blocks, even when hardware items are missing from the active configuration.
   * Displays warning icon when outdated Vuforia and TensorFlow blocks are used ([SkyStone issue #27](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/27))

## Version 5.1 (20190820-222104)
* Defines default PIDF parameters for the following motors:
    * REV Core Hex Motor
    * REV 20:1 HD Hex Motor
    * REV 40:1 HD Hex Motor
* Adds back button when running on a device without a system back button (such as a Control Hub)
* Allows a REV Control Hub to update the firmware on a REV Expansion Hub via USB
* Fixes [SkyStone issue #9](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/9)
* Fixes [ftc_app issue #715](https://github.com/ftctechnh/ftc_app/issues/715)
* Prevents extra DS User clicks by filtering based on current state.
* Prevents incorrect DS UI state changes when receiving new OpMode list from RC
* Adds support for REV Color Sensor V3
* Adds a manual-refresh DS Camera Stream for remotely viewing RC camera frames.
    * To show the stream on the DS, initialize **but do not run** a stream-enabled opmode, select the Camera Stream option in the DS menu, and tap the image to refresh. This feature is automatically enabled when using Vuforia or TFOD—no additional RC configuration is required for typical use cases. To hide the stream, select the same menu item again.
    * Note that gamepads are disabled and the selected opmode cannot be started while the stream is open as a safety precaution.
    * To use custom streams, consult the API docs for `CameraStreamServer#setSource` and `CameraStreamSource`.
* Adds many Star Wars sounds to RobotController resources.
* Added Skystone Sounds Chooser Sample Program.
* Switches out startup, connect chimes, and error/warning sounds for Star Wars sounds
* Updates OnBot Java to use a WebSocket for communication with the robot
    * The OnBot Java page no longer has to do a full refresh when a user switches from editing one file to another

Known issues:
* Camera Stream
    * The Vuforia camera stream inherits the issues present in the phone preview (namely [ftc_app issue #574](https://github.com/ftctechnh/ftc_app/issues/574)). This problem does not affect the TFOD camera stream even though it receives frames from Vuforia.
    * The orientation of the stream frames may not always match the phone preview. For now, these frames may be rotated manually via a custom `CameraStreamSource` if desired.
* OnBotJava
    * Browser back button may not always work correctly
    * It's possible for a build to be queued, but not started. The OnBot Java build console will display a warning if this occurs.
    * A user might not realize they are editing a different file if the user inadvertently switches from one file to another since this switch is now seamless. The name of the currently open file is displayed in the browser tab.

## Version 5.0 (built on 19.06.14)
 * Support for the REV Robotics Control Hub.
 * Adds a Java preview pane to the Blocks editor.
 * Adds a new offline export feature to the Blocks editor.
 * Display Wi-Fi channel in Network circle on Driver Station.
 * Adds calibration for Logitech C270
 * Updates build tooling and target SDK.
 * Compliance with Google's permissions infrastructure (Required after build tooling update).
 * Keep Alives to mitigate the Motorola Wi-Fi scanning problem.  Telemetry substitute no longer necessary.
 * Improves Vuforia error reporting.
 * Fixes ftctechnh/ftc_app issues 621, 713.
 * Miscellaneous bug fixes and improvements.

## Version 4.3 (built on 18.10.31)
 * Includes missing TensorFlow-related libraries and files.

## Version 4.2 (built on 18.10.30)
 * Includes fix to avoid deadlock situation with WatchdogMonitor which could result in USB communication errors.
     - Comm error appeared to require that user disconnect USB cable and restart the Robot Controller app to recover.
     - robotControllerLog.txt would have error messages that included the words "E RobotCore: lynx xmit lock: #### abandoning lock:"
 * Includes fix to correctly list the parent module address for a REV Robotics Expansion Hub in a configuration (.xml) file.
     - Bug in versions 4.0 and 4.1 would incorrect list the address module for a parent REV Robotics device as "1".
     - If the parent module had a higher address value than the daisy-chained module, then this bug would prevent the Robot Controller from communicating with the downstream Expansion Hub.
 * Added requirement for ACCESS_COARSE_LOCATION to allow a Driver Station running Android Oreo to scan for Wi-Fi Direct devices.
 * Added google() repo to build.gradle because aapt2 must be downloaded from the google() repository beginning with version 3.2 of the Android Gradle Plugin.
     - Important Note: Android Studio users will need to be connected to the Internet the first time build the ftc_app project.
     - Internet connectivity is required for the first build so the appropriate files can be downloaded from the Google repository.
     - Users should not need to be connected to the Internet for subsequent builds.
     - This should also fix buid issue where Android Studio would complain that it "Could not find com.android.tools.lint:lint-gradle:26.1.4" (or similar).
 * Added support for REV Spark Mini motor controller as part of the configuration menu for a servo/PWM port on the REV Expansion Hub.
 * Provide examples for playing audio files in an OpMode.
 * Block Development Tool Changes
     - Includes a fix for a problem with the Velocity blocks that were reported in the FTC Technology forum (Blocks Programming subforum).
     - Change the "Save completed successfully." message to a white color so it will contrast with a green background.
     - Fixed the "Download image" feature so it will work if there are text blocks in the OpMode.
 * Introduce support for Google's TensorFlow Lite technology for object detetion for 2018-2019 game.
     - TensorFlow lite can recognize Gold Mineral and Silver Mineral from 2018-2019 game.
     - Example Java and Block OpModes are included to show how to determine the relative position of the gold block (left, center, right).

## Version 4.1 (released on 18.09.24)

Changes include:
 * Fix to prevent crash when deprecated configuration annotations are used.
 * Change to allow FTC Robot Controller APK to be auto-updated using FIRST Global Control Hub update scripts.
 * Removed samples for non supported / non legal hardware.
 * Improvements to Telemetry.addData block with "text" socket.
 * Updated Blocks sample OpMode list to include Rover Ruckus Vuforia example.
 * Update SDK library version number.

## Version 4.0 (released on 18.09.12)

Changes include:
 * Initial support for UVC compatible cameras
    - If UVC camera has a unique serial number, RC will detect and enumerate by serial number.
    - If UVC camera lacks a unique serial number, RC will only support one camera of that type connected.
    - Calibration settings for a few cameras are included (see TeamCode/src/main/res/xml/teamwebcamcalibrations.xml for details).
    - User can upload calibration files from Program and Manage web interface.
    - UVC cameras seem to draw a fair amount of electrical current from the USB bus.
         + This does not appear to present any problems for the REV Robotics Control Hub.
	 + This does seem to create stability problems when using some cameras with an Android phone-based Robot Controller.
	 + FTC Tech Team is investigating options to mitigate this issue with the phone-based Robot Controllers.
    - Updated sample Vuforia Navigation and VuMark OpModes to demonstrate how to use an internal phone-based camera and an external UVC webcam.

 * Support for improved motor control.
    - REV Robotics Expansion Hub firmware 1.8 and greater will support a feed forward mechanism for closed loop motor control.
    - FTC SDK has been modified to support PIDF coefficients (proportional, integral, derivative, and feed forward).
    - FTC Blocks development tool modified to include PIDF programming blocks.
    - Deprecated older PID-related methods and variables.
    - REV's 1.8.x PIDF-related changes provide a more linear and accurate way to control a motor.

 * Wireless
    - Added 5GHz support for wireless channel changing for those devices that support it.
        + Tested with Moto G5 and E4 phones.
	+ Also tested with other (currently non-approved) phones such as Samsung Galaxy S8.

* Improved Expansion Hub firmware update support in Robot Controller app
    - Changes to make the system more robust during the firmware update process (when performed through Robot Controller app).
    - User no longer has to disconnect a downstream daisy-chained Expansion Hub when updating an Expansion Hub's firmware.
        + If user is updating an Expansion Hub's firmware through a USB connection, he/she does not have to disconnect RS485 connection to other Expansion Hubs.
	+ The user still must use a USB connection to update an Expansion Hub's firmware.
	+ The user cannot update the Expansion Hub firmware for a downstream device that is daisy chained through an RS485 connection.
    - If an Expansion Hub accidentally gets "bricked" the Robot Controller app is now more likely to recognize the Hub when it scans the USB bus.
        + Robot Controller app should be able to detect an Expansion Hub, even if it accidentally was bricked in a previous update attempt.
	+ Robot Controller app should be able to install the firmware onto the Hub, even if if accidentally was bricked in a previous update attempt.

 * Resiliency
    - FTC software can detect and enable an FTDI reset feature that is available with REV Robotics v1.8 Expansion Hub firmware and greater.
        + When enabled, the Expansion Hub can detect if it hasn't communicated with the Robot Controller over the FTDI (USB) connection.
	+ If the Hub hasn't heard from the Robot Controller in a while, it will reset the FTDI connection.
	+ This action helps system recover from some ESD-induced disruptions.
    - Various fixes to improve reliability of FTC software.

 * Blocks
    - Fixed errors with string and list indices in blocks export to java.
    - Support for USB connected UVC webcams.
    - Refactored optimized Blocks Vuforia code to support Rover Ruckus image targets.
    - Added programming blocks to support PIDF (proportional, integral, derivative and feed forward) motor control.
    - Added formatting options (under Telemetry and Miscellaneous categories) so user can set how many decimal places to display a numerical value.
    - Support to play audio files (which are uploaded through Blocks web interface) on Driver Station in addition to the Robot Controller.
    - Fixed bug with Download Image of Blocks feature.
    - Support for REV Robotics Blinkin LED Controller.
    - Support for REV Robotics 2m Distance Sensor.
    - Added support for a REV Touch Sensor (no longer have to configure as a generic digital device).
    - Added blocks for DcMotorEx methods.
        + These are enhanced methods that you can use when supported by the motor controller hardware.
	+ The REV Robotics Expansion Hub supports these enhanced methods.
	+ Enhanced methods include methods to get/set motor velocity (in encoder pulses per second), get/set PIDF coefficients, etc..

 * Modest Improvements in Logging
    - Decrease frequency of battery checker voltage statements.
    - Removed non-FTC related log statements (wherever possible).
    - Introduced a "Match Logging" feature.
        + Under "Settings" a user can enable/disable this feature (it's disabled by default).
	+ If enabled, user provides a "Match Number" through the Driver Station user interface (top of the screen).
	    * The Match Number is used to create a log file specifically with log statements from that particular OpMode run.
	    * Match log files are stored in /sdcard/FIRST/matlogs on the Robot Controller.
	    * Once an OpMode run is complete, the Match Number is cleared.
	    * This is a convenient way to create a separate match log with statements only related to a specific OpMode run.

 * New Devices
    - Support for REV Robotics Blinkin LED Controller.
    - Support for REV Robotics 2m Distance Sensor.
    - Added configuration option for REV 20:1 HD Hex Motor.
    - Added support for a REV Touch Sensor (no longer have to configure as a generic digital device).

 * Miscellaneous
    - Fixed some errors in the definitions for acceleration and velocity in our javadoc documentation.
    - Added ability to play audio files on Driver Station
    - When user is configuring an Expansion Hub, the LED on the Expansion Hub will change blink pattern (purple-cyan)  to indicate which Hub is currently being configured.
    - Renamed I2cSensorType to I2cDeviceType.
    - Added an external sample OpMode that demonstrates localization using 2018-2019 (Rover Ruckus presented by QualComm) Vuforia targets.
    - Added an external sample OpMode that demonstrates how to use the REV Robotics 2m Laser Distance Sensor.
    - Added an external sample OpMode that demonstrates how to use the REV Robotics Blinkin LED Controller.
    - Re-categorized external Java sample OpModes to "TeleOp" instead of "Autonomous".

Known issues:
 * Initial support for UVC compatible cameras
    - UVC cameras seem to draw significant amount of current from the USB bus.
        + This does not appear to present any problems for the REV Robotics Control Hub.
	+ This does seem to create stability problems when using some cameras with an Android phone-based Robot Controller.
	+ FTC Tech Team is investigating options to mitigate this issue with the phone-based Robot Controllers.
    - There might be a possible deadlock which causes the RC to become unresponsive when using a UVC webcam with a Nougat Android Robot Controller.

 * Wireless
    - When user selects a wireless channel, this channel does not necessarily persist if the phone is power cycled.
        + Tech Team is hoping to eventually address this issue in a future release.
	+ Issue has been present since apps were introduced (i.e., it is not new with the v4.0 release).
    - Wireless channel is not currently displayed for Wi-Fi Direct connections.

 * Miscellaneous
    - The blink indication feature that shows which Expansion Hub is currently being configured does not work for a newly created configuration file.
        + User has to first save a newly created configuration file and then close and re-edit the file in order for blink indicator to work.

## Version 3.6 (built on 17.12.18)

Changes include:
 * Blocks Changes
     - Uses updated Google Blockly software to allow users to edit their OpModes on Apple iOS devices (including iPad and iPhone).
     - Improvement in Blocks tool to handle corrupt OpMode files.
     - Autonomous OpModes should no longer get switched back to tele-op after re-opening them to be edited.
     - The system can now detect type mismatches during runtime and alert the user with a message on the Driver Station.
 * Updated javadoc documentation for setPower() method to reflect correct range of values (-1 to +1).
 * Modified VuforiaLocalizerImpl to allow for user rendering of frames
     - Added a user-overrideable onRenderFrame() method which gets called by the class's renderFrame() method.

## Version 3.5 (built on 17.10.30)

Changes with version 3.5 include:
 * Introduced a fix to prevent random OpMode stops, which can occur after the Robot Controller app has been paused and then resumed (for example, when a user temporarily turns off the display of the Robot Controller phone, and then turns the screen back on).
 * Introduced a fix to prevent random OpMode stops, which were previously caused by random peer disconnect events on the Driver Station.
 * Fixes issue where log files would be closed on pause of the RC or DS, but not re-opened upon resume.
 * Fixes issue with battery handler (voltage) start/stop race.
 * Fixes issue where Android Studio generated OpModes would disappear from available list in certain situations.
 * Fixes problem where OnBot Java would not build on REV Robotics Control Hub.
 * Fixes problem where OnBot Java would not build if the date and time on the Robot Controller device was "rewound" (set to an earlier date/time).
 * Improved error message on OnBot Java that occurs when renaming a file fails.
 * Removed unneeded resources from android.jar binaries used by OnBot Java to reduce final size of Robot Controller app.
 * Added MR_ANALOG_TOUCH_SENSOR block to Blocks Programming Tool.

## Version 3.4 (built on 17.09.06)

Changes with version 3.4 include:
 * Added telemetry.update() statement for BlankLinearOpMode template.
 * Renamed sample Block OpModes to be more consistent with Java samples.
 * Added some additional sample Block OpModes.
 * Reworded OnBot Java readme slightly.

## Version 3.3 (built on 17.09.04)

This version of the software includes improves for the FTC Blocks Programming Tool and the OnBot Java Programming Tool.

Changes with verion 3.3 include:
 * Android Studio ftc_app project has been updated to use Gradle Plugin 2.3.3.
 * Android Studio ftc_app project is already using gradle 3.5 distribution.
 * Robot Controller log has been renamed to /sdcard/RobotControllerLog.txt (note that this change was actually introduced w/ v3.2).
 * Improvements in I2C reliability.
 * Optimized I2C read for REV Expansion Hub, with v1.7 firmware or greater.
 * Updated all external/samples (available through OnBot and in Android project folder).
 * Vuforia
    - Added support for VuMarks that will be used for the 2017-2018 season game.
 * Blocks
    - Update to latest Google Blockly release.
    - Sample OpModes can be selected as a template when creating new OpMode.
    - Fixed bug where the blocks would disappear temporarily when mouse button is held down.
    - Added blocks for Range.clip and Range.scale.
    - User can now disable/enable Block OpModes.
    - Fix to prevent occasional Blocks deadlock.
 * OnBot Java
    - Significant improvements with autocomplete function for OnBot Java editor.
    - Sample OpModes can be selected as a template when creating new OpMode.
    - Fixes and changes to complete hardware setup feature.
    - Updated (and more useful) onBot welcome message.

Known issues:
 * Android Studio
    - After updating to the new v3.3 Android Studio project folder, if you get error messages indicating "InvalidVirtualFileAccessException" then you might need to do a File->Invalidate Caches / Restart to clear the error.
 * OnBot Java
    - Sometimes when you push the build button to build all OpModes, the RC returns an error message that the build failed.  If you press the build button a second time, the build typically suceeds.

## Version 3.2 (built on 17.08.02)

This version of the software introduces the "OnBot Java" Development Tool.  Similar to the FTC Blocks Development Tool, the FTC OnBot Java Development Tool allows a user to create, edit and build OpModes dynamically using only a Javascript-enabled web browser.

The OnBot Java Development Tool is an integrated development environment (IDE) that is served up by the Robot Controller.  OpModes are created and edited using a Javascript-enabled browser (Google Chromse is recommended).  OpModes are saved on the Robot Controller Android device directly.

The OnBot Java Development Tool provides a Java programming environment that does NOT need Android Studio.



Changes with version 3.2 include:
 * Enhanced web-based development tools
    - Introduction of OnBot Java Development Tool.
    - Web-based programming and management features are "always on" (user no longer needs to put Robot Controller into programming mode).
    - Web-based management interface (where user can change Robot Controller name and also easily download Robot Controller log file).
    - OnBot Java, Blocks and Management features available from web based interface.

* Blocks Programming Development Tool:
    - Changed "LynxI2cColorRangeSensor" block to "REV Color/range sensor" block.
    - Fixed tooltip for ColorSensor.isLightOn block.
    Added blocks for ColorSensor.getNormalizedColors and LynxI2cColorRangeSensor.getNormalizedColors.

* Added example OpModes for digital touch sensor and REV Robotics Color Distance sensor.
* User selectable color themes.
* Includes many minor enhancements and fixes (too numerous to list).

Known issues:
* Auto complete function is incomplete and does not support the following (for now):
     - Access via *this* keyword
     - Access via *super* keyword
     - Members of the super cloass, not overridden by the class
     - Any methods provided in the current class
     - Inner classes
     - Can't handle casted objects
     - Any objects coming from an parenthetically enclosed expression

## Version 3.10 (built on 17.05.09)

This version of the software provides support for the REV Robotics Expansion Hub.  This version also includes improvements in the USB communication layer in an effort to enhance system resiliency.  If you were using a 2.x version of the software previously, updating to version 3.1 requires that you also update your Driver Station software in addition to updating the Robot Controller software.

Also note that in version 3.10 software, the setMaxSpeed and getMaxSpeed methods are no longer available (not deprecated, they have been removed from the SDK). Also note that the new 3.x software incorporates motor profiles that a user can select as he/she configures the robot.

Changes include:
 * Blocks changes
    - Added VuforiaTrackableDefaultListener.getPose and Vuforia.trackPose blocks.
    - Added optimized blocks support for Vuforia extended tracking.
    - Added atan2 block to the math category.
    - Added useCompetitionFieldTargetLocations parameter to Vuforia.initialize block.  If set to false, the target locations are placed at (0,0,0) with target orientation as specified in https://github.com/gearsincorg/FTCVuforiaDemo/blob/master/Robot_Navigation.java tutorial OpMode.
 * Incorporates additional improvements to USB comm layer to improve system resiliency (to recover from a greater number of communication disruptions).

**************************************************************************************

Additional Notes Regarding Version 3.00 (built on 17.04.13)

In addition to the release changes listed below (see section labeled "Version 3.00 (built on 17.04.013)"), version 3.00 has the following important changes:

1. Version 3.00 software uses a new version of the FTC Robocol (robot protocol).  If you upgrade to v3.0 on the Robot Controller and/or Android Studio side, you must also upgrade the Driver Station software to match the new Robocol.
2. Version 3.00 software removes the setMaxSpeed and getMaxSpeed methods from the DcMotor class.  If you have an OpMode that formerly used these methods, you will need to remove the references/calls to these methods.  Instead, v3.0 provides the max speed information through the use of motor profiles that are selected by the user during robot configuration.
3. Version 3.00 software currently does not have a mechanism to disable extra i2c sensors.  We hope to re-introduce this function with a release in the near future.

**************************************************************************************

## Version 3.00 (built on 17.04.13)

*** Use this version of the software at YOUR OWN RISK!!! ***

This software is being released as an "alpha" version.  Use this version at your own risk!

This pre-release software contains SIGNIFICANT changes, including changes to the Wi-Fi Direct pairing mechanism, rewrites of the I2C sensor classes, changes to the USB/FTDI layer, and the introduction of support for the REV Robotics Expansion Hub and the REV Robotics color-range-light sensor.  These changes were implemented to improve the reliability and resiliency of the FTC control system.

Please note, however, that version 3.00 is considered "alpha" code.  This code is being released so that the FIRST community will have an opportunity to test the new REV Expansion Hub electronics module when it becomes available in May.  The developers do not recommend using this code for critical applications (i.e., competition use).

*** Use this version of the software at YOUR OWN RISK!!! ***

Changes include:
 * Major rework of sensor-related infrastructure.  Includes rewriting sensor classes to implement synchronous I2C communication.
 * Fix to reset Autonomous timer back to 30 seconds.
 * Implementation of specific motor profiles for approved 12V motors (includes Tetrix, AndyMark, Matrix and REV models).
 * Modest improvements to enhance Wi-Fi P2P pairing.
 * Fixes telemetry log addition race.
 * Publishes all the sources (not just a select few).
 * Includes Block programming improvements
    - Addition of optimized Vuforia blocks.
    - Auto scrollbar to projects and sounds pages.
    - Fixed blocks paste bug.
    - Blocks execute after while-opModeIsActive loop (to allow for cleanup before exiting OpMode).
    - Added gyro integratedZValue block.
    - Fixes bug with projects page for Firefox browser.
    - Added IsSpeaking block to AndroidTextToSpeech.
 * Implements support for the REV Robotics Expansion Hub
    - Implements support for integral REV IMU (physically installed on I2C bus 0, uses same Bosch BNO055 9 axis absolute orientation sensor as Adafruit 9DOF abs orientation sensor).    - Implements support for REV color/range/light sensor.
    - Provides support to update Expansion Hub firmware through FTC SDK.
    - Detects REV firmware version and records in log file.
    - Includes support for REV Control Hub (note that the REV Control Hub is not yet approved for FTC use).
    - Implements FTC Blocks programming support for REV Expansion Hub and sensor hardware.
    - Detects and alerts when I2C device disconnect.

## Version 2.62 (built on 17.01.07)
  * Added null pointer check before calling modeToByte() in finishModeSwitchIfNecessary method for ModernRoboticsUsbDcMotorController class.
  * Changes to enhance Modern Robotics USB protocol robustness.

## Version 2.61 (released on 16.12.19)
  * Blocks Programming mode changes:
     - Fix to correct issue when an exception was thrown because an OpticalDistanceSensor object appears twice in the hardware map (the second time as a LightSensor).

## Version 2.6 (released on 16.12.16)
  * Fixes for Gyro class:
     - Improve (decrease) sensor refresh latency.
     - fix isCalibrating issues.
  * Blocks Programming mode changes:
     - Blocks now ignores a device in the configuration xml if the name is empty. Other devices work in configuration work fine.

## Version 2.5 (internal release on released on 16.12.13)
  * Blocks Programming mode changes:
     - Added blocks support for AdafruitBNO055IMU.
     - Added Download OpMode button to FtcBocks.html.
     - Added support for copying blocks in one OpMode and pasting them in an other OpMode. The clipboard content is stored on the phone, so the programming mode server must be running.
     - Modified Utilities section of the toolbox.
     - In Programming Mode, display information about the active connections.
     - Fixed paste location when workspace has been scrolled.
     - Added blocks support for the android Accelerometer.
     - Fixed issue where Blocks Upload OpMode truncated name at first dot.
     - Added blocks support for Android SoundPool.
     - Added type safety to blocks for Acceleration.
     - Added type safety to blocks for AdafruitBNO055IMU.Parameters.
     - Added type safety to blocks for AnalogInput.
     - Added type safety to blocks for AngularVelocity.
     - Added type safety to blocks for Color.
     - Added type safety to blocks for ColorSensor.
     - Added type safety to blocks for CompassSensor.
     - Added type safety to blocks for CRServo.
     - Added type safety to blocks for DigitalChannel.
     - Added type safety to blocks for ElapsedTime.
     - Added type safety to blocks for Gamepad.
     - Added type safety to blocks for GyroSensor.
     - Added type safety to blocks for IrSeekerSensor.
     - Added type safety to blocks for LED.
     - Added type safety to blocks for LightSensor.
     - Added type safety to blocks for LinearOpMode.
     - Added type safety to blocks for MagneticFlux.
     - Added type safety to blocks for MatrixF.
     - Added type safety to blocks for MrI2cCompassSensor.
     - Added type safety to blocks for MrI2cRangeSensor.
     - Added type safety to blocks for OpticalDistanceSensor.
     - Added type safety to blocks for Orientation.
     - Added type safety to blocks for Position.
     - Added type safety to blocks for Quaternion.
     - Added type safety to blocks for Servo.
     - Added type safety to blocks for ServoController.
     - Added type safety to blocks for Telemetry.
     - Added type safety to blocks for Temperature.
     - Added type safety to blocks for TouchSensor.
     - Added type safety to blocks for UltrasonicSensor.
     - Added type safety to blocks for VectorF.
     - Added type safety to blocks for Velocity.
     - Added type safety to blocks for VoltageSensor.
     - Added type safety to blocks for VuforiaLocalizer.Parameters.
     - Added type safety to blocks for VuforiaTrackable.
     - Added type safety to blocks for VuforiaTrackables.
     - Added type safety to blocks for enums in AdafruitBNO055IMU.Parameters.
     - Added type safety to blocks for AndroidAccelerometer, AndroidGyroscope, AndroidOrientation, and AndroidTextToSpeech.

## Version 2.4 (released on 16.11.13)
  * Fix to avoid crashing for nonexistent resources.
  * Blocks Programming mode changes:
     - Added blocks to support OpenGLMatrix, MatrixF, and VectorF.
     - Added blocks to support AngleUnit, AxesOrder, AxesReference, CameraDirection, CameraMonitorFeedback, DistanceUnit, and TempUnit.
     - Added blocks to support Acceleration.
     - Added blocks to support LinearOpMode.getRuntime.
     - Added blocks to support MagneticFlux and Position.
     - Fixed typos.
     - Made blocks for ElapsedTime more consistent with other objects.
     - Added blocks to support Quaternion, Velocity, Orientation, AngularVelocity.
     - Added blocks to support VuforiaTrackables, VuforiaTrackable, VuforiaLocalizer, VuforiaTrackableDefaultListener.
     - Fixed a few blocks.
     - Added type checking to new blocks.
     - Updated to latest blockly.
     - Added default variable blocks to navigation and matrix blocks.
     - Fixed toolbox entry for openGLMatrix_rotation_withAxesArgs.
     - When user downloads Blocks-generated OpMode, only the .blk file is downloaded.
     - When user uploads Blocks-generated OpMode (.blk file), Javascript code is auto generated.
     - Added DbgLog support.
     - Added logging when a blocks file is read/written.
     - Fixed bug to properly render blocks even if missing devices from configuration file.
     - Added support for additional characters (not just alphanumeric) for the block file names (for download and upload).
     - Added support for OpMode flavor (“Autonomous” or “TeleOp”) and group.
  * Changes to Samples to prevent tutorial issues.
  * Incorporated suggested changes from public pull 216 (“Replace .. paths”).
  * Remove Servo Glitches when robot stopped.
  * if user hits “Cancels” when editing a configuration file, clears the unsaved changes and reverts to original unmodified configuration.
  * Added log info to help diagnose why the Robot Controller app was terminated (for example, by watch dog function).
  * Added ability to transfer log from the controller.
  * Fixed inconsistency for AngularVelocity
  * Limit unbounded growth of data for telemetry.  If user does not call telemetry.update() for LinearOpMode in a timely manner, data added for telemetry might get lost if size limit is exceeded.

## Version 2.35 (released on 16.10.06)
  * Blockly programming mode - Removed unnecesary idle() call from blocks for new project.

## Version 2.30 (released on 16.10.05)
  * Blockly programming mode:
     - Mechanism added to save Blockly OpModes from Programming Mode Server onto local device
     - To avoid clutter, blocks are displayed in categorized folders
     - Added support for DigitalChannel
     - Added support for ModernRoboticsI2cCompassSensor
     - Added support for ModernRoboticsI2cRangeSensor
     - Added support for VoltageSensor
     - Added support for AnalogInput
     - Added support for AnalogOutput
     - Fix for CompassSensor setMode block
  * Vuforia
     - Fix deadlock / make camera data available while Vuforia is running.
     - Update to Vuforia 6.0.117 (recommended by Vuforia and Google to close security loophole).
  * Fix for autonomous 30 second timer bug (where timer was in effect, even though it appeared to have timed out).
  * opModeIsActive changes to allow cleanup after OpMode is stopped (with enforced 2 second safety timeout).
  * Fix to avoid reading i2c twice.
  * Updated sample OpModes.
  * Improved logging and fixed intermittent freezing.
  * Added digital I/O sample.
  * Cleaned up device names in sample OpModes to be consistent with Pushbot guide.
  * Fix to allow use of IrSeekerSensorV3.

## Version 2.20 (released on 16.09.08)
  * Support for Modern Robotics Compass Sensor.
  * Support for Modern Robotics Range Sensor.
  * Revise device names for Pushbot templates to match the names used in Pushbot guide.
  * Fixed bug so that IrSeekerSensorV3 device is accessible as IrSeekerSensor in hardwareMap.
  * Modified computer vision code to require an individual Vuforia license (per legal requirement from PTC).
  * Minor fixes.
  * Blockly enhancements:
     - Support for Voltage Sensor.
     - Support for Analog Input.
     - Support for Analog Output.
     - Support for Light Sensor.
     - Support for Servo Controller.

## Version 2.10 (released on 16.09.03)
 * Support for Adafruit IMU.
 * Improvements to ModernRoboticsI2cGyro class
    - Block on reset of z axis.
    - isCalibrating() returns true while gyro is calibration.
 * Updated sample gyro program.
 * Blockly enhancements
    - support for android.graphics.Color.
    - added support for ElapsedTime.
    - improved look and legibility of blocks.
    - support for compass sensor.
    - support for ultrasonic sensor.
    - support for IrSeeker.
    - support for LED.
    - support for color sensor.
    - support for CRServo
    - prompt user to configure robot before using programming mode.
 * Provides ability to disable audio cues.
 * various bug fixes and improvements.

## Version 2.00 (released on 16.08.19)
 * This is the new release for the upcoming 2016-2017 FIRST Tech Challenge Season.
 * Channel change is enabled in the FTC Robot Controller app for Moto G 2nd and 3rd Gen phones.
 * Users can now use annotations to register/disable their OpModes.
 * Changes in the Android SDK, JDK and build tool requirements (minsdk=19, java 1.7, build tools 23.0.3).
 * Standardized units in analog input.
 * Cleaned up code for existing analog sensor classes.
 * setChannelMode and getChannelMode were REMOVED from the DcMotorController class.  This is important - we no longer set the motor modes through the motor controller.
 * setMode and getMode were added to the DcMotor class.
 * ContinuousRotationServo class has been added to the FTC SDK.
 * Range.clip() method has been overloaded so it can support this operation for int, short and byte integers.
 * Some changes have been made (new methods added) on how a user can access items from the hardware map.
 * Users can now set the zero power behavior for a DC motor so that the motor will brake or float when power is zero.
 * Prototype Blockly Programming Mode has been added to FTC Robot Controller.  Users can place the Robot Controller into this mode, and then use a device (such as a laptop) that has a Javascript enabled browser to write Blockly-based OpModes directly onto the Robot Controller.
 * Users can now configure the robot remotely through the FTC Driver Station app.
 * Android Studio project supports Android Studio 2.1.x and compile SDK Version 23 (Marshmallow).
 * Vuforia Computer Vision SDK integrated into FTC SDK.  Users can use sample vision targets to get localization information on a standard FTC field.
 * Project structure has been reorganized so that there is now a TeamCode package that users can use to place their local/custom OpModes into this package.
 * Inspection function has been integrated into the FTC Robot Controller and Driver Station Apps (Thanks Team HazMat… 9277 & 10650!).
 * Audio cues have been incorporated into FTC SDK.
 * Swap mechanism added to FTC Robot Controller configuration activity.  For example, if you have two motor controllers on a robot, and you misidentified them in your configuration file, you can use the Swap button to swap the devices within the configuration file (so you do not have to manually re-enter in the configuration info for the two devices).
 * Fix mechanism added to all user to replace an electronic module easily.  For example, suppose a servo controller dies on your robot. You replace the broken module with a new module, which has a different serial number from the original servo controller.  You can use the Fix button to automatically reconfigure your configuration file to use the serial number of the new module.
 * Improvements made to fix resiliency and responsiveness of the system.
 * For LinearOpMode the user now must for a telemetry.update() to update the telemetry data on the driver station.  This update() mechanism ensures that the driver station gets the updated data properly and at the same time.
 * The Auto Configure function of the Robot Controller is now template based.  If there is a commonly used robot configuration, a template can be created so that the Auto Configure mechanism can be used to quickly configure a robot of this type.
 * The logic to detect a runaway OpMode (both in the LinearOpMode and OpMode types) and to abort the run, then auto recover has been improved/implemented.
 * Fix has been incorporated so that Logitech F310 gamepad mappings will be correct for Marshmallow users.

## Release 16.07.08

 * For the ftc_app project, the gradle files have been modified to support Android Studio 2.1.x.

## Release 16.03.30

 * For the MIT App Inventor, the design blocks have new icons that better represent the function of each design component.
 * Some changes were made to the shutdown logic to ensure the robust shutdown of some of our USB services.
 * A change was made to LinearOpMode so as to allow a given instance to be executed more than once, which is required for the App Inventor.
 * Javadoc improved/updated.

## Release 16.03.09

 * Changes made to make the FTC SDK synchronous (significant change!)
    - waitOneFullHardwareCycle() and waitForNextHardwareCycle() are no longer needed and have been deprecated.
    - runOpMode() (for a LinearOpMode) is now decoupled from the system's hardware read/write thread.
    - loop() (for an OpMode) is now decoupled from the system's hardware read/write thread.
    - Methods are synchronous.
    - For example, if you call setMode(DcMotorController.RunMode.RESET_ENCODERS) for a motor, the encoder is guaranteed to be reset when the method call is complete.
    - For legacy module (NXT compatible), user no longer has to toggle between read and write modes when reading from or writing to a legacy device.
 * Changes made to enhance reliability/robustness during ESD event.
 * Changes made to make code thread safe.
 * Debug keystore added so that user-generated robot controller APKs will all use the same signed key (to avoid conflicts if a team has multiple developer laptops for example).
 * Firmware version information for Modern Robotics modules are now logged.
 * Changes made to improve USB comm reliability and robustness.
 * Added support for voltage indicator for legacy (NXT-compatible) motor controllers.
 * Changes made to provide auto stop capabilities for OpModes.
    - A LinearOpMode class will stop when the statements in runOpMode() are complete.  User does not have to push the stop button on the driver station.
    - If an OpMode is stopped by the driver station, but there is a run away/uninterruptible thread persisting, the app will log an error message then force itself to crash to stop the runaway thread.
 * Driver Station UI modified to display lowest measured voltage below current voltage (12V battery).
 * Driver Station UI modified to have color background for current voltage (green=good, yellow=caution, red=danger, extremely low voltage).
 * javadoc improved (edits and additional classes).
 * Added app build time to About activity for driver station and robot controller apps.
 * Display local IP addresses on Driver Station About activity.
 * Added I2cDeviceSynchImpl.
 * Added I2cDeviceSync interface.
 * Added seconds() and milliseconds() to ElapsedTime for clarity.
 * Added getCallbackCount() to I2cDevice.
 * Added missing clearI2cPortActionFlag.
 * Added code to create log messages while waiting for LinearOpMode shutdown.
 * Fix so Wi-Fi Direct Config activity will no longer launch multiple times.
 * Added the ability to specify an alternate i2c address in software for the Modern Robotics gyro.

## Release 16.02.09

 * Improved battery checker feature so that voltage values get refreshed regularly (every 250 msec) on Driver Station (DS) user interface.
 * Improved software so that Robot Controller (RC) is much more resilient and “self-healing” to USB disconnects:
    - If user attempts to start/restart RC with one or more module missing, it will display a warning but still start up.
    - When running an OpMode, if one or more modules gets disconnected, the RC & DS will display warnings,and robot will keep on working in spite of the missing module(s).
    - If a disconnected module gets physically reconnected the RC will auto detect the module and the user will regain control of the recently connected module.
    - Warning messages are more helpful (identifies the type of module that’s missing plus its USB serial number).
 * Code changes to fix the null gamepad reference when users try to reference the gamepads in the init() portion of their OpMode.
 * NXT light sensor output is now properly scaled.  Note that teams might have to readjust their light threshold values in their OpModes.
 * On DS user interface, gamepad icon for a driver will disappear if the matching gamepad is disconnected or if that gamepad gets designated as a different driver.
 * Robot Protocol (ROBOCOL) version number info is displayed in About screen on RC and DS apps.
 * Incorporated a display filter on pairing screen to filter out devices that don’t use the “<TEAM NUMBER>-“ format. This filter can be turned off to show all Wi-Fi Direct devices.
 * Updated text in License file.
 * Fixed formatting error in OpticalDistanceSensor.toString().
 * Fixed issue on with a blank (“”) device name that would disrupt Wi-Fi Direct Pairing.
 * Made a change so that the Wi-Fi info and battery info can be displayed more quickly on the DS upon connecting to RC.
 * Improved javadoc generation.
 * Modified code to make it easier to support language localization in the future.

## Release 16.01.04

 * Updated compileSdkVersion for apps
 * Prevent Wi-Fi from entering power saving mode
 * removed unused import from driver station
 * Corrrected "Dead zone" joystick code.
 * LED.getDeviceName and .getConnectionInfo() return null
 * apps check for ROBOCOL_VERSION mismatch
 * Fix for Telemetry also has off-by-one errors in its data string sizing / short size limitations error
 * User telemetry output is sorted.
 * added formatting variants to DbgLog and RobotLog APIs
 * code modified to allow for a long list of OpMode names.
 * changes to improve thread safety of RobocolDatagramSocket
 * Fix for "missing hardware leaves robot controller disconnected from driver station" error
 * fix for "fast tapping of Init/Start causes problems" (toast is now only instantiated on UI thread).
 * added some log statements for thread life cycle.
 * moved gamepad reset logic inside of initActiveOpMode() for robustness
 * changes made to mitigate risk of race conditions on public methods.
 * changes to try and flag when Wi-Fi Direct name contains non-printable characters.
 * fix to correct race condition between .run() and .close() in ReadWriteRunnableStandard.
 * updated FTDI driver
 * made ReadWriteRunnableStanard interface public.
 * fixed off-by-one errors in Command constructor
 * moved specific hardware implmentations into their own package.
 * moved specific gamepad implemnatations to the hardware library.
 * changed LICENSE file to new BSD version.
 * fixed race condition when shutting down Modern Robotics USB devices.
 * methods in the ColorSensor classes have been synchronized.
 * corrected isBusy() status to reflect end of motion.
 * corrected "back" button keycode.
 * the notSupported() method of the GyroSensor class was changed to protected (it should not be public).

## Release 15.11.04.001

 * Added Support for Modern Robotics Gyro.
  - The GyroSensor class now supports the MR Gyro Sensor.
  - Users can access heading data (about Z axis)
  - Users can also access raw gyro data (X, Y, & Z axes).
  - Example MRGyroTest.java OpMode included.
 * Improved error messages
  - More descriptive error messages for exceptions in user code.
 * Updated DcMotor API
 * Enable read mode on new address in setI2cAddress
 * Fix so that driver station app resets the gamepads when switching OpModes.
 * USB-related code changes to make USB comm more responsive and to display more explicit error messages.
  - Fix so that USB will recover properly if the USB bus returns garbage data.
  - Fix USB initializtion race condition.
  - Better error reporting during FTDI open.
  - More explicit messages during USB failures.
  - Fixed bug so that USB device is closed if event loop teardown method was not called.
 * Fixed timer UI issue
 * Fixed duplicate name UI bug (Legacy Module configuration).
 * Fixed race condition in EventLoopManager.
 * Fix to keep references stable when updating gamepad.
 * For legacy Matrix motor/servo controllers removed necessity of appending "Motor" and "Servo" to controller names.
 * Updated HT color sensor driver to use constants from ModernRoboticsUsbLegacyModule class.
 * Updated MR color sensor driver to use constants from ModernRoboticsUsbDeviceInterfaceModule class.
 * Correctly handle I2C Address change in all color sensors
 * Updated/cleaned up OpModes.
  - Updated comments in LinearI2cAddressChange.java example OpMode.
  - Replaced the calls to "setChannelMode" with "setMode" (to match the new of the DcMotor  method).
  - Removed K9AutoTime.java OpMode.
  - Added MRGyroTest.java OpMode (demonstrates how to use MR Gyro Sensor).
  - Added MRRGBExample.java OpMode (demonstrates how to use MR Color Sensor).
  - Added HTRGBExample.java OpMode (demonstrates how to use HT legacy color sensor).
  - Added MatrixControllerDemo.java (demonstrates how to use legacy Matrix controller).
 * Updated javadoc documentation.
 * Updated release .apk files for Robot Controller and Driver Station apps.

## Release 15.10.06.002

 * Added support for Legacy Matrix 9.6V motor/servo controller.
 * Cleaned up build.gradle file.
 * Minor UI and bug fixes for driver station and robot controller apps.
 * Throws error if Ultrasonic sensor (NXT) is not configured for legacy module port 4 or 5.


## Release 15.08.03.001

 * New user interfaces for FTC Driver Station and FTC Robot Controller apps.
 * An init() method is added to the OpMode class.
   - For this release, init() is triggered right before the start() method.
   - Eventually, the init() method will be triggered when the user presses an "INIT" button on driver station.
   - The init() and loop() methods are now required (i.e., need to be overridden in the user's OpMode).
   - The start() and stop() methods are optional.
 * A new LinearOpMode class is introduced.
   - Teams can use the LinearOpMode mode to create a linear (not event driven) program model.
   - Teams can use blocking statements like Thread.sleep() within a linear OpMode.
 * The API for the Legacy Module and Core Device Interface Module have been updated.
   - Support for encoders with the Legacy Module is now working.
 * The hardware loop has been updated for better performance.
