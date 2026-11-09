# 2026 年提交审查报告（WBR_MINI）

> 审查对象：`D:\RM2027\EC_dev\WBR_MINI`（分支 `mini`）
>
> 审查口径：`git log --all --since=2026-01-01 --until=2027-01-01`，共 **35 个提交**（2026-10-13 ~ 2026-10-29）
>
> 审查方式：逐提交 diff 通读 + 与当前工作区代码对照（行号均指**当前工作区**文件）
>
> 结论口径：`保留` / `保留但需修` / `临时·影响功能` / `历史·已被覆盖`

---

## 0. 一句话结论

这 35 个提交完成了「标准 RoboMaster 步兵 → mini 轮腿」的裁剪：删掉了云台/发射/自瞄链路、换用 DM4310 + 绝对编码器、加了电池监测和蜂鸣器。
**骨架（电机驱动、绝对编码器、IMU 方向、DJI 发包、headless 裁剪）是值得保留的资产；但最后 6 个 `test:`/`para:`/`fix:` 提交把底盘参考轨迹改成了半成品状态，其中 2 处是明显的临时手改，且工作区还压着 28 处 `// TODO refactor` 标记和一批注释掉的死代码**（5.1/5.2/5.4 已清掉 8 处，剩 20 处）。

优先处理 3 件事（P0）：
1. ✅ **已修复**：`chassis.cpp:448-451` 速度参考被**双重积分后又覆盖**，速度闭环实际失效（见 5.1）；
2. ✅ **已修复**：`chassis.cpp:440/458` 腿长补偿 `l_extra` 被注释，倾角保护失效（见 5.1）；
3. ✅ **已修复**：`chassis.cpp:217-222` 腿角差补偿 PID 被注释但 `T_err` 仍参与力矩输出 —— 已按 5.2 **彻底删除**。

---

## 1. 提交总览

| # | 提交 | 日期 | 标题 | 规模 | 判定 |
|---|------|------|------|------|------|
| 1 | `5a6dba0` | 10-29 | test: modify chassis reference func for test | +6/−6 | **临时·影响功能** |
| 2 | `b044243` | 10-29 | test: 关闭髋关节腿角差力矩补偿 | +4/−4 | **临时·影响功能** |
| 3 | `26b2c54` | 10-29 | para: 机器人参数和控制参数 | +50/−46 | 保留但需修（需上机验证） |
| 4 | `961e4d8` | 10-29 | fix: chassis冗余代码 | −2 | 保留 |
| 5 | `36922f3` | 10-29 | fix: imu旋转矩阵方向定义错误 | +5/−3 | 保留（关键修复） |
| 6 | `6f563cd` | 10-29 | fix: motor构造函数传参错误 | +7/−5 | 保留（关键修复） |
| 7 | `d9b0928` | 10-29 | fix&refactor: 蜂鸣器未初始化，删除冗余控制代码 | +7/−301 | 保留（大清理） |
| 8 | `f9c9a2d` | 10-29 | fix: 修改项目名为WBR_MINI | +1/−1 | 保留 |
| 9 | `61fe92f` | 10-29 | update: 修改机器人参数重新计算参数 | +25/−25 | 保留（参数同步） |
| 10 | `22e66ad` | 10-28 | feat: 开机音乐 | +38/−22 | 保留但需重构（当前已禁用） |
| 11 | `4d44184` | 10-28 | fix: 添加buzzer初始化 | +1 | 保留 |
| 12 | `17989ae` | 10-26 | fix:修复整圈定义错误 | +4/−4 | 保留 |
| 13 | `d38bfa2` | 10-25 | feat:补充buzzer函数，微调pid | +49/−1 | 保留框架；PID 改动需回看 |
| 14 | `5ad1174` | 10-25 | feat:添加buzzer,修改电压阈值 | +57/−31 | 保留框架（实现有缺陷） |
| 15 | `2cd8277` | 10-23 | feat:添加整圈定义 | +4/−4 | 历史·已被 #12 修正 |
| 16 | `474c21b` | 10-23 | feat:headless | +11/−6 | 保留 |
| 17 | `5b53396` | 10-23 | doc:补充注释 | +4/−6 | 保留 |
| 18 | `c1f67e0` | 10-23 | fix:关闭其他挡位，后续开发其他功能 | +7/−6 | **临时·功能残缺** |
| 19 | `aa0f729` | 10-23 | fix:修改轮电机减速比，能跑 | +2/−2 | 历史·已被 #6 取代 |
| 20 | `e60ec09` | 10-23 | param:修改目标值，无yaw角控制 | +12/−11 | **临时·影响功能** |
| 21 | `8e7d3e5` | 10-23 | feat:adc读取电池电压，电压过低停止工作 | +74/−1 | 保留但需修（重复实现） |
| 22 | `6725f8a` | 10-22 | param:imu零飘，附代码 | +27/−1 | 一半保留（bias 值）／注释块应删 |
| 23 | `a9fe254` | 10-22 | fix:修正轮电机左右错误 | +2/−2 | 保留（并入 #6） |
| 24 | `0c9d783` | 10-22 | param:拆电机重调零 | +1/−1 | 历史·已被 #3 覆盖 |
| 25 | `90c28f4` | 10-22 | fix:修复dji电机发包bug | +1/−1 | 保留（关键修复） |
| 26 | `34b0aac` | 10-21 | fix:修改角度定义 | +2/−2 | 历史·已被覆盖 |
| 27 | `f0a3fd2` | 10-21 | param:同步matlab参数 | +4/−4 | 历史·已被 #33 覆盖 |
| 28 | `5d66ca3` | 10-20 | compile:取消microlib，能过编译 | +1/−1 | 保留（编译必需） |
| 29 | `c39c7d3` | 10-20 | fix:修复缺失的文件,增加yaw速度 | +1834/−3 | 保留（体积问题见 4.7） |
| 30 | `6572f51` | 10-19 | feat: leg motor use absolute encoder | +59/−85 | 保留（核心改造） |
| 31 | `bcce860` | 10-19 | feat: headless | +9/−37 | 保留 |
| 32 | `c740ca1` | 10-19 | param: update jmr2 id | +1/−1 | 保留 |
| 33 | `eab7236` | 10-13 | feat: update mit motor to fit dm motor | +111/−69 | 保留（核心） |
| 34 | `cf8b1cb` | 10-13 | param: imu orientation param | +3/−3 | 历史·已被 #5 修正 |
| 35 | `a059ee4` | 10-13 | param: mini wbr param | +217/−180 | 保留（mini 基线） |

**作者分布**：`BruceLannn`（10-13、10-19 共 5 个）、`Nostalgia`（10-20 ~ 10-26 共 13 个）、`Airman Han`（10-28 ~ 10-29 共 17 个）。前两位是设备/参数接入，最后一位是最后两天的调试与清理。

---

## 2. 建议保留的代码（核心资产）

### 2.1 电机层：DM4310 适配 + 绝对编码器（`eab7236`、`6572f51`、`c740ca1`）

**为什么保留**：这是整台 mini 能站起来的前提，且是"一次性做对"的改造。

- `eab7236` 把 MIT 协议的 HT04 参数换成 DM4310：位置 ±12.566 rad、速度 ±30 rad/s、力矩 ±10 N·m（`motor_monitor.cpp:110-125`）；`canRxMsgCallback` 把 `rx_data[0]` 从"整字节比较"改为 `(rx_data[0] & 0x0F)` 比较（`mit_motor_driver.cpp:163`），解决了多电机 ID 解析失败；
- 同一提交把 `setCmd` 的环形队列判断从"只看队尾"改成"全队列查重"（`mit_motor_driver.cpp:61-74`），并区分 `MOTOR_MODE=0xFC / RESET=0xFD / ZERO=0xFE / CLEAR_ERROR=0xFB`，符合达妙协议；
- `6572f51` 把上电"撞限位找零点"（`JointCali_t`，会顶到机械限位）换成"绝对编码器角度 − 标定偏移"（`chassis.cpp:271-300`），去掉了 4 个关节的上电动作，安全性提升明显；同时把腿长范围从 0.15~0.4 m 收到 0.05~0.2 m（`chassis.cpp:456`），与 mini 尺寸匹配。

**保留但注意**：
- 关节零点是 4 个硬编码常数（`chassis.h:204-206`：`-109.00, -48.92, 14.60, 26.80`），换电机/拆装后必须重标定，建议在 `doc/` 里补一份"标定步骤 + 当前值记录表"；
- `MITMotorDriver::handle()`（`mit_motor_driver.cpp:50-57`）目前**没有任何地方调用**，见 3.6。

### 2.2 IMU 方向与零漂（`cf8b1cb` → `36922f3` → `6725f8a`）

- 安装方向矩阵 `R_imu` 最终为 `{{0,-1,0},{1,0,0},{0,0,1}}`（`imu_monitor.cpp:39-43`），即绕 Z 轴 −90° 的逆矩阵。`36922f3` 的修正与 `cf8b1cb` 恰好互为逆，说明之前姿态是反的（pitch/roll 符号错），这个提交**必须保留**。
- `6725f8a` 把陀螺零漂写死为 `{0.00133, -0.000876549, -0.0008851835}`（`imu_monitor.cpp:49`）。**数值保留**（是实测值），但同一提交里塞进 `imu.cpp:89-109` 的 20 行注释代码应删除（见 4.4）。

### 2.3 DJI 电机发包（`90c28f4`）

`can_monitor.cpp:53` 由 `ID_5_8` 改为 `ID_1_4`，对应轮电机 ID 1/2。这是"电机收到帧但不动"的典型原因，保留。

### 2.4 headless 裁剪（`bcce860`、`474c21b`、`d9b0928`）

mini 没有云台/发射/自瞄，这三个提交把这部分从启动流程和控制循环里摘掉：
- `robotStartup()` 不再等待双板通信、不再初始化云台和拨弹（`control.cpp:209-218`）；
- 控制循环只留 `chassis.mappingRef(0.f)` + `chassis.handle()`（`control.cpp:141-145`）；
- `d9b0928` 一次性删除 301 行 `robotControl()` 里的云台/发射/自瞄/键鼠逻辑，方向正确。

**但裁剪不彻底**：`Gimbal/Shoot/Gate/Autoaim/CVComm/power_limit` 对象仍在构造（`control.cpp:49-56`），`robotReset()` 每次复位还在操作它们（`control.cpp:197-206`），`kbctrl` 整段参数（`control.cpp:90-102`）已无人引用。详见 4.5。

### 2.5 电池监测（`8e7d3e5`）

ADC3 + 10:1 分压，低电压进入 STOP 并鸣叫。原始实现是 `serialToolTask` 里每 1 s 采样一次、阈值 22 V。**已按 5.4 重构为 `BatteryMonitor` 类：100 ms 采样 + 8 点滑动平均 + 22.0/22.5 V 迟滞**（`battery_monitor.cpp`、`control.cpp:120`、`control.cpp:358`），保留其"低电压保护"的核心意图。

### 2.6 蜂鸣器"非阻塞化"方向（`5ad1174`、`d38bfa2`、`4d44184`）

旧实现 `playNote()` 里 `while (HAL_GetTick() < stop_tick_) osDelay(10);` 是阻塞式播放，会把 1 kHz 控制循环卡住。新实现改成 `SetMusicNote()/Mute()/Handle()` 状态机，方向完全正确，**框架保留**；但实现细节有 5 个坑（见 3.7、4.3）。

### 2.7 工程与编译（`c39c7d3`、`5d66ca3`、`f9c9a2d`）

- 补 `lib/arm_math/*.lib|*.a` 与 `stm32f4xx_hal_cortex.c/h`（此前缺失导致 clone 后编译不过）；
- `5d66ca3` 关闭 microlib（`useUlib=0`），因为 arm_math 依赖标准 C 库，二者必须成对保留；
- `.gitignore` 的 `*RTE*` → `**/RTE/` 修正了误伤。

### 2.8 参数体系（`a059ee4`、`f0a3fd2`、`26b2c54`、`61fe92f`）

`a059ee4` 把 2.6 m 级 WBR 的模型整体缩到 mini：轮半径 60→35 mm、轮距 260→153.3 mm、整车质量 14.76→2.319 kg、连杆 0.075/0.15/0.3 → 0.0375/0.075/0.135 m，并把 `KK1~KK6` 换成 6 组 4×10 的 LQR 增益、`KA/KB` 换成多项式拟合系数。**这是 mini 的基线，必须保留**。

`26b2c54` 再改 4 项：`m_b 1.919→2.319`、`l_1u/l_1d 0.075→0.135`、4 个编码器偏移、`KK` 全套；`61fe92f` 同步了 `doc/controller_param/KK.txt` 和两个 `.mlx`。

> ⚠️ 这里有个**隐藏的正确性修复**：`6572f51` 把 `l_pid` 写成 `PID(1.2e3, 0, 2.5e5, 0, 0, 0.25)`，第 5 个参数 `out_max=0`，`PID::calc()` 里 `math::limit(output, -0, 0)` 恒等于 0——腿长环**整段是死的**；`psi_spid` 同样 `out_max=0`。直到 `26b2c54` 才改成 `l_pid(200,0,2500,0,10,0.2)`、`psi_spid(3.6,0,0,0,5,1)`。也就是说 10-19 到 10-29 之间"腿长/roll 调不动"很可能就是这个原因，`26b2c54` 属于**功能性修复**，不能当普通调参回滚。

---

## 3. 会影响正常功能的代码（按优先级）

### 3.1 【P0】速度参考被双重积分后又覆盖 —— `5a6dba0`

> **状态：已于 2026-10-29 按 5.1 修复**（`chassis.cpp:437-464`）。以下保留问题原始分析，便于回溯。

```cpp
// 修改前 chassis.cpp:447-451
ref_.sd += ref_.sd_D1 * 1e-3f;                                          // ①
ref_.sd = math::limit(ref_.sd + ref_.sd_D1 * 1e-3f, fdb_.s - 0.2f, ...); // ② 又积一次
ref_.sd = fdb_.s;                                                        // ③ 直接抹掉
```

- ① 和 ② 都做了 `sd += sd_D1*dt`，等于**每周期积分 2 次**；
- ③ 立刻把 `ref_.sd` 拉回实测值，前面积分全部作废；
- 结果：`ref_.xd[0]` 恒等于 `fdb_.s`，位置误差恒 0，只剩 `xd[1]` 的速度前馈在起作用。表现就是"能走但速度跟不准、松杆不停/急停"。

这个提交的标题就是 `test:`，同一提交还把 `// ref_.phid = fdb_.phi;` 注释掉、把 `ref_.ld` 改成忽略 `l_extra`。本次采用**定点修复**而非整体回滚：`phid_D1`、`ld`/`l_extra` 的原始意图已在 5.1 中恢复，其余未动。

### 3.2 【P0】倾角保护失效 —— `5a6dba0`

> **状态：已于 2026-10-29 按 5.1 修复**（`l_extra` 恢复计算，`ld` 重新减去它）。

```cpp
// chassis.cpp:439-440
// 机体倾角过大时缩短腿长
// ref_.l_extra = math::limit((fabs(fdb_.theta_b) - 0.15f) * 2.5f, 0, 0.25f);
// chassis.cpp:458
ref_.ld = math::limit(ref_.l, 0.05, 0.2f);   // 应为 ref_.l - ref_.l_extra
```

`l_extra` 的赋值被注释、`ld` 又不再减它 → `doc/plan.md` 第 4 节勾选的"大角度状态缩短腿长"功能实际处于关闭状态。倾角超过 0.15 rad 时腿不再收短，抗前倾/后倾能力下降。

### 3.3 【P0】腿角差补偿 PID 被注释但输出仍在用 —— `b044243`

> **状态：已于 2026-10-29 按 5.2 彻底删除**（结构体、成员、计算块、叠加项全部移除）。以下保留原始分析。

```cpp
// chassis.cpp:216-222  计算被整段注释
// chassis.cpp:229-232  但仍然参与输出
u_.T_bll = locomotion_controller_.u[2] + locomotion_controller_.theta_l_err_resist.T_err;
u_.T_blr = locomotion_controller_.u[3] - locomotion_controller_.theta_l_err_resist.T_err;
```

`T_err` 从此永远保持初值 0。虽然数值上等于"关闭"，但这是**靠未初始化成员恰好为 0** 才安全（全局对象 BSS 清零）。一旦以后挪成局部对象或加构造函数，就是未定义行为。要么恢复计算，要么把 `T_err` 显式置 0 并删掉这两行叠加。

### 3.4 【P1】遥控器只剩"左上+右上"一个挡位可用 —— `c1f67e0` + `d9b0928`

```cpp
// control.cpp:221-251
if (l==UP && r==UP)  → DIRECT，可用（前后 + 转向 + 腿长）
else                 → 全部 STOP
```

`e60ec09` 先关掉 yaw 控制，`c1f67e0` 把 GYRO/CRAWL 改成 FOLLOW/STOP 并注释掉"左上右中"整段，`d9b0928` 再把剩下的分支全部改成 STOP。目前 6 个挡位里 5 个直接断电。

同时 `targetVx()/targetVy()/targetWz()`（`chassis.h:84` 一带）已无人写入，`mappingRef()` 的 FOLLOW/GYRO 分支（`chassis.cpp:42-76`）成了死路径。**要么按新接口补回挡位表，要么把 FOLLOW/GYRO 分支一并删掉**，不要留半套。

### 3.5 【P1】功率限制完全失效 —— `474c21b`/`d9b0928`

`power_limit.handle()` 的调用被注释后再删除，`controlInit()` 里却还在 `power_limit.init()`（`control.cpp:110`）。后果：没有电容能量/底盘功率约束，比赛规则下可能超功率；且 `WBRChassisPower` 读裁判系统数据仍占 CPU。

### 3.6 【P1】MIT 电机错误恢复是死代码 —— `eab7236`

```cpp
// mit_motor_driver.cpp:50-57  定义了 handle()：读状态码、清错、重发 MOTOR_MODE
// 但全仓库没有任何地方调用 mit_motor_driver[i].handle()
```

`motor_monitor.cpp:272-275` 自己手写了一遍 `motor_->handle()` + `connect_` 检查，绕过了驱动层的 `state_code_` 判断。结果：电机过压/过流/过温进入 error 后，`CLEAR_ERROR` 永远不会发；`state_code_`（`mit_motor_driver.h:111`）也没初始化，首帧前是随机值。

### 3.7 【P1】蜂鸣器实现的 5 个坑 —— `5ad1174` + `22e66ad`

> **状态：已于 2026-10-29 按 5.3 重写**（enum 状态机、单一时间基准、实算分频）。以下保留原始分析。

文件：`bsp_buzzer.cpp:32-73`、`bsp_buzzer.h:84-88`

1. `times_`、`note_`、`start_tick` **没有默认初始化**（`bsp_buzzer.h:84-88`），只靠全局对象零初始化兜底；`Handle()` 里 `times_==9` 会用 `1e6/note_`，`note_=0` 时得到 `inf` 再截断成 `uint32`，属未定义行为；
2. `SetBuzzer()` 的条件 `if (times != 0 || HAL_GetTick() - start_tick > 3000)` 语义反直觉：`times==0`（静音请求）反而要靠 3 s 超时才生效；
3. `start_tick` 写了但**从没被读**（真正用的是 `period_start_tick`），两套时间基准；
4. 静音只设 `ARR=0`，不清 `CCR`（`bsp_buzzer.cpp:60`、`66`、`71`），某些 OC 模式下输出会保持高电平，蜂鸣器可能一直响；
5. `1e6 / note_`、`1e5 / note_` 是 **double 除法**再隐式转整型，且默认 APB1 定时器时钟不是 1 MHz，音高会整体偏移。

另外：`boardBuzzerHandle()` 前 9 s 每 1 ms 调一次 `SetBuzzer(0, 0)`（`control.cpp:366-371`），`start_tick` 被反复刷新，实际效果就是静音；真正的 `playStartUpMusic()` 在工作区里已被注释（`control.cpp:367`）。功能处于"代码在、开关关"的悬空状态。

### 3.8 【P2】电池电压有两套实现 —— `8e7d3e5`

> **状态：已于 2026-10-29 按 5.4 合并**（删除 `get_battery_voltage()`，只保留 `BatteryMonitor` 一个实现）。

- `batteryMonitorHandle()`：`serialToolTask` 每 1 s 调一次，写全局 `battery_voltage`（`battery_monitor.cpp:21-30`）；
- `get_battery_voltage()`：`control.cpp:163-174` 另起一份 `HAL_ADC_Start(&hadc3) + Poll`，**没有任何调用点**（仅第 39 行声明）。

两份代码操作同一个 `hadc3`，一旦有人恢复调用 `get_battery_voltage()` 就会和 1 s 任务抢 ADC；且 `10.f` 分压比在 `battery_monitor.h:18` 和 `control.cpp:173` 各写一份魔法数。

### 3.9 【P2】IMU 自动零漂校准是注释状态 —— `6725f8a`

`imu.cpp:89-109` 20 行校准逻辑被 `/* */` 包住，同时 `imu.h:92-96` 新增的 `gyro_bias[]/accel_sen/calibratecount/calibrate_num` **全部无人使用**（真正的 bias 在 `imu_monitor.cpp:49` 写死）。后果：零漂不随温度补偿，长时间跑会缓慢漂移；但代码看起来"已经实现了"。

### 3.10 【P2】`#if ROBOT_NATIONAL_DEMO` 分支引用未定义符号 —— `eab7236`

`motor_monitor.cpp:148-152` 引用了 `dm6006::p_min` 等，但 `dm6006` 命名空间在整个文件里**没有定义**（只定义了 `dm4310`）。当前宏未定义所以能过编译，**一旦有人打开这个宏就会编译失败**。同段还有 `GMY` 挂在 CAN1 0x16，和现有 `GMY`（can2 id5）会冲突。

### 3.11 【P2】参数提交混入格式噪音 —— `26b2c54`

同一提交里既改了 `KK` 增益、编码器偏移、质量/连杆，又把 `ThetaLErrorResist_t`（已在 5.2 删除）、`LegController_t` 的大括号风格整体重排（`chassis.h:386`、`394-401`）。以后 `git blame` 追参数来源会被格式改动淹没。

---

## 4. 丑陋的临时代码清单

| # | 位置 | 问题 | 处理建议 |
|---|------|------|----------|
| 4.1 | ~~`chassis.cpp:448-451`~~ | ~~双重积分 + 立即覆盖~~ | ✅ 已按 5.1 重写 |
| 4.2 | `chassis.cpp:59` | `ref_.phid = fdb_.phi` 强制覆盖（455 行的注释已随 5.1 删除）；FOLLOW/CRAWL 的朝向逻辑仍等于没写 | 删掉 59 行强制覆盖 |
| 4.3 | ~~`bsp_buzzer.cpp:34`、`63-72`~~ | ~~`times_=9/10` 魔法值、`start_tick` 死字段、double 除法~~ | ✅ 已按 5.3 重写为 enum 状态机 |
| 4.4 | `imu.cpp:89-109`、`imu.h:92-96` | 20 行注释块 + 4 个未使用成员 | 整段删除；要自动校准就单独实现 |
| 4.5 | `control.cpp:49-56`、`90-102`、`197-206` | `Gimbal/Shoot/Gate/Autoaim/CVComm/power_limit` 仍构造并在 `robotReset()` 里调用；`kbctrl` 12 行参数零引用；`fly_height_remain_time` 零引用 | 全部删除，或收进 `#ifdef` 编译开关 |
| 4.6 | `chassis.h:400` | `static_integral` out_max=0 且无人调用 | 删除 |
| 4.7 | `lib/arm_math/arm_cortexM4lf_math.lib`(13 MB)+ `.a`(3 MB)，`RM_frame` 与 `upper_board` 各一份 | 仓库体积 +32 MB；二进制无法 review | 改用 Git LFS，或只保留 `.a` 由 CMake 引用 |
| 4.8 | `mcu/stm32f407/MDK-ARM/stm32f407.uvoptx` | 写入个人 ST-Link 序列号、watch 窗口 | `*.uvoptx` 加入 `.gitignore` |
| 4.9 | ~~`control.cpp:173`、`battery_monitor.h:18`~~ | ~~分压比 `10.f` 两处魔法数~~ | ✅ 已收敛为 `BatteryMonitor` 内部的 `kDividerRatio`（5.4） |
| 4.10 | ~~`chassis.cpp:458`~~ | ~~`math::limit(ref_.l, 0.05, 0.2f)` 混用字面量~~ | ✅ 已随 5.1 改为 `0.05f` |
| 4.11 | `mit_motor_driver.cpp:60-68` | 队列 `memmove` 后再写 `list[4]`，边界靠 `cnt>5` 兜 | 改成 `std::array` + 环形索引 |
| 4.12 | 工作区 28 处 `// TODO refactor`（5.1/5.2/5.4 后剩 20 处） | 只标记不修，且部分插在语句中间 | 见 5.5，逐条消化或换成 issue |

---

## 5. 修改建议（可直接落地）

### 5.1 恢复速度/腿长参考（P0，对应 5a6dba0）✅ 已应用

**状态：已修改 `RM_frame/app/chassis.cpp:437-464`（2026-10-29）。** 改动 4 处：

1. 恢复 `ref_.l_extra` 计算，`ref_.ld` 重新减去 `l_extra`；
2. 删掉 `ref_.sd += ref_.sd_D1 * 1e-3f;` 与 `limit(ref_.sd + ref_.sd_D1 * 1e-3f, ...)` 的**双重积分**，只保留一次；
3. 删掉 `ref_.sd = fdb_.s;`（它让前面积分全部作废，`xd[0]` 永远等于实测值）；
4. `phid_D1` 恢复为 `ref_.wz * 0.5f`；`0.05` 统一写 `0.05f`；顺带删除两行历史公式注释。

落地后的代码：

```cpp
// 处理目标状态
void WBRChassis::handleRef(void) {
  // 机体倾角过大时缩短腿长
  ref_.l_extra = math::limit((fabs(fdb_.theta_b) - 0.15f) * 2.5f, 0.f, 0.25f);

  // 目标状态处理
  ref_.sd_D1 = math::limit(ref_.sd_D1, fdb_.s_D1 - 0.5f, fdb_.s_D1 + 0.5f);
  ref_.sd_D1 = math::limit(ref_.sd_D1, -ref_.sd_D1_max, ref_.sd_D1_max);
  // 速度参考单次积分，并限制在实测位移附近，避免位置参考漂移
  ref_.sd += ref_.sd_D1 * 1e-3f;
  ref_.sd = math::limit(ref_.sd, fdb_.s - 0.2f, fdb_.s + 0.2f);

  ref_.phid = math::loopLimit(ref_.phid, fdb_.phi - PI, fdb_.phi + PI);
  ref_.wz = math::limit(ref_.wz, -ref_.wz_max, ref_.wz_max);
  ref_.phid_D1 = ref_.wz * 0.5f;
  ref_.l = math::limit(ref_.l, 0.05f, 0.2f);
  ref_.ld = math::limit(ref_.l - ref_.l_extra, 0.05f, 0.2f);
  ...
}
```

**两点说明**：

- `phid_D1 = ref_.wz * 0.5f` 在当前 `DIRECT` 模式下等价于 0（`ref_.wz` 只有 `mappingRef()` 的 `GYRO` 分支会赋值），所以这次改动不会改变现在的转向行为；等 FOLLOW/GYRO 挡位恢复后才起前馈作用。若担心 `wz` 残留，可在 `mappingRef()` 非 GYRO 分支里显式清零。
- `ref_.sd` 初值为 0，开机第一个周期会被限幅拉到 `fdb_.s ± 0.2` 内，不会产生位置阶跃。

**待验证**：本报告第 6 节验证表第 1、2、3 步（静止跟随、推杆匀速/松杆、前倾 0.15 rad 收腿）。

> 注：5.1 只处理了参考轨迹；腿角差补偿已按 5.2 删除，见下节。

### 5.2 删除腿角差补偿（P0，对应 b044243）✅ 已应用（删除方案）

**状态：已彻底删除，未采用"恢复补偿"方案（2026-10-29）。** 删除清单：

| 文件 | 删除内容 | 原位置 |
|------|----------|--------|
| `chassis.h` | `ThetaLErrorResist_t` 结构体 + `theta_l_err_resist` 成员（含 `T_err` 与 `PID(150,0,10,0,50)`） | 385-390 |
| `chassis.cpp` | 被注释的 PID 计算块 | 216-221 |
| `chassis.cpp` | `u_.T_bll/T_blr` 中的 `± T_err` 叠加 | 229-232 |

改动后：

```cpp
  u_.T_lwl = locomotion_controller_.u[0] +
             locomotion_controller_.model_adapt.T_lwl +
             locomotion_controller_.speed_limit.T_lwl;
  u_.T_lwr = locomotion_controller_.u[1] +
             locomotion_controller_.model_adapt.T_lwr +
             locomotion_controller_.speed_limit.T_lwr;
  u_.T_bll = locomotion_controller_.u[2];   // 不再叠加 T_err
  u_.T_blr = locomotion_controller_.u[3];
```

**为什么选删除而不是恢复**：

1. `PID(150, 0, 10, 0, 50)` 是 2.6 m 整车时期的增益，mini 的 `KK1~KK6`、`m_b`、`l_1u/l_1d` 已全部重算（`a059ee4`、`26b2c54`），旧增益与新模型不匹配；
2. 调度项 `limit((ref_.ld - 0.05f) * 5.0f, 0, 1)` 在当前腿长区间 0.05~0.2 m 内会**长期饱和**在 1，等于没有调度；
3. 补偿量恒为 0 的写法掩盖了"这条腿角差通道已废弃"的事实，后来人容易误以为它还在工作。

若后续实测仍出现长短腿/roll 偏置，应基于当前模型重新设计（例如用 `observer_.ll.p.theta - observer_.lr.p.theta` 经低通后做小增益反馈），而不是回滚旧代码。旧实现可用下面的命令取回参考：

```bash
git show b044243^:RM_frame/app/chassis.cpp | sed -n '/theta_l_err_resist/,+5p'
```

**验证**：全量构建通过，FLASH `157832 → 157744 B`（−88 B）、RAM `60224 → 60152 B`（−72 B），无新增告警。

### 5.3 蜂鸣器重写（P1，对应 3.7）✅ 已应用

**状态：2026-10-29 重写为"模式枚举 + 单一时间基准 + 整数分频"。**

| 文件 | 改动 |
|------|------|
| `bsp_buzzer.h` | `enum class Mode { kSilent, kBeep, kNote }` 替换 `times_ = 9/10` 魔法值；成员全部默认初始化；删除未使用的 `MusicNote_t` |
| `bsp_buzzer.cpp` | `Handle()` 改成 switch 状态机；`silent()` 同时清 ARR 和 CCR；按 `HAL_RCC_GetPCLK1Freq()` 实算计数频率，50% 占空比 |
| `control.cpp` | `boardBuzzerHandle()` 改用 `Mute()` 显式静音；音乐开关保持注释状态 |

```cpp
void BoardBuzzer::Handle(void) {
  const uint32_t now = HAL_GetTick();
  switch (mode_) {
    case Mode::kNote:                     // 持续音符
      tone(note_);
      break;
    case Mode::kBeep: {                   // 响 N 声后自动静音
      const uint32_t elapsed = now - beep_start_tick_;
      if (elapsed >= static_cast<uint32_t>(beep_count_) * kBeepPeriodMs) {
        Mute();
      } else if (elapsed % kBeepPeriodMs < kBeepPeriodMs / 2u) {
        tone(note_);
      } else {
        silent();
      }
      break;
    }
    default:
      silent();
      break;
  }
}
```

**修掉的 5 个坑**（对应 3.7）：

1. 魔法值 `times_=9/10` → `enum class Mode`，`note_=0` 时不会再算 `1e6/0`；
2. `SetBuzzer()` 语义写清楚：`times==0` 就是静音，正在响时不重入（响完由调用方再次触发），不再靠"3 s 超时"猜；
3. 删掉死字段，时间基准只剩 `beep_start_tick_` 一个；
4. `silent()` 同时 `SET_AUTORELOAD(0)` 和 `SET_COMPARE(0)`，不会因 CCR 残留导致长鸣；
5. 音高改为 **实算而不是硬编码 1 MHz**：`counter_clock = APB1定时器时钟 / (PSC+1)`，这里 PSC=167、APB1 定时器 84 MHz → 计数 500 kHz。原来按 `1e6/note` 算，实际发声**低了整整一个八度**（H1 会变成 ~523 Hz）。`tone()` 里还加了 `arr < 2` 的保护。

**验证**：全量构建通过，FLASH `156412 → 154884 B`、RAM `59240 B`（14.77% / 45.20%），无新增告警。上机确认：上电静音、低压时 8 声短鸣、电压恢复后停止。

### 5.4 电池监测合并（P2）✅ 已应用（类封装 + 独立任务）

**状态：2026-10-29 完成"单一实现 + 类封装 + 滤波 + 迟滞 + 独立 batteryTask"。** 改动 3 个文件：

| 文件 | 改动 |
|------|------|
| `battery_monitor.h` | 定义 `class BatteryMonitor`（`init()/handle()/voltage()/isLow()`），声明 `extern BatteryMonitor battery;` |
| `battery_monitor.cpp` | 8 点滑动平均 + 22.0/22.5 V 迟滞；分压比、ADC 参考、阈值全部收敛到匿名 namespace 的 `constexpr` |
| `control.cpp` | 删除 `get_battery_voltage()` 与 `#include "adc.h"`；两处判断改 `battery.isLow()` |
| `robot.cpp` | 定义 `BatteryMonitor battery(&hadc3);`；新增独立 `batteryTask`（100 ms 周期）接管采样，`serialToolTask` 恢复成纯串口任务 |

类定义：

```cpp
class BatteryMonitor {
 public:
  static constexpr uint8_t kFilterSize = 8;   // 滑动平均点数

  explicit BatteryMonitor(ADC_HandleTypeDef* hadc) : hadc_(hadc) {}

  void init();     // 启动 ADC，开机调用一次
  void handle();   // 采样一次并更新滤波值，建议 100ms 调用

  float voltage() const { return voltage_; }
  bool isLow() const { return low_; }

 private:
  ADC_HandleTypeDef* hadc_;

  float samples_[kFilterSize] = {};
  uint8_t index_ = 0;
  uint8_t count_ = 0;
  bool low_ = true;          // 首次采样前按低压处理，避免未确认电压就上力
  float voltage_ = 0.0f;
};
```

调用点与任务：

```cpp
BatteryMonitor battery(&hadc3);   // robot.cpp:60  定义(与 ultra_cap/board_comm 同层)

osThreadId batteryTaskHandle;
void batteryTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  battery.init();
  for (;;) {
    battery.handle();               // 100ms 采样一次
    osDelayUntil(&tick, 100);
  }
}

osThreadDef(battery_task, batteryTask, osPriorityLow, 0, 128);
batteryTaskHandle = osThreadCreate(osThread(battery_task), NULL);

battery.isLow();                  // control.cpp:120(STOP 条件)、358(低电压鸣叫)
```

**设计取舍**：

- **采样从 `serialToolTask` 拆到独立 `batteryTask`**：串口任务周期 25 ms、栈 1024，让它兼职电池采样会把"低频保护逻辑"和"高频调试输出"耦合在一起；拆出来后电池任务只有 100 ms 周期、128 栈，职责单一，也方便以后加电压日志/曲线；
- **`battery.init()` 放在任务入口**，不再放 `controlInit()`：外设由使用它的任务自己初始化，避免"控制任务先启动、ADC 还没起"的隐式顺序依赖；
- **不再有全局变量和自由函数**：`battery_voltage` 改为 `voltage()`，`batteryLow()` 改为 `isLow()`，滤波缓冲/计数/状态全部是成员；
- **`low_` 初值 `true`** 代替原来的 `count_ == 0 || low_` 短路写法，语义相同但不再是"半个布尔表达式"；
- **阈值用 `constexpr` 而非 `#define`**，作用域受控、有类型；`kFilterSize` 只在类内声明一次，数组长度直接引用它；
- **ADC 句柄构造函数注入**，与 `CapComm(&hcan1)`、`MITMotorDriver(&JML1, &hcan2, ...)` 的既有风格一致；
- **采样周期 1 s → 100 ms**：8 点平均若配 1 s 采样，低压响应要 8 s，对保护来说太慢；100 ms × 8 = 0.8 s 窗口，既能压住加速时的电压跌落，又能在 1 s 内切断动力。

**验证**：全量构建通过，FLASH `157992 → 158072 B`（+80 B）、RAM `60184 → 60192 B`（+8 B），无新增告警。

### 5.5 死代码清理（P1）✅ 已应用

按"删/留"二选一，不要注释：

1. `control.cpp`：删 `kbctrl`、`fly_height_*`、`gimbal/shoot/autoaim/cv_comm/gate/power_limit` 的对象与 `robotReset()` 调用；`controlInit()` 里删 `gate.init()/power_limit.init()`；
2. `chassis.cpp`：删 `tof_.handle()` 注释、`//ref_.sd_D1 = limit(..., -(4 - ref_.l*3)...)` 等历史公式；
3. `imu.cpp/h`：删注释块与 4 个未使用成员；
4. `chassis.h:400`：删 `static_integral`；
5. `mit_motor_driver`：要么在 `motorMonitorHandle()` 里调用 `mit_motor_driver[i].handle()`，要么删掉该方法和 `state_code_`；
6. `motor_monitor.cpp:148-152`：删 `#if ROBOT_NATIONAL_DEMO` 分支，或补齐 `dm6006` 命名空间定义。

**状态：已按"删除 + 接线"执行（2026-10-29）。** 改动 9 个文件：

| 文件 | 改动 |
|------|------|
| `control.cpp` | 删 `Gimbal/Shoot/Gate/Autoaim/WBRChassisPower` 实例与 7 个不再需要的 `extern`/`#include`；删 `robotReset()` 里 5 行跨模块调用、`controlInit()` 里 `gate.init()/power_limit.init()`；删 `kbctrl` 命名空间、`fly_height_remain_time/fly_height_time`、被注释的云台回正块、`robotStartup()` 里未使用的 `flag`；8 处 `// TODO refactor` 一并清掉 |
| `serial_tool.cpp/h` | 删 `txGimbalData/txPowerLimitData/txAutoaimData` 与对应 `extern`、`#include`（函数体里引用的对象已不存在，留着必然链接失败） |
| `client_ui.cpp/h` | 删对 `gimbal/power_limit/autoaim/shoot` 的 `extern` 与绘制：自瞄/能量机关角度数字、`abandon_cap_` 双色电容条、弹丸连发圆环；`followAngle` 改用底盘航向误差 `loopLimit(ref_.phid - fdb_.phi)`；头文件去掉 4 个已无用的 include |
| `board_comm.cpp/h` | 删 `extern Gate gate` 与 `tx_msg_.gate_state = gate.state_`；`gate_state` 成员改为默认 0 |
| `chassis.cpp` | 删 `//tof_.handle();` 与"读取TOF"注释、`handle()` 里的 `// TODO refactor` |
| `base/imu/imu.cpp/h` | 删 20 行自动零漂校准注释块 + `gyro_bias/accel_sen/calibratecount/calibrate_num` 4 个成员 |
| `chassis.h` | 删 `LegController_t::static_integral` |
| `mit_motor_driver.cpp/h` | 选"接线"：`allMotorsHandle()` 改调 `mit_motor_driver[i].handle()`（顺带补上 `CLEAR_ERROR` 恢复路径，见 3.6）；`state_code_` 显式初始化为 `MIT_MOTOR_STATE_DISABLE` |
| `motor_monitor.cpp` | 删 `#if ROBOT_NATIONAL_DEMO` 那段引用未定义 `dm6006::` 的分支 |

**为什么"接线"而不是删 `handle()`**：`handle()` 里的 `CLEAR_ERROR` 是 3.6 记的那个 P1——电机过压/过流/过温后唯一的恢复手段。删掉它等于把 bug 固化成"永远不能恢复"；接到 `allMotorsHandle()` 后，`state_code_ == ENABLE` 时不再重发 `MOTOR_MODE`，行为与原来一致，只在错误/掉线时才多发两条命令。

**仍保留的 `// TODO refactor`（7 处，都对应未完成项，不是遗留垃圾）**：`chassis.cpp:58`（4.2 强制 `phid = fdb_.phi`）、`motor_monitor.cpp:26`（轮电机参数待定）、`bsp_buzzer.cpp:33/52`、`bsp_buzzer.h:83`（5.3 蜂鸣器重写）、`mit_motor_driver.cpp:61/107`（4.11 命令队列边界）。

**验证**：全量构建通过（`cmake -G Ninja`，arm-none-eabi 13.3.1），FLASH `158072 → 155236 B`（−2836 B）、RAM `60192 → 59232 B`（−960 B），除既有的 `-Wwrite-strings`（FreeRTOS `osThreadDef`）外无新增告警。`gimbal.cpp/shoot.cpp/autoaim.cpp/power_limit.cpp` 仍留在 `app/` 下参与编译，但已无任何引用，下一步可整文件删除。

### 5.6 提交/分支规范（防止再次出现"临时提交进主干"）

- 所有 `test:` / `param:` 提交先落在 `mini-debug` 分支，验证后 `git rebase -i` 压成一条带说明的提交再合入 `mini`；
- 参数只用一处定义：`chassis.h` 与 `doc/controller_param/*.txt` 二者选一为源，另一个由脚本生成（现在靠手抄，`26b2c54` 已经出现 `KK` 同步、`KA/KB` 未同步的偏差）；
- `.gitignore` 补：`*.uvoptx`、`*.uvguix.*`、`*.lib`（走 LFS）、`.dsh_tmp/`。

---

## 6. 建议的验证顺序

改完 P0/P1 后，按下面顺序上机，避免一次动太多变量：

| 步骤 | 验证项 | 判据 |
|------|--------|------|
| 1 | 静止直立，`ref_.sd` 是否跟随 `fdb_.s` | 串口打印，误差 < 5 mm，无持续爬行 |
| 2 | 推杆给速度，松杆 | 能匀速、松杆减速到 0，不振荡 |
| 3 | 人为前倾 0.15 rad | `l_extra > 0`，腿长 `ld < l`，能回正 |
| 4 | 单腿垫高 10 mm | 补偿已删，改看 roll（`psi`）是否由 `psi_ppid/psi_spid` 拉回；若持续偏，说明需要重新设计腿角差通道 |
| 5 | 电池从 24 V 缓慢拉到 < 22 V，再回升到 > 22.5 V | 22 V 以下进 STOP + 蜂鸣 8 声；回升过 22.5 V 才恢复，不在 22~22.5 V 之间反复跳 |
| 6 | 拔关节电机 CAN | 进 STOP（`chassis.cpp:92-95` 已有该保护） |

---

## 7. 附：复查用命令

```bash
# 2026 全部提交（含分支）
git log --all --since=2026-01-01 --until=2027-01-01 --date=short \
        --pretty=format:'%h %ad %an %s' --stat

# 5.1/5.2 已应用：查看本次改动
git diff -- RM_frame/app/chassis.cpp RM_frame/app/chassis.h

# 取回被删的腿角差补偿旧实现（仅作参考）
git show b044243^:RM_frame/app/chassis.cpp

# 若以后仍要整体回滚 5a6dba0（会同时推翻 5.1，慎用）
git show 5a6dba0 --stat

# 只看某个提交的改动
git show 26b2c54 -- RM_frame/app/chassis.h

# 检查 KA/KB/KK 是否与 doc/controller_param 一致
git diff --no-index <(sed -n '1,60p' doc/controller_param/KK.txt) <(sed -n '250,320p' RM_frame/app/chassis.h)
```

---

## 8. 结论摘要

- **该留**：电机驱动/协议（`eab7236`）、绝对编码器（`6572f51`）、IMU 方向（`36922f3`）、DJI 发包（`90c28f4`）、headless 裁剪（`bcce860`/`474c21b`/`d9b0928`）、电池监测（`8e7d3e5`）、mini 参数基线（`a059ee4`）、编译修复（`c39c7d3`/`5d66ca3`）。
- **该修**：`5a6dba0`（双重积分+腿长补偿关闭）→ **已修（5.1）**；`b044243`（补偿 PID 注释但输出仍在用）→ **已删（5.2，走删除方案）**；`26b2c54`（参数正确但格式噪音，且需上机验证 psi 增益 42→4.2）→ 待验证；`22e66ad`（音乐实现有 5 个坑且当前被禁用）→ **已重构（5.3）**，音乐开关仍保持注释。
- **该删**：`imu.cpp` 注释块、`control.cpp` 云台/发射残留对象与 `kbctrl`、~~`get_battery_voltage()` 重复实现（已随 5.4 删除）~~、`static_integral`、`#if ROBOT_NATIONAL_DEMO` 死分支、工作区剩余 20 处 `// TODO refactor` 中已无意义的部分。
