# dicision · 任务赛决策机 —— 使用与调参手册

> 对应《主状态机.md》**v3（2026-07-06 极简定案版）**。
> 目标（用户原话）：**"先能完成放两个箱子再说。"** —— 只跑1趟、搬2箱、完赛即可。只搬 back 排、选一条异色缝（分数相同选路径最短）。
> ⚠ v3 协议变更：放置子状态 `05 01`=放左托盘 / `05 02`=放右托盘（原"层数"作废），须与 X5 对齐。

一句话看懂这套系统：**决策机 B 是教练**，比赛全程只干两件事——用串口指挥 X5（眼和手：识别、抓、放），用 `navigate_to_pose` 指挥导航 FSM（腿：走到打好的点）。教练自己不碰球，所以这个包 90% 的行为都由**参数**决定——参数就是场边那面旋钮墙，本手册把每个旋钮逐个讲清楚：干什么的、默认多少、什么时候拧、拧错了什么症状。

```
dicision/
└── src/mission_director/
    ├── mission_director/
    │   ├── mission_director_node.py   节点入口 + 全部参数声明 + start/abort 服务
    │   ├── arm_link.py                串口线：协议编解码 + 收帧线程 + dry_run 仿真
    │   ├── nav_client.py              腿：读路点 yaml → 发 goal → 阻塞等结果
    │   ├── strategy.py                脑：答案 + 8 箱色 → 趟队列 / 放置规划（纯逻辑，无 ROS）
    │   └── mission.py                 教练本人：S0 初始化→S1 识别→S2 规划→搬运→回家
    ├── config/mission.yaml            旋钮墙本体
    ├── launch/mission.launch.py       只起决策机（导航、底盘另起，见 §1.3）
    └── test/test_offline.py           离线自测 20 项（不接任何硬件）
```

---

## 1. 分阶段跑法（从简到繁，每级有验收）

### 阶段 0 · 串口台架（桌上做，狗完全不动）

B 和 X5 用杜邦线交叉对接（B.TX↔A.RX、B.RX↔A.TX、共地），**不需要 ROS、不需要 build**，一段 Python 直接盘它：

```bash
cd ~/dog_auto/dicision/src/mission_director
python3 - <<'EOF'
import logging, time
logging.basicConfig(level=logging.INFO, format="%(message)s")
from mission_director.arm_link import ArmLink, S1_INIT, S1_RECOGNIZE, S1_SCAN

arm = ArmLink(logging.getLogger("bench"), port="/dev/ttyUSB0")   # 设备号按实际改
arm.open()
for s1, name in [(S1_INIT, "初始化"), (S1_RECOGNIZE, "数字识别"), (S1_SCAN, "扫描8箱")]:
    print(f"\n=== 发 {name} ===")
    arm.send_cmd(s1)
    t0 = time.time()
    while time.time() - t0 < 60:                 # 每条命令最多等 60s
        f = arm.get_event(timeout=0.5)
        if not f: continue
        print("收到:", f)
        if f.get("kind") == "state" and f.get("state3") == 0x02:
            break                                 # s3=02 完成 → 下一条
arm.close()
EOF
```

**验收**：三条命令都等到 `s3=02`，且识别/扫描后打印出数字帧（答案 + 8 箱色），日志 hex 无丢帧。TX 有日志、RX 一片死寂 → 十有八九线序没交叉或没共地。

### 阶段 1 · 单趟闭环（当前目标 ★）

`home → entry → gapXX_pre → gapXX_back → 抓右 → 抓左 → 直穿减速带 → zoneXX_approach → 横移 → 放两箱`。两箱是**异色**的（选缝时已保证），各落进自己颜色的区即验收。放完狗停在原地，怎么回来是阶段 2 的事。

先无实物彩排一遍（不开串口、导航纯仿真，本机跑全流程看日志）：

```bash
cd ~/dog_auto/dicision && colcon build --symlink-install && source install/setup.bash
ros2 launch mission_director mission.launch.py dry_run:=true
```

实机跑，四个终端按序起（顺序错了 FSM 等不到定位）：

```bash
# ① 狗底盘        ros2 launch nav_bringup dog_bringup.launch.py
# ② 定位          ros2 launch nav_bringup dog_navigation.launch.py map_name:=arena run_nav2:=False
# ③ 导航 FSM      ros2 launch simple_nav_fsm simple_nav.launch.py
# ④ 决策机（阶段1建议手动扳机，人喊开始再触发）
ros2 launch mission_director mission.launch.py auto_start:=false
ros2 service call /mission_director/start std_srvs/srv/Trigger   # 扳机
ros2 service call /mission_director/abort std_srvs/srv/Trigger   # 急停（取消导航+微调归零）
ros2 topic echo /mission_director/phase                          # 旁观它现在干到哪一步
```

### 阶段 2 · 返程 + 多趟（可选，暂缓）

v3 只要求 1 趟就完赛。若日后想搬更多，候选方案留档在主状态机.md §10.3（原路穿缝回 entry，下一趟另选颜色不重复的异色缝；注意列内 back 先于 front 的约束），到时候讨论定案。

### 阶段 3 · 鲁棒（暂缓）

重试、恢复预算、比赛时间预算，旋钮都已在墙上（§3-D/E），阶段 1 先全部保持默认。

> v3 三条铁律：**只抓 back 排**（idx0-3，front 全弃）· **一趟两箱必异色**（同色缝弃）· **微调只在停稳后发生**（停-发-调-干）。

---

## 2. 打点巡场清单（15 个点，人牵狗像导游踩点走一圈）

比赛地图建好后，人牵着狗**按下面顺序**走一圈，每到一个位置把狗**摆成届时想要的朝向**（打点记录的就是当下位姿），然后：

```bash
ros2 run simple_nav_fsm record_waypoint <点名>
```

| # | 点名 | 站哪、朝哪 | 叮嘱 |
|---|------|-----------|------|
| 1 | `home` | 启动区中央，朝场内 | 识别+扫描都在这原地做（定案），要让 X5 看得到题板和 8 箱 |
| 2 | `entry` | 进场后第一个安全位，朝北 | 每趟的出发集结点，离墙留余量 |
| 3-5 | `gap01_pre` `gap12_pre` `gap23_pre` | 各缝正前方，**身体正对缝** | FSM 无避障，进缝前必须已摆正；gap12_pre 可能≈entry |
| 6-8 | `gap01_back` `gap12_back` `gap23_back` | 缝中站位，朝北 | **只打 back 排**（靠减速带 idx0-3）；front 排 v3 不搬、不打；狗居中两侧箱距对称臂才够得着 |
| 9-11 | `zone01_approach` `zone12_approach` `zone23_approach` | **各缝正前方**、减速带北侧，朝归位区 | 每缝一个：必须和对应缝共线，过带那段才是纯直线 |
| 12-15 | `place_green` `place_gray` `place_blue` `place_red` | 各色区正前，朝区（箱子向前放） | 场地左→右 = 绿0·灰1·蓝2·红3（图册图10，别按 v1 旧图的红灰蓝绿打） |

打完自查一眼：`cat ~/.simple_nav_fsm_waypoints.yaml` 应该正好 15 个名字（v3 不打 3 个 front 点）。决策机开跑前也会自动点名（见 `strict_waypoints`），缺谁报谁。

---

## 3. 旋钮墙：参数逐个讲

> 都在 `config/mission.yaml`，launch 时可用 `-p 名:=值` 临时覆盖。标 ⚠ 的 = 上机前必须现场确认。

### A. 串口两根线

**`serial_port`** = `/dev/ttyUSB0` ⚠ —— B 和 X5 之间那根线在系统里的名字。插上后 `ls /dev/ttyUSB*` 看真名；报 Permission denied 就 `sudo usermod -aG dialout $USER` 再重新登录。插拔过 USB 口，编号可能会漂，比赛机建议用 udev 规则固定别名。

**`serial_baud`** = `115200` —— 波特率，双方约死的，动它 = 通讯变乱码。永远别拧。

### B. 协议翻译官（帧 ↔ 语义）

**`color_names`** = `["", red, blue, gray, green]` —— 颜色码 1-4 → 名字的查表（下标 0 占位）。协议定死 1红 2蓝 3灰 4绿，不要动；它同时决定放置路点名 `place_<名>`。

**`answer_to_color`** = `[0,1,2,3,4]` —— 答案值 → 高分颜色码，默认恒等（答案 2 → 颜色码 2 = 蓝，主状态机 §6.1）。X5 发来 1-4 以外的值 → 自动按"无高分"兜底照搬 8 箱。除非赛规临场变映射，否则不拧。

**`move_unit_scale`** = `1.0` ⚠ —— 移动帧 x/y/z 的单位换算成米。协议写的是米所以默认 1.0；台架时若发现"X5 说挪 5cm、帧里却是 50"，那是 X5 发了毫米 → 改 `0.001`，一个旋钮救全场。

**`move_frame_sign`** = `[1,1,1]` ⚠ —— x/y/z 三轴的正负修正。症状极其好认：X5 让狗往右蹭、狗偏往左蹭 → 把第一位改成 `-1`；前后反了改第二位。台架核对③时拿实物摆一次就能定。

### C. 抓与放的章法

**`grab_first`** = `right` —— 站进缝里先抓哪边。抓是"一次一个托盘"串行发命令（先 `AB 04 02` 右、再 `AB 04 01` 左），换成 `left` 只是换个先后，机械上哪边顺手听 X5 的。

**`place_by_tray_color`** = `true`（v3 放置逻辑）—— 放置不再靠"假定取箱顺序"，而是**到区后比对颜色**：抓箱时 B 记住左/右托盘各是什么色；到某个归位区，若区色==左托盘色就发 `05 01`（放左托盘），若==右托盘色就发 `05 02`（放右托盘）。两托盘异色，必有且仅有一侧匹配。先到哪个区由路径远近算出（近的先去）。原 `unload_order` 参数已废弃。

**`gap_chain`** = `[back]` —— v3 **只搬 back 排**（靠减速带 idx0-3），front 排完全不搬。别加 front：一趟只取一条缝的 back 两箱即完赛。多趟阶段若要搬 front，务必遵守"同列 back 先被搬走才能搬 front"。

**`zone_capacity`** = `2` —— 每个色区最多叠几箱（400mm 区放 250mm 箱，第二只只能摞上去=第2层）。规则不变不拧。

**`overflow_policy`** = `skip` —— 某色区满了还来同色箱怎么办：`skip` 认怂跳过（保平），`nearest_free` 沿归位横线找最近有空的区硬放（搏分，可能不得分但清了托盘）。若核对④确认每色恰好 2 箱，这旋钮一辈子不会触发。

**`place_line_order`** = `[green, gray, blue, red]` —— 归位区从左到右的真实顺序，`nearest_free` 找"最近空区"靠它量距离。已按图册图10 更正（v1 文档画反了）；到了场地和实物对一眼即可。

### D. 耐心值（超时，单位秒）

**`nav_timeout_sec`** = `30` —— 单段导航的耐心。场内单段普遍 1~3m，30s 很富裕；超时会主动 cancel 掉 goal 再重试。狗慢或段长可加大，但它 × 段数 ≤ 比赛总时长，自己心里有数。

**`grab_timeout_sec`** = `40` / **`place_timeout_sec`** = `40` —— 单次抓 / 单次放（放含微调往复）的耐心。台架时掐个真实秒表，按"实测 × 1.5"回填最稳。

**`arm_timeout`** = `180` —— 其余机械臂命令（初始化/识别/扫描）的耐心。X5 初始化和视觉识别可能真的慢，先给足；台架实测后再收紧。

**`nav_server_wait_sec`** = `15` —— 开跑前等 FSM 的 `navigate_to_pose` 服务上线多少秒。终端③没起或起晚了，这里会报"导航服务器未就绪"。

**`match_time_budget_sec`** = `0`（关）—— 比赛总时间预算，超了就跳过剩余趟直接回家保平。**阶段 1 保持 0**；阶段 3 按规则时长减去安全余量再开。

### E. 跌倒了爬起来（重试与恢复）

**`nav_retry` / `grab_retry` / `place_retry`** = 各 `1` —— 对应动作失败（s3=03 或超时）后原地再试几次。重试也是要吃时间的，1 次是性价比点；台架发现 X5 偶发抽风再加。

**`max_recover`** = `3` —— 全场恢复预算。导航段彻底失败、抓空、放砸各记 1 次；刷爆额度 = 状态已不可信，跳过剩余任务直接回家，别在场上表演连环翻车。

### F. 蚂蚁步（微调 —— v3：抓取+放置两阶段，距离值平稳控速）

**`nudge_mode`** = **`displacement`** ⚠ —— 距离值模式（v3 定案）：机械臂回传"还差多少距离"，B 用自写函数把它换算成**底盘小速度×持续时间**，分步平稳逼近、以稳为主，不照位移一把冲。抓取、放置两阶段同一套。`direction_flag`(v1) 作废留档，别选。

**`nudge_v`** = `0.05` m/s —— 平稳挪的蚂蚁步速度：1 秒 5cm，**以稳为主**。太快（>0.1）容易过冲，冲过头机械臂又得重新对；还会被 cmd_vel_guard 钳位（vx≤0.15/vy≤0.10）导致"以为挪够实际没到"。0.05 起步，实测过冲就再降。

**`nudge_w`** = `0.15` rad/s、**`nudge_z_as_yaw`** = `false` —— 旋转微调预留位。协议移动帧只有 x/y/z 平移，X5 若日后用 z 位表达旋转，把开关打开、z 的符号就变成 ±nudge_w 的原地转。X5 没定之前保持 false。

**`nudge_max_duration`** = `2.0` s —— 单次微调的安全上限（0.05m/s × 2s = 最远 10cm）。X5 一口气要 30cm？多半是帧错了，宁可截断让它再发一次，也不让狗盲奔。

**`nudge_cmd_vel_topic`** = `/cmd_vel_raw` —— 微调速度从这里发，经 cmd_vel_guard（限幅 + 断流归零兜底）再到狗。FSM 空闲时不占这条路，不会打架。**别直发 `/cmd_vel`** 绕开保险丝。

### G. 路点与开关

**`waypoints_file`** = `~/.simple_nav_fsm_waypoints.yaml` —— 和 FSM 共用同一本地址簿（定案：直接复用，绝不维护两份）。

**`strict_waypoints`** = `true` —— 开跑前按 15 点名册点名，缺一个就拒跑并报出名字。**比赛日保持 true**——它拦下的是"打点漏了一个、狗跑一半没地址"这种最冤的翻车。只有本机联调想跳过打点时才临时 false。

**`return_via`** = `[]` —— 阶段 2 的返程途经点列表，阶段 1 用不上，留空。

**`auto_start`** = `true` / **`start_delay`** = `2.0` —— 扳机方式。比赛=true（上电即跑，延迟 2s 给各节点缓冲）；**调试强烈建议 `auto_start:=false`**，人喊开始再 call `~/start`，不然一 launch 狗就窜出去。

### H. 无实物彩排（dry_run 全家桶）

**`dry_run`** = `false` —— 总开关。true = 不开串口、导航睡觉假装到点，纯逻辑彩排整场流程，笔记本上就能跑。**上实机前记得关**（launch 默认就是 false）。

**`dry_run_answer`** = `2`、**`dry_run_box_colors`** = `[1,2,3,4,2,3,1,4]` —— 彩排剧本：假装识别出的答案和 8 箱色。默认剧本正是主状态机 §6.3 的例子（答案 2 → 高分蓝在 idx1/idx4 → 先跑 gap01）。想验哪种局面就编哪种剧本。

**`dry_run_step_sec`** = `0.3`、**`dry_run_nav_sec`** = `0.5` —— 彩排里机械臂每步 / 每段导航假装耗时，调快调慢看日志节奏用。

**`dry_run_nudge`** = `false` —— true 时彩排放置会假装来一次"s3=04 + 移动帧"，专门验微调链路（含 20Hz 持续发布与到位即停）。

---

## 4. 阶段 1 推荐配置（可整段核对 mission.yaml）

跑前 30 秒扫一遍这几行 + `serial_port` 即可，其余全默认：`nudge_mode: "displacement"`（距离值平稳）、`place_line_order: [green, gray, blue, red]`（绿灰蓝红）、`gap_chain: [back]`（只搬 back 排）。

---

## 5. ⚠ 文档已定案、代码待同步的 2 处（阶段 1 开工第一件事）

现仓库代码是 v2 逻辑（四趟8箱），v3 极简后需要按下面同步（开工第一件事）：

1. **策略 `strategy.py`**：从"选2条缝、四趟队列"改为"选1条异色缝"——只看 back 排 idx0-3，判每缝两箱是否异色、含高分优先、**分数相同选路径最短**，输出单趟。
2. **只搬 back**：去掉 front 排相关逻辑与 3 个 front 打点。
3. **抓取加微调**：`mission.py` 抓取段接入"停-发-调-干"（原 v2 抓取无微调）。
4. **微调改距离值平稳控速**：把"位移量一把走"换成"距离→速度×时间分步平稳挪"，加 `nudge_max_duration` 截断。
5. **路点表**：`CANONICAL_WAYPOINTS` 的 `zone_approach` 1 个 → `zone01/12/23_approach` 3 个；每趟第⑤段 goto 本缝对应的 `zoneXX_approach`。
6. **放置靠托盘记色+区色比对**：抓时记左右托盘色；到区后比对区色，区色==左托盘色发 `05 01`、==右托盘色发 `05 02`（放置子状态已重定义为放左/放右托盘）。
7. **两区访问顺序按写死偏好方向**（主状态机 §6.6）：左缝靠左优先、右缝靠右优先、中缝中间优先、平票先左后右——查表出顺序，不算真实距离。

这批改完即达阶段1单趟闭环。改动不小但方向明确，可先 `dry_run` 验证再上实机。

---

## 6. 故障速查（症状 → 大概率原因 → 拧哪）

| 症状 | 大概率原因 | 处置 |
|------|-----------|------|
| TX 有日志、RX 死寂 | 线没交叉 / 没共地 / 设备号错 | 查线序；`serial_port` |
| RX 全是"丢帧/坏帧尾" | 波特率不符 / 线太长干扰 | 双方确认 115200；缩短线、加地 |
| 发命令 X5 毫无反应 | X5 还没起来 | 核对⑤，B 侧晚点发或加重发 |
| 微调方向反 / 距离离谱 | 轴符号 / 单位 | `move_frame_sign` / `move_unit_scale` |
| 微调"没挪够就停" | 被 guard 钳位或撞上限 | 降 `nudge_v`；查 `nudge_max_duration` |
| 一开跑就拒跑"缺路点" | 15 点没打齐 / 名字打错 | 按 §2 补点；日志会报缺谁 |
| 报"导航服务器未就绪" | 终端③ FSM 没起 | 按 §1.3 顺序重来；`nav_server_wait_sec` |
| 段段导航超时 | 定位没起 / 地图不对 | 查终端②；`nav_timeout_sec` 不是病根别乱加 |
| 异色两箱放反了区 | 托盘记色或横移判断错 | 查抓取时左右托盘色记录、放置选区逻辑 |
| 狗一 launch 就窜 | auto_start | 调试永远 `auto_start:=false` |

---

## 7. 赛前和 X5 当面核对的 5 句话（贴机器上）

① 微调距离的单位/含义？（"还差多少米"）实物验一次 → 定 B 的距离→速度×时间换算函数
② 微调 x右+/y前+ 方向摆实物验一次 → `move_frame_sign`
③ 抓取阶段确认也会发 s3=04+微调距离（v3 已按"抓放都微调"设计）
④ 每色恰好 2 箱？（已口头确认）→ 溢出逻辑永不触发
⑤ X5 上电到能收命令要几秒？→ B 初始化的等待策略
