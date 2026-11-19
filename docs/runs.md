# 实验记录

## 源码基线

官方源码为 `j96w/DexCap@4b0bed0966c87368f3cde4476aadb7585c3b94b5`。`main` 的 `5ca7487` 修复了 HDF5 原始动作与归一化统计之间的精度不一致。测试在原版 `SequenceDataset` 上得到 `-1.00000207`，修复后通过。

## 本地硬件预验证

使用 Blackwell RTX 5070 Ti Laptop、Python 3.11、PyTorch `2.10.0+cu130`，从官方 Wiping HDF5 的第一条序列读取前 24 帧。PointNet 和 `SetXFPCDEncoder` 各跑 2 个 epoch、每个 3 步。两次都从 epoch 2 checkpoint 加载并输出有限的 46 维动作。这些运行用于验证训练与推理链路。

## pro1

项目位于 `~/dexcap-repro`。Conda 环境位于 `envs/train`，Python 3.11.18，PyTorch `2.10.0+cu130`，GPU 为 RTX PRO 5000 72 GB Blackwell。CUDA 张量运算已成功执行。

`60d5b3a` 的 `scripts/run_experiment.sh` 已在 `pro1` 的三个实验入口分别跑通 `smoke`。Wiping PointNet 使用当前项目源码，其余两组使用 `worktrees/setxf` 的 `4263783`。三个第 2 轮 checkpoint 均输出有限的 46 维动作。入口日志为 `runs/repro-entrypoint-pointnet.log`、`runs/repro-entrypoint-wiping-setxf.log` 和 `runs/repro-entrypoint-setxf.log`。   [运行方式](reproduction.md#运行一次完整实验)

使用复现提交 `55963d8` 和该环境的 PyBullet，回放 Wiping 官方 `demo_0` 共 111 步，点云为 `10000 × 6`。两件物体触发抓取，海绵与盘子接触 4 步，相对盘面移动 0.0131 m，完整擦拭为 0 次。服务器记录为 `runs/wiping-demo0-pro1.json`。

使用 `434cd64` 的台面和物体尺寸校准及柔性手部接触回放同一示范。两件物体均触发抓取，海绵与盘子接触 42 步，接触时累积相对移动 0.3730 m，完整擦拭仍为 0 次。服务器记录为 `runs/wiping-demo0-table-aligned-pro1.json`。 [场景参数与判据](simulation.md)

使用 `76a296c` 回放 Packaging 官方 `demo_0` 共 131 步，点云为 `10000 × 6`。累计 52 个右手与包装物接触点、1197 个左手与盖板接触点。包装物入盒，盖板未闭合。服务器记录为 `runs/packaging-demo0-passive-pro1.json`。

PointNet 和 `SetXFPCDEncoder` 在 Wiping 官方数据的前 24 帧分别完成两轮训练，每轮三个梯度步。两个 checkpoint 均输出有限的 46 维动作，并分别读取仿真观测执行了 12 步闭环控制。这两次闭环均为 0 个海绵与盘子接触步。服务器记录为 `runs/wiping-policy-smoke-pro1.json` 和 `runs/wiping-setxf-smoke-pro1.json`。

同一批 Wiping 接口 checkpoint 也执行了每种策略 20 回合、每回合 12 步的闭环试验。两个策略逐回合的盘子和海绵初始坐标完全相同，均为 0/20 仿真成功。这批结果用于验证随机场景和配对流程。服务器记录为 `runs/wiping-pointnet-20trial-smoke-pro1.json` 和 `runs/wiping-setxf-20trial-smoke-pro1.json`。

Packaging 的 SetXF 策略也用官方 `demo_0` 的前 24 帧完成 6 个梯度步。同一个 checkpoint 读取 Packaging 仿真点云与手部状态，执行了 12 步，记录到右手与物体接触并触发抓取，物体尚未入盒。服务器记录为 `runs/packaging-setxf-smoke-pro1.json`。

使用 `cb561d3` 的可移动盒体与柔性手部接触回放官方 Packaging `demo_0`。131 步中盒体留在桌面，最接近桌面中心时比起点近 0.0519 m，终点与起点相距 0.0664 m。包装物仍由右手抓持，盒盖打开，仿真判定未成功。此前刚性手部接触的回放在第 110 步加速盒体，随后盒体滑出桌面。服务器记录为 `runs/packaging-demo0-compliant-pro1.json`。

Packaging 的 6 梯度步 SetXF checkpoint 在当前场景也完成了 12 步闭环。记录到右手抓持、左手碰触盒体和盖板，盒体向桌面中心最大推进 0.0198 m，仿真判定未成功。服务器记录为 `runs/packaging-setxf-compliant-smoke-pro1.json`。

使用 `70561e7` 的点云裁剪重新执行 Wiping PointNet 和 Packaging SetXF 的接口短跑。两者都以 `10000 × 6` 点云完成 12 步闭环，仿真结果均为未成功。记录为 `runs/wiping-policy-cropped-smoke-pro1.json` 和 `runs/packaging-setxf-cropped-smoke-pro1.json`。

使用 `8e9a89b` 的镜像左手重跑两个官方示范。Wiping 的 111 步中有 47 步海绵与盘子接触，接触期间累计相对移动 0.2979 m，完整擦拭 0 次。Packaging 的 131 步中，盒体一度向桌面中心靠近 0.0482 m，包装物仍被右手抓持、盖板打开。服务器记录为 `runs/wiping-demo0-mirrored-pro1.json` 和 `runs/packaging-demo0-mirrored-pro1.json`。

使用 `90d5e6c` 的凹面盘和正面接触判据回放 Wiping。111 步中有 32 步接触盘体，其中 0 步属于正面擦拭，完整擦拭为 0 次。服务器记录为 `runs/wiping-demo0-dish-pro1.json`。在同一 Conda 环境运行 `python -m unittest discover -s tests -p test_wiping_surface.py`，确认正面往返计 2 次，盘底往返计 0 次。

使用 `b25cbd0` 和 Packaging 的接口短跑权重随机摆放 5 次，每回合 12 步。5 组盒位和物体位置均在设定的采样范围内，入盒为 0/5，完整成功为 0/5。服务器记录为 `runs/packaging-setxf-5trial-smoke-pro1.json`。 [摆放与指标](simulation.md)

使用 `fe4678a` 将同一个 6 梯度步 SetXF 接口权重在六种物体上各运行 5 回合，每回合 6 步。核对了全部 30 回合的 `10000 × 6` 点云、六个物体和几何参考示范，以及五组在各物体间完全相同的摆放坐标。入盒与完整成功均为 0/30。服务器记录为 `runs/packaging-*-six-object-smoke-pro1.json`。这批短跑用于验证多物体评估入口。 [物体尺寸和来源](simulation.md#packaging-仿真)

使用 `e5227cb` 的接触链节抓取回放两条官方示范。Wiping 的盘体接触从 32 步增至 51 步，正面接触仍为 0。Packaging 的盒体一度向桌面中心靠近 0.0987 m，包装物仍被右手抓持，盖板打开。服务器记录为 `runs/wiping-demo0-finger-grasp-pro1.json` 和 `runs/packaging-demo0-finger-grasp-pro1.json`。

使用 `df4d58e` 的相向双指夹盘条件回放官方 Wiping。两件物体均触发抓取，111 步中有 41 步盘体接触、0 步正面接触、0 次完整擦拭。服务器记录为 `runs/wiping-demo0-opposing-grasp-pro1.json`。

`b2741ee` 改用竖直擦拭计数。在 `pro1` 的倾斜盘面回归试验中，同一侧上下往返计 2 次，横向往返和盘底往返均计 0 次。官方 111 步回放仍为 41 步盘体接触、0 步正面接触、0 次完整擦拭。服务器记录为 `runs/wiping-demo0-vertical-strokes-pro1.json`。 [当前计数定义](simulation.md)

`48d87e6` 使用手指张开触发 Packaging 的物体释放。用官方动作回放时，第 51 步松手，包装物落在盒外。相同场景的 `pro1` 回放结束时 `object_released=true`、`object_in_box=false`、`lid_closed=false`。服务器记录为 `runs/packaging-demo0-hand-opening-pro1.json`。

`432b9d2` 接入原始深度测得的初始场景。同一释放条件下，最大盒体倾角从约 58.2° 降至 1.6°，持物到盒中心在盒平面内的最近距离从 24.1 cm 缩至 7.8 cm。最终盒体平移距离从约 9.0 cm 增至 23.2 cm。服务器回放完成 131 步，包装物已释放，仍未入盒或合盖。记录为 `runs/packaging-demo0-raw-calibrated-pro1.json`。

用新场景本地确认六种物体在 30 组初始场景的裁剪点云中都可见，并各有一种能装入盒中的姿态。`pro1` 用现有 SetXF 的接口权重运行了绿罐的 5 个摆放，每回合 6 步，入盒和完整成功为 0/5。记录为 `runs/packaging-green-can-calibrated-smoke-pro1.json`。

完整 Wiping 和 Packaging HDF5 正在续传。完整数据到位后，`tmux` 会话 `dexcap-experiments` 下的 Wiping PointNet、Wiping SetXF 和 Packaging SetXF 作业各先执行完整文件短跑，成功后继续对应长训。`dexcap-full-sim` 等待每个成功完成的长训保存 epoch 3000 的 checkpoint。两种 Wiping 策略各执行 20 组配对摆放，Packaging 的六种物体各执行 5 组摆放。每回合 400 步，结果写入对应的 `runs/*-final-sim.json`。 [服务器项目操作路径](reproduction.md)

### Wiping 单示范训练

在等待完整 HDF5 时，从同一官方数据版本提取 `demo_0` 的全部 111 帧，保存为 `data/derived/wiping-official-demo-0.hdf5`。PointNet 使用 `37361d1`，SetXF 训练使用 `4263783`，仿真均使用 `37361d1`。两组保留 `pointcloud-full.json` 的样本、优化和扩散设置，只调整为 5 个 epoch、每轮 100 个梯度步，并于第 5 轮保存。均使用种子 1。

| 编码器 | 各轮训练损失 | 400 步盘体接触 | 正面接触 | 完整擦拭 |
|---|---|---:|---:|---:|
| PointNet | 1.0064、0.7120、0.4518、0.3940、0.3568 | 0 | 0 | 0 |
| SetXF | 1.0063、0.7015、0.4467、0.3900、0.3542 | 22 | 0 | 0 |

两组第 5 轮权重均输出有限的 46 维离线动作，并在同一参考摆放完成 400 步仿真闭环。各自的日志、配置和仿真 JSON 在服务器的 `runs/wiping-demo0-{pointnet,setxf}-seed1-pilot-20261119T091714Z*`。训练损失只描述这一条示范上的拟合，正式配对评估仍使用完整数据训练的权重。

## 官方原始数据重建

从官方 Wiping 原始 ZIP 提取第一段示范的标定、`frame_0` 和 `frame_2220`，以单帧夹具运行无界面的 PyBullet 指尖 IK 和 RGB-D 点云构建。输出一个 HDF5 时间步、46 维动作和 `1000 × 6` 点云。 [重建命令与验证范围](raw-retarget.md)

## 本地 Plate Wiping 仿真

| 输入 | 控制步 | 海绵与盘子接触步 | 相对盘面移动 | 完整擦拭 | 仿真判定 |
|---|---:|---:|---:|---:|---|
| 官方 `demo_0` 动作回放 | 111 | 9 | 0.0596 m | 0 | 未成功 |
| 6 梯度步的 PointNet 接口 checkpoint | 12 | 0 | 0 m | 0 | 未成功 |
| 校准后重跑官方 `demo_0` | 111 | 43 | 0.4166 m | 0 | 未成功 |

三个回合均从 Bullet 场景重建 `10000 × 6` 的点云，并报告两件物体触发抓取。12 步闭环用于检查仿真观测、DDIM 推理和动作回写。 [场景和指标](simulation.md)

对照官方首帧裁剪平面，在相同场景的 10000 个 Wiping 模拟点中，裁剪前只有 1067 个位于保留侧。加入裁剪后，10000 个点都在保留侧，官方首帧也满足同一阈值。Packaging 首帧通过同样的裁剪检查，其物体和桌面点比例仍与录制数据存在差异。

另做接触判据诊断。官方动作先触发两件物体的抓取，随后让海绵在盘面坐标中按脚本做往返运动。连续 361 步都有物理接触，记录 2 次跨盘和 0.548 m 位移。脚本运动用于验证计数逻辑。官方动作回放的接触集中在盘缘，较早固定的抓取姿态是已观察到的场景误差来源。

对官方示范每隔五步抽取一次点云，以颜色分割和拟合估计海绵与盘面相对位置。在擦拭阶段（帧索引 75），官方样本的海绵中心约在盘面上方 5.6 cm，镜像左手场景同一步的海绵中心仍偏向盘面下方。检查三组记录关节角时，镜像场景的左右手指尖和碰撞包围盒对应一致，说明手部几何对齐后仍需解决抓取姿态误差。

用帧索引 40 至 105 内每隔五帧的 14 个点云估计持物期间的海绵位置。仅改变固定约束的父链节，在相同回放动作下把海绵的平均 3D 位置误差从 54.9 mm 降至 25.5 mm。位置来自颜色分割后点云的中位数，与真实质心有偏差。

用帧索引 40 至 95 的 12 个点云估计盘缘中心和盘面法向，在相同回放动作中对照夹盘条件。等待两根手指形成相向接触后，抓取由第 26 步推迟到第 28 步，盘心平均位置误差从 70.2 mm 降至 54.8 mm，法向平均夹角从 38.9° 降至 32.3°。

在官方轨迹帧索引 65、75、80、85、90 上，海绵相对盘心沿盘面竖直方向的估计坐标约为 −119、−50、−120、−7、−125 mm。这个片段在盘心同侧有多次竖直转向。新仿真采用 60 mm 的单程位移阈值，仍要求正面载荷接触。

### 手指驱动对照

在 `61eb9c3` 的相同场景和动作上，对照每控制步直接更新手指关节状态与连续关节位置控制。位置控制的最大力矩和速度来自官方 LEAP URDF 的 `effort` 和 `velocity`，分别为 0.95 N·m 和 8.48 rad/s。掌部插值版本另外在每个控制步的十二个物理子步中插值掌部目标。夹盘、擦拭和成功判据相同。

| 仿真控制 | 盘心平均误差 | 法向平均误差 | 海绵平均位置误差 | 正面接触步 | 完整擦拭 |
|---|---:|---:|---:|---:|---:|
| 每控制步更新关节状态 | 54.8 mm | 32.3° | 27.2 mm | 0 | 0 |
| 关节位置控制，增益 0.10 | 74.3 mm | 35.0° | 67.4 mm | 42 | 0 |
| 关节位置控制，增益 0.25 | 75.5 mm | 33.3° | 69.8 mm | 5 | 0 |
| 关节位置控制，增益 0.25，掌部插值 | 52.8 mm | 42.9° | 80.6 mm | 5 | 0 |

位置控制的正面接触增加，持物位置跟踪误差也明显增加。Packaging 对照中四种控制均触发包装物抓取，均未入盒或合盖。正式仿真继续沿用表中第一种控制方式。

### 手部点云颜色对照

`8026189` 对齐了训练数据的手部 RGB 编码。Wiping 的第 40 帧，官方点云中有 4752 个零 RGB 点，仿真从 0 增至 4655 个。Packaging 的第 20 帧，官方点云中有 3280 个零 RGB 点，仿真从 0 增至 1014 个。这两个固定采样对照中，点的 XYZ 和所有非手部点的颜色完全一致。仿真从相机可见像素采样，官方数据另对重定向后的手部网格采样，因此手部点数量仍可能不同。

用同一个 Wiping 单示范 SetXF 第 5 轮权重，在 `pro1` 固定摆放和种子分别跑 120 步。原渲染颜色只触发海绵抓取，有 5 步正面接触。手部 RGB 置零后，盘子与海绵均触发抓取，有 3 步正面接触。两次均为 0 次完整擦拭。服务器记录为 `runs/wiping-{rendered,masked}-hand-color-ab-pro1.json`。

## 本地 Packaging 仿真

用固定盒体和被动铰链回放官方 `demo_0` 的 131 步，渲染 `10000 × 6` 点云。累计收到 52 个右手与包装物的接触点、1197 个左手与盒盖的接触点。包装物已释放并落在盒内，盒盖尚未闭合，最终判定为未成功。 [场景和指标](simulation.md)

同一段示范在当前柔性接触场景中将盒子向桌面中心推进约 0.0514 m，终点与起点相距 0.0653 m。包装物仍由右手抓持，盒盖打开。

官方示范约第 50–60 帧已清楚看到包装物在盒中。第 60 帧盒内白色包装标签的点云中位位置约为 `[0.418, -0.046, -0.001] m`。旧抓取条件下，模拟包装物在第 60 步仍被抓住，中心为 `[0.385, -0.224, 0.066] m`，纸盒最大倾角约 66°。单独约束纸盒保持水平也未使持物到达盒口。改用手指张开的释放条件修正了松手时序，剩余偏差集中在物体与盒口的相对位置。

从官方 ZIP 提取了 Packaging 首帧 `frame_2710` 的 RGB-D 和标定，换算后的相机矩阵与 HDF5 `demo_0` 首帧一致。原始深度拟合的桌面在盒子附近约为 −0.035 m，在包装物附近约为 −0.022 m。包装物的可见初始平面包络约为 14.0 × 5.6 cm，偏航约 64°，高度到约 8.8 cm。像素裁剪会影响边界估计。处理后点云中较大的抬高平面对应盒盖。试用实测台面和初始物体位姿后，包装物更接近盒口，但回放仍未入盒。该样本和标定位于 `pro1` 的 `data/raw/save_packaging_wild_1-20/save_data_packaging_01/`。 [数据来源](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0)
