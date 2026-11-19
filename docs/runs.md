# 实验记录

## 源码基线

官方源码为 `j96w/DexCap@4b0bed0966c87368f3cde4476aadb7585c3b94b5`。`main` 的 `5ca7487` 修复了 HDF5 原始动作与归一化统计之间的精度不一致。测试在原版 `SequenceDataset` 上得到 `-1.00000207`，修复后通过。

## 本地硬件预验证

使用 Blackwell RTX 5070 Ti Laptop、Python 3.11、PyTorch `2.10.0+cu130`，从官方 Wiping HDF5 的第一条序列读取前 24 帧。PointNet 和 `SetXFPCDEncoder` 各跑 2 个 epoch、每个 3 步。两次都从 epoch 2 checkpoint 加载并输出有限的 46 维动作。这些运行用于验证训练与推理链路。

## pro1

项目位于 `~/dexcap-repro`。Conda 环境位于 `envs/train`，Python 3.11.18，PyTorch `2.10.0+cu130`，GPU 为 RTX PRO 5000 72 GB Blackwell。CUDA 张量运算已成功执行。

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

完整 Wiping 和 Packaging HDF5 正在续传。完整数据到位后，`tmux` 会话 `dexcap-experiments` 下的 Wiping PointNet、Wiping SetXF 和 Packaging SetXF 作业各先执行完整文件短跑，成功后继续对应长训。`dexcap-full-sim` 等待每个成功完成的长训保存 epoch 3000 的 checkpoint。两种 Wiping 策略各执行 20 组配对摆放，Packaging 执行一组，每回合 400 步，结果写入对应的 `runs/*-final-sim.json`。 [服务器项目操作路径](reproduction.md)

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

## 本地 Packaging 仿真

用固定盒体和被动铰链回放官方 `demo_0` 的 131 步，渲染 `10000 × 6` 点云。累计收到 52 个右手与包装物的接触点、1197 个左手与盒盖的接触点。包装物已释放并落在盒内，盒盖尚未闭合，最终判定为未成功。 [场景和指标](simulation.md)

同一段示范在当前柔性接触场景中将盒子向桌面中心推进约 0.0514 m，终点与起点相距 0.0653 m。包装物仍由右手抓持，盒盖打开。
