# 实验记录

## 源码基线

官方源码为 `j96w/DexCap@4b0bed0966c87368f3cde4476aadb7585c3b94b5`。`main` 的 `5ca7487` 修复了 HDF5 原始动作与归一化统计之间的精度不一致。测试在原版 `SequenceDataset` 上得到 `-1.00000207`，修复后通过。

## 本地硬件预验证

使用 Blackwell RTX 5070 Ti Laptop、Python 3.11、PyTorch `2.10.0+cu130`，从官方 Wiping HDF5 的第一条序列读取前 24 帧。PointNet 和 `SetXFPCDEncoder` 各跑 2 个 epoch、每个 3 步。两次都从 epoch 2 checkpoint 加载并输出有限的 46 维动作。这些运行用于验证训练与推理链路。

## pro1

项目位于 `~/dexcap-repro`。Conda 环境位于 `envs/train`，Python 3.11.18。Wiping 的完整数据和 PyTorch 依赖正在准备。正式结果将从实际训练日志填入。

## 官方原始数据重建

从官方 Wiping 原始 ZIP 提取第一段示范的标定、`frame_0` 和 `frame_2220`，以单帧夹具运行无界面的 PyBullet 指尖 IK 和 RGB-D 点云构建。输出一个 HDF5 时间步、46 维动作和 `1000 × 6` 点云。 [重建命令与验证范围](raw-retarget.md)

## 本地 Plate Wiping 仿真

| 输入 | 控制步 | 海绵与盘子接触步 | 相对盘面移动 | 完整擦拭 | 仿真判定 |
|---|---:|---:|---:|---:|---|
| 官方 `demo_0` 动作回放 | 111 | 9 | 0.0596 m | 0 | 未成功 |
| 6 梯度步的 PointNet 接口 checkpoint | 12 | 0 | 0 m | 0 | 未成功 |

两个回合均从 Bullet 场景重建 `10000 × 6` 的点云，并报告两件物体触发抓取。12 步闭环用于检查仿真观测、DDIM 推理和动作回写。 [场景和指标](simulation.md)
