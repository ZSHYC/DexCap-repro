# 实验记录

## 源码基线

官方源码为 `j96w/DexCap@4b0bed0966c87368f3cde4476aadb7585c3b94b5`。`main` 的 `5ca7487` 修复了 HDF5 原始动作与归一化统计之间的精度不一致。测试在原版 `SequenceDataset` 上得到 `-1.00000207`，修复后通过。

## 本地硬件预验证

使用 Blackwell RTX 5070 Ti Laptop、Python 3.11、PyTorch `2.10.0+cu130`，从官方 Wiping HDF5 的第一条序列读取前 24 帧。PointNet 和 `SetXFPCDEncoder` 各跑 2 个 epoch、每个 3 步。两次都从 epoch 2 checkpoint 加载并输出有限的 46 维动作。这些运行用于验证训练与推理链路。

## pro1

项目位于 `~/dexcap-repro`。Conda 环境位于 `envs/train`，Python 3.11.18。Wiping 的完整数据和 PyTorch 依赖正在准备。正式结果将从实际训练日志填入。
