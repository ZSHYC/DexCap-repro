# DexCap 论文复现

`repro/mujoco` 分支的 `simulation.rollout` 使用双 Franka、双 LEAP，提供 Wiping、Packaging、Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing 的 MuJoCo 场景。Wiping 和 Packaging 使用已发布数据训练的权重完成了 115 回合闭环评估。Ball Collecting 和 Sponge Picking 的默认摆放均已通过标准入口完成脚本回放。 [MuJoCo 运行方式与当前验证](docs/mujoco-migration.md)

在 `pro1` 上复现 DexCap / DexIL 的点云策略训练，使用作者发布的 Wiping 和 Packaging 数据。当前阶段聚焦仿真，支持两项公开任务的无界面轨迹回放和策略闭环。公开源码位于 `DexCap/`，取自 [`j96w/DexCap@4b0bed0`](https://github.com/j96w/DexCap/tree/4b0bed0966c87368f3cde4476aadb7585c3b94b5)。原作者许可见 [DexCap/LICENSE](DexCap/LICENSE)。 [DexCap 论文](https://arxiv.org/html/2403.07788v2#S5) · [运行 MuJoCo](docs/mujoco-migration.md) · [与论文表格的对应关系](docs/reproduction.md#与论文表格的对应关系)

## 项目入口

所有运行从 `pro1:~/dexcap-repro` 进入。`configs/` 保存训练配置，`envs/train` 是 Conda 环境。`data/` 和 `runs/` 分别链接到 `/data/dexcap-repro/data/` 和 `/data/dexcap-repro/runs/`，后续下载、预处理和训练输出通过这些入口写入数据盘。 [复现操作说明](docs/reproduction.md)

## MuJoCo 仿真结果

在 `pro1` 上用校正后的 LEAP 实体网格、原有第 3000 轮权重和与基线配对的摆放种子，完成每回合 400 步的评估。表中 Wiping 数据来自本次海绵接触求解更新前的评估。

| 实验 | 分项结果 | 完整成功 |
| --- | --- | ---: |
| Wiping PointNet | 两件均抓取 1/20，内盘面接触 1/20 | 0/20 |
| Wiping SetXF | 两件均抓取 1/20，内盘面接触 1/20 | 0/20 |
| Packaging 训练物体 | 入盒 0/30，合盖 7/30 | 0/30 |
| Packaging 未见物体 | 入盒 0/45，合盖 6/45 | 0/45 |

[MuJoCo 汇总 JSON](docs/assets/dexcap-formal-mujoco.json) · [回放、动作诊断与服务器路径](docs/mujoco-migration.md)

## PyBullet 基线

三组训练各完成 3000 轮，每轮 500 个梯度步。正式评估使用第 3000 轮权重，合计 115 个回合，每回合 400 步。

| 实验 | 分项结果 | 完整成功 |
| --- | --- | ---: |
| Wiping PointNet | 两件均抓取 6/20，内盘面接触 0/20 | 0/20 |
| Wiping SetXF | 两件均抓取 10/20，内盘面接触 1/20 | 0/20 |
| Packaging 训练物体 | 入盒 0/30，合盖 9/30 | 0/30 |
| Packaging 未见物体 | 入盒 0/45，合盖 9/45 | 0/45 |

上表记录 PyBullet 策略闭环的结果。论文表格中的百分比来自真实机器人。 [正式汇总 JSON](docs/assets/dexcap-formal-simulation.json) · [实验记录](docs/runs.md) · [评估口径](docs/reproduction.md#与论文表格的对应关系)

## 训练环境

`pro1` 的 Conda 安装位于 `envs/conda`。从新的 shell 进入项目时运行：

```bash
export DEXCAP="$HOME/dexcap-repro"
cd "$DEXCAP"
source "$DEXCAP/envs/conda/etc/profile.d/conda.sh"
conda activate "$DEXCAP/envs/train"
export PYTHONPATH="$DEXCAP/DexCap/STEP3_train_policy"
```

在新服务器上首次创建时使用 `conda create --override-channels -c conda-forge --prefix "$DEXCAP/envs/train" python=3.11 pip`。激活后安装训练依赖：

```bash
python -m pip install --index-url https://download.pytorch.org/whl/cu130 \
  torch==2.10.0 torchvision==0.25.0
python -m pip install -r requirements-train.txt
```

## Wiping 短跑

下载 [官方 Wiping HDF5](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0) 后执行：

```bash
revision=e468f07000de0f8d3234a9c60a9d8208344675e0
data="$DEXCAP/data/official/$revision/hand_wiping_1-14_5actiongap_10000points.hdf5"
cd "$DEXCAP/DexCap/STEP3_train_policy/robomimic"
CUDA_VISIBLE_DEVICES=0 python -u scripts/train.py \
  --config "$DEXCAP/configs/pointcloud-smoke.json" \
  --dataset "$data" --output "$DEXCAP/runs" \
  --name "wiping-pointnet-seed1-smoke-$(date -u +%Y%m%dT%H%M%SZ)"
```

短跑需要完成 2 个 epoch，保存 `model_epoch_2.pth`。`configs/pointcloud-full.json` 对应公开配置的 3000 × 500 梯度步，可通过 `--dataset` 用于 Wiping 或 Packaging。 [复现操作说明](docs/reproduction.md) · [实验记录](docs/runs.md)
