# DexCap 论文复现

在 `pro1` 上复现 DexCap / DexIL 的点云策略训练，使用作者发布的 Wiping 和 Packaging 数据。公开源码位于 `DexCap/`，取自 [`j96w/DexCap@4b0bed0`](https://github.com/j96w/DexCap/tree/4b0bed0966c87368f3cde4476aadb7585c3b94b5)。原作者许可见 [DexCap/LICENSE](DexCap/LICENSE)。论文主结果用 Perceiver 和实体机器人。公开代码默认用 PointNet，适合作为第一组对照。 [DexCap 论文](https://arxiv.org/html/2403.07788v2#S5)

## 项目入口

所有运行从 `pro1:~/dexcap-repro` 进入。`configs/` 保存训练配置，`envs/train` 保存 Conda 环境，`data/official/<revision>/` 保存官方数据，`runs/` 保存模型和日志。环境、数据和训练产物保留在服务器上。 [复现操作说明](docs/reproduction.md)

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
