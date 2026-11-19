# DexCap 复现指南

本文记录在 `pro1` 上使用公开数据训练 DexCap 策略并在 PyBullet 中评估的工作路径。Wiping 比较 PointNet 和注意力编码器，Packaging 评估同一个策略在六种训练物体和九种未见物体上的表现。 [仿真入口](simulation.md)

## 复现范围

官方数据发布的固定版本提供 Wiping 和 Packaging 的处理后 HDF5 以及对应原始录制。论文另外报告了 Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing，该版本缺少这四项的示范。当前可直接用发布数据训练并回放的仿真覆盖前两项。 [官方数据文件清单](https://huggingface.co/api/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0?recursive=true) · [论文任务](https://arxiv.org/html/2403.07788v2#S5)

| 层级 | 目标 | 完成依据 |
| --- | --- | --- |
| 环境 | Conda 环境能调用 Blackwell GPU，并导入仓库内的 robomimic | CUDA 张量运算成功，导入路径位于本项目 |
| 训练闭环 | 官方 Wiping 和 Packaging HDF5 能完成短训、保存并加载 checkpoint | 日志明确完成，checkpoint 对三帧真实观测输出有限的 46 维动作 |
| 算法对照 | PointNet 和 `SetXFPCDEncoder` 使用相同数据与训练参数 | 两组实验均保留生效配置、日志和 checkpoint |
| 仿真 | 以官方示范设定初始场景，在 PyBullet 回放或执行策略 | 模拟器产生点云和手部状态，记录 Wiping 抓取与擦拭、Packaging 入盒与合盖 |

服务器训练成功只能证明数据加载、优化和模型推理链路可用。论文报告的任务成功率来自实体机器人试验，不能用训练 loss 或离线动作预测代替。

## 项目目录

远程服务器统一使用以下入口：

```text
~/dexcap-repro/
├── DexCap/              # 跟踪的上游源码快照
├── configs/             # 本项目维护的实验配置
├── docs/                # 复现记录
├── scripts/             # 训练、推理和仿真入口
├── envs/
│   └── train/           # Python 3.11 Conda 训练环境
├── data/
│   ├── official/        # 官方处理后数据
│   ├── raw/             # 原始录制，后续阶段使用
│   └── derived/         # 自行重建的数据
└── runs/                # 日志、TensorBoard 和 checkpoint
```

在每个新终端中设置项目路径：

```bash
export DEXCAP="$HOME/dexcap-repro"
export DATA_REV="e468f07000de0f8d3234a9c60a9d8208344675e0"
```

源码基线定位在上游提交 `4b0bed0966c87368f3cde4476aadb7585c3b94b5`。实验改动提交到私有仓库 `ZSHYC/DexCap-repro`，数据、Conda 环境和训练产物不进入 Git。每次实验的里程碑和证据摘要记录在 `docs/runs.md`；该文件记录结论，不收纳大型原始日志或 checkpoint。

## 运行一次完整实验

安装下文的 Conda 环境、下载对应 HDF5，并为 SetXF 建立 `worktrees/setxf` 后，可从项目根目录启动单项实验。三个命令分别在各自的终端或 `tmux` 窗口运行。

```bash
cd "$DEXCAP"
scripts/run_experiment.sh wiping-pointnet full
scripts/run_experiment.sh wiping-setxf full
scripts/run_experiment.sh packaging-setxf full
```

三个命令默认分别使用 GPU 0、1、2。换卡时给各命令设置一个 `CUDA_VISIBLE_DEVICES` 编号，训练器将该卡映射为 `cuda:0`。每个命令先短训并加载第 2 轮权重，随后按完整配置训练，加载第 3000 轮权重做仿真。两种 Wiping 策略各跑同一组 20 个摆放，Packaging 的六种物体各跑 5 回合。每组的有效配置、日志、权重和仿真 JSON 都在 `runs/`。

`pro1` 上的 `dexcap-packaging-unseen` 会话等待 Packaging 的第 3000 轮权重。训练完成后，它用 GPU 3–6 对九种未见物体各跑 5 回合。评估可用同一个 checkpoint 手动重跑。 [未见物体及运行命令](simulation.md#未见物体)

只验证训练与离线推理时使用 `smoke`。第三个参数可以指定已提取的 HDF5，用于在完整数据下载期间验证入口。

```bash
scripts/run_experiment.sh wiping-pointnet smoke data/derived/wiping-official-demo-0.hdf5
```

`pro1` 已有实验由 `dexcap-experiments` 和 `dexcap-full-sim` 两个后台会话负责。当前作业和中间结果记录在实验日志中。 [正在运行的实验](runs.md)

## 1. 建立 Conda 训练环境

`pro1` 上 Conda 安装在 `envs/conda`，Python 3.11 环境在 `envs/train`：

```bash
source "$DEXCAP/envs/conda/etc/profile.d/conda.sh"
conda activate "$DEXCAP/envs/train"
```

在新服务器上先安装 Conda，再用 `conda create --override-channels -c conda-forge --prefix "$DEXCAP/envs/train" python=3.11 pip` 创建环境。激活后安装支持 Blackwell 的 PyTorch 和 DexCap 依赖：

```bash
python -m pip install \
  torch==2.10.0 torchvision==0.25.0 \
  --index-url https://download.pytorch.org/whl/cu130

python -m pip install -r "$DEXCAP/requirements-train.txt"
```

仓库自带 robomimic fork，不另装 PyPI 版本：

```bash
export PYTHONPATH="$DEXCAP/DexCap/STEP3_train_policy"
```

检查 CUDA 和导入来源。失败时分别处理 PyTorch 安装或 `PYTHONPATH`，通过后无需重复检查：

```bash
python - <<'PY'
import torch
import robomimic

assert torch.cuda.is_available()
x = torch.ones((32, 32), device="cuda")
assert (x @ x)[0, 0].item() == 32
assert "/DexCap/STEP3_train_policy/robomimic/" in robomimic.__file__

print("torch:", torch.__version__)
print("CUDA runtime:", torch.version.cuda)
print("GPU:", torch.cuda.get_device_name(0))
print("robomimic:", robomimic.__file__)
PY
```

## 2. 下载官方 Wiping 数据

`pro1` 当前不能稳定直连 Hugging Face。已验证的路径是让服务器通过本机 SSH 连接访问数据。

本机用发布方的 Xet 客户端试拉过 Packaging。55 秒内只生成了约 0.13 MB 的会话文件，且本机试拉位置剩余约 12.5 GB。当前 29.1 GB 文件通过下文的 SSH 路径直接续传到 `pro1`。 [发布方的下载方式](https://huggingface.co/docs/hub/datasets-downloading)

在本机终端保持以下连接：

```bash
ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -R 127.0.0.1:18787 \
  pro1
```

在 `pro1` 的另一个终端下载。中断后重新运行同一命令会续传：

```bash
export DATA_DIR="$DEXCAP/data/official/$DATA_REV"
export DATASET="$DATA_DIR/hand_wiping_1-14_5actiongap_10000points.hdf5"
mkdir -p "$DATA_DIR"

curl --fail --location --show-error \
  --proxy socks5h://127.0.0.1:18787 \
  --connect-timeout 15 --retry 5 --continue-at - \
  --output "$DATASET.part" \
  "https://huggingface.co/datasets/chenwangj/DexCap-Data/resolve/$DATA_REV/hand_wiping_1-14_5actiongap_10000points.hdf5" \
  && mv "$DATASET.part" "$DATASET"
```

下载成功后进入短训。若短训在加载数据时失败，用下面的命令区分 HDF5 文件问题、版本不符和训练配置问题：

```bash
python - "$DATASET" <<'PY'
import sys
import h5py
import numpy as np

with h5py.File(sys.argv[1], "r") as dataset:
    data = dataset["data"]
    demo = data["demo_0"]
    length = int(demo.attrs["num_samples"])

    assert demo["actions"].shape == (length, 46)
    assert demo["obs/pointcloud"].shape[1:] == (10000, 6)
    assert np.isfinite(demo["actions"][:]).all()
    assert np.isfinite(demo["obs/pointcloud"][:3]).all()

    print("demos:", len(data))
    print("steps:", int(data.attrs["total"]))
    print("demo_0 actions:", demo["actions"].shape)
    print("demo_0 pointcloud:", demo["obs/pointcloud"].shape)
PY
```

当前官方文件应显示 255 条 HDF5 序列和 27,901 个时间步。每个点为 XYZRGB 六维，动作由双腕位置、双腕四元数和双手关节位置拼成 46 维。

## 3. 短训验证训练闭环

从训练目录运行命令。`--dataset` 和 `--output` 会覆盖配置中的占位路径：

```bash
cd "$DEXCAP/DexCap/STEP3_train_policy/robomimic"
export PYTHONPATH="$DEXCAP/DexCap/STEP3_train_policy"
export DATASET="$DEXCAP/data/official/$DATA_REV/hand_wiping_1-14_5actiongap_10000points.hdf5"

run_name="wiping-pointnet-seed1-smoke-$(date +%Y%m%d-%H%M%S)"
set -o pipefail
CUDA_VISIBLE_DEVICES=0 python -u scripts/train.py \
  --config "$DEXCAP/configs/pointcloud-smoke.json" \
  --dataset "$DATASET" \
  --name "$run_name" \
  --output "$DEXCAP/runs" \
  2>&1 | tee "$DEXCAP/runs/$run_name.console.log"
```

训练入口会捕获 Python 异常并打印 `run failed with error`，因此仅看进程退出码不够。日志必须出现：

```bash
grep -nE "finished run successfully|run failed with error|save checkpoint|Loss" \
  "$DEXCAP/runs/$run_name.console.log"
```

出现 `run failed with error` 时按堆栈修正具体问题。成功时应保存两轮 loss 和 `model_epoch_2.pth`。

公开数据的动作是双精度。原始数据加载器曾在计算 min-max 统计时把动作先转成单精度，可能造成归一化值略超出 `[-1, 1]`。本项目在共享数据加载逻辑中保留双精度统计，避免第一批训练随机触发动作范围检查。

## 4. 加载 checkpoint 做离线推理

用同一实验的第二轮 checkpoint 和官方 `demo_0` 前三帧观测运行策略：

```bash
ckpt="$(find "$DEXCAP/runs/$run_name" -type f -name model_epoch_2.pth -print -quit)"
test -n "$ckpt"

CUDA_VISIBLE_DEVICES=0 python "$DEXCAP/scripts/checkpoint_inference.py" "$ckpt" "$DATASET"
```

这个检查能发现 checkpoint 无法反序列化、输入字段不匹配或推理产生无效数值。通过后，训练闭环完成，可以启动完整训练。

## 5. 完整 PointNet 训练

```bash
cd "$DEXCAP/DexCap/STEP3_train_policy/robomimic"
run_name="wiping-pointnet-seed1-full-$(date +%Y%m%d-%H%M%S)"

CUDA_VISIBLE_DEVICES=0 python -u scripts/train.py \
  --config "$DEXCAP/configs/pointcloud-full.json" \
  --dataset "$DATASET" \
  --name "$run_name" \
  --output "$DEXCAP/runs" \
  2>&1 | tee "$DEXCAP/runs/$run_name.console.log"
```

官方训练参数为 batch size 16、学习率 `1e-4`、3000 epochs，每个 epoch 500 个梯度步。配置使用三帧观测、20 步动作预测和 DDIM。点云文件每帧保存 10,000 点，编码器前向时随机抽取 1,000 点。

每次训练用一张 GPU，多个实验通过不同的 `CUDA_VISIBLE_DEVICES` 并行。在 `pro1` 的全量数据对照中，增加数据读取 worker 能缩短单个实验的数据等待。新启动的全量训练配置使用 8 个 worker。 [吞吐实测](runs.md#数据读取吞吐)

训练输出位于：

```text
runs/<实验名>/<启动时间>/
├── config.json
├── logs/log.txt
├── models/
└── videos/
```

`config.json` 是该次运行真正生效的配置。分析结果时同时记录源码提交、数据 revision、实验名、GPU 和 checkpoint epoch。

## 6. PointNet 与论文主模型对照

公开训练代码在观测键包含 `pointcloud` 时强制选用 `PointNet`。论文表格中的主模型使用 Perceiver 类点云编码器。仓库提供 `SetXFPCDEncoder`，但默认分支把它注释掉。

本项目的 `main` 分支用 PointNet，`repro/setxf` 分支用注意力编码器。两条分支共用 `configs/pointcloud-smoke.json` 和 `configs/pointcloud-full.json`，通过实验名标识任务与编码器。需要同时运行时，在项目内建立独立工作树：

```bash
cd "$DEXCAP"
git worktree add worktrees/setxf repro/setxf
export PYTHONPATH="$DEXCAP/worktrees/setxf/DexCap/STEP3_train_policy"
cd "$DEXCAP/worktrees/setxf/DexCap/STEP3_train_policy/robomimic"
```

若分支尚未创建，先在当前 PointNet 基线提交上创建 `repro/setxf`，再添加 worktree。对照时保持以下变量一致：

- 同一份 Wiping HDF5；
- 同一随机种子、batch size、学习率和训练步数；
- 相同观测键、动作归一化和扩散参数；
- 每个实验使用独立名称与输出目录。

在 `repro/setxf` 上用共享的 `pointcloud-smoke.json` 走完短训和 checkpoint 推理，再用 `pointcloud-full.json` 启动完整训练。命名采用 `wiping-setxf-...`，PointNet 则采用 `wiping-pointnet-...`。如果注意力版本只完成服务器训练，应将结论写成“公开 `SetXFPCDEncoder` 的训练复现”，不能直接写成“复现论文 70% 真机成功率”。

## 7. 原始数据重建

官方处理后 HDF5 足够复现训练链路。重建数据还需要原始 ZIP、桌面坐标对齐、手部标定和人工裁剪标记。

旧数据流程为：

1. `STEP1_collect_data/replay_human_traj_vis.py` 回放 RGB-D 和手部轨迹；
2. 必要时用 `calculate_offset_vis_calib.py` 修正 SLAM 初始漂移；
3. 用 `transform_to_robot_table.py` 对齐机器人桌面坐标；
4. 用 `demo_clipping_3d.py` 标记任务片段；
5. 在 `STEP2_build_dataset` 中运行 `demo_create_hdf5.py`。

数据构建会执行 PyBullet 指尖 IK、RGB-D 点云重建、桌面与背景过滤，并把 LEAP 手模型点云加入观测。项目的构建入口支持 `--headless`，已用官方 Wiping 原始帧验证。 [无界面重建命令](raw-retarget.md)

2024 年新版采集代码使用 VIVE Ultimate Tracker 和 D435，保存的位姿文件和旧版处理脚本读取的 T265/L515 格式不同。新版采集数据接入旧构建流程需要明确的坐标变换和标定转换，当前仓库没有提供完整转换入口。

## 8. 仿真

通过 `python -m simulation.rollout` 回放官方第一个示范，或加载训练 checkpoint，让策略接收仿真点云和手部状态。Wiping 记录接触和擦拭，Packaging 记录物体入盒和盖板闭合。 [命令、场景近似与指标](simulation.md)

## 公开材料带来的比较限制

- 公开数据只包含 Wiping 和 Packaging 的原始及处理后数据，不能覆盖论文全部六个任务。
- 数据页没有发布论文所有基线、人工纠错数据或作者训练好的 checkpoint。
- 公开配置走 PointNet，而论文最佳结果来自 Perceiver 类编码器。
- 论文描述了将点云与动作轨迹同步做二维平移。三组正式实验沿用发布入口的默认行为。另开 Packaging 对照，在完整数据和同一种子下比较每轴最多 10 cm 的同步平移。幅度取值源于本轮实验。 [论文方法](https://arxiv.org/html/2403.07788v2#S4.SS2) · [平移实验](runs.md#同步二维平移对照)
- 训练脚本中的环境构建与 rollout 主体被注释，训练日志不能直接产生论文任务成功率。
- HDF5 中一个人工 clip 会按 `action_gap=5` 生成五条错相序列，因此 `demo_*` 数量不等于独立人工示范数量。

实验记录区分训练闭环、编码器对照、原始数据重建和仿真结果。

## Git 提交节奏

每个能独立说明和复查的结果单独提交并推送：

1. 项目结构、Conda 环境说明和基础配置；
2. 动作归一化修正及短训证据；
3. PointNet 完整训练配置和结果记录；
4. `SetXFPCDEncoder` 切换、短训和对照结果；
5. 原始数据重建和仿真各自的实验结果。

训练数据、Conda 环境和大型 checkpoint 留在服务器。仓库提交源码、配置、命令、关键日志摘要和实验结论。

截至本指南建立时，`pro1` 上的首轮任务已经启动，但尚无完成的正式训练可报告。只有日志、checkpoint 和离线推理三项证据齐备后，才在 `docs/runs.md` 将对应训练里程碑标记为完成。
