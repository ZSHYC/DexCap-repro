# DexCap MuJoCo 迁移

将 Wiping 和 Packaging 的示范回放与策略评估迁到同一套双 Franka、双 LEAP 的 MuJoCo 环境。Packaging 保留六个训练物体、九个未见物体和五组配对摆放。论文还描述了 Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing。公开数据仓库提供 Wiping 和 Packaging 的示范，另外四项的官方轨迹与训练权重仍需要数据来源。 [论文任务](https://arxiv.org/html/2403.07788v2#A1.SS6) · [作者数据发布](https://huggingface.co/datasets/chenwangj/DexCap-Data)

## 模型与控制

用 Menagerie 的 Franka Panda 机械臂和本项目已有的 LEAP 关节定义构建完整机器人。仿真中的实际腕位和 32 个手指关节值转回发布数据的坐标及数值格式，沿用 46 维动作和三帧点云观测。用腕位目标驱动机械臂，在物理子步内执行力矩或受限力控制。手指通过关节目标和有限驱动力运动。任务物体通过碰撞、摩擦和重力移动。基座位置、法兰到掌座的变换以及物体材料参数需要对照公开标定和原始示范验证。

## 迁移顺序

机器人模型、两项任务的物体及判据、RGB-D 点云可以独立实现。每部分先验证具体失败，包括关节或资产无法装配、控制步直接改写物体状态、对接触面或盒内几何的误判，以及渲染深度与相机标定不一致。统一入口随后加载官方 HDF5 和现有权重，保留原有输出指标与摆放种子，记录引擎和模型配置。

先回放两项任务的官方动作，记录双腕跟踪误差、关节限位、碰撞、抓持和释放。逐项诊断影响任务完成的差异，再使用同样的最终权重和摆放做随机场景的闭环评估。正式结果需要覆盖两组 Wiping 各 20 回合、Packaging 15 个物体各 5 回合。场景运行、示范回放成功与策略达标分别以各自的实测结果为依据。

## 运行

`pro1` 的开发工作树是 `/home/zhangshihao/dexcap-repro/worktrees/mujoco/`。`data/` 指向 `/data/dexcap-repro/data/`，官方 HDF5、模型资产和安装包缓存都在数据盘。`runs/` 指向 `/data/dexcap-repro/runs/mujoco/`，保存 MuJoCo 日志、轨迹和结果。工作树复用 `/home/zhangshihao/dexcap-repro/envs/train/` 的 Conda 环境。

在 Conda 环境安装 `requirements-train.txt` 中的 MuJoCo 3.6.0。`scripts/fetch_mujoco_assets.sh` 把固定版本的 Panda 模型和许可证放在 `data/mujoco_menagerie/`。借助原项目的 `data/` 链接，这些资产和官方 HDF5 一起存储在 `pro1` 数据盘。

`pro1` 的 NVIDIA 驱动提供厂商 EGL 库，项目的通用 EGL 库来自 Ubuntu 24.04 的 `libegl1`、`libglvnd0` 和 `libopengl0`，已经解包到数据盘的 `data/mujoco-gl/`。在 `pro1` 打开新终端后使用以下设置。

```bash
cd ~/dexcap-repro/worktrees/mujoco
source envs/conda/etc/profile.d/conda.sh
conda activate "$PWD/envs/train"
export LD_LIBRARY_PATH="$PWD/data/mujoco-gl/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MUJOCO_EGL_DEVICE_ID=0 CUDA_VISIBLE_DEVICES=0
bash scripts/fetch_mujoco_assets.sh
MUJOCO_GL=egl python -m simulation.rollout data/derived/wiping-demo-0-playback.hdf5 \
  --output runs/wiping-replay.json
MUJOCO_GL=egl python -m simulation.rollout data/derived/packaging-demo-0-playback.hdf5 \
  --task packaging --output runs/packaging-replay.json
```

## 官方动作回放和策略接入

两段官方 `demo_0` 分别执行完 111 和 131 个目标动作，每个目标由 OSC 跟踪，至双腕进入 5 mm、5° 容差或达到论文规定的 10 次控制上限。50 ms 控制周期和首个周期内的 30% 腕位插值时长参考上游 Deoxys，容差是本项目选值。经修正的法兰朝向来自 LEAP 掌座几何，实际安装滚转角和垫高仍需确认。 [论文控制方案](https://arxiv.org/html/2403.07788v2#A1.SS4) · [Deoxys 插值配置](https://github.com/UT-Austin-RPL/deoxys_control/blob/main/deoxys/config/osc-pose-controller.yml) · [LEAP 安装件](https://v1.leaphand.com/leap_cad)

| 回放 | 右腕误差中位数 | 左腕误差中位数 | 实际结果 |
| --- | ---: | ---: | --- |
| Wiping | 4.5 mm | 14.9 mm | 盘子和海绵都触发抓取指标。371 个控制周期接触盘体，内盘面擦拭为 0。 |
| Packaging | 5.8 mm | 14.8 mm | 红银包装短时抬起后掉落。左手完成合盖，物体未入盒。 |

Packaging 红银包装使用原始深度的 14.0 × 5.6 × 8.8 cm 包络。盒盖按首帧深度以约 3.35° 开始，并有实测约 70 mm 的独立折舌。主铰链在模型中按弧度编译，可实际转至 180°。纸板厚度近似 3 mm，折舌刚度与阻尼未做实物标定。黄色折叠布的 150 × 130 × 40 mm 包络来自发布点云，以三维柔性网格模拟受力、变形和入盒支撑。其有效弹性参数仍为估计。几何测量记录在 `runs/packaging-lid-tab-measurement.json`。

两段回放在采样的控制周期均未出现机械臂碰撞接触，且能生成 `10000 × 6` 点云。当前泡棉材料、手指接触系数、纸盒铰链和物体表面参数仍需要用实测或录制行为标定。两段回放均未达成完整任务。

在 `pro1` 的同一个 Conda 环境验证了 EGL 渲染和三份现有第 3000 轮权重。PointNet Wiping、SetXF Wiping 和 SetXF Packaging 分别在 GPU 0、1、2 完成两步策略闭环，均输出 `10000 × 6` 点云。记录位于数据盘的 `runs/*-policy-smoke.json`。

逐步诊断位于数据盘。Wiping 的模型和录制手部表面中位距离约为 2.5–2.8 mm。抓取阶段有些目标姿态会把手指几何移入桌面约 3 cm。离桌后物体姿态也出现偏差，第 72 步盘面法向与录制估计相差约 59°。仿真海绵中心处在背面一侧约 53 mm。包装物抓取要求同时出现相向手指接触、质心抬升 1 cm 且不受桌面支撑。

- 渲染关键帧位于 `runs/dexcap-mujoco-replay-keyframes.png`。
- 回放指标位于 `runs/wiping-measured-sponge-demo0.json` 和 `runs/packaging-measured-hinge-demo0.json`。
- Packaging 的抓取、掉落和合盖最早见于第 21、26、123 步。包装物始终未接触盒底。每步的物体位置和盒口边界见 `runs/packaging-measured-hinge-contact-trace.json`。
- 手部几何、持盘姿态和相机条件诊断位于 `runs/wiping-geometry-alignment.json`、`runs/wiping-demo0-plate-facing.json` 和 `runs/wiping-action-diagnostics-mujoco.json`。

## 复用权重评估

沿用已训练的第 3000 轮权重。PointNet 使用当前分支的训练代码，SetXF 使用 `worktrees/setxf`。每次仿真最多执行 400 个策略动作。Wiping 使用相同的种子 1–20 比较两份权重。Packaging 对训练和未见物体各使用同一组种子 1–5。结果写入数据盘 `runs/formal-mujoco/`。

```bash
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=0 CUDA_VISIBLE_DEVICES=0
export PYTHONPATH="$PWD/DexCap/STEP3_train_policy"
release=data/official/e468f07000de0f8d3234a9c60a9d8208344675e0
dataset="$release/hand_wiping_1-14_5actiongap_10000points.hdf5"
checkpoint=../../runs/wiping-pointnet-seed1-full-20261119T102945Z/20261119182948/models/model_epoch_3000.pth
mkdir -p runs/formal-mujoco
python -m simulation.rollout "$dataset" --checkpoint "$checkpoint" --trials 20 \
  --output runs/formal-mujoco/wiping-pointnet.json
```

用 `--first-seed` 可以把多个摆放拆给独立进程。每块按固定种子生成，按种子合并后使用与单进程评估相同的配对。用 `MUJOCO_EGL_DEVICE_ID` 设置渲染卡，用 `CUDA_VISIBLE_DEVICES` 设置策略推理卡。 `pro1` 评估分配在 GPU 0–3。

历史 PyBullet 仿真代码和入口保存在基线提交。当前分支的 `simulation.rollout`、示范图和策略动作诊断均运行 MuJoCo。官方数据重建仍沿用作者的 PyBullet 指尖 IK。 [基线提交](https://github.com/ZSHYC/DexCap-repro/tree/0ee5418)


## 115 回合闭环结果

| 权重及任务 | 回合 | 抓取或入盒 | 内盘面接触或合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: |
| PointNet Wiping | 20 | 两件均抓取 1 | 内盘面接触 0 | 0 |
| SetXF Wiping | 20 | 两件均抓取 0 | 内盘面接触 0 | 0 |
| SetXF Packaging 训练物体 | 30 | 入盒 0 | 合盖 20 | 0 |
| SetXF Packaging 未见物体 | 45 | 入盒 0 | 合盖 22 | 0 |

Wiping PointNet 分别有 4 个回合抓起盘子、2 个回合抓起海绵。SetXF 分别为 5 和 0。Packaging 的 15 个物体都完成 5 个回合。所有结果均由 MuJoCo 3.6.0 生成。论文中的成功率来自实体机器人。 [逐物体汇总](assets/dexcap-formal-mujoco.json) · [论文评估](https://arxiv.org/html/2403.07788v2#S5.SS1)

在 `pro1` 上复算已保存的汇总。

```bash
folder=runs/formal-mujoco
python -m simulation.summarize_results "$folder/wiping-pointnet.json" \
  "$folder/wiping-setxf.json" "$folder/packaging-setxf-seed1-full-20261119T111945Z" \
  --output "$folder/summary.json"
```

同一 Wiping 示范的 15 个时刻和 3 个推理种子用于对照动作误差。下表是第一步动作的右腕、左腕位置误差中位数，单位 mm。每个条件只比较同一个时刻和种子的策略输出。录制、移动相机渲染和固定相机渲染都沿用录制的本体状态，完整仿真输入由仿真提供本体状态。

| 权重 | 录制输入 | MuJoCo 跟随录制相机 | MuJoCo 固定相机 | 完整仿真输入 |
| --- | ---: | ---: | ---: | ---: |
| PointNet | 3.3 / 3.9 | 7.8 / 11.5 | 7.5 / 11.4 | 11.8 / 15.0 |
| SetXF | 3.4 / 4.6 | 5.9 / 12.6 | 6.1 / 12.1 | 6.8 / 11.9 |

使用录制相机位姿时，两种权重仍有明显左腕误差。当前差异也来自接触后物体姿态和观测的变化。诊断在数据盘 `runs/formal-mujoco/wiping-*-action-diagnostics.json`。场景仍使用估计的泡棉、织物和铰链材料参数，以及依据录制估计的物体外形和安装位置。
