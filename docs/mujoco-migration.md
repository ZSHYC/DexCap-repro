# DexCap MuJoCo 迁移

将 Wiping 和 Packaging 的示范回放与策略评估迁到同一套双 Franka、双 LEAP 的 MuJoCo 环境。Packaging 保留六个训练物体、九个未见物体和五组配对摆放。论文还描述了 Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing。公开数据仓库提供 Wiping 和 Packaging 的示范，另外四项的官方轨迹与训练权重仍需要数据来源。 [论文任务](https://arxiv.org/html/2403.07788v2#A1.SS6) · [作者数据发布](https://huggingface.co/datasets/chenwangj/DexCap-Data)

## 模型与控制

用 Menagerie 的 Franka Panda 机械臂和本项目已有的 LEAP 关节定义构建完整机器人。仿真中的实际腕位和 32 个手指关节值转回发布数据的坐标及数值格式，沿用 46 维动作和三帧点云观测。用腕位目标驱动机械臂，在物理子步内执行力矩或受限力控制。手指通过关节目标和有限驱动力运动。任务物体通过碰撞、摩擦和重力移动。基座位置、法兰到掌座的变换以及物体材料参数需要对照公开标定和原始示范验证。

## 迁移顺序

机器人模型、两项任务的物体及判据、RGB-D 点云可以独立实现。每部分先验证具体失败，包括关节或资产无法装配、控制步直接改写物体状态、对接触面或盒内几何的误判，以及渲染深度与相机标定不一致。统一入口随后加载官方 HDF5 和现有权重，保留原有输出指标与摆放种子，记录引擎和模型配置。

先回放两项任务的官方动作，记录双腕跟踪误差、关节限位、碰撞、抓持和释放。逐项诊断影响任务完成的差异，再使用同样的最终权重和摆放做随机场景的闭环评估。正式结果需要覆盖两组 Wiping 各 20 回合、Packaging 15 个物体各 5 回合。场景运行、示范回放成功与策略达标分别以各自的实测结果为依据。

## 运行

`pro1` 的开发工作树是 `/home/zhangshihao/dexcap-repro/worktrees/mujoco/`。`data/` 指向 `/data/dexcap-repro/data/`，官方 HDF5、模型资产和安装包缓存都在数据盘。`runs/` 指向 `/data/dexcap-repro/runs/mujoco/`，保存 MuJoCo 日志、轨迹和结果。工作树复用系统盘 `/home/zhangshihao/dexcap-repro/envs/train/` 的 Conda 环境。旧训练权重在数据盘项目的 `runs/`，归档权重在 `auxiliary-checkpoints/`。新下载、重建数据、训练输出和 MuJoCo 试验记录均通过工作树的 `data/` 或 `runs/` 写入数据盘。

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
| Wiping | 3.3 mm | 4.7 mm | 抓起海绵，盘子未抓起。53 个控制周期接触盘体，内盘面擦拭为 0。 |
| Packaging | 4.3 mm | 4.2 mm | 红银包装未抓起或入盒，纸盒未合盖。 |

Packaging 红银包装使用原始深度的 14.0 × 5.6 × 8.8 cm 包络。盒盖按首帧深度以约 3.35° 开始，并有实测约 70 mm 的独立折舌。主铰链在模型中按弧度编译，可实际转至 180°。纸板厚度近似 3 mm，折舌刚度与阻尼未做实物标定。黄色折叠布的 150 × 130 × 40 mm 包络来自发布点云，以三维柔性网格模拟受力、变形和入盒支撑。其有效弹性参数仍为估计。几何测量记录在 `runs/packaging-lid-tab-measurement.json`。

两段回放在采样的控制周期均未出现机械臂碰撞接触，且能生成 `10000 × 6` 点云。泡棉材料、手指接触系数、纸盒铰链和物体表面参数仍需要进一步标定。两段回放均未达成完整任务。

在 `pro1` 的同一个 Conda 环境验证了 EGL 渲染和三份现有第 3000 轮权重。PointNet Wiping、SetXF Wiping 和 SetXF Packaging 分别在 GPU 0、1、2 完成两步策略闭环，均输出 `10000 × 6` 点云。记录位于数据盘的 `runs/*-policy-smoke.json`。

早期手模型的逐步诊断位于数据盘。该版本 Wiping 的模型和录制手部表面中位距离约为 2.5–2.8 mm。抓取阶段有些目标姿态会把手指几何移入桌面约 3 cm。离桌后物体姿态也出现偏差，第 72 步盘面法向与录制估计相差约 59°。仿真海绵中心处在背面一侧约 53 mm。包装物抓取要求同时出现相向手指接触、质心抬升 1 cm 且不受桌面支撑。

- 校正前的渲染关键帧位于 `runs/dexcap-mujoco-replay-keyframes.png`。
- 回放指标位于 `runs/wiping-measured-sponge-demo0.json` 和 `runs/packaging-measured-hinge-demo0.json`。
- Packaging 的抓取、掉落和合盖最早见于第 21、26、123 步。包装物始终未接触盒底。每步的物体位置和盒口边界见 `runs/packaging-measured-hinge-contact-trace.json`。
- 手部几何、持盘姿态和相机条件诊断位于 `runs/wiping-geometry-alignment.json`、`runs/wiping-demo0-plate-facing.json` 和 `runs/wiping-action-diagnostics-mujoco.json`。

## 复用权重评估

沿用已训练的第 3000 轮权重。PointNet 使用当前分支的训练代码，SetXF 使用 `worktrees/setxf`。每次仿真最多执行 400 个策略动作。Wiping 使用相同的种子 1–20 比较两份权重。Packaging 对训练和未见物体各使用同一组种子 1–5。较早的配对基准位于数据盘 `runs/leap-physical-benchmark/`。更新海绵接触求解后的 40 个 Wiping 回合位于 `runs/sponge-contact-benchmark/`。进一步按双手实际夹持计分的 Wiping 结果写入 `runs/wiping-held-stroke-benchmark/`，更新接触求解后的 75 个 Packaging 回合写入 `runs/packaging-contact-benchmark/`，这批进程启动时仍使用原来的入盒边界判据。

```bash
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=0 CUDA_VISIBLE_DEVICES=0
export PYTHONPATH="$PWD/DexCap/STEP3_train_policy"
release=data/official/e468f07000de0f8d3234a9c60a9d8208344675e0
dataset="$release/hand_wiping_1-14_5actiongap_10000points.hdf5"
checkpoint=../../runs/wiping-pointnet-seed1-full-20261119T102945Z/20261119182948/models/model_epoch_3000.pth
mkdir -p runs/replay-evaluation
python -m simulation.rollout "$dataset" --checkpoint "$checkpoint" --trials 20 \
  --output runs/replay-evaluation/wiping-pointnet.json
```

用 `--first-seed` 可以把多个摆放拆给独立进程。每块按固定种子生成，按种子合并后使用与单进程评估相同的配对。用 `MUJOCO_EGL_DEVICE_ID` 设置渲染卡，用 `CUDA_VISIBLE_DEVICES` 设置策略推理卡。 已完成的配对评估使用 GPU 0–3。

历史 PyBullet 仿真代码和入口保存在基线提交。当前分支的 `simulation.rollout`、示范图和策略动作诊断均运行 MuJoCo。官方数据重建仍沿用作者的 PyBullet 指尖 IK。 [基线提交](https://github.com/ZSHYC/DexCap-repro/tree/0ee5418)


## 115 回合闭环结果

| 权重及任务 | 回合 | 抓取或入盒 | 内盘面接触或合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: |
| PointNet Wiping | 20 | 两件都曾抓起 1 | 有效内盘面接触 0 | 0 |
| SetXF Wiping | 20 | 两件都曾抓起 0 | 有效内盘面接触 0 | 0 |
| SetXF Packaging 训练物体 | 30 | 入盒 0 | 合盖 4 | 0 |
| SetXF Packaging 未见物体 | 45 | 入盒 0 | 合盖 7 | 0 |

115 个回合均使用 MuJoCo 3.6.0 和校正后的 LEAP 实体网格。逐回合核对确认数据、权重、步数和初始摆放与 `runs/leap-physical-benchmark/` 中的结果一致。Wiping PointNet 有 2 个回合抓起盘子、2 个回合抓起海绵，SetXF 分别为 0 和 3。有效内盘面接触要求同一物理子步两手都实际持物。Packaging 的接触参数已更新，75 个回合按后续加入 0.1 mm 入盒边界容差之前的条件计分。论文中的成功率来自实体机器人。 [逐物体汇总](assets/dexcap-formal-mujoco.json) · [论文评估](https://arxiv.org/html/2403.07788v2#S5.SS1)

在 `pro1` 上复算已保存的汇总。

```bash
wiping=runs/wiping-held-stroke-benchmark
packaging=runs/packaging-contact-benchmark/packaging-setxf-seed1-full-20261119T111945Z
python -m simulation.summarize_results "$wiping/wiping-pointnet.json" \
  "$wiping/wiping-setxf.json" "$packaging" --output "$wiping/summary.json"
```

同一 Wiping 示范的 15 个时刻和 3 个推理种子用于对照动作误差。下表是第一步动作的右腕、左腕位置误差中位数，单位 mm。每个条件只比较同一个时刻和种子的策略输出。录制、移动相机渲染和固定相机渲染都沿用录制的本体状态，完整仿真输入由仿真提供本体状态。

| 权重 | 录制输入 | MuJoCo 跟随录制相机 | MuJoCo 固定相机 | 完整仿真输入 |
| --- | ---: | ---: | ---: | ---: |
| PointNet | 3.3 / 3.9 | 7.4 / 10.9 | 7.4 / 10.8 | 10.8 / 15.8 |
| SetXF | 3.4 / 4.6 | 5.8 / 13.5 | 5.9 / 13.8 | 6.6 / 12.3 |

使用录制相机位姿时，两种权重仍有明显左腕误差。当前差异也来自接触后物体姿态和观测的变化。诊断在数据盘 `runs/wiping-rim-measurement/wiping-*-action-diagnostics.json`。场景仍使用估计的泡棉、织物和铰链材料参数，以及依据录制估计的物体外形和安装位置。


## 实测红银包装主体与挂片

首帧原始深度的红色顶面给出约 122 × 34 mm 主体，结合桌面和可见底边得到约 94 mm 高度。薄挂片向长轴外延伸约 38 mm，沿同一纸面测得约 11 × 28 mm 的开孔。当前模型使用主体和带孔挂片的碰撞几何，保留此前设定的 0.12 kg 整体质量。图像无法确认包装内有无电池，纸板厚度 1.4 mm 是估计值。几何拟合和像素坐标保存为 `runs/packaging-red-carton-depth.{py,json}`。

131 步官方动作回放中，分件模型第 27–40 步持续满足抓持条件，随后掉落。前一版整块模型第 21–25 步满足条件。新版整段动作仍未入盒或完成合盖，第 92–103 步采样到右臂末端与大纸盒折舌接触。用同一份 SetXF 权重和种子 1–5 闭环复跑新版红银包装，0/5 入盒，4/5 合盖。旧版红银包装为 0/5 入盒、3/5 合盖。逐步轨迹与评估结果位于 `runs/packaging-red-carton-slotted-trace.json` 和 `runs/red-carton-measurement/red-box.final-sim.json`。上节的新版纸盒接触参数下，同一组种子得到 0/5 入盒、1/5 合盖。


## Wiping 盘沿观测

将官方示范抓取前 13 帧的 13,820 个盘沿点按录制中的盘心与法向配准。用每像素约 3.9 mm 的正投影聚合蓝色区域。交替把帧分成两组后，两组同时有足够样本的像素上，蓝色分类一致率约 91.7%。配准和取样记录位于 `runs/wiping-rim-texture-{fit.json,samples.npz}`，生成脚本为 `runs/wiping-fit-rim-texture.py`。高频印刷细节和盘底外观仍缺少直接观测。

公用盘沿剖面生成 MuJoCo 渲染网格。EGL 图像验证了盘沿图案的方向。原来承受物体接触的碰撞轮廓、盘子惯性和抓取判据继续使用。校正 LEAP 手模型前，111 步同动作回放抓起两件物体，内盘面接触为 0。可见图案与开环结果在 `runs/wiping-measured-rim-render.png` 和 `runs/wiping-measured-rim-replay.json`。


## Sponge Picking

论文图 5 中的单手拾取任务使用与 Wiping 相同外观的分层海绵。仿真复用已测量的弹性海绵，移除盘子，按论文在 40 × 70 cm 区域采样。单只手用两处不同的网格表面形成相向接触，海绵脱离桌面且质心上升超过 30 cm 时，任务通过。 [论文图 5 与任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6)

旧手模型使用公开 Wiping 轨迹的接近段和按厘米递增的腕部目标，在单手接触中把海绵抬高 306.0 mm。该结果依赖包含指尖 IK 定位球的旧模型。校正 LEAP 后，复跑同一脚本只抬高约 80.1 mm，未过 30 cm 阈值。验证脚本和每步记录存于数据盘的 `runs/sponge-picking-physical-lift-probe.{py,json}` 和 `runs/sponge-picking-leap-mesh-lift.{py,json}`。

`python -m simulation.rollout --task sponge-picking` 接受项目使用的 46 维动作 HDF5 与兼容策略权重。作者当前公开数据文件未包含 Sponge Picking 轨迹或权重。 [发布的文件列表](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/main)


同一条分阶段夹持脚本的接触诊断显示，基线峰值采样时手部合计法向力约 265 N，最大接触穿透约 15.7 mm。单次对照把 flex 的 `solref` 时间常数设为 20 ms，总力峰值时的手部接触力约 4.2 N，最大穿透约 2.5 mm。两组都未建立持续的抬升夹持。该参数最初用于诊断，脚本和结果在 `/data/dexcap-repro/benchmarks/sponge-picking-flex-solref-ab.{py,json}`。 [MuJoCo 接触参数](https://mujoco.readthedocs.io/en/3.6.0/modeling.html#contact-parameters)


用真实指尖规划接近，再按两指法向力反馈调整手指目标，可以从默认桌面形成相向夹持。最初的夹持会在抬升约 74.1 mm 后滑脱。按相向接触重新回放后，74.1 mm 的夹持记录保持，另一次碰撞弹出的旧读数从 86.8 mm 改为 0。相关轨迹保存于 `runs/sponge-picking-tactile-centered.{py,json}`。

新版 Wiping 和 Sponge Picking 共用 20 ms 海绵接触时间常数，以及 MuJoCo 的摩擦约束阻抗比 4。材料摩擦系数沿用原值。带负载的试验把海绵压缩约 9.8 mm，逐步撤去压板后恢复原厚度。标准入口回放 305 个脚本动作，将海绵从默认桌面抬升 **306.4 mm**，在 30 cm 以上用同一只手保持相向接触 12 个指令，双臂无碰撞。策略反馈轨迹及标准回放结果分布在 `runs/sponge-picking-tactile-stiffer-friction.{py,json}` 和 `runs/sponge-picking-scripted-rollout.json`。此验证覆盖一次固定摆放。 [MuJoCo 防滑说明](https://mujoco.readthedocs.io/en/3.6.0/modeling.html#preventing-slip)

```bash
python -m simulation.rollout data/derived/sponge-picking-scripted-reference.hdf5 \
  --task sponge-picking --output runs/sponge-picking-scripted-rollout.json
```

## Packaging 六类参考动作回放

在官方 Packaging HDF5 上指定 `--task packaging --object blue-block` 可以读取该物体的原始参考动作。六类物体分别使用 `demo_0`、`demo_50`、`demo_100`、`demo_150`、`demo_175`、`demo_250`。起点使用各自首帧可见物体点的平面包络中心。长方体红盒、蓝积木还采用首帧点云估计的偏航角。其余形状缺少可靠朝向，纸盒仍用共同的参考起点。点云测量存于 `runs/packaging-reference-placement-fits.json`。

| 物体 | 参考轨迹动作数 | 入盒 | 合盖 |
| --- | ---: | ---: | ---: |
| 红银包装 | 131 | 否 | 是 |
| 蓝积木 | 119 | 否 | 是 |
| 绿梨 | 141 | 否 | 否 |
| 绿罐 | 199 | 否 | 否 |
| 卷尺 | 91 | 否 | 是 |
| 黄色折叠布 | 147 | 否 | 是 |

表中六条参考回放使用校正前的 LEAP 手模型，结果位于 `runs/packaging-reference-replays/`。闭环评估从与原基线配对的随机种子采样。

红银包装的录制红色表面在第 37 步约位于 z = 223 mm。分件模型使用上一版参考起点时，同一步质心高 89 mm，右腕沿目标的位置误差约 4.4 mm，盒体高度滞后远超腕位差。相应录制轨迹在 `runs/packaging-official-red-surface-track.json`。六个原始 RGB-D 帧拟合出的台面倾角可辨，局部高度随帧变化约 12–15 mm。只把首帧倾斜桌面代入回放会提前失去抓持，暂作为标定诊断记录在 `runs/packaging-table-plane-multi-frame.json` 与 `runs/packaging-red-carton-tilted-table-probe.json`。


从默认摆放用完整双臂执行 463 个目标动作，右手真实夹起红银包装、转运并松手。物体完全落入纸盒且由盒底承托，`object_in_box=true`，机械臂无碰撞。Packaging 场景保留材料摩擦、几何与惯量，使用椭圆摩擦锥、Newton 容差 `1e-10` 与摩擦约束阻抗比 12，以减少软约束在转运中的漂移。脚本与结果位于数据盘 `runs/packaging-red-carton-centered-placement.{py,json}` 和 `runs/packaging-red-carton-scripted-placement-rollout.json`。 [MuJoCo 求解设置](https://mujoco.readthedocs.io/en/3.6.0/modeling.html#solver-settings)

```bash
python -m simulation.rollout data/derived/packaging-red-carton-scripted-placement.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-scripted-placement-rollout.json
```

同一场景和求解参数下，单独回放 22 个左手动作可把空盒的主铰链合至 182.2°，撤手后保持合盖。自由纸盒随动作移动约 100 mm。首次组合轨迹在标准重放中夹折舌时，无名指撞桌的接触力约 96 N，扰动了拇指。保持无名指的接近姿态，继续用拇指夹住折舌，可在同一初始摆放下完成 585 步标准回放。右手抓起红银包装、放到盒底并撤回，左手合盖后撤离。最终 `object_in_box=true`、`lid_closed=true`、`sim_success=true`，机械臂碰撞计数为 0，自由纸盒移动约 81.5 mm。对照和最终动作生成脚本在 `runs/packaging-lid-contact-trace-{world,canonical}.json` 和 `runs/packaging-red-carton-table-clear-closure.py`。标准回放只验证了这一种物体和固定摆放。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-table-clear-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-table-clear-rollout.json
```


## Ball Collecting

`python -m simulation.rollout --task ball-collecting` 使用双 Franka 与 LEAP、右侧的球和左侧的自由线篮。两处各在论文规定的 40 × 30 cm 区域采样。直径 67 mm、质量 57.7 g 的球用网球标准范围中值。篮子约 230 × 150 × 130 mm，按照图中球径比例估计，用 50 根可碰撞圆柱组成交叉底网和侧壁，保留开口和线间空隙。篮重和表面摩擦为假设。 [论文图 5 与任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6) · [ITF 网球规格](https://www.itftennis.com/media/7221/2026-rules-of-tennis-english.pdf)

判据要求右手形成相向接触、球实际抬离桌面，再释放到篮中。进篮通过完整球体位置、篮底铁丝接触和低速度判定。物理试验验证了球经重力落入篮底后能够稳定承托。在默认球和篮子位置，一条 252 步脚本轨迹通过标准回放入口完成了从空手接近、夹球到球由篮底承托的全过程。右手用三处真实手部表面建立抓持，球相对起点最高上升约 169 mm，转运时接触篮沿并落入篮中。全程没有机械臂碰撞。结果只覆盖默认摆放和这条脚本动作，公开数据仍缺该任务的作者轨迹与权重。生成脚本及逐步接触记录见 `runs/ball-leap-tabletop-reference.{py,json}`。

```bash
python -m simulation.rollout data/derived/ball-collecting-scripted-reference.hdf5 \
  --task ball-collecting --output runs/ball-leap-tabletop-rollout.json
```

球掉下桌面后由估计在桌面下 0.75 m 的地面承托。


## Scissor Cutting

`python -m simulation.rollout --task scissor-cutting` 使用固定分隔托盘、两个可碰撞的剪刀刚体、被动铰链、与铰链实际相连的镂空手柄和粉色纸带。纸带两侧由软连接保持连续。每个 MuJoCo 子步读取刀刃压力，只有剪刀真实闭合且双侧压力到达接缝邻域时才断开局部纸带键。累计切口超过 3 mm 为论文的切割成功条件。初始布局来自图 5，尺寸和纸张断裂力仍按仿真设定。纸带只支持已建模的接缝。 [论文图 5 与任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6)

在包含双臂和摄像的正式 2 ms 场景中，对铰链施加力矩后实际断开一个 4 mm 接缝段。把纸带放到刀口之外时切口保持为 0。这个试验验证机构和子步回调，整段双手抓剪刀、提纸和切割仍缺官方轨迹。记录见 `runs/scissor-production-hook-probe.{py,json}`。


使用旧 44 × 22 mm 柄孔的接触夹具时，真实 LEAP 触碰手柄使剪刀从 0.450 rad 闭合到 0.199 rad，峰值手柄接触力 14.6 N。接触只维持三个物理子步，剪刀随后脱手。旧柄孔无法可靠穿过拇指网格，轨迹存于 `runs/scissor_leap_aligned_fixture.{py,json}`。

按 LEAP 网格改为 50 × 40 mm 的双指孔及 40 × 42 mm 的拇指孔，并补齐两侧连到被动铰链的柄臂。保守包络检查得到指面与孔边的总净余量约 5.3–6.7 mm，闭合位置两环仍分离约 14.6 mm。新版机构试验仍能在实际刀刃接触下切开 4 mm 接缝。测量见 `runs/scissor_handle_fit_check.json`，新版预插入试验验证了两指能在两个孔内保持正净空。物理执行中，接触只短暂支撑了剪刀，两个柄孔未同时承载。默认桌面姿态的尝试还出现约 25.1 mm 的手与桌面或托盘穿透。结果见 `runs/scissor_leap_new_handle_fixture.json` 和 `runs/scissor_leap_tabletop_reach.json`。随后用右手食指从剪刀轴附近托住 0.62 s，实际抬升约 25.6 mm，活动把手与托盘隔板的峰值净空增加约 117 mm。左手插入与交接仍需验证。记录在 `runs/scissor_right_shank_tilt.{py,json}`。

## Tea Preparing

`python -m simulation.rollout --task tea-preparing` 使用固定茶盘、可移动的空心瓶与茶壶、弹簧镊子和可自由落下的茶粒。瓶盖有独立自由关节。螺纹通过 MuJoCo 的转角与轴向滑动约束表达，达到两圈并实际升到约 4 mm 的退丝位置后解除连接，瓶盖保留当时的位姿和速度。壶内茶粒必须由壶体实际支撑并低速稳定，工具归位依靠实际托盘接触。论文规定了两圈和任务顺序，螺距、器具尺寸、材料与归位容差为估计。 [论文任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6)

在支稳茶瓶的测试中，对帽体施加 0.007 N·m 力矩，转一圈时实际升约 2 mm 并保持连接，接近两圈和 4 mm 后成为独立刚体。正式双臂场景中，从壶口上方放开的一粒茶在约 1.75 s 后由壶底承托并计为入壶。整段双臂开盖、辅助倒茶和归位仍缺作者的动作与权重。结果见 `runs/tea-pot-production-drop.json`。

把瓶体固定为试验夹具，右侧 Franka 与 LEAP 靠真实手指接触驱动瓶盖达到 **1.19 圈**，相应轴向行程 2.32 mm。重新夹持过程中会丢失非拇指接触，仍未达到两圈退丝。记录见 `runs/tea-leap-cap-robot-probe.{py,json}`。另一次换指试验在转到约 0.58 圈后失去拇指制动接触，盖回退至约 0.48 圈，见 `runs/tea-leap-cap-hinge-regrasp-probe.{py,json}`。


## 六个场景的渲染与接入验证

六个场景均以同一套双 Franka、双 LEAP、OSC 和 RGB-D 入口在 `pro1` 加载，执行实际控制并输出 `10000 × 6` 点云。35 项 MuJoCo 专测和汇总测试通过。场景总览图与接入记录保存于数据盘 `runs/task-scene-overview-grasp-physics/six-mujoco-scenes.png` 和 `runs/task-scene-overview-grasp-physics/smoke.json`。Scissor Cutting 和 Tea Preparing 已验证关键机构的物理响应，仍缺整段双臂任务动作。


## 校正 LEAP 指尖碰撞与惯量

DexCap 的重定向 URDF 比 LEAP V1 官方物理模型多四个 `realtip` 固定链接。它们各带半径 5 mm 的圆球和 0.1 kg 的惯量条目，用于表示指尖 IK 目标。现在构建 MuJoCo 手部时只加载官方物理模型对应的 17 个实体链接。原始 DexCap URDF 仍供发布数据的重建使用。编译后每只手质量由 0.909 kg 校正为 0.509 kg。 [LEAP V1 官方 URDF](https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf)

首次校正 LEAP 之后，两条参考动作按当时的放置与控制参数回放。该版本 Wiping 的右、左腕跟踪误差中位值约为 2.3/4.6 mm，未抓起盘子或海绵。Packaging 红盒的中位误差约为 4.8/5.6 mm，本次未持续抓起、入盒或合盖。校正后的回放见数据盘 `runs/{wiping,packaging}-physical-leap-replay.json`。

另用 Wiping 和 Packaging 的 `demo_0` 分别对照 25 和 22 个录制观测。将已发布的腕姿和当前解码的手关节传给校正后的 LEAP 网格，距离手部 12 mm 内的录制黑点到网格采样点的中位距离均约 0.83 mm，逐帧的 2 mm 以内比例中位数约 98%。这验证了输入腕姿、当前手指顺序和作者预处理点云之间的对应。数据及脚本在 `runs/{wiping,packaging}-released-hand-observation-order.{py,json}`。

Wiping 的前 54 步中，第 24 步食指先撞盘沿。第 28 步仅一个控制周期检测到拇指和另一指的同时接触，盘始终受桌面支撑。第 42–44 步右腕位置误差约 0.2 mm，盘中心相比录制状态已偏离约 71–79 mm。手部保持相向接触是当前的主要缺口。逐步接触法向、支撑和腕姿记录在 `runs/wiping-current-wrist-plate-proof.{py,json}`。

从已经夹到盘沿的声明式试验起点，三处真实指面持续接触，将盘子从桌面抬高 81.8 mm。默认放置时，夹持需要的下侧接触点与桌面等高。另用真实食指压盘内侧斜唇，盘保持桌面支撑并倾至 7.26°，对侧下缘净空峰值 21.1 mm。超过 5 mm 的净空持续 0.30 s，盘随即滑动回落。两组结果分别见 `runs/wiping-physical-plate-grasp-probe.{py,json}` 和 `runs/wiping-plate-tilt-clearance-probe.{py,json}`。


双手支撑对比中，正交左指支点比对侧平夹更容易产生净空。根据盘体实时位姿移动左腕，仍只在实际支点接触下维持 150 ms 的 8 mm 以上净空，未执行插指。目标移动 15.1 mm 时腕部只跟上 6.5 mm，显示这套左手姿态缺少可达余量。数据见 `runs/wiping-plate-{orthogonal,rotating}-brace-probe.json`。右手两指垫支点的静态构型可保留约 7.5 mm 桌面净空。通过正常力限控制从上方接近时，腕部仍偏约 8.1 mm，未建立预期支点接触。记录在 `runs/wiping-right-local-pivot-probe.{py,json}`。

海绵接触参数更新后的 40 个 Wiping 回合中，旧判据给出 PointNet 0/20、SetXF 2/20 成功。两条被判成功的 SetXF 回合的峰值接触力约 15–16 kN，原判据还允许未同时持物的盘面接触累计为有效笔画。两条种子的单独复跑均未重现成功。现在只在右手相向夹持已离桌的盘子、左手相向夹持已离桌的海绵，并发生内盘面接触时累计擦拭笔画。保留原始内盘面接触次数供诊断。原始汇总和复跑记录在 `runs/sponge-contact-benchmark/`。
