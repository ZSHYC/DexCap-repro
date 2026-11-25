# DexCap MuJoCo 迁移

六项任务共用双 Franka、双 LEAP 的 MuJoCo 环境。Wiping 和 Packaging 可加载公开示范以及本项目已训练的权重。Packaging 保留六个训练物体、九个未见物体和五组配对摆放。Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing 使用本项目在仿真中生成的参考动作验证，另外四项的官方轨迹与训练权重仍需要数据来源。 [论文任务](https://arxiv.org/html/2403.07788v2#A1.SS6) · [作者数据发布](https://huggingface.co/datasets/chenwangj/DexCap-Data)

## 模型与控制

用 Menagerie 的 Franka Panda 机械臂和本项目已有的 LEAP 关节定义构建完整机器人。仿真中的实际腕位和 32 个手指关节值转回发布数据的坐标及数值格式，沿用 46 维动作和三帧点云观测。用腕位目标驱动机械臂，在物理子步内执行力矩或受限力控制。手指通过关节目标和有限驱动力运动。任务物体通过碰撞、摩擦和重力移动。基座位置、法兰到掌座的变换以及物体材料参数需要对照公开标定和原始示范验证。


### 碰撞统计的物理采样

每个 50 ms 控制周期记录其中任意 2 ms 子步出现的机械臂或双手间接触，单个周期最多计一次。同手、双手间的最大穿透也跨全部子步取峰值。任务判据沿用原控制周期采样。旧版本只读取每个控制周期末端的接触，旧报告中的碰撞计数和峰值对应当时的采样。

按新统计重新运行 20 个固定初态捡海绵和两次辅助收球的标准 EGL 回放，均通过各自任务，臂、双手间与同手接触指标均为零。胶带座入盒仍能复现，左手早期穿透峰值由控制周期末端约 2.89 mm 更新为逐子步约 3.05 mm。擦盘第一笔仍为 71.84 mm，右手穿透为零，实测左食指逐子步峰值约 1.71 mm。两个任务仍需完成合盖或第二笔。证据见数据盘 `benchmarks/substep-reference-audit/standard-substep-replays/summary.json`。

### LEAP 关节止挡

在满载泡茶的右手试验里，旧模型虽然读取了 URDF 的角度范围，但实际指尖关节曾越限 3.47 rad，MuJoCo 未产生警告。上轮两圈退丝的手形也已越限，因此该轨迹作为诊断记录保存。当前 32 个 LEAP 关节继续使用 URDF 原角度范围，止挡时间常数从 20 ms 调为 4 ms，并加入估计的 0.0005 kg·m² 电机反射惯量。以相当于一个伺服的 0.95 N·m 向指尖关节两端施加隔离负载，峰值越限从约 0.0433 rad 降到 0.0007 rad。相同泡茶接近、合拢及首段转腕的对照中，最大越限从 3.47 rad 降到约 0.0004 rad。只改止挡而不加反射惯量的对照仍越限约 2.60 rad。未对真实舵机惯量做实物标定。 [MuJoCo 关节与反射惯量](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-joint)

改动后通过标准入口重放四份完整参考。305 步 Sponge Picking 抬升约 303.46 mm，265 步 Ball Collecting 完成落篮。586 步红银包装参考在第 119 至 420 动作保持物体，随后按原动作张手时物体仍在盒外。这份参考能合盖。另复核 463 步转运参考，物体由右手实际搬运、释放并留在盒内，纸盒位移约 0.97 mm。这份参考尚需接上闭盖。946 步 Wiping 参考抓起海绵，盘子未形成对向夹持。第 154 动作只有右食指接触盘边，拇指仍差约 27.5 mm。单独补偿拇指没有建立对夹。Scissor Cutting 和 Tea Preparing 分别跑了 8 步及 1 步的入口复测。六次回放都输出 `10000 × 6` 点云，逐物理子步测得最大的手指越限约 0.00492 rad，出现在 Wiping。六次均无求解器警告。旧模型下的其它参考成功记录仍需按现行资产复核。数据见 `runs/tea-leap-limit-load-*.json` 和 `runs/leap-stop-physical-*.json`。红银转运和抓盘诊断分别见 `runs/leap-stop-physical-red-scripted-placement-audited-762a293.json` 和 `/data/dexcap-repro/benchmarks/wiping-leap-stop-right-grip-diagnostic.json`。

标准回放每个 2 ms 子步采样双手的实际角度。任一关节偏出模型止挡超过 0.05 rad 时，在当前控制周期后结束该回合，`invalid_hand_motion=true`，该回合在所有结果统计中计作失败。0.05 rad 是容纳软止挡微小越限的仿真容差，本批校准参考回放的实测最大值约为其十分之一。在一次实际入口测试中，仅第 5 个子步注入 0.16 rad 的瞬时越限，动作采样前已恢复，回放仍在第一动作结束并判定失败。正常 535 步包装轨迹完成，峰值仅 0.00141 rad。见 `runs/packaging-stop-transient-injection.json` 与 `runs/packaging-red-combined-standard-gated.json`。

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
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
bash scripts/fetch_mujoco_assets.sh
MUJOCO_GL=egl python -m simulation.rollout data/derived/wiping-demo-0-playback.hdf5 \
  --output runs/wiping-replay.json
MUJOCO_GL=egl python -m simulation.rollout data/derived/packaging-demo-0-playback.hdf5 \
  --task packaging --output runs/packaging-replay.json
```

精细接触的回放固定 BLAS 与 OpenMP 各用一个线程。用同一份原生动作重放擦盘时，多线程计算曾改变盘子和海绵的接触轨迹。单线程诊断与标准入口的擦拭结果一致。

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

Packaging 的固定示范回放使用各物体录制起点的偏航。带种子的动作回放和策略评估使用任务的同一份初始摆放规则。以蓝积木种子 5 的实际 SetXF 前 27 个目标及其 50 ms 周期数复放。修正前，动作回放多用了示范中的 61.47° 偏航，未复现策略的抓持。修正后，抓持标志、关节与物体积分状态一致。记录和 HDF5 在数据盘 `benchmarks/packaging-material-calibrated-setxf/` 的 `blue-block-seed5-prefix-{before,after}-yaw-fix.json` 和 `blue-block-seed5-learned-prefix-reference.hdf5`。

剪纸和备茶的带种子回合也按种子生成场景。剪刀沿托盘支撑线移动 ±8 mm，靠墙纸带沿墙移动 ±35 mm。备茶在托盘坐标下让茶瓶沿 x 移动 0–18 mm、沿 y 移动 −14–0 mm，茶壶两轴各移动 ±12 mm。托盘朝向、镊子和木托架摆放固定。上述范围依据现有模型的物理支撑选取，原文没有公布这两项的具体采样范围。标准评估入口的种子 1、2 已实测生成不同坐标。正式场景在采样边界沉降 0.6 s 后，刀片仍有原支撑，纸带上端高度保持约 82 mm，备茶仍保留 108 颗茶叶且镊子由木托承载。记录位于 `/data/dexcap-repro/benchmarks/seeded-tool-scenes/`。 [论文评估与摆放定义](https://arxiv.org/html/2403.07788v2#S5.SS1)

历史 PyBullet 仿真代码和入口保存在基线提交。当前分支的 `simulation.rollout`、示范图和策略动作诊断均运行 MuJoCo。官方数据重建仍沿用作者的 PyBullet 指尖 IK。 [基线提交](https://github.com/ZSHYC/DexCap-repro/tree/0ee5418)


在复合掌心模型上以发布的 Wiping HDF5 与已有的第 3000 轮权重重跑配对闭环。两种编码器各取相同的种子 1–20，每回合上限 400 个策略动作。PointNet 有 3 个回合抓起盘、2 个抓起海绵、1 个回合两物均曾抓起。SetXF 的对应计数为 2、4、0。两种编码器各有 2 个有效回合接触内盘，均未完成完整擦拭，成功数均为 0/20。完整轨迹可在固定摆放下通过标准回放，随机闭环表统计已有权重的实际输出。数据与启动命令位于 `runs/wiping-compound-palm-benchmark/`。

SetXF 的种子 9 触发了 MuJoCo `mjWARN_BADQACC`，原始日志记录于模拟时间 23.02 s。分析时按物理异常回合计为失败。现在标准入口遇到 MuJoCo 警告会终止该回合，并记录 `executed_steps` 和 `physics_warnings`。异常回合计入总尝试次数，其任务指标在汇总中不记为通过。


种子 9 的独立复盘把异常定位到盘子自由关节的旋转自由度。第 60 个策略动作中，仿真报告海绵贯入盘底约 68 mm，盘子被异常接触力抛离桌面。第 73 个动作盘子撞到房间地面后触发数值警告。以上是失稳轨迹的数值记录。逐物理步诊断见 `/data/dexcap-repro/benchmarks/wiping-setxf-seed9-badqacc-diagnostic.json`。


## 115 回合闭环复测

在 `pro1` 上固定 `3fb7ef3` 的源码、机器人和任务模型，用 GPU 0、1、2、3、4、6、7 并行运行。复用第 3000 轮权重、发布的 HDF5、各组原有的配对种子及 400 步上限。全部 115 个回合执行完毕，未触发物理警告或手指越限终止。

| 权重及任务 | 回合 | 抓持或入盒 | 接触内盘或合盖 | 完整成功 |
| --- | ---: | --- | --- | ---: |
| PointNet Wiping | 20 | 盘 1，海绵 0 | 内盘面接触 1 | 0 |
| SetXF Wiping | 20 | 盘 0，海绵 1 | 内盘面接触 1 | 0 |
| SetXF Packaging 训练物体 | 30 | 入盒 0 | 合盖 7 | 0 |
| SetXF Packaging 未见物体 | 45 | 入盒 0 | 合盖 14 | 0 |

完整数据、启动参数和汇总在 `/data/dexcap-repro/runs/mujoco/palm-oval-closed-loop-3fb7ef3/`。原始内盘面接触包含未同时夹持盘和海绵的样本。该批次以两凸块掌部模型为准，后续十凸块模型与右手抬盘参考另有记录。

对两凸块批次中实际抓起物体的两个 Wiping 回合，用十凸块模型 `dd14a48` 和原种子复跑。PointNet 种子 12 仍抓起盘子，SetXF 种子 13 原先抓起海绵，新模型未抓起。两个回合均执行 400 步，无物理警告、手指越限或完整任务成功。这两个种子用于观察掌部几何改变后的抓持差异。结果在 `/data/dexcap-repro/runs/mujoco/palm-x10-paired-contact-dd14a48/`。

## 115 回合闭环结果

| 权重及任务 | 回合 | 抓取或入盒 | 内盘面接触或合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: |
| PointNet Wiping | 20 | 两件都曾抓起 1 | 内盘面接触 0 | 0 |
| SetXF Wiping | 20 | 两件都曾抓起 0 | 内盘面接触 0 | 0 |
| SetXF Packaging 训练物体 | 30 | 入盒 0 | 合盖 4 | 0 |
| SetXF Packaging 未见物体 | 45 | 入盒 0 | 合盖 7 | 0 |

115 个回合均使用 MuJoCo 3.6.0 和校正后的 LEAP 实体网格。Wiping 结果来自这批评估运行时的 125 mm 半径盘型。逐回合核对确认数据、权重、步数和初始摆放与 `runs/leap-physical-benchmark/` 中的结果一致。Wiping PointNet 有 2 个回合抓起盘子、2 个回合抓起海绵，SetXF 分别为 0 和 3。表中的内盘面接触统计原始几何接触。累计擦拭笔画要求在同一次计分采样中双手都实际持物。Packaging 的接触参数已更新，75 个回合按后续加入 0.1 mm 入盒边界容差之前的条件计分。论文中的成功率来自实体机器人。 [逐物体汇总](assets/dexcap-formal-mujoco.json) · [论文评估](https://arxiv.org/html/2403.07788v2#S5.SS1)

后续从绿罐实际入盒检查定位到 Packaging 圆柱的尺寸传递错误。对象表使用 `(半径, 半径, 半高)`，MuJoCo 圆柱使用前两项作为半径和半高。现在在场景装配时转换尺寸，因此绿罐使用 34 mm 半高，蓝杯底使用 3 mm 半高，单体和复合物体的质量与配置的 120 g 一致。旧表为修正前模型，受影响的绿罐、卷尺、耳塞瓶、叠块、蓝杯、长颈鹿、狮子和弹簧夹均需使用新模型重评估。

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


为拆分相机颜色和点分布的影响，对 `demo_0` 至 `demo_3` 的接近段取 28 组配对三帧输入，每组使用两个固定推理种子，并保持录制腕位、手关节和相机位姿。当前渲染下，PointNet 和 SetXF 的右、左腕误差中位数分别为 **6.79 / 7.63 mm** 和 **4.41 / 6.23 mm**。仅替换匹配表面的 RGB 后，两组分别为 **6.95 / 7.21 mm** 和 **4.15 / 6.82 mm**。按录制数据的表面类别数重采样后，分别为 **7.27 / 8.18 mm** 和 **4.49 / 6.65 mm**。颜色和各表面点数的单独调整都未稳定缩小双腕误差，因此没有据此改变生产输入。完整数据在数据盘 `benchmarks/wiping-observation-gap/wiping-{pointnet,setxf}-{color,coverage}-ablation.json`。

## Wiping 盘沿观测

从作者原始 ZIP 按字节范围提取了 `demo_0` 对应的四个 RGB-D 样本，文件在数据盘 `data/raw/save_wipe_1-14/save_data_wipe_1-14_01/`。校正后四帧相机矩阵与 HDF5 第 0、15、25、70 帧一致。原始深度拟合得到盘子位置桌高约 −26.3 至 −27.4 mm，海绵位置约 −34.3 至 −35.1 mm。以首帧局部桌面为基准，盘沿外侧半径 115–125 mm 范围内亮色表面的高度中位数约 22.9 mm。原始取样和拟合记录在 `runs/wiping-raw-samples.log` 和 `runs/wiping-raw-source-geometry.{py,json}`。 [作者原始数据](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0)

实验中用测得的桌面、物体初始倾角以及对准桌面法线的重力重放同一条 111 步官方轨迹。第 107 步出现约 12.8 kN 的接触力峰值和腕跟踪失稳。此后才记录到盘子离桌和内盘面接触，擦拭距离仍为 0。生产场景继续使用原水平桌面。对照见 `runs/wiping-raw-table-replay-ab.{py,json}`。

将官方示范抓取前 13 帧的 13,820 个盘沿点按录制中的盘心与法向配准。用每像素约 3.9 mm 的正投影聚合蓝色区域。交替把帧分成两组后，两组同时有足够样本的像素上，蓝色分类一致率约 91.7%。配准和取样记录位于 `runs/wiping-rim-texture-{fit.json,samples.npz}`，生成脚本为 `runs/wiping-fit-rim-texture.py`。高频印刷细节和盘底外观仍缺少直接观测。

公用盘沿剖面生成 MuJoCo 渲染网格。EGL 图像验证了盘沿图案的方向。纹理首次加入时沿用了当时的碰撞轮廓、盘子惯性和抓取判据。校正 LEAP 手模型前，111 步同动作回放抓起两件物体，内盘面接触为 0。可见图案与开环结果在 `runs/wiping-measured-rim-render.png` 和 `runs/wiping-measured-rim-replay.json`。


随后对原始深度的第 0 和 15 帧各自做外缘圆拟合，半径分别为 133.81 和 133.82 mm，圆拟合残差中位数均低于 0.7 mm。生产场景据此更新盘心、半径和碰撞与渲染共用的径向剖面。115–125 mm 的盘沿高于盘底内表面约 21 mm。盘底和盘沿按 2 mm 建模，厚度和惯量均缺少实物标定。测量与模型对照在 `runs/wiping-measured-thin-plate-profile.{py,json}`。标准入口完成 111 步并输出 `10000 × 6` 点云，两件物体均未抓起，机械臂接触计数为 19。记录在 `runs/wiping-measured-plate-standard-rollout.json`。

将五帧原始右手指尖投影到同步画面中，观察到拇指先进入盘内、食指从外侧靠近。第 27 帧发布动作在机器人运动学中把食指碰撞网格压入仿真桌面约 32.7 mm，拇指网格到录制指尖约 15.7 mm。正常回放第 27 步的右食指桌面接触力约 74.7 N，采到的受力点与发布网格的三角面相符。记录在 `runs/wiping-source-right-hand-contact-path.{py,json}` 和 `runs/wiping-leap-hull-contact-diagnosis.{py,json}`。

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


蓝积木后来用校正后的双 LEAP 回放了官方 `demo_50`。右手与物体接触 25 个周期，积木未抬起。以红盒参考为起点调整到蓝积木的实测位置后，静态腕姿候选可产生相向法线，但从上方实际进入时仍未取得相向夹持。最高积木中心离桌约 18 mm，与它平放时的半高相符。记录见 `runs/packaging-blue-block-official-reference-rollout.json` 和 `runs/packaging-blue-block-tilted-grasp-probe.json`。

复合掌心下，把蓝积木预抓腕姿倾斜约 −35°，右食指与拇指在默认摆放形成相向抓持，积木实际离桌约 18.6 mm 后滑落。减慢抬升后积木曾稳定在约 32 mm 高处，但接触点移动后已不满足相向抓持判据。两次均未完成入盒。数据见 `runs/packaging-blue-block-opposed-grasp-66b7507.json` 和 `runs/packaging-blue-block-opposed-grasp-slow-lift-66b7507.json`。


保持蓝积木的初始预抓姿态，在抬升中将腕俯仰从 −35° 缓调至 −30°，延长食指与拇指的相向抓持。50 动作标准入口重放在 `demo_50` 默认摆放获得 `object_grasped=true`，机械臂碰撞计数 0。此时积木中心高约 61.8 mm，长轴接近竖直，最低角比盒沿低约 49.0 mm。继续抬升和转腕的试验均失去抓持，尚未入盒。

```bash
python -m simulation.rollout data/derived/packaging-blue-block-apex-grasp-native.hdf5 \
  --task packaging --object blue-block --output runs/packaging-blue-block-apex-grasp-native-rollout.json
```


在当前完整碰撞模型重新执行上述 50 步，右手物体接触累计 133 个周期，终态 `object_grasped=false`。右腕最大位置误差约 54.84 mm，右中指与拇指末端最大采样穿透约 14.69 mm。机械臂与双手间接触步数均为零，无物理警告。当前模型采用这次复验的结果。数据见 `/data/dexcap-repro/benchmarks/runs/blue-block-demo50-apex-verify.json`。

重新测量右食指、拇指实际接触点，按蓝积木两侧真实法线拟合腕姿和指形。中指、无名指与闲置左手保持张开。实际抓取时两指负载约 3.05 N 与 3.02 N，相向法线点积为 −1。逐步物理记录中，积木最低点最高离桌约 33.9 mm，末态仍保持相向接触。456 步独立 EGL 回放确认抓持，最大右腕位置误差降至 5.60 mm。另一组对向抓姿用于整套转运，从同一 `demo_50` 摆放夹起积木并移到开盒上方。倾斜松手后，积木平躺在盒底，顶面低于盒沿约 40.6 mm。左手随后拉折舌并合盖。

整套 **1,237 步**在标准 MuJoCo 入口从原摆放用 EGL 独立复放，`object_released`、`object_in_box`、`lid_closed` 和 `sim_success` 均为真。盒位移约 0.36 mm，机械臂、双手间及同手采样穿透均为零，生成 `10000 × 6` 点云，无物理警告。此轨迹中右手负责抓取与放置，随后撤离闭盖区。原 50 步参考的抬升和盒沿限制属于更早的动作。新轨迹、逐步物理记录和 EGL 结果在数据盘 `benchmarks/packaging-blue-current/`。

```bash
blue=/data/dexcap-repro/benchmarks/packaging-blue-current
python -m simulation.rollout "$blue/blue-block-complete-reference.hdf5" \
  --task packaging --object blue-block --output "$blue/blue-block-complete-egl.json"
```

随后把右食指稳盒的接触点下移到箱壁局部高度约 32 mm，避开盒盖的实际翻转区域。按实际接触力调节右腕位置，同时用左手合盖。**1,977 步**标准 EGL 回放完成入盒、合盖与双腕撤离。每 2 ms 的物理记录显示，右食指顶箱壁、左手接触盒盖连续同时承力 2,116 个子步，为 **4.232 s**。右食指峰值 2.61 N，右手未接触盖板。按当时未包含移盒居中的计分，终态为 `sim_success=true`。机械臂、双手间和同手采样碰撞均为零，无物理警告。回放数据为 `blue-block-bimanual-reference.hdf5`，结果与逐子步受力汇总为 `blue-block-bimanual-egl.json`、`blue-block-bimanual-egl-contacts.json`，均位于上述目录。

绿罐采用校正后的 68 mm 全高和 120 g 质量。首次实物动作放入后，盒底单点接触使罐子持续摇摆。在相同运动末态上，MuJoCo 原生多点碰撞检测将盒底最大接触贯入从约 3.15 mm 减至约 0.105 mm，罐子随即稳定在盒底。多点检测在包含圆柱零件的 Packaging 场景装配时启用，已有尺寸、质量与任务计分条件保持原值。数据盘 `benchmarks/packaging-green-current/corrected-height/green-can-collision-manifolds.json` 记录了该对照。 [MuJoCo 多点碰撞说明](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#option-flag-multiccd)

校正模型从绿罐的发布摆放 `demo_150` 执行右手抓取、转运、竖直放入、撤手和左手闭盖。**1,322 步**完整 EGL 复放得到 `object_in_box=true`、`lid_closed=true` 和 `sim_success=true`，无物理警告，机械臂、双手间和同手采样碰撞均为零。该参考的右手撤离后由左手完成闭盖。HDF5 与回放结果在绿罐同目录的 `green-can-complete-reference.hdf5`、`green-can-complete-egl.json`。

蓝积木接近纸盒时还需要最低角增加约 49 mm 净空。试验中沿当前箱缘跨入需要纸盒倾斜约 31°，所执行的左手路线未接触到箱壁。单独保持右手末动作 0.30 s 时蓝积木失去夹持，随后将左手接近与原拾取动作同步，首次碰撞出现在左臂下降阶段，右手随后失去夹持。未执行放入。记录见 `runs/packaging-blue-{box-tip,apex-brace-feasibility,sync-left-prepose}-66b7507.json`。

用 `demo_100` 的参考摆放对绿梨做椭球短轴两侧接近。受力前两指垫的纵向位置仍偏高约 19 mm，实际下降时两侧没形成对向夹持。绿梨向盒壁移动，始终由桌面支撑。见 `runs/packaging-green-pear-fit-66b7507.json`。

隔离对照发现，旧绿梨把中间长度的半轴 27 mm 设在竖直方向，构成不稳定的静置姿态。4.5 s 内，即使双手始终不接触，绿梨也会滚约 103.46 mm。生产模型保留原三个半轴，将最短的 25 mm 轴设为竖直。相同的无手接触试验记录最大漂移 0.49 mm，加 1° 复位扰动时为 1.35 mm。发布 `demo_100` 的可见点云没有完整露出底面，因此这里确定的是现行椭球的物理静置朝向。数据盘 `benchmarks/packaging-pear-complete/` 保存对照 `pear-resting-pose-physical-comparison.json` 及更新后 `stable-pose/pear-stable-production-rest.json`。

在修正后的场景中，按物体和指垫的实际最近点拟合夹持，再独立求张指下降通道。从张手开始分别执行食指—拇指和中指—拇指路径。157 步、170 步标准 EGL 回放均取得相向夹持并抬离桌面约 39–42 mm，无机械臂接触、双手碰撞或物理警告。原抬升预设的闲置左手仍有约 4.18 mm 采样同手穿透。回放和动作在 `benchmarks/packaging-pear-complete/stable-pose/pear-lift-{4-1-2-0,8-0-0-1}-*`。

右手夹梨越过盒沿后，两个实际释放状态均有盒底支撑。复算中，接触维数 3 和 4 的两份状态在末段 100 个样本均未达到静置判据。仅为梨与盒底指定六维接触，启用已有的滚动阻力系数后，两份状态均达到 100/100。纸盒其他表面与任务判据不参与这项改动。对照见数据盘同目录的 `pear-box-floor-contact-dimension.json`。 [MuJoCo 接触维数和滚动阻力](https://mujoco.readthedocs.io/en/3.6.0/computation/index.html#contact)

从 `demo_100` 对应的默认摆放执行 **8,982 个腕位和手指目标**，标准 EGL 回放确认纸盒移动约 **85.38 mm** 并达到居中判据，右手抓起绿梨、越过盒沿、松手后由盒底承托，右手扶盒、左手合盖，最终 `sim_success=true`。以 2 ms 子步检查，两手连续同时承力 **1.046 s**。机械臂接触和双手间碰撞计数均为零，最大同手采样穿透约 0.53 mm，无物理警告。回放和接触审计为同目录的 `green-pear-centered-complete-egl.json` 与 `green-pear-centered-complete-egl-contacts.json`。

```bash
pear=/data/dexcap-repro/benchmarks/packaging-pear-complete/stable-pose
python -m simulation.rollout "$pear/green-pear-centered-complete-reference.hdf5" \
  --task packaging --object green-pear --output "$pear/green-pear-centered-complete-replay.json"
```

在相同绿梨模型上以已训练的 SetXF 第 3,000 轮权重逐一复测种子 1–5。每轮用 EGL 点云闭环执行 400 步。仅种子 5 有右手与梨的接触，共累计 82 个采样接触点。五轮均未抓起或入盒，无物理警告。逐轮报告为同目录的 `green-pear-rolling-floor-policy-seed-{1..5}.json`。

改用高位预抓，下降中首次检测到手与梨接触时，拇指垫最近间隙约 1.49 mm，中指垫仍约 49.3 mm。停在该处单独合拢中指仍未抓起，梨从初始位置移动约 97.7 mm。记录见 `runs/packaging-green-pear-signed-gap-close-b6d8e6d.json`。

从绿梨长轴上方重拟合夹点时，避开桌面的候选仍出现拇指穿入约 2.54 mm，另一指还差约 5.98 mm，接触法线也未相向。随后高位预抓对照中未抓起梨。记录见 `runs/packaging-green-pear-topdown-longaxis-fit-b6d8e6d.json`。

单指实际推动默认姿态下的绿梨后，梨发生翻转和位移，最终朝远离盒壁的方向移动且仍由桌承托。该分支未得到可供夹取的盒壁支撑。记录见 `runs/packaging-green-pear-singlepad-wall-support-0a3652d.json`。

绿罐的官方 `demo_150` 参考有 143 个右手接触周期。减少折叠预抓姿态的自碰撞后实际执行下探和抬升，仍未建立持续的相向抓持。记录见 `runs/packaging-green-can-official-contact-trace.json` 和 `runs/packaging-green-can-neutral-pregrasp-lift-probe.json`。

分块掌心模型上，红银包装仍落入盒底。旧收尾的 115 个左手接触周期把盖子沿开盖方向拨动，最低到约 −4.2°。重选实际折舌接触位置后，左手可通过接触把自由盖子带到约 64.2°，峰值约 71.5°，期间包装始终在盒内。这条动作依赖施加目标时的控制周期数，撤手时盖子重新打开。用标准回放时序重放也没有保持折舌接触。记录见 `runs/leap-palm-packaging-lid-hook-continuation.json` 和 `runs/leap-palm-packaging-lid-arc-finish.json`。

复合掌心下重新完成了红银包装的入盒和闭盖。左手拇指通过真实接触勾住纸盒折舌，把盖子带到约 64.6°，随后沿盖子边缘越过，撤手后盖子稳定在约 180.3°。586 个动作的标准回放确认包装仍在盒内、已释放、盒盖闭合，最终 `sim_success=true`。整段无机械臂碰撞，纸盒平移约 0.67 mm。这次验证覆盖红银包装的固定参考摆放。轨迹将每个动作实际执行的 50 ms 控制周期数保存在 `control_cycles_per_action` 中，前段保留原自适应时序得到的执行长度，闭盖尾段使用实际的 5 或 7 个周期。动作生成器和实测记录见 `runs/packaging-red-carton-live-unbroken.{py,json}`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-live-timed-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-live-timed-rollout.json
```


### 校正模型的 Packaging SetXF 闭环评估

在 MuJoCo 3.6.0 的完整双臂场景执行现有 SetXF 权重 `model_epoch_3000.pth`，使用公开 Packaging HDF5、真实 EGL 点云输入和相同的五组种子摆放。六个训练物件 30 轮，九个未见物件 45 轮，每轮实际执行 400 个策略步。这批结果采用后续纸板显色校准前的材质。

| 物件组 | 右手接触物件 | 抓起后失去抓持 | 入盒 | 合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 训练物件 | 8/30 | 1/30 | 0/30 | 10/30 | 0/30 |
| 未见物件 | 19/45 | 5/45 | 0/45 | 12/45 | 0/45 |
| 合计 | 27/75 | 6/75 | 0/75 | 22/75 | 0/75 |

有 20 轮出现机械臂接触，采样到的最大同手穿透约 15.75 mm。75 轮均无物理求解器警告或手指越限触发。蓝积木和绿罐的完整参考回放使用各自专门生成的动作。闭环逐轮结果和评估参数见数据盘 `benchmarks/packaging-current-policy-summary.json` 及其中引用的 15 个报告。

另在红盒、蓝积木和绿罐各取 13 个时点，共 39 组三帧输入。每组固定录制的关节与腕位观测、策略随机种子和相机标定，只将录制点云替换为对应姿态下的 EGL 点云。右腕和左腕预测分别变化约 18.58 和 13.48 mm（中位数）。相对录制动作，右腕和左腕的中位误差分别从 4.39、3.37 mm 增至 21.07、15.13 mm。场景设置各观测姿态时，腕位最大误差约 0.132 mm。该对照覆盖三份示范的前段，不含闭环接触误差。测量脚本与输出位于数据盘 `benchmarks/packaging-current-noncylinder-policy/paired-reference-camera-probe.py` 和 `benchmarks/packaging-paired-initial-camera-probe.json`。

仿真 RGB-D 按作者数据处理在拼入 LEAP 手部点之前，去除超过 1 m 的场景深度。EGL 回归场景把约 1.15 m 远的盒面排除，近处几何仍正常投影。在上述 39 组配对输入中，裁剪前后的腕位预测相同。逐样本对照位于数据盘 `benchmarks/packaging-paired-depth-ablation.json`。

另对五个物件的 45 个录制时点，按实际渲染几何分区测量。逐组计算仿真点到录制点的最近邻距离，再取各组中位数的中位数，得到手部约 4.2 mm、纸盒盒盖 11.3 mm、盒身 18.4 mm、目标物 13.2 mm。45 个时点中有 19 个在最终点云里留有桌面点。盒身点每帧中位数约 2,755。红盒原始深度的六帧在固定盒心坐标处拟合到约 −41 至 −31 mm 的桌高，在物件处约 −29 至 −19 mm。盒边点云差异还受视角和录制中物件运动影响，单帧测量不足以确定一个公共桌高偏移。区域结果、原始深度拟合分别见 `benchmarks/packaging-paired-surface-regions.json`、`benchmarks/packaging-source-table-planes.json`。

在三份示范的同一组 39 个输入上，用 24 mm 内的近邻仅替换仿真点的 RGB，右腕和左腕误差中位数从 21.07、15.13 mm 降至 6.16、5.76 mm。该替换仅用于定位输入差异。生产渲染根据红盒原始 RGB-D 中同一视角的真实盒底和展开盒盖校准纸板材质。盒底逐通道绝对误差中位数从 `[0.035, 0.118, 0.204]` 降至 `[0.035, 0.027, 0.024]`，颜色范围为 0–1。采用此材质直接渲染，39 组输入的腕位误差中位数为右 9.65 mm、左 10.44 mm。对照在数据盘 `benchmarks/packaging-paired-{color-ablation,material-test}.json`，原始图像比对在 `benchmarks/packaging-current-noncylinder-policy/raw-material-{comparison,current-source}.log`。

在另外三个留出物件绿梨、卷尺和黄布的 39 组输入中，生产材质相对旧材质的右、左腕误差中位数分别由 32.13、19.74 mm 降到 12.60、8.60 mm。对红盒、蓝积木、绿罐各重跑种子 1 和 5，数据、权重、初始摆放和 400 步保持配对。实际曾抓起物件的轮数由 1/6 增至 3/6，三次抓起均发生于种子 5，结束前均失去抓持。入盒 0/6，合盖 1/6，六轮无数值警告或手指越限终止。蓝积木种子 1 出现 245 个有机械臂接触的控制周期。配对记录在 `benchmarks/packaging-paired-calibrated-material-holdout.json` 和 `benchmarks/packaging-material-calibrated-setxf/summary.json`。

对蓝积木种子 5 的 400 步重复评估添加实际接触力记录。第 27 步有且仅有一个目标采样检测到不同手指实测相向承力，食指与中指组合的法线点积约 −0.743，成对法向载荷的较小值最高约 8.30 N。第 28 步两指已失去物件接触，积木与盒心仍相距数十厘米。含接触力记录的实际评估与原配对试验完全一致。报告和第 27 步积分态位于 `benchmarks/packaging-material-calibrated-setxf/` 的 `blue-block-seed5-physical-grasp-trace.json` 和 `blue-block-seed5-grip-checkpoint.npz`。

纸板显色校准后的完整评估已结束。各物件的种子 1–5、初始摆放、公开 HDF5 和现有 SetXF 第 3000 轮权重与旧材质批次逐一配对，仍为每回合 400 动作。

| 物件组 | 抓起后脱手 | 入盒 | 合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: |
| 训练物件 | 3/30 | 0/30 | 6/30 | 0/30 |
| 未见物件 | 5/45 | 0/45 | 10/45 | 0/45 |
| 合计 | 8/75 | 0/75 | 16/75 | 0/75 |

与旧材质配对比较，右手接触物件的回合由 27 增至 37，抓起后脱手由 6 增至 8，有机械臂接触的回合由 20 降至 11，合盖由 22 降至 16。75 轮无物理警告或越限终止。实际评估仍未产生入盒。汇总、逐回合引用和合并脚本位于数据盘 `benchmarks/packaging-material-calibrated-setxf/` 的 `all-objects-summary.json`、`summarize-material-rollouts.py`。

蓝积木的种子 5 另做了从实际 SetXF 抓持状态开始的物理接续。保留原 27 个策略动作，后续用有限力腕位与指关节目标调整夹持，右手再抬升约 96.6 mm。左手沿已抓住的箱壁向工作区中心拖盒，右手在约 5.4 s 的移盒、松手及撤离阶段始终持物。盒体实测平移约 164.2 mm，移盒过程中最接近中心约 2.7 mm，撤手后距中心 26.2 mm。76 步标准 EGL 复放得到相同物理末态，无机械臂触碰、双手碰撞或物理警告。原策略前 27 步有约 15.36 mm 的采样同手最大穿透，复放继承了该过程。HDF5 与标准报告在 `benchmarks/packaging-material-calibrated-setxf/` 的 `blue-block-seed5-center-and-lift-{reference.hdf5,egl.json}`。这组接续使用仿真状态选择动作。让左臂短程撤开并绕右末关节的实际轴调整腕姿后，另一段 **115 步**标准 EGL 回放把积木运至 `[0.5444, -0.0345, 0.1315] m`，终态仍有两个不同手指相向承力，较小的接触力为 7.69 N，右末关节限位余量 0.323 rad，机械臂和双手间碰撞为零。积木包络距完全跨入盒口仍差约 63.7 mm。复放与逐步试算见同目录 `blue-block-seed5-centered-transfer-{reference.hdf5,egl.json}` 和 `blue-block-seed5-joint7-reclamp-1-12mm.json`。 随后在上缘脱手前调整食指和闲置无名指，闭环积分中的最近包络由距盒口约 63.7 mm 降至 10.2 mm。接续向盒内送腕并松开食指、中指，蓝积木在真实重力和盒底碰撞下平躺并稳定。**223 步**标准 EGL 回放得到 `object_released=true`、`placed_in_centered_box=true`、`object_in_box=true`，机械臂、双手间碰撞和物理警告均为零。轨迹、报告及最终积分状态在同目录的 `blue-block-seed5-centered-placement-{reference.hdf5,egl.json,egl.npz}`。

### 留出物件的动作回放

九个未见物件可以用同一个 MuJoCo 入口执行预先记录的控制目标。动作 HDF5 使用 `data/demo_0`，可以用 `--first-seed` 指定与策略评估一致的复位摆放。公开视频仍是物件外观的参考。用蓝杯验证了默认和种子 1 的 3 步原生动作，均产生 `10000 × 6` EGL 点云，无物理警告。该回放验证动作入口，蓝杯完整任务仍需生成并验证。

```bash
replay=/data/dexcap-repro/benchmarks/packaging-unseen-replay
python -m simulation.rollout "$replay/blue-mug-native-entry-reference.hdf5" \
  --task packaging --object blue-mug --first-seed 1 --output "$replay/blue-mug-native-seed1-replay.json"
```

**蓝杯杯柄抓持。** 左手先移盒居中，右中指与拇指再相向夹住杯柄。标准 EGL 入口从默认摆放执行 **6,539 步**，末态检测到持杯，杯体根位置上升约 **18.9 mm**，杯已脱离桌面。两个接触载荷约 1.61 N 和 1.47 N，接触法向点积约 −0.915。机械臂、双手和同手碰撞均为零，无物理警告。续段一度把杯体再抬高并向纸盒方向移动，随后从夹口滚出。抓持 HDF5 和独立 EGL 报告位于数据盘 `benchmarks/packaging-unseen-replay/blue-mug/` 的 `blue-mug-centered-handle-grasp-*`，续段及接触记录位于 `mug-follow-angular-*`。

进一步以食指入杯、拇指夹外壁的方式抓杯沿。独立的原生控制段在纸盒居中后干净抬升杯体 **85.52 mm**，随后直立放入盒中并释放，三类碰撞与物理警告为零。直立杯在盒坐标的最高点为 **111.37 mm**。当前刚体可闭合纸盒的内高是 **75 mm**。杯身模型的十边形凸包给出至少 **90.35 mm** 的最小投影高度，完全闭盖在当前尺寸下不可行。把盖角放宽到当前 6° 判据进行的几何搜索，找到的最佳候选仍贯入约 7.06 mm。原生抓取与放置在蓝杯目录的 `rim/`，几何测量在 `source/mug-lid-clearance.json`。作者视频约 30.02 s 显示杯子直立在盒内。43.95 s 的近末帧侧视角中盒盖仍明显倾斜，蓝杯从缝隙可见，另一侧的高视角遮住了该缝隙。抽帧 `mug-30.jpg` 和 `mug-43.95.jpg` 保存于同一 `source/` 目录。 [作者未见物件视频](https://dex-cap.github.io/assets/normal_packaging_unseenobj2.mp4)

红苹果原用最长轴竖直的椭球。在标准双臂完整场景中，4.5 s 内双手无接触时，默认复位最大漂移约 **363.52 mm**。加入 1° 扰动的对照漂移约 166.21 mm。现在用保留原 79 × 79 × 87 mm 包络的曲面和约 21.2 mm 平底接触面，表示直立支撑。尺寸和总质量继续采用项目估计。新版默认复位最大漂移约 **0.003 mm**，1° 扰动约 **1.26 mm**，无手接触或物理警告。平底尺寸由现有包络和静置力学确定，尚无公开苹果原始深度可直接量测。完整双臂对照在 `benchmarks/packaging-unseen-replay/red-apple/` 的 `apple-production-rest.json` 和 `apple-flat-base-production-rest.json`。

两份实际释放后的苹果都由盒底承托，但在三维接触下仍持续滚动。相同积分态各续跑 9 s，末段 100 个样本分别只有 1 和 0 个满足入盒静置。为苹果曲面和盒底启用与绿梨相同的六维接触后，两份均为 100/100。数值采用模型已有的滚动阻力，接触对局限于苹果曲面和盒底。结果见同目录 `compare-apple-box-rolling-friction.json`。 [MuJoCo 接触维数](https://mujoco.readthedocs.io/en/3.6.0/computation/index.html#contact)

修正苹果静置支撑后，用当前 SetXF 权重运行各 400 步的种子 1–5。三个回合出现右手接触，其中种子 5 曾抓起后脱手。全组入盒和整体成功均为 0/5，无物理警告。报告在同目录 `red-apple-flat-base-policy-seed-{1..5}.json`。

**红苹果完整居中回放。** 从未见物件默认摆放独立执行 **9,066 步**。纸盒移动约 **91.20 mm**，右手相向夹持苹果并实际释放在居中盒底，随后右手稳盒、左手合盖。两手连续同时承力 **0.986 s**。末态盖板约 **176.25°**，满足现行 6° 合盖容差，`sim_success=true`。机械臂和双手间碰撞均为零，同手采样最大穿透约 0.53 mm，无物理警告。标准 EGL 复放和接触审计为同目录 `red-apple-centered-complete-egl{,-contacts}.json`。

```bash
apple=/data/dexcap-repro/benchmarks/packaging-unseen-replay/red-apple
python -m simulation.rollout "$apple/red-apple-centered-complete-reference.hdf5" \
  --task packaging --object red-apple --output "$apple/red-apple-centered-complete-replay.json"
```

**耳塞罐完整居中回放。** 从未见物件的默认摆放执行 **9,132 步**，左手先把盒移动约 **91.51 mm**，右手相向夹起罐子并在保持夹持时转腕。选择在实际箱口上方仍保持夹持、罐轴已接近水平的状态松手。罐子在重力和碰撞下横卧在盒底，罐轴离水平约 **0.07°**。右手稳盒、左手合盖时连续同时承力 **1.038 s**。末态盖板约 **180.00°**，`sim_success=true`。全程机械臂接触和双手碰撞为零，同手采样最大穿透约 0.53 mm，无物理警告。该参考采用现有 70 mm 直径、118 mm 高的圆柱近似。标准 EGL 复放和逐子步审计见数据目录的 `earplug-jar-centered-complete-egl{,-contacts}.json`。

```bash
jar=/data/dexcap-repro/benchmarks/packaging-unseen-replay/earplug-jar
python -m simulation.rollout "$jar/earplug-jar-centered-complete-reference.hdf5" \
  --task packaging --object earplug-jar --output "$jar/earplug-jar-centered-complete-replay.json"
```

**积木组完整居中回放。** 从默认摆放独立执行 **9,244 步**。左手移盒居中，右中指与拇指夹住黄色积木侧面，实测离桌抬升约 **68.7 mm**。整组积木越过盒沿，在盒内释放并由盒底承托。右手扶盒、左手合盖，逐 2 ms 审计记录双手连续承力 **1.016 s**。盒盖约 **180.00°**，`object_released`、`object_in_box`、`placed_in_centered_box` 和 `sim_success` 均为真。机械臂接触和双手间穿透均为零，采样到的最大同手穿透约 0.53 mm，无物理警告。数据盘 `benchmarks/packaging-unseen-replay/stacked-bricks/` 保存物理分段、标准参考和接触报告。

```bash
bricks=/data/dexcap-repro/benchmarks/packaging-unseen-replay/stacked-bricks
python -m simulation.rollout "$bricks/stacked-bricks-centered-complete-reference.hdf5" \
  --task packaging --object stacked-bricks --output "$bricks/stacked-bricks-centered-complete-replay.json"
```

**狮子摆件完整居中回放。** 右食指和拇指夹持躯干，将物件越过已居中的盒沿，在空中旋腕把物件转到侧躺方向。按实测腕位重新对齐后松手，物件在盒底静置，竖向包络上界约 47.5 mm。独立 EGL 复放 **9,249 步**，通过抓起后释放、居中入盒和双手闭盖。双手连续承力 **1.330 s**，盖板约 **180.00°**。机械臂接触与双手间穿透均为零，最大同手采样穿透约 0.53 mm，无求解器警告。数据盘 `benchmarks/packaging-unseen-replay/lion/` 下的 `lion-centered-complete-reference.hdf5`、标准报告 `lion-centered-complete-egl.json` 和接触审计 `lion-centered-complete-egl-contacts.json` 保留实测记录。

**长颈鹿摆件入盒。** 按实测已居中纸盒与摆放重新拟合右腕入场，使用原生腕位目标跨越合适的关节路径。食指和拇指相向夹持躯干，物件离桌抬升约 39.0 mm，随后搬入居中纸盒、松手并由箱底支撑。独立的分段控制验证 `object_released=true`、`object_in_box=true`、`placed_in_centered_box=true`，未出现机械臂、双手间、同手碰撞或求解器警告。落稳后箱坐标最高约 154.2 mm，仍妨碍盖板闭合。数据盘 `benchmarks/packaging-unseen-replay/giraffe/` 下的 `giraffe-centered-lift-4-1-0-1-0.*` 与 `giraffe-side-carry-4-1-0-1-0-3-0.*` 为已通过的分段。 左手实测压住一面外壁，再由右手推头颈，物体仍会带动箱体移动，未得到可合盖的稳定盒内姿态。物体与纸盒保留原有自由度，见 `giraffe-wall-braced-tip.{npz,json}`。

**胶带座接近与盒盖运动。** 由右手从桌面受支撑的状态推近胶带座，四条路线均有真实位移。最初一条约 132 mm 路线使纸盒移动，盖板自约 −0.15° 转至约 105°，再抓持时自由盖板已转到约 180°。随后的两指抓持、搬运和释放把胶带座留在盖板上，四个盖板触点合计约 1.18 N。将桌面接近终点留在盒壁外后，四条物理路线分别前移约 53–92 mm，均保持盖板打开、无盒壁接触。开盖状态的反侧再抓仍在末端腕位误差及单侧接触处失败。实测动作、状态及接触读数均在数据盘 `benchmarks/packaging-unseen-replay/tape-dispenser/`，包括 `dispenser-stage-near-box-0.*`、`dispenser-stage-near-box-4..7` 和 `dispenser-drop-stage-0-4-0-10-0-0-0-14-1-support.log`。 短侧重抓另得到四组两指承力约 2.4–3.3 N 的状态，指垫接触法向点积为 0.64–0.89，桌面仍承力约 16.6–19.7 N。该路线把胶带座压在桌上，未形成离桌夹持。见同目录 `dispenser-short-bridge-lift-4-4-1-10-*.json`。

用左手无名指的实际接触保持盖板后，原模式 1 的右手推近、松开和撤离完成 955 个 50 ms 控制周期，盖角维持在约 −2.13 至 −0.15°，胶带座末态约为 `[0.469, -0.093] m`。撤离段在右手对盖板施加约 4.40 N 时，左手对向承力约 1.75 N，盖角仍约 −0.86°。该回放未出现机械臂、双手间接触或物理警告，起始动作已有约 2.89 mm 同手穿透。证据为同目录 `left-lid-brace-case-0-mode-1.{npz,json}` 和 `left-lid-brace-case-0-right-contact-open.{npz,json}`，尚未完成重抓、入盒与合盖。 从场景初态拼接前 6,151 步移盒动作和这段撑盖动作，独立 EGL 回放执行 7,106 步，复现相同末态角度、位置和整段撑盖角度范围，生成 `10000 × 6` 点云。标准 HDF5 与结果见 `dispenser-braced-open-reference.hdf5` 和 `dispenser-braced-open-egl{,-lid}.json`。

撑盖后续用以前验证的右手抓型，按实测物体起点修正抓取位置。追加 503 个实际目标后，食指和拇指持续相向抓持 28 个控制周期，约 2.43、2.37 N，桌面不再承力。物体离桌约 59 mm，盒盖终值 −0.36°。独立标准 EGL 从初态执行 7,609 步，`object_grasped=true`，复现胶带座末态高度约 56.5 mm、盖角 −0.36°，无机械臂、双手间接触、越限或求解器警告。前缀已有约 2.89 mm 同手穿透。第一次继续过盒口时右无名指与左拇指相碰，因此仍待验证搬运和放置。工作盘同目录下的 `dispenser-braced-lift-reference.hdf5`、`dispenser-braced-lift-egl{,-lid}.json`、`braced-lift-stage0-case0.{npz,json}` 保存了结果。

接着先沿盒坐标 y 偏移避开左手，右手通过盒盖摆扫范围后解除撑盖。左手再移开 25 mm 并升高 25 mm，给右手让出回到盒中心的走廊。追加 601 个实际目标后右手在盒内松开。四处盒底接触合计约 1.177 N。独立标准 EGL 从初态执行 8,210 步，复现抓取后释放、在居中纸盒内落稳，生成 `10000 × 6` 点云，无臂及双手间接触、越限或物理警告。盒盖仍开且物体包络最高约 144.9 mm，需要在盒内侧放后才能尝试合盖。见 `dispenser-braced-in-box-reference.hdf5`、`dispenser-braced-in-box-egl{,-lid}.json` 和 `braced-lift-lateral-route2.{npz,json}`。

盒内重抓在盒底承托时把厚轴的平面方向误差从约 68.5° 降至 4.53°。反向翻转时实测第七关节到达上限，腕位跟踪偏差约 28 mm，随后物体短暂失去盒底承托。以实际关节态求替代分支，能覆盖 0–12° 倾斜的分支起点需改变约 0.224 rad。保持实测腕位让原 OSC 回中，三周期内第七关节仅回退约 1.2 mrad，已有一侧指垫失载。见同目录 `dispenser-in-box-flat-route6.json`、`dispenser-flat-elbow-diagnostic.json` 和 `dispenser-flat-elbow-nullspace.json`。随后在真实盒底支撑上缓慢释放，换右中指从红壳高处沿盒坐标 −x 推动，峰值约 1.67 N。物体上界由约 153.4 降至 115.5 mm，末态仍位于盒内，盒底承力约 1.06 N，盒移动约 0.085 mm，无臂或双手间接触和物理警告。红壳随后离开推线，物体仍高于可闭盖位置。接续工件及原生动作在 `dispenser-flat-push-route1.{npz,json}`。

**弹簧夹活动关节。** 复用原有外形和 120 g 总质量。销轴和一侧夹臂随自由物体主体运动，另一侧通过一个相对铰链连接。夹臂和夹嘴仍与桌面、手指和箱体发生碰撞。全部九块实体表面参与抓持和盒内包络判断，铰链运动也参与静置判据。最大开角 44° 参照同尺寸夹具的约 65 mm 开口选取，预紧、0.30 N·m/rad 弹簧刚度和 0.01 N·m·s/rad 阻尼为估计值。两手柄分别施加 5 N 相向载荷，实测最大开角约 **24.93°**，卸载后回到距零位约 **0.022°**，无物理警告。隔离物理试验见数据盘 `benchmarks/packaging-spring-clamp/measure-spring-clamp-handle-response.json`。 [同尺寸夹具参考](https://ronixtools.com/en/product/6-inch-spring-clamp/) · [MuJoCo 铰链与弹簧](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-joint)

弹簧夹的 580 步独立标准 EGL 回放验证了右手张开下探、相向闭合和抬升。峰值质心抬升 **45.04 mm**，其中 4 个动作实际满足抓持，保持抓持时最高 **41.56 mm**。随后夹具脱手，机械臂、双手间、同手碰撞和求解器警告均为零。动作 `spring-clamp-approach-reference.hdf5` 和报告 `spring-clamp-approach-{egl,contact-audit}.json` 在上述数据目录。当前活动关节模型使用相同 SetXF 权重和种子 1–5 执行每轮 400 步，种子 5 曾抓起后脱手，完整入盒为 0/5。逐轮报告为 `spring-clamp-final-policy-seed-{1..5}.json`。

### 包装整轮计分

包装整轮要求放下物件时，工作区中心 `[0.4, 0.0] m` 落在按实际朝向计算的纸盒内底投影内。论文规定先把盒子移到桌子中间，未给出距离阈值，本项目用实际内底尺寸定义该区域。放置后还要求在同一 2 ms 物理子步里，一只手接触箱体，另一只手接触正向闭合的盒盖或挂片，两处法向载荷均超过 0.05 N。最后物件必须仍在盒内、盖板已合上，且物件包络位于实际闭合盖板内表面下方。用盖板实时姿态投影刚体几何或柔性表面顶点，容许至多 1 mm 的接触贯入。独立字段继续记录入盒、合盖角度和双手承力。 [论文包装流程](https://arxiv.org/html/2403.07788v2#A1.SS6)

咖喱盒在持物时可以转到接近平放，实际松手后翻回立放。旧 9,193 步复放的盖板角度到位，但盒体横截盖板内表面约 **39.90 mm**，该状态不计为整轮成功。把同一盖板几何判据应用到已保存的蓝积木、绿罐、绿梨、卷尺、红苹果和耳塞罐的实际末态，均满足盖板净空条件。红苹果约 0.50 mm 的接触贯入处于上述容差内。测量和回放在数据盘 `benchmarks/packaging-unseen-replay/golden-curry/` 的 `audit-closed-lid-terminal-clearance.json` 和 `golden-curry-centered-complete-egl.json`，后者保留旧计分，仅用于复查。

同一咖喱盒末态的几何距离为 −39.90 mm，MuJoCo 零检测边距的盒形接触没有返回该对接触点。将主盖板的 `margin` 与 `gap` 同设为盖板半厚 1.5 mm 后，同一状态得到四个有效接触点。两实体仍有 0.5 mm 净空时，检测点不产生求解器约束。生产模型用此配置从初态复放八条轨迹，咖喱盒实际挡住盖板，最终约 **148.60°**，判定未成功。红银包装、蓝积木、绿罐、绿梨、卷尺、红苹果和耳塞罐全部重新通过，无机械臂接触、双手碰撞或物理警告。数据盘 `benchmarks/packaging-lid-contact-corrected/summary.json` 汇总标准结果，`replay-lid-contact-corrected.sh` 保存各条命令。原接触分析见咖喱盒目录的 `collision-debug-agent/diagnosis.json`。 [MuJoCo 接触边距](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-geom)

从咖喱盒实际立放的积分态继续，右手先推倒盒体，再沿长边解开卡在箱壁上的边角，最后从露出的端面向内下方推。盒体在盒底自由静置，距平放约 0.010°，竖向包络为 1.37–31.37 mm。合并原前缀与这些手指动作后，标准 EGL 入口从默认初态复放 **10,857 步**，实际入盒、居中及双手闭盖均通过，机械臂接触和双手间碰撞均为零，没有物理警告。逐 2 ms 审计记录双手连续承力 1.044 s、累计 1.278 s，独立复放的末态与生成轨迹的积分末态一致。末态 `object_grasped=false` 且 `object_released=true`，后者要求该回合曾相向抓起物体并脱离桌面。咖喱盒目录中的 `golden-curry-flat-complete-reference.hdf5`、`golden-curry-flat-complete-egl.json` 及 `golden-curry-flat-complete-egl-contacts.json` 保存完整轨迹和报告。

在加入移盒居中条件前，以相同 MuJoCo 和 EGL 入口复放四条动作。蓝积木的 1,977 步和红纸盒的 8,364 步双手参考均计为整轮成功。蓝积木的 1,237 步及绿罐的 1,322 步单手参考均入盒并合盖，双手闭盖字段为假。四次复放均无物理警告。报告在数据盘 `benchmarks/packaging-bimanual-score/`。上文 75 轮 SetXF 结果全部未入盒，因此使用新判据时完整成功数仍为 0。

按加入居中条件前的计分，在绿罐实测入盒后，让右食指顶住箱体低处，左手带动盖板。盖板转到约 62.03° 后，右手向外斜上撤离，再让左手跨过盖板的闭合中点。这段撤离避免了早期试验在 136–144° 发生的机械臂接触。用标准 EGL 入口从默认初态复放 **2,076 步**，罐子仍在盒内，盒盖闭合，`bimanual_lid_closure` 和 `sim_success` 均为真。每 2 ms 的接触审计记录双手连续承力最长 0.384 s、累计 1.484 s，右食指峰值 3.58 N、左手对盖板峰值 8.43 N，右手未碰盖。没有机械臂、双手间或同手碰撞和物理警告。这段绿罐参考用默认外盒摆放，盒体仅平移约 0.47 mm。数据盘 `benchmarks/packaging-green-current/corrected-height/` 保存了 `green-can-bimanual-reference.hdf5`、标准报告 `green-can-bimanual-egl.json` 和逐物理步汇总 `green-can-bimanual-egl-contacts.json`。

```bash
green=/data/dexcap-repro/benchmarks/packaging-green-current/corrected-height
python -m simulation.rollout "$green/green-can-bimanual-reference.hdf5" \
  --task packaging --object green-can --output "$green/green-can-bimanual-replay.json"
```

用现行判据再从初态复放三条双手合盖参考。8,364 步红纸盒在盒底投影覆盖工作区中心时放置，最终 `sim_success=true`。固定盒位的 1,977 步蓝积木和 2,076 步绿罐均通过入盒和双手合盖子项，`placed_in_centered_box=false`。数据盘 `benchmarks/packaging-center-placement-score/` 保存三个报告。

**种子 5 蓝积木完整回放。** 从上述 223 步居中入盒状态接续，把红盒的双手合盖动作变换到当前实测箱体坐标。右食指先稳住箱壁，左手拉动折舌；盖板到约 62.62° 时撤开右手，左手继续完成合盖。**1,586 步**标准 EGL 回放从种子 5 初态复现整轮成功，机械臂和双手间碰撞为零，无物理警告。逐 2 ms 审计在盖板沿闭合方向运动时记录 721 个双手承力子步，连续最长 601 个，即 1.202 s；右手箱壁与左手盖板法向载荷峰值分别约 2.59 N、6.60 N。前 27 步采自 SetXF，其余为仿真辅助选择的执行器目标。记录保留了前 27 步约 15.36 mm 的最大同手穿透。报告与接触审计为下方数据目录的 `blue-block-seed5-centered-complete-egl{,-contacts}.json`。

```bash
ref=/data/dexcap-repro/benchmarks/packaging-material-calibrated-setxf
python -m simulation.rollout "$ref/blue-block-seed5-centered-complete-reference.hdf5" \
  --task packaging --object blue-block --first-seed 5 --output "$ref/blue-block-seed5-centered-complete-replay.json"
```

**绿罐完整居中回放。** 从绿罐默认摆放执行左手移盒并撤离，外盒由约 `[0.482, 0.115] m` 移到 `[0.3967, 0.08294] m`，内底覆盖工作区中心。随后右手从原位置抓起绿罐，按已移动的盒体坐标转运并竖放。在实际 Panda 及 LEAP 网格上重新求右食指的低位箱壁支撑，右手在盖板约 58.64° 时离开闭盖路径，左手完成合盖。**9,182 步**标准 EGL 从默认初态复放，`placed_in_centered_box`、`object_in_box`、`bimanual_lid_closure` 和 `sim_success` 全部为真。连续双手承力最长 517 个物理子步，约 1.034 s。机械臂及双手间碰撞为零，最大采样同手穿透约 0.53 mm，无物理警告。报告位于 `benchmarks/packaging-green-current/corrected-height/green-can-centered-complete-egl{,-contacts}.json`。

```bash
green=/data/dexcap-repro/benchmarks/packaging-green-current/corrected-height
python -m simulation.rollout "$green/green-can-centered-complete-reference.hdf5" \
  --task packaging --object green-can --output "$green/green-can-centered-complete-replay.json"
```

**卷尺完整居中回放。** 采用 `demo_175` 的发布摆放。用食指与拇指的真实相向接触夹住圆周，转运到居中纸盒上方再张手，选取由盒底承托、桌面无载荷的放置状态继续合盖。**9,023 步**标准 EGL 从初态独立复放，纸盒实测移动 **91.48 mm**，释放、居中入盒、合盖和整轮计分均通过。合盖时两手连续同时承力 **1.048 s**，机械臂接触和双手间碰撞均为零，同手采样最大穿透约 0.53 mm，无物理警告。本次使用当前圆柱卷尺模型。完整报告和逐 2 ms 子步接触审计为 `tape-measure-centered-complete-egl{,-contacts}.json`。

```bash
tape=/data/dexcap-repro/benchmarks/packaging-tape-measure-complete
python -m simulation.rollout "$tape/tape-measure-centered-complete-reference.hdf5" \
  --task packaging --object tape-measure --output "$tape/tape-measure-centered-complete-replay.json"
```

随后按三份发布示范中的卷尺点云补上深色外壳、黄色环带和偏置蓝标。黄边和蓝标作为无质量、无碰撞的渲染几何。对 `demo_175`、`demo_176` 和 `demo_180` 共 30 组三帧输入保持摄像机、腕位、随机种子和策略权重配对，卷尺区域的平均绝对 RGB 差中位数由 **0.335 降至 0.116**。相对录制动作的右、左腕误差中位数由 **9.93、9.67 mm** 降至 **6.07、6.46 mm**。壳体厚度仍使用圆柱近似，公开点云没有充分覆盖其底面用于独立标定。采用新外观的 `tape-measure-centered-complete-layered{,-contacts}.json` 复放了同一完整轨迹，整段物理结果、末态和逐子步受力汇总完全相同。数据盘同目录的 `tape-case-offset-rgb-paired-demo_{175,176,180}.json` 保存配对测量。 [MuJoCo 几何体外观与惯量](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-geom)

外观更新后，现有 SetXF 权重的五个种子仍均未抓起或入盒。每轮闭环 400 步，只有种子 5 观察到右手接触。逐轮结果为同目录的 `tape-layered-policy-seed-{1..5}.json`。

**黄布边缘剥离参考。** 从 `demo_250` 对应摆放执行 160 个目标和 200 个控制周期。右手抬起布边，布料质心最高上升约 **14.36 mm**，远侧仍由桌面支撑，拇指与中指受力法向变为同向。完整抓持及入盒均未通过。标准复放无机械臂接触、双手碰撞、同手穿透、手指越限或物理警告。HDF5、逐步物理数据和报告见数据盘 `benchmarks/packaging-yellow-cloth-complete/sponge-guided/` 的 `yellow-cloth-edge-peel-clean-*`。

## Ball Collecting

`python -m simulation.rollout --task ball-collecting` 使用双 Franka 与 LEAP、右侧的球和左侧的自由线篮。两处各在论文规定的 40 × 30 cm 区域采样。直径 67 mm、质量 57.7 g 的球用网球标准范围中值。篮子约 230 × 150 × 130 mm，按照图中球径比例估计，用 50 根可碰撞圆柱组成交叉底网和侧壁，保留开口和线间空隙。篮重和表面摩擦为假设。 [论文图 5 与任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6) · [ITF 网球规格](https://www.itftennis.com/media/7221/2026-rules-of-tennis-english.pdf)

判据要求右手形成相向接触、球实际抬离桌面，再释放到篮中。进篮通过完整球体位置、篮底铁丝接触和低速度判定。物理试验验证了球经重力落入篮底后能够稳定承托。在默认球和篮子位置，一条 252 步脚本轨迹通过标准回放入口完成了从空手接近、夹球到球由篮底承托的全过程。右手用三处真实手部表面建立抓持，球相对起点最高上升约 169 mm，转运时接触篮沿并落入篮中。全程没有机械臂碰撞。结果只覆盖默认摆放和这条脚本动作，公开数据仍缺该任务的作者轨迹与权重。生成脚本及逐步接触记录见 `runs/ball-leap-tabletop-reference.{py,json}`。

```bash
python -m simulation.rollout data/derived/ball-collecting-scripted-reference.hdf5 \
  --task ball-collecting --output runs/ball-leap-tabletop-rollout.json
```

使用分块掌心后，旧 252 步参考的球会在桌面上滚出夹口。重新拟合右手的食指端、中指远端指节与拇指端，分担滚转力矩，恢复了默认摆放的完整捡球。实测球心从约 3 mm 升到约 231 mm，随后搬至自由线篮上方并松手。球最终由两根篮底铁丝承托。265 动作标准回放确认抬球、释放、进篮，`sim_success=true`，机械臂碰撞计数为 0。逐步记录在 `runs/ball-collecting-4-7-16-basket-deposit.json`，标准回放汇总在 `runs/ball-collecting-4-7-16-reference-rollout.json`。

```bash
python -m simulation.rollout data/derived/ball-collecting-compound-palm-reference.hdf5 \
  --task ball-collecting --output runs/ball-collecting-compound-palm-rollout.json
```

球掉下桌面后由估计在桌面下 0.75 m 的地面承托。


在现行 LEAP 参数下，保留 463 步实际入盒前缀，再抬高和调整左手指形，从实测纸盒位姿重新接近折舌，得到 535 动作的完整回放。红银包装留在盒内，盒盖闭合，`sim_success=true`。峰值盖角 181.22°，左手与盖接触 137 次，机械臂环境碰撞为零，双手之间有 69 个穿透采样，最深约 4.55 mm。数据在 `data/derived/packaging-red-carton-scripted-lid-retarget-6dd459f.hdf5` 及 `runs/packaging-red-combined-standard-gated.json`。

碰撞回放将上述 69 个采样定位到入盒后的关盖弧线。右手先真实上抬 90 mm 并撤至安全位，左手再接触折舌和关盖。610 步回放确认物体留在盒内、盒盖闭合、`sim_success=true`，双手碰撞和机械臂环境碰撞均为零，无物理警告。纸盒约移动 0.79 mm。记录见 `runs/packaging-red-scripted-lid-right-retreat-6dd459f-rollout.json`。

另一物体绿梨（`demo_100`）的真实几何能拟合出台面净空为正的中指和拇指相向目标。原直接高位接近在张指前记录到绿梨移动约 103 mm，后续未抓起。红银包装的抓取轨迹也不能直接移植到绿梨。证据见 `runs/packaging-green-pear-{radial-pad-fit,retracted-transit}-9003771.json`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-scripted-lid-right-retreat-6dd459f.hdf5   --task packaging --output runs/packaging-red-scripted-lid-right-retreat-6dd459f-rollout.json
```

## Scissor Cutting

`python -m simulation.rollout --task scissor-cutting` 使用固定分隔托盘、两个可碰撞的剪刀刚体、被动铰链、与铰链实际相连的镂空手柄和粉色纸带。120 × 20 mm 的纸带由 10 × 5 个可碰撞单元和 85 条具有弯曲阻力的可断裂连接构成。剪刀闭合时，两片刀刃都必须在刀锋交汇点附近压到同一条网格边才会断开该边。报告的切口按断开的边长累加，分辨率由 4 mm 和 12 mm 的网格边决定。机器人有效抓持时累计切口超过 3 mm 为论文的任务判据。初始布局参考图 5，几何和材料参数仍为估计。 [论文图 5 与任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6) · [MuJoCo 焊接约束](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#equality-weld)

偏离纸带中部的夹具试验中，旧单排接缝的切口为 0。新模型记录了 3 次局部断裂事件，断开边长累计 40 mm。纸带移到刀口之外时为 0。在默认摆放下，纸带由托盘底和侧壁承托，连续静置 25 s，无物理警告。标准双臂入口执行 8 步并输出 `10000 × 6` 点云。记录见 `runs/scissor-paper-grid-cut-fixture.json`、`runs/scissor-paper-grid-settle-1-.120.json` 和 `runs/scissor-paper-grid-production-entry.json`。

在包含双臂和摄像的正式 2 ms 场景中，对铰链施加力矩后实际断开一个 4 mm 接缝段。把纸带放到刀口之外时切口保持为 0。这个试验验证机构和子步回调，整段双手抓剪刀、提纸和切割仍缺官方轨迹。记录见 `runs/scissor-production-hook-probe.{py,json}`。


使用旧 44 × 22 mm 柄孔的接触夹具时，真实 LEAP 触碰手柄使剪刀从 0.450 rad 闭合到 0.199 rad，峰值手柄接触力 14.6 N。接触只维持三个物理子步，剪刀随后脱手。旧柄孔无法可靠穿过拇指网格，轨迹存于 `runs/scissor_leap_aligned_fixture.{py,json}`。

按 LEAP 网格改为 50 × 40 mm 的双指孔及 40 × 42 mm 的拇指孔，并补齐两侧连到被动铰链的柄臂。保守包络检查得到指面与孔边的总净余量约 5.3–6.7 mm，闭合位置两环仍分离约 14.6 mm。新版机构试验仍能在实际刀刃接触下切开 4 mm 接缝。测量见 `runs/scissor_handle_fit_check.json`，新版预插入试验验证了两指能在两个孔内保持正净空。物理执行中，接触只短暂支撑了剪刀，两个柄孔未同时承载。默认桌面姿态的尝试还出现约 25.1 mm 的手与桌面或托盘穿透。结果见 `runs/scissor_leap_new_handle_fixture.json` 和 `runs/scissor_leap_tabletop_reach.json`。随后用右手食指从剪刀轴附近托住 0.62 s，实际抬升约 25.6 mm，活动把手与托盘隔板的峰值净空增加约 117 mm。按抬升后的姿态拟合左手，食指和拇指分别有约 2.6 mm 和 12.1 mm 的柄孔净空。固定柄贴着托盘边墙，左食指连杆的拟合插入位置与墙重叠约 14 mm，尚未执行插入与交接。记录在 `runs/scissor_right_shank_tilt.{py,json}` 和 `runs/scissor_bimanual_support_insert.{py,json}`。

复核图 5(e) 的首帧可见，剪刀两柄伸出托盘短端，刀片在托盘内。场景据此保留四面外墙，沿托盘长边增加贯通隔板，并在剪刀所在的前排加入短隔板。默认剪刀以有真实托盘接触的校准姿态静置。当时纸带默认平放，尝试让旧版柔性纸带靠墙立放时，约 0.1–0.2 s 内便折落。记录在 `runs/scissor_source_layout_ab.{py,json}` 和 `runs/scissor_source_upright_paper.{py,json}`。 [图 5(e)](https://arxiv.org/html/2403.07788v2/tasks_v5.png)

在正式布局中，剪刀在 0.6 s 悬停准备期间仅移动约 0.13 mm。继续插指时拇指首先接触活动柄孔上缘，随后剪刀滑移约 82 mm。两个柄孔各有接触样本，未在同一物理子步同时承载。正式场景的腕误差、柄孔和托盘接触见 `runs/scissor_production_left_insert.{py,json}`。

此前的平放纸带曾移到前格内。原摆放与长隔板重叠 0.8 mm，首个子步产生 40 个接触。新摆放净空为 27 mm，没有首步隔板接触。另一次声明初始贴近端墙的右手试验中，食指触到纸带、拇指未接触，纸带落回托盘，接缝保持完整。记录在 `runs/scissor_right_paper_pickup.{py,json}`。

在改正后的正式场景中，按约 1 mm 的插入间隔控制左腕，测得食指尖穿过固定柄孔平面约 7.1 mm，拇指尖穿过活动柄孔平面约 4.3 mm。两处网格的截面均在各自孔内，剪刀位移小于 0.4 mm。该阶段没有手与剪刀受力接触，中指也未入孔。记录在 `runs/scissor_left_two_pad_insertion_clear_tray.{py,json}`。

同一插入动作已导出 71 个标准动作，`simulation.rollout` 在 `pro1` 上输出 `10000 × 6` 点云，未记录机械臂碰撞。动作到两指穿孔为止。

```bash
python -m simulation.rollout data/derived/scissor-left-through-loops-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-left-through-loops-rollout.json
```

后续分步加载让两个柄孔短暂在同一 2 ms 子步受力。锁定该姿态和直接向上提拉均失去双柄接触，剪刀未抬起。右手在前格内按压并拖动平放纸带时，只维持了 4 ms 指尖接触，纸带没有移到墙边。记录在 `runs/scissor_left_two_pad_{contact_latch,immediate_lift}.json` 和 `runs/scissor_right_paper_edge_present.json`。

改用从活动柄下方托举。自由剪刀从默认姿态开始，只有机器人以无接触预抓位初始化。实测左拇指能让整把剪刀离开托盘和桌面约 0.314 s。有效区间的腕误差低于 5 mm，随后摆动导致脱手。截取至脱手前的 499 步经标准入口重放，剪刀轴最大升高约 116.4 mm，机械臂碰撞计数为 0。此段验证的是单柄提拉，尚未稳定保持两个柄孔并操作开合。

```bash
python -m simulation.rollout data/derived/scissor-left-thumb-lift-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-left-thumb-lift-rollout.json
```

逐子步接触记录在 `runs/scissor_left_underhandle_swing_follow_lift.json`。独立纸带试验中的浅支撑可在原托盘稳定存在，纸带外缘下净空约 17 mm，完整 LEAP 拇指仍会撞桌。110 mm 墙高的候选试验中保留了原隔板拓扑，纸带边缘净空约 60 mm。该墙高来自图像比例估算，尚未用于标准模型，也未验证右手提纸。见 `runs/scissor_paper_tall_wall_calibration.json`。

在正式场景尝试提前一个控制动作闭合左食指。剪刀轴离初始位置升高超过 110 mm 时，空中两个柄孔同时受力 2 ms。食指对固定柄主要向下施力，拇指从下方托住活动柄，两处接触随后失稳。记录见 `runs/scissor_airborne_fixed_loop_anticipatory_capture.{py,json}`。

110 mm 高墙候选让右食指与拇指在提拉阶段累计有 140 个物理子步相向接触，短暂剥离时纸段平均升高约 6.3 mm，仍有纸段留在托盘上。静置后纸带折成沿底板和隔板贴合的 L 形，尝试提拉时右腕落后约 50 mm。原剪刀摆放还与候选外墙重叠约 17.5 mm。该候选留在试验目录。记录见 `runs/scissor_right_paper_tall_wall_pickup.{py,json}`。

高墙候选的补充尝试在初次对夹后按实测接触收拢两指，抬升阶段有 277 个双指接触子步，但仍至少有 17 个纸带与托盘接触。沿隔间拉出再上抬、降低试验纸带摩擦及增加连接的弯曲刚度都未让纸带整个卸载。记录见 `runs/scissor_right_paper_tall_wall_adaptive_closure.{py,json}`。在另一条低力支撑试验的起点，左拇指单独托住悬空剪刀时，食指垫距固定柄约 54 mm，限幅 6 mm 的补入先失去了拇指接触。见 `runs/scissor_left_underhandle_fixed_loop_brace.{py,json}`。


将接近固定柄的食指力调节放到 2 ms 物理子步，双柄在悬空时有连续 15 个子步同时受力，食指峰值约 0.73 N。随剪刀转动，活动柄在拇指上的接触法向从向上转到接近水平，随后滑脱。见 `runs/scissor_airborne_tactile_brace_extended.json`。

后续从更早出现双柄接触的状态开始，左腕在上提时同时向托盘外侧移。剪刀脱离托盘和桌面后，左食指与拇指对两个独立柄的接触连续保持约 3.4 s。这段内剪刀轴向外移动约 111 mm，升高约 25 mm，机械臂碰撞计数为 0。334 步的标准参考回放记录剪刀轴最高比初始位置高约 117 mm，纸带仍在原位。详细的双柄接触在 `runs/scissor-two-handle-carry-reference-trace.json`。

```bash
python -m simulation.rollout data/derived/scissor-two-handle-carry-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-two-handle-carry-reference-rollout.json
```

起刀后，左腕再上提约 30 mm 并向中央平移，同时绕已接触的位置缓慢转腕约 30°。双柄持续受力且无托盘或桌面支撑的区间约 7.4 s，剪刀轴在该区间沿 Y 方向向中央移动约 232 mm。458 步标准回放记录剪刀轴最高比初始位置高约 145 mm，机械臂碰撞计数为 0。逐子步数据见 `runs/scissor_center_roll_roll30.json`。

```bash
python -m simulation.rollout data/derived/scissor-grasp-and-transfer-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-grasp-and-transfer-reference-rollout.json
```

另一次沿同一前缀让中指随姿态接近固定柄，连续 439 个物理子步（三指约 0.88 s）同时受力，且剪刀无外部支撑。拇指网格穿过活动柄孔，食指和中指的受力仍来自固定柄外侧。见 `runs/scissor_three_finger_aperture_bottom402.json`。在测得的悬空姿态上固定腕位，静态求解能让中指截面进入固定柄孔。用真实手指执行由外侧进入的过渡时未保持该入孔状态。见 `runs/scissor_middle_aperture_clearance_fit.json`。

纸带试验保留 35 mm 原托盘墙和原剪刀摆放，将纸带两端沿现有外墙自然垂到桌面和托盘底。实测接缝在墙顶稳定，距桌面约 39 mm。右拇指在该候选的一次无数值警告试验中短暂带起全部纸段，最低纸段上升约 2.1 mm，食指未接触。按实际接触逐子步延伸食指的试验仍未形成对夹。关节空间力矩控制将拟合腕误差降至约 23 mm，在 0.15 s 内食指仍未到达纸面。记录见 `runs/scissor_paper_normal_tray_overhang_settle.json`、`runs/scissor_right_paper_force_conditioned_acquire.json` 和 `runs/scissor_right_paper_joint_pd_highgain.json`。

现在 `cut_length_m` 记录实际断开的全部接缝。两物都必须离开外部支撑。左拇指与活动柄、左食指或中指与固定柄都必须实际受力，右手必须相向夹持纸带。此外用真实手指网格与柄孔平面的截面验证穿环，要求拇指穿过活动柄、食指和中指各自穿过固定柄。满足这些条件时断开的接缝才计入 `held_cut_length_m`。后者超过 3 mm 才记为完整任务成功。直接驱动机构的试验仍会得到真实 4 mm 切口，完整任务成功标志保持为假。见 `runs/scissor-production-held-cut-probe.json`。


正式场景现在让纸带以约 8° 的倾角靠在原有短墙内侧，纸带底边和托盘底均可实际接触。分段保持各自的自由关节，段间增加有限弯曲刚度，五个局部接缝仍由真实刀刃受力触发断开。旧接触参数使纸带在约 12 s 滑倒，提高纸带接触阻抗后通过 25 s 静置测试。458 步双臂回放中，22.90 s 后最高纸段中心仍高于托盘底约 88 mm，五个接缝完整。剪刀最高抬升约 145 mm，机械臂碰撞计数为 0。将纸带水平放入刀口的独立机构试验断开 8 mm 接缝，无刀刃接触的试验断开 0 mm，两条均未达到完整任务成功。记录见 `runs/scissor-upright-paper-transfer-trace.json`、`runs/scissor-grasp-and-transfer-upright-paper-rollout.json` 和 `runs/scissor-upright-paper-production-cut-probe.json`。

从标准起点执行 334 步后，固定柄由左食指接触，活动柄由左拇指接触，剪刀已经离开托盘。保持测得的腕姿，只调四个拇指关节，然后继续 12 步。铰链角从 0.0885 rad 闭至 0.0480 rad。到达最小角度时，两柄接触力分别约 0.20 N 和 0.61 N，剪刀没有外部支撑。双柄同时受力且无外部支撑的最长区间为 0.348 s。仅保持腕姿的对照闭合约 0.0056 rad。346 步标准回放的机械臂碰撞计数为 0。动作生成脚本在 `runs/scissor-left-closure-reference.py`，逐子步接触记录在 `runs/scissor_left_finger_close_observed_reference.json`。

```bash
python -m simulation.rollout data/derived/scissor-left-closure-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-left-closure-reference-rollout.json
```

右手隔离试验从纸带自然沉降后的状态和预抓姿态开始。最初两指相向受力点在纸面内相差约 25 mm。按实际 LEAP 网格的接触区域重新拟合，首次对夹时该距离下降到约 3.3 mm。纸带仍迅速旋转滑脱。多点碰撞、触觉力调节和第三指支撑的试验均未取得持续提纸。见 `runs/scissor_paper_wrench_known0.json`、`runs/scissor_paper_patch_capture_small-soft.json` 和 `runs/scissor_paper_tripod_capture_small-1.json`。

网格截面复核显示，悬空闭合所用的食指和中指均在固定柄孔外。原轨迹第 264 步时，剪刀仍受托盘支撑，食指已实际进入固定柄孔。此时中指离开口约 21.5 mm，实际推进时先被活动柄挡住。记录见 `runs/scissor_left_aperture_prefix334.json`、`runs/scissor_left_aperture_supported_scan.json` 和 `runs/scissor_left_middle_supported_insert264_contactpair.json`。

原预抓状态的闲置指节与掌部有约 26 mm 重叠，拇指根关节在前 0.1 s 偏离约 0.33 rad。把全部手指自碰撞纳入拟合后，有预抓候选的最小自碰撞净空约 0.06 mm。中部抓取候选在前 0.1 s 内的拇指根关节角变化不到 0.001 rad。相向接触期间最低纸段曾升约 5.5 mm，无托盘和桌面支撑。两垫各超过 0.05 N 的连续夹持仍只有 6 ms。逐步记录见 `runs/scissor_paper_jaw_tracking_mid.json` 和 `runs/scissor_paper_hand_clear_capture_m1.json`。

实测穿环判据在第 264 步识别到拇指和食指入环，中指未入环。第 334 步只有拇指仍入环。第 264 步用真实拇指接触把被动铰链从 0.1596 rad 打开到 0.2417 rad，食指在整个开环阶段仍穿过固定柄。整手检查在原预抓参考中发现了超过 60 N 的掌部与拇指指节自碰撞，当前参考还需重新拟合整手。相关记录为 `runs/scissor-aperture-metric-replay.json`、`runs/scissor_left_thumb_open264.json` 和 `runs/scissor_left_aperture_lowself_scan.json`。

右手提拉时，食指垫和拇指垫会在纸带外先碰到彼此，测得最大重叠约 1.75 mm。上部纸段相对腕部转动约 13° 后脱出夹口。只通过控制留出两垫间隙会消除明显的自碰撞，但该姿态仍无法维持对纸带的受力。记录见 `runs/scissor_paper_retention_diagnostics_m3.json` 和 `runs/scissor_paper_guarded_pinch_m-b.json`。

腕部根据纸带上段的实测位姿做有限幅跟随，提拉阶段仍由 Franka 和 LEAP 的执行器完成。最低纸段在真实相向受力且无外部支撑时升高约 56 mm。两垫各超过 8 mN 的连续区间约 0.25 s，各超过 0.05 N 的连续区间约 46 ms。继而向中央转运，在保持相向接触的样本里，上部纸段沿 Y 方向最多移动约 37 mm。该隔离动作仍有指节自碰撞，尚未和左手持剪刀动作合并。见 `runs/scissor_paper_pose_follow_mid-roll.json` 和 `runs/scissor_paper_pose_carry_roll70.json`。

复合掌心下，以固定柄孔实际截面拟合中指先入的姿态。联合调整 Panda 腕姿、LEAP 关节与被动铰链角后，中指截面进入固定柄孔，手距桌面约 30 mm。活动柄仍与左手指节重叠约 3.37 mm，因此没有物理执行该接近动作。扩大开度会把干涉位置沿活动柄转移到掌侧。候选与间隙记录见 `runs/scissor_left_middle_first_coupled_seeded.{py,json}`。


重新利用右手与剪刀柄及枢轴的实际接触，在托盘内将剪刀转向。转动阶段真实接触覆盖约 0.642 s，峰值法向力约 3.08 N，固定柄孔中心最高抬升约 12.8 mm。与托盘 +Y 外墙的间隙由约 58.7 mm 增至 69.2 mm，柄孔平面实测转动约 6.35°。这段动作提供了新的左手接近方向，尚未完成穿环。逐子步记录见 `runs/scissor_right_shank_roll_access_fine.{py,json}`。


在这个支撑姿态，左食指和拇指到各自柄孔边界的静态净空约 2.55 和 4.59 mm。用碰撞网格包络计算，左手最低点仍离桌约 8.27 mm。顺序执行时右手在左腕抵达前已失去柄部支撑，左手尚未接触柄。把左手预抓提前到右手支撑采样的起点后，整机起始姿态的两臂碰撞网格重叠约 81.1 mm，因此该同步方案未执行。净空及物理接触证据见 `runs/scissor_left_baseline_aabb_diag.json` 和 `runs/scissor_bimanual_contact_sync.json`。


右手经真实接触将剪刀放在倾斜的托盘支撑姿态，再完全撤回。自由静置 1 s 后剪刀由三处托盘接触承托，固定环平面倾斜约 17.67°。随后仅左手前进，食指和拇指的中心轴延长线在约 0.55 s 内与两孔相交。使用生产判据复核，食指、中指和拇指的完整网格截面均未通过环口，两个柄也未形成同时受力。在同一腕姿下平移手部测得的网格，严格双环截面要求腕目标至少比当前再降低约 36.3 mm，超出原路线的约 8.3 mm 桌面余量。采样中也没有食指、中指和拇指同时通过的候选。证据见 `runs/scissor_right_setdown_home.{py,json}`、`runs/scissor_left_after_setdown_mesh_verification.json` 与 `runs/scissor-staged-mesh-axial-search.json`。


默认剪纸布局按官方视频重新校准。闭合托盘墙由 35 mm 调至估计的 75 mm，剪刀初始自由位姿及开角取真实重力静置后的状态，纸带位置也随墙高重新计算。60 s 候选试验中剪刀平面约倾斜 30.65°，两柄中心平均越出短墙约 54.5 mm，高于墙沿约 34.7 mm。刀片和两侧柄臂由托盘底、隔板与短墙支撑，最后 1 s 剪刀轴位移小于 0.04 mm。纸带由另一端短墙和底板支撑，五个接缝完整。默认布局通过独立 25 s 静置测试，完整双臂入口也输出了 10000 × 6 点云。上文的 35 mm 托盘轨迹对应早期布局，新的插入和切割参考仍需在高墙布局上重做。试验见 `runs/scissor_official_rim_reset_candidate_recomputed_paper.{py,json}` 和 `runs/scissor-official-rim-entry.json`。 [官方剪纸视频](https://dex-cap.github.io/assets/normal_scissor.mp4)


在高墙默认场景，以同一对环中心尝试正反两个入环方向。反向接近避开了前臂与隔板；735 个候选中有 33 个让左拇指和食指网格分别穿过两环平面。它们均存在其他手指或指节与柄环及柄臂的穿透，满足截面条件的最佳姿态还有约 16.3 mm 的拇指指节碰撞，未执行这些接近动作。记录见 `runs/scissor_left_rim_opposite_mesh_search.json`。


当前默认剪刀使用分段椭圆柄环。固定环开口约 80 × 40 mm，活动环约 40 × 37 mm，厚度约 4 mm。按图像比例缩短了两侧柄臂。被动铰链增加 0.002 N·m 的估计静摩擦。61 s 自由静置后，两环中心分别在短墙上方约 26.9 mm 和 18.4 mm，最后 1 s 环中心漂移不超过 0.18 mm。三处开合角采样的两环净空均超过 11 mm。±0.02 N·m 的独立机构试验确认铰链仍可闭合和打开。评分按椭圆边界复核整个手指网格的过环截面，独立切割测试仍能实际切开纸带。尺寸与摩擦按照片和物理静置校准。上文的方形柄环接近结果留作历史。证据见 `runs/scissor_oval_near_center_physics_report.json` 和 `runs/scissor-oval-production-entry.json`。 [官方剪纸视频](https://dex-cap.github.io/assets/normal_scissor.mp4)


在当前椭圆柄环上，以错开的食指和拇指姿态重新拟合。静态最优端点的指网格截面都能通过双环，目标接触最大重叠约 0.985 mm。沿环面法向后退 10 mm 得到完全分离的起点，61 个接近样本无非预期碰撞。正常控制器执行 3 s 的实际接近时，剪刀仅移动约 0.09 mm，仍未同时穿环。腕误差约 0.73 mm。腕误差补偿和关节空间力矩控制也未建立动态同时入环，剪刀在接触中轻微移动。相关数据见 `runs/scissor_current_oval_{thumb_refine,progressive_entry,adaptive_entry,joint_pd_entry}.json`。


另一次从分离悬停开始，让右食指先真实承托固定柄杆，再同步推进左手。右手支撑只能保持约 0.10 s，左食指最终以完整截面进入固定环，活动环随剪刀倾转，拇指仍未通过。组合动作中剪刀约移动 1.15 mm，右手峰值接触力约 4.66 N。记录见 `runs/scissor_current_oval_right_brace.{py,json}`。


从实际入固定环的状态继续复核时，发现右手指节与左食指仍有约 3.8 mm 的交叠。改为右手完全撤离后才推进左手，实际采样的手间交叠降至零，但旧左手关节路径插入桌面约 38.8 mm，未重新取得入环。标准回放现记录 `hand_collision_steps` 和 `hand_max_penetration_m`，分别表示发生双手穿透的控制采样数和最大穿透深度，用于诊断接近路径。轨迹见 `runs/scissor_current_oval_{index_first_thumb,retreat_then_index}.json`，回放入口验证见 `runs/scissor-hand-collision-telemetry-entry.json`。


右手完全撤离后，将旧的左手关节直线路径改为先去高位，再保持腕姿分 41 个目标沿 Cartesian 路线下降。从真实初始位执行抬高 80 mm 的路线，105 个观测周期内左手与桌面、托盘、剪刀及右臂接触均为零，最终分离悬停的腕位置误差约 0.123 mm。随后以剪刀当前位姿更新双环目标，完整扫掠可找到静态双指端点，动态峰值食指和拇指接触力约 0.54 N 和 0.30 N。剪刀在接触中移动约 2.21 mm，尚未测到实际同时入环。证据见 `runs/scissor_current_oval_{high_clearance_route,live_micro_entry}.json`。

以 2 ms 物理步跟踪自由剪刀位姿，实际对向接触持续约 0.24 s。用实测关节位置更新目标后，食指和拇指对双环受力均大于 0.01 N 的连续样本首末间隔为 0.38 s，剪刀移动约 0.29 mm，无非预期碰撞。实测食指尖和拇指连杆距环面最小约 0.5–0.7 mm，增加轴向行程和向环内移动后仍未同时穿环。这里覆盖食指与拇指，中指入固定环及双手切割仍需执行。记录见 `runs/scissor_current_oval_physics_rate_{relative,measured_gate-0.06}.json` 和 `runs/scissor_current_oval_ring_section_inward_correction-0.06.json`。

按指尖网格到环面的实测距离分别调节食指和拇指关节后，拇指截面共 15 个 2 ms 子步通过活动环，最长连续 10 步。食指尚未通过固定环。将食指先向环内侧移动再推进的对照增加了环缘受力，仍未取得同时入环。两次试验均无非预期碰撞。数据见 `runs/scissor_current_oval_ring_section_{digit_correction,sequential_digits}-0.06.json`。

加入新纸带后，从实际初始位复跑原 80 mm 抬高路线，105 个观测周期无预期外接触，分离悬停腕误差仍约 0.123 mm。另找到了食指严格通过固定环、整手无接触的静态端点。从高位下降到这个端点时，伸直中指会碰固定环，收折中指与无名指又会使无名指碰环。该接近未执行。记录见 `runs/scissor_paper_grid_high_clearance_route.json` 及 `runs/scissor_current_oval_index_center_thread{,_route}.json`。

采用当前 LEAP 止挡，在渐进收指的安全前缀之后以实际力矩再推进名义 4 mm。食指接触固定环的最大实际侵入 0.166 mm，峰值 0.506 N，无其他接触。再沿运动中的环法向追踪时，相对实际推进约 1.42 mm，食指尖开始碰到活动环顶部后停止，仍未测得完整截面穿环。完整 2 ms 力和关节记录见 `runs/scissor_current_assets_force_limited_gate.json` 与 `runs/scissor_current_assets_relative_compliant_pass.json`。

按指尖与活动环的实际接触 Jacobian 作局部调整，先解除该接触，并在固定环接触保留时再推进 4 mm，活动环顶部随后再次接触。利用同一位置的实际接触力打开被动铰链，撤手并静置后开角保持增加 0.0103 rad，柄环中心偏开约 0.71 mm。由这个实际状态重新插入，初始活动环净空为 0.252 mm，食指后来与活动环分离，但在固定环持续受力时把剪刀整体推移约 20.46 mm，仍未形成完整穿环。数据见 `runs/scissor_current_assets_{index_tip_jacobian_detour,passive_handle_open,open_handle_insert}.json`。

尝试让右食指压住剪刀固定柄再由左食指穿环。右手下降路径的完整碰撞网格预计与托盘隔板相交约 4–5 mm，物理执行在该段之前停止。此前 120 条实际控制命令中，剪刀位移约 0.12 mm，右食指尚未接触固定柄。记录见 `runs/scissor_current_assets_bimanual_shank_brace_insert.json`。

左食指随后改从外露柄环推动，把剪刀压向托盘侧壁。撤手后剪刀沿之前的滑移方向保留约 6.77 mm 位移，固定柄和活动柄各有一处托盘支撑接触。侧壁沿原滑移方向的阻力峰值约 0.151 N，撤手后约 0.020 N。记录见 `runs/scissor_current_assets_ring_seat_against_tray.json`。

从撤手后的实际就位状态再插入，剪刀位移由约 20.46 mm 降至 9.18 mm，托盘反向承载峰值约 0.558 N。插入执行到约 77% 时，食指与活动环接触达到约 2.02 N，触发试验的 2 N 停止条件。未完成穿环。记录见 `runs/scissor_current_assets_seated_insert.json`。

使用就位后的实测姿态计算，原来停止处的名义目标与活动环第 7 段相隔约 12.8 mm。改走 6 mm 横向避让后，扫掠在更早的 53% 路程处让近端食指网格侵入固定环约 1.62 mm，未执行该避让。记录见 `runs/scissor_current_assets_seated_lateral_detour.json`。

按每段实测腕姿逐步跟进的动作执行到约 53%。首次接触为食指近端碰固定环底部，此时剪刀在托盘上已经自然移动约 2.53 mm。活动环第 7–9 段在失败处仍有约 7.2–9.3 mm 净空，整个分支未测到活动环载荷。固定环载荷峰值约 0.494 N，剪刀继续移动和旋转，腕姿跟踪未恢复，未完成穿环。记录见 `runs/scissor_current_assets_seated_mass_servo_insert_no_passive_drift_cutoff.json`。

根据实际固定环底部法向联动食指末两关节卸载，固定环初始接触由约 0.157 N 降至零，接受行程推进到 55%。后续固定环接触使自由剪刀位移约 9.23 mm，腕姿误差再度扩大。活动环仍无受力，未完成穿环。记录见 `runs/scissor_current_assets_fixed_bottom_normal_unload_insert.json`。

由左手已经进入到约 52% 的实际状态拟合右手高位支撑。高位指尖可以到达，沿竖直方向下降时右食指先碰到左手并与固定环多个构件交叠，同时右掌碰到托盘隔板。该路径未进入物理执行。记录见 `runs/scissor_current_assets_overhead_exposed_fixed_brace_insert.json`。

改为左手先撤离，再从高位拟合右手，消除了双手之间的碰撞。允许右食指同时接触相邻固定环构件后，完整右手仍会碰到托盘隔板或外壁。保留其余手指原姿态的候选由中指连杆先碰外壁，预计深约 0.35 mm。该路径未进入物理执行。记录见 `runs/scissor_current_assets_resequenced_overhead_brace_insert_index_only.json`。

复核此前 55% 执行位置的真实开口坐标，接触固定环底部的是食指近端，末端指尖在环面内距中心仍约 81 mm。由该实测姿态逐步追踪移动环心时，剪刀反而平移约 12.52 mm，未发生穿环。改到对侧直接对准指尖，端点几何仍与固定环交叠约 2.57 mm，且下降前已有指节碰撞，因此没有执行这条对侧轨迹。记录见 `runs/scissor_current_assets_{live_opening_distal_follow,opposite_distal_hover}.json`。

## Tea Preparing

`python -m simulation.rollout --task tea-preparing` 使用固定茶盘、可移动的木质托槽、空心瓶和茶壶、弹簧镊子以及可自由落下的茶粒。瓶盖有独立自由关节。螺纹通过 MuJoCo 的转角与轴向滑动约束表达，达到两圈并实际升到约 4 mm 的退丝位置后解除连接，瓶盖保留当时的位姿和速度。壶内茶粒必须由壶体实际支撑并低速稳定，工具归位依靠实际茶盘或木槽接触。论文规定了两圈和任务顺序，螺距、器具尺寸、材料与归位容差为估计。 [论文任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6)

在支稳茶瓶的测试中，对帽体施加 0.007 N·m 力矩，转一圈时实际升约 2 mm 并保持连接，接近两圈和 4 mm 后成为独立刚体。正式双臂场景中，从壶口上方放开的一粒茶在约 1.75 s 后由壶底承托并计为入壶。整段双臂开盖、辅助倒茶和归位仍缺作者的动作与权重。结果见 `runs/tea-pot-production-drop.json`。

把瓶体固定为试验夹具，右侧 Franka 与 LEAP 靠真实手指接触驱动瓶盖达到 **1.19 圈**，相应轴向行程 2.32 mm。重新夹持过程中会丢失非拇指接触，仍未达到两圈退丝。记录见 `runs/tea-leap-cap-robot-probe.{py,json}`。另一次换指试验在转到约 0.58 圈后失去拇指制动接触，盖回退至约 0.48 圈，见 `runs/tea-leap-cap-hinge-regrasp-probe.{py,json}`。固定腕后用手指交替推进，瓶盖在两个切向推动方向均回退。记录在 `runs/tea-leap-cap-finger-gait-probe.{py,json}`。


最后一段退丝单独使用固定瓶身、编译时预旋约 1.7 圈的夹具。右手通过接触驱动剩余螺纹行程，盖在约 4 mm 轴向位移处释放。右手随后继续接触，将已独立的瓶盖上抬约 9 mm。记录在 `runs/tea-leap-cap-last-turn-lift-probe.{py,json}`。从零开始的重新抓握仍有回旋。提高螺纹干摩擦虽能抵住重力回退，却降低了换指过程的累计转角。另一次尝试沿瓶轴退离、径向重新靠近，盖在无手部接触的退离阶段已回到零，之后掌部和指节接触又产生偏心转矩。见 `runs/tea-leap-thread-friction-regrasp-probe.json` 和 `runs/tea-leap-cap-axial-radial-regrasp-probe.json`。


自由瓶身的左手探针将腕跟踪误差降到约 1.55 mm，接触仍在瓶体同一侧。瓶体仍受茶盘承托，未达到离桌对向夹持。记录在 `runs/tea-leap-joint-clearance-wrap-probe.{py,json}`。

另一条轨迹用左手侧推，让起始竖直的自由瓶身绕托盘接触点倾斜。瓶身倾到约 82° 时出现了拇指和食指的真实相向接触。瓶身随后沿托盘滑约 19 mm，食指接触丢失。记录在 `runs/tea-leap-tilt-pivot-grip-probe.{py,json}`。 跟随瓶身平移时，相向接触最长只持续约 50 ms。跟踪瓶身相对姿态的试验在首个保持周期出现约 52 mm 左腕误差，未进入提瓶。

在托盘内将自由瓶身初始放在 `[0.47, -0.075] m` 的边框试验中，左手经真实接触将瓶身推到边框，拇指和食指相向接触持续 0.3 s。瓶身随后在对向接触下沿边框滚回内侧，仍由托盘底承托。请求上提 24 mm 时实际瓶身只升约 1 mm。记录在 `runs/tea-leap-rim-supported-lift-probe.json` 和 `runs/tea-leap-rim-roll-exit-lift-probe.json`。

在同一声明的边框摆放中，从竖直自由瓶和闭合瓶盖开始。左手倾瓶后用拇指和两根手指配合真实托盘沿支住瓶身。右手连续 18 次转腕期间，瓶身倾角保持在 78.3–79.6°，世界偏航变化约 1.8°。右手前六次与瓶盖有接触，螺纹回退约 0.04 圈后接触丢失。反向转腕和缩短夹距也未取得有接触证据的正向旋转，因此完整退丝仍待验证。见 `runs/tea-leap-tilted-rim-counterhold-right-turn-result.json`。针对该反持姿态拟合了两组右手指尖，测试的 Panda 腕姿分支到达关节限位，仍未形成两个真实指面的相向接触。见 `runs/tea-leap-cap-reachable-fit-result.json`。


另一条试验用真实左拇指和托盘支点将自由瓶停在约 37.4°。此姿态离线拟合到左手两面相向接触，全臂最小关节余量约 0.89 rad。物理执行闭合时，瓶体在第二侧形成受力前出现摆动，左手仍只有短暂单侧支撑。该姿态的两支右腕拟合也未形成可用的对向指尖接触。见 `runs/tea-leap-midtilt-study-result.json`。


默认镊子短轴桥的两处静态指垫间隙约为 +0.026 mm 和 −0.127 mm。完整接触诊断显示，该末端拟合姿态同时使右臂和手部与桌面重叠约 100 mm，与托盘重叠约 63 mm。关节空间控制也被碰撞推离，0.4 s 内腕部漂移约 89 mm，未进入闭合。沿两支弹簧臂的 30 个候选也未找到双指对向接触且具有足够关节余量的姿态。见 `runs/tea-leap-tweezers-joint-pd-result.json` 和 `runs/tea-leap-tweezers-outer-arm-result.json`。


将右腕的手背轴朝向校正后，默认镊子从上方接近的路径仍受瓶盖或茶壶阻挡。另设已开盖的分阶段场景，盖子自由放在托盘上。瓶子保持原位时，右腕下降路径与瓶壁重叠约 6.8 mm。将瓶子移开后最严重的下降干涉约为 4.8 mm，闭合时食指垫仍与托盘底重叠约 26.3 mm。自由沉降确认镊子的两根弹簧臂靠住托盘边，倾角约 4.6°。按 110 mm 杠杆长度估计，需再抬高约 9° 才能得到所需净空。两条侧向推入候选仍与桌面干涉，未执行。记录见 `runs/tea-leap-post-uncap-tweezers-result.json` 和 `runs/tea-leap-post-uncap-tweezers-support-result.json`。

已开盖的分阶段场景中，镊子的托盘边缘支点与长臂端点相距约 69 mm。考虑到已有倾角，还需再抬约 9°。真实食指末端离镊子仍有约 17 mm 时便已贴近托盘边，采用食指中节的两条路线也被下游指节阻挡。另测试了 24 个位于托盘右上的自由初始摆放，8 个可形成部分悬挑。桥部净空约 14 mm 的摆放能静置。改为从同高度夹住左右侧面时，食指仍在距接触点约 52 mm 处遇到托盘底板。未执行碰撞路径。记录见 `runs/tea-leap-tweezers-rim-lever-result.json`、`runs/tea-tweezers-rim-placement-result.json` 和 `runs/tea-leap-sideface-grasp-result.json`。

另从闭合瓶盖和自由瓶身重复了双手反扭矩试验。瓶体倾斜后由左手和原托盘边保持支撑。右手只有单侧指腹接触盖子，前三个切向动作的轴向扭矩向回拧方向，螺纹回退约 0.0043 圈。随后出现的小段正向旋转发生在右手已失去接触之后。记录见 `runs/tea-leap-bimanual-tangent-baseline-result.json`。

新掌心下，对自由直立瓶身重新拟合左手两侧夹持。一个腕姿的对侧法向点积约 −0.74，Panda 关节余量约 0.92 rad，但该姿态要求拇指穿入瓶身约 21 mm，而且折叠的闲置指节会撞桌。绕世界竖直轴转腕 90° 后仍撞桌。未执行这些闭合轨迹，也未执行右手开盖。记录见 `runs/tea-leap-upright-two-pad-grip-result.json`。

先用左手横推瓶身，再让瓶底与托盘边沿真实接触。经过翻转和撤手，瓶子由托盘底板与边沿共同承托，连续 0.90 s 没有机器人手接触，最终瓶身相对竖直方向倾斜约 89.45°。瓶盖仍贴近桌面。试验脚本和接触记录见 `runs/tea-leap-midtilt-stowed-support-probe.{py,json}`。


从上述支撑阶段用受力矩限制的关节空间控制驱动右拇指，在瓶体仍与托盘接触时推到瓶身。实测推力约 0.526 N，瓶盖底部与桌面的间隙由 0 增至约 5.94 mm。随后拇指保持接触约 0.25 s，间隙经历起伏，第六个周期失去接触。瓶盖仍有螺纹约束，且手指没有接触瓶盖。结果与控制脚本见 `runs/tea-leap-rim-stage-thumb-jointpd-{probe.py,result.json}`。


使用默认器具摆放，从无接触的机器人预抓位开始，右食指由上方接触自由镊子。追踪真实接触点并调节下压力后，手与镊子接触连续覆盖 170 个 50 ms 周期，实测峰值接触力约 0.13 N，镊子整体最大转角约 8.37°。保持同一预抓位的无接触对照最大转角约 0.72°。两组的桥部离桌间隙均约 6–7 mm，镊子仍由托盘支撑，尚未夹起。机械臂碰撞计数均为 0。脚本、动作结果与自由沉降对照见 `runs/tea-tweezers-contact-follow-measured.py` 和 `runs/tea-tweezers-contact-follow-{e,control}.json`。


持续跟随镊子的上表面接触点并沿器具长轴施力，右食指接触保持约 12.5 s，镊子最大转角约 18.97°，桥部离桌间隙约 6.91 mm。通过高位换腕后实际执行了接近和闭合，未形成相向夹持或抬升。记录见 `runs/tea-tweezers-axial-tip-a.json` 和 `runs/tea-tweezers-high-route-pinch-a.json`。

在托盘角落的试验中采用明确声明的瓶子初始位置 `[0.43, -0.015] m`。左手物理调整并撤回后，瓶体连续 0.5 s 由现有托盘边沿承托，按瓶盖当前姿态计算的最低点离桌至少 28.68 mm。记录见 `runs/tea-leap-sequential-pocket-cap-probe.json`。沿此分阶段起点继续，右手拇指建立两处真实盖体接触，初次接触法向力峰值约 0.286 N。一次 0.5° 腕部绕轴动作后，两处接触仍在，终态法向力约 0.063 N，瓶盖相对螺纹前进约 0.000090 圈，离桌约 27.46 mm。109 个样本无物理警告。这段动作还未形成可持续的旋盖。记录见 `runs/tea-leap-topthumb-turnpulse-probe.{py,json}`。

改用 10 ms 周期，根据实际瓶盖接触法向调节指垫压力。该分阶段试验让右拇指保持正法向接触 1.53 s，瓶盖螺纹在滑开之前前进约 0.04549 圈（16.38°），指垫峰值约 0.527 N，盖体距桌始终至少 26.23 mm。未计入滑开后的转动。手指之间另有约 42 N 的自碰撞，且这一段瓶身由托盘支撑。对非相邻指节的碰撞显式加约束后，最佳位姿仍有约 1.4–2.9 mm 的手部自穿透，且未找到可执行的接近路线。证据见 `runs/tea-leap-topthumb-contact-normal-long.{py,json}` 和 `runs/tea-leap-topthumb-weld-clear-roll-*.json`。执行原分阶段动作时使用 Conda 默认 BLAS 线程设置，单线程执行会得到不同的接触轨迹。

官方视频首帧显示，机器人右侧有单独的木质浅槽，两臂镊子由浅槽托住，合拢一端伸至右沿外。早期场景把镊子放在茶盘另一侧，且缺少浅槽。将现有镊子在试验中移到右侧边沿可让拇指和食指的目标间隙分别达到约 0.36 和 0.30 mm，但接近时食指与茶壶相交约 21.6 mm，闭合时仍与盘底相交约 26.6 mm。精确摆放和工具尺寸尚未定标，新的摆放只保存在试验中。布局首帧和试验见 `runs/official-tea-layout/{tea-t0.jpg,tea-official-tool-layout-comparison.json}`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

默认布局现采用外缘高约 28 mm、承物面高约 24 mm 的茶盘，并加入可自由移动且由盘面支撑的木槽。瓶、壶与镊子按首帧估计的位置摆放，镊子的合拢端伸出右侧。第一组薄盘试验约 6.5 s 后镊子滚脱。更新后完整双臂场景在 30 s 真实接触静置后，镊子位移约 2.1 mm、仍由木槽承托。标准三动作入口无物理警告。离线右手下降路径未出现负间隙，闭合时食指仍与木槽底相交约 16.2 mm，本次没有执行该闭合路径。上文的开盖试验发生在早期茶盘布局。证据见 `runs/tea-official-tool-cradle-{30s,probe}.json`、`runs/tea-official-cradle-production-entry.json` 和 `runs/official-tea-layout/tea-t0-tray-plane-fit.json`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

原摆放的镊子桥部和弹簧臂在第一步之前穿入木槽端壁，最大重叠约 7.67 mm，满载双臂场景自然运行 10 s 后仍约 5.73 mm。把镊子的初始桥心抬高 11 mm，保持原来的平面位置和方向，让自由镊子靠重力落在木槽上。对照中初态无镊子与木槽重叠，10 s 后由木槽端壁和侧壁承托，最大压入约 0.087 mm，桥心平面漂移约 1.60 mm。已有木槽回归加入初始碰撞断言，先复现失败，修正后 11 项泡茶测试通过。标准双臂入口执行 3 步并输出 `10000 × 6` 点云，无 MuJoCo 警告。对照与入口记录见 `runs/tea-tool-cradle-start-height-{0,11}mm.json` 和 `runs/tea-raised-tweezers-production-entry.json`。

从修正后的真实沉降状态单独测试取镊子，右食指向桥部施加法向力后沿轴向拉动。镊子向槽外移动约 9.25 mm，并实际转出木槽。撤手后由茶盘底面和边沿承托。该状态下对桥部重新拟合侧夹，仍没有相向夹持且可全程通过的候选，未执行夹合。记录见 `runs/tea-raised-tool-traction{,-settle}.json` 和 `runs/tea-raised-tool-edge-pinch.json`。

用新指形从实际静置姿态进入，并按触力调整指形。拇指和食指最终分别对镊子施加约 0.127 N 和 0.009 N，但法线未相向，镊子仍有茶盘和木槽接触。进一步合拢会先导致指尖之间接触。尚未出现离开支撑的有效抓持。记录见 `runs/tea-raised-tool-edge-tactile-full-tool.json`。


以官方抓取画面为依据，在新布局试探镊子距合拢端约 35 mm 处两条弹簧臂的外侧接触。下降路线无负间隙，闭合时右拇指与盘底仍相交约 24.6 mm，未执行闭合。当前镊子臂厚 4 mm，后续需要用视频核准指垫接触处的截面。记录见 `runs/tea-official-tool-armgrasp-integrated-probe.json`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

官方视频把旋下的盖子松手放在茶盘右侧的桌面。`cap_on_right` 因此接受右侧盘内或紧邻右沿的主桌面支撑，同时要求盖子已脱离螺纹、静止并且没有手部接触。盘外区域宽度为估计值。测试覆盖盘内、右沿外、左侧和远离茶盘四种静置位置。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

持瓶和取镊子的判据要求相应一侧机器人手的两个不同指节或掌部构件产生相向的正法向力，而且器具已经离开桌面和托盘支撑。镊子在开盖后才能记录为已使用，归位时必须由托盘承托并已松手。回归测试覆盖没有手接触的器具位移。 [论文任务顺序](https://arxiv.org/html/2403.07788v2#A1.SS6)


官方开盖画面可见收窄的瓶颈。默认模型据此加入估计半径 27 mm、高 14 mm 的空心瓶颈。帽壁内半径与瓶颈多边形最大外接半径之间留有约 3.2 mm 净空。直接加入瓶颈后，原铰链惯量太低，在沿螺纹正向施加 0.007 N·m 的固定瓶身试验中，帽体与螺杆支架的最大角度差接近 180°，未完成退丝。生产代码补入等效铰链惯量和 0.00016 N·m 的估计干摩擦。两组同动作对照中，生产焊接设置的最大角度差约 0.086°，按直径缩放焊接权重并提高阻抗的候选约 1.29°。两组均退丝，生产设置的轴向导程最大误差约 0.00030 mm。生产设置在停施力矩 4 s 后回退约 0.00021 圈，施加反向力矩仍可回拧。证据见 `runs/tea-thread-production-weld-ab.json` 和 `runs/tea-thread-neck-production-entry.json`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4) [MuJoCo 焊接约束](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#equality-weld)


在生产场景的独立固定瓶身试验中，右臂和 LEAP 仅用正常执行器第一次将瓶盖推进约 0.61 圈。张手后轴向退 70 mm，复位右腕期间瓶盖无手部接触，仅回退约 0.00219 圈。再次接触时在首次形成相向力后停下闭合，并执行下一段转腕，瓶盖峰值达到约 0.841 圈和 1.681 mm 轴向行程。第二段转动的后期失去非拇指接触，盖子仍在螺纹上。另用默认自由瓶身验证左手接近，两个指垫从完全分离的姿态开始，在实测瓶身位置反馈下建立接触，0.8–1.3 s 的五个采样点均存在相向受力。尝试抬升时这对接触最终滑脱，瓶子始终由托盘承托。证据见 `runs/tea-live-cap-{mechanism,regrasp-opposedstop}-probe.json` 和 `runs/tea-thread-neck-free-bottle-radial-close.json`。


根据官方视频侧视图的三条模压环带，正式瓶身采用 2 mm 壁厚、连接瓶颈的肩部及三条可接触环肋。环带高度按画面比例估计，塑料密度和壁厚暂无实物测量。计算出的空瓶身质量约 65.5 g，早期厚壁模型约 96.2 g。夹具将瓶身以 150° 倾角放在壶口上方，合盖时 8 粒全部留在瓶或盖内，初始开盖时 8 粒经瓶口进入壶内，无侧壁漏出。此前按球半径缩小壶内判定区域会遗漏壶底实际支撑的两粒。现在按颗粒中心是否在壶内、实际壶接触和速度判定，凸多边形边角的颗粒也能计入。默认双臂入口输出点云且无物理警告。标准几何记录见 `runs/tea-bottle-production-ribbed-transfer.json` 与 `runs/tea-bottle-ribbed-production-entry.json`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

在新版瓶身的固定瓶体夹具上复跑右手换握动作，实际帽体峰值约 0.907 圈，轴向约 1.814 mm，结束时约 0.904 圈。自由瓶身的初步合拢试验曾出现瞬时离托盘，未形成相向夹持，不算抬瓶。记录见 `runs/tea-live-cap-regrasp-ribbed-probe.json` 和 `runs/tea-bottle-material-ribbed-physical-close.json`。


把瓶内填充从 8 颗球换成 63 块估计尺寸的茶叶团是独立试验。装料质量由 12 g 增至约 25.9 g，站立时最高粒中心约 27 mm。150° 倾角下，封盖留住全部 63 粒。开盖后，63 粒最终都在壶中，其中只有 30 粒直接触壶，其它由茶粒层承托。正式计数现在沿壶内受支持的颗粒接触链判断，包含稳定叠放的茶粒。这组 63 粒试验提供了后续装料标定的对照。数据见 `runs/tea-filled-bottle-transfer-sensitivity.json` 和 `runs/tea-filled-bottle-pot-support.json`。


默认装料按官方侧视图估计为约半瓶。108 个折叠茶叶团使用可碰撞的独立椭球，按重力沉降后的瓶内相对位姿复位，最高表面约 50.4 mm，装料约 37.0 g。椭球尺寸和密度为估计；装料的体积质量与既有散装茶叶测量同量级。以 150° 倾角隔离验证，新默认封盖保住全部 108 片，开盖夹具将 108 片稳定倒入壶内。夹具测量不依赖手部抓取。较重装料的抬瓶和双手过渡仍需验证。装料位姿和试验见 `simulation/assets/tea_charge_poses.txt`、`runs/tea-source-fill-108.json` 和 `runs/tea-production-filled-{sealed,open}-fixture.json`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4) [精制绿茶堆积密度实测](https://www.jstage.jst.go.jp/article/cha1953/1971/34/1971_34_37/_article)

加入更多几何后，原离屏多重采样在木槽和托盘边缘将 RGB 编码的分割编号 351 和 137 混出了编号 361。禁用离屏多重采样后，标准双臂入口完成三步，输出 `10000 × 6` 点云且无物理警告。当前填充物使 CPU 求解更慢。Newton 在封盖夹具的对照中保持有限状态，CG 在 0.008 s 出现加速度警告。生产采用已验证的 Newton 设置。见 `runs/tea-production-filled-entry.json` 和 `runs/tea-loaded-solver-timing-probe.json`。 [MuJoCo 离屏多重采样参数](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#visual-quality)


用默认满载瓶验证右手退丝时，瓶身由试验夹具固定。原换握脚本将同一拇指的两个连杆误判为不同手指对握。单线程试验改为只有拇指与另一根手指对向受力时才再次转腕，第二次合拢阶段仅靠拇指接触将盖推至约 1.142 圈和 2.285 mm 的轴向行程。该阶段没有形成第二次对握，瓶盖仍在螺纹上。两次试验采用不同的 BLAS 线程数。记录见 `runs/tea-live-cap-regrasp-loaded-{probe,distinct-digits}.json`。

重复使用右手张开、轴向撤离和合拢动作，把同一满载夹具上的瓶盖推进至峰值 1.752 圈和 3.504 mm。最后一段张手时下降到约 1.36 圈，随后已经脱离盖子的复位只损失约 0.002 圈。末次合拢后停在 1.246 圈，螺纹仍连接。回退主要发生在张手阶段。记录见 `runs/tea-live-cap-loaded-incremental-thumb.json`。

在同一默认满载固定瓶身夹具上，最后一轮先保持指形，向外撤 20 mm 再沿瓶轴抬高 70 mm，瓶盖从 1.742 圈微退到 1.715 圈。随后张手时拇指末端再次触到盖沿，以正向转矩把盖子带过两圈，螺纹连接在子步中解除，峰值轴向行程 4.021 mm。用上一段实际积分保存的状态单独重放，得到同一退丝阶段和峰值，无物理警告。脱离后的盖子落回瓶颈附近，取走并放至茶盘右侧仍需执行。记录见 `runs/tea-live-cap-loaded-clear-then-open.json` 和 `runs/tea-live-cap-loaded-retreat-from-checkpoint.json`。

这两份轨迹使用旧止挡。保存状态的逐关节复核证实，退丝前个别指尖关节已偏出 URDF 范围 3.45 rad，退丝后还有关节偏出约 1.81 rad。该旧轨迹的退丝行程不计入现行止挡模型的结果。泡茶的握持计分现在按 LEAP 的实际手指区分接触，避免把一根手指的两个连杆算作有效对握。

备茶瓶身、瓶颈、瓶盖和茶壶的圆壁使用 24 个相邻凸楔体，使内部与外部切面采用同一角分段。拼接处保留数微米的交叠，容纳网格数值误差。瓶底和壶底延伸到内壁多边形的最外点，封住与圆形底板之间的角部间隙。按每个圆壁 720 个方向测量，旧瓶身、瓶颈、瓶盖、壶壁的最薄处约为 1.04、1.55、2.07、3.11 mm。修正后分别为 2.00、3.00、4.00、6.00 mm，最厚处相对各自标称厚度增加小于 0.9%，最大径向误差约 0.37 mm。盖裙与瓶颈之间仍有超过 2 mm 净空。退丝、退丝后自由释放、茶叶落壶及堆叠支撑测试通过。53 项 MuJoCo 测试通过，默认双臂场景标准入口完成 3 步并输出 `10000 × 6` 点云。测量及入口结果在 `/data/dexcap-repro/benchmarks/tea-vessel-ring-geometry/`。

在现行止挡模型上单独验证取盖。夹具固定满载瓶身，起始声明盖子已脱扣并停在瓶颈，左臂在停放位。右手食指和拇指实际相向夹持，使盖子完全脱离瓶颈并上升 56.3 mm，连续 0.44 s 的采样同时满足对向受力与不接触瓶身。增加合拢预载后，盖子在同样条件下横移 56.3 mm，之后滑脱，尚未放置到托盘右侧。取盖与搬运试验所记录的两个手的关节均未越出 URDF 范围。记录见 `runs/tea-detached-cap-home-fixture-pinch.json` 和 `runs/tea-detached-cap-home-fixture-place-preload.json`。

保持相同的实际提盖起点，依据食指和拇指的实测法向力逐步调节合拢，连续夹持横移提高到约 100 mm，仍在到达右侧前滑脱。试验中把腕部随盖子下滑只增加了垂直偏移。绕盖中心转腕的动作在横移前松脱，增加中指的试验在中指建立受力前丢失原有夹持。数据见 `runs/tea-detached-cap-home-fixture-place-{force,force-height,roll}.json` 和 `runs/tea-detached-cap-middle-preload.json`。

此前取盖检查点继承了已倒伏的茶壶状态。本轮除预设瓶盖已退丝及固定瓶身外，从默认器具摆放重新初始化。右手复现 56.3 mm 提盖，先松开会向瓶身方向推盖的食指，随后松开拇指，盖子在托盘上脱手静置。右手从实际保存的状态分三次沿托盘推动，最终盖心世界坐标约 `[0.5264, -0.0700, 0.0080] m`，由托盘支撑且右手已撤开，`cap_on_right=true`。该段无物理警告，实测手部关节无越限。壶全程自由，最终较默认位置水平移约 30 mm。记录见 `runs/tea-detached-cap-fresh-layout-pinch.json`、`runs/tea-detached-cap-fresh-layout-ordered-release.json`、`runs/tea-detached-cap-diagonal-{slide-thumb-clear,reapproach}.json` 和 `runs/tea-detached-cap-right-zone.json`。

从盖子仍在螺纹上的同一满载夹具独立起步，右拇指和食指取得相向受力。腕部转动在约 10° 命令时失去食指接触。保持腕姿、利用接触点 Jacobian 驱动手指时，两个相反的切向指令分别测到约 -0.00078 和 -0.00471 N·m 的平均轴向力矩，方向均为拧紧，未完成松盖。记录见 `runs/tea-loaded-threaded-cap-modern-pinch.json` 和 `runs/tea-loaded-threaded-cap-twist-{torque,reversed-torque}.json`。

现行止挡下，用同一 108 片填充物的满载固定瓶身夹具，从盖子仍上螺纹的状态连续积分。右食指在帽壁侧面偏心处向内施力，沿实测盖姿跟进。首段转过 0.613 圈，轴向升起 1.226 mm，峰值接触力约 5.93 N。四次撤手并按实时盖姿重新选择施力位置后，瓶盖达到退丝阈值，约 1.998 圈，轴向行程 3.996 mm，`cap_unthreaded=true`。全程未出现 MuJoCo 警告或 LEAP 关节越限。记录见 `runs/tea-thread-offset-facet-{drive,regrasp}.json`。

从第五段的实际积分状态继续，右手先撤开，再以食指与拇指对夹，实测抬盖 64.5 mm 且瓶身接触消失。连续搬运和依次松指后，盖子脱手并静置在托盘，世界坐标约 `[0.4811, 0.0496, 0.0289] m`。自由茶壶较默认起点水平移动约 31.5 mm。该段无物理警告，LEAP 关节均在 URDF 范围内。记录见 `runs/tea-threaded-to-cap-lift.json` 和 `runs/tea-threaded-to-tray-release.json`。

放盖后减慢右手接近并在高位停稳，沿托盘把盖子水平推移约 16.8 mm 后再次撤手。盖子仍获托盘支撑，`cap_on_right=false`。继续沿该方向的候选会让食指接触茶壶外壁。改沿茶壶外侧接近，推程采样没有形成有效施力，盖子仍在托盘中段。记录见 `runs/tea-threaded-tray-push-slow.json` 和 `runs/tea-threaded-tray-push-pot-arc.json`。

直接从同一真实提盖状态在高位绕过茶壶，再向托盘右侧下降。第一条路线使盖子落到工具托槽，调整落点后，右手最后相向持盖时盖心高约 119.6 mm，下降过程中盖子在预定松指前滑脱，最后由托盘承托，脱手静置于 `[0.5041, -0.0763, 0.0289] m`，`cap_on_right=true`。无 MuJoCo 警告或手关节越限。该序列从同一满载瓶仍上螺纹的实际积分状态接续，瓶身全程由试验夹具固定。记录见 `runs/tea-threaded-to-right-{direct,open-pocket}.json`。

改让默认位置的满载瓶保持自由，左右手分别从预抓姿态开始，只用执行器控制。左手通过合拢取得托盘承托下的夹持，闭合末采样拇指与食指分别约 1.45 N 和 2.42 N。右食指从仍旋紧的瓶盖侧面偏心推进，同时左手保持相向受力。右手向上修正 2 mm 后，瓶盖实际转过 0.0216 圈并轴向升起 0.043 mm。右手触盖且法向力超过 0.01 N 的 468 个物理子步中，有 442 个子步也测到左手相向受力，左手未接触瓶盖。继续内推时，候选路径会碰到瓶颈，螺纹仍连接。以同样的右手初始姿态独立执行、左手停在停放位的对照转过约 0.0133 圈。三次均无 MuJoCo 警告或手关节越限。记录见 `runs/tea-free-upright-cap-{baseline,counterhold}.json` 和 `runs/tea-free-upright-raised-cap-counterhold.json`。

从实际自由瓶接触状态撤开右手并改从上方接近，右食指以约 1 N 法向力预压帽顶，再沿圆周拖动。30 段拖动把相对螺纹角从 0.0216 圈推进到 0.3076 圈。盖顶法向力超过 0.01 N 的 2364 个子步中，1800 个同时检测到左手对瓶身的相向受力，结束时瓶身保持托盘支撑。无物理警告和手关节越限。记录见 `runs/tea-free-upright-cap-top-drive-adjusted.json`。

继续沿圆周拖动时，实际峰值达到 0.745 圈，最终约 0.732 圈和 1.464 mm 轴向行程。后半段左手丢失相向持力，瓶身倾斜约 62.5°。下一目标会使食指近端触及帽顶或帽壁，未执行这一步。自由瓶尚未退丝，试验未出现 MuJoCo 警告或手关节越限。记录见 `runs/tea-free-upright-cap-top-continuous.json`。

改从累计 0.4146 圈、左手仍对向夹持的实际积分检查点继续。右食指在同一清晰扇区拖动，每轮抬手到盖顶上方再回到起点。五轮实际结束角依次为 0.749、1.084、1.419、1.754 和 1.995 圈。最后一轮达到模型退丝阈值，轴向行程 3.991 mm，`cap_unthreaded=true`。瓶身全程自由，结束时仍由左手相向夹持和托盘共同支撑，较默认起点平移约 4.15 mm、倾斜约 0.11°，未出现 MuJoCo 警告或手关节越限。记录在 `runs/tea-free-upright-cap-top-cycles.json`，实际脱扣积分状态在 `runs/tea-free-upright-cap-top-released.npz`。

从该实际状态继续，右手撤离后按当前盖姿重新对准，食指和拇指夹住盖子并抬高 62.3 mm。最后仍检测到相向抓持，瓶盖已完全与瓶身分离。瓶身仍为自由物体，左手保持先前控制目标。记录见 `runs/tea-free-upright-threaded-cap-lift.json`。

茶壶实际平移后，改用托盘右侧的桌面作为放盖位置。转运中让右腕绕世界竖直轴转 30°，避开原固定腕姿的右臂自碰撞。盖子在到达预定松指点前滑脱，最终由桌面独自承托，静置在 `[0.5137, -0.2248, -0.0160] m`，`cap_on_right=true`。瓶身仍自由并由托盘承托，结束时左手只有拇指与瓶身接触。整段接续同一次真实退丝、抓取和搬运动力学状态，无 MuJoCo 警告或 LEAP 关节越限。记录和末状态见 `runs/tea-free-upright-cap-on-table-reoriented.{json,npz}`。

修正镊子初始高度后，从默认满载自由瓶重新执行整段开盖。五轮帽顶拖动再次达到约 1.995 圈、3.991 mm 并真实脱扣。原右手取盖姿态会碰到左拇指，改为左手先保留指形抬离托盘边沿，再张手撤开，右手相向夹持并抬起盖子约 63.2 mm。把最终平面目标向内移 22 mm 后，盖子在托盘右侧脱手静置，末状态确认由桌面独立承托，`cap_on_right=true`。瓶身仍自由、由托盘承托。退丝、提盖与搬运数据无 MuJoCo 警告或 LEAP 关节越限。记录见 `runs/tea-raised-layout-free-upright-cap-top-cycles.json`、`runs/tea-raised-layout-left-raised-cap-lift.json` 和 `runs/tea-raised-layout-cap-on-table-nearer.{json,npz}`。

从新版放盖后的同一实际状态继续接近镊子桥部。右食指峰值约 0.594 N，镊子在拉动结束时相对初态转动约 43.8°、移动约 5.57 mm，桌上的盖子仍满足右侧静置判据。再撤离右手后，镊子仍由木槽承托，此时未形成有效持具。当前可接续的状态和物理记录见 `runs/tea-raised-layout-post-uncap-tool-{traction,settle}.{json,npz}`。

从该状态按两条弹簧臂的外侧面规划夹取。直接下降时右食指先撞到木槽侧壁。增加指垫净空并在高位换腕后实际到达预抓位，末次腕关节跟踪误差约 0.00021 rad。进一步合拢的候选会先造成两指相碰及食指碰木槽底，未测得镊子载荷。另做一个根据视频比例估计的渐缩镊臂候选，每臂夹持端截面约 11.5 × 7.5 mm，尖端约 4 × 4 mm。完整场景静置 6 s 后仍由木槽承托，平面移动约 1.90 mm，无仿真警告。记录见 `runs/tea-arm-grip-live-pads.json` 和 `runs/tea-tool-tapered-arms-candidate.json`。

从渐缩候选真实落座状态执行同样的高位换腕，食指接触镊臂的峰值约 0.144 N。再从该物理状态沿镊臂错开拇指约 12 mm，取得 109 个 2 ms 子步的相向接触。保存有支撑的实际夹持，按指垫受力继续预载，两侧分别达到约 0.200 和 0.392 N。抬腕时镊子实际只升约 0.35 mm，拇指失去接触，镊子仍由木槽承托。该候选用于独立抓取验证，默认场景使用现有生产镊臂。记录见 `runs/tea-tapered-{arm-grip,tool-stagger,tool-preload}.json` 和 `runs/tea-tapered-tool-pinch-hold.npz`。

从上述历史放盖状态另行恢复，左手增加食指预载后重新产生约 0.761 N 和 1.238 N 的两侧受力。跟随瓶身平面位置，手腕实际升高约 8.42 mm 时仍有对向接触，瓶身最高升约 1.28 mm，始终得到托盘支撑。继续抬腕后拇指沿壁滑脱。瓶盖位移小于 1 µm。记录见 `runs/tea-open-bottle-presentation-xyfollow-baseline.json`。同一放盖状态下，上下夹取原镊子桥的候选与桌面或木槽相交。改用食指从上方接近，到施力前，无名指远端连杆会先碰到桌上的瓶盖，因此没有测到有效镊子载荷。旧摆放的结果见 `runs/tea-post-uncap-{bridge-grip,tweezer-traction}.json`。

利用实际打开的瓶口，左手退到高位后沿瓶轴下探，食指实测接触两处瓶颈内壁，最终法向力合计约 0.326 N。108 片茶粒都留在瓶里。原来外侧拇指在撤手时就失去接触，内外没有形成对向夹持，未提瓶。尝试同时保持拇指朝向时，腕部在入口前会让掌侧与拇指相交约 0.56 mm。记录见 `runs/tea-open-mouth-neck-{physical,route-orientation-preserved}.json`。

保存上述实际食指内壁接触，改在食指进入后再合拢外侧拇指。食指仍在内壁施力，结束时约 0.315 N。外侧拇指间隙一度降至零，未测到对应法向力，瓶子在食指负载下横向滑动约 9.35 mm，仍由托盘承托。108 片茶粒无掉落。记录见 `runs/tea-open-mouth-inner-state.npz` 和 `runs/tea-open-mouth-thumb-physical.json`。

先用口内食指把瓶身推向托盘正 y 边，实际接触反力峰值约 0.40 N，瓶身仍可沿 x 方向移动。改在瓶口内部将食指从正 x 侧移到负 x 侧，令瓶身靠到负 x 边，获得约 0.105 N 向内反力，108 片茶粒均未落出。此时中指若从外侧加入会与瓶壁和口内食指交叠，未执行提瓶。记录见 `runs/tea-open-mouth-rim-thumb-physical.json` 和 `runs/tea-open-mouth-xneg-{seat,middle-fit}.json`。

从侧面施力结束时的实际自由瓶状态另做左手承重试验。右手离开后，左手保持对向受力超过 0.30 s。4 mm 抬腕命令使瓶子实际升高约 2.96 mm，但始终与托盘接触。抬升时拇指侧先滑脱，未形成无支撑搬运。记录见 `runs/tea-free-upright-counterhold-lift-pressure5.json`。

左臂以真实接触把默认满载自由瓶推到托盘壁，最大支撑倾角约 101.43°，壁面峰值约 9.46 N。现行手部参数下，进一步合拢只让拇指碰到瓶身，未建立相向夹持。两手每个 2 ms 物理子步的实测关节均在 URDF 范围内。试验及腕部中点再推进 10 mm 的对照见 `runs/tea-loaded-xneg-rim-ab24e72{-telemetry,-midpoint40-telemetry}.json`。

左手食指改对准托盘内的上半瓶壁，目标接触位置高约 44.5 mm。先垂直抬腕 50 mm 再旋转的完整路径仍使食指网格穿入托盘底面约 47.9 mm，该路线未送进物理控制。记录见 `runs/tea-loaded-b6d8e6d-upperwall-lift-then-rotate.json`。

从同一实际托盘沿支撑状态，改让左手先撤到高位再弯曲食指下探，沿无桌面或托盘碰撞的路线接触上方瓶肋，测得约 2.02 N。进一步合拢时拇指未到达对侧，瓶体继续靠托盘沿支撑，未提瓶。记录见 `runs/tea-loaded-4a8674d-overhand-flexindex-conclusion.json`。

## 六个场景的渲染与接入验证

六个场景均以同一套双 Franka、双 LEAP、OSC 和 RGB-D 入口在 `pro1` 加载，执行实际控制并输出 `10000 × 6` 点云。35 项 MuJoCo 专测和汇总测试通过。场景总览图与接入记录保存于数据盘 `runs/task-scene-overview-grasp-physics/six-mujoco-scenes.png` 和 `runs/task-scene-overview-grasp-physics/smoke.json`。Scissor Cutting 和 Tea Preparing 已验证关键机构的物理响应，仍缺整段双臂任务动作。关闭离屏多重采样后，重新用五个场景各执行一步，并与满载泡茶三步结果一起确认六个场景都输出相同结构的点云，且无物理警告。记录见 `runs/no-msaa-*-entry.json` 与 `runs/tea-production-filled-entry.json`。


## 校正 LEAP 指尖碰撞与惯量

DexCap 的重定向 URDF 比 LEAP V1 官方物理模型多四个 `realtip` 固定链接。它们各带半径 5 mm 的圆球和 0.1 kg 的惯量条目，用于表示指尖 IK 目标。现在构建 MuJoCo 手部时只加载官方物理模型对应的 17 个实体链接。原始 DexCap URDF 仍供发布数据的重建使用。编译后每只手质量由 0.909 kg 校正为 0.509 kg。 [LEAP V1 官方 URDF](https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf)

原掌部 STL 含明显的凹槽。将整个 STL 作为单一碰撞网格时，MuJoCo 用凸包填平凹槽。中性姿态中原有两处拇指与掌部假穿透约为 12.5 mm 和 5.0 mm。对真实三角网格的测量确认这两处接触点均在释放的掌壳外。碰撞掌部现在使用从原 STL 生成的 64 个凸块。主壳体用 CoACD 1.0.14 分为 16 块，其余 48 个连通部件各保留一个凸包。双手中性姿态的原两处接触均消失，另外的真实指节碰撞仍发生，掌部质量仍为 0.237 kg。渲染使用同一原始 STL。运行只读取提交中的分块资产，CoACD 用于离线生成。分解参数和实测点对比在 `runs/coacd-palm-hulls/`，双手实测接触在 `runs/leap-palm-decomposition-ab.json`。

擦盘接盘时又定位到掌部第 13 个凸块与拇指近节的假接触。触点距原始掌部三角面约 5.16 mm、距拇指三角面约 0.04 mm，但旧凸块报告 0.086 mm 穿透和 6.30 N 法向力。将原始掌部三角面裁切到此凸块的原界面，再用保留的表面重建该凸包。双手在同一实测关节姿态下均剩约 0.37 mm 净空，其他 63 块几何及掌部惯量没有调整。52 项 MuJoCo 测试通过，标准入口的 Sponge Picking 仍抬升约 310.5 mm，Ball Collecting 仍完成落篮。擦盘相同接触位置反馈路线越过该假接触，后续受腕部跟踪误差限制，盘仍受支撑。证据见 `/data/dexcap-repro/benchmarks/wiping-c6-thumb-palm-source-contact.json`、`leap-palm-part13-refinement/` 和 `palm-thumb-wiping-merge-validation/`。 [MuJoCo 凸碰撞与网格](https://mujoco.readthedocs.io/en/stable/computation/index.html#convex-collisions)

右腕按实测位置调整目标后，另一姿态仍出现同一区域的假接触。原始掌部三角面距触点约 4.69 mm，旧凸块报告约 0.082 mm 穿透和 6.06 N 法向力。把第 13 块里保留的原始裁切三角面沿其原局部 z 范围中点分在两个凸包内。两组实测姿态在左右手分别获得约 1.51 和 0.91 mm 净空，两个完整场景的对应状态均无该接触。源三角面的覆盖和其余 63 块碰撞体保留。52 项 MuJoCo 测试通过，标准入口再次验证海绵抬升约 310.5 mm，球释放后进入篮内。生成和复核记录在 `/data/dexcap-repro/benchmarks/leap-palm-part13-z2-final/`。

后续动力学续段共测到四组掌部凸包假接触。最后一处报告约 6.80 N，触点距原始掌部三角面约 2.28 mm。将原第 13 块覆盖的裁切三角面沿局部 x 划分为十个凸包，保留所有对应三角面及其余 63 块。左右手在四组实测姿态的最小净空为 2.78、2.48、1.97 和 1.37 mm，四个完整场景状态均消除了该接触。52 项 MuJoCo 测试通过，Sponge Picking 与 Ball Collecting 的标准参考仍成功。生成和测量见 `/data/dexcap-repro/benchmarks/leap-palm-part13-x10-v4/`。

更新后六个任务均能用标准入口加载、控制和输出点云，37 项 MuJoCo 测试通过。原动作在新模型上的实测结果为 Sponge Picking 仍完成，红银包装仍入盒但没有合盖，Ball Collecting 未抬球，Wiping 未满足两笔擦拭。剪刀参考仍抬高约 145 mm，备茶本次只跑了 3 步入口验证。具体结果在 `runs/leap-palm-*-rollout.json`。重新执行隔离提纸时，最低纸段在双侧受力时抬高约 51 mm。两垫各超过 8 mN 且全条无支撑的最长连续区间为 0.37 s。仍存在指垫之间的接触，需要继续校准。见 `runs/scissor_paper_pose_follow_palm-roll.json`。

首次校正 LEAP 之后，两条参考动作按当时的放置与控制参数回放。该版本 Wiping 的右、左腕跟踪误差中位值约为 2.3/4.6 mm，未抓起盘子或海绵。Packaging 红盒的中位误差约为 4.8/5.6 mm，本次未持续抓起、入盒或合盖。校正后的回放见数据盘 `runs/{wiping,packaging}-physical-leap-replay.json`。

另用 Wiping 和 Packaging 的 `demo_0` 分别对照 25 和 22 个录制观测。将已发布的腕姿和当前解码的手关节传给校正后的 LEAP 网格，距离手部 12 mm 内的录制黑点到网格采样点的中位距离均约 0.83 mm，逐帧的 2 mm 以内比例中位数约 98%。这验证了输入腕姿、当前手指顺序和作者预处理点云之间的对应。数据及脚本在 `runs/{wiping,packaging}-released-hand-observation-order.{py,json}`。

Wiping 的前 54 步中，第 24 步食指先撞盘沿。第 28 步仅一个控制周期检测到拇指和另一指的同时接触，盘始终受桌面支撑。第 42–44 步右腕位置误差约 0.2 mm，盘中心相比录制状态已偏离约 71–79 mm。手部保持相向接触是当前的主要缺口。逐步接触法向、支撑和腕姿记录在 `runs/wiping-current-wrist-plate-proof.{py,json}`。

从已经夹到盘沿的声明式试验起点，三处真实指面持续接触，将盘子从桌面抬高 81.8 mm。默认放置时，夹持需要的下侧接触点与桌面等高。另用真实食指压盘内侧斜唇，盘保持桌面支撑并倾至 7.26°，对侧下缘净空峰值 21.1 mm。超过 5 mm 的净空持续 0.30 s，盘随即滑动回落。两组结果分别见 `runs/wiping-physical-plate-grasp-probe.{py,json}` 和 `runs/wiping-plate-tilt-clearance-probe.{py,json}`。


双手支撑对比中，正交左指支点比对侧平夹更容易产生净空。根据盘体实时位姿移动左腕，仍只在实际支点接触下维持 150 ms 的 8 mm 以上净空，未执行插指。目标移动 15.1 mm 时腕部只跟上 6.5 mm，显示这套左手姿态缺少可达余量。数据见 `runs/wiping-plate-{orthogonal,rotating}-brace-probe.json`。右手两指垫支点的静态构型可保留约 7.5 mm 桌面净空。通过正常力限控制从上方接近时，腕部仍偏约 8.1 mm，未建立预期支点接触。记录在 `runs/wiping-right-local-pivot-probe.{py,json}`。从外沿横向接近的三次试验均避开桌面碰撞，最多同时接触两处手部几何。盘仍由桌面支撑，未形成抬升抓持。记录在 `runs/wiping-raised-rim-pinch-motion.{py,json}`。

按录制画面让拇指从上方进入、食指从外侧接近，并在闭合时跟随盘子的水平位移，获得 7 个相向接触采样。右手未碰桌，机械臂碰撞计数为 0。食指实际受力仍来自外沿侧壁，盘心横移约 74 mm，最大抬升约 2.1 mm。记录在 `runs/wiping-source-opposed-plate-relative-motion.{py,json}`。左手独立实验从未接触的默认海绵出发，镜像 Sponge Picking 抓姿后由中指和拇指实际抬起海绵约 38 mm，随后侧滑。三指试验延长了相向持物的采样次数，三处接触始终未同时持续到 0.3 s。记录见 `runs/wiping-left-sponge-mirrored-opposed-replay.{py,json}` 和 `runs/wiping-left-sponge-late-third-pad-clearance-hold.{py,json}`。

对海绵单个残余桌面角点做剥离时，两个接触点合力从约 0.044 N 降到低于 0.012 N，拇指和中指维持相向夹持超过 2 s。继续旋转时两指先滑脱，海绵未完全离桌。记录在 `runs/wiping-left-sponge-contact-peel-corrected.{py,json}`。

让左食指先压盘内缘，盘子绕桌面支点倾斜约 21°。沿右食指插入弧，净空大于 10 mm 的窗口约 0.4 s。固定右腕角度后重新拟合拇指，在盘底净空约 39 mm 时采到右食指在盘底、拇指在盘沿的一次有效相向接触。保持该姿态的试验缩小了盘面平移，仍未延长相向接触。试验未让左手提前释放。见 `runs/wiping-left-index-only-sector26-gap.json` 和 `runs/wiping-sector26-left-assisted-right-fixed35-contact-latch.json`。

从默认物体摆放和无接触机器人预抓位出发，左食指先压盘内缘，右食指进入盘底。待腕部减速后小步闭合右拇指，维持食指和拇指相向受力，左手撤开。按固定世界目标抬升并根据盘姿有限速地调整右腕，盘子完全离桌，仅由右手接触时上抬约 109 mm。相向接触连续覆盖 26 个 50 ms 周期采样，首末间隔 1.25 s。记录在 `runs/wiping-right-plate-lift-reference-trace.json`。


这段持盘动作已导出为 387 步，标准入口记录到 `plate_grasped=true`、机械臂碰撞计数 0 和 `10000 × 6` 点云。HDF5 的 `robot_frame=mujoco_world` 标明初始机器人目标和动作都采用 MuJoCo 世界坐标，`control_cycles_per_action=1` 标明每个动作执行一个 50 ms 的控制周期。试验中把贴近盘沿的动作经 DexCap 坐标往返转换后，回放未能完成持盘，因此参考文件直接保存实际发给控制器的目标。

```bash
python -m simulation.rollout data/derived/wiping-right-plate-lift-reference.hdf5 \
  --task wiping --output runs/wiping-right-plate-lift-reference-rollout.json
```


双手取物试验中，原左手转场使海绵平移约 60 mm，按初始位置抓取时漏抓。将左手转场中段抬高约 45 mm，通过海绵网格的当前位置和偏航角更新接近目标，再用真实接触力小幅闭合拇指和中指。右手持盘和左手持海绵同时离桌，维持相向接触 5 个连续 50 ms 样本，首末间隔 0.20 s。窗口内海绵中心抬高约 33–36 mm，盘心仍高于初始位置约 94–104 mm。盘子在该段下降约 9 mm，下一步需要稳定双持并在盘内走擦拭轨迹。详细记录见 `runs/wiping-two-object-lift-reference-trace.json`。

396 步的双手取物参考在标准入口复跑后，`plate_grasped` 和 `sponge_grasped` 均为真，机械臂碰撞计数为 0，实际擦拭笔画为 0。

```bash
python -m simulation.rollout data/derived/wiping-two-object-lift-reference.hdf5 \
  --task wiping --output runs/wiping-two-object-lift-reference-rollout.json
```


在两手取物后，让右手搬盘时绕盘沿接触点缓慢转腕，左手同时带海绵靠近。连续 23 个 50 ms 样本里两件物体均离桌且有相向接触，首末间隔 1.10 s。两物中心距离由约 281 mm 减到约 197 mm，海绵实际接触盘沿，最靠内接触半径约 128 mm。406 步参考回放有 12 个海绵与盘子的接触周期，峰值总接触力约 6.3 N，机械臂碰撞计数为 0。内盘面判据半径为 90 mm，这条参考的擦拭笔画为 0。记录见 `runs/wiping-rim-rendezvous-reference-trace.json`。

```bash
python -m simulation.rollout data/derived/wiping-rim-rendezvous-reference.hdf5 \
  --task wiping --output runs/wiping-rim-rendezvous-reference-rollout.json
```


将盘子横移提前到提盘阶段，右臂在 5 s 的抬升过程中同时逐渐向海绵移动。逐步完成法向抬离盘沿、向盘内移动和下压后，470 步参考首次在双持离桌时接触内盘面。稳定横移试验连续双持了 3.45 s，延伸到入盘的试验在首次盘内接触前累计有一段 3.25 s 的连续双持。首次内盘面接触位于半径约 60.6 mm 处，海绵与盘面的合计法向接触力约 0.77 N。标准入口记录到 1 个内盘面接触周期、0 个完整笔画和 0 个机械臂碰撞周期。

```bash
python -m simulation.rollout data/derived/wiping-interior-contact-reference.hdf5 \
  --task wiping --output runs/wiping-interior-contact-reference-rollout.json
```


掌心分块前的 Wiping 脚本参考从默认物体摆放开始。双手分别抓起自由盘子和弹性海绵。左手越过盘沿后转腕，让海绵的一侧宽面压向盘内。沿盘内的竖直投影方向擦拭，在第一笔被记录后反向，用实测海绵与盘的接触力调整法向目标。第 31.95 s 和 34.30 s 分别记录向上、向下的有效笔画。686 步标准回放判定 `sim_success=true`，两条笔画满足现有 60 mm 阈值，累计有效接触移动约 195 mm，机械臂碰撞计数为 0。此结果来自脚本动作。逐周期记录位于 `runs/wiping-two-stroke-reference-trace.json`，轨迹生成脚本位于同名 `.py`。

```bash
python -m simulation.rollout data/derived/wiping-two-stroke-reference.hdf5 \
  --task wiping --output runs/wiping-two-stroke-reference-rollout.json
```


新掌心模型下，右手保留盘沿夹持并额外抬高，左手执行镜像后的海绵动作。两物相向受力且同时离桌约 0.75 s，盘子最高抬升约 152 mm，海绵最高抬升约 49 mm，560 动作标准回放确认盘和海绵均已抓起。继续同步抬升并用真实接触力调整手指后，连续同时持物延长到 1.55 s。566 动作标准回放保留了这段动作，机械臂没有碰撞。接近盘面时右手盘沿后续发生滑转，尚未恢复两笔擦拭。记录见 `runs/wiping-palm-extra-rise-q45.json` 和 `runs/wiping-palm-continuous-wipe-ext.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-extended-dual-hold-reference.hdf5 \
  --task wiping --output runs/wiping-palm-extended-dual-hold-rollout.json
```

在复合掌心模型下，同步抬升和靠近把离桌双持延长到 9.25 s。左手沿盘面法向抬起海绵，越过盘沿后下压，并用真实接触力调整擦拭轨迹。817 动作标准回放得到 144 个内盘面接触周期，其中有 114 个采样同时满足双手相向抓持、两物离桌及内盘面法向接触力超过 0.08 N。两物连续离桌双持约 10.35 s。有效擦拭累计约 55.08 mm，尚未达到完整笔画判据，结束前左手食指失去海绵接触。机械臂碰撞计数为 0。逐动作记录见 `runs/wiping-palm-over-rim-f.json`，左手接触失效诊断见 `/data/dexcap-repro/benchmarks/wiping-left-sponge-slip-diagnostic.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-over-rim-f.hdf5 \
  --task wiping --output runs/wiping-palm-over-rim-f-rollout.json
```


调整左手抓取位置，使三指接触点靠近海绵中心后，另一条双手轨迹连续离桌持物约 13.25 s。921 步标准回放记录到 60 个内盘面接触周期，累计有效擦拭约 41.77 mm，机械臂碰撞计数 0。两笔擦拭仍待完成。轨迹生成记录见 `runs/wiping-palm-centered-wipe-d.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-centered-wipe-d.hdf5 \
  --task wiping --output runs/wiping-palm-centered-wipe-d-rollout.json
```


逐控制周期的整机复查确认，两个较长的擦拭片段中断时双手仍夹住物体，海绵与内盘面的接触先消失。两段中断前的连续投影分别为 23.36 mm 和 36.40 mm，均未达到 60 mm。盘内接触半径仍小于 90 mm。证据位于 `/data/dexcap-repro/benchmarks/wiping-strict-reset-{wipe-d,loaded-f}-thread1.json`。

按实测法向力逐步补偿压力后，1121 动作标准回放得到 267 个内盘面接触周期，累计有效擦拭约 60.97 mm，机械臂碰撞计数 0。补偿后的最后 260 个采样都保持双手离桌夹持，其中 258 个采样的内盘面法向接触力超过 0.08 N。连续投影仍未达到完整笔画长度。生成轨迹和逐步数据见 `runs/wiping-palm-sustained-preload-e.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-sustained-preload-e.hdf5 \
  --task wiping --output runs/wiping-palm-sustained-preload-e-rollout.json
```


保留原擦拭的切向目标，只在内盘面即将失去接触时小幅补偿左腕法向位移。标准回放在第 895 个动作完成向上的第一笔，触发时连续投影约 61.73 mm。接触连续保持到第 897 个动作，投影达到约 68.13 mm。双手在触发时分别相向夹住已离桌的盘子和海绵，机械臂碰撞计数为 0。这条轨迹的反向回程未达到 60 mm 的连续阈值。逐动作计分和接触证据见 `/data/dexcap-repro/benchmarks/wiping-first-pass-reversal-diagnostic.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-gap-correction-f.hdf5 \
  --task wiping --steps 897 --output runs/wiping-palm-first-stroke-rollout.json
```


在复合掌心模型中完成了默认摆放下的两笔擦盘。原回程中，左食指直接压到盘底，随后盘子重新碰桌。同期左中指和拇指仍有相向夹持。回程调整让食指退开，由左中指与拇指继续持海绵，同时微调双腕的回程位置和姿态。两份候选轨迹均从初始场景通过标准入口，方向依次为向上、向下。保留 946 动作的轨迹作回放参考，第 895 和 946 个动作分别记为有效笔画，累计有效行程约 136.14 mm。第二笔触发时盘底离桌约 79.2 mm，左手没有直接接触盘子，海绵与盘面接触力约 2.98 N。整段机械臂碰撞计数为 0，`sim_success=true`。轨迹只包含实际发给执行器的目标。证据位于 `/data/dexcap-repro/benchmarks/wiping-search-return-a-physical.json`。

```bash
python -m simulation.rollout data/derived/wiping-palm-search-return-a.hdf5 \
  --task wiping --output runs/wiping-palm-search-return-a-rollout.json
```


在现行止挡模型上，左手改变外缘支点后，对侧盘底间隙可稳定保持约 11.2 mm。撤开右食指并先让右拇指接触后，接触可维持约 1.75 s。把食指在接近前预先合拢到约八成、仍留有净空时，最终获得连续约 5.35 s 的双指相向受力，并覆盖左手完整撤离。盘仍由桌面承托。绕盘沿调整腕姿约 8° 后，拇指接触法向仍主要朝下，腕部上抬时两指脱开，未形成自由抬盘。记录见 `/data/dexcap-repro/benchmarks/wiping-right-plate-lift-stop-{opposed-refit,thumb-first-settle,thumb-first-preshape,oriented-lift}-trace.json`。

继续从盘沿下方接近时，完整拇指侧面先撞到外侧盘壁。最终实测法向力约 11.1 N，竖直分量约为向下 1.3 N，盘仍由桌面承托。在这次左支撑姿态，半径 115–125 mm 处的盘下净空约 30 mm。搜索 800 个腕姿与接触位置后，候选里拇指网格的最小竖直投影厚度约 38.6 mm。出现向上法向的候选同时与盘体穿入约 11.6 mm 或更多。物理轨迹见 `/data/dexcap-repro/benchmarks/wiping-right-plate-lift-stop-lower-rim-route-v2-trace.json`。

改用左手压盘外缘，以自由盘与桌面的实际接触作支点。右侧外缘离桌间隙峰值约 46.7 mm，连续 4.0 s 超过 40 mm。273 动作独立重放的末状态倾角约 7.94°，半径 115 mm 处约有 41.76 mm 净空，半径 125 mm 处为 41.95–44.41 mm，桌面仍有七个实际支点接触且无物理警告。当前已复现撑盘姿态，右手接盘需在这段真实支撑状态上执行。记录见 `/data/dexcap-repro/benchmarks/wiping-left-high-tilt-{support-trace,reference-replay}.json`。

完整拇指放到盘底的候选仍会让右手的其他连杆或掌部碰桌。改用食指从下方承重后，实际测到食指与拇指相向受力，盘的手部净向上载荷峰值约 4.59 N，超过盘重约 2.45 N。受力点仍在距盘心约 133 mm 的外缘。执行过程中食指近端先碰桌，腕误差增大并使盘回落，盘始终受桌面支撑。记录见 `/data/dexcap-repro/benchmarks/wiping-{full-thumb-high-gap-plan,index-under-high-tilt-continuation-trace}.json`。

改用高位切向转移再分段下降后，右手完整网格与桌面始终保持至少约 14.9 mm 净空。首次物理接触发生在盘沿净空仍约 41.3 mm 时，食指近端压到盘上表面，法向力约 0.79 N。随后盘回落，食指尖未到达盘底。记录见 `/data/dexcap-repro/benchmarks/wiping-index-under-high-tilt-staged-trace.json`。

隔离食指收回命令确认数据和执行器映射正确。目标为 −0.269 rad 时，执行器连续 160 周期在 −0.95 N·m 饱和，实测关节仍约 0.984 rad。碰撞发生在同一食指近端与远端网格，也发生在食指与中指网格之间。仅收起中指可让关节最低到约 0.103 rad，尚未到达原目标。保持现有电机上限和碰撞几何，后续改用能实际达到的指形。记录见 `/data/dexcap-repro/benchmarks/wiping-index-q0-isolation.json`。

先伸直食指远端关节，再执行收回动作，目标 −0.269 rad 的实际误差降到 0.00185 rad，连续 200 周期没有食指自碰，保留现行电机力矩。随后为中指与盘、食指之间寻找净空时，一个静态候选分别留下约 10.4 mm、3.7 mm 或更多的间隙。物理转移中，中指两个执行器仍被交叉指节接触推到负向饱和，实际中指首先碰盘，使盘沿间隙降至约 22.8–34.2 mm，未进入接盘下降。记录见 `/data/dexcap-repro/benchmarks/wiping-index-q0-preshape.json` 和 `/data/dexcap-repro/benchmarks/wiping-middle-gap-physical-trace.json`。

把中指和无名指都先伸直，再展开中指，实际关节误差降到 0.00172 rad，且没有手内接触。随后屈指到目标的约 67.5% 时，指节之间重新出现实际碰撞，中指屈曲执行器达到 −0.95 N·m，未能达到夹盘所需的候选手形。记录见 `/data/dexcap-repro/benchmarks/wiping-sequential-preshape-trace.json`。

保持无名指远端伸直，只对中指做部分屈曲，高位转移后的 200 个采样均保持左手与桌面支撑，未发生右手碰盘或碰桌，盘下间隙约 39.75–42.11 mm。实测腕仍偏离目标约 19.9 mm，手与盘网格间最近估计约 0.05 mm，中指近端与远端自身接触约 30 N，因此未下降接盘。记录见 `/data/dexcap-repro/benchmarks/wiping-neutral-ring-acquisition-trace.json`。

取实际约 30 N 自碰时的关节角进行静态几何复核，MuJoCo 在中指近端与远端的单凸包之间报告约 0.982 mm 侵入。导入的原始三角表面在同一姿态下最近约相距 5.94 mm。碰撞和显示网格来自完全相同的 STL、比例及姿态，凸碰撞点到远端原始表面约 6.43 mm，表明单凸包填充了原网格的凹区。原网格并非闭合实体，距离来自三角表面测量。记录见 `/data/dexcap-repro/benchmarks/wiping-middle-collision-geometry.json`。

LEAP V1 食指、中指和无名指共用的 MCP 与 DIP 碰撞网格现各由两块凸体构成。离线生成沿 STL 的三角形边进行精确切片，再对两个半区分别求凸包。MCP 切面沿局部 `x-y` 的投影中点，DIP 切面沿局部 `y` 的投影中点。每块凸体包含对应的所有裁切三角面，双手沿用发布模型的网格比例、左右镜像与关节位置。三个实测姿态的原单凸包假穿透为 0.98、1.56 和 4.45 mm，分块后间隙分别为 4.85、4.52 和 1.54 mm。同一组样本中，中指 DIP 与无名指指尖的有效接触仍报告约 1.89 mm 碰撞深度。渲染继续使用原模型，Wiping 与 Sponge Picking 以不同物理连杆上的相向接触认定夹持。生成脚本与测试数据见 `/data/dexcap-repro/benchmarks/leap-middle-triangle-slices.py` 和 `/data/dexcap-repro/benchmarks/leap-middle-slices/validation.json`。

六个场景均通过标准入口验证，输出 `10000 × 6` 点云，49 项 MuJoCo 测试通过。Wiping 重放 273 步撑盘动作，无物理警告或双臂碰撞。Ball Collecting 的 265 步参考成功将球抓起后释放到篮内。红银包装的原 610 步动作能够关盖，当前回放中物体未抓起或入盒。剪纸与备茶各验证三步入口。记录在 `runs/wiping-leap-finger-parts-high-tilt-rollout.json` 与 `runs/leap-finger-parts-*-rollout.json`。

Sponge Picking 的原 305 步轨迹最高抬升 299.4 mm。在右腕末段平滑增加 12 mm 上移目标后，标准入口回放实测抬升 310.5 mm，通过 300 mm 任务判据，且无物理警告或双臂碰撞。轨迹见 `data/derived/sponge-picking-finger-parts-lift-12mm.hdf5`，结果见 `runs/leap-finger-parts-sponge-lift-12mm-rollout.json`。

新版分块的接盘重放在慢速屈指、高位转移、稳定支撑和逐步下降中均无手内接触。高位稳定区的 200 个采样中，盘下净空为 39.83–42.13 mm，右手未碰盘或碰桌。从实际到达的腕位重新建立下降起点后，沿世界竖直方向逐指令下降 0.25 mm。第 55 个下降采样首次由中指指尖碰盘外沿，实测法向力 0.120 N，接触法向的盘面局部 z 分量朝下。此时盘下净空 41.83 mm，右手最低网格至桌面 32.34 mm，食指尚未从盘底承重。记录见 `/data/dexcap-repro/benchmarks/wiping-neutral-ring-parts-descent-b-trace.json`。

用现有 SetXF 第 3000 轮权重在 GPU 6 和 7 重跑 Wiping 种子 9 和 15。两回合均执行 400 动作，无物理警告，均未抓起盘或海绵，擦拭笔画为 0。此次结果只覆盖这两个摆放。记录见 `runs/leap-finger-parts-wiping-setxf-seed-{9,15}.json`。

两次接盘延伸分别增加中指和无名指的单关节预屈，并从腕部实测位置重新建立下降起点。最新一次完成 103 个各 0.25 mm 的下降指令，右手全程无自碰和桌面接触，盘下间隙保持约 41.72–42.65 mm。终点右腕跟踪误差约 1.35 mm。首次碰盘仍发生在中指指尖与盘上沿，两个接触点合计约 0.112 N，食指仍未在盘底建立承重接触。记录见 `/data/dexcap-repro/benchmarks/wiping-neutral-ring-parts-descent-f-trace.json`。

盘沿碰撞体改为 64 个角扇区 × 5 道径向坡段的凸楔体，四角及底边与显示网格共用，由实测径向剖面生成，壳厚沿盘面局部 z 轴为 2 mm。原 32 个矩形扇区在 179,200 个同址双面射线采样中有 2,528 个位置至少漏掉一面，命中点的视在厚度约为 0.008–3.95 mm。修正后本组全部命中，壳厚均为 2.00 mm，顶面相对连续径向剖面的最大误差约 0.215 mm。

同一段 273 动作撑盘重放中，原版与新版均在末段保持约 80 mm 的盘下净空。新版随后固定保持最后一个腕位命令 200 周期，末态倾角约 21.0°、净空约 80.5 mm、盘与桌面有 9 个实际支点接触，无机械臂碰撞、双手碰撞或物理警告。标准 EGL 入口输出 `10000 × 6` 点云，50 项 MuJoCo 测试通过。此处净空沿盘局部 r125 mm、z14.5 mm 参考线计算。早先约 42 mm 的记录来自较长的接盘续段。几何和重放对照见 `/data/dexcap-repro/benchmarks/wiping-plate-convex-rim/`。

在这一撑盘状态下重新执行右手完整预成形并保持左腕目标，1,112 步实际动作得到无右手接触的规划起点，盘仍由桌面支撑。从这个起点找出五条满足机器臂关节范围及原生接触约束的路线，并用正常执行器重放其中一条 49 路点路线。右食指从下方接触第 2 道径向坡段的第 53 扇区，最大实测盘面局部向上法向载荷为 0.746 N，右手无自碰、无碰桌，盘始终有桌面支撑。记录见 `/data/dexcap-repro/benchmarks/wiping-wedges-clear-preshape-state.npz`、`wiping-index-shared-wedges-arm-approach-b.json` 和 `wiping-index-wedge53-live-c9-trace.json`。

从这次加载后的实测盘姿重新拟合拇指位置。四个拇指关节分段调整后，以局部盘面法线接近上表面，食指和拇指在真实接触中连续相向受力 41 个控制周期、约 2.05 s。食指力约 0.738–0.769 N，拇指力约 0.008–0.031 N。两指接触分别在 `rim_2_53` 的下表面与 `rim_2_57` 的上表面，盘仍由桌面支撑。逐步力记录见 `/data/dexcap-repro/benchmarks/wiping-index-wedge53-live-c9-dynamic-thumb-trace.json`。1,629 步可复放动作存于 `data/derived/wiping-index-thumb-wedge-opposed-reference.hdf5`，标准入口完整回放无物理警告、机械臂碰撞或双手碰撞，结果见 `runs/wiping-index-thumb-wedge-opposed-reference-rollout.json`。

逐级加强拇指预载后，1,665 步动作的末态保持约 1.358 N 食指力和 0.612 N 拇指力，无新增碰撞或物理警告。从该状态继续保持左手撑盘并尝试竖直抬腕。右腕实际升高约 5.87 mm 时，盘心仅上移约 1.24 mm，仍与桌面接触。下一个目标拇指力降到 0.194 N，触发该试验的保持门限。在整个采样中，右手对盘面的局部向上合力峰值约 1.80 N。记录见 `/data/dexcap-repro/benchmarks/wiping-index-wedge53-c9-loaded-opposed-prefix.npz` 与 `wiping-index-wedge53-live-c9-preload-extension-trace.json`。

用真实接触点雅可比闭环调节拇指并跟随盘面转动，右腕上抬命令从 7 mm 延长至 26 mm。实测腕上升 24.93 mm、盘心上升 7.39 mm，桌面仍有三个支撑接触，其竖直反力约 9.58 N。此时左手竖直向下分量约 9.28 N。独立左手沿接触反法线减载试验使桌面反力从约 8.51 N 降至 5.38 N，右食指、拇指仍保持约 1.47/0.78 N 相向接触。1,806 步动作已从初始场景复放，最终接触力与盘位姿一致。记录见 `/data/dexcap-repro/benchmarks/wiping-c9-thumb-feedback-lift-trace.json` 和 `wiping-c9-left-support-retreat-b-prefix.npz`。另一次双臂调整盘倾角的物理回放维持支撑，将左侧盘下净空增至 14.01 mm，仍低于该指尖原网格的最小厚度 21.03 mm。记录见 `wiping-c9-bilateral-flatten-trace.json`。

对实测接触点做摩擦与力矩平衡估计后，改用食指接触较内侧的平盘底、拇指接触外圈上表面。沿着两个预期触点的连线把腕部滚转 8°，重新生成机械臂路线，留出腕部最后一轴余量。从默认场景重放 1,451 步后，食指在盘底实际产生约 1.005 N 向盘面局部上方的载荷，峰值腕位误差约 4.47 mm，无机械臂或双手碰撞。按实测食指位置重新拟合拇指后，静态估计要达到约 5.10/2.92 N 法向力才可平衡盘重及力矩，计算最大指关节力矩约 0.71 N·m，低于模型限值 0.95 N·m。实际轨迹和静态拟合分别见 `wiping-index-flat-base-c6-roll8-physical-trace.json` 与 `wiping-c6-thumb-measured-balance-fit.json`。

实际预载达到食指 4.95 N、拇指 2.70 N，并在设定范围内连续维持 14 个控制周期。保持右手对夹，左腕按实测接触法向退让后，左手与盘脱开，桌面竖直反力由约 6.22 N 降到 0.95 N。此时右食指、拇指法向力分别约 4.15、2.83 N。2,485 步原生动作见 `data/derived/wiping-plate-c6-handoff-reference.hdf5`，标准入口完整复放时无碰撞或物理警告，输出 `10000 × 6` 点云。结果见 `runs/wiping-plate-c6-handoff-reference-rollout.json`，完整力记录见 `/data/dexcap-repro/benchmarks/wiping-c6-two-hand-transfer-trace.json`。

独立右腕上抬及双指闭环续段有少量采样没有桌面或左手接触，按盘沿最低点估计的最大净空仅约 0.031 mm，之后重新受桌面支撑。食指接触从平底附近滑到坡段，基于实际接触点的静态平衡估计随后也失效。闭环控制测到约 5.45/3.00 N 的峰值附近对夹，未稳定悬空。数据见 `wiping-c6-both-pad-feedback-lift-trace.json`、`wiping-c6-contact-anchor-lift-trace.json` 和 `wiping-c6-measured-wrench-history.json`。

再次按实测腕位调整，双块掌部模型的擦盘右手续段从第 2598 步推进到第 2707 步。终点食指和拇指法向力为 5.76 和 3.50 N，桌面与左手仍接触盘子。此时第 13 块与拇指近节在第三组实测姿态产生约 6.32 N 接触，触点距掌部原始三角面约 3.55 mm，距拇指三角面约 0.034 mm。记录见 `/data/dexcap-repro/benchmarks/wiping-c6-palm13-z2-rebased-wrist-trace.json` 和同目录的 `wiping-c6-palm13-z2-source-contact.json`。

在 `af96b28` 的另一条执行器回放中，绕右食指内侧预期接触点与拇指外侧接触点的连线滚转 −4°，可在加大预载时保留更靠内的平底支点。第 2264 步食指在半径约 69.36 mm 的平底承受 2.01 N，半径 90 mm 的盘沿坡段承受约 0.39 N。未滚转的旧路线在高载阶段 3.87 N 均落在半径约 90 mm 处。这个回放已确认食指支点的改变，余下的拇指加载和双手交接需继续验证。数据见 `/data/dexcap-repro/benchmarks/wiping-large-lever-index-roll-load-extension-trace.json`。

加入拇指近节的距离反馈和基于实测盘姿的右腕补偿后，从默认摆放保存 3,001 个执行器目标。标准入口在十块掌部模型上重放并返回 `plate_grasped=true`，没有物理警告、机械臂碰撞或双手碰撞。以 2 ms 物理子步检查支点，右手食指和拇指相向承载，最长连续 969 个样本没有桌面或左手载荷。首末样本相隔 1.936 s，盘下净空约 4.95–15.24 mm。本段验证右手抬盘并持有，海绵留在桌面，擦拭笔画为 0。轨迹见 `data/derived/wiping-plate-c6-right-attitude-reference.hdf5`，逐子步记录见 `/data/dexcap-repro/benchmarks/palm-x10-merge-validation/wiping-attitude-2ms.json`。

单独重放 Wiping 的左手海绵拾取时，海绵由左中指和拇指相向夹持，从默认桌面抬升 39.64 mm。抬升超过 30 mm、对夹且无桌面接触的连续观测跨度为 3.5 s。标准入口重放前 162 步时 `sponge_grasped=true`，无碰撞或警告。右臂停在远处，盘位移约 0.007 mm。这段证据覆盖海绵拾取。数据见 `data/derived/wiping-left-sponge-isolated-current-reference.hdf5`，结果见 `runs/wiping-left-sponge-isolated-current-rollout.json`。

左手采用同一默认海绵摆放，在腕位约 `[0.571, -0.052, 0.136] m` 起步后分段抬高、横移、下降和夹持。128 个 50 ms 控制周期共 6.40 s。标准入口返回 `sponge_grasped=true`，无物理警告、机械臂碰撞或双手碰撞。右腕在独立试验中停车。轨迹见 `data/derived/wiping-left-sponge-fast-reference.hdf5`，回放和生成脚本在 `/data/dexcap-repro/benchmarks/`。

重新核对抬盘参考的左手退让段后，发现原折叠指形存在同手食指自碰。力最大的接触约 19.34 N，触点距离两个原始网格三角面分别约 0.325 和 0.336 mm。改左手食指和中指关节并配合腕位后，终点中指仍对盘施加约 0.64 N，且原自碰已清除。此后的横移让食指保持避让。实测接触、调姿和手形拟合分别见 `wiping-c6-x10-left-sponge-transfer-a-left-self-source.json`、`wiping-c6-x10-left-sponge-transfer-d-trace.json`、`wiping-left-sponge-inactive-index-fit.json`。

保持左食指避让后，3,049 个执行器目标的标准回放中，右手持续承载离桌的盘子，左手同时完成进场和海绵闭合。2 ms 子步记录中，连续 2,149 个样本只有右手承盘，时间跨度 4.296 s，最大盘下净空 25.62 mm。左手中指和拇指曾在同一子步出现相向接触，共 212 个子步，海绵仍由桌面支撑。标准入口验证 `plate_grasped=true` 和 `sponge_grasped=false`。动作见 `data/derived/wiping-plate-hold-left-sponge-approach-reference.hdf5`，逐子步接触见 `/data/dexcap-repro/benchmarks/wiping-sponge-approach-2ms.json`。

对照两条回放的实际触点，双持接续中左手中指压在海绵顶面，法向几乎竖直向下。单手拾取在同一阶段由中指贴海绵侧面，中指外展关节实际约为 −0.44 rad。双持接续原来仍命令约 −0.85 rad，改为跟踪单手回放实际测得的中指与拇指角度，保留左食指避让。3,107 步标准回放从默认物体摆放开始，只有 46 维执行器目标。2 ms 子步记录确认双手连续 1.284 s 分别相向持有已离桌的盘和海绵。海绵最高抬升 52.1 mm，该区间盘下净空至少 70.9 mm。标准入口无物理警告、机械臂碰撞或双手碰撞。轨迹见 `data/derived/wiping-dual-object-hold-reference.hdf5`，实测触点、指形和验证记录在 `/data/dexcap-repro/benchmarks/wiping-coordinated-grasp/`。

双持补偿末段沿现有接触小幅让开右腕关节止挡，同时提高盘下净空。左腕保留海绵持姿接近盘面，下降时补偿中指和拇指实际夹持力。标准入口从默认摆放重放 3,240 个动作，记录 16 个内盘面接触周期，有效擦拭累计 31.79 mm，峰值海绵与盘接触力 0.837 N。计分仍为 0 笔，当前单笔投影要求 60 mm。原动力学续段再前进时，盘倾斜约 66°，右拇指卸载，桌面重新承受约 1.80 N。该时刻右腕第七轴仍有约 0.087 rad 余量。动作见 `data/derived/wiping-dual-hold-interior-contact-reference.hdf5`，标准回放与续段载荷见 `/data/dexcap-repro/benchmarks/wiping-coordinated-grasp/full-interior-contact-rollout.json` 和同目录的 `plate-support-endpoints.json`。

逐接触测得，原路径的海绵先压住远侧盘沿。右拇指受力点从距盘心约 125 mm 滑到约 134 mm，随后卸载。单独保持左腕时，右无名指可从盘下提供约 1.89 N 的额外向上载荷，但该分支在 1.75 s 时失去左侧海绵夹持。改为保留原右手执行器序列、保持左腕世界朝向，并先将海绵移到内盘面上方。3,266 步标准回放在默认摆放完成第一笔，记录 48 个有效内盘面接触周期，累计距离 64.66 mm，峰值接触力 0.653 N，两手在计分时仍分别相向持有离桌物体。全程无求解器警告、机械臂碰撞或双手碰撞。回程试验中右拇指沿盘沿向外滑并卸载。提高往返速度或增加辅助支撑的分支能延长右手持盘，但左手中指可能先失去海绵接触。仍未达到第二笔。右手保持住盘的一个分支中，左腕目标单步由约 2 mm 跳至 11.21、20.63 mm。随后海绵和盘面约 1.20 N 的接触伴随左中指卸载。海绵倾角在后三个动作从约 37° 变到 78°，拇指也卸载。保留接触力、关节和位姿实测数据于同目录的 `left-return-contact-audit.json`。回放和逐周期记录在 `/data/dexcap-repro/benchmarks/wiping-coordinated-grasp/stroke-support/`。 接续把左腕单步平移限至 2 mm 后，双持仍只维持 0.60 s。按实测接触法向做双腕反馈，发现盘上两组接触斑块相距约 130 mm，法向速度方向相反。同步调整保持双持约 0.65 s，盘体仍产生约 20° 滚转误差。绕外侧接触转动海绵后，内侧斑块卸载，单斑块维持 0.45 s，峰值法向力约 2.56 N，实测接触力矩峰值约 0.202 N·m。后续盘体下沉，未形成第二笔。逐步结果见同目录的 `left-return-{bounded-continuation,coordinated-contact-axis,single-patch-pivot}.json`。

按实测外侧接触力矩轴分配右手接触，最佳短段中食指、无名指和拇指分别提供约 −0.235、−0.083、+0.239 N·m，海绵约 +0.076 N·m，50 ms 端点轴向残差约 −0.0024 N·m。2 ms 子步也测得约 −0.0013 N·m 的残差。无名指接触力矩改变方向，双持约 0.40 s 后拇指仍卸载。沿半径最大约 81 mm 的盘内路线拟合 60 mm 回程时，三种右手支撑方案在海绵只内移约 4 mm 时均失去左手夹持。失效前后海绵接触点跳动约 46 mm，当时右手仍持盘，盘底净空约 50.7 mm。下一阶段需要调整左手夹持海绵的受力位置。数据见同目录 `left-return-right-contact-redistribution.json` 和 `left-return-stable-native-inner-route.json`。

```bash
python -m simulation.rollout data/derived/wiping-first-physical-stroke-reference.hdf5 \
  --task wiping --output runs/wiping-first-physical-stroke-rollout.json
```

剪刀 334 步参考在紧邻分块提交前后的相同布局分别抬升 4.00 和 4.04 mm，未形成双手柄自由夹持，均无碰撞或物理警告。历史抬升 117.25 mm 的实验使用了 y 方向相差约 18.3 mm 的初始剪刀位置。现行布局需要重新规划接近开孔的手腕和指形。对照见 `runs/scissor-e941e61-matched-334.json` 和 `runs/leap-finger-parts-scissor-carry-rollout.json`。

用现行剪刀位姿直接穿固定环，12 个腕部滚转方向均可把食指完整截面送入开孔。最少干涉的 330° 方案仍使无名指尖最深穿桌约 43.87 mm。以实测固定刀刃与托盘底的接触点为支点，静态转动约 21° 后开孔中心抬高约 57.05 mm，手部通过完整截面且无干涉，剪刀仍有真实几何支撑。对应的剪刀枢轴下侧需跟随上移约 32.2 mm。下一阶段需要执行器实际触碰枢轴并建立该转动。证据见 `runs/scissor_current_roll330_blade_floor_pivot_feasibility.json`。

右食指接近枢轴下侧时，刀片和刀柄挡在前面。改用托盘外的固定环第 10 段，指腹刚触及该段时，食指近端与第 13、14 段相交约 1.64 和 2.71 mm。该方案的腕部逆运动学可达，完整手部仍需换入手方向。证据见 `runs/scissor_current_right_outer_handle_lift_search_tangent.json`。

在最新托盘布局试验中，旧侧向路线先撞隔板，未到达剪刀柄。改由上方收拢手指后，右食指在固定柄上方连续施力约 0.288 s，峰值 0.701 N，过程中无其他机器人接触。固定环向托盘内侧约移动 0.205 mm，结束时离邻墙仍约 0.915 mm。此段验证了合法的顶部接触，刀柄孔仍未套入手指。记录见 `runs/scissor_1ccb0a2_overhead_shank_recovery/dynamic_top_drag.json`。

在十凸块掌部模型中，剪刀柄顶部的位姿反馈路线把固定环向内移动约 0.28 mm。托盘底、隔板与邻墙仍提供三点支撑，环侧间隙仍小于 1 mm。从上方接近露出的柄侧面时，完整食指先碰椭圆环第 19 或 20 段，最早测到 0.14–0.93 mm 侵入。该侧面路线在离线阶段终止。动态顶部接触和静态侧面扫掠在 `runs/scissor_66b836c_shank_feedback_recovery/`。

改从上方用左食指压固定刀片侧面，首次真实接触约 1.01 N，侵入约 0.027 mm，未出现其他机器人碰撞。加载随后由托盘底和柄侧邻墙共同承受，活动柄没有抬升，固定环仅向内约 0.096 mm。逐动力学子步见 `runs/scissor_ed8bba8_left_blade_lever_recovery/`。

右食指接近托盘墙上方露出的活动柄下侧。调整未参与操作的指关节后，使用同一双臂、手和任务几何的关节控制诊断执行 3,534 个 2 ms 子步，记录到 927 个右食指托柄接触子步，峰值法向力 0.0503 N，无其他机器人接触或求解器警告。活动环净抬升约 5.00 mm，剪刀始终受托盘承托，尚未提起剪刀或剪纸。记录见 `runs/scissor_89a2f80_moving_handle_underlift/underhandle_dynamic_final.json`。 沿同一路径把抬升目标延至 15 mm 后，活动环、固定环和工具基座分别实测抬升约 15.02、3.08、4.89 mm。806 个子步记录到食指接触，峰值约 0.130 N。结束时活动刀片与托盘底和桌面仍接触，固定刀片还与前隔板接触。抬升目标增至 30 mm 时，活动环实际抬升 20.96 mm，工具基座抬升 8.14 mm，食指峰值载荷 0.156 N。三处刀片支撑仍在，未脱离托盘。三条续段见同目录的 `underhandle_dynamic_continuation{15,30,40}.json`。继续加大抬升时，该诊断中的活动环达到 40.03 mm，工具基座抬升 17.70 mm，食指峰值约 0.181 N，刀片仍由桌面和托盘承托。

上述关节控制诊断自行编译场景，采用默认摩擦锥与桌面参数，并从预先沉降的工具姿态开始。在项目正式 `MujocoScene` 中保留正式桌面摩擦、椭圆摩擦锥和默认工具位置，再通过执行器沉降并重放同一关节路径。第 1,646 个子步右手中指末节先与固定环第 17 段接触，法向力约 0.0931 N，侵入约 0.0098 mm，未到活动柄。正式场景的复验位于 `runs/scissor_89a2f80_moving_handle_underlift/scissor-move40-production-control.json`。 对这个实测姿态求中指关节对碰撞间隙的导数后，将其第二关节目标降低 0.020 rad，再次用正式场景和受限关节力矩运行。活动环净抬升 40.01 mm，固定环净抬升 19.54 mm，工具基座净抬升 18.62 mm。789 个 2 ms 子步有预期食指接触，峰值 0.200 N，无其他机器人接触或物理警告。结束时刀片仍由桌面、前隔板和托盘底承托。生产模型结果见同目录的 `production_middle_joint5_minus020.json`。

每 50 ms 取一次受限关节控制的实测腕位和手形，转换为标准 46 维动作后，从默认摆放完成 214 步腕位控制回放。剪刀基座最高抬升 16.87 mm，无物理警告、机械臂或双手碰撞。剪刀仍未完全离盘，纸张未抬起，未切开纸张。逐步数据和标准结果位于同目录的 `scissor-move40-standard-source.json` 和 `scissor-move40-standard-rollout.json`。

进一步加大抬升目标后，生产场景的活动柄实际峰值为 42.68 mm，固定环约 22.75 mm。腕位误差扩大到约 47 mm 后，右食指与活动柄失去接触。2 ms 动力学状态和执行目标保存在同目录 `production_middle_joint5_minus020_lift60_peak_{checkpoint,targets}.npz`。峰值时固定环下缘高出托盘外壁约 23.74 mm，可供指尖截面的毛余量约 2.11 mm。完整左手和机械臂的进孔拟合仍有约 8.68 mm 的掌部桌面侵入和约 1.30 mm 的食指近端与固定环侵入。以真实活动刀片支撑点为枢轴，静态转动 5.75° 可让开掌部，近端与环仍侵入约 3.74 mm。两次执行器试验中剪刀沿相反方向转动后脱开。证据见同目录的 `dual-ring-fullmesh-allpair-fit.json` 和 `dual-ring-pivot-*`。这些结果用于定位左手接取所需的额外净空。

在无状态改写的抬柄中段，实际活动柄抬升约 20 mm。尝试在固定柄杆下方增加左手支点，该表面法向可提供约 0.133 N·m/N 的正向力臂。联合调整双 Panda 和两个接触手指后，保留有效触点的较好解仍有约 14.55 mm 的双臂重叠。继续让右肘让位约 39 mm 的解，两个接触点均已离开目标。证据在同目录的 `dual-ring-two-contact-summary.json`、`dual-ring-left-press-summary.json` 和 `dual-ring-coupled-summary.json`。

```bash
python -m simulation.rollout data/derived/scissor-cutting-moving-handle-reference.hdf5 \
  --task scissor-cutting --output runs/scissor-cutting-moving-handle-rollout.json
```

固定和活动椭圆柄环的碰撞体使用共享边界的 24 和 20 块凸楔体。内侧边与标称椭圆相切，保留 4 mm 厚度以及现有被动铰链。原矩形分段按每个柄 1,440 个角度测量，固定环和活动环最多向开孔内侵入约 0.57 和 0.30 mm。在标称内半径的 99% 位置，原版分别误挡 838 和 912 个采样，改后均为 0。新版的多边形顶点相对标称椭圆往外约 0–0.34 mm。标准动作回放完成 334 步，剪刀仍只抬升约 4.08 mm，无物理警告。托盘支撑、两环位置与纸带稳定性现有回归通过，51 项 MuJoCo 测试通过。证据见 `/data/dexcap-repro/benchmarks/scissor-handle-box-ray-measurement.json` 与 `scissor-continuous-oval/`。

红银包装的旧接触模型在第 125 步左右靠食指和拇指的相向法向夹住包装物。现行模型同一阶段仅食指接触，物体随后被推开。拇指微闭合和腕部补偿两种尝试都未恢复抓取或入盒。另一条旧参考在前 463 步使用同一动作，差分发生在撤手关盖阶段。对照及补偿回放在 `/data/dexcap-repro/benchmarks/runs/red-610-*-grasp-trace.json` 和同目录的 `red-610-triangle-*-retune-*-rollout.json`。

另用 60 mm 高位路径到达新的打开手形，接近阶段包装仅移动约 0.108 mm。首次触碰时食指尖离包装还有约 18.22 mm。把固定腕姿倾转约 23.6° 可在静态把两指配到相向，但食指近端和中指网格会侵入包装约 3.9–6.4 mm。这种姿态未执行动态回放。实际高位接近与倾角几何记录见 `/data/dexcap-repro/benchmarks/runs/red-live-4-16-high-approach-trace.json` 和 `red-tilted-4-16-pregrasp-fit.json`。

中指与拇指替代抓法仍让多段指节穿入包装。左臂的桌面滚转尝试在接触前失去腕位跟踪。红银包装的挂耳槽几何为 12 × 28 mm，当前指尖横截面无法按测试姿态穿槽。对于挂耳外缘的上拉方案，在对侧长边为支点的静态估计中约需 0.57 N 向上载荷。对应数据见 `/data/dexcap-repro/benchmarks/runs/red-left-supported-roll-trace.json` 和 `red-hanger-hook-geometry.json`。

红银包装的挂耳路线在静态关节路径上保留约 0.198 rad 的腕部第七轴余量。换算为腕位目标并实际执行后，第六轴先碰硬止挡，随后第七轴也逼近限位。停止时腕位误差约 14.1 mm，尚未碰到挂耳。当前笛卡尔控制器的冗余自由度回归默认姿态，这次执行偏离了静态逆解的关节分支。证据见 `/data/dexcap-repro/benchmarks/runs/red-hanger-branch-detour-plan.json` 和同目录的 `red-hanger-branch-detour-trace.json`。

用真实力矩边界下的关节跟踪重放包装挂耳高位路线，前 99 段无接触，腕部第六、第七轴保留约 0.16、0.24 rad 余量。继续保持近挂耳姿态后，无名指末端同时接触纸盒两侧壁，尚未触到包装挂耳。最终腕位误差约 5.62 mm，两轴力矩均未到限值。证据见 `/data/dexcap-repro/benchmarks/runs/red-hanger-joint-detour-dynamics-trace.json`。

分别调整无名指和拇指目标后，右食指经同一受限关节路径接触红银包装挂耳，无右手自碰或盒壁接触。沿实测接触方向斜向续推约 6 mm 后，挂耳法向载荷约 0.172 N，作用在包装物上的竖直载荷峰值约 0.158 N，低于此前估计的约 0.566 N 翻滚门槛。包装物最高抬升 0.36 mm，仍受桌面支撑。见 `/data/dexcap-repro/benchmarks/runs/red-hanger-diagonal-load-6mm-trace.json`。

红银包装初始底面贴桌，约 21.1 mm 的实际食指最小截面无法从下方进入。沿挂耳切向推移时，实测峰值法向力约 0.180 N，包装物仅移动约 0.10 mm。另拟合左手从上方接近主盒的路线。双手可以在同一初始动作下合法复位，实际左臂行进时，无名指末节与掌部产生约 2.24 N 的自碰载荷，因此在接触包装之前停止。记录在 `/data/dexcap-repro/benchmarks/runs/red-hanger-away-push-trace.json` 与同目录的 `left-overhead-red-bimanual-prepose-native-contact.json`。

复核左臂的整条载荷路径后，确认旧待机姿态的右臂阻挡左臂。受力最大的双臂接触约 11.48 N，另有右臂与左掌间约 6.45 N 的接触。让右臂在默认关节姿态待机，保持已测的包装摆放，用受限关节力矩跟踪左臂，左手在第 93 个目标接触包装顶部，初始法向力约 0.881 N。随后沿远离纸盒方向续推，包装净位移约 13.87 mm，其中朝远离纸盒方向约 8.34 mm，转动约 15.58°，峰值法向力约 1.23 N，机械臂碰撞计数为 0。此为 MuJoCo 物理诊断过程，使用左臂关节目标，尚未恢复现行手模型下的完整入盒动作。原碰撞与续推数据见 `/data/dexcap-repro/benchmarks/runs/left-overhead-red-full-constraint-audit.json` 和同目录的 `left-overhead-red-carton-away-active.json`。

将同一动作的实测腕位换成 46 维标准输入后，从默认包装摆放重放 159 个动作。实测包装平移约 13.04 mm、转动约 14.87°。54 个动作端点记录到左手载荷超过 0.015 N，峰值约 1.08 N。重放无物理警告、机械臂碰撞、双手碰撞或关节越限失败。该文件覆盖包装重新摆位，包装尚未被右手夹起。接触和位移数据在 `/data/dexcap-repro/benchmarks/runs/left-overhead-red-carton-cartesian-observed-trace.json`。 继续保持末目标 2 s 时，包装额外移动约 0.92 mm，左手仍有约 0.087 N 支撑载荷。执行向上退让时，左手瞬态接触载荷约 0.307 N，包装回退约 12.8 mm。此阶段的位姿依赖左手支撑，后续右手接近需要保留该支撑或换退出方向。对照见 `/data/dexcap-repro/benchmarks/runs/red-reposition-fit-causal-control.json`。

重新拟合右手食指与拇指时保留左手支撑并让开左拇指。269 步标准回放末端左无名指仍承受约 0.066 N。用实测预抓状态和整臂碰撞间隙规划右侧关节路线，受限力矩重放达到预抓位，右手两指约为 1.69 和 1.80 N，对向法线点积约 −0.918。继续上提时，挂耳和左中指一度仍在承载。左手退出后的 2 ms 连续检查中，右手独立相向夹持且桌面和左手载荷低于 0.005 N 的最长区间为 9 个子步，即 18 ms。随后挂耳重新承重，继续上提时两指滑脱。把关节路线转为标准腕位目标做对照，912 个动作记录到 151 个右手与包装接触，仍出现双手碰撞，未形成稳定提起。在右手短暂独立承托的实际腕姿下，拟合非抓取指时最接近的无名指尖网格仍距包装约 28.6 mm。几何结果见 `carton-third-real-mesh-surface.json`。物理与标准数据位于 `/data/dexcap-repro/benchmarks/runs/` 下的 `carton-native-dual-margin.json`、`carton-right-only-substep-physical.json`、`carton-standard-grasp-candidate-rollout.json`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-supported-transfer-prefix.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-supported-transfer-rollout.json
```

```bash
python -m simulation.rollout data/derived/packaging-red-carton-left-reposition-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-left-reposition-rollout.json
```

复核两侧力方向发现，先抬红银包装的旧分支中，左中指曾对包装施加约 1.48 N 的向下分力，同时挂耳被压在桌面。改为右手建立相向夹持后先让左手退出，再开始右腕上提。受限关节控制的 2 ms 记录确认右手独立承托连续 935 个子步，即 1.87 s，包装中心抬升约 54.94 mm。把实测腕位转换为标准 46 维动作后，从默认摆放重放 793 步，连续独立夹持 740 个子步，即 1.48 s，末端包装最低点离桌约 7.44 mm。更快的腕位上提接续共 857 步，独立夹持最长 2,340 个子步，即 4.68 s，末端包装最低点比纸盒上沿高约 35.7 mm。两条标准回放均无机械臂碰撞、双手碰撞或物理警告。实测标准积分状态和载荷位于 `/data/dexcap-repro/benchmarks/runs/` 的 `carton-standard-lift-state.npz`、`carton-default-rim-state.npz` 和对应的 `*-substep-audit.json`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-above-rim-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-above-rim-rollout.json
```

保持上提的同时让左臂退开后，横移开始时双指切向力达到摩擦极限，手指实际电机力矩仍有余量。提前预紧后，在相向夹持且包装底高于盒沿时，标准腕位控制曾向盒方向运输约 157 mm。加入手指沿包装面的锚点反馈后，向盒方向约 169 mm 后仍失夹。失效前右食指的接触点到达包装长边末端。证据见同目录的 `carton-preloaded-slip-audit.json`、`carton-standard-carry-face-anchors.json`。

另外从开放纸盒上方进入，左手通过内侧壁把盒体推向工作区中心。标准回放 1,282 步，盒体平移约 31.85 mm，距工作区中心的最小距离减少约 28.78 mm。盖板保持打开，无手到盖板接触、机械臂碰撞、双手碰撞或物理警告。原始盒坐标接触力和动作位于同目录 `box-centering-wall3-following-loaded-prefix.json`，单独重放保留该接触阶段。随后失去内壁接触的试验见 `box-centering-wall3-following-to70-extended.json`。 从实测末态抬起左手，按当前盒位和偏航重新进入后，2,333 步标准回放使盒体共平移 44.72 mm，距工作区中心减少 38.00 mm，最终约为 103.24 mm。盖板接触、碰撞事件和物理警告均为 0。重入动作在 `box-centering-wall3-reentry-loaded-prefix.json`。

```bash
python -m simulation.rollout data/derived/packaging-box-centerward-reentry-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-box-centerward-reentry-rollout.json
```

从默认摆放连续执行三次重入后，5,752 步标准回放把外盒平移约 85.07 mm，距工作区中心从 141.24 mm 减至约 81.76 mm。两次额外重入在连续动力学试验中进一步减至 76.51 mm。三次重入的标准动作接上高位过渡与红盒拾取后，共 7,143 步。外盒保留推移结果，末端右手仍独立抬持红盒，包装最低点离桌约 97.64 mm。2 ms 审计记录最长独立夹持 2,183 子步，即 4.37 s。标准回放无机械臂碰撞、双手碰撞、关节越限失败或物理警告。额外重入与组合回放记录分别位于数据盘 `benchmarks/runs/box-centering-wall3-final-two.json` 与 `benchmarks/runs/box-centering-and-red-lift-standard.json`。

```bash
python -m simulation.rollout data/derived/packaging-box-centering-and-red-lift-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-box-centering-and-red-lift-rollout.json
```

另从默认摆放把右食指和拇指的夹点移至红银包装长边中段，两侧法线相向。先撤左手，再用右手抬升并越过盒壁，夹持下降至盒底上方约 2 mm 后松手。1,144 步标准回放复现了盒底承托和 `object_in_box=true`，连续独立右手夹持 7,706 个 2 ms 子步，即 15.41 s，无机械臂碰撞、双手碰撞、关节越限失败或物理警告。末态开盖，包装上缘仍比盒沿高约 18 mm。标准回放与接触审计位于数据盘 `benchmarks/runs/carton-supported-deposit-standard{,-audit}.json`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-supported-deposit-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-supported-deposit-rollout.json
```

**红盒早期闭盖回放。** 从同一次默认摆放开始，左手先推移外盒，右手用包装中段的相向夹点拾起红盒并运到盒口。腕位翻转后，红盒在盒底以宽面稳定，上缘距盒底约 38 mm。右手撤高并停到安全位置，左手通过折舌使盖板越过竖直位置，再沿盖沿完成合盖和撤离。7,665 个动作的标准回放得到 `object_in_box=true`、`lid_closed=true` 和 `sim_success=true`。终态外盒平移 81.10 mm，距工作区中心最近时缩短 59.48 mm。撤手后盖角约 180.31°。机械臂碰撞、双手碰撞与物理警告均为 0。该回放覆盖红银包装的固定参考摆放。完整指标在数据盘 `benchmarks/runs/packaging-red-carton-complete-standard.json`，收尾接触及逐步盖角在同目录 `box-flat-carton-live-closure.json`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-complete-mujoco-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-complete-rollout.json
```

按论文的闭盖协作步骤，从同一平放入盒前缀继续让右食指抵住外侧箱壁，同时由左手拉折舌并合盖。首次从侧壁稳盒时，盖板在约 67.5° 处触及右中指。用同一实体盖板和折舌扫掠筛选右手姿态后，8,364 步标准回放从默认摆放完成推盒、入盒、右手稳盒、左手合盖和两手撤离。箱壁与右食指、盖板与左手同时持续受力 2,093 个 2 ms 子步，为 4.186 s。右手对箱壁峰值约 1.55 N，右手全程未触盖，机械臂碰撞、双手碰撞和物理警告均为 0。最终 `sim_success=true`。标准回放和逐个物理子步的受力位于数据盘 `benchmarks/runs/red-box-closing-right-brace-standard{,-audit}.json`。 [论文动作顺序](https://arxiv.org/html/2403.07788v2#A1.SS6)

```bash
python -m simulation.rollout data/derived/packaging-red-carton-bimanual-closure-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-bimanual-closure-rollout.json
```

在备茶的独立渐缩镊子试验中，现有手部网格用接触力反馈获得食指约 0.190 N、拇指约 0.220 N 的相向预载。抬升阶段维持相向接触 465 个物理子步，托架法向力始终至少 0.149 N，实际最大上移约 0.272 mm，仍未离架。记录见 `runs/tea-convex47-tapered-feedback-lift-v2.json`。

对现有工具另试中指与拇指夹持，在动力学回放中有 296 个 2 ms 子步出现相向接触，最佳同时法向力约 0.103 和 0.030 N，托架仍承重，随后拇指脱开。在默认镊子上从侧面尝试滚动未建立手部接触，工具仍由原托架支撑。证据见 `runs/tea-convex47-alt-digit-physical.json` 和 `runs/tea-tweezers-top-tangent-production-cradle-roll-a.json`。


备茶在默认镊子和木托架上进行连续两次推移。首次右食指与镊子的接触峰值为 1.12 N，卸载后镊子沿世界 y 轴移动约 18.79 mm。由实际到位的镊子姿态重新接近后，第二次接触峰值为 0.505 N。撤手稳定后镊子相对原姿态转动约 38.40°，总水平位移约 44.7 mm，托架仍承担约 0.089 N，茶盘底未接触镊子。第二次推移无机械臂碰撞或物理警告。逐步记录见 `runs/tea-production-cradle-roll-contact.json` 和 `runs/tea-production-cradle-slide-extension.json`。

用圆壁更新前的实际推移末态复核备茶木托。木托底已经与托盘外侧边接触，穿入约 0.20 mm。六组右手高端夹持候选均受整臂可达性或托盘边缘限制，左手固定木托的 18 个候选也没有找到合法路线。沿托盘中心推移的试验会使木托离开现有支点。拟合和支撑数据在 `runs/tea-production-exposed-highend-fit.json`、`runs/tea-offline-left-cradle-hold.json` 和 `runs/tea-offline-cradle-rim-arrest.json`。

将原工具接触腕位变换到实测倾转约 38° 的镊子末态后，沿托盘外侧的 55 个候选腕位可达。加入原推指所需的 8 mm 下压后，中指先与木托外端和侧壁分别侵入约 1.97、1.50 mm。只重调中指可避开木托，但会触及其他指链。保留当时托盘、木托和工具的真实几何，未在该路线执行动态推送。见 `runs/tea-ed8bba8-outward-static.json` 与 `runs/tea-ed8bba8-link8-retune.json`。

保留右食指接触点，以腕工具轴旋转约 −15° 后，在该镊子摆放上得到 3 条完整手部静态无干涉路径。筛选覆盖高位预姿、下降和沿 −Y 移动约 25 mm 的 51 个位置。备茶木托、托盘和镊子支撑未在该静态试验中移除。物理回放用于确认负载下的接触和离架。静态数据见 `runs/tea-89a2f80-index-pivot-static.json`。 两次真实推移后衔接这条腕位路线，标准 OSC 的预抓和下降末端误差分别约 17.18 和 18.15 mm。手与镊子接触力为 0，后续外推未执行，木托仍承担约 0.0923 N。记录见 `runs/tea-89a2f80-index-pivot-physical.json`。受限关节力矩接续首先保存从默认摆放所得的第一段工具状态，供复用的积分状态见 `runs/tea-pivot-joint-pd-stage1-first-slide-settled.npz`。 第二次推移的全场景积分状态也已保存为 `runs/tea-pivot-joint-pd-stage2-second-slide-settled.npz`。在该实际姿态上，原 −15° 路线的拇指末端与桌面静态侵入约 3.40 mm。调整拇指后静态扫掠通过，实际接近出现约 0.423 rad 的最大关节跟踪差，右食指没有给镊子施力。记录见 `runs/tea-pivot-joint-thumb-clear.json`。 根据实际关节误差限速后，最大跟踪差降至约 0.060 rad。镊子在完整到位前仍在木托内移动，手未给镊子施加载荷。记录见 `runs/tea-pivot-joint-rate-follow.json`。

从备茶第一次推移后的实测姿态出发，重新进入镊子高端，使右食指—工具接触峰值达 0.316 N。后续保载超过 2.39 s，末端木托仍承担约 0.232 N，茶盘底载荷为零。该腕位下拇指无法接触工具，尚无相向夹持或离架。轨迹和接续积分状态位于 `runs/tea-pivot-joint-stage1-{grip,pinch}.json` 和相应的 `*.npz`。

用当前生产模型重新从满载自由瓶和上紧的瓶盖起步。左手先在托盘承重下建立相向夹持，右手直接从帽顶拖动。第六轮达到 1.9955 圈并真实解除螺纹连接，末态左食指和拇指对瓶分别约 0.373 N 和 0.449 N，瓶身只平移约 3.82 mm。受限关节控制全程无 MuJoCo 警告或手指越限。脚本、动作和实际退丝积分状态位于 `/data/dexcap-repro/benchmarks/tea-bottle-native/` 下的 `cap-top-from-pregrasp.py`、`tea-production-cap{,-actions,-released}.*`。当前生产模型中的瓶身全程自由。

从同一实测退丝状态，左右两手先撤开后按当前盖姿重拟合。最初的右手夹持上升约 4.65 mm 即滑脱。使用两指载荷反馈继续试验后，盖子离开瓶身并在相向夹持下累计抬高约 29.18 mm，随后失去抓持。记录分别在 `tea-raised-layout-left-raised-cap-lift.json` 和 `tea-production-cap-tactile-lift.json`。直接把受限关节轨迹的实测腕位用于标准 OSC 时，前 375 步未产生有效旋转。加 3 mm 顶压补偿后，180 步从闭盖转约 0.0995 圈，无机械臂或双手碰撞及物理警告。该补偿只用于标准转换试验。

在拓宽木托前的完整双臂模型上，从同一退丝和提盖状态继续真实转运。盖子沿托盘外侧搬运，期间一直由相向的两指承重。到右侧时夹持滑脱，盖子落在盘底。右手撤开后，盘框坐标约为 `[-20.7, -101.5, 58.8] mm`，标准持位控制下 `cap_on_right=true` 连续 1 s。这个末态附带原生积分状态，见数据盘 `benchmarks/tea-bottle-native/tea-current-cap-placed-right.{json,npz}`。此开盖及放盖证据来自下面的木托更新以前。

官方视频显示右手在约 16.25–17.5 s 从上方夹住木臂靠近桥端的一段，提起后木托留在茶盘。按首帧托盘四角整正估计，木托外宽约 90–105 mm、净宽 35–42 mm，夹持端外悬约 25–33 mm。生产场景把原 4 mm 方截面木臂改为桥端约 19.6 × 8.6 mm、前端 4 × 4 mm 的渐缩网格，木托改为外宽约 95 mm 的自由浅口槽，内侧用向外倾斜的实体围缘。木托位置按夹持方向和座稳情况做小幅调整，尺寸仍为影像估计。两个弹簧铰链、物体自由度及原摩擦参数保留。帧、像素端点和尺度假设在 `runs/tea-source-dimension-estimate/`。 [官方备茶视频](https://dex-cap.github.io/assets/normal_tea.mp4)

新几何在独立 30 s 试验中落稳后持续由木托支撑，末端相对起点移动约 13.77 mm。完整双臂、茶粒和其余器具同时在场时，以已经放盖的声明式阶段起点运行 6 s，镊子仍由木托承托，后者由托盘支撑，位移约 6.07 mm。两组整臂预抓扫掠无干涉。物理接近后两指实际分别测到约 0.14 和 0.15 N 的相向载荷，继续合拢时法线改变并与木托口接触，尚未离托。备茶 12 项测试通过，标准入口另输出了 `10000 × 6` 点云。数据在 `/data/dexcap-repro/benchmarks/tea-tool-source-model/scaledTrough/`。

剪刀重排试验中，在线跟随固定柄网格使右中指首次与原有食指托力同时出现，中指约 0.079 N、食指约 0.112 N。随后限制在同一段柄面反馈，该阶段中指载荷峰值约 0.034 N，末关节距上限约 0.014 rad。剪刀仍由托盘支撑，该右手试验用于寻找可重排的抓法。记录位于 `runs/scissor_89a2f80_moving_handle_underlift/dual-ring-samehand-middle-{online-servo,force-hold-continue}.json`。

擦盘的第二笔试验先撤掉原先向下压住海绵的左无名指接触。17 个约 50 ms 周期内，左中指和拇指仍各约有 4.3 N，双手继续同时持盘和海绵，末段连续 5 个周期无名指载荷为零。随后按实测状态尝试让无名指从海绵下侧进入，扫掠仍接触柔性海绵或其他刚体，未执行该路径。记录位于 `/data/dexcap-repro/benchmarks/wiping-coordinated-grasp/stroke-support/left-return-ring-{release,under-from-release-plan}.json`。

海绵接触参数更新后的 40 个 Wiping 回合中，旧判据给出 PointNet 0/20、SetXF 2/20 成功。两条被判成功的 SetXF 回合的峰值接触力约 15–16 kN，原判据还允许未同时持物的盘面接触累计为有效笔画。两条种子的单独复跑均未重现成功。现在只在右手相向夹持已离桌的盘子、左手相向夹持已离桌的海绵，并发生内盘面接触时累计擦拭笔画。保留原始内盘面接触次数供诊断。原始汇总和复跑记录在 `runs/sponge-contact-benchmark/`。

### 加宽木托后的备茶续段

从默认满载茶瓶与闭盖开始，在新木托的完整现场重验退丝。左手相向稳瓶，右手分六轮转到 1.99541 圈，螺纹连接实际解除。瓶身平移约 3.83 mm，全程无物理警告。两手撤开后，以实测瓶盖位置重拟合右手并将原抓取腕位提高 2 mm。右手两指相向抬盖，盖子与瓶身分离，独立提盖段最高抬升约 17.65 mm。之后的高位续段将相向夹持且脱离瓶身的盖子抬升约 67.51 mm，开始横移后滑脱，仍未完成右侧放置。该续段数据为 `tea-current-cap-carry-overhead.json`。积分状态和受限关节控制记录位于数据盘 `benchmarks/tea-bottle-native/after-scaled-trough/` 的 `tea-production-cap.json`、`tea-production-cap-anchored-raised.json` 和 `tea-anchored-cap-carry-seed-*.npz`。

从声明已放盖的独立阶段起点验证提瓶。把左食指和拇指尽量放在圆瓶同高的相对壁面，重新规划经过瓶肩的整臂路径，再按实际相向接触保持夹力。左手将满载瓶抬离托盘，瓶身最高上移约 40.95 mm。2 ms 积分中，只有左手夹持且无托盘支撑的最长区间为 1,463 个子步，即约 2.93 s。右臂退让后仍可夹持运瓶至茶壶上方。首条原生动力学倾倒路线在转腕时滑脱。改用更接近垂直于两指触点连线的转轴后，瓶身实际倾斜达到约 64.85°，该周期开始时仍有约 0.95 N 的较小夹持载荷。之后仍然滑脱，两条路线均无茶粒入壶。脚本、逐步受力与阶段积分状态在数据盘 `benchmarks/tea-bottle-native/` 的 `lift-bottle-planes-routed.py`、`tea-balanced-bottle-lift-routed.json`、`tea-balanced-bottle-lift-held.npz`、`tea-balanced-bottle-pour-parked.json` 和 `tea-balanced-bottle-pour-transverse-partial.json`。论文备茶还要求右手用镊子辅助倾倒，并将两件物品放回。 [操作顺序](https://arxiv.org/html/2403.07788v2#A1.SS6)

### 擦盘回程的第三支点

擦盘的独立回程试验中，右无名指接入后承担第三支点，右食指和拇指继续夹住离桌的盘。无名指入接时峰值约 83.45 N，接触收敛后后段约 4.0–4.7 N。第三支点的连续观测段为 0.65 s。该起点海绵与盘心相距约 293.8 mm，每腕每个 50 ms 周期移动 2 mm。相向推进时左中指逐渐卸载，海绵脱夹前未重新接触盘面。第二笔尚未建立。记录见数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/left-return-right-ring-bilateral-approach.json`。

第一笔后让右腕沿实测盘法向外移约 1.6 mm，可将海绵和盘的载荷从 1.776 N 降至 0.694 N，连续保持双手夹持四个控制周期。用刚完成第一笔的实际世界位移反向，海绵实际只反向约 0.75 mm 后，反向载荷使右手中的盘滚转，盘下净空从约 58 mm 降到 29–36 mm。左腕跟踪误差小于 1.3 mm，第二笔仍未成立。记录见 `benchmarks/wiping-coordinated-grasp/stroke-support/left-return-{outboard-unload-second,early-world-reverse}.json`。

当前生产模型已从原始初态独立 EGL 复放一条 3,259 步第一笔参考。计数为 **1**、有效距离 **70.62 mm**，当时双手各自真实夹持盘与海绵，峰值擦拭力 1.252 N。该路径的采样最大同手穿透约 **14.4 mm**，机械臂、双手间接触和求解器警告均为零。数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/left-return-first-counted-current-model.hdf5` 及同前缀 `-egl.json` 为标准入口的数据和证据。反向续作在早段第 14 个目标观察到左中指海绵接触力约从 1.63 N 降至 0，拇指仍约 5.27 N，是当前反向失持的具体位置。

原生回放将左食指改为第三个支点，在第 3,261 步观测到其承力 2.92 N，左中指和拇指分别约 3.72、4.93 N。此时两手分别持有盘和海绵，第一笔累计 79.46 mm，未受桌面支撑。第三点有效 19 个控制周期，进入回程约 8 mm 后仍失持。该段的原生回放证据为同目录 `left-return-shifted-index-tripod-native-g-audit.json`，已导出标准入口 HDF5 `left-return-first-counted-index-tripod-current-model.hdf5`。

把右手入场时的盘沿抓点提早转动约 4°，右食指的盘局部抓点由约 `[47, -62] mm` 移到 `[39, -65] mm`。3,266 个目标的实际回放记录了 92.47 mm 的第一笔。后续先卸载约 0.8 mm 再开始回程，腕位反向目标累计到 8.5 mm 时仍保持双手持物，累计计分的有效擦拭距离增至 98.85 mm。反向目标到 9 mm 时左手海绵脱夹，右手仍持盘。第二笔尚未计分。实际联合接触和动作位于同目录的 `left-return-shifted-rim-pickup-4deg-early.*` 和 `left-return-shifted-rim-reverse.*`。

将左食指触点加入第一笔的原生轨迹后，食指确实提供抗滚作用，实测峰值约 2.92 N、最大反向力矩约 0.073 N·m。回程约 7 mm 时左中指主斑块滑离，盘仍在右手中。食指原触点在掌坐标只漂移约 1.53 mm，补偿这段漂移和滚转未能恢复夹持，第二笔仍未计分。数据盘同目录 `left-return-shifted-index-tripod-native-f.*` 与 `left-return-shifted-index-follow-i.*` 保存实际动作和接触记录。

### 剪纸的标准提纸回放

从默认托盘、剪刀及纸带摆放开始，右拇指越过纸带顶边，食指和拇指闭合，随后将完整纸带抬离支撑。223 个标准 46 维目标以 50 ms 周期从同一初始摆放回放，纸带最低段增加约 54.32 mm。实际两指相向夹持且整条纸带没有托盘或桌面支撑的最长连续区间约为 2.25 s。右食指和拇指的峰值载荷约 0.07175 和 0.07149 N，未触发机械臂碰撞、双手碰撞或物理警告。

在此真实右手持纸末态开始左臂接近，242 个静态路径点对右臂、纸带、托盘及桌面均无穿透。增加 102 个标准动作后纸带一直离盘，左拇指曾严格进入活动环，食指和中指尚未进入固定环。继续加载时活动环对拇指约 7.5–11.5 N，固定柄外侧约 2.2–3.3 N，左腕跟踪误差回升至约 17.5 mm，尚未开始切割。力、几何和动作记录位于 `runs/scissor_paper_default_pickup_c4d447c/` 的 `scissor-paper-default-pickup-standard-contact-audit-left-reach.json` 与 `scissor-paper-default-pickup-standard-left-reach-actions.npz`。

```bash
python -m simulation.rollout \
  runs/scissor_paper_default_pickup_c4d447c/scissor-paper-default-pickup-standard.hdf5 \
  --task scissor-cutting --output runs/scissor-paper-default-pickup-rollout.json
```

从左手已经操作剪刀的实际积分状态继续，将右手原取纸路径沿纸带向中心移动约 12 mm。实际食指和拇指跨第 8、9 列持纸。223 步取纸末端夹力为 0.132 N，继续搬运 60 步后为 0.132 N，再下降 40 步至刀口后仍为 0.131 N。定刃和动刃分别接触了不同的纸单元，峰值刀刃载荷约 0.00553 N，85 个接缝保持完整。绕刀尖让纸完全退出后，夹力仍有 0.124 N。此后保持腕姿回送 40 mm，工具侧实际只移约 2 mm，问题转到右臂的进刀方向和腕姿。数据盘 `runs/mujoco/scissor_left_first_then_paper_a8c3318/` 中的 `paper-deeper-column-{pickup,diagonal,descend}.json` 和 `paper-deeper-column-bond31-{fullclear,reinsert-reanchor}.json` 记录了动作与实际接触。

从退出刀尖的实际状态，把右腕空间目标分段执行，每段保持 250 ms。40 mm 目标实测前送约 38 mm，持纸载荷约 0.118 N。此时托盘墙仍在承托纸带中段。沿实际受力继续拉离，经三段后纸带完全脱离托盘和刀刃，夹力 0.138 N。再次回送自由纸带时，活动刃实测接触约 0.00648 N，但纸带曲率改变，固定刃尚未压到同一接缝。85 个接缝仍完整。动作、载荷和关节裕量见同目录的 `fullclear-freewrist-execute.json`、`fullclear-freewrist-peel3-execute.json`、`freepaper-return-execute.json`。

精确接触审计确认定刃曾对第 4 列接缝施力约 0.00856 N，主动合刃后，动刃接触的是相邻第 5 列的纸单元。尝试沿实测纸缝节距平移约 10.50 mm 时，纸带维持离托，但弯曲使定刃只在最初 16 个物理子步保持目标接缝接触。改在离刀口更近的第 7 列抓取时，拇指约 0.121 N、食指约 0.0026 N，纸仍靠托盘支撑。第 8、9 列的离托相向夹持目前可复现。试验见同目录的 `freepaper-bond44-fixed-contact-checkpoint.json`、`freepaper-bond44-onepitch-align-close.json` 和 `paper-closer-col7-pickup.json`。

备茶的独立三指提瓶续段按手指原网格的接触间距调整食指和中指。在满载瓶约上移 23.1 mm 时，左三指实际载荷约为 1.07、0.70、0.47 N，托盘和右手载荷均为零。稳定提瓶阶段分别保存约 15、20、23 mm 的积分状态。续段把右腕单独退开时，左食指一度卸载，说明提瓶与倾倒之间仍需连续维持夹持。记录在数据盘 `benchmarks/tea-bottle-native/` 的 `tea-three-pad-current-bottle-lift.json` 和 `tea-three-pad-current-held-*mm.npz`。提瓶起步与右腕退让合并，再补偿实际瓶口位置和两指载荷差。由受限关节动力学实测，夹持下倾角达到约 80.19° 时，拇指和食指约为 0.47、1.74 N，中指约 0.16 N，瓶口距壶中心平面约 12 mm。此时拇指末轴在 −1.34 rad 止挡附近，拇指触点已从瓶侧滑到瓶口上缘。再倾倒失去夹持，未有茶粒沉降在壶内。保存的 `tea-pour-balanced-held-80deg.npz` 有三个指垫的实际受力，完整记录为 `tea-balanced-bottle-pour-three-pad-balanced-wrist.json`。

### 捡球的随机摆放回放

用同一条 265 步参考动作测试论文定义的球与线篮初始摆放范围。种子 1–20 的位置来自正式场景采样器。接近和抬球段按初始球位平移，放球段按初始篮位平移，搬运段在两个位移之间插值。动作使用原 HDF5 的世界坐标，按正式腕位控制跟踪。20 个种子都实际抬球，19 个把球释放至篮中并由篮底承托。种子 16 在转运末端出现双手碰撞，右腕目标误差约 151 mm。该回放已知初始物体位置，作为场景和控制验证。逐回合记录和动作生成脚本在数据盘 `benchmarks/ball-seeded-reference/world/`。

种子 16 的原抬球阶段里，无名指和中指间的实际最大网格穿透约 4.66 mm。保持实际夹球的三指不动，另调整闲置无名指，并降低搬运高度、渐进俯腕、在篮口中心上方稳定后松手。365 个目标通过同一标准入口，球实际释放并由篮底承托。回放没有机械臂碰撞、双手碰撞或物理警告，点云为 `10000 × 6`。该回放使用单个 `--first-seed` 从正式采样器恢复初始状态。

```bash
ball=/data/dexcap-repro/benchmarks/ball-seeded-reference/world
python -m simulation.rollout "$ball/seed-16-standard-reference.hdf5" \
  --task ball-collecting --first-seed 16 --output "$ball/seed-16-standard-rollout.json"
```


### 擦盘第一笔末端的三指支撑

从 3,266 步参考在第 3,238 步后接续，使右无名指于第 3,254 步入接并在第 3,259 步完成第一笔计分。末态盘和海绵仍离桌，右食指、拇指和无名指分别约 10.31、7.45、9.24 N，左中指和拇指分别约 3.11、3.68 N，累计有效擦拭距离约 83.52 mm。从这次已接触内盘的末态执行回程，右手三点支撑保持，海绵约移动 25.5 mm 后因受压滑脱。该回程的盘面接触峰值约 26.74 N，计分仍为一笔。接续动作和积分态见数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/` 的 `left-return-first-pass-ring-delayed*.{json,npz}`。

### 捡球参考中的同手接触

标准回放的 `same_hand_max_penetration_m` 和 `same_hand_max_penetration_pair` 记录每个控制周期采样到的同手最大穿透和接触对。原捡球参考回放测到右中指与右无名指约 4.66 mm 穿透、20.81 N 法向力。触点到 LEAP 原始表面约 0.41 和 0.61 mm。重设闲置右无名指后，默认摆放和种子 16 保持落篮。查看新输出时还定位到闲置左食、中指之间约 8.99 mm 的穿透。将左手改为自然张开，265 步默认摆放的 EGL 回放仍完成抓起、释放和落篮，测到同手及双手间穿透均为 0。对照轨迹和实测 JSON 在数据盘 `benchmarks/ball-seeded-reference/world/stowed/`。避开右中指与无名指接触后，原来仅平移腕位的随机搬运容易滑脱。20 个种子均能提球，其中 4 个用平移路线落篮，另外 8 个用实测球位和新搬运路线落篮。其余种子的移动与释放仍需调试。

```bash
ball=/data/dexcap-repro/benchmarks/ball-seeded-reference/world/stowed
python -m simulation.rollout "$ball/ball-collecting-open-rest-reference.hdf5" \
  --task ball-collecting --output "$ball/open-rest-rollout.json"
```

### 备茶的标准腕位控制退丝

从默认满载茶瓶和上紧瓶盖起步，用标准 46 维腕位和手指目标控制完整双臂。沿原录制的旋拧弧段重复施力，每个周期用实际帽顶载荷调节右手高度，并跟随自由瓶实测位置补偿双腕。第 2,301 个 50 ms 控制周期，约 115.05 s，旋转到 1.995295 圈并解除模型中的螺纹连接。该实测无机械臂碰撞、双手碰撞或物理警告。末态积分状态已保存供取盖接续，未完成右侧放盖或倒茶。记录在数据盘 `benchmarks/tea-bottle-native/after-scaled-trough/` 的 `tea-standard-arc-repetition.json` 和 `tea-standard-osc-cap-released.npz`。

```bash
tea=/data/dexcap-repro/benchmarks/tea-bottle-native/after-scaled-trough
python -m simulation.rollout "$tea/tea-standard-osc-cap-unthread-reference.hdf5" \
  --task tea-preparing --output "$tea/tea-standard-osc-cap-unthread-rollout.json"
```

以上导出经独立标准入口实际复放 2,301 步，终态同样为 1.995295 圈且 `cap_unthreaded=true`。EGL 输出 `10000 × 6` 点云，机械臂、双手及采样同手穿透均为 0，无物理警告。标准出口文件为 `tea-standard-osc-cap-unthread-rollout.json`。

改用生产完整隐式积分后，同一份标准目标又用 EGL 执行 **2,301 步**。末态实测 **1.995103 圈**且 `cap_unthreaded=true`，无求解器警告、机械臂接触、双手接触或采样同手穿透，输出 `10000 × 6` 点云。实测文件为同目录 `tea-standard-osc-cap-unthread-implicit-2ms.json`。

### 退丝后的取盖与进剪刀柄

左手中指先外让，再抬高约 40 mm 换肘，从远离壶嘴的一侧下降。左食指在保留 `cap_on_right=true` 时实际接触镊子木托，该次力采样约 **2.32 N**。食指随后滑离，镊子仍由木托承载。原生动作与末态在数据盘 `benchmarks/tea-bottle-native/tea-prepour-left-high-elbow-insertion.*`。

剪刀固定环外弧经实体接触重新定位后，从无接触保存态中指优先插入。末态 `final_strict_middle=true`，拇指仍在活动环外，双环同时承力时长为 0。中指的下一段必须在保持开孔截面约束时重定向拇指。记录在数据盘 `runs/mujoco/scissor_left_first_then_paper_a8c3318/left-fixed16-rotated-middle-first-servo.*`。

从标准腕位控制实际退丝后的积分态撤手，再在高位调整右手指形并限速接近。食指和拇指首次相向抬盖约 11.98 mm。用这一实测夹持接续上提，盖子在无瓶身支撑时累计上移约 21.3 mm，其侧壁随后重新触到瓶颈。按实际接触法向侧向退出后，瓶颈接触短时消失，双指仍有载荷，继续移动时滑脱。取盖仍未完成右侧放置。逐步受力与实际积分态见数据盘 `benchmarks/tea-bottle-native/after-scaled-trough/after-standard-release/` 的 `tea-standard-cap-quiet-extraction.json`、`tea-standard-cap-carry-live-tilt.json` 和 `tea-standard-cap-unhook-and-carry.json`。

从上述真实脱离瓶颈的积分态，右手继续快速侧移。盖子脱手后在托盘上滚动，最终在右侧静止，世界坐标约 `[0.35188, -0.01172, 0.01099] m`。末态托盘承载约 0.247 N，双手和瓶颈均已脱离盖子，`cap_unthreaded=true`、`cap_on_right=true`，无物理警告。该段使用实际关节执行器的延续试验，从独立验证的标准退丝回放接续。右侧静置的积分状态与动作记录为 `tea-south-cap-transfer-south_direct.npz`、`tea-right-cap-supported-actions.npz`，逐步结果为 `tea-right-cap-supported-checkpoint.json`，同在上述数据盘目录。

放盖后，右腕实测撤至世界坐标约 `[0.4405,-0.1820,0.3022] m`。左手按瓶的实际新位置进入，相向拇指与食指建立抓持。初次抬升约 6.88 mm 时，食指与中指发生接触并触发停止。让闲置中指随实测间隙外让后，同一积分态接续实测抓持离托盘最高 **57.07 mm**，峰值时食指与拇指仍分别承力约 1.28 和 0.62 N，托盘和右手承力均为零，`bottle_used=true`、`cap_on_right=true`。继续上提后瓶身滑脱。保留了 25、35、45、50 mm 带载积分态供转运倒茶接续，记录为 `tea-after-right-cap-separated-lift-lift.json` 与 `tea-after-right-cap-separated-lift-*mm.npz`。本段尚未倒入茶叶。

为让右食指及时接住仍由左手夹持的茶瓶，右手在左手提瓶期间接近。按实测指垫间距分段减速建立侧向接触，连续 **2.256 s** 实测两手均有载荷且托盘承力为零。从保存的积分态继续实际转运，左拇指与食指保持相向接触、右食指仍承力，在 **2.932 s** 连续带载段内瓶身沿世界 x 正向移动约 **86.5 mm**、y 负向约 44.9 mm。最高位移处右食指法向载荷约 2.38 N，托盘承力为零。另一次接续将带载瓶身移到世界 x 约 0.570 m，左指接触法线转为近正交，仍未倒入茶叶。结果与实测积分状态见 `tea-side-support-distance-rate-3.json`、`tea-range-supported-translation{,-second}.json` 和 `tea-range-supported-translation-best.npz`。

接续向壶口移动时，茶瓶滑脱并在托盘上近乎竖直落稳，距壶口中心的水平距离约 114 mm。撤手后，托盘约承力 1.02 N，双手对瓶载荷为零，108 粒茶叶均仍在瓶内，盖子保持放在右侧。实际积分态和从前一状态接续的执行器记录见同目录的 `tea-staged-upright-near-pot.{json,npz}`。随后打开左手并保持腕位，清除了无名指与手掌残余接触，瓶身位置变化小于 0.1 mm，108 粒茶叶仍在瓶内。较干净的接续状态保存在 `tea-staged-upright-near-pot-clear.{json,npz}`。

从该状态重新形成左拇指、食指和中指三点抓持。瓶身离开托盘最高 **18.59 mm**，最长连续离托且三指有载荷的区间为 **1.136 s**。在抬升约 15.02 mm 的保存点，三指约承力 1.46、0.60、0.85 N，拇指和食指接触法线点积约 −0.89。实测与带载积分态见同目录 `tea-near-pot-three-pad-bottle-lift.json` 和 `tea-near-pot-three-pad-held-15mm.npz`。

从 15 mm 保存点按三个实测指垫斑块继续抬升和转腕。抬升阶段瓶身相对托盘上的初始位置最高上移约 **126 mm**，食指和拇指仍承力，瓶身没有外部支撑。随后瓶口移到壶口上方附近，瓶轴相对竖直倾斜约 **53°**，仍朝上。下一步左拇指触及壶壁并触发停止，入壶粒数为 0。逐步轨迹见同目录 `tea-staged-lift-tip-126.json`。

将更高一组抬升目标用于同一抓持，瓶身最高离初始支撑位置约 **162 mm**，两指仍承力；倾斜后拇指卸载，入壶 0 粒。从抬升段保存的另一积分态，实际收起左中指，同时保留左拇指与食指对瓶的夹持。选中路线连续至少 0.576 s 实测两指有载、第三指和托盘无载荷，末态两指约承力 1.67 和 1.35 N。该状态用于给右手腾出受力空间，记录见 `tea-staged-lift-tip-clear-162-+0.json` 和 `tea-staged-left-middle-clear-2.{json,npz}`。

在抬升茶瓶时提前把右手移到预抓位，收起左中指后由右食指实际接触瓶侧。双手共同承力且瓶身未获托盘承托的最长连续段为 **0.726 s**。一个保存点的右食指载荷约 1.25 N，绕世界 y 轴朝倾倒方向的实测力矩约 +0.0231 N·m，左拇指和食指约承力 1.63、1.16 N。该点还测到右食指与左中指轻触。后续同步转腕时右手曾失载，跟随路线则遇到双食指靠近，入壶仍为 0 粒。接续态和结果见 `tea-staged-bilateral-right-ahead-load-0-.37{,-best}.*`、`tea-staged-lift-tip-bilateral-live-follow-*.json`。

另从左手倾斜茶瓶的实测序列保存壶沿受力点。该状态瓶底和瓶壁实际接触壶沿四个斑块，接触法向力合计约 6.19 N，作用于瓶身的垂直分量约 +5.89 N。瓶口仍朝上。完整场景积分态在同目录的 `tea-staged-bottle-over-pot-201.npz`，可从真实壶沿支点接续。

从壶沿承重的物理状态继续转腕，按食指和中指网格净空调节闲置中指，之后用两指夹住茶瓶并越过水平。将瓶口向实际移动后的壶口平移，从 160 号保存状态接续时，第 53 个控制目标首次得到 `tea_deposited_count=1`。实测粒子 83 同时由壶底和壶壁支持，保存点瓶轴倾斜约 **106.5°**，左拇指和食指约承力 **1.06、1.82 N**。壶本身在接触过程中倾斜约 62°。入壶后的茶粒在最后交互末态仍在壶内，末周期未满足静置计分条件。该段没有完成镊子辅助、两件物体归位或整轮计分。

首次入壶结果和实际积分态位于同一数据目录的 `tea-staged-lift-tip-live-mouth-128-49--17-160-32-127.json`、`tea-staged-lift-tip-live-mouth-128-49--17-160-32-127-best.npz`。颗粒身份、速度与接触力审计在 `tea-first-actuated-pour-grain-audit.json`。核对截面接续物理的脚本为 `position-live-tilted-bottle-over-pot.py`。

归位动作中茶瓶从左手滑落并接触托盘。两个实际落瓶积分态用 `implicitfast` 和 2 ms 子步续跑时，快速翻滚的非轴对称茶粒均触发 `mjWARN_BADQACC`。从同一状态执行同样的撤手时序，完整隐式积分在 2 ms 下跑完 **3.608 s**，两个场景均无警告、壶体回正，最后约 1 s 分别稳定保留 **1 粒和 2 粒**。1 ms 也通过这两组对照。生产 Tea 场景采用完整隐式积分和原来的 2 ms 子步。独立回归用实际量级的三轴角速度验证同种椭球茶粒，旧积分在 32 步触发警告，新的积分执行 400 步。结果与积分态在同目录 `tea-post-release-integrator-*-*ms.{json,npz}`。 [MuJoCo 积分器与陀螺力](https://mujoco.readthedocs.io/en/3.6.0/computation/index.html#numerical-integration)

从保留 2 粒的稳定状态继续，左手到达横放瓶身底部，用食指沿托盘实际推移约 105 mm，再撤离。两份终态都由托盘承托，茶瓶离原始瓶位的平面距离分别为 **33.22 和 33.56 mm**，仍横放，满足当前 35 mm 归位条件并已完全脱手。壶直立且各保有 2 粒茶叶。连续动作和终态审计在同目录 `tea-post-pour-bottle-return-{0,3}-terminal.npz` 和 `tea-post-pour-bottle-return-{0,3}-terminal-audit.json`，其中归位和脱手结果继承同链前段真实的提瓶计分。完整备茶尚需要右手取用镊子、在倒茶中辅助并归还。 [论文备茶流程](https://arxiv.org/html/2403.07788v2#A1.SS6)

从右手实际悬持整条纸带的标准回放接续，左食指完整截面在固定环内、左拇指完整截面在活动环内同时维持 0.200 s，左食指在末态约有 0.0693 N 载荷。中指有静态完整进环解，直线路径在左食指近端与左中指末端之间测到约 1.00 mm 穿透和 7.62 N 法向力。另规划的五点绕行总关节路径长约 2.054 rad，在实际执行 1.614 s 后，纸带一部分重新接触托盘。此时食指和拇指仍入环，纸带仍由右两指相向持有。中指未完成入环，剪刀未离托盘或切纸。数据见 `runs/scissor_paper_left_functional_c4d447c/` 的 `left-thumb-preshape-after-middle-clear.json` 和 `left-middle-rrt-tactile-servo.json`。

从同一条 223 步抓纸回放重新提前预置左中指，左腕绕世界 X 轴滚转 −7° 后通过了该段进入轨迹。拼接为 345 个 46 维目标，纸带仍由右手相向悬持。再保持 80 个控制周期，纸带仍保持离托，固定柄的瞬态载荷从约 10.11 N 降至 0.016 N，剪刀主体上移约 12.6 mm，随后由转轴与托盘侧边接触承担约 0.368 N。该末态提供了新的进环入口，三指均未达到完整进环。结果在 `runs/scissor_paper_early_middle_c4d447c/` 的 `standard223-middle-waypoint2-roll7-left-reach.json` 与 `standard223-middle-waypoint2-roll7-settle-native.json`。

在上述高位支点沿原路径上撤左腕 60 mm，右手仍悬持纸带。合并进入、保持、撤腕为 465 步，通过标准 EGL 从同一初态独立回放。纸带与剪刀的峰值抬升分别为 57.41 和 28.38 mm，机械臂、双手间接触步数为零，无物理警告。剪刀最后在托盘侧边支点约 50.08 mm 高处静止。回放断键为 0，切纸长度为 0。HDF5 和 EGL 汇总在同一目录的 `standard223-middle-waypoint2-roll7-elevated-position-reference.hdf5` 与 `standard223-middle-waypoint2-roll7-elevated-position-egl.json`。

从真实高位支点改走固定柄侧推。左食指先在完整网格上接触固定环，随后分三段向托盘内侧施力，使剪刀主体过托盘边沿中心线约 2.12 mm。主动撤去食指后，连续 1.082 s 无柄载，沿推动轴回弹约 0.085 mm，终态坐标约 `[0.40045, 0.19797, 0.06984] m`，纸带仍离托。无物理警告或记录到的非目标碰撞。稳定态和逐步输出在同一数据目录的 `second-support-index-release-settle-final-state.npz`、`second-support-index-release-settle.json`，可从这里继续入环。

从右手保持纸带、左食指和拇指已进环的真实状态，先收左食指，再让中指接触剪刀固定环。中指实测峰值约 0.0628 N。中指和拇指同时施力时产生约 0.0050 N·m 的转矩，但合计法向载荷约 0.0716 N，剪刀自身重量约 0.2727 N，托盘仍承力。重新接回食指的路径出现约 1.04 mm 的食指与中指网格交叠，未达成三指同时承力或持续离托。记录为数据盘 `runs/mujoco/scissor_left_first_then_paper_a8c3318/left-threepoint-middle-optimized-lift-v3.*`。

保留现有 40 × 37 mm 活动环后，完整网格能容纳轴向 +15 mm 的拇指薄段。实际从托盘支撑位走向该端点时，拇指先压活动环外侧，使剪刀移位约 7.38 mm 并打开至 0.568 rad。改由外侧拇指保持接触并沿关合方向施力，两段连续实测使铰链从 **0.35252 rad** 降到 **0.33821 rad**。后段持续负向运动约 23.18 s，拇指峰值约 0.114 N。剪刀由原托盘和隔板承托，纸带 85 条粘结均完好，未切断。状态、动作和实测报告位于数据盘 `runs/mujoco/scissor_left_first_then_paper_a8c3318/` 的 `current-outer-friction-close-continue*`。

再从同一个物理状态连续执行右手抓纸、搬运和向刀口进给，同时左拇指对活动柄施力。100 个目标内铰链实测关合约 0.03654 rad，固定刀刃实际触纸。活动刀刃离纸约 1.88 mm，尚未形成双刀刃夹压。纸带仍为 85 条完整粘结。记录见同目录 `current-carry-close-combined.json`、`current-carry-close-combined-state.npz`。

继续按活动刃与纸带的实测距离进给，首次同时接触两片刀刃，纸夹持约 0.0462 N，双刃总载荷约 0.00303 N，接触落在不同纸单元。对齐至 `paper_bond_4_3_0` 的后续最佳中间态中，活动刃距目标单元约 0.059 mm，固定刃约 3.24 mm，尚未形成同一接缝的双刃夹压。回到标准 223 步抓纸末态，用实测距离加深贴面 0.90 mm，约 0.60 s 观察区间内的右手相向载荷最低约 0.0536 N，纸带保持离托。接触与深抓记录见同目录 `current-paper-movingblade-livefeed.json`、`current-bond43-blade-gaps.json` 和 `paper223-deep900-grip.json`。

擦盘在第 3,259、3,260、3,261 步保存第一笔已计分的积分态，盘和海绵均仍离台。第 3,261 步载荷最低，为约 1.04 N。从此态尝试双腕小幅反向退载，随后接触力由约 1.06 N 回升到 2.55 N。两手仍夹持物体，盘下净空约 58.54 mm，未实测离面。逐步复算确认海绵在盘坐标的法向高度仅变化约 −0.003 mm，而切向位移约 15.5 mm，盘本身同时平移和倾转。按实际斑块反向转动，法向开口约增加 1.03 mm，最深接触仍约 0.74 mm，载荷增至约 3.78 N。下一次接触调整从第一笔更早的轻载状态开始。记录见数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/` 的 `left-return-early-pass-checkpoints.{json,npz}` 与 `left-return-early-light-second-pass-e.json`。

备茶的原尺寸木托与镊子从实际接触状态继续。右拇指顶到镊子桥的上沿时，镊子相对木托横移约 17.43 mm、抬高约 1.81 mm，木托自身只移动约 1 mm。拇指滑脱后木托仍以约 0.337 N 支撑镊子。再次下探桥端垂直侧面时，上木臂首先挡住拇指并承受约 0.310 N 向下载荷，尚未把镊子送到托盘上。记录在 `/data/dexcap-repro/benchmarks/tea-tool-source-model/scaledTrough/source-tool-cradle-to-tray-summary.md`。

从擦盘第一笔的第 3,238 步改用实测盘姿闭环，第一笔载荷约 0.23–0.57 N，盘面倾角不超过 46.1°。换向前若增加右环指部分支撑，第一个反向周期仍持住两件物体，第二周期右拇指与环指同时失载，盘下落约 7.29 mm。无环指路线同样在第二周期失载。由此把下一轮调节集中到右手对盘的抓持力矩。细节见 `benchmarks/wiping-coordinated-grasp/stroke-support/` 的 `left-return-light-first-controlled-reverse-{e,g}.json`。

擦盘在第一笔进行到第 3,256 步时，以右中指增加实测盘下支撑，随后保持该指形至换向。回程盘连续保持 11 个动作，其中两手同时实际持物 7 个动作。第一回程动作的中指载荷约 0.96 N，关于先前失持方向的力矩约 −0.065 N·m。盘随后仍保持，先失载的是左手夹海绵的中指，第二笔尚未达计分。这次结果存于 `benchmarks/wiping-coordinated-grasp/stroke-support/left-return-middle-support-frozen-left.{json,npz}`。

左食指从镊子桥侧推，实际桌面上的木托在原托盘边沿形成了双边支点。工具相对木托沿局部 y 方向又移动约 31.75 mm。继续推时镊子滚转，指垫从桥侧移到顶部并失载。工具仍由木托承重。外露木臂到木托上沿的垂直间隙约 11.1 mm，所测完整食指形状无法从下方通过。结果在 `benchmarks/tea-tool-source-model/scaledTrough/` 中的 `source-tool-left-bridge-rim-arrest-push-v3.json`、`source-tool-rim-arrest-clearance.json` 与 `source-tool-right-index-underarm-fit.json`。

右食指从镊子桥的外露端沿实测间隙方向推压，镊子桥首次与托盘 x 正向边沿接触并承力。边沿承力超过 0.02 N 持续 0.384 s，末态约 0.032 N，同时木托仍承力约 0.302 N。随后推动自由木托，其实际横移约 0.93 mm 后镊子脱离盘沿，重量仍由木托承担。实测记录和接续积分态在 `benchmarks/tea-tool-source-model/scaledTrough/source-tool-right-tool-to-rim-tip-summary.md`。

另用 3,138 个动作从原始场景复放，左无名指、中指和拇指在盘与海绵双持的端点分别承力约 1.64、3.92 和 3.65 N。指间网格间距约 10.85 mm。下一周期无名指卸载，固定左腕与指形或暂停 40 个周期均没有恢复支撑。这次验证覆盖接触盘面前的阶段，第二笔未完成。端点和复放结果见 数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/` 的 `left-return-ring-inner-stable-prefix.npz` 和 `left-return-ring-inner-stable-prefix-verification.json`。

擦盘另试无名指先远离、再从海绵下方接近。真实执行中盘和海绵连续双持 80 周期，无物理警告。海绵相对手的变形和位移使无名指一直在上侧，结束时离目标下表面约 19.2 mm，继续接近时原夹持滑脱。该路线没有新增计分。可复放的 3,196 个动作和结果见 `benchmarks/wiping-coordinated-grasp/stroke-support/` 的 `left-return-ring-under-stable-route-prefix.npz` 和 `left-return-ring-under-free-run-f.json`。

镊子盘沿接触建立后，左食指从木托另一端开口向外移木托，右食指保持与工具的接触。先退出约 6.42 mm，后续稳定段再退出约 18.80 mm。最佳可复现状态中，木托与托盘沿的承力分别约 0.236、0.217 N，右食指与木托无接触。镊子仍有木托支撑。数据盘 `benchmarks/tea-tool-source-model/scaledTrough/` 中保存了 `source-tool-bimanual-rim-best-checkpoint.npz` 和 `source-tool-bimanual-rim-left-withdraw-summary.md`。

从已倒茶、已归瓶的诊断状态推进右指接触点，木托累计滚转约 23.13°，镊子对木托的最小载荷约 0.253 N，托盘仍未承托镊子。另从上方实际接触镊子，食指峰值约 9.02 N，载荷方向使镊子向下楔入木托，最终仍由木托承重。证据在数据盘 `benchmarks/tea-bottle-native/after-scaled-trough/after-standard-release/tool-cradle-roll/` 的 `tea-mainline-roll16-counterhold-recenter-summary.md` 和 `tea-mainline-roll23-tool-pickup-topology-summary.md`。尚待在倒茶前取到镊子并执行辅助动作。

另从已开盖并把盖子放在右侧的真实倒茶前状态继续，先做原生沉降再从两侧接近镊子桥端。两指峰值约 0.362 N、0.647 N，实测法向点积最低约 −0.101，工具未形成相向夹持。托盘一度受力，但木托最小承力仍约 0.256 N。备用中指或无名指下托的完整手路径也会与盘底或闲置手指干涉。数据盘 `benchmarks/tea-bottle-native/` 保存了 `tea-prepour-tool-acquisition-summary.md` 和 `tea-prepour-underside-topology-summary.md`。

从已开盖且盖子在右侧的倒茶前积分态，右食指先真实沿木托开口外推，工具在第一段连续接触时相对木托外移约 2.89 mm。接续按实际指垫法向小幅转腕，沿开口方向的法向分量从约 0.042 增到 0.185，再使工具外移约 1.65 mm。末态木臂从最后承力点外露约 38.5 mm，仍由木托承重。其余指垫在此腕位离工具约 69–96 mm，需要重新预抓。实测文件见数据盘 `benchmarks/tea-bottle-native/` 的 `tea-prepour-tip-outward-slide-normal-corrected.json` 和 `tea-prepour-tip-refit-continuation-dynamic-contact.json`。

从镊子已由托盘独立承托的状态，左食指实际把自由木托移开约 35 mm。撤手后镊子仍由托盘承托，木托对镊子载荷为零，壶倾角约 0.05°，盖子仍在右侧。重新计算右手原高空路径，木托导轨已经让开，末段仍有右掌对盖壁约 0.163 mm 的静态干涉，尚未执行夹取。工件为数据盘 `benchmarks/tea-bottle-native/tea-pot-nearclear-cradle-shift-execute-v7.{npz,json}` 和 `tea-pot-nearclear-cradle-shift-right-recheck.json`。

另以实际手部接触把已退丝的瓶盖移到右侧的另一处有效放置点，水平位移约 54.5 mm。沿原 0° 局部滚转进场时，盖与壶保有净空。第 85% 下探阶段右拇指与残存木托导轨、开口各出现约 0.465 mm 的静态干涉。见数据盘 `tea-cap-shift-for-tool-execute-v2.{npz,json}` 和 `tea-post-cap-zero-roll-contact.json`。

擦盘首笔的原右手自碰发生在最早 790 个目标内，当时右中指、无名指未对盘承力。将这两根手指在最初 1,000 步保持为原第 1,000 步已经达到的闲置姿态，再接回原动作。标准 EGL 复放 3,259 步，实际计到 71.84 mm 的一笔，右无名指后段真实承盘峰值仍为 7.34 N，右手穿透降为零。仍需处理左食指自身连杆的约 1.19 mm 接触。工件在数据盘 `benchmarks/wiping-coordinated-grasp/stroke-support/first-wipe-ring-clear-b.hdf5` 和 `first-wipe-ring-clear-b-egl.json`。

### 捡海绵随机摆放的物理基线

使用 305 步、默认摆放可抬升 310.51 mm 的参考，以摆放差平移右腕初态和目标轨迹。在 x 轴 ±0.20 m、y 轴 ±0.35 m 的场景分布内，20 个种子的无相机标准动力学复放完成 10 个任务，重放与生成的物理结果一致。右食指与中指最大采样穿透约 13.49 mm，多个种子右臂与闲置左手接触。这批数据用于定位手指与双腕的调整位置。汇总见 `benchmarks/sponge-seeded-reference/sponge-seeded-reference-summary.json`。

重新安排右食指、中指和拇指接触面，并从真实三指带载状态调整抬升闭合量。默认摆放的标准 EGL 回放执行 **532 个动作**和 **659 个 50 ms 控制周期**，抬升 **305.65 mm**，通过 300 mm 判据。末态由右食指与拇指相向持有，分别约 0.595 N 与 0.557 N，中指在抬升中卸载。机械臂接触、双手间穿透、同手穿透与手部越限均为零，无物理警告，得到 `10000 × 6` 点云。此结果覆盖默认摆放，随机摆放继续按同样的标准复验。

```bash
sponge=/data/dexcap-repro/benchmarks/sponge-seeded-reference
python -m simulation.rollout "$sponge/default-three-pad-clean-pick-reference.hdf5" \
  --task sponge-picking --output "$sponge/default-three-pad-clean-pick-egl.json"
```

三个代表性随机摆放也已用真实 EGL 独立回放。种子 1、7、19 分别抬升 **304.27、324.12、318.00 mm**，均为 532 个动作和 659 个控制周期，机械臂、双手间、同手及手部越限计数均为零。种子 1 修正接近海绵的腕位，种子 7 重定向抬升通道，种子 19 在抬升后段逐步调整拇指闭合和腕位。各自的 HDF5 和 `*-egl.json` 在相同数据目录的 `seed-{01,07,19}-three-pad-clean-*`。最终种子 1–20 的 **20/20** 个随机摆放均独立 EGL 通过，抬升 **304.27–368.10 mm**，无物理警告及上述碰撞和越限。种子 5 和 15 在第 331 个目标后调整了抬升通道。种子 8、12、13 各自调整了进入路线，其中种子 8 与 13 还分别调整了抬升路线和后段夹持。逐种子 HDF5 和 EGL 报告见 `sponge-clean-egl-summary.json`。按种子选定的动作、回放与抬升量另见 `sponge-clean-seeds-01-20-summary.json`。

### 捡海绵使用同一初态

上述各摆放的原参考从各自抓取通道起步。固定起点试验改用一对共同的实际腕位与手形，在完整双臂模型中先上提约 60 mm、移向该摆放的高位通道，再沿原动作抓取。各用 185 个 50 ms 腕位目标进入原参考。种子 19 首次接续在起升时食指与拇指失载，仅抬升约 32.8 mm。将原轨迹第 355 步后的拇指闭合目标提前平滑提高 0.01 rad，并在原第 430–439 步平滑回接，最终实际抬升约 345.68 mm。20 个种子均从共同初始姿态单独通过标准 EGL 的 717 步回放，实际抬升约 305.13–368.15 mm，无机械臂、双手间、同手碰撞、越限或物理警告。逐回合 HDF5、结果与种子 19 的修正轨迹位于数据盘 `benchmarks/sponge-seeded-reference/common-start/fixed-start-egl-summary.json` 指向的文件。种子 1、16 重新逐 50 ms 采集，各保留 844 帧 RGB-D 点云及关节、腕位与动作，见同目录 `controller-rate/sponge-seed-{01,16}.{hdf5,json}`。

### 捡球的 20 个随机摆放标准复放

初始收拢无名指后，中指与球的法向载荷曾从约 0.38 N 降至零，导致搬运中滑脱。调整中指的两关节目标，在第 96–140 个动作间逐步增加 0.03 rad，并依据实际持球位置规划两段搬运。闲置左手张开，左腕沿 y 轴移出搬运区 120 mm。第 20 个种子按实测腕位误差额外补偿搬运终点 `[15, -40] mm`。保留各动作实际执行的控制周期数。

在 `pro1` 上，用真实 EGL 相机和相同种子复放 20 份轨迹。每份 345 步，种子 1–20 全部完成抬球、松手及落篮，球实际由篮底承托。机械臂接触步、双手穿透和采样到的同手最大穿透均为 0，无物理警告。每回合生成 `10000 × 6` 点云。这里验证的是按初始摆放生成的参考动作。HDF5 文件及实测逐回合汇总 `three-pad-open-left-seeds-01-20-egl-summary.json` 在数据盘 `benchmarks/ball-seeded-reference/world/stowed/`。

在相同场景把其中四条完整参考转成 DexCap 训练 HDF5，逐步写入 EGL 实测点云、机器人本体状态和实际执行的动作目标，共 1,380 帧。种子 1–3 训练，种子 4 验证。训练加载器逐个 HDF5 检查训练和验证掩码是否重叠，并让验证动作使用训练集的归一化参数。使用 PointNet 扩散策略从头训练 1 轮（20 个更新），验证损失 1.00063，已保存检查点。独立种子 5 的闭环评估在第 33 步因无效手部运动提前终止，球未抬起、未落篮。数据集、配置、检查点和日志位于 `/data/dexcap-repro/benchmarks/ball-simulation-policy/`。

```bash
ball=/data/dexcap-repro/benchmarks/ball-seeded-reference/world/stowed
python -m simulation.rollout "$ball/seed-20-three-pad-open-left-reference.hdf5" \
  --task ball-collecting --first-seed 20 --output "$ball/seed-20-three-pad-open-left-reference-egl-standard.json"
```

### 收球的固定起点与留出评估

用原先随物体摆放调整右腕起点的 16 条轨迹生成 5,520 帧。种子 1–12 训练，13–16 验证，17–20 留出。PointNet 扩散策略训练 30 轮，每轮 100 个更新。在保存的检查点中按验证损失选第 25 轮，损失 0.17516。对种子 17–20 各测试一次原参考起点和共用种子 1 起点，八次均未抬球，手指越限导致提前结束。示范中闲置左手的手指目标恒为零。原反归一化依然允许网络噪声改变这些目标，现按训练集的恒定分量还原，保留平面平移增强能够改变的腕位分量。用同一权重、同一初始化和相同种子重新评估，仅从训练示范补齐恒定动作的反归一化系数。八个配对回合均运行到 400 步，无手指越限、双手碰撞或物理警告。采样机械臂接触步数中位数从 255.5 降到 0，仍无右手触球、抬球或落篮。原始数据、检查点和配对报告在 `/data/dexcap-repro/benchmarks/ball-simulation-policy/`。

另从统一右腕世界坐标 `[0.563, -0.159, 0.324] m` 开始，在高位实际移动到各摆放的抓球通道，接续完整物理动作。左腕起点也一致。20 个种子的 399 步原生复放均抬球、释放并落篮，没有机械臂、双手、同手穿透及物理警告。留出的 17–20 又由标准 EGL 入口独立复放，四条均通过。完整原生轨迹为同目录 `neutral-ball-seed-*.hdf5`，训练用的逐步实测点云导出到 `neutral/`。

策略闭环现在每条动作只推进一个 50 ms 控制周期，与论文的 20 Hz 控制频率一致。旧入口会等待腕位跟踪，八个已修复归一化的 400 步历史回合分别用了 161.4–197.05 s 仿真时间。新入口独立测试 12 个策略动作实测 0.60 s。同一引用轨迹的 60 步复放在给定的 75 个控制周期内推进 3.75 s。记录为 `pointnet-cadence-20hz-smoke.json` 和 `recorded-cadence-smoke.json`。 [DexCap 实验设置](https://arxiv.org/html/2403.07788v2#S5.SS1)

固定起点的 16 条轨迹已导出 6,384 帧。仍按种子 1–12 训练、13–16 验证，训练 30 轮后从已保存的检查点中按验证损失选第 20 轮，损失约 0.15069。种子 17–20 从同一固定腕位分别跑满 400 个闭环动作，每次实测 20.0 s，无机械臂接触、双手碰撞、手指越限或物理警告，四回合均未触球。数据、配置、训练日志、权重与测试输出在 `benchmarks/ball-simulation-policy/neutral/`。下一批训练数据按实际 50 ms 控制周期采集观测，保留原轨迹中的真实保持时间。

按每个真实 50 ms 控制周期采集的数据已独立完成 16 条轨迹的抬球与落篮，训练种子 1–12 共 7,245 帧，验证种子 13–16 共 2,284 帧。验证损失最低的已保存检查点为第 30 轮，损失 0.11876。四个留出种子 17–20 各执行 1,000 个动作并推进 50 s，均无机械臂接触、双手碰撞、手指越限或物理警告，右手触球次数均为 0。直接以四条示范的真实观测输入该策略，起始点云中每帧可辨认的球面点在起步阶段最少为 436 个，前 54 步的右腕预测目标误差中位数为 97.0–104.0 mm。下一项有依据的诊断是动作预测与条件输入的对应，而非延长物理评估。数据、训练与测量文件位于 `benchmarks/ball-simulation-policy/at-20hz/`，离线诊断为 `pointnet-20hz-teacher-state-probe.json`。保持同一权重和输入，将 DDIM 推理步数由 10 增至 50，四条示范的早段右腕目标误差中位数仍在 87.7–125.2 mm，未获得一致改善。记录为 `pointnet-20hz-teacher-state-50steps.json`。

同一收球数据上的时序对照保持三帧观测和 20 目标预测窗，将训练样本开端改为以真实过去帧对齐，并给冷启动样本补齐首帧。30 轮时未降低前 54 帧的离线右腕目标误差。延长原对齐到 90 轮后，四条示范的该误差中位数由约 97–104 mm 降至 37–57 mm。验证噪声损失从 0.11749 升至 0.17494。沿用 20 Hz 和固定初态，原诊断种子 17 与新种子 21–24 的五次闭环均执行完整 1,000 步、无机械臂或双手碰撞、无越限，触球仍为 0。对种子 17 的实际物理提取显示，参考第 7.15 s 首次触球，策略前 15 s 的最近指尖仍距球约 59.3 mm，右腕最低约 234 mm，参考最低约 125 mm。相关对照、闭环和指尖读数位于 `benchmarks/ball-simulation-policy/at-20hz/history-ablation/`。

在相同的 9,529 帧训练及验证数据上，用现有 SetXF 编码器训练 90 轮。相同四条示范的前 54 步右腕误差中位数为 30.1–53.5 mm。固定初态的种子 17、21 经标准 EGL 策略入口各完成 400 步，两次均未触球、抬球或落篮，无手指越限及物理警告。报告为同目录 `setxf-epoch90-chunk10.json` 和 `setxf-epoch90-rollout-{17,21}.json`。

用种子 17 对照相同初态的真实腕位。参考在 7.15 s 首次触球，腕位最低约 124.9 mm。SetXF 前 11.5 s 的实际腕位最低约 280.0 mm，最近的手部碰撞面距球约 111.9 mm。以同一示范的真实观测直接推理，把预计接触阶段的坐标转换分开计算，SetXF 的平移分量中位误差约 62.5 mm，由腕向变化和安装偏移引入的分量约 2.1 mm。PointNet 同一段分别为 50.8、1.6 mm。实际指尖距离和分阶段误差见同目录 `setxf-epoch90-approach.json` 及 `{pointnet,setxf}-epoch90-error-source.json`。

同一数据使用现有 BC Transformer 直接回归动作。第 30 轮用种子 17 示范观测预测时，初段、接近、接触和抓取段的右腕误差中位数分别为 7.7、10.7、12.0、12.1 mm。用该权重从固定初态跑种子 17、21，各 650 步均未触球。种子 17 的腕位最终停在约 `[0.462, -0.219, 0.316] m`，同场参考触球时为约 `[0.575, -0.209, 0.125] m`。该试验位于同目录 `bc-transformer-epoch30-{error-source,rollout-17,rollout-21,approach-17,approach-21}.json`。

在相同 PointNet 网络和数据上另做直接预测干净动作的扩散对照，加入噪声的方法、输入和控制周期均保持一致。第 30 轮在种子 17 的闭环中于 4.35 s 触球，650 步累计 248 个采样接触，腕位最低约 124.7 mm。球沿桌面移动约 102 mm，未抬起。首触时腕部相对球的 x 偏移约 38.7 mm，参考首触约 11.5 mm。在该策略闭环的前 11.5 s 采样中，右食指仅一个采样承力超过 0.02 N。虽有 54 个相向接触采样，全部仍有桌面载荷；参考到其第 210 个目标共测得 81 个无桌面载荷的相向接触采样。诊断与训练源位于同目录 `diffusion-clean-action-epoch30-{rollout-17,approach-17,grip-17}.json`、`teacher-grip-17.json` 及 `clean-action-source/`。

同一检查点在两个新种子 21、22 的 650 步闭环没有触球，最小指尖间隙分别为 10.7、55.7 mm。另用同样的三帧 BC Transformer 对照 RGB-D 定位，输入由已有 XYZRGB 观测提取的篮筐和球初始位置、可见球的实时位置及可见性，加上原双手本体状态。训练与闭环都用相同提取代码，策略未读取模拟器的物体状态。16 条训练及验证轨迹的首帧定位误差最大约 2.91 mm（篮）、0.031 mm（球），9,529 帧中 8,774 帧获得球的拟合。第 45 轮在种子 17、21 各跑 650 步，分别采样到 211、374 个接触。两轮都只沿桌面移动了球，未抬起或落篮。文件为同目录 `ball_camera_geometry.py`、`bc-camera-geometry.json` 和 `bc-camera-geometry-epoch45-rollout-{17,21}.json`。

用训练种子 1–12 的低位接近动作估计右腕相对球的平面偏移约 `[11.42, -19.55] mm`。只在右腕接近时利用当前 RGB-D 球位置纠正平面目标，保留策略的指令形状。种子 17、21 各 550 步，指尖到球的最小间隙分别由不纠偏时的几何接触变为 1.33、0.81 mm，球保持原摆放，未触球。末态腕位约 132.0、129.6 mm，高于种子 17 物理参考的首次触球腕位约 125.1 mm。对照输出为 `bc-camera-aligned-{rollout,approach}-{17,21}.json`。

### 额外收球摆放与腕位恢复

种子 21–24 从固定机器人初态独立执行 399 步标准 EGL 参考，四回合均抬球、释放并在实际篮底静置，机械臂、双手、同手碰撞计数及物理警告为零。种子 23 原两段搬运在 Panda 关节限位处到达不了篮心，保持同一真实三指夹持，用绕世界 x 轴 24° 的逐步转腕将球移动到腕前，随后达到篮心。参考在数据盘 `benchmarks/ball-simulation-policy/neutral-ball-seed-{21..24}.hdf5`，报告在 `at-20hz/extra-layout-reference/`。这些新布局用于对照评估，没有加入训练。

在相同 RGB-D 平面纠正基础上，下探时用训练示范的低位目标 124.37 mm 继续同一策略的手指动作。种子 17、21 的球最高分别上升约 26.60、27.16 mm。前 15 s 中分别有 84、83 个采样同时满足离桌且真实相向承力，三个接触几何为食指末节、中指和拇指末节。两组均未达到 30 mm 任务阈值，也未入篮。控制脚本与采样在 `history-ablation/trace-relocated-wrist-low-policy.py` 和 `bc-camera-aligned-low-{approach,grip,rollout}-*.json`。

同一模型的独立训练对照另采集了六次实际腕位偏移后的恢复段，来自既有训练种子 1–6。先以完整机械臂执行偏移，再记录纠正目标、当前 RGB-D 几何和本体状态，扰动目标不作为监督。六段最终均通过物理落篮，腕位初始误差范围约 20.8–313.5 mm，进入接近通道时误差约 0.16–21.75 mm。记录的 1,350 帧只加入训练，目标验证集保留原种子 13–16。配置、合并数据和生成记录位于 `history-ablation/bc-camera-recovery.json`、`ball-20hz-camera-with-recovery.hdf5` 和 `recovery/`。

在实测无桌面承托后，加入 36 个周期的短程上提目标，手指动作仍由同一权重预测。两个 650 步回合分别实际升高 46.43、45.85 mm 并通过抬球判据，均未入篮，无机械臂或双手间碰撞、手指越限或物理警告。文件为同目录 `bc-camera-aligned-lift-{approach,rollout}-{17,21}.json`。

在恢复数据上用原 BC 配置训练 45 轮，完全关闭腕位辅助后，种子 17、21 的 650 步闭环分别采样 596 次和 0 次触球，均无抬球和落篮。直接干净动作扩散对照的第 60 轮在验证种子 13、16 的接触阶段，右腕目标误差中位数分别为 11.4、12.9 mm。对照文件为 `bc-camera-recovery-epoch45-rollout-{17,21}.json`、`diffusion-camera-recovery-epoch60-wrists.json`。 该扩散权重在种子 17、22 的各 650 步无辅助闭环中均未触球，实际指尖最近仍距球约 153.8、151.1 mm。见 `diffusion-camera-recovery-epoch60-rollout-{17,22}.json`。

从上述短程上提结束时的真实三指抓持继续，以实际抓持相对位姿、机械臂关节范围和篮网几何求搬运路径。保持当时策略输出的有限力手形，实际越过篮沿后用训练示范的张手目标释放。种子 17 保持腕朝向，种子 21 绕世界 x 轴逐步转腕 24°。两个 560 步回合均完成抬球、释放与物理篮底静置。保存实际执行的 20 Hz 命令后，从同一初态用标准 EGL 入口独立复放，均推进 28.0 s，产生 `10000 × 6` 点云，触球采样数与原回合一致，均无臂、双手间、同手碰撞、越限或物理警告。这里验证的是用学习策略产生抓持手形并辅助腕位和搬运的执行。工件为同目录 `bc-camera-assisted-transfer-{rollout,replay}-{17,21}.json`、`bc-camera-assisted-transfer-reference-{17,21}.hdf5` 及搬运的轨迹规划记录。

在原训练摆放的种子 7、8、10 运行同一辅助抓取与搬运，其中 7 和 10 完成落篮，8 未抬球。只从前两条回合的完整 620 步实际命令重新执行物理、逐 50 ms 采集 RGB-D 几何与本体状态，分别保留 403、453 帧。两回合首次满足任务条件在第 387、437 步。只增加到原训练分组，验证与测试布局保持原样。

使用同一 45 轮 BC 配置对照两种右、左腕位表示。绝对目标在种子 17、21 的各 650 步无辅助闭环中均未触球，两回合腕位最低均在 318 mm 以上。以相对当前本体观测的平移作为目标时，种子 17 未触球，种子 21 采样到 31 次手球接触，两回合均未抬球或落篮。只变化动作前六个平移分量。数据及结果位于 `history-ablation/ball-camera-with-assisted{,-relative}.hdf5`、`assisted-training-seed-{07,10}.json` 和 `bc-camera-assisted{,-relative}-epoch45-rollout-{17,21}.json`。

以同一相对腕位数据再加 20 Hz 的经过时间观测，保持其余模型配置、训练划分和运行控制周期。两个无辅助回合各 650 步，种子 17、21 的指尖最近分别离球约 14.0、123.2 mm，均未触球。种子 21 出现 276 个有机械臂接触的控制周期。结果为 `bc-camera-relative-clock-epoch45-rollout-{17,21}.json`。
