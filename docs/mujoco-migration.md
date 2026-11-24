# DexCap MuJoCo 迁移

将 Wiping 和 Packaging 的示范回放与策略评估迁到同一套双 Franka、双 LEAP 的 MuJoCo 环境。Packaging 保留六个训练物体、九个未见物体和五组配对摆放。论文还描述了 Sponge Picking、Ball Collecting、Scissor Cutting 和 Tea Preparing。公开数据仓库提供 Wiping 和 Packaging 的示范，另外四项的官方轨迹与训练权重仍需要数据来源。 [论文任务](https://arxiv.org/html/2403.07788v2#A1.SS6) · [作者数据发布](https://huggingface.co/datasets/chenwangj/DexCap-Data)

## 模型与控制

用 Menagerie 的 Franka Panda 机械臂和本项目已有的 LEAP 关节定义构建完整机器人。仿真中的实际腕位和 32 个手指关节值转回发布数据的坐标及数值格式，沿用 46 维动作和三帧点云观测。用腕位目标驱动机械臂，在物理子步内执行力矩或受限力控制。手指通过关节目标和有限驱动力运动。任务物体通过碰撞、摩擦和重力移动。基座位置、法兰到掌座的变换以及物体材料参数需要对照公开标定和原始示范验证。


### LEAP 关节止挡

在满载泡茶的右手试验里，旧模型虽然读取了 URDF 的角度范围，但实际指尖关节曾越限 3.47 rad，MuJoCo 未产生警告。上轮两圈退丝的手形也已越限，因此该轨迹作为诊断记录保存。当前 32 个 LEAP 关节继续使用 URDF 原角度范围，止挡时间常数从 20 ms 调为 4 ms，并加入估计的 0.0005 kg·m² 电机反射惯量。以相当于一个伺服的 0.95 N·m 向指尖关节两端施加隔离负载，峰值越限从约 0.0433 rad 降到 0.0007 rad。相同泡茶接近、合拢及首段转腕的对照中，最大越限从 3.47 rad 降到约 0.0004 rad。只改止挡而不加反射惯量的对照仍越限约 2.60 rad。未对真实舵机惯量做实物标定。 [MuJoCo 关节与反射惯量](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-joint)

改动后通过标准入口重放四份完整参考。305 步 Sponge Picking 抬升约 303.46 mm，265 步 Ball Collecting 完成落篮。586 步红银包装参考在第 119 至 420 动作保持物体，随后按原动作张手时物体仍在盒外。这份参考能合盖。另复核 463 步转运参考，物体由右手实际搬运、释放并留在盒内，纸盒位移约 0.97 mm。这份参考尚需接上闭盖。946 步 Wiping 参考抓起海绵，盘子未形成对向夹持。第 154 动作只有右食指接触盘边，拇指仍差约 27.5 mm。单独补偿拇指没有建立对夹。Scissor Cutting 和 Tea Preparing 分别跑了 8 步及 1 步的入口复测。六次回放都输出 `10000 × 6` 点云，逐物理子步测得最大的手指越限约 0.00492 rad，出现在 Wiping。六次均无求解器警告。旧模型下的其它参考成功记录仍需按现行资产复核。数据见 `runs/tea-leap-limit-load-*.json` 和 `runs/leap-stop-physical-*.json`。红银转运和抓盘诊断分别见 `runs/leap-stop-physical-red-scripted-placement-audited-762a293.json` 和 `/data/dexcap-repro/benchmarks/wiping-leap-stop-right-grip-diagnostic.json`。

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

历史 PyBullet 仿真代码和入口保存在基线提交。当前分支的 `simulation.rollout`、示范图和策略动作诊断均运行 MuJoCo。官方数据重建仍沿用作者的 PyBullet 指尖 IK。 [基线提交](https://github.com/ZSHYC/DexCap-repro/tree/0ee5418)


在复合掌心模型上以发布的 Wiping HDF5 与已有的第 3000 轮权重重跑配对闭环。两种编码器各取相同的种子 1–20，每回合上限 400 个策略动作。PointNet 有 3 个回合抓起盘、2 个抓起海绵、1 个回合两物均曾抓起。SetXF 的对应计数为 2、4、0。两种编码器各有 2 个有效回合接触内盘，均未完成完整擦拭，成功数均为 0/20。完整轨迹可在固定摆放下通过标准回放，随机闭环表统计已有权重的实际输出。数据与启动命令位于 `runs/wiping-compound-palm-benchmark/`。

SetXF 的种子 9 触发了 MuJoCo `mjWARN_BADQACC`，原始日志记录于模拟时间 23.02 s。分析时按物理异常回合计为失败。现在标准入口遇到 MuJoCo 警告会终止该回合，并记录 `executed_steps` 和 `physics_warnings`。异常回合计入总尝试次数，其任务指标在汇总中不记为通过。


种子 9 的独立复盘把异常定位到盘子自由关节的旋转自由度。第 60 个策略动作中，仿真报告海绵贯入盘底约 68 mm，盘子被异常接触力抛离桌面。第 73 个动作盘子撞到房间地面后触发数值警告。以上是失稳轨迹的数值记录。逐物理步诊断见 `/data/dexcap-repro/benchmarks/wiping-setxf-seed9-badqacc-diagnostic.json`。


## 115 回合闭环结果

| 权重及任务 | 回合 | 抓取或入盒 | 内盘面接触或合盖 | 完整成功 |
| --- | ---: | ---: | ---: | ---: |
| PointNet Wiping | 20 | 两件都曾抓起 1 | 内盘面接触 0 | 0 |
| SetXF Wiping | 20 | 两件都曾抓起 0 | 内盘面接触 0 | 0 |
| SetXF Packaging 训练物体 | 30 | 入盒 0 | 合盖 4 | 0 |
| SetXF Packaging 未见物体 | 45 | 入盒 0 | 合盖 7 | 0 |

115 个回合均使用 MuJoCo 3.6.0 和校正后的 LEAP 实体网格。Wiping 结果来自这批评估运行时的 125 mm 半径盘型。逐回合核对确认数据、权重、步数和初始摆放与 `runs/leap-physical-benchmark/` 中的结果一致。Wiping PointNet 有 2 个回合抓起盘子、2 个回合抓起海绵，SetXF 分别为 0 和 3。表中的内盘面接触统计原始几何接触。累计擦拭笔画要求在同一次计分采样中双手都实际持物。Packaging 的接触参数已更新，75 个回合按后续加入 0.1 mm 入盒边界容差之前的条件计分。论文中的成功率来自实体机器人。 [逐物体汇总](assets/dexcap-formal-mujoco.json) · [论文评估](https://arxiv.org/html/2403.07788v2#S5.SS1)

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


蓝积木接近纸盒时还需要最低角增加约 49 mm 净空。试验中沿当前箱缘跨入需要纸盒倾斜约 31°，所执行的左手路线未接触到箱壁。单独保持右手末动作 0.30 s 时蓝积木失去夹持，随后将左手接近与原拾取动作同步，首次碰撞出现在左臂下降阶段，右手随后失去夹持。未执行放入。记录见 `runs/packaging-blue-{box-tip,apex-brace-feasibility,sync-left-prepose}-66b7507.json`。

用 `demo_100` 的参考摆放对绿梨做椭球短轴两侧接近。受力前两指垫的纵向位置仍偏高约 19 mm，实际下降时两侧没形成对向夹持。绿梨向盒壁移动，始终由桌面支撑。见 `runs/packaging-green-pear-fit-66b7507.json`。

绿罐的官方 `demo_150` 参考有 143 个右手接触周期。减少折叠预抓姿态的自碰撞后实际执行下探和抬升，仍未建立持续的相向抓持。记录见 `runs/packaging-green-can-official-contact-trace.json` 和 `runs/packaging-green-can-neutral-pregrasp-lift-probe.json`。

分块掌心模型上，红银包装仍落入盒底。旧收尾的 115 个左手接触周期把盖子沿开盖方向拨动，最低到约 −4.2°。重选实际折舌接触位置后，左手可通过接触把自由盖子带到约 64.2°，峰值约 71.5°，期间包装始终在盒内。这条动作依赖施加目标时的控制周期数，撤手时盖子重新打开。用标准回放时序重放也没有保持折舌接触。记录见 `runs/leap-palm-packaging-lid-hook-continuation.json` 和 `runs/leap-palm-packaging-lid-arc-finish.json`。

复合掌心下重新完成了红银包装的入盒和闭盖。左手拇指通过真实接触勾住纸盒折舌，把盖子带到约 64.6°，随后沿盖子边缘越过，撤手后盖子稳定在约 180.3°。586 个动作的标准回放确认包装仍在盒内、已释放、盒盖闭合，最终 `sim_success=true`。整段无机械臂碰撞，纸盒平移约 0.67 mm。这次验证覆盖红银包装的固定参考摆放。轨迹将每个动作实际执行的 50 ms 控制周期数保存在 `control_cycles_per_action` 中，前段保留原自适应时序得到的执行长度，闭盖尾段使用实际的 5 或 7 个周期。动作生成器和实测记录见 `runs/packaging-red-carton-live-unbroken.{py,json}`。

```bash
python -m simulation.rollout data/derived/packaging-red-carton-live-timed-reference.hdf5 \
  --task packaging --object red-box --output runs/packaging-red-carton-live-timed-rollout.json
```


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

这两份轨迹使用旧止挡。保存状态的逐关节复核证实，退丝前个别指尖关节已偏出 URDF 范围 3.45 rad，退丝后还有关节偏出约 1.81 rad。当前模型仍需重新取得两圈退丝证据。泡茶的握持计分现在按 LEAP 的实际手指区分接触，避免把一根手指的两个连杆算作有效对握。

在现行止挡模型上单独验证取盖。夹具固定满载瓶身，起始声明盖子已脱扣并停在瓶颈，左臂在停放位。右手食指和拇指实际相向夹持，使盖子完全脱离瓶颈并上升 56.3 mm，连续 0.44 s 的采样同时满足对向受力与不接触瓶身。增加合拢预载后，盖子在同样条件下横移 56.3 mm，之后滑脱，尚未放置到托盘右侧。取盖与搬运试验所记录的两个手的关节均未越出 URDF 范围。记录见 `runs/tea-detached-cap-home-fixture-pinch.json` 和 `runs/tea-detached-cap-home-fixture-place-preload.json`。

左臂以真实接触把默认满载自由瓶推到托盘壁，最大支撑倾角约 101.43°，壁面峰值约 9.46 N。现行手部参数下，进一步合拢只让拇指碰到瓶身，未建立相向夹持。两手每个 2 ms 物理子步的实测关节均在 URDF 范围内。试验及腕部中点再推进 10 mm 的对照见 `runs/tea-loaded-xneg-rim-ab24e72{-telemetry,-midpoint40-telemetry}.json`。

## 六个场景的渲染与接入验证

六个场景均以同一套双 Franka、双 LEAP、OSC 和 RGB-D 入口在 `pro1` 加载，执行实际控制并输出 `10000 × 6` 点云。35 项 MuJoCo 专测和汇总测试通过。场景总览图与接入记录保存于数据盘 `runs/task-scene-overview-grasp-physics/six-mujoco-scenes.png` 和 `runs/task-scene-overview-grasp-physics/smoke.json`。Scissor Cutting 和 Tea Preparing 已验证关键机构的物理响应，仍缺整段双臂任务动作。关闭离屏多重采样后，重新用五个场景各执行一步，并与满载泡茶三步结果一起确认六个场景都输出相同结构的点云，且无物理警告。记录见 `runs/no-msaa-*-entry.json` 与 `runs/tea-production-filled-entry.json`。


## 校正 LEAP 指尖碰撞与惯量

DexCap 的重定向 URDF 比 LEAP V1 官方物理模型多四个 `realtip` 固定链接。它们各带半径 5 mm 的圆球和 0.1 kg 的惯量条目，用于表示指尖 IK 目标。现在构建 MuJoCo 手部时只加载官方物理模型对应的 17 个实体链接。原始 DexCap URDF 仍供发布数据的重建使用。编译后每只手质量由 0.909 kg 校正为 0.509 kg。 [LEAP V1 官方 URDF](https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf)

原掌部 STL 含明显的凹槽。将整个 STL 作为单一碰撞网格时，MuJoCo 用凸包填平凹槽。中性姿态中原有两处拇指与掌部假穿透约为 12.5 mm 和 5.0 mm。对真实三角网格的测量确认这两处接触点均在释放的掌壳外。碰撞掌部现在使用从原 STL 生成的 64 个凸块。主壳体用 CoACD 1.0.14 分为 16 块，其余 48 个连通部件各保留一个凸包。双手中性姿态的原两处接触均消失，另外的真实指节碰撞仍发生，掌部质量仍为 0.237 kg。渲染使用同一原始 STL。运行只读取提交中的分块资产，CoACD 用于离线生成。分解参数和实测点对比在 `runs/coacd-palm-hulls/`，双手实测接触在 `runs/leap-palm-decomposition-ab.json`。 [MuJoCo 凸碰撞与网格](https://mujoco.readthedocs.io/en/stable/computation/index.html#convex-collisions)

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


海绵接触参数更新后的 40 个 Wiping 回合中，旧判据给出 PointNet 0/20、SetXF 2/20 成功。两条被判成功的 SetXF 回合的峰值接触力约 15–16 kN，原判据还允许未同时持物的盘面接触累计为有效笔画。两条种子的单独复跑均未重现成功。现在只在右手相向夹持已离桌的盘子、左手相向夹持已离桌的海绵，并发生内盘面接触时累计擦拭笔画。保留原始内盘面接触次数供诊断。原始汇总和复跑记录在 `runs/sponge-contact-benchmark/`。
