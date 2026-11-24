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

按 LEAP 网格改为 50 × 40 mm 的双指孔及 40 × 42 mm 的拇指孔，并补齐两侧连到被动铰链的柄臂。保守包络检查得到指面与孔边的总净余量约 5.3–6.7 mm，闭合位置两环仍分离约 14.6 mm。新版机构试验仍能在实际刀刃接触下切开 4 mm 接缝。测量见 `runs/scissor_handle_fit_check.json`，新版预插入试验验证了两指能在两个孔内保持正净空。物理执行中，接触只短暂支撑了剪刀，两个柄孔未同时承载。默认桌面姿态的尝试还出现约 25.1 mm 的手与桌面或托盘穿透。结果见 `runs/scissor_leap_new_handle_fixture.json` 和 `runs/scissor_leap_tabletop_reach.json`。随后用右手食指从剪刀轴附近托住 0.62 s，实际抬升约 25.6 mm，活动把手与托盘隔板的峰值净空增加约 117 mm。按抬升后的姿态拟合左手，食指和拇指分别有约 2.6 mm 和 12.1 mm 的柄孔净空。固定柄贴着托盘边墙，左食指连杆的拟合插入位置与墙重叠约 14 mm，尚未执行插入与交接。记录在 `runs/scissor_right_shank_tilt.{py,json}` 和 `runs/scissor_bimanual_support_insert.{py,json}`。

复核图 5(e) 的首帧可见，剪刀两柄伸出托盘短端，刀片在托盘内。场景据此保留四面外墙，沿托盘长边增加贯通隔板，并在剪刀所在的前排加入短隔板。默认剪刀以有真实托盘接触的校准姿态静置。纸带初始仍平放，尝试让现有柔性纸带靠墙立放时，约 0.1–0.2 s 内便折落。记录在 `runs/scissor_source_layout_ab.{py,json}` 和 `runs/scissor_source_upright_paper.{py,json}`。 [图 5(e)](https://arxiv.org/html/2403.07788v2/tasks_v5.png)

在正式布局中，剪刀在 0.6 s 悬停准备期间仅移动约 0.13 mm。继续插指时拇指首先接触活动柄孔上缘，随后剪刀滑移约 82 mm。两个柄孔各有接触样本，未在同一物理子步同时承载。正式场景的腕误差、柄孔和托盘接触见 `runs/scissor_production_left_insert.{py,json}`。

默认平放纸带已移到前格内。原摆放与长隔板重叠 0.8 mm，首个子步产生 40 个接触。新摆放净空为 27 mm，没有首步隔板接触。另一次声明初始贴近端墙的右手试验中，食指触到纸带、拇指未接触，纸带落回托盘，接缝保持完整。记录在 `runs/scissor_right_paper_pickup.{py,json}`。

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

## Tea Preparing

`python -m simulation.rollout --task tea-preparing` 使用固定茶盘、可移动的空心瓶与茶壶、弹簧镊子和可自由落下的茶粒。瓶盖有独立自由关节。螺纹通过 MuJoCo 的转角与轴向滑动约束表达，达到两圈并实际升到约 4 mm 的退丝位置后解除连接，瓶盖保留当时的位姿和速度。壶内茶粒必须由壶体实际支撑并低速稳定，工具归位依靠实际托盘接触。论文规定了两圈和任务顺序，螺距、器具尺寸、材料与归位容差为估计。 [论文任务定义](https://arxiv.org/html/2403.07788v2#A1.SS6)

在支稳茶瓶的测试中，对帽体施加 0.007 N·m 力矩，转一圈时实际升约 2 mm 并保持连接，接近两圈和 4 mm 后成为独立刚体。正式双臂场景中，从壶口上方放开的一粒茶在约 1.75 s 后由壶底承托并计为入壶。整段双臂开盖、辅助倒茶和归位仍缺作者的动作与权重。结果见 `runs/tea-pot-production-drop.json`。

把瓶体固定为试验夹具，右侧 Franka 与 LEAP 靠真实手指接触驱动瓶盖达到 **1.19 圈**，相应轴向行程 2.32 mm。重新夹持过程中会丢失非拇指接触，仍未达到两圈退丝。记录见 `runs/tea-leap-cap-robot-probe.{py,json}`。另一次换指试验在转到约 0.58 圈后失去拇指制动接触，盖回退至约 0.48 圈，见 `runs/tea-leap-cap-hinge-regrasp-probe.{py,json}`。固定腕后用手指交替推进，瓶盖在两个切向推动方向均回退。记录在 `runs/tea-leap-cap-finger-gait-probe.{py,json}`。


最后一段退丝单独使用固定瓶身、编译时预旋约 1.7 圈的夹具。右手通过接触驱动剩余螺纹行程，盖在约 4 mm 轴向位移处释放。右手随后继续接触，将已独立的瓶盖上抬约 9 mm。记录在 `runs/tea-leap-cap-last-turn-lift-probe.{py,json}`。从零开始的重新抓握仍有回旋。提高螺纹干摩擦虽能抵住重力回退，却降低了换指过程的累计转角。另一次尝试沿瓶轴退离、径向重新靠近，盖在无手部接触的退离阶段已回到零，之后掌部和指节接触又产生偏心转矩。见 `runs/tea-leap-thread-friction-regrasp-probe.json` 和 `runs/tea-leap-cap-axial-radial-regrasp-probe.json`。


自由瓶身的左手探针将腕跟踪误差降到约 1.55 mm，接触仍在瓶体同一侧。瓶体仍受茶盘承托，未达到离桌对向夹持。记录在 `runs/tea-leap-joint-clearance-wrap-probe.{py,json}`。

另一条轨迹用左手侧推，让起始竖直的自由瓶身绕托盘接触点倾斜。瓶身倾到约 82° 时出现了拇指和食指的真实相向接触。瓶身随后沿托盘滑约 19 mm，食指接触丢失。记录在 `runs/tea-leap-tilt-pivot-grip-probe.{py,json}`。 跟随瓶身平移时，相向接触最长只持续约 50 ms。跟踪瓶身相对姿态的试验在首个保持周期出现约 52 mm 左腕误差，未进入提瓶。

在托盘内将自由瓶身初始放在 `[0.47, -0.075] m` 的边框试验中，左手经真实接触将瓶身推到边框，拇指和食指相向接触持续 0.3 s。瓶身随后在对向接触下沿边框滚回内侧，仍由托盘底承托。请求上提 24 mm 时实际瓶身只升约 1 mm。记录在 `runs/tea-leap-rim-supported-lift-probe.json` 和 `runs/tea-leap-rim-roll-exit-lift-probe.json`。

## 六个场景的渲染与接入验证

六个场景均以同一套双 Franka、双 LEAP、OSC 和 RGB-D 入口在 `pro1` 加载，执行实际控制并输出 `10000 × 6` 点云。35 项 MuJoCo 专测和汇总测试通过。场景总览图与接入记录保存于数据盘 `runs/task-scene-overview-grasp-physics/six-mujoco-scenes.png` 和 `runs/task-scene-overview-grasp-physics/smoke.json`。Scissor Cutting 和 Tea Preparing 已验证关键机构的物理响应，仍缺整段双臂任务动作。


## 校正 LEAP 指尖碰撞与惯量

DexCap 的重定向 URDF 比 LEAP V1 官方物理模型多四个 `realtip` 固定链接。它们各带半径 5 mm 的圆球和 0.1 kg 的惯量条目，用于表示指尖 IK 目标。现在构建 MuJoCo 手部时只加载官方物理模型对应的 17 个实体链接。原始 DexCap URDF 仍供发布数据的重建使用。编译后每只手质量由 0.909 kg 校正为 0.509 kg。 [LEAP V1 官方 URDF](https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf)

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


海绵接触参数更新后的 40 个 Wiping 回合中，旧判据给出 PointNet 0/20、SetXF 2/20 成功。两条被判成功的 SetXF 回合的峰值接触力约 15–16 kN，原判据还允许未同时持物的盘面接触累计为有效笔画。两条种子的单独复跑均未重现成功。现在只在右手相向夹持已离桌的盘子、左手相向夹持已离桌的海绵，并发生内盘面接触时累计擦拭笔画。保留原始内盘面接触次数供诊断。原始汇总和复跑记录在 `runs/sponge-contact-benchmark/`。
