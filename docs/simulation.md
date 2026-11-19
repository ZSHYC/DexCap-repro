# DexCap 仿真

项目基于公开的 Wiping 和 Packaging 示范建立了无界面的 PyBullet 场景。作者公开了手部重定向和训练代码，并在上游说明仿真回放需要另外构建场景数字孪生。这里使用官方 LEAP 手 URDF，根据各任务 `demo_0` 的图像、点云和手部轨迹估计场景。 [作者关于仿真的说明](https://github.com/j96w/DexCap/issues/6#issuecomment-2209325098)

策略的两只手使用各自的腕部坐标。仿真用示范的 `frame_0_eef_quat` 和发布的标定配置将目标映射到同一个 Bullet 世界，执行后将模拟器的手部位姿与关节状态变回策略坐标。相机使用 HDF5 中的第一帧外参、发布的 L515 内参和 Bullet 深度缓存生成 `10000 × 6` XYZRGB 点云。控制频率为 20 Hz，物理步进为 240 Hz。

仿真和原始数据重建共用官方的台面平面裁剪。Bullet 以 L515 原始分辨率的四分之一宽高渲染，因此裁剪的像素数阈值除以 16。若裁剪后可见点不足 10000 个，则有放回采样。

官方数据构建按手部中心投影是否落在 84 × 84 图像内决定是否加入该手，再对完整 LEAP 网格采样并将 RGB 置零。仿真用同样的可见性规则，从 Bullet 各链节的实际位姿恢复手部网格表面。每只手在裁剪前采样约 5000 点，按渲染分辨率将官方的 80000 点除以 16。渲染时保留手对物体的遮挡，从深度点云中去掉手部像素，再加入手部表面采样，一同裁剪和下采样。桌面和物体仍使用渲染颜色。简化场景会改变手部点和场景点的比例。 [官方构建逻辑](../DexCap/STEP2_build_dataset/dataset_utils.py) · [对照结果](runs.md)

两只 LEAP 手的掌部按目标做运动学移动，任务物体是动力学刚体。手碰到对应物体时，模拟器用固定约束近似抓取，选取法向力最大的接触链节作为约束的父链节。持物会随该链节的后续运动。手部碰撞使用近似的接触刚度 (2500 N/m) 和阻尼 (50 N·s/m)，限制运动学手在碰撞时产生的过大冲量。两个场景均以 `demo_0` 为标定参考，省略了机械臂运动学、软材料形变以及抓取的力学细节。仿真指标只描述对应场景。 [Bullet 柔性接触示例](https://github.com/bulletphysics/bullet3/blob/master/examples/MultiBody/MultiBodySoftContact.cpp)

发布的数据构建代码把右手网格的局部 `y` 坐标取反，生成左手点云。仿真据此镜像同一右手 URDF 的关节、渲染和碰撞几何，使同组关节角在左右两手中生成镜像姿态。 [数据构建代码](../DexCap/STEP2_build_dataset/pybullet_ik_bimanual.py)

## 回放官方示范

在 `pro1` 进入项目并激活训练环境。运行依赖来自 `requirements-train.txt`，回放本身使用 CPU：

```bash
export DEXCAP="$HOME/dexcap-repro"
cd "$DEXCAP"
source "$DEXCAP/envs/conda/etc/profile.d/conda.sh"
conda activate "$DEXCAP/envs/train"
export DATASET="$DEXCAP/data/official/e468f07000de0f8d3234a9c60a9d8208344675e0/hand_wiping_1-14_5actiongap_10000points.hdf5"

python -m simulation.rollout "$DATASET" --output "$DEXCAP/runs/wiping-replay.json"
```

服务器上的 `data/derived/wiping-demo-0-playback.hdf5` 包含从官方文件取出的 111 步动作、相机外参和手部状态。完整 HDF5 下载期间可以将上面的 `DATASET` 指向这份文件。

Wiping 场景以原始示范的 `frame_2220` 深度和官方首帧点云校准。把深度投影到 Bullet 场景使用的坐标后，在两个物体初始位置附近估计的台面高度约为 −0.035 m 到 −0.026 m，仿真用 −0.030 m 的水平面近似。盘子由 4 mm 的浅底和一圈斜面构成，盘缘约高 24 mm。海绵整体高度取 40 mm，海绵长轴相对 `+y` 方向旋转约 19°。 [原始数据重建](raw-retarget.md)

盘子需要两根不同手指同时形成相向接触后才建立抓取约束。两条接触法向的点积须小于 −0.25。盘子仍贴近桌面时，抓取约束上限为 20 N。盘子碰撞包围盒的最低点高于桌面 2 mm 后，抓取约束以当时盘心为目标位置，并把盘面法向调整到从 11 条官方示范估计出的右掌相对方向。Bullet 使用 200 N 上限执行这一朝向变化。这些近似分别描述离桌时的受力和提起后的持盘姿态。 [受力、标定与留出回放](runs.md#盘子离桌与擦拭阶段的受力)

## 运行闭环策略

从项目根执行。`CKPT` 指向 `runs/` 下实际保存的 Wiping checkpoint：

```bash
export PYTHONPATH="$DEXCAP/DexCap/STEP3_train_policy"
CUDA_VISIBLE_DEVICES=0 python -m simulation.rollout "$DATASET" \
  --checkpoint "$CKPT" --steps 12 \
  --output "$DEXCAP/runs/wiping-policy-smoke.json"
```

注意力模型用 `PYTHONPATH="$DEXCAP/worktrees/setxf/DexCap/STEP3_train_policy"`。策略每次读取最近三帧仿真观测。完成接口试跑后，去掉 `--steps` 用默认的 400 步时长评估。

`sponge_plate_contact_steps` 统计海绵与盘子的所有接触步。`wipe_surface_contact_steps` 只统计在盘面正侧施加法向力的接触步。`wipe_distance` 以米计，只累计连续正面接触时海绵相对于盘面的位移。

`wipe_passes` 沿世界竖直方向在盘面上的投影计数。每段连续正面接触确定一次投影方向，沿该方向的单程位移至少 6 cm，且竖直分量至少等于横向分量，才计一次。持续朝同一方向只计一次，随后可在转向后计下一次。同一侧的上下往返也可以计数。`sim_success` 要求两件物体都已触发抓取，并累计至少两次竖直擦拭。论文描述了方向和次数，6 cm 是本仿真根据公开轨迹采用的阈值。 [论文任务说明](https://arxiv.org/html/2403.07788v2#S5) · [实验记录](runs.md)

### 在相同摆放上比较 Wiping 策略

```bash
CUDA_VISIBLE_DEVICES=0 python -m simulation.rollout "$DATASET" \
  --checkpoint "$CKPT" --trials 20 \
  --output "$DEXCAP/runs/wiping-20-trials.json"
```

使用种子 1 到 20。每个种子分别在两件物体参考位置周围的 40 × 30 cm 区域采样，不采用盘子与海绵初始轮廓重叠的摆放。每轮重置策略，输出记录两个起始坐标、任务指标和汇总成功率。PointNet 和 SetXF 使用相同的种子与摆放。论文的试验次数和布置区域用于设计此协议，区域中心、固定相机和物理模型仍取自项目的近似场景。单次运行保持参考摆放。 [论文评估协议与任务细节](https://arxiv.org/html/2403.07788v2#S5.SS1.SSS0.Px4)

## Packaging 仿真

纸盒底部和四壁构成一个有质量的刚体。盖板通过被动铰链连接，质心在面板中心，约 0° 和 180° 处模拟打开与闭合止挡。左手可以通过接触推动纸盒和盖板。右手两根不同手指在物体两侧施力时建立抓取。持物接触纸盒后，抓取从固定姿态改为固定夹持点，物体可在纸盒接触力作用下转动。拇指到其它指尖的最短距离相比持物时的最小值增大 4 cm 时，解除抓取约束。入盒按释放后的物体在纸盒坐标下的几何边界判断，入盒和合盖同时满足时记录 `sim_success`。`box_displacement_m` 记录盒体起点与终点的水平距离，`box_center_progress_m` 记录回合中向桌面中心靠近的最大距离。

场景使用官方 `frame_2710` 的深度估计初始几何。仿真用 −3 cm 的水平面近似桌面。盒体外部平面尺寸约 29.8 × 19 cm，参考中心约为 `[0.482, 0.115] m`，初始偏航约 −62.8°。盖板初始向外展开，设定角度约 6°，铰链的近似阻力取 0.003 N·m。红银色包装主体的初始中心约为 `[0.517, -0.223] m`，仿真偏航约 64°。包装主体使用灰色材质，初始朝上的窄面有红色印刷层。 [测量与对照结果](runs.md)

```bash
PACKAGING_DATA="$DEXCAP/data/derived/packaging-demo-0-playback.hdf5"
python -m simulation.rollout "$PACKAGING_DATA" --task packaging \
  --output "$DEXCAP/runs/packaging-replay.json"
```

`PACKAGING_DATA` 可以换成版本目录下的完整 `hand_packaging_wild_1-20_5actiongap_10000points.hdf5`。为同一个 Packaging checkpoint 指定要评估的物体，种子固定为 1 至 5。

```bash
export PYTHONPATH="$DEXCAP/worktrees/setxf/DexCap/STEP3_train_policy"
CUDA_VISIBLE_DEVICES=2 python -m simulation.rollout "$PACKAGING_DATA" \
  --task packaging --checkpoint "$CKPT" --object blue-block --trials 5 \
  --output "$DEXCAP/runs/packaging-blue-block-5-trials.json"
```

每个种子在参考盒位周围的 10 × 10 cm 区域和参考物体位置周围的 40 × 30 cm 区域采样。不同物体使用同一相机和同样的摆放坐标。结果记录物体名、几何参考示范、每回合的 `box_xy` 和 `object_xy`，以及终点入盒率和完整成功率。省略 `--object` 时使用红银色包装物。 [论文的任务与初始摆放](https://arxiv.org/html/2403.07788v2#A1.SS6) · [实验记录](runs.md)

论文的完整 Packaging 协议覆盖 6 个训练中物体和 9 个未见物体。公开 Packaging HDF5 有 480 条序列，提供图像、点云和动作。论文图 10 提供物体照片，公开数据和源码缺少可直接用于仿真的物体网格、尺寸及分组标签。新增物体需要从可辨认的观测估计几何，并记录样本来源。 [论文评估协议](https://arxiv.org/html/2403.07788v2#S5.SS1.SSS0.Px4) · [固定版数据文件](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0)

从这 480 条序列每隔 25 条抽取一张首帧图像和点云，共 20 张。六种物体以所见示范的 RGB 和点云估计外形。图像裁剪和遮挡会影响尺寸估算，所有物体均使用 0.12 kg 的刚体近似。

| `--object` | 几何参考 | 仿真形状及尺寸 |
|---|---|---|
| `red-box` | `demo_0` 红银色包装主体 | 长方体 12 × 3 × 8.7 cm |
| `blue-block` | `demo_50` 蓝色积木 | 长方体 11 × 5.5 × 3.6 cm |
| `green-pear` | `demo_100` 绿色梨形物体 | 椭球 5.8 × 5 × 5.4 cm |
| `green-can` | `demo_150` 绿色罐状包装 | 圆柱，直径 8.1 cm、高 6.8 cm |
| `tape-measure` | `demo_175` 黑黄卷尺状物体 | 圆柱，直径 7.6 cm、高 3.4 cm |
| `yellow-cloth` | `demo_225`、`demo_250` 黄色织物 | 长方体 15 × 13 × 4 cm |

抽样数据保存在 `pro1` 的 `data/derived/packaging-objects-keyframes.hdf5`，首帧拼图位于 `runs/packaging-objects-keyframes.png`。这些外观描述由录制图像得到，论文的评测分组需要结合图 10 确认。 [论文物体照片](https://arxiv.org/html/2403.07788v2/exp_obj_v3.png)
