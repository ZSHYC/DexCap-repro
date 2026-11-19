# Plate Wiping 仿真

项目基于公开 Wiping 示范建立了无界面的 PyBullet 场景。作者公开了手部重定向和训练代码，并在上游说明仿真回放需要另外构建场景数字孪生。这里使用官方 LEAP 手 URDF，根据 `demo_0` 的第一帧点云估计桌面高度、盘子和海绵的尺寸及位置。 [作者关于仿真的说明](https://github.com/j96w/DexCap/issues/6#issuecomment-2209325098)

策略的两只手使用各自的腕部坐标。仿真用示范的 `frame_0_eef_quat` 和发布的标定配置将目标映射到同一个 Bullet 世界，执行后将模拟器的手部位姿与关节状态变回策略坐标。相机使用 HDF5 中的第一帧外参、发布的 L515 内参和 Bullet 深度缓存生成 `10000 × 6` XYZRGB 点云。控制频率为 20 Hz，物理步进为 240 Hz。

两只 LEAP 手的掌部按目标做运动学移动，盘子和海绵是动力学刚体。手碰到对应物体后，模拟器用固定约束近似抓取。当前场景只校准了 `demo_0` 的固定摆放，省略了机械臂运动学和海绵形变。仿真指标只描述此场景。

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

## 运行闭环策略

从项目根执行。`CKPT` 指向 `runs/` 下实际保存的 Wiping checkpoint：

```bash
export PYTHONPATH="$DEXCAP/DexCap/STEP3_train_policy"
CUDA_VISIBLE_DEVICES=0 python -m simulation.rollout "$DATASET" \
  --checkpoint "$CKPT" --steps 12 \
  --output "$DEXCAP/runs/wiping-policy-smoke.json"
```

注意力模型用 `PYTHONPATH="$DEXCAP/worktrees/setxf/DexCap/STEP3_train_policy"`。策略每次读取最近三帧仿真观测。完成接口试跑后，去掉 `--steps` 用默认的 400 步时长评估。

`sponge_plate_contact_steps` 是发生海绵与盘子接触的控制步数。`wipe_distance` 以米计，只累计连续接触时海绵相对于盘面的位移。`wipe_passes` 要求在同一段连续接触内，沿盘面横跨两个端点。`sim_success` 要求两件物体都已触发抓取，并累计至少两次跨盘擦拭。 [已执行的实验](runs.md)
