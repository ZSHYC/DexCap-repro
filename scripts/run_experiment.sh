#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s {wiping-pointnet|wiping-setxf|packaging-setxf} {smoke|full} [dataset.hdf5]\n' "$0" >&2
    exit 2
}

(( $# >= 2 && $# <= 3 )) || usage
experiment=$1
stage=$2
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
tree=$root
case "$experiment" in
    wiping-pointnet) task=wiping; gpu=0 ;;
    wiping-setxf) task=wiping; gpu=1; tree="$root/worktrees/setxf" ;;
    packaging-setxf) task=packaging; gpu=2; tree="$root/worktrees/setxf" ;;
    *) usage ;;
esac
[[ "$stage" == smoke || "$stage" == full ]] || usage

revision=e468f07000de0f8d3234a9c60a9d8208344675e0
if [[ "$task" == wiping ]]; then
    filename=hand_wiping_1-14_5actiongap_10000points.hdf5
else
    filename=hand_packaging_wild_1-20_5actiongap_10000points.hdf5
fi
dataset=$(realpath -e "${3:-$root/data/official/$revision/$filename}")
python="$root/envs/train/bin/python"
runs="$root/runs"
mkdir -p "$runs"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-$gpu}"
export PYTHONPATH="$tree/DexCap/STEP3_train_policy"

train() {
    local config_name=$1 epoch=$2
    name="$experiment-seed1-$config_name-$(date -u +%Y%m%dT%H%M%SZ)"
    local log="$runs/$name.console.log"
    cd "$PYTHONPATH/robomimic"
    printf 'Training commit %s\nProject commit %s\nDataset %s\nGPU %s\n' \
        "$(git -C "$tree" rev-parse HEAD)" "$(git -C "$root" rev-parse HEAD)" \
        "$dataset" "$CUDA_VISIBLE_DEVICES" | tee "$log"
    "$python" -u scripts/train.py --config "$root/configs/pointcloud-$config_name.json" \
        --dataset "$dataset" --output "$runs" --name "$name" 2>&1 | tee -a "$log"
    if ! grep -qF 'finished run successfully!' "$log"; then
        printf 'Training did not report completion. See %s\n' "$log" >&2
        exit 1
    fi
    checkpoint=$(find "$runs/$name" -type f -name "model_epoch_$epoch.pth" -print -quit)
    if [[ ! -s "$checkpoint" ]]; then
        printf 'Epoch %s checkpoint is missing from %s\n' "$epoch" "$runs/$name" >&2
        exit 1
    fi
    cd "$root"
}

train smoke 2
"$python" "$root/scripts/checkpoint_inference.py" "$checkpoint" "$dataset" \
    | tee "$runs/$name.inference.log"
[[ "$stage" == full ]] || exit 0

train full 3000
if [[ "$task" == wiping ]]; then
    "$python" -m simulation.rollout "$dataset" --task "$task" --checkpoint "$checkpoint" \
        --trials 20 --output "$runs/$name.final-sim.json"
else
    objects=$("$python" -c 'from simulation.packaging import PACKAGING_OBJECTS; print(*PACKAGING_OBJECTS)')
    for object in $objects; do
        "$python" -m simulation.rollout "$dataset" --task "$task" --checkpoint "$checkpoint" \
            --object "$object" --trials 5 --output "$runs/$name-$object.final-sim.json"
    done
fi
