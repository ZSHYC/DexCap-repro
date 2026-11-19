# Raw data to HDF5 on a headless server

DexCap's Step 2 converts the released raw camera and glove recordings into the
HDF5 format consumed by policy training. This step includes PyBullet inverse
kinematics for both LEAP hands. It is data retargeting, not a simulator rollout.

The released script opened three GUI paths unconditionally: the PyBullet GUI,
an Open3D point-cloud window, and an OpenCV image window. The builder now has a
`--headless` option which changes only those display paths. Omitting the option
keeps the original GUI behavior.

## Environment note

Run the builder inside the project's conda environment. The published
dependencies do not constrain `trimesh`; current `trimesh 5.x` removed the
`as_open3d` property used by DexCap's `yourdfpy` path. Pin the compatible 3.x
version in that conda environment:

```bash
conda activate dexcap
python -m pip install "trimesh==3.23.5"
```

For a server-only environment, `opencv-python-headless` and `open3d-cpu` are
sufficient. The reproduction check used Open3D CPU 0.17.0, PyBullet 3.2.6,
yourdfpy 0.0.56, and Python 3.10.

## Expected raw directory layout

Do not point the command at one recording directory. Point it at the parent
which contains all recording directories:

```text
data/raw/
├── save_wipe_1-14/
│   ├── save_data_wipe_1-14_01/
│   │   ├── clip_marks.json
│   │   ├── calib_offset.txt
│   │   ├── calib_offset_left.txt
│   │   ├── frame_0/
│   │   └── frame_2220/
│   └── ...
└── save_packaging_wild_1-20/
    ├── save_data_packaging_01/
    │   ├── clip_marks.json
    │   ├── map_to_robot_table_trans.txt
    │   ├── frame_0/
    │   └── ...
    └── ...
```

The default live-IK path writes generated `right_joints.txt` and
`left_joints.txt` into the processed frame directories. Run that path on a
writable copy of the raw recording. When every selected frame has archived
LEAP joints, `--use-recorded-joints` reads those values and preserves the
source files.

## Run the retargeting

The code uses relative paths for calibration YAML and LEAP-hand mesh files, so
run it from `STEP2_build_dataset`:

```bash
export DEXCAP="$HOME/dexcap-repro"
mkdir -p "$DEXCAP/data/derived"
cd "$DEXCAP/DexCap/STEP2_build_dataset"
```

Wiping data:

```bash
python demo_create_hdf5.py \
  "$DEXCAP/data/raw/save_wipe_1-14" \
  "$DEXCAP/data/derived/hand_wiping_1-14_5actiongap_10000points.hdf5" \
  --action-gap 5 \
  --num-points-to-sample 10000 \
  --headless
```

Packaging/in-the-wild data also requires its table translation. With the
untouched official joint files present, use the following command.

```bash
python demo_create_hdf5.py \
  "$DEXCAP/data/raw/save_packaging_wild_1-20" \
  "$DEXCAP/data/derived/hand_packaging_wild_1-20_5actiongap_10000points.hdf5" \
  --action-gap 5 \
  --num-points-to-sample 10000 \
  --in-wild-data \
  --use-recorded-joints \
  --headless
```

Do not pass `--in-wild-data` for wiping. To use the original interactive
visualizers on a machine with a display, omit `--headless`.

## What was verified

A fixture was extracted from the published Wiping ZIP using HTTP ranges rather
than downloading the 22.7 GB archive. It used the official calibration files,
`frame_0`, and the first real frame (`frame_2220`) of the official first clip
(`frame_2220` through `frame_2770`). The fixture clip mark was narrowed to that
single frame so the full reconstruction could be exercised cheaply.

For this one-frame fixture, `--action-gap 1` produces a single sequence. With
`DISPLAY` unset, the command completed in about five seconds and produced
one HDF5 demo with:

```text
actions                         (1, 46)
obs/agentview_image             (1, 84, 84, 3)
obs/pointcloud                  (1, 1000, 6)
obs/robot0_eef_pos              (1, 6)
obs/robot0_eef_quat             (1, 8)
obs/robot0_eef_hand             (1, 32)
states                          (1, 16)
glove_states                    (1, 63)
dones                           [1]
data.attrs.total                1
```

All numeric arrays checked were finite. The fixture exercised image and depth
reading, two-hand IK, point-cloud construction, masking, and HDF5 writing
without a display.

### Packaging first ten raw frames

The first official clip spans `frame_2710` through `frame_3360`. The fixture
includes the calibration and table alignment files, `frame_0`, and the ten
consecutive frames from `frame_2710` through `frame_2719`. The original
annotation is preserved in `clip_marks.json.official`. The working
`clip_marks.json` selects the available ten frames. The archive was read by
HTTP byte range from the pinned data revision. [Official data revision](https://huggingface.co/datasets/chenwangj/DexCap-Data/tree/e468f07000de0f8d3234a9c60a9d8208344675e0)

The build ran in the existing local Python 3.10 Conda environment with no
display. The raw fixture, generated HDF5, and log were copied to `pro1`. With
the build dependencies enabled in Conda, the equivalent command on `pro1`
uses its copy of the fixture.

```bash
export DEXCAP="$HOME/dexcap-repro"
cd "$DEXCAP/DexCap/STEP2_build_dataset"
python demo_create_hdf5.py \
  "$DEXCAP/data/raw/packaging-rebuild-fixture/save_packaging_wild_1-20" \
  "$DEXCAP/data/derived/packaging-rebuild-fixture.hdf5" \
  --action-gap 5 --num-points-to-sample 10000 --in-wild-data --headless
```

The output contains `demo_0` through `demo_4`, with two frames per sequence,
46 values per action, and `10000 × 6` points per frame. All checked numeric
arrays were finite. Compared with the corresponding first two frames in the
released demos, the images and camera states are identical. Wrist positions,
quaternions, and first-action wrist targets match within `1.11e-16`. The
glove observations match within `2.22e-16`. The median nearest-neighbor XYZ
distance from rebuilt to released points ranges from 1.70 to 1.72 mm across
the five sequences.

With live IK, the largest finger-joint difference per sequence ranges from
1.82 to 2.74 rad after undoing the stored half-scale. Repeating this build
reproduced its own hand states and actions exactly. The build log is at
`runs/packaging-rebuild-fixture.log`.

### Rebuild with archived hand joints

The official Packaging ZIP contains 16 joint angles per hand alongside the
camera and glove inputs. In the eleven inspected frames, the archived joint
files cover both hands. In the ten clip frames, the builder's
`(joints - pi) * 0.5` encoding matches all 32 released hand values exactly.
The generation history of those archived angles is unavailable. The
published build computes fresh IK with PyBullet, which the workspace
requirements pin to version 3.2.6. [Official build requirements](https://github.com/j96w/DexCap/blob/4b0bed0966c87368f3cde4476aadb7585c3b94b5/install/env_ws_requirements.txt)

The `--use-recorded-joints` path uses those angles for the HDF5 targets and
reverses the left-hand robot sign convention for the forward kinematics
mesh. The fixture on `pro1` retains the original archived joint files.

```bash
cd "$DEXCAP/DexCap/STEP2_build_dataset"
python demo_create_hdf5.py \
  "$DEXCAP/data/raw/packaging-recorded-joints-fixture/save_packaging_wild_1-20" \
  "$DEXCAP/data/derived/packaging-recorded-rebuild.hdf5" \
  --action-gap 5 --num-points-to-sample 10000 --in-wild-data \
  --use-recorded-joints --headless
```

The five rebuilt demo sequences match the released 32-component hand
observations for both frames and the hand component of each first action
exactly. The largest difference across all 46 components of each first
action is `1.11e-16`. The last frame of each fixture sequence uses the
builder's short-clip padding. Across the ten frames, the median of the ten
nearest-neighbor XYZ medians was 1.71 mm. The clip had no paired zero-RGB
hand points for a direct comparison with the release clouds. The
forward-kinematics round trip was checked separately by writing the solved
joints for a single raw frame, then loading them with the new option.
With the same sampling seed, all 80,000 points per hand were identical.
The build ran in the local Conda environment, and the HDF5 and
`runs/packaging-recorded-rebuild.log` were copied to `pro1`. Policy training
uses the released HDF5.

### Visible hand points at Packaging frame 2810

The released `demo_0` frame at index 20 corresponds to raw `frame_2810`
and contains 3280 zero-RGB hand points. A second fixture preserves the
same calibration and archived joints. It builds one frame with
`--action-gap 1 --num-points-to-sample 10000 --in-wild-data
--use-recorded-joints --headless`.

The rebuilt camera and all 32 hand values exactly match the released
frame. Maximum wrist position and quaternion component differences are
`5.55e-17` and `8.33e-17`. The rebuild contains 3289 zero-RGB points,
with a hand-cloud centroid difference of 1.52 mm. For the hand points,
the median nearest-neighbor distance is 4.26 mm from rebuilt to release
and 4.27 mm in the opposite direction. The respective 95th percentiles
are 8.35 and 8.28 mm. Two builds of this fixture with separate random
surface samples produced directional medians of 4.25 and 4.30 mm.

The raw fixture is on `pro1` at
`data/raw/packaging-recorded-joints-visible-hand-fixture/`. The HDF5 file
is `data/derived/packaging-recorded-joints-visible-hand-frame2810.hdf5`.
