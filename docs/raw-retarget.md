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
    ├── save_data_packaging_wild_1-20_01/
    │   ├── clip_marks.json
    │   ├── map_to_robot_table_trans.txt
    │   ├── frame_0/
    │   └── ...
    └── ...
```

The builder writes generated `right_joints.txt` and `left_joints.txt` files
into each processed frame directory. Keep the extracted raw tree writable.

## Run the retargeting

The code uses relative paths for calibration YAML and LEAP-hand mesh files, so
run it from `STEP2_build_dataset`:

```bash
cd /path/to/project/DexCap/STEP2_build_dataset
```

Wiping data:

```bash
python demo_create_hdf5.py \
  /path/to/data/raw/save_wipe_1-14 \
  /path/to/data/processed/hand_wiping_1-14_5actiongap_10000points.hdf5 \
  --action-gap 5 \
  --num-points-to-sample 10000 \
  --headless
```

Packaging/in-the-wild data requires its additional table translation:

```bash
python demo_create_hdf5.py \
  /path/to/data/raw/save_packaging_wild_1-20 \
  /path/to/data/processed/hand_packaging_wild_1-20_5actiongap_10000points.hdf5 \
  --action-gap 5 \
  --num-points-to-sample 10000 \
  --in-wild-data \
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

With `DISPLAY` unset, the command completed in about five seconds and produced
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

All numeric arrays checked were finite. This proves the raw-image, depth,
two-hand IK, point-cloud, masking, and HDF5 write path works without a display.
It does not establish full-dataset runtime or reproduce every released HDF5
value: the complete Wiping and Packaging archives were intentionally not
downloaded, and the Packaging raw path was not separately sampled.
