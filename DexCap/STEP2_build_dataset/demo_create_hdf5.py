import argparse
import os

from dataset_utils import extract_dataset_folder_last_two_digits, process_hdf5


def main():
    parser = argparse.ArgumentParser(description="Build a DexCap HDF5 dataset from the released raw data.")
    parser.add_argument("dataset_base_dir", help="Directory containing the raw recording directories")
    parser.add_argument("output_hdf5_file", help="HDF5 file to create")
    parser.add_argument("--action-gap", type=int, default=5)
    parser.add_argument("--num-points-to-sample", type=int, default=10000)
    parser.add_argument("--in-wild-data", action="store_true", help="Enable packaging/in-the-wild translation data")
    parser.add_argument("--headless", action="store_true", help="Disable Open3D, OpenCV, and PyBullet GUI windows")
    parser.add_argument(
        "--use-recorded-joints", action="store_true",
        help="Read the archived LEAP joints from each frame and preserve those files",
    )
    args = parser.parse_args()

    sub_dirs = [
        os.path.join(args.dataset_base_dir, directory)
        for directory in os.listdir(args.dataset_base_dir)
        if os.path.isdir(os.path.join(args.dataset_base_dir, directory))
    ]
    dataset_folders = sorted(
        sub_dirs,
        key=lambda path: extract_dataset_folder_last_two_digits(os.path.basename(path)),
    )

    process_hdf5(
        args.output_hdf5_file,
        dataset_folders,
        args.action_gap,
        args.num_points_to_sample,
        in_wild_data=args.in_wild_data,
        headless=args.headless,
        use_recorded_joints=args.use_recorded_joints,
    )


if __name__ == "__main__":
    main()
