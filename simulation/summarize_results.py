import argparse
import json
from pathlib import Path

from simulation.packaging_objects import PACKAGING_OBJECTS, PACKAGING_UNSEEN_OBJECTS


def read_report(path, task, trial_count):
    report = json.loads(path.read_text())
    if report["task"] != task or report["steps"] != 400:
        raise ValueError(f"{path} must contain 400-step {task} trials")
    if Path(report["checkpoint"]).name != "model_epoch_3000.pth":
        raise ValueError(f"{path} must use the epoch 3000 checkpoint")
    if [trial["seed"] for trial in report["trials"]] != list(range(1, trial_count + 1)):
        raise ValueError(f"{path} must contain seeds 1 through {trial_count}")
    return report


def pairing(report, first, second):
    dataset = Path(report["dataset"]).parts[-2:]
    positions = [(trial[first], trial[second]) for trial in report["trials"]]
    return dataset, positions


def tally(trials, metrics):
    result = {"trial_count": len(trials)}
    for name, flag in metrics.items():
        count = sum(
            trial[flag] and not (trial.get("physics_warnings") or trial.get("invalid_hand_motion"))
            for trial in trials
        )
        result[f"{name}_count"] = count
        result[f"{name}_rate"] = count / len(trials)
    return result


def engine(report):
    return report.get("engine", "pybullet"), report.get("engine_version")


def summarize(pointnet_path, setxf_path, packaging_run):
    pointnet = read_report(pointnet_path, "wiping", 20)
    setxf = read_report(setxf_path, "wiping", 20)
    if engine(pointnet) != engine(setxf):
        raise ValueError("The Wiping reports must use the same physics engine and version")
    if pairing(pointnet, "plate_xy", "sponge_xy") != pairing(setxf, "plate_xy", "sponge_xy"):
        raise ValueError(f"{setxf_path} and {pointnet_path} must use the same dataset and placements")
    wiping_metrics = {
        "plate_grasped": "plate_grasped",
        "sponge_grasped": "sponge_grasped",
        "both_grasped": "both_grasped",
        "interior_contact": "interior_contact",
        "success": "sim_success",
    }
    wiping = {}
    for name, report in (("pointnet", pointnet), ("setxf", setxf)):
        trials = [
            {
                **trial,
                "both_grasped": trial["plate_grasped"] and trial["sponge_grasped"],
                "interior_contact": trial["wipe_surface_contact_steps"] > 0,
            }
            for trial in report["trials"]
        ]
        wiping[name] = {"checkpoint": report["checkpoint"], **tally(trials, wiping_metrics)}

    packaging = {}
    reference_placements = None
    reference_checkpoint = None
    metrics = {"placement": "object_in_box", "lid_closed": "lid_closed", "success": "sim_success"}
    for group, objects in (("trained", PACKAGING_OBJECTS), ("unseen", PACKAGING_UNSEEN_OBJECTS)):
        object_scores = {}
        group_trials = []
        for name in objects:
            if group == "trained":
                path = Path(f"{packaging_run}-{name}.final-sim.json")
            else:
                path = Path(f"{packaging_run}-unseen-final") / f"{name}.json"
            report = read_report(path, "packaging", 5)
            if engine(report) != engine(pointnet):
                raise ValueError(f"{path} must use the same physics engine and version as the Wiping reports")
            if (report["object_group"], report["object"]) != (group, name):
                raise ValueError(f"{path} must contain trials for {group} object {name}")

            # Training writes an absolute checkpoint path and the unseen job writes a relative path.
            checkpoint = Path(report["checkpoint"]).parts[-4:]
            placements = pairing(report, "box_xy", "object_xy")
            if reference_checkpoint is None:
                reference_checkpoint = checkpoint
                reference_placements = placements
                packaging["checkpoint"] = report["checkpoint"]
            if checkpoint[0] != packaging_run.name or checkpoint != reference_checkpoint:
                raise ValueError(f"{path} must use the same Packaging checkpoint from {packaging_run}")
            if placements != reference_placements:
                raise ValueError(f"{path} must use the same Packaging dataset and placements")

            object_scores[name] = tally(report["trials"], metrics)
            group_trials.extend(report["trials"])
        packaging[group] = {**tally(group_trials, metrics), "objects": object_scores}

    return {"epoch": 3000, "steps_per_trial": 400, "engine": engine(pointnet)[0],
            "engine_version": engine(pointnet)[1], "wiping": wiping, "packaging": packaging}


def main():
    parser = argparse.ArgumentParser(description="Summarize the formal DexCap simulation evaluations")
    parser.add_argument("wiping_pointnet", type=Path, help="PointNet rollout JSON")
    parser.add_argument("wiping_setxf", type=Path, help="SetXF rollout JSON")
    parser.add_argument("packaging_run", type=Path, help="Packaging run directory or prefix under runs/")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        summary = summarize(args.wiping_pointnet, args.wiping_setxf, args.packaging_run)
        args.output.write_text(json.dumps(summary, indent=2) + "\n")
    except (OSError, KeyError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
