import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from simulation.packaging import PACKAGING_OBJECTS, PACKAGING_UNSEEN_OBJECTS
from simulation.summarize_results import summarize


class SimulationSummaryTest(unittest.TestCase):
    def test_counts_complete_reports_and_rejects_unpaired_placements(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            wiping_paths = [root / "pointnet.json", root / "setxf.json"]
            for index, path in enumerate(wiping_paths, start=1):
                trials = [
                    {"seed": seed, "plate_xy": [seed, 0], "sponge_xy": [0, seed],
                     "plate_grasped": seed <= index + 2, "sponge_grasped": seed in (1, 3, 5),
                     "sponge_plate_contact_steps": 12 if seed <= 3 else 0,
                     "wipe_surface_contact_steps": 6 if seed <= index else 0,
                     "sim_success": seed == 1}
                    for seed in range(1, 21)
                ]
                path.write_text(json.dumps({
                    "task": "wiping", "steps": 400, "trials": trials,
                    "dataset": "data/official/release/wiping.hdf5",
                    "checkpoint": f"runs/{path.stem}/1/models/model_epoch_3000.pth",
                }))

            run = root / "packaging"
            unseen = root / "packaging-unseen-final"
            unseen.mkdir()
            for group, objects in (("trained", PACKAGING_OBJECTS), ("unseen", PACKAGING_UNSEEN_OBJECTS)):
                for item in objects:
                    if group == "trained":
                        path = root / f"packaging-{item}.final-sim.json"
                    else:
                        path = unseen / f"{item}.json"
                    checkpoint = f"runs/{run.name}/1/models/model_epoch_3000.pth"
                    trials = [
                        {"seed": seed, "box_xy": [seed, 0], "object_xy": [0, seed],
                         "object_in_box": seed <= 2, "lid_closed": seed in (1, 3),
                         "sim_success": seed == 1}
                        for seed in range(1, 6)
                    ]
                    path.write_text(json.dumps({
                        "task": "packaging", "steps": 400, "trials": trials,
                        "dataset": "data/official/release/packaging.hdf5",
                        "checkpoint": checkpoint if group == "unseen" else f"/remote/{checkpoint}",
                        "object_group": group, "object": item,
                    }))

            result = summarize(*wiping_paths, run)
            pointnet, setxf = (result["wiping"][name] for name in ("pointnet", "setxf"))
            self.assertEqual(pointnet["trial_count"], 20)
            self.assertEqual(pointnet["plate_grasped_count"], 3)
            self.assertEqual(pointnet["sponge_grasped_count"], 3)
            self.assertEqual(pointnet["both_grasped_count"], 2)
            self.assertEqual(pointnet["interior_contact_count"], 1)
            self.assertEqual(pointnet["interior_contact_rate"], 1 / 20)
            self.assertEqual(pointnet["success_count"], 1)
            self.assertEqual(setxf["plate_grasped_count"], 4)
            self.assertEqual(setxf["interior_contact_count"], 2)
            self.assertEqual(result["packaging"]["trained"]["trial_count"], 30)
            self.assertEqual(result["packaging"]["trained"]["placement_count"], 12)
            self.assertEqual(result["packaging"]["unseen"]["trial_count"], 45)
            self.assertEqual(result["packaging"]["unseen"]["lid_closed_count"], 18)
            self.assertEqual(result["packaging"]["unseen"]["success_count"], 9)

            path = unseen / f"{PACKAGING_UNSEEN_OBJECTS[-1]}.json"
            unpaired = json.loads(path.read_text())
            unpaired["trials"][0]["object_xy"] = [0, 0]
            path.write_text(json.dumps(unpaired))
            with self.assertRaisesRegex(ValueError, "placements"):
                summarize(*wiping_paths, run)

            unpaired["trials"][0]["object_xy"] = [0, 1]
            unpaired["checkpoint"] = unpaired["checkpoint"].replace("3000", "300")
            path.write_text(json.dumps(unpaired))
            with self.assertRaisesRegex(ValueError, "epoch 3000"):
                summarize(*wiping_paths, run)
