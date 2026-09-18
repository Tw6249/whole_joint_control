"""Batch regressions using fake processes and temporary logs, never hardware."""
import argparse
import contextlib
import csv
import datetime as dt
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.hip_knee import batch_common as common
from experiments.hip_knee import run_real_p1_batch as p1
from experiments.hip_knee import run_real_p2_batch as p2


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "config.yaml"
        self.config.write_text("log_path: test.csv\n", encoding="utf-8")
        self.spec = SimpleNamespace(method="pd", config=self.config)
        self.args = argparse.Namespace(execute=True, no_sudo=True, direct=Path("fake-direct"), duration=10.0, pause=0,
                                       target="knee", disturbance_method="manual_push",
                                       out_dir=self.root / "out")

    def log(self, **changes):
        path = self.root / "data" / "run" / "test.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        row = dict(repeat_id="r01", condition_id="condition", disturbance_target="knee",
                   disturbance_method="manual_push", dt="0.01")
        row.update(changes)
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=["cycle", *row])
            writer.writeheader()
            writer.writerow(dict(cycle=0, **row))
            writer.writerow(dict(cycle=1000, **row))
        return path

    def test_completed_log_requires_matching_metadata_and_duration(self):
        expected = dict(condition_id="condition", disturbance_target="knee",
                        disturbance_method="manual_push")
        path = self.log()
        def find():
            return common.recent_completed_log(self.root, self.spec, 1, dt.datetime.now(), 10, expected)
        self.assertEqual(find(), path)
        for changes in [dict(repeat_id="r02"), dict(condition_id="other"),
                        dict(disturbance_target="hip"), dict(disturbance_method="other"),
                        dict(dt="0.001")]:
            with self.subTest(changes=changes):
                self.log(**changes)
                self.assertIsNone(find())

    def test_dry_run_has_no_process_or_manifest(self):
        self.args.execute = False
        for module in (p1, p2):
            with patch.object(common.subprocess, "run") as run, \
                 patch.object(module, "open_manifest") as manifest, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertIsNone(module.run_batch(self.args, [(1, module.METHODS["pd"])]))
                run.assert_not_called()
                manifest.assert_not_called()

    def test_failed_run_stops_and_flushes_manifest(self):
        self.check_execution(returncode=1, completed=False, expected_calls=1, raises=True)
        self.check_execution(returncode=-6, completed=False, expected_calls=1, raises=True)

    def test_complete_abort_retains_existing_acceptance_rule(self):
        self.check_execution(returncode=-6, completed=True, expected_calls=2, raises=False)

    def test_success_retains_p2_manifest_metadata(self):
        self.check_execution(returncode=0, completed=True, expected_calls=2, raises=False)

    def check_execution(self, returncode, completed, expected_calls, raises):
        for module in (p1, p2):
            with self.subTest(phase=module.__name__, returncode=returncode):
                self.args.out_dir = self.root / module.__name__ / f"{returncode}_{completed}"
                spec = module.METHODS["pd"]
                log = module.REPO_ROOT / "data" / "fake.csv" if completed else None
                plan = [(1, spec), (2, spec)]
                with patch.object(common.subprocess, "run", return_value=SimpleNamespace(returncode=returncode)) as run, \
                     patch.object(module, "recent_completed_log", return_value=log), \
                     patch.object(module, "command_for", return_value=["fake-direct"]), \
                     contextlib.redirect_stdout(io.StringIO()):
                    if raises:
                        with self.assertRaises(SystemExit):
                            module.run_batch(self.args, plan)
                    else:
                        module.run_batch(self.args, plan)
                    self.assertEqual(run.call_count, expected_calls)
                manifest, = self.args.out_dir.glob("*.csv")
                with manifest.open(newline="", encoding="utf-8") as fh:
                    rows = list(csv.DictReader(fh))
                self.assertEqual(len(rows), expected_calls)
                expected_status = "failed" if raises else ("ok" if returncode == 0 else "accepted_after_complete_abort")
                self.assertEqual(rows[0]["status"], expected_status)
                if module is p2:
                    self.assertEqual(rows[0]["target"], "knee")
                    self.assertEqual(rows[0]["disturbance_method"], "manual_push")
                    self.assertEqual(rows[0]["condition"], "P2-K_PD_anti_phase_knee_disturbance")
                else:
                    self.assertNotIn("target", rows[0])

    def test_confirmation_still_required(self):
        self.args.yes = False
        for module in (p1, p2):
            with patch("builtins.input", return_value="wrong"), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit):
                    module.confirm_or_exit(self.args, [])


if __name__ == "__main__":
    unittest.main()
