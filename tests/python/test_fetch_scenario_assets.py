import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "tools" / "data" / "fetch_scenario_assets.py"


class FetchScenarioAssetsCliTest(unittest.TestCase):
    def test_tracked_manifests_dry_run_all_required_shapes(self):
        result = self.run_script("--all", "--dry-run")

        self.assertEqual(result.returncode, 0, result.stderr)
        for asset_id in [
            "apophis_centered_mean_size_obj",
            "apophis_elongated_550m_obj",
            "hayabusa_itokawa_64q_dsk",
            "orex_bennu_spc_03170mm_dsk",
            "hera_didymain_06650mm_dsk",
            "naif_eros_dsk_q64",
            "pds_new_horizons_arrokoth_obj",
            "rosetta_67p_shape_high_fidelity",
            "jpl_pds_toutatis_radar_shape",
        ]:
            self.assertIn(asset_id, result.stdout)
        self.assertNotIn("gaskell_eros_shape_v1_1", result.stdout)
        self.assertNotIn("usgs_msi_albedo_2023", result.stdout)

    def test_dry_run_defaults_to_shape_assets(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            data_root = Path(tmp_dir) / "data"
            self.write_manifest(
                data_root,
                "Eros",
                [
                    self.asset("eros_shape", "shape", "scenarios/Eros/assets/shape/eros.bds"),
                    self.asset(
                        "eros_optional_shape",
                        "shape",
                        "scenarios/Eros/assets/shape/eros_high_fidelity.obj",
                        required_for_shape_runnable=False,
                    ),
                    self.asset("eros_albedo", "albedo", "scenarios/Eros/assets/albedo/eros.zip", size_gb=2.5),
                ],
            )

            result = self.run_script("--data-root", data_root, "--scenario", "Eros", "--dry-run")

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("eros_shape", result.stdout)
            self.assertNotIn("eros_optional_shape", result.stdout)
            self.assertNotIn("eros_albedo", result.stdout)

    def test_large_assets_require_allow_large(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            data_root = Path(tmp_dir) / "data"
            self.write_manifest(
                data_root,
                "Eros",
                [
                    self.asset("eros_shape", "shape", "scenarios/Eros/assets/shape/eros.bds"),
                    self.asset("eros_albedo", "albedo", "scenarios/Eros/assets/albedo/eros.zip", size_gb=2.5),
                ],
                default_shape_asset_id="eros_shape",
            )

            result = self.run_script(
                "--data-root",
                data_root,
                "--scenario",
                "Eros",
                "--include",
                "albedo",
                "--dry-run",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("eros_albedo", result.stderr)
            self.assertIn("--allow-large", result.stderr)

    def test_missing_download_url_reports_manifest_asset_and_expected_path(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            data_root = Path(tmp_dir) / "data"
            manifest_path = self.write_manifest(
                data_root,
                "Toutatis",
                [
                    self.asset(
                        "toutatis_shape",
                        "shape",
                        "scenarios/Toutatis/assets/shape/toutatis.obj",
                        download_url="",
                    )
                ],
                default_shape_asset_id="toutatis_shape",
            )
            expected_local_path = data_root / "scenarios" / "Toutatis" / "assets" / "shape" / "toutatis.obj"

            result = self.run_script("--data-root", data_root, "--scenario", "Toutatis")

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("toutatis_shape", result.stderr)
            self.assertIn("download_url", result.stderr)
            self.assertIn(str(manifest_path), result.stderr)
            self.assertIn(str(expected_local_path), result.stderr)
            self.assertIn("--verify-only", result.stderr)

    def test_manifest_local_paths_must_stay_inside_data_root(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            data_root = Path(tmp_dir) / "data"
            self.write_manifest(
                data_root,
                "BadBody",
                [self.asset("bad_shape", "shape", "../outside.obj")],
                default_shape_asset_id="bad_shape",
            )

            result = self.run_script("--data-root", data_root, "--scenario", "BadBody", "--dry-run")

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("outside data root", result.stderr)
            self.assertIn("bad_shape", result.stderr)

    def run_script(self, *args):
        return subprocess.run(
            [sys.executable, str(SCRIPT_PATH), *map(str, args)],
            check=False,
            capture_output=True,
            text=True,
        )

    @staticmethod
    def asset(
        asset_id,
        asset_type,
        local_path,
        download_url="https://example.test/asset.bin",
        size_gb=0.1,
        required_for_shape_runnable=None,
    ):
        if required_for_shape_runnable is None:
            required_for_shape_runnable = asset_type == "shape"
        return {
            "asset_id": asset_id,
            "asset_type": asset_type,
            "local_path": local_path,
            "source_url": "https://example.test/source",
            "download_url": download_url,
            "sha256": "",
            "size_gb": size_gb,
            "fidelity": "test",
            "required_for_shape_runnable": required_for_shape_runnable,
        }

    @staticmethod
    def write_manifest(data_root, scenario_name, assets, default_shape_asset_id=None):
        manifest_dir = data_root / "scenarios" / scenario_name
        manifest_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = manifest_dir / "manifest.json"
        manifest = {
            "schema_version": 1,
            "scenario_name": scenario_name,
            "canonical_name": scenario_name,
            "aliases": [scenario_name],
            "confidence": "test",
            "tags": ["shape_runnable"],
            "default_shape_asset_id": default_shape_asset_id or assets[0]["asset_id"],
            "assets": assets,
        }
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        return manifest_path


if __name__ == "__main__":
    unittest.main()
