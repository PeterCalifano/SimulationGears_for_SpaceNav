"""Exercise the scenario-asset CLI with tracked, local fixtures.

Each test copies fixture manifests into a temporary data root and binds their
download URLs to the small payloads checked in beside them. Downloads and
verification use that temporary destination. Production scenario manifests,
asset installations and remote services are not test inputs.

Manifest editing retains JSON dictionaries at the fixture boundary; the CLI
owns schema validation.

Example:
    python3 -m pytest -q tests/python/test_fetch_scenario_assets.py
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "tools" / "data" / "fetch_scenario_assets.py"
FIXTURE_ROOT = Path(__file__).parent / "fixtures" / "scenario_assets"
SOURCE_ROOT = FIXTURE_ROOT / "sources"
PRIMARY_SCENARIO = "FixtureBodyA"
PRIMARY_ASSET = "fixture_required_shape"


class FetchScenarioAssetsCliTest(unittest.TestCase):
    """Test CLI selection and file handling against self-contained fixtures.

    Attributes:
        data_root: Temporary copy of the tracked fixture manifests and all
            destinations created by the test.
    """

    data_root: Path

    def setUp(self) -> None:
        """Copy fixture manifests and bind downloads to tracked local files."""
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.data_root = Path(directory.name) / "data"
        shutil.copytree(FIXTURE_ROOT / "manifests", self.data_root)

        # Bind portable fixture URLs without changing the checked-in manifests.
        for manifest_path in self.data_root.glob("scenarios/*/manifest.json"):
            manifest: dict[str, Any] = json.loads(manifest_path.read_text(encoding="utf-8"))
            for asset in manifest["assets"]:
                download_url = asset["download_url"]
                self.assertTrue(download_url.startswith("fixture://"))
                source_path = SOURCE_ROOT / download_url.removeprefix("fixture://")
                self.assertTrue(source_path.is_file(), f"Missing tracked fixture: {source_path}")
                asset["download_url"] = source_path.resolve().as_uri()
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    def test_dry_run_all_required_shapes(self) -> None:
        """Plan every required fixture shape while destinations are absent."""
        result = self._run_script("--all", "--dry-run")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(PRIMARY_ASSET, result.stdout)
        self.assertIn("fixture_secondary_shape", result.stdout)
        self.assertNotIn("fixture_optional_shape", result.stdout)
        self.assertNotIn("fixture_large_albedo", result.stdout)
        self.assertIn("would_download", result.stdout)
        self.assertFalse(self._destination_path().exists())

    def test_dry_run_defaults_to_required_shape_assets(self) -> None:
        """Select required shapes from a manifest containing optional assets."""
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--dry-run")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(PRIMARY_ASSET, result.stdout)
        self.assertNotIn("fixture_optional_shape", result.stdout)
        self.assertNotIn("fixture_large_albedo", result.stdout)

    def test_large_assets_require_allow_large(self) -> None:
        """Reject a declared large asset before fetching its tiny payload."""
        result = self._run_script(
            "--scenario", PRIMARY_SCENARIO, "--include", "albedo", "--dry-run"
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("fixture_large_albedo", result.stderr)
        self.assertIn("--allow-large", result.stderr)

    def test_allow_large_plans_local_fixture(self) -> None:
        """Honor explicit permission for the declared-size planning case."""
        result = self._run_script(
            "--scenario", PRIMARY_SCENARIO, "--include", "albedo", "--allow-large", "--dry-run"
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("fixture_large_albedo", result.stdout)

    def test_missing_download_url_reports_asset_and_expected_path(self) -> None:
        """Report the owning manifest and destination for an unavailable source."""
        manifest_path = self._update_asset(PRIMARY_ASSET, download_url="")
        result = self._run_script("--scenario", PRIMARY_SCENARIO)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn(PRIMARY_ASSET, result.stderr)
        self.assertIn("download_url", result.stderr)
        self.assertIn(str(manifest_path), result.stderr)
        self.assertIn(str(self._destination_path()), result.stderr)
        self.assertIn("--verify-only", result.stderr)

    def test_manifest_local_paths_must_stay_inside_asset_root(self) -> None:
        """Reject a fixture destination that escapes its temporary asset root."""
        self._update_asset(PRIMARY_ASSET, local_path="../outside.obj")
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--dry-run")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside asset root", result.stderr)
        self.assertIn(PRIMARY_ASSET, result.stderr)

    def test_unsupported_content_format_reports_manifest_and_asset(self) -> None:
        """Reject unknown format metadata before downloading a valid fixture."""
        manifest_path = self._update_asset(
            PRIMARY_ASSET, content_format="unsupported_fixture_format"
        )
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--dry-run")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported content_format", result.stderr)
        self.assertIn(PRIMARY_ASSET, result.stderr)
        self.assertIn(str(manifest_path), result.stderr)
        self.assertFalse(self._destination_path().exists())

    def test_wavefront_obj_content_format_rejects_non_obj_download(self) -> None:
        """Reject the tracked invalid payload without installing a destination."""
        manifest_path = self._update_asset(
            PRIMARY_ASSET,
            download_url=(SOURCE_ROOT / "not_obj.tab").resolve().as_uri(),
            sha256="",
        )
        result = self._run_script("--scenario", PRIMARY_SCENARIO)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn(PRIMARY_ASSET, result.stderr)
        self.assertIn("wavefront_obj", result.stderr)
        self.assertIn(str(manifest_path), result.stderr)
        self.assertFalse(self._destination_path().exists())

    def test_tab_named_wavefront_obj_download_is_installed_as_obj(self) -> None:
        """Install the tracked TAB-named OBJ with its pinned content checksum."""
        result = self._run_script("--scenario", PRIMARY_SCENARIO)
        source_path = SOURCE_ROOT / "triangle.tab"

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self._destination_path().read_bytes(), source_path.read_bytes())
        manifest_path = self.data_root / "scenarios" / PRIMARY_SCENARIO / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        asset = next(asset for asset in manifest["assets"] if asset["asset_id"] == PRIMARY_ASSET)
        self.assertEqual(hashlib.sha256(self._destination_path().read_bytes()).hexdigest(),
                         asset["sha256"])

    def test_verify_only_checks_pinned_fixture_checksum(self) -> None:
        """Verify an installed copy of the tracked OBJ without a download URL."""
        destination_path = self._destination_path()
        destination_path.parent.mkdir(parents=True)
        shutil.copyfile(SOURCE_ROOT / "triangle.tab", destination_path)
        self._update_asset(PRIMARY_ASSET, download_url="")

        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--verify-only")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"OK {PRIMARY_SCENARIO} {PRIMARY_ASSET}", result.stdout)

    def test_verify_only_rejects_modified_fixture(self) -> None:
        """Reject a changed temporary destination against the pinned checksum."""
        destination_path = self._destination_path()
        destination_path.parent.mkdir(parents=True)
        destination_path.write_bytes((SOURCE_ROOT / "triangle.tab").read_bytes() + b"# changed\n")

        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--verify-only")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Checksum mismatch", result.stderr)
        self.assertIn(PRIMARY_ASSET, result.stderr)

    def test_verify_only_reports_missing_fixture(self) -> None:
        """Report a missing temporary destination without relying on local data."""
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--verify-only")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn(PRIMARY_ASSET, result.stderr)
        self.assertIn(str(self._destination_path()), result.stderr)
        self.assertIn("--asset-id", result.stderr)

    def test_explicit_asset_selects_optional_shape(self) -> None:
        """Select an optional shape explicitly from a fully populated manifest."""
        result = self._run_script(
            "--scenario", PRIMARY_SCENARIO, "--asset-id", "fixture_optional_shape", "--dry-run"
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("fixture_optional_shape", result.stdout)
        self.assertNotIn(PRIMARY_ASSET, result.stdout)

    def test_obj_content_format_installs_and_writes_sidecar_receipt(self) -> None:
        """Accept the schema-v2 ``obj`` format and record a receipt beside a non-body path."""
        self._update_asset(PRIMARY_ASSET, content_format="obj")
        result = self._run_script("--scenario", PRIMARY_SCENARIO)

        self.assertEqual(result.returncode, 0, result.stderr)
        receipt_path = self._destination_path().with_name("triangle.obj.acquisition.json")
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        self.assertEqual(receipt["asset_id"], PRIMARY_ASSET)
        self.assertEqual(receipt["local_path"], "assets/fixture_a/shape/triangle.obj")
        self.assertEqual(receipt["sha256"], hashlib.sha256(self._destination_path().read_bytes()).hexdigest())
        self.assertEqual(receipt["bytes"], self._destination_path().stat().st_size)
        current_umask = os.umask(0)
        os.umask(current_umask)
        self.assertEqual(self._destination_path().stat().st_mode & 0o777, 0o666 & ~current_umask)

    def test_dsk_content_format_checks_das_identifier_and_body_receipt(self) -> None:
        """Install a DAS/DSK payload under assets/bodies and reject a non-DSK one."""
        dsk_source = self.data_root.parent / "fixture.bds"
        dsk_source.write_bytes(b"DAS/DSK " + bytes(1016))
        self._update_asset(
            PRIMARY_ASSET,
            content_format="dsk",
            local_path="assets/bodies/fixture_a/shape/fixture.bds",
            download_url=dsk_source.as_uri(),
            sha256="",
        )
        result = self._run_script("--scenario", PRIMARY_SCENARIO)

        self.assertEqual(result.returncode, 0, result.stderr)
        receipt_path = (self.data_root / "assets" / "bodies" / "fixture_a" / "provenance"
                        / f"{PRIMARY_ASSET}_acquisition.json")
        self.assertTrue(receipt_path.is_file(), result.stdout)

        dsk_source.write_bytes(b"DAF/SPK " + bytes(1016))
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--overwrite")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not DAS/DSK", result.stderr)

    def test_archive_member_receipt_records_compressed_sha256(self) -> None:
        """Extract one declared ZIP member, record the archive SHA-256 and leave no temporary folder."""
        archive_path = self.data_root.parent / "fixture.zip"
        with zipfile.ZipFile(archive_path, "w") as archive:
            archive.write(SOURCE_ROOT / "triangle.tab", "nested/triangle.obj")
        self._update_asset(
            PRIMARY_ASSET,
            content_format="obj",
            local_path="assets/bodies/fixture_a/shape/triangle.obj",
            download_url=archive_path.as_uri(),
            archive_member="nested/triangle.obj",
        )
        result = self._run_script("--scenario", PRIMARY_SCENARIO)

        self.assertEqual(result.returncode, 0, result.stderr)
        shape_folder = self.data_root / "assets" / "bodies" / "fixture_a" / "shape"
        self.assertEqual(sorted(path.name for path in shape_folder.iterdir()), ["triangle.obj"])
        receipt_path = (self.data_root / "assets" / "bodies" / "fixture_a" / "provenance"
                        / f"{PRIMARY_ASSET}_acquisition.json")
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        self.assertEqual(receipt["compressed_sha256"], hashlib.sha256(archive_path.read_bytes()).hexdigest())

        self._update_asset(PRIMARY_ASSET, compressed_sha256="0" * 64)
        result = self._run_script("--scenario", PRIMARY_SCENARIO, "--overwrite")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Archive checksum mismatch", result.stderr)

    def test_asset_root_defaults_to_rendering_data_environment(self) -> None:
        """Read manifests from the data root and write payloads under SPECTRA_RT_RENDERING_DATA."""
        assets_root = self.data_root.parent / "rendering_assets"
        assets_root.mkdir()
        result = subprocess.run(
            [sys.executable, str(SCRIPT_PATH), "--data-root", str(self.data_root),
             "--scenario", PRIMARY_SCENARIO],
            check=False,
            capture_output=True,
            text=True,
            env={**os.environ, "SPECTRA_RT_RENDERING_DATA": str(assets_root)},
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((assets_root / "assets" / "fixture_a" / "shape" / "triangle.obj").is_file())
        self.assertFalse(self._destination_path().exists())

    def _run_script(self, *args: str | Path) -> subprocess.CompletedProcess[str]:
        """Run the real CLI against this test's temporary fixture roots.

        The temporary data root also serves as the asset root, so a developer's
        ``SPECTRA_RT_RENDERING_DATA`` is never written by these tests.

        Args:
            *args: Additional CLI arguments; both roots are supplied here.

        Returns:
            CLI exit status and captured text output.
        """
        return subprocess.run(
            [sys.executable, str(SCRIPT_PATH), "--data-root", str(self.data_root),
             "--assets-root", str(self.data_root), *map(str, args)],
            check=False,
            capture_output=True,
            text=True,
        )

    def _destination_path(self) -> Path:
        """Return the primary fixture's destination inside the temporary root."""
        return self.data_root / "assets" / "fixture_a" / "shape" / "triangle.obj"

    def _update_asset(self, asset_id: str, **fields: object) -> Path:
        """Change one temporary fixture asset for a failure or selection case.

        Args:
            asset_id: Asset in the primary fixture manifest.
            **fields: JSON field replacements for the test case.

        Returns:
            Path to the modified temporary manifest.

        Raises:
            StopIteration: If the requested asset is absent from the fixture.
        """
        manifest_path = self.data_root / "scenarios" / PRIMARY_SCENARIO / "manifest.json"
        manifest: dict[str, Any] = json.loads(manifest_path.read_text(encoding="utf-8"))
        asset = next(asset for asset in manifest["assets"] if asset["asset_id"] == asset_id)
        asset.update(fields)
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        return manifest_path


if __name__ == "__main__":
    unittest.main()
