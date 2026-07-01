#!/usr/bin/env python3
"""Fetch optional SimulationGears scenario assets from tracked manifests.

This script is intentionally outside the MATLAB runtime path. Scenario
builders keep failing fast when assets are missing; users can run this CLI when
they explicitly want to synchronize data under the SimGears data root.

Examples:
    Dry-run the runnable shape assets for Eros::

        python3 tools/data/fetch_scenario_assets.py --scenario Eros --dry-run

    Fetch one explicit asset into a custom data root::

        python3 tools/data/fetch_scenario_assets.py --data-root /data/simgears \
            --scenario Eros --asset-id naif_eros_dsk_q64
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAX_SIZE_GB = 1.0
REQUIRED_MANIFEST_FIELDS = {
    "schema_version",
    "scenario_name",
    "canonical_name",
    "aliases",
    "confidence",
    "tags",
    "default_shape_asset_id",
    "assets",
}
REQUIRED_ASSET_FIELDS = {
    "asset_id",
    "asset_type",
    "local_path",
    "source_url",
    "download_url",
    "sha256",
    "size_gb",
    "fidelity",
    "required_for_shape_runnable",
}


class FetchError(RuntimeError):
    """Raised when manifests or assets cannot be processed safely."""

    pass


@dataclass(frozen=True)
class ScenarioManifest:
    """Scenario manifest loaded from ``data/scenarios/<name>/manifest.json``.

    Attributes:
        name: Canonical scenario name from the manifest payload.
        path: Absolute path to the manifest file.
        payload: Decoded JSON manifest dictionary.
    """

    name: str
    path: Path
    payload: dict


@dataclass(frozen=True)
class AssetPlan:
    """Resolved work item for one scenario asset.

    Attributes:
        scenario_name: Canonical scenario name.
        asset_id: Manifest asset identifier.
        asset_type: Asset category such as ``shape`` or ``albedo``.
        data_root: Absolute SimulationGears data root.
        local_path: Absolute destination path for the asset.
        manifest_path: Absolute path to the owning manifest.
        download_url: Direct download URL, if available.
        sha256: Expected checksum for the final local asset, if declared.
        size_gb: Declared asset size in GB.
        payload: Original decoded asset dictionary.
    """

    scenario_name: str
    asset_id: str
    asset_type: str
    data_root: Path
    local_path: Path
    manifest_path: Path
    download_url: str
    sha256: str
    size_gb: float
    payload: dict


def main(argv: list[str] | None = None) -> int:
    """Run the scenario-asset fetch CLI.

    Args:
        argv: Optional argument vector. ``None`` uses ``sys.argv``.

    Returns:
        Process-style exit code. ``0`` means all selected assets were planned,
        verified, or fetched successfully.
    """

    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        data_root = resolve_data_root(args.data_root)
        selected_manifests = select_manifests(data_root, args.scenario, args.all)
        include_types = set(args.include or ["shape"])
        required_shape_only = args.asset_id is None and args.include is None
        plans: list[AssetPlan] = []
        for manifest in selected_manifests:
            plans.extend(
                build_asset_plans(
                    data_root=data_root,
                    manifest=manifest,
                    asset_ids=args.asset_id,
                    include_types=include_types,
                    required_shape_only=required_shape_only,
                    allow_large=args.allow_large,
                    max_size_gb=args.max_size_gb,
                )
            )

        if not plans:
            raise FetchError("No assets matched the requested filters.")

        for plan in plans:
            process_asset_plan(plan, dry_run=args.dry_run, verify_only=args.verify_only, overwrite=args.overwrite)
    except FetchError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    return 0


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser.

    Returns:
        Configured ``argparse.ArgumentParser`` for this script.
    """

    parser = argparse.ArgumentParser(
        description="Fetch optional SimulationGears scenario assets from data/scenarios/*/manifest.json."
    )
    scenario_group = parser.add_mutually_exclusive_group(required=True)
    scenario_group.add_argument("--scenario", action="append", help="Scenario name or alias. May be repeated.")
    scenario_group.add_argument("--all", action="store_true", help="Process all scenario manifests.")
    parser.add_argument("--data-root", type=Path, default=None, help="SimulationGears data root.")
    parser.add_argument("--asset-id", action="append", help="Specific asset id to process. May be repeated.")
    parser.add_argument(
        "--include",
        action="append",
        choices=("shape", "albedo", "spectral", "spice"),
        help=(
            "Asset type to include when --asset-id is omitted. When omitted, "
            "only required shape-runnable assets are selected."
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the plan without downloading.")
    parser.add_argument("--verify-only", action="store_true", help="Only verify that selected assets exist and match checksums.")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing local assets.")
    parser.add_argument("--allow-large", action="store_true", help="Allow assets larger than --max-size-gb.")
    parser.add_argument("--max-size-gb", type=float, default=DEFAULT_MAX_SIZE_GB, help="Preferred size limit.")
    return parser


def resolve_data_root(cli_data_root: Path | None) -> Path:
    """Resolve and validate the SimulationGears data root.

    Args:
        cli_data_root: Optional path from ``--data-root``.

    Returns:
        Absolute data-root path.

    Raises:
        FetchError: If the resolved data root does not exist.
    """

    if cli_data_root is not None:
        data_root = cli_data_root
    elif os.environ.get("SIMGEARS_DATA_ROOT"):
        data_root = Path(os.environ["SIMGEARS_DATA_ROOT"])
    else:
        data_root = REPO_ROOT / "data"

    data_root = data_root.expanduser().resolve()
    if not data_root.is_dir():
        raise FetchError(
            f"SimulationGears data root not found: {data_root}\n"
            "Create it, pass --data-root, or set SIMGEARS_DATA_ROOT."
        )
    return data_root


def select_manifests(data_root: Path, scenario_names: list[str] | None, process_all: bool) -> list[ScenarioManifest]:
    """Select scenario manifests by name, alias, or ``--all``.

    Args:
        data_root: Absolute SimulationGears data root.
        scenario_names: Scenario names or aliases requested on the CLI.
        process_all: Whether to return every manifest.

    Returns:
        Matching scenario manifests.

    Raises:
        FetchError: If any requested scenario is unknown.
    """

    manifests = load_all_manifests(data_root)
    if process_all:
        return manifests

    assert scenario_names is not None
    selected: list[ScenarioManifest] = []
    available_names: list[str] = []
    for manifest in manifests:
        names = manifest_aliases(manifest.payload)
        available_names.extend(sorted(names))
        for scenario_name in scenario_names:
            if scenario_name.casefold() in names:
                selected.append(manifest)
                break

    missing = [name for name in scenario_names if all(name.casefold() not in manifest_aliases(m.payload) for m in manifests)]
    if missing:
        raise FetchError(
            "Unknown scenario(s): "
            + ", ".join(missing)
            + "\nAvailable names and aliases: "
            + ", ".join(sorted(set(available_names)))
        )

    return selected


def load_all_manifests(data_root: Path) -> list[ScenarioManifest]:
    """Load every tracked scenario manifest under a data root.

    Args:
        data_root: Absolute SimulationGears data root.

    Returns:
        Loaded and schema-checked manifests sorted by path.

    Raises:
        FetchError: If the manifest folder, JSON, or schema is invalid.
    """

    scenarios_root = data_root / "scenarios"
    if not scenarios_root.is_dir():
        raise FetchError(f"Scenario manifest folder not found: {scenarios_root}")

    manifests: list[ScenarioManifest] = []
    for manifest_path in sorted(scenarios_root.glob("*/manifest.json")):
        try:
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise FetchError(f"Invalid JSON manifest {manifest_path}: {exc}") from exc

        validate_manifest_schema(payload, manifest_path)
        manifests.append(ScenarioManifest(name=str(payload["scenario_name"]), path=manifest_path, payload=payload))

    if not manifests:
        raise FetchError(f"No scenario manifests found under {scenarios_root}")

    return manifests


def validate_manifest_schema(payload: dict, manifest_path: Path) -> None:
    """Validate the fields needed by the fetch script.

    Args:
        payload: Decoded manifest payload.
        manifest_path: Manifest path used in diagnostics.

    Raises:
        FetchError: If required manifest or asset fields are missing.
    """

    missing_fields = sorted(REQUIRED_MANIFEST_FIELDS.difference(payload))
    if missing_fields:
        raise FetchError(f"Invalid manifest {manifest_path}: missing field(s): {', '.join(missing_fields)}")

    assets = payload["assets"]
    if not isinstance(assets, list) or not assets:
        raise FetchError(f"Invalid manifest {manifest_path}: assets must be a non-empty list.")

    for asset in assets:
        missing_asset_fields = sorted(REQUIRED_ASSET_FIELDS.difference(asset))
        if missing_asset_fields:
            asset_id = asset.get("asset_id", "<unknown>")
            raise FetchError(
                f"Invalid manifest {manifest_path}: asset {asset_id} missing field(s): "
                + ", ".join(missing_asset_fields)
            )


def manifest_aliases(payload: dict) -> set[str]:
    """Return case-folded names that can identify a scenario manifest.

    Args:
        payload: Decoded manifest payload.

    Returns:
        Case-folded canonical names and aliases.
    """

    names = {str(payload["scenario_name"]).casefold(), str(payload["canonical_name"]).casefold()}
    names.update(str(alias).casefold() for alias in payload.get("aliases", []))
    return names


def build_asset_plans(
    data_root: Path,
    manifest: ScenarioManifest,
    asset_ids: list[str] | None,
    include_types: set[str],
    required_shape_only: bool,
    allow_large: bool,
    max_size_gb: float,
) -> list[AssetPlan]:
    """Build local fetch/verify plans for selected assets.

    Args:
        data_root: Absolute SimulationGears data root.
        manifest: Loaded scenario manifest.
        asset_ids: Explicit asset ids requested by the user.
        include_types: Asset types requested when no explicit ids are given.
        required_shape_only: Whether the default selection should keep only
            assets required for shape-runnable scenarios.
        allow_large: Whether to allow assets above ``max_size_gb``.
        max_size_gb: Preferred size limit in GB.

    Returns:
        Resolved asset plans.

    Raises:
        FetchError: If an asset id is unknown, a path escapes the data root, or
        a selected asset exceeds the configured size limit.
    """

    assets = list(manifest.payload["assets"])
    if asset_ids:
        requested = set(asset_ids)
        known = {str(asset["asset_id"]) for asset in assets}
        missing = sorted(requested.difference(known))
        if missing:
            raise FetchError(
                f"Manifest {manifest.path} for {manifest.name} does not define asset id(s): "
                + ", ".join(missing)
            )
        assets = [asset for asset in assets if str(asset["asset_id"]) in requested]
    else:
        assets = [asset for asset in assets if str(asset["asset_type"]) in include_types]
        if required_shape_only:
            assets = [asset for asset in assets if bool(asset["required_for_shape_runnable"])]

    plans: list[AssetPlan] = []
    for asset in assets:
        size_gb = float(asset["size_gb"])
        asset_id = str(asset["asset_id"])
        if size_gb > max_size_gb and not allow_large:
            raise FetchError(
                f"Asset {asset_id} for {manifest.name} is {size_gb:.3g} GB, above the "
                f"{max_size_gb:.3g} GB preferred limit. Re-run with --allow-large to fetch it explicitly."
            )

        plans.append(
            AssetPlan(
                scenario_name=manifest.name,
                asset_id=asset_id,
                asset_type=str(asset["asset_type"]),
                data_root=data_root,
                local_path=resolve_local_asset_path(data_root, str(asset["local_path"]), manifest.path, asset_id),
                manifest_path=manifest.path,
                download_url=str(asset["download_url"]),
                sha256=str(asset["sha256"]),
                size_gb=size_gb,
                payload=asset,
            )
        )

    return plans


def resolve_local_asset_path(data_root: Path, local_path: str, manifest_path: Path, asset_id: str) -> Path:
    """Resolve a manifest local path and prevent path traversal.

    Args:
        data_root: Absolute SimulationGears data root.
        local_path: Manifest-relative asset path.
        manifest_path: Manifest path used in diagnostics.
        asset_id: Asset id used in diagnostics.

    Returns:
        Absolute local asset path.

    Raises:
        FetchError: If the path resolves outside ``data_root``.
    """

    resolved_path = (data_root / local_path).resolve()
    try:
        resolved_path.relative_to(data_root)
    except ValueError as exc:
        raise FetchError(
            f"Asset {asset_id} in {manifest_path} has local_path {local_path!r}, "
            f"which resolves outside data root {data_root}: {resolved_path}"
        ) from exc
    return resolved_path


def process_asset_plan(plan: AssetPlan, dry_run: bool, verify_only: bool, overwrite: bool) -> None:
    """Execute one asset plan according to the selected mode.

    Args:
        plan: Resolved asset work item.
        dry_run: Print intended actions without touching assets.
        verify_only: Require local assets to exist and verify checksums.
        overwrite: Replace existing assets during fetch.

    Raises:
        FetchError: If the local asset is missing, cannot be fetched, or fails
        checksum validation.
    """

    exists = plan.local_path.exists()
    if dry_run:
        status = "exists" if exists else "would_download"
        print(f"DRY-RUN {plan.scenario_name} {plan.asset_id} {plan.asset_type} {plan.local_path} {status}")
        return

    if verify_only:
        verify_existing_asset(plan)
        print(f"OK {plan.scenario_name} {plan.asset_id} {plan.local_path}")
        return

    if exists and not overwrite:
        verify_asset_checksum(plan)
        print(f"EXISTS {plan.scenario_name} {plan.asset_id} {plan.local_path}")
        return

    if not plan.download_url:
        verify_command = cli_command_for_plan(plan, "--verify-only")
        raise FetchError(
            f"Asset {plan.asset_id} for scenario {plan.scenario_name} is missing and has no download_url.\n"
            f"Expected: {plan.local_path}\n"
            f"Manifest: {plan.manifest_path}\n"
            "Add the asset manually at the expected path, or update download_url in the manifest.\n"
            "Verify after manual placement with:\n"
            f"  {verify_command}"
        )

    fetch_asset(plan, overwrite=overwrite)
    verify_asset_checksum(plan)
    print(f"FETCHED {plan.scenario_name} {plan.asset_id} {plan.local_path}")


def verify_existing_asset(plan: AssetPlan) -> None:
    """Verify that a selected asset already exists locally.

    Args:
        plan: Resolved asset work item.

    Raises:
        FetchError: If the path is missing or checksum validation fails.
    """

    if not plan.local_path.exists():
        fetch_command = cli_command_for_plan(plan)
        raise FetchError(
            f"Required local asset {plan.asset_id} for scenario {plan.scenario_name} was not found.\n"
            f"Expected: {plan.local_path}\n"
            f"Manifest: {plan.manifest_path}\n"
            "Fetch it with:\n"
            f"  {fetch_command}"
        )
    verify_asset_checksum(plan)


def cli_command_for_plan(plan: AssetPlan, *extra_args: str) -> str:
    """Build a copy-pasteable fetch command for one asset plan.

    Args:
        plan: Resolved asset work item.
        *extra_args: Additional CLI arguments to append.

    Returns:
        Command string rooted at the repository script path.
    """

    command_parts = [
        "python3",
        "tools/data/fetch_scenario_assets.py",
        "--scenario",
        plan.scenario_name,
        "--asset-id",
        plan.asset_id,
    ]
    command_parts.extend(extra_args)
    return " ".join(command_parts)


def verify_asset_checksum(plan: AssetPlan) -> None:
    """Verify a declared final-asset SHA-256 checksum.

    Args:
        plan: Resolved asset work item.

    Raises:
        FetchError: If a declared checksum does not match the local file.
    """

    if not plan.sha256 or plan.local_path.is_dir():
        return

    actual_sha256 = sha256_file(plan.local_path)
    if actual_sha256.casefold() != plan.sha256.casefold():
        raise FetchError(
            f"Checksum mismatch for asset {plan.asset_id}.\n"
            f"Expected sha256: {plan.sha256}\n"
            f"Actual sha256:   {actual_sha256}\n"
            f"Local path:      {plan.local_path}\n"
            f"Manifest:        {plan.manifest_path}"
        )


def fetch_asset(plan: AssetPlan, overwrite: bool) -> None:
    """Fetch one asset using direct-file or explicit archive metadata.

    Args:
        plan: Resolved asset work item.
        overwrite: Replace an existing destination.

    Raises:
        FetchError: If an archive lacks explicit extraction metadata or the
        download/extraction fails.
    """

    if is_archive_download(plan) and not has_archive_metadata(plan):
        raise FetchError(
            f"Asset {plan.asset_id} download_url appears to be an archive, but {plan.manifest_path} "
            "does not declare archive_member or unpack_to. Add explicit archive metadata before fetching."
        )

    if has_archive_metadata(plan):
        fetch_archive_asset(plan, overwrite=overwrite)
    else:
        fetch_direct_file(plan, overwrite=overwrite)


def is_archive_download(plan: AssetPlan) -> bool:
    """Return whether an asset URL appears to reference a ZIP archive."""

    return plan.download_url.lower().split("?", 1)[0].endswith(".zip")


def has_archive_metadata(plan: AssetPlan) -> bool:
    """Return whether a manifest explicitly declares archive handling."""

    return bool(plan.payload.get("archive_member") or plan.payload.get("unpack_to"))


def fetch_direct_file(plan: AssetPlan, overwrite: bool) -> None:
    """Download a direct asset URL into its final local path.

    Args:
        plan: Resolved asset work item.
        overwrite: Replace an existing destination.

    Raises:
        FetchError: If the download fails.
    """

    plan.local_path.parent.mkdir(parents=True, exist_ok=True)
    if plan.local_path.exists() and overwrite:
        remove_existing_path(plan.local_path)

    with tempfile.NamedTemporaryFile(delete=False, dir=plan.local_path.parent, prefix=plan.local_path.name + ".") as tmp_file:
        tmp_path = Path(tmp_file.name)

    try:
        download_to_path(plan.download_url, tmp_path)
        tmp_path.replace(plan.local_path)
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise


def fetch_archive_asset(plan: AssetPlan, overwrite: bool) -> None:
    """Download a ZIP asset and extract only explicitly declared content.

    Args:
        plan: Resolved asset work item with ``archive_member`` or ``unpack_to``.
        overwrite: Replace an existing destination.

    Raises:
        FetchError: If checksum validation or extraction fails.
    """

    with tempfile.TemporaryDirectory(prefix="simgears-asset-") as tmp_dir:
        archive_path = Path(tmp_dir) / "asset.zip"
        download_to_path(plan.download_url, archive_path)

        compressed_sha256 = str(plan.payload.get("compressed_sha256", ""))
        if compressed_sha256:
            actual_sha256 = sha256_file(archive_path)
            if actual_sha256.casefold() != compressed_sha256.casefold():
                raise FetchError(
                    f"Archive checksum mismatch for asset {plan.asset_id}.\n"
                    f"Expected sha256: {compressed_sha256}\n"
                    f"Actual sha256:   {actual_sha256}\n"
                    f"Manifest:        {plan.manifest_path}"
                )

        if plan.payload.get("archive_member"):
            extract_archive_member(archive_path, str(plan.payload["archive_member"]), plan.local_path, overwrite)
        else:
            unpack_to = resolve_local_asset_path(
                plan.data_root,
                str(plan.payload["unpack_to"]),
                plan.manifest_path,
                plan.asset_id,
            )
            extract_archive_tree(archive_path, unpack_to, overwrite)


def extract_archive_member(archive_path: Path, member_name: str, local_path: Path, overwrite: bool) -> None:
    """Extract one declared ZIP member to a final local path.

    Args:
        archive_path: Downloaded ZIP archive.
        member_name: Exact member name expected inside the archive.
        local_path: Final local path for the extracted file.
        overwrite: Replace an existing destination.

    Raises:
        FetchError: If the declared member is absent.
    """

    local_path.parent.mkdir(parents=True, exist_ok=True)
    if local_path.exists() and overwrite:
        remove_existing_path(local_path)

    with zipfile.ZipFile(archive_path) as archive:
        if member_name not in archive.namelist():
            raise FetchError(f"Archive {archive_path} does not contain declared member {member_name}.")
        with archive.open(member_name) as source, tempfile.NamedTemporaryFile(
            delete=False, dir=local_path.parent, prefix=local_path.name + "."
        ) as target:
            shutil.copyfileobj(source, target)
            tmp_path = Path(target.name)

    try:
        tmp_path.replace(local_path)
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise


def extract_archive_tree(archive_path: Path, unpack_to: Path, overwrite: bool) -> None:
    """Extract a ZIP archive into a validated destination folder.

    Args:
        archive_path: Downloaded ZIP archive.
        unpack_to: Destination folder under the data root.
        overwrite: Replace an existing destination folder.

    Raises:
        FetchError: If any ZIP member would escape ``unpack_to``.
    """

    if unpack_to.exists() and overwrite:
        remove_existing_path(unpack_to)
    unpack_to.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(archive_path) as archive:
        for member in archive.infolist():
            destination = (unpack_to / member.filename).resolve()
            try:
                destination.relative_to(unpack_to)
            except ValueError as exc:
                raise FetchError(f"Archive member {member.filename} would extract outside {unpack_to}.") from exc
        archive.extractall(unpack_to)


def download_to_path(url: str, output_path: Path) -> None:
    """Stream a URL to a local file.

    Args:
        url: Source URL.
        output_path: Local file to write.

    Raises:
        FetchError: If the URL cannot be downloaded.
    """

    request = urllib.request.Request(url, headers={"User-Agent": "SimulationGearsDataFetcher/1.0"})
    try:
        with urllib.request.urlopen(request) as response, output_path.open("wb") as target:
            shutil.copyfileobj(response, target)
    except Exception as exc:
        raise FetchError(f"Failed to download {url}: {exc}") from exc


def sha256_file(path: Path) -> str:
    """Compute a file SHA-256 digest.

    Args:
        path: File path.

    Returns:
        Lower-case hexadecimal SHA-256 digest.
    """

    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def remove_existing_path(path: Path) -> None:
    """Remove a file or directory before replacing it.

    Args:
        path: Existing file or directory path.
    """

    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
