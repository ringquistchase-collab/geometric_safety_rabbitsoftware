"""Smoke tests for the public paper reproduction path."""

from pathlib import Path
import subprocess
import sys
import tarfile

from paper.check_reproducibility_artifacts import check_run
from paper.reproduce_paper_outputs import locate_run_root

REPO_ROOT = Path(__file__).resolve().parents[2]
INCLUDED_ARCHIVE = REPO_ROOT / "paper" / "results" / "paper_full_20260502_0738.tar.gz"
PROJECTION_ARCHIVE = REPO_ROOT / "paper" / "results" / "projection_deterministic_51_cap_100nmi.tar.gz"


def test_included_archive_supports_public_reproduction_dry_run(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "paper.reproduce_paper_outputs",
            "--output-root",
            str(tmp_path),
            "--archive",
            str(INCLUDED_ARCHIVE),
            "--skip-application",
            "--dry-run",
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "IMPORT mixed-turn stress maps" in result.stdout
    assert "IMPORT projection-sensitivity supplement figure" in result.stdout
    assert not (tmp_path / "manuscript").exists()


def test_included_archive_passes_artifact_checker(tmp_path: Path) -> None:
    with tarfile.open(INCLUDED_ARCHIVE, "r:gz") as archive:
        archive.extractall(tmp_path, filter="data")

    run_root = locate_run_root(tmp_path)

    assert check_run(run_root, PROJECTION_ARCHIVE, require_rendered=True) == []
