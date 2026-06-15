#!/usr/bin/env python3
"""Derive wheel versions for six-axis-platform CI builds."""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = REPO_ROOT / "pyproject.toml"


def die(message: str) -> None:
    print(f"ci-set-release-version: {message}", file=sys.stderr)
    raise SystemExit(1)


def fallback_version() -> str:
    text = PYPROJECT.read_text(encoding="utf-8")
    match = re.search(
        r'^\s*fallback_version\s*=\s*"([^"]+)"',
        text,
        re.MULTILINE,
    )
    if not match:
        die(f"missing [tool.setuptools_scm] fallback_version in {PYPROJECT}")
    return match.group(1)


def write_env_file(path: Path, base_version: str, pkg_version: str) -> None:
    path.write_text(
        f"DAQ_BASE_VERSION={base_version}\nPKG_VERSION={pkg_version}\n",
        encoding="utf-8",
    )


def main() -> int:
    os.chdir(REPO_ROOT)
    env_file = REPO_ROOT / os.environ.get("CI_VERSION_ENV_FILE", "ci-version.env")
    tag = os.environ.get("CI_COMMIT_TAG", "")

    if tag:
        if not re.fullmatch(r"v?[0-9]+\.[0-9]+\.[0-9]+", tag):
            die(f"release tags must be vX.Y.Z or X.Y.Z; got {tag}")
        base_version = tag.removeprefix("v")
        pkg_version = base_version
    else:
        base_version = os.environ.get("DAQ_BASE_VERSION", fallback_version())
        short_sha = os.environ.get("CI_COMMIT_SHORT_SHA", "local")
        pkg_version = os.environ.get(
            "PKG_VERSION",
            f"{base_version}+sha.{short_sha}",
        )

    write_env_file(env_file, base_version, pkg_version)
    print(f"DAQ_BASE_VERSION={base_version}")
    print(f"PKG_VERSION={pkg_version}")
    print(f"wrote {env_file.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
