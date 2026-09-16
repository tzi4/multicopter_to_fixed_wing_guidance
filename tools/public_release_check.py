#!/usr/bin/env python3
"""Fail closed when material unsafe for a public source repository is tracked."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REQUIRED = {
    "README.md",
    "LICENSE",
    "NOTICE",
    "THIRD_PARTY_NOTICES.md",
    "CONTRIBUTING.md",
    "DATA_POLICY.md",
    "PUBLIC_RELEASE.md",
    "SECURITY.md",
    "requirements.txt",
    "requirements-lock.txt",
    ".github/workflows/ci.yml",
}
FORBIDDEN_PREFIXES = (
    ".claude/",
    "IRL_Tests/",
    "run/",
    "logs/",
    "old_los_codes/",
    "videos/",
    "system_static_tests/",
)
FORBIDDEN_NAMES = {
    "all_star_env.zip",
    "eeprom.bin",
}
FORBIDDEN_SUFFIXES = {
    ".tlog",
    ".rlog",
}
MAX_PUBLIC_FILE_BYTES = 25 * 1024 * 1024

SECRET_PATTERNS = (
    ("private key", re.compile(r"BEGIN (?:RSA|OPENSSH|EC|DSA) PRIVATE KEY")),
    ("GitHub token", re.compile(r"gh[pousr]_[A-Za-z0-9_]{20,}")),
    ("AWS access key", re.compile(r"AKIA[0-9A-Z]{16}")),
    ("OpenAI-style secret", re.compile(r"sk-[A-Za-z0-9_-]{20,}")),
)
PERSONAL_PATH = re.compile(r"/home/[A-Za-z0-9._-]+(?:/|\b)")
PRIVATE_LOCATION_PATTERNS = (
    re.compile(r"41\.10[0-9]+"),
    re.compile(r"28\.5(?:4|5)[0-9]+"),
    re.compile(r"40\.967[0-9]+"),
    re.compile(r"29\.336[0-9]+"),
    re.compile(r"47\.402[0-9]+"),
    re.compile(r"8\.539[0-9]+"),
)
LOCATION_TEXT_SUFFIXES = {
    ".cfg",
    ".json",
    ".md",
    ".param",
    ".parm",
    ".plan",
    ".py",
    ".sh",
    ".txt",
    ".yaml",
    ".yml",
}


def tracked_files() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
    )
    return [item.decode("utf-8") for item in result.stdout.split(b"\0") if item]


def text_content(path: Path) -> str | None:
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if b"\0" in data[:8192]:
        return None
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


def main() -> int:
    errors: list[str] = []
    files = tracked_files()
    file_set = set(files)

    for required in sorted(REQUIRED - file_set):
        errors.append(f"required release file is not tracked: {required}")

    for name in files:
        path = ROOT / name
        if name in FORBIDDEN_NAMES:
            errors.append(f"private/generated file is tracked: {name}")
        if name.startswith(FORBIDDEN_PREFIXES):
            errors.append(f"private/generated path is tracked: {name}")
        if path.suffix.lower() in FORBIDDEN_SUFFIXES:
            errors.append(f"raw telemetry file is tracked: {name}")
        try:
            size = path.stat().st_size
        except OSError:
            continue
        if size > MAX_PUBLIC_FILE_BYTES:
            errors.append(
                f"file exceeds public source limit ({size / 1024 / 1024:.1f} MiB): {name}"
            )

        text = text_content(path)
        if text is None:
            continue
        if PERSONAL_PATH.search(text):
            errors.append(f"personal absolute path found: {name}")
        if (path.suffix.lower() in LOCATION_TEXT_SUFFIXES
                and any(pattern.search(text)
                        for pattern in PRIVATE_LOCATION_PATTERNS)):
            errors.append(f"known private/test-site coordinate found: {name}")
        for label, pattern in SECRET_PATTERNS:
            if pattern.search(text):
                errors.append(f"possible {label} found: {name}")

    if errors:
        print("PUBLIC RELEASE CHECK: FAILED", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(f"PUBLIC RELEASE CHECK: OK ({len(files)} tracked files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
