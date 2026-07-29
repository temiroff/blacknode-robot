"""Installed Blacknode Robot package version."""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version


def service_version() -> str:
    try:
        return version("blacknode-robot")
    except PackageNotFoundError:
        return "0+unknown"
