"""Local bearer-token storage and request authentication."""

from __future__ import annotations

import hashlib
import hmac
import os
from pathlib import Path
import secrets
import tempfile


AUTH_TOKEN_FILENAME = "auth.token"
MIN_TOKEN_LENGTH = 32


def load_auth_token(path: str | Path) -> str:
    token_path = Path(path)
    try:
        token = token_path.read_text(encoding="utf-8").strip()
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"pairing token not found: {token_path}") from exc
    if len(token) < MIN_TOKEN_LENGTH or any(character.isspace() for character in token):
        raise ValueError(f"invalid pairing token: {token_path}")
    return token


def save_auth_token(path: str | Path, token: str | None = None) -> tuple[Path, str]:
    """Atomically create or replace a private token file."""
    token_path = Path(path)
    value = token or secrets.token_urlsafe(32)
    if len(value) < MIN_TOKEN_LENGTH or any(character.isspace() for character in value):
        raise ValueError("pairing token must contain at least 32 characters and no whitespace")

    token_path.parent.mkdir(parents=True, exist_ok=True)
    if os.name != "nt":
        os.chmod(token_path.parent, 0o700)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=token_path.parent,
            prefix=f".{token_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary.write(f"{value}\n")
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_path = Path(temporary.name)
        if os.name != "nt":
            os.chmod(temporary_path, 0o600)
        os.replace(temporary_path, token_path)
        if os.name != "nt":
            os.chmod(token_path, 0o600)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return token_path, value


def token_fingerprint(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()[:12]


def authorization_matches(header: str | None, token: str) -> bool:
    if not header:
        return False
    scheme, separator, candidate = header.partition(" ")
    if not separator or scheme.lower() != "bearer":
        return False
    candidate = candidate.strip()
    return bool(candidate) and hmac.compare_digest(candidate, token)
