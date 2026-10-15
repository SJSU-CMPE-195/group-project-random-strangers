from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from collections.abc import Mapping


def register_map_grabber(register_map_path: str | Path) -> Mapping[str, int]:
    """Read an I2C address map from a C++ header file.

    Args:
        register_map_path: Relative or absolute path to the C++ header
            containing the ``I2C_ADDRESS_MAP`` enum.

    Returns:
        A mapping from enum member names to integer register addresses. The
        mapping is a regular ``dict`` before Python 3.15 and a ``frozendict``
        on Python 3.15 and newer.

    Raises:
        FileNotFoundError: If ``register_map_path`` does not exist.
        ValueError: If the expected enum or an enum member value is invalid.
    """

    source = _resolve_register_map_path(register_map_path).read_text(encoding="utf-8")
    enum_body = _extract_enum_body(source)
    register_map = _parse_enum_members(enum_body)

    if sys.version_info >= (3, 15):
        from frozendict import frozendict

        return frozendict(register_map)

    try:
        from frozendict import frozendict
    except ImportError as error:
        raise RuntimeError(
            "Python versions before 3.15 require the 'frozendict' package."
        ) from error

    return frozendict(register_map)
def _resolve_register_map_path(register_map_path: str | Path) -> Path:
    """Resolve relative paths from this module's directory."""

    path = Path(register_map_path)
    if path.is_absolute():
        return path

    return (Path(__file__).resolve().parent / path).resolve()


#a regular expression to match the declaration of the I2C_ADDRESS_MAP enum.
_ENUM_DECLARATION = re.compile(
    r"\benum(?:\s+class)?\s+I2C_ADDRESS_MAP\b[^{}]*\{",
    re.MULTILINE
)

#a regular expression to match integer literals in C++.
_INTEGER_LITERAL = re.compile(
    r"(?P<literal>0[xX][0-9a-fA-F]+|0[bB][01]+|0[oO][0-7]+|\d+)"
    r"(?P<suffix>[uUlL]*)"
)

def _extract_enum_body(source: str) -> str:
    """Return the text between the braces of the address-map enum."""

    declaration = _ENUM_DECLARATION.search(source)
    if declaration is None:
        raise ValueError("Could not find enum I2C_ADDRESS_MAP")

    opening_brace = declaration.end() - 1
    closing_brace = _find_matching_brace(source, opening_brace)
    return source[opening_brace + 1:closing_brace]


def _find_matching_brace(source: str, opening_brace: int) -> int:
    """Find the closing brace matching ``opening_brace``."""

    brace_depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            brace_depth += 1
        elif source[index] == "}":
            brace_depth -= 1
            if brace_depth == 0:
                return index

    raise ValueError("I2C_ADDRESS_MAP enum has no matching closing brace")


def _parse_enum_members(enum_body: str) -> dict[str, int]:
    """Parse C++ enum members
    """

    members: dict[str, int] = {}
    automatic_value = 0

    for declaration in _remove_comments(enum_body).split(","):
        declaration = declaration.strip()
        if not declaration:
            continue

        name, separator, value_expression = declaration.partition("=")
        name = name.strip()
        if re.fullmatch(r"[A-Za-z_]\w*", name) is None:
            raise ValueError(f"Invalid I2C_ADDRESS_MAP member: {name!r}")

        if separator:
            value = _parse_integer_value(value_expression.strip(), name)
        else:
            value = automatic_value

        members[name] = value
        automatic_value = value + 1

    if not members:
        raise ValueError("I2C_ADDRESS_MAP enum does not contain any members")

    return members


def _remove_comments(enum_body: str) -> str:
    """Remove comments that occur within the enum body."""

    enum_body = re.sub(r"/\*.*?\*/", "", enum_body, flags=re.DOTALL)
    return re.sub(r"//[^\r\n]*", "", enum_body)


def _parse_integer_value(expression: str, member_name: str) -> int:
    """Parse an integer literal used as an enum member value."""

    literal = _INTEGER_LITERAL.fullmatch(expression.strip())
    if literal is None:
        raise ValueError(
            f"Invalid value for I2C_ADDRESS_MAP member {member_name}: "
            f"{expression!r}"
        )

    raw_literal = literal.group("literal")
    if raw_literal.lower().startswith("0x"):
        base = 16
    elif raw_literal.lower().startswith("0b"):
        base = 2
    elif raw_literal.lower().startswith("0o"):
        base = 8
    else:
        base = 10

    return int(raw_literal, base)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(description="Parse an I2C register map.")
    parser.add_argument("register_map_path", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_args()
    print(register_map_grabber(arguments.register_map_path))