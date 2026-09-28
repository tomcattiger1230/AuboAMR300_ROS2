"""Shared AGV status field semantics."""
from __future__ import annotations

from typing import Any, Mapping


# The 1012 response includes ``electric`` as the electric loop state. On the
# current robot, ``electric=True`` is reported while the physical e-stop is not
# active, so it must not be treated as an e-stop source.
ESTOP_TRIGGER_FIELDS: tuple[str, ...] = (
    "emergency",
    "driver_emc",
    "electric_emc",
    "soft_emc",
)


def is_active_flag(value: Any) -> bool:
    """Return whether an SDK status value means an active/triggered flag."""
    if value is True:
        return True
    if value == 1:
        return True
    if isinstance(value, str):
        return value.strip().lower() in {"true", "1", "yes", "on"}
    return False


def estop_triggered_keys(estop: Mapping[str, Any] | None) -> list[str]:
    """Return e-stop source keys that are currently triggered."""
    if not isinstance(estop, Mapping):
        return []
    return [key for key in ESTOP_TRIGGER_FIELDS if is_active_flag(estop.get(key))]


def estop_is_triggered(estop: Mapping[str, Any] | None) -> bool:
    """Return whether any actual e-stop source is triggered."""
    return bool(estop_triggered_keys(estop))


def estop_triggered_labels(estop: Mapping[str, Any] | None, labels: Mapping[str, str]) -> list[str]:
    """Return display labels for triggered e-stop sources."""
    return [labels.get(key, key) for key in estop_triggered_keys(estop)]
