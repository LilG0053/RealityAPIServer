"""Numeric settings the dashboard can change with a slider."""

import math


class TunableParameter:
    """Class attribute that reads as a float and shows up as a dashboard slider.

    Reading ``Class.NAME`` or ``instance.NAME`` returns the current value.
    Only parameters with a finite minimum and maximum are listed for the
    dashboard; an infinite bound is a fixed setting, not a slider.
    """

    _registry: list["TunableParameter"] = []

    def __init__(self, default: float, *, minimum: float, maximum: float, step: float, label: str):
        if not math.isfinite(step) or step <= 0:
            raise ValueError("step must be a positive finite number")
        if not math.isfinite(default):
            raise ValueError("default must be finite")
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.step = float(step)
        self.label = label
        self.name = ""
        self.value = self._clamp(float(default))

    def __set_name__(self, owner, name: str) -> None:
        self.name = f"{owner.__name__}.{name}"
        if math.isfinite(self.minimum) and math.isfinite(self.maximum):
            TunableParameter._registry.append(self)

    def __get__(self, obj, objtype=None) -> float:
        return self.value

    def __set__(self, obj, value: float) -> None:
        self.set(value)

    def set(self, value: float) -> float:
        """Store a new value snapped to the slider step and clamped to the range."""
        self.value = self._clamp(float(value))
        return self.value

    def _clamp(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("value must be finite")
        snapped = self.minimum + round((value - self.minimum) / self.step) * self.step
        if snapped < self.minimum:
            snapped = self.minimum
        elif snapped > self.maximum:
            snapped = self.maximum
        return snapped

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "label": self.label,
            "value": self.value,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "step": self.step,
        }


def list_tunables() -> list[dict]:
    return [parameter.to_dict() for parameter in TunableParameter._registry]


def set_tunable(name: str, value: float) -> dict:
    for parameter in TunableParameter._registry:
        if parameter.name == name:
            parameter.set(value)
            return parameter.to_dict()
    raise KeyError(f"Unknown tunable parameter: {name}")
