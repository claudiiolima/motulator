"""Base classes for controls."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Protocol

import numpy as np

from motulator.common._converter_mode import ConverterMode


# %%
class References(Protocol):
    """Protocol defining the interface for reference signals."""

    T_s: float  # Sampling period for next control cycle
    d_abc: Sequence[float]  # Duty ratios for three-phase PWM


@dataclass
class TimeSeries:
    """Container for control system's discrete-time data."""

    t: np.ndarray = field(default_factory=lambda: np.array([]))
    # Populated in post_process
    fbk: Any = field(default_factory=dict)
    ref: Any = field(default_factory=dict)


class ControlSystem[Mdl, Meas, Ref: References, Fbk](Protocol):
    """
    Base class for control systems.

    This class defines the interface for control systems. It is a generic class that can
    be used with different models, measurements, feedback signals, and reference
    signals. The class provides methods for saving, post-processing, and clearing data.

    """

    t: float
    converter_mode: ConverterMode | Callable[[float], ConverterMode]
    active_converter_mode: ConverterMode
    enabled: bool
    # Time and signal history
    _t: list[float]
    _history: dict[str, dict[str, list]]

    def __init__(self) -> None:
        self.t: float = 0.0  # Controller time
        self.converter_mode = ConverterMode.NORMAL
        self.active_converter_mode = ConverterMode.NORMAL
        self.enabled = True
        # Initialize the data buffer
        self._t: list[float] = []
        self._history: dict[str, dict[str, list]] = {}

    def set_converter_mode(
        self, mode: ConverterMode | Callable[[float], ConverterMode]
    ) -> None:
        """Set the converter operating mode or its time-dependent reference."""
        self.converter_mode = mode

    def resolve_converter_mode(self, t: float) -> ConverterMode:
        """Resolve the converter mode reference at the given time."""
        mode = (
            self.converter_mode(t)
            if callable(self.converter_mode)
            else self.converter_mode
        )
        return ConverterMode(mode)

    def on_disabled(self, mdl: Mdl) -> None:
        """Track the physical system while converter control is disabled."""

    def flying_start(self, mdl: Mdl) -> None:
        """Synchronize control states before re-enabling normal modulation."""

    def _prepare_converter_mode(self, mdl: Mdl) -> None:
        """Update the enable state and run mode-transition hooks."""
        was_enabled = self.enabled
        t = getattr(mdl, "t0", self.t)
        self.active_converter_mode = self.resolve_converter_mode(t)
        self.enabled = self.active_converter_mode is ConverterMode.NORMAL
        if not self.enabled:
            self.on_disabled(mdl)
        elif not was_enabled:
            self.flying_start(mdl)

    def get_measurement(self, mdl: Mdl) -> Meas:
        """Get measurements from the model."""
        ...

    def get_feedback(self, meas: Meas) -> Fbk:
        """Get feedback signals from the model."""
        ...

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute controller output based on feedback."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller internal states."""
        self.t = (self.t + ref.T_s) % 1e9  # Avoid overflow

    def save(self, t: float, **signal_groups: Any) -> None:
        """Save a single timestep of data."""
        self._t.append(t)
        # Save all signals from each group
        for group_name, signals in signal_groups.items():
            if group_name not in self._history:
                self._history[group_name] = {}
            for key, value in vars(signals).items():
                self._history[group_name].setdefault(key, []).append(value)

    def run_control_loop(self, mdl: Mdl) -> tuple[float, Sequence[float]]:
        """Run the default control loop, can be overridden."""
        self._prepare_converter_mode(mdl)
        meas = self.get_measurement(mdl)
        fbk = self.get_feedback(meas)
        ref = self.compute_output(fbk)
        self.save(self.t, ref=ref, fbk=fbk)
        self.update(ref, fbk)
        return self.get_duty_ratios(ref)

    def get_duty_ratios(self, ref: References) -> tuple[float, Sequence[float]]:
        """Extract duty ratios from the reference signals."""
        return ref.T_s, ref.d_abc

    def __call__(self, mdl: Mdl) -> tuple[float, Sequence[float]]:
        """Make the control system callable."""
        return self.run_control_loop(mdl)

    def post_process(self) -> TimeSeries:
        """Convert stored lists to numpy arrays."""
        ts = TimeSeries()
        ts.t = np.array(self._t)
        # Convert each signal group to a namespace
        for group_name, signals in self._history.items():
            group_data = {k: np.array(v) for k, v in signals.items()}
            setattr(ts, group_name, SimpleNamespace(**group_data))
        return ts

    def clear_data(self) -> None:
        """Clear all stored data."""
        self._t.clear()
        self._history.clear()
