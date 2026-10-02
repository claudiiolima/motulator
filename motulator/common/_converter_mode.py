"""Converter operating modes shared by control and model components."""

from enum import Enum


class ConverterMode(str, Enum):
    """Converter operating mode.

    ``NORMAL`` respects the switching states generated from the duty ratios.
    ``ALL_PHASE_OPEN`` turns off all transistors. ``ACTIVE_SHORT_LOW`` and
    ``ACTIVE_SHORT_HIGH`` turn on all low-side or all high-side transistors,
    respectively.
    """

    NORMAL = "normal"
    ALL_PHASE_OPEN = "all_phase_open"
    ACTIVE_SHORT_LOW = "active_short_low"
    ACTIVE_SHORT_HIGH = "active_short_high"
