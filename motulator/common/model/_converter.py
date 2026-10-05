"""
Continuous-time models for converters.

A three-phase voltage-source inverter with optional DC-bus dynamics is modeled, along
with a six-pulse diode bridge rectifier supplied from a stiff grid. Complex space
vectors are used also for duty ratios and switching states, wherever applicable.

"""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from math import sqrt
from typing import Any

import numpy as np

from motulator.common._converter_mode import ConverterMode
from motulator.common.model._base import Subsystem
from motulator.common.utils._utils import abc2complex, complex2abc, empty_array


# %%
@dataclass
class Inputs:
    """Input variables."""

    q_c_ab: complex | np.ndarray = 0j
    converter_mode: ConverterMode = ConverterMode.NORMAL
    i_c_ab: complex = 0j
    u_c_open_ab: complex = 0j
    i_dc: float | Callable[[float], float] | None = None


@dataclass
class Outputs:
    """Output variables for interconnection."""

    u_c_ab: complex
    u_dc: float


class VoltageSourceConverter(Subsystem):
    """
    Lossless three-phase voltage-source converter with constant DC-bus voltage.

    The switching input can be a complex duty-ratio or switching-state vector, or an
    explicit three-phase state array. The separate :class:`ConverterMode` input either
    respects this switching input or overrides it with Hi-Z or active short. In an
    explicit phase-state array, 1 turns on the high-side transistor, 0 turns on the
    low-side transistor, and -1 turns off both transistors. In the latter case, the
    antiparallel diodes determine the pole voltage from the phase-current direction.

    The switches are ideal, except for the dead time. When a leg is blanked, i.e.,
    neither of its switches conducts, the current direction determines which diode
    conducts. The leg state is then `(1 - sign(i))/2`, where `i` is the phase current
    in the beginning of the blanking interval.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V).
    i_tol : float, optional
        Current threshold for releasing the antiparallel diodes in Hi-Z (A), defaults
        to 1e-3.
    t_d : float, optional
        Dead time (s), defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents (A) determining the leg states during blanking,
        defaults to `np.sign`. A smooth function, such as `2/pi*arctan(i/i_d)`, can be
        used to model the effect of parasitic capacitances, for example.

    """

    n_legs: int = 3

    def __init__(
        self,
        u_dc: float,
        i_tol: float = 1e-3,
        t_d: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
    ) -> None:
        self.u_dc = u_dc
        self.i_tol = i_tol
        self.t_d = t_d
        self.sign = sign
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=u_dc)
        self.state = None
        self._history = None

    def set_external_dc_current(self, i_dc: Callable[[float], float]) -> None:
        """Set external DC current (A)."""
        raise NotImplementedError

    @staticmethod
    def _get_phase_states(
        q_c_ab: Any, converter_mode: ConverterMode
    ) -> np.ndarray | None:
        """Resolve the converter mode and explicit phase-leg states."""
        mode = ConverterMode(converter_mode)
        if mode is not ConverterMode.NORMAL:
            states = {
                ConverterMode.ALL_PHASE_OPEN: -1,
                ConverterMode.ACTIVE_SHORT_LOW: 0,
                ConverterMode.ACTIVE_SHORT_HIGH: 1,
            }
            return np.full(3, states[mode], dtype=int)

        if np.isscalar(q_c_ab):
            return None

        q_c_abc = np.asarray(q_c_ab)
        if q_c_abc.shape != (3,):
            raise ValueError("Phase switching states must have shape (3,)")
        if not np.all(np.isin(q_c_abc, (-1, 0, 1))):
            raise ValueError("Phase switching states must be -1, 0, or 1")
        return q_c_abc.astype(int)

    @staticmethod
    def _get_open_circuit_pole_voltages(inp: Any, u_dc: float) -> np.ndarray:
        """Get floating pole voltages and apply the diode rail clamps."""
        u_open_abc = complex2abc(inp.u_c_open_ab)
        # A three-wire load leaves the common-mode voltage free. Center the phase
        # voltages between the rails before applying the antiparallel-diode clamps.
        u_0 = 0.5 * (u_dc - np.max(u_open_abc) - np.min(u_open_abc))
        return np.clip(u_open_abc + u_0, 0.0, u_dc)

    def compute_output_voltage(self, inp: Any, u_dc: float | None = None) -> Any:
        """Compute terminal voltage including floating and diode-conduction states."""
        if u_dc is None:
            u_dc = self.out.u_dc
        q_c_abc = self._get_phase_states(inp.q_c_ab, inp.converter_mode)
        if q_c_abc is None:
            return inp.q_c_ab * u_dc

        i_c_abc = complex2abc(inp.i_c_ab)
        u_open_abc = self._get_open_circuit_pole_voltages(inp, u_dc)
        u_c_abc = q_c_abc.astype(float) * u_dc
        is_all_phase_open = q_c_abc == -1
        # Positive current flows out through the low-side diode and negative current
        # flows in through the high-side diode. At zero current, neither diode conducts
        # and the terminal floats at the load-induced open-circuit voltage.
        u_hiz_abc = np.where(
            i_c_abc > self.i_tol, 0.0, np.where(i_c_abc < -self.i_tol, u_dc, u_open_abc)
        )
        u_c_abc = np.where(is_all_phase_open, u_hiz_abc, u_c_abc)
        return abc2complex(u_c_abc)

    def compute_effective_switching_state(
        self, inp: Any, u_dc: float | None = None
    ) -> Any:
        """Compute effective switching vector, including floating pole voltages."""
        if u_dc is None:
            u_dc = self.out.u_dc
        if u_dc == 0:
            return 0j
        return self.compute_output_voltage(inp, u_dc) / u_dc

    def set_gate_signals(self, q_abc: np.ndarray, b_abc: np.ndarray) -> None:
        """Set the switching state based on the gate and blanking signals."""
        i_abc = complex2abc(self.inp.i_c_ab)
        self.inp.q_c_ab = abc2complex(q_abc + 0.5 * b_abc * (1 - self.sign(i_abc)))

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A), including diode conduction."""
        q_eff_ab = self.compute_effective_switching_state(inp)
        return 1.5 * np.real(q_eff_ab * np.conj(inp.i_c_ab))

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.u_c_ab = self.compute_output_voltage(self.inp)

    def meas_dc_voltage(self) -> float:
        """Measure converter DC-bus voltage (V)."""
        return self.out.u_dc

    def rhs(self, t: float) -> list[complex]:
        """Default empty implementation."""
        return []

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "VoltageSourceConverterTimeSeries"]:
        """Create time series."""
        return "converter", VoltageSourceConverterTimeSeries(t, self)


@dataclass
class VoltageSourceConverterTimeSeries[T: VoltageSourceConverter]:
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[T]
    u_dc: np.ndarray = field(default_factory=empty_array)
    q_c_ab: np.ndarray = field(default_factory=empty_array)
    converter_mode: np.ndarray = field(default_factory=empty_array)
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_c_open_ab: np.ndarray = field(default_factory=empty_array)
    q_eff_ab: np.ndarray = field(default_factory=empty_array)
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    i_dc_int: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        self.u_dc = np.full(np.size(t), subsystem.u_dc)

    def compute_zoh_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Resolve floating terminals, diode conduction, and derived signals."""
        if np.size(self.u_c_open_ab) != np.size(self.i_c_ab):
            self.u_c_open_ab = np.zeros_like(self.i_c_ab)
        self.u_c_ab = np.array(
            [
                subsystem.compute_output_voltage(
                    Inputs(
                        q_c_ab=q_c_ab,
                        converter_mode=converter_mode,
                        i_c_ab=i_c_ab,
                        u_c_open_ab=u_c_open_ab,
                    ),
                    u_dc,
                )
                for q_c_ab, converter_mode, i_c_ab, u_c_open_ab, u_dc in zip(
                    self.q_c_ab,
                    self.converter_mode,
                    self.i_c_ab,
                    self.u_c_open_ab,
                    self.u_dc,
                    strict=True,
                )
            ]
        )
        self.q_eff_ab = np.divide(
            self.u_c_ab, self.u_dc, out=np.zeros_like(self.u_c_ab), where=self.u_dc != 0
        )
        self.i_dc_int = 1.5 * np.real(self.q_eff_ab * np.conj(self.i_c_ab))

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Default empty implementation."""


# %%
@dataclass
class CapacitiveDCBusConverterStates:
    """State variables."""

    u_dc: float


@dataclass
class CapacitiveDCBusConverterStateHistory:
    """State history."""

    u_dc: list[complex] = field(default_factory=list)


class CapacitiveDCBusConverter(VoltageSourceConverter):
    """
    Lossless voltage-source converter with capacitive DC-bus dynamics.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V).
    C_dc : float
        DC-bus capacitance (F).
    t_d : float, optional
        Dead time (s), defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents (A) determining the leg states during blanking,
        defaults to `np.sign`.

    """

    def __init__(
        self,
        u_dc: float,
        C_dc: float,
        t_d: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
    ) -> None:
        super().__init__(u_dc, t_d, sign)
        self.C_dc = C_dc
        self.state: CapacitiveDCBusConverterStates = CapacitiveDCBusConverterStates(
            self.u_dc
        )
        self._history: CapacitiveDCBusConverterStateHistory = (
            CapacitiveDCBusConverterStateHistory()
        )

    def set_external_dc_current(self, i_dc: Callable[[float], float]) -> None:
        """Set external DC current (A)."""
        self.inp.i_dc = i_dc

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real
        super().set_outputs(t)

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives for DC-bus voltage."""
        if callable(self.inp.i_dc):
            i_dc = self.inp.i_dc(t)
        elif isinstance(self.inp.i_dc, (int, float)):
            i_dc = self.inp.i_dc
        else:
            i_dc = 0.0
        i_dc_int = self.compute_internal_dc_current(self.inp)
        d_u_dc = (i_dc - i_dc_int) / self.C_dc
        return [d_u_dc]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "CapacitiveDCBusConverterTimeSeries"]:
        """Create time series from state list."""
        return "converter", CapacitiveDCBusConverterTimeSeries(t, self)


@dataclass
class CapacitiveDCBusConverterTimeSeries(
    VoltageSourceConverterTimeSeries[CapacitiveDCBusConverter]
):
    """Continuous time series."""

    subsystem: InitVar[CapacitiveDCBusConverter]

    def __post_init__(self, t: np.ndarray, subsystem: CapacitiveDCBusConverter) -> None:
        self.u_dc = np.array(subsystem._history.u_dc)


# %%


@dataclass
class FrequencyConverterStates:
    """State variables."""

    u_dc: complex  # Imaginary part is always zero
    i_L: complex = 0j  # Imaginary part is always zero
    exp_j_theta_g: complex = complex(1)


@dataclass
class FrequencyConverterStateHistory:
    """State history."""

    u_dc: list[complex] = field(default_factory=list)
    i_L: list[complex] = field(default_factory=list)
    exp_j_theta_g: list[complex] = field(default_factory=list)


class FrequencyConverter(VoltageSourceConverter):
    """
    Frequency converter with a six-pulse diode bridge.

    A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The diode
    bridge is connected to the voltage-source inverter. The grid inductance is zero.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    L_dc : float
        DC-bus inductance (H).
    U_g : float
        Grid voltage (V, line-line, rms).
    f_g : float
        Grid frequency (Hz).
    t_d : float, optional
        Dead time (s), defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents (A) determining the leg states during blanking,
        defaults to `np.sign`.

    """

    def __init__(
        self,
        C_dc: float,
        L_dc: float,
        U_g: float,
        f_g: float,
        t_d: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
    ) -> None:
        u_dc = sqrt(2) * U_g
        super().__init__(u_dc, t_d, sign)
        self.C_dc = C_dc
        self.L_dc = L_dc
        self.w_g = 2 * np.pi * f_g
        self.u_g = sqrt(2 / 3) * U_g
        self.state: FrequencyConverterStates = FrequencyConverterStates(u_dc)
        self._history: FrequencyConverterStateHistory = FrequencyConverterStateHistory()

    def compute_voltages(self, state: Any) -> tuple[Any, Any]:
        """Compute grid and rectified voltages."""
        # Grid voltage
        u_g_ab = self.u_g * state.exp_j_theta_g
        u_g_abc = complex2abc(u_g_ab)
        # Output voltage of the diode bridge
        u_di = np.ptp(u_g_abc, axis=0)
        return u_g_ab, u_di

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real
        super().set_outputs(t)

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        # Rectified voltage and internal DC current
        _, u_di = self.compute_voltages(self.state)
        i_dc_int = self.compute_internal_dc_current(self.inp)
        # State derivatives
        d_u_dc = (self.state.i_L.real - i_dc_int) / self.C_dc
        d_i_L = (u_di - self.state.u_dc.real) / self.L_dc
        d_exp_j_theta_g = 1j * self.w_g * self.state.exp_j_theta_g
        # Inductor current cannot be negative due to the diode bridge
        if self.state.i_L.real < 0 and d_i_L < 0:
            d_i_L = 0
        return [d_u_dc, d_i_L, d_exp_j_theta_g]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "FrequencyConverterTimeSeries"]:
        """Time series."""
        return "converter", FrequencyConverterTimeSeries(t, self)


@dataclass
class FrequencyConverterTimeSeries(
    VoltageSourceConverterTimeSeries[FrequencyConverter]
):
    """Continuous time series."""

    subsystem: InitVar[FrequencyConverter]

    # State variables
    i_L: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    u_g_ab: np.ndarray = field(default_factory=empty_array)
    u_di: np.ndarray = field(default_factory=empty_array)
    u_g_abc: np.ndarray = field(default_factory=empty_array)
    q_g_abc: np.ndarray = field(default_factory=empty_array)
    i_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: FrequencyConverter) -> None:
        self.u_dc = np.real(np.array(subsystem._history.u_dc))
        self.i_L = np.real(np.array(subsystem._history.i_L))
        self.exp_j_theta_g = np.array(subsystem._history.exp_j_theta_g)
        self.u_g_ab, self.u_di = subsystem.compute_voltages(self)
        self.u_g_abc = complex2abc(self.u_g_ab)
        # Diode bridge switching states (-1, 0, 1)
        self.q_g_abc = (self.u_g_abc.max(axis=0) == self.u_g_abc).astype(int) - (
            self.u_g_abc.min(axis=0) == self.u_g_abc
        ).astype(int)
        # Grid current space vector
        self.i_g_ab = abc2complex(self.q_g_abc) * self.i_L
