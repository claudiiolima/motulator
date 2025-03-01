# pylint: disable=C0103
"""
This script configures sensorless vector control for an synchronous motor.

"""
# %%
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from control.common import SpeedCtrl
from control.sm.vector import CurrentRef, CurrentCtrl, SensorlessObserver
from control.sm.vector import SensorlessVectorCtrl, Datalogger
from control.sm.opt_refs import OptimalLoci
from helpers import LUT, Sequence  # , Step
from config.mdl_syrm_7kW import mdl


# %%
@dataclass
class BaseValues:
    """
    This data class contains the base values computed from the rated values.
    These are used for plotting the results.

    """
    # pylint: disable=too-many-instance-attributes
    w: float = 2*np.pi*105.8
    i: float = np.sqrt(2)*15.5
    u: float = np.sqrt(2/3)*370
    p: int = 2
    psi: float = u/w
    P: float = 1.5*u*i
    Z: float = u/i
    L: float = Z/w
    tau: float = p*P/w


# %% Define the controller parameters
@dataclass
class CtrlParameters:
    """
    This data class contains parameters for the control system of a 6.7-kW
    synchronous relutance motor.

    """
    # pylint: disable=too-many-instance-attributes
    # Sampling period
    T_s: float = 250e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*100
    alpha_fw: float = 2*np.pi*20
    alpha_s: float = 2*np.pi*4
    # Observer
    w_o: float = 2*np.pi*40
    # Maximum values
    tau_M_max: float = 2*20.1
    i_s_max: float = 2*np.sqrt(2)*15.5
    i_sd_min: float = .25*np.sqrt(2)*15.5
    # Nominal values
    u_dc_nom: float = 540
    w_nom: float = 2*np.pi*105.8
    # Motor parameter estimates
    R_s: float = 0.54
    L_d: float = 41.5e-3
    L_q: float = 6.2e-3
    psi_f: float = 0
    p: int = 2
    J: float = .015


# %% Optimal references
base = BaseValues()
pars = CtrlParameters()
opt_refs = OptimalLoci(pars)
i_s_mtpa = opt_refs.mtpa(2*pars.i_s_max)
tau_M_mtpa = opt_refs.torque(i_s_mtpa)
pars.i_sd_mtpa = LUT(tau_M_mtpa, i_s_mtpa.real)
i_s_mtpv = opt_refs.mtpv(2*pars.i_s_max)
pars.i_sq_mtpv = LUT(i_s_mtpv.real, i_s_mtpv.imag)
# Plot the control loci
opt_refs.plot(2*pars.i_s_max, base)

# %% Choose controller
speed_ctrl = SpeedCtrl(pars)
current_ref = CurrentRef(pars)
current_ctrl = CurrentCtrl(pars)
observer = SensorlessObserver(pars)
datalog = Datalogger()
ctrl = SensorlessVectorCtrl(pars, speed_ctrl, current_ref,
                            current_ctrl, observer, datalog)

# %% Profiles
# Speed reference
times = np.array([0, .5, 1, 1.5, 2, 2.5,  3, 3.5, 4])
values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*base.w
mdl.speed_ref = Sequence(times, values)
# External load torque
times = np.array([0, .5, .5, 3.5, 3.5, 4])
values = np.array([0, 0, 1, 1, 0, 0])*20.1
mdl.mech.tau_L_ext = Sequence(times, values)  # tau_L_ext = Step(1, 14.6)
# Stop time of the simulation
mdl.t_stop = mdl.speed_ref.times[-1]

# %% Print the control system data
print('\nSensorless vector control')
print('-------------------------')
print('Sampling period:')
print('    T_s={}'.format(pars.T_s))
print('Motor parameter estimates:')
print(('    p={}  R_s={}  L_d={}  L_q={}'
       ).format(pars.p, pars.R_s, pars.L_d, pars.L_q))
print(current_ref)
print(current_ctrl)
print(observer)
print(speed_ctrl)

# %% Print the profiles
print('\nProfiles')
print('--------')
print('Speed reference:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.speed_ref))
print('External load torque:')
with np.printoptions(precision=1, suppress=True):
    print('    {}'.format(mdl.mech.tau_L_ext))
