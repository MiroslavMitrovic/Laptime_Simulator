""" Tire model class.



"""
from dataclasses import dataclass
from scripts.utilities import *
from typing import Callable
import math


@dataclass
class TireParams:
    mu0: float = 1.0            # base friction coefficient (dry asphalt ~1.0)
    Fz0: float = 4000.0         # reference vertical load [N]
    C_alpha0: float = 80000.0   # reference cornering stiffness [N/rad] at Fz0
    C_SR0: float = 100000.0  # reference longitudinal stiffness [N] at Fz0
    n_exp: float = 0.9          # load-sensitivity exponent for stiffness scaling
    eps_v: float = 0.1          # small velocity [m/s] to avoid singularities
    t0: float = 0.07            # base pneumatic trail [m] (for aligning moment Mz)
    p_trail: float = 1.5        # exponent controlling trail decay with slip angle
    R: float = 0.365            # Dynamic Rolling radius [m]


@dataclass
class TireForces:
    Fx: float = 0.0
    Fy: float = 0.0
    Mz: float = 0.0

class Tire:
    """Class that will represent a tire model of the vehicle."""

    def __int__(self):
        """Constructor."""

    def calculate_tire_forces(
            P: TireParams,
            Vx: float,
            Vy: float,
            omega: float,
            Fz: float,
            mu_fn: Callable[[float], float] = None
    ) -> TireForces:
        """
        Compute Tire forces for given states using the simple model.

        Args:
            P: TireParams
            Vx, Vy: contact patch velocities [m/s]
            omega: wheel angular speed [rad/s]
            Fz: normal load [N]
            mu_fn: optional function mu = mu_fn(Fz) to vary friction with load.
                   If None, uses constant P.mu0.

        Returns:
            TireForces(Fx, Fy, Mz)
        """

        # 1) slips
        denominator = max(max(abs(Vx), abs(P.R * omega)), P.eps_v)
        SR = (P.R * omega - Vx) / denominator
        alpha = math.atan2(Vy, abs(Vx) + P.eps_v)

        # 2) stiffness scaling with load (load sensitivity) based on reference values of load Fz0
        # Prevent negative/near-zero loads causing issues
        Fz_clamped = max(Fz, 1.0)
        s = (Fz_clamped / P.Fz0) ** P.n_exp
        C_alpha = P.C_alpha0 * s
        C_SR = P.C_SR0 * s

        # 3) Friction limit and pure-slip (smooth) forces
        mu = mu_fn(Fz_clamped) if mu_fn is not None else P.mu0
        F_lim = mu * Fz_clamped

        alpha_sat = F_lim / max(C_alpha, 1.0)
        SR_sat = F_lim / max(C_SR, 1.0)

        Fy0 = - C_alpha * smooth_sat(alpha, alpha_sat)  # sign convention
        Fx0 = C_SR * smooth_sat(SR, SR_sat)

        # 4) friction circle projection for combined slip
        mag0 = math.hypot(Fx0, Fy0)
        scale = F_lim / mag0 if mag0 > F_lim else 1.0
        Fx = Fx0 * scale
        Fy = Fy0 * scale

        # 5) simple aligning moment via pneumatic trail
        # trail decays with slip angle magnitude relative to alpha_sat
        # Mz positive when Fy tends to re-center (hence negative sign: Mz = -Fy * t_p)
        if alpha_sat <= 0.0:
            tp = 0.0
        else:
            tp = P.t0 / (1.0 + (abs(alpha) / alpha_sat) ** P.p_trail)
        Mz = - Fy * tp

        return TireForces(Fx=Fx, Fy=Fy, Mz=Mz)

