"""A class containing the utility functions.



"""
import math

def smooth_sat(x: float, x_sat: float) -> float:
    """
    Smooth saturation: y = x / sqrt(1 + (x/x_sat)^2).
    For |x| << x_sat -> y ≈ x (linear); for |x| >> x_sat -> y -> sign(x) * x_sat.

    Args:
            x: current value
            x_sat: saturated value

    Returns:
            smoothed value
    """
    # Guard against zero/negative saturation
    x_sat = max(x_sat, 1e-12)
    return x / math.sqrt(1.0 + (x / x_sat) * (x / x_sat))

