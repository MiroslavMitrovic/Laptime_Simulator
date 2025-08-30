import pytest
from scripts.tire_model import *



def test_tyre_forces_longitudinal_velocity():
    """Unit testing the calculation of Forces based on Velocity"""
    tire_data = TireParams()
    tire_forces = TireForces()
    Vx = 50.0
    Vy = 0.0
    omega = 100
    Fz = 2500
    in_data = [Vx, Vy, omega, Fz]
    tire_forces = Tire.calculate_tire_forces(tire_data,in_data[0], in_data[1],
                                             in_data[2],in_data[3])

    assert (tire_forces.Fx, tire_forces.Fy, tire_forces.Mz) == (-2475.3946348235054, 0.0, 0.0)


def test_tyre_forces_longitudinal_and_lateral_velocity():
    """Unit testing the calculation of Forces based on Velocity"""
    tire_data = TireParams()
    tire_forces = TireForces()
    Vx = 50.0
    Vy = 20.0
    omega = 250
    Fz = 2500
    in_data = [Vx, Vy, omega, Fz]
    tire_forces = Tire.calculate_tire_forces(tire_data,in_data[0], in_data[1],
                                             in_data[2],in_data[3])

    assert (tire_forces.Fx, tire_forces.Fy, tire_forces.Mz) == (1771.5414484799946, -1763.9843809731997,
                                                                5.262030143545877)