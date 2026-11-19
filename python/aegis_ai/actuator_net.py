# -*- coding: utf-8 -*-
"""Neural Actuator Models & Multi-DOF Kinematics (DRV8871 Unified)."""

from .aegis_sim.actuators import (
    DCMotorModel,
    DriveBamActuator,
    ActuatorNetMLP,
    ActuatorNetLSTM,
    fit_actuator_mlp_from_csv,
)
from .aegis_sim.kinematics import (
    Kinematics2DOF,
    StewartPlatform6DOF,
    StewartPlatform6DOFV3,
)

__all__ = [
    "DCMotorModel",
    "DriveBamActuator",
    "ActuatorNetMLP",
    "ActuatorNetLSTM",
    "fit_actuator_mlp_from_csv",
    "Kinematics2DOF",
    "StewartPlatform6DOF",
    "StewartPlatform6DOFV3",
]
