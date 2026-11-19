# -*- coding: utf-8 -*-
"""Thermohygrometric Psychrometry (Magnus-Tetens) & Speed of Sound Compensation.

Corrige os bugs documentados de sensor_de_umidade_do_ar_caseiro (.ino L036 typo 83->92 e L132 intervalo vazio).
"""

import math


def saturation_vapor_pressure_hpa(temp_c: float) -> float:
    """Pressao de vapor saturado e_s(T) em hPa pela formula de Magnus-Tetens."""
    return 6.112 * math.exp((17.67 * temp_c) / (temp_c + 243.5))


def relative_humidity_magnus(t_dry_c: float, t_wet_c: float, pressure_hpa: float = 1013.25) -> float:
    """Calcula umidade relativa (%) contínua sem descontinuidades de tabela.

    Args:
        t_dry_c: Temperatura bulbo seco (C).
        t_wet_c: Temperatura bulbo umido (C), tipicamente <= t_dry_c.
        pressure_hpa: Pressao barometrica local (padrao 1013.25 hPa).
    """
    t_wet_eff = min(t_dry_c, t_wet_c)
    gamma_psych = 0.000662 * pressure_hpa
    e_s_dry = saturation_vapor_pressure_hpa(t_dry_c)
    e_s_wet = saturation_vapor_pressure_hpa(t_wet_eff)
    e_vapor = e_s_wet - gamma_psych * (t_dry_c - t_wet_eff)
    rh = 100.0 * (e_vapor / max(e_s_dry, 1e-6))
    return max(0.0, min(100.0, rh))


def speed_of_sound_mps(temp_c: float, rh_pct: float = 50.0, pressure_hpa: float = 1013.25) -> float:
    """Velocidade do som compensada c(T, RH) em m/s para HC-SR04."""
    t_k = max(-40.0, temp_c)
    rh = max(0.0, min(100.0, rh_pct))
    e_s = saturation_vapor_pressure_hpa(t_k)
    vapor_fraction = (rh / 100.0) * (e_s / max(pressure_hpa, 1.0))
    c_dry = 331.3 * math.sqrt(1.0 + t_k / 273.15)
    return float(c_dry * (1.0 + 0.0016 * vapor_fraction))
