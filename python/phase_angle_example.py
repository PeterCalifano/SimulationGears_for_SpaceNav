# pip install spiceypy numpy
import spiceypy as sp
import numpy as np
from pathlib import Path
import os
from contextlib import contextmanager

@contextmanager
def cd(path: Path):
    prev = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)

# --- Inputs ---
kernel_path = Path("/home/peterc/devDir/projects-DART/data/future/phase-C/kernels/mk/kernels.mk");

# --- Load kernels ---
with cd(kernel_path.parent):
    sp.furnsh(str(kernel_path))

# --- Epoch ---
et = sp.str2et("28 August 2025 12:00 UTC")

# --- Vectors in J2000, no aberration, observer EARTH ---
moon_pos_eci, _ = sp.spkpos("MOON", et, "J2000", "NONE", "EARTH")
sun_pos_eci,  _ = sp.spkpos("SUN",  et, "J2000", "NONE", "EARTH")

# --- Unit vectors ---
moon_u = moon_pos_eci / np.linalg.norm(moon_pos_eci)
sun_u  = sun_pos_eci  / np.linalg.norm(sun_pos_eci)

# --- Angle between unit vectors (deg) ---
# Clip for numerical safety
dot_us = float(np.clip(np.dot(sun_u, moon_u), -1.0, 1.0))
aux_pa_deg = np.degrees(np.arccos(dot_us))

# Your MATLAB computes PA as 180 - angle(Sun, Moon) as seen from Earth
pa_moon_deg = 180.0 - aux_pa_deg

print(f"Angle [deg] {pa_moon_deg:0.4f}")

# Unload kernels when done
sp.kclear()
