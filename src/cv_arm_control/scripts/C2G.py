#!/usr/bin/env python3

import numpy as np

C2G = np.array(
    [
        [0.0, -1.0, 0.0, 0.065],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

__all__ = ["C2G"]
