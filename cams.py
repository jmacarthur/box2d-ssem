# Cam definition

from collections import namedtuple

Cam = namedtuple("Cam", field_names = ["xpos", "ypos", "steps", "offset", "signal_name", "horizontal"])

cams = [
    Cam(300, 200, [(0.0,0.02)], 1, "PC INJECTOR", horizontal=False),
    Cam(150,300, [(0.05,0.07)], 0, "MEMORY DECODER INPUT HOLDOFF", horizontal=False),
    Cam(-400, 120, [(0.02, 0.07), (0.31,0.1), (0.64,0.1), (0.95,0.03)], -1, "MEMORY RETURN", horizontal=True),
    Cam(-300,100, [(0.03,0.11), (0.17,0.05), (0.31,0.1), (0.48,0.05), (0.65,0.1), (0.96,0.03)], -1, "MEMORY DECODER OUTPUT HOLDOFF", horizontal=True)
]

