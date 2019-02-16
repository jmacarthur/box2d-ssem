# Cam definition

from collections import namedtuple

Cam = namedtuple("Cam", field_names = ["xpos", "ypos", "steps", "offset", "signal_name"])

cams = [
    Cam(300, 200, [(0.0,0.02)], 1, "PC INJECTOR"),
]

