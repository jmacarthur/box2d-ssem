# Cam definition

import namedtuple

Cam =namedtuple(["xpos", "ypos", "steps", "offset", "signal_name"])

cams = [
    Cam(300,200, 100, [(0.0,0.02)], 1, "PC INJECTOR"),
]
    
