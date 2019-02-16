# Cam definition

from collections import namedtuple

_Cam = namedtuple("_Cam", field_names = ["xpos", "ypos", "steps", "offset", "signal_name", "horizontal", "reverse_direction", "bump_height"])

class Cam(_Cam):
    def __new__(_cls, xpos, ypos, steps, offset, signal_name, horizontal=False, reverse_direction=False, bump_height=3):
        'Create new instance of Point(x, y)'
        return _Cam.__new__(_cls, xpos, ypos, steps, offset, signal_name, horizontal, reverse_direction, bump_height)

cams = [
    Cam(300, 200, [(0.0,0.02)], 1, "PC INJECTOR", horizontal=False),
    Cam(150,300, [(0.05,0.07), (0.32, 0.06)], 0, "MEMORY DECODER INPUT HOLDOFF", horizontal=False),
    Cam(-400, 120, [(0.02, 0.07), (0.31,0.1), (0.64,0.1), (0.95,0.03)], -1, "MEMORY RETURN", horizontal=True),
    Cam(-300,100, [(0.03,0.11), (0.17,0.05), (0.31,0.1), (0.48,0.05), (0.65,0.1), (0.96,0.03)], -1, "MEMORY DECODER OUTPUT HOLDOFF", horizontal=True),

    # Cam 8: Sender eject.
    # Note timing hazard. We cannot raise selector and eject until
    # regenerated data is written back, so we delay for a few
    # seconds here.  If gravity or timing changes, expect this to
    # break.
    Cam(600, -430, [(0.02, 0.04), (0.30,0.04)], 0, "SENDER EJECT", horizontal=True),
    # Cam 5: Regenerator 1
    Cam(800, 100, [(0.24,0.05), (0.56,0.05)], 0, "UPPER REGEN CONTROL", horizontal=True),

    # Cam 6: Split to instruction register
    Cam(400,-120, [(0.18, 0.12)], 2, "TO INSTRUCTION REGISTER", horizontal=True, reverse_direction=True, bump_height=4),

    # Cam 7: Instruction selector holdoff (vertical)
    Cam(320, 300, [(0.04, 0.2), (0.32,0.06)], 0, "INSTRUCTION OUTPUT HOLDOFF"),



    
]

