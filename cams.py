# Cam definition

from collections import namedtuple

_Cam = namedtuple("_Cam", field_names = ["xpos", "ypos", "steps", "offset", "signal_name", "horizontal", "reverse_direction", "bump_height", "follower"])

class Cam(_Cam):
    def __new__(_cls, xpos, ypos, steps, offset, signal_name, horizontal=False, reverse_direction=False, bump_height=3, follower=True):
        'Create new instance of Point(x, y)'
        return _Cam.__new__(_cls, xpos, ypos, steps, offset, signal_name, horizontal, reverse_direction, bump_height, follower)

instruction_ready_point = 0.50 # Instruction decoder should be set up, ready for cams to use

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

    # Cam 9(?): LDN Trigger.
    Cam(850, 0, [(instruction_ready_point,0.05)], 0, "LDN TRIGGER", horizontal=True, reverse_direction=False, bump_height=4),

    # Cam 11(?): Instruction follower holdoff (horizontal)
    Cam(1000, 100, [(0.02, 0.2), (0.15,0.25)], -1, "IP OUTPUT HOLDOFF", horizontal=True),

    # Cam 12: Fires main memory injector, injecting all 8 columns. If STO is on, this diverts to the subtractor reader. If not, it
    # will fall through the memory and be discarded.
    Cam(0, 300, [(0.62,0.02)], 1, "MAIN INJECTOR"),
        # Cam 13: Divert to subtractor reader on STO.
        # Also diverts the regenerator output on STO; we must separately discard that.
    Cam(1000, 0, [(0.49,0.2)], 2, "STO TRIGGER", horizontal=True, reverse_direction=True, bump_height=3.5),
    # Cam 14: Divert to instruction pointer, on JRP (and JMP via the same lever).
    # Cam pattern *nearly* identical to #13, please adjust to see if it works
    Cam(1100, 0, [(0.5,0.2)], 2, "JRP TRIGGER", horizontal=True, reverse_direction=True),

    # Cam 15: Secondary discard, of any data falling through the memory just after main inject
    Cam(-500,-150, [(0.67,0.07)], 2, "DISCARD 2", reverse_direction=True, horizontal=True),
        # Cam 16: Fires bottom regenerator (usually empty, unless STO is on)
    Cam(-500,-300, [(0.87,0.02)], 0, "LOWER REGEN CONTROL", horizontal=True),

        # Cam 17: Reset PC on JMP
    Cam(1230, 0, [(0.5,0.1)], 2, "JMP TRIGGER", horizontal=True, reverse_direction=True),

    # Cam 18: Runs CMP.
    # Cam pattern is identical to #9.
    Cam(900,200, [(instruction_ready_point,0.05)], -1, "CMP TRIGGER", horizontal=True, reverse_direction=False),
    # Cam 19: Inc PC.
    Cam(-95,-450, [(0.85,0.05)], 0, "INC PC", horizontal=True, reverse_direction=False, bump_height=5, follower=False)


]


