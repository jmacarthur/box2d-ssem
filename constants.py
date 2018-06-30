from Box2D.b2 import filter

bb_diameter = 6.35

selector_rods = 3
memory_rows = 1<<selector_rods
pitch = 22
follower_spacing = 14

# Instruction format for the 8-bit machine: least significant 5 bits are address; top 3 bits are instruction.
# Instructions are:
JMP = 0
JRP = 1
LDN = 2
STO = 3
SUB = 4
CMP = 6
STOP = 7

bar_gate_raisers = False

filters = [filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF),
           filter(groupIndex=2, categoryBits=0x0002, maskBits=0x0000),
           filter(groupIndex=3, categoryBits=0x0004, maskBits=0x0000)]

