from constants import *

test_set = [  {        "name": "LDN Test",

    "initial_memory": [ (LDN<<5)+1,  # Load negative from location 1
                      0x02,
                      0x04,
                      0x08,
                      0x10,
                      0x20,
                      0x40,
                        0xFF],
    "expected_accumulator": 0xFE
    },
    {
        "name": "STO Test",
        "initial_memory": [ (STO<<5)+1, # Store to location 1
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 0x00,
        "memory_update": (1, 0),
        "expected_pc": 0x01
    },
    {
        "name": "SUB Test",
        "initial_accumulator": 76,
        "initial_memory": [ (SUB<<5)+4, # Subtract location 4
                            0x02,
                            0x04,
                            0x08,
                            22,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 54
    },
    {
        "name": "JMP Test",
        "initial_memory": [ (JMP<<5)+2, # Jump to the address in location 2 (0x4) which is 0x5 after the auto-increment.
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 0x00,
        "expected_pc": 0x05
    },
    {
        "name": "JRP Test",
        "initial_memory": [ (JRP<<5)+2, # Jump relative by the address in location 2 (0x4) which is 0x5 after the auto-increment.
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 0x00,
        "expected_pc": 0x05
    },
    {
        "name": "CMP Test",
        "initial_accumulator": 0xF0,
        "initial_memory": [ (CMP<<5)+6, # Skip next instruction if accumulator is negative. At the moment, CMP also acts as a sub so we must point it at a zero memory location.
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x00,
                            0xFF], 
        "expected_pc": 0x02
    },

    {
        "name": "Sequenced instruction test",
        "initial_accumulator": 0xF0,
        "initial_memory": [ (CMP<<5)+6, # Skip next instruction if accumulator is negative. At the moment, CMP also acts as a sub so we must point it at a zero memory location.
                            0x02,
                            (SUB<<5)+1,
                            0x08,
                            0x10,
                            0x20,
                            0x00,
                            0xFF], 
        "expected_accumulator": 0xEE,
        "expected_pc": 3,
        "cycles": 2
    },
              {
        "name": "SUB then LDN",
        "initial_accumulator": 0xF0,
        "initial_memory": [ (SUB<<5)+2, # Skip next instruction if accumulator is negative. At the moment, CMP also acts as a sub so we must point it at a zero memory location.
                            (LDN<<5)+3,
                            0x02,
                            0x08,
                            0x10,
                            0x20,
                            0x00,
                            0xFF], 
        "expected_accumulator": 0xF8,
        "expected_pc": 2,
        "cycles": 2
    }


              
 ]
