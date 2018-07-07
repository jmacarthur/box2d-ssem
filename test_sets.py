from constants import *

test_set = [  {
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
        "initial_memory": [ (JMP<<5)+2, # Jump to the address in location 2 (0x4)
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 0x00,
        "expected_pc": 0x01
    },
    {
        "initial_memory": [ (JRP<<5)+2, # Jump relative by the address in location 2 (0x4)
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            0xFF], 
        "expected_accumulator": 0x00,
        "expected_pc": 0x01
    },
    {
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
    }
    
 ]
