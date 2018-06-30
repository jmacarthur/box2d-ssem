from constants import *

test_set = [  {
    # At the moment, PC starts at 0xFF.
    "initial_memory": [ 0xFF,
                      0x02,
                      0x04,
                      0x08,
                      0x10,
                      0x20,
                      0x40,
                        (LDN<<5)+1], # Load from location 1
    "expected_accumulator": 0x02
    },
    {
    # At the moment, PC starts at 0xFF.
        "initial_memory": [ 0xFF,
                            0x02,
                            0x04,
                            0x08,
                            0x10,
                            0x20,
                            0x40,
                            (STO<<5)+1], # Store to  location 1
        "expected_accumulator": 0x00,
        "memory_update": (1, 0)
    }
    
 ]
