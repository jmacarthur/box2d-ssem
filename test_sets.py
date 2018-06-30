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
                      (LDN<<5)+1] # Store acc to memory location 1
    }
 ]
