#!/usr/bin/env python

import logging
import random

from constants import *

# SSEM Emulator module. This is meant to emulate the SSEM, to provide tests for the Box2D version.

logging.basicConfig(level=logging.INFO)

def twos_comp(num):
    if num < 0:
        return (1<<memory_columns)+num
    return num

class SSEM_State():
    def __init__(self):
        self.pc = 0
        self.mem = [0] * memory_rows
        self.accumulator = 0
    def advance(self):
        logging.info("Initial PC == {}".format(self.pc))
        instruction = self.mem[self.pc]
        if memory_columns == 32:
            instruction_address = instruction & ((1 << 13)-1)
            instruction_op = (instruction >> 13) & 7
        elif memory_columns == 8:
            instruction_address = instruction & ((1<<5)-1)
            instruction_op = (instruction >> 5) & 7
        else:
            raise Exception("Memory widths other than 8 or 32 are not supported.")
        logging.info("Executing {} on address {} (wrapped to {})".format(instruction_opcodes[instruction_op], instruction_address, instruction_address % memory_rows))

        if instruction_op == SB2 or instruction_op == HLT:
            logging.warn("Executing unsupported {} opcode.".format(instruction_opcodes[instruction_op]))

        if instruction_op == JMP:
            self.pc = self.mem[instruction_address % memory_rows]
        elif instruction_op == JRP:
            self.pc += self.mem[instruction_address % memory_rows]
            self.pc %= memory_rows
        elif instruction_op == LDN:
            self.accumulator = twos_comp(-self.mem[instruction_address % memory_rows])
        elif instruction_op == STO:
            self.mem[instruction_address % memory_rows] = self.accumulator
        elif instruction_op == SUB or instruction_op == SB2:
            self.accumulator = twos_comp(self.accumulator - self.mem[instruction_address % memory_rows])
        elif instruction_op == CMP:
            mem_value = self.mem[instruction_address % memory_rows]
            if mem_value != 0:
                logging.warn("Executing CMP with non-zero operand")
        elif instruction_op == HLT:
            # Rather than do anything special, this just prevents the
            # advancing of PCs, so it just executes this operation over and over.
            # Equivalent to JRP with an argument of -1.
            self.pc -= 1
        self.pc += 1
        self.pc %= memory_rows
    def __str__(self):
        return "IP:%2.2X ACC:%8.8X  %s"%(self.pc, twos_comp(self.accumulator), " ".join("%8.8X"%twos_comp(x) for x in self.mem))

def main():
    startstate = SSEM_State()
    # Put the machine in a random state and run one cycle
    startstate.pc = random.randint(0,memory_rows-1)
    startstate.mem = [random.randint(0,255)+i for i in range(0,memory_rows)]
    startstate.accumulator = random.randint(0,(1<<memory_columns)-1)
    print(startstate)
    startstate.advance()
    print(startstate)
    
if __name__=="__main__":
    main()
