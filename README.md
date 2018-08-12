This is an implementation of the Manchester Small-scale experimental machine (SSEM or 'Baby') in Box2D. There's no practical reason for doing this, it's purely for fun.

Skip to 'Requirements' or 'Usage' if you want to try this out.

Overview
========
The SSEM was the first stored-program electronic computer to run, which it first did in 1948. It was extremely simple, consisting of a 32x32 bit memory, a subtractor and the minimum necessary control logic. This simplicity makes it a good design if you want to implement a computer in unusual media.

Ball bearings represent data; a ball bearing repesents a '1' and the absence of one is a zero. The memory unit holds a binary pattern for each word. To read memory, a whole row is ejected from memory and each bearing drops into a separate channel. Falling ball bearings are diverted to different parts of the machine, such as the subtractor, the instruction register or the program counter. Since reading the value out of memory removed it from memory, it's necessary to copy the data and write back in most cases. When the ball bearings pass a certain point, they trigger a release of the same pattern of bearings from a hopper at the top of the machine, which fall back into the memory.

The first memory read is to the instruction register, and this feeds two decoders - a 3-to-8 decoder for the memory (which will be a 5-to-32 decoder in the finished version) and a 3-to-8 decoder for the instruction. The instruction decoder drops one connecting rod relevant to the instruction, so only one action will take place on a given cycle. For example, although the cam which resets the accumulator turns on every cycle, it will only be connected to the reset bar if LDN is selected.

Near the base of the machine are the subtractors. Dropping a bearing down one channel will toggle the bit at the bottom and the bearing will fall out to the left or right. If it falls left, it's considered a carry and acts as a subtraction on the next most significant bit. Dropping a bearing into the rightmost column when the subtractor is already storing '0' will toggle every bit in sequence. Since the subtractors maintain state, they function as accumulators too. The main subtractor, on the left, is the arithmetic unit. The smaller one on the right is the program counter.

Subtractors also have an output path which diverts falling ball bearings without changing the stored data. This is used to read the value in the accumulator for a STO operation; a mechanism similar to the memory 'regenerator' sends mechanical signals to the hopper to inject the right pattern of data into the memory.

Cheats
======
There are two pieces of 'magic' - bits necessary for operation which don't follow the normal laws of physics Box2D implements. First, there are two layers in the project, analagous to a second Box2D world behind the main one, which is necessary for some data paths to cross over. There are several invisible regions which will flip a bearing to the other layer if the ball goes into one. If this were a physical machine, that region would just be a hole between the layers and a small ramp.

Secondly, any ball bearings falling out of the bottom of the machine are teleported back to the hopper at the top. This could be implemented as a physical lift or pump in Box2D physics, but it doesn't add much to the effect and costs a lot of computing time.

Limitations
===========
The current implementation only has an 8x8 bit memory cell, although the design is quite easily extendable to a 32x32 version. The SSEM's instruction format only uses 8 bits, of which 3 are the operation code and 5 are the address, so this fits into the 8 bit version.

The CMP (compare) instruction requires that you use an address which contains zero. This isn't a significant restriction, but requires that we dedicate one memory location to hold zero at all times, and means we can't directly run software meant for the original SSEM, so I expect to fix this soon.

HLT (halt) is unimplemented at the moment. 'JRP [31]' will put the processor into an infinite loop on one instruction, which will effectively halt the machine.

Speed is of course much slower than the original; running one instruction takes about 20 seconds without the GUI or 180 seconds with it. The original SSEM took about 1 millisecond. Since the entire point of the project is to provide a visual spectacle rather than to crunch numbers, this isn't a big problem.

Requirements
============

You'll need:

* PyBox2D, from https://github.com/pybox2d/pybox2d.git
* swig (a Debian/Ubuntu package)
* Pygame if you want to run the graphical front end.

swig and pygame can be installed on a Debian-style system with 'apt install swig' and 'apt install python3-pygame' respectively.

To run 'main.py', you'll need to put the PyBox2d framework module onto the path, for example:

    export PYTHONPATH=/home/jimm/apps/pybox2d/examples

You'll also need to modify some of the relative imports inside PyBox2D. Since we're using the 'framework' module outside of the main directory, we need to modify the source to remove the relative imports, e.g. in framework.py:

    from .settings import fwSettings

needs to be changed to:

    from settings import fwSettings

In future we should find a better way of doing this import, but this works for the time being.

Usage
=====

Command line options are in flux. The following will work at the time of writing:

Start the system with default configuration:

    ./main.py

Run automated test number 3 and exit when complete:

    ./main.py --test 3

Run a randomly-generated test with seed 17 and exit when complete:

    ./main.py --randomtest 17

Run randomly-generated test with seed 33, without the graphical front end:

    ./main.py 0 --randomtest 33 --headless

Once running, you can pan around with the arrow keys or by dragging with the right mouse button. You can zoom in by pressing 'Z' and out by pressing 'X'.

If you're not running an automated test, press 'R' to start the cams which run the computer. When running with --test or --randomtest, the machine starts automatically.

Clicking on a moving part with the left mouse button and dragging allows you to apply a force to that part which can be used to move ball bearings and levers around. Naturally, doing this means the result of any tests are invalid.