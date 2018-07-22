#!/usr/bin/env python
#
"""
Very simple headless framework for PyBox2D - meant for automatic testing
"""

from framework import FrameworkBase

class HeadlessFramework(FrameworkBase):
    def run(self):
        """
        Main loop.

        Runs until 'stopFlag' is set.
        """

        while not self.stopFlag:
            self.SimulationLoop()

        self.world.contactListener = None
        self.world.destructionListener = None
        self.world.renderer = None

    def Print(self, str, color=(0,0,0,0)):
        """
        Draw some text at the top status lines
        and advance to the next line.
        """
        pass

