#!/usr/bin/env python3

PACKAGE_PARENT=".."

from framework import (Framework, main)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape)

class Memory (Framework):
    name = ""

    def __init__(self):
        super(Memory, self).__init__()

        ground = self.world.CreateStaticBody(
            shapes=[edgeShape(vertices=[(-40, 0), (40, 0)])]
        )

        bodyA = self.world.CreateDynamicBody(
            position=(-10, 10),
            fixtures=fixtureDef(shape=polygonShape(vertices = [(-10,10), (10,10), (10,-10)]), density=5.0),
        )

    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

