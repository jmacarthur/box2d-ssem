#!/usr/bin/env python3

from framework import (Framework, main)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)

def makeBox(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x, y), 0))


# Circle example
#shape=circleShape(radius=6.35,pos=(0,10)),

class Memory (Framework):
    name = ""

    def __init__(self):
        super(Memory, self).__init__()


        fixed_shapes = []
        for row in range(0,8):
            fixed_shapes.append(makeBox(0,21*row,14,7))
       
        ground = self.world.CreateStaticBody(shapes=fixed_shapes)

        bodyA = self.world.CreateDynamicBody(
            position=(-10, 0),
            fixtures=[fixtureDef(
                shape=circleShape(radius=6.35, pos=(0,100)),
                density=1.0,
                filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))]

        )

        row_injector_fixtures = []
        for col in range(0,8):
            row_injector_fixtures.append(fixtureDef(shape=makeBox(21*col,0,7,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
        injector = self.world.CreateDynamicBody(
            position=(-10, 0),
            fixtures=row_injector_fixtures,

        )

        
    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

