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


        memory_fixed_shapes = []
        for row in range(0,8):
            for col in range(0,8):
                memory_fixed_shapes.append(makeBox(22*col,14*row,14,7))
                memory_fixed_shapes.append(makeBox(22*col+7,14*row,3,14))
        groundBox = makeBox(-20,-10,40,1)

        groundBody = self.world.CreateStaticBody(shapes=groundBox)
        memory_fixed = self.world.CreateStaticBody(shapes = memory_fixed_shapes)
        bodyA = self.world.CreateDynamicBody(
            position=(-10, 0),
            fixtures=[fixtureDef(
                shape=circleShape(radius=6.35/2, pos=(22,150)),
                density=1.0,
                filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))]

        )

        row_injector_fixtures = []
        for col in range(0,8):
            row_injector_fixtures.append(fixtureDef(shape=makeBox(3+22*col,0,3,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=makeBox(22*8+7,0,7,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
        injectors=[]
        for col in range(0,8):
            injectors.append(self.world.CreateDynamicBody(position=(0, 7+14*col), fixtures=row_injector_fixtures))
            self.world.CreatePrismaticJoint(
                bodyA=injectors[-1], 
                bodyB=groundBody,
                anchor=groundBody.worldCenter,
                axis=(1, 0),
                lowerTranslation=7,
                upperTranslation=21,
                enableLimit=True
            )
                            
                             
        row_ejector_fixtures = []
        for col in range(0,8):
            row_ejector_fixtures.append(fixtureDef(shape=makeBox(22*col,0,14,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_ejector_fixtures.append(fixtureDef(shape=makeBox(22*8+7,3,3,13), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        ejectors = []
        for col in range(0,8):
            ejector = self.world.CreateDynamicBody(
                position=(0, 14*col),
                fixtures=row_ejector_fixtures,
            )
            pj2 = self.world.CreatePrismaticJoint(
                bodyA=ejector, 
                bodyB=groundBody,
                anchor=groundBody.worldCenter,
                axis=(1, 0),
                maxMotorForce=100.0,
                motorSpeed=0.0,
                enableMotor=True,
                lowerTranslation=0,
                upperTranslation=10,
                enableLimit=True,

            )

    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

