#!/usr/bin/env python3

from framework import (Framework, main)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)

def makeBox(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x+width/2, y+height/2), 0))


# Circle example
#shape=circleShape(radius=6.35,pos=(0,10)),

selector_rods = 3
memory_rows = 1<<selector_rods

class Memory (Framework):
    name = ""


    def slide_joint(self, body1, body2, axis, limit1, limit2):
        return self.world.CreatePrismaticJoint(
            bodyA=body1, 
            bodyB=body2,
            anchor=body2.worldCenter,
            axis=axis,
            maxMotorForce=10000.0,
            enableMotor=True,
            lowerTranslation=limit1,
            upperTranslation=limit2,
            enableLimit=True,
            )
        
    
    def __init__(self):
        super(Memory, self).__init__()


        memory_fixed_shapes = []
        for row in range(0,8):
            for col in range(0,8):
                memory_fixed_shapes.append(makeBox(22*col,14*row,14,7))
                memory_fixed_shapes.append(makeBox(22*col+14-3+1,14*row+6,3,8))
        groundBox = makeBox(-20,-10,40,1)

        groundBody = self.world.CreateStaticBody(shapes=groundBox)
        memory_fixed = self.world.CreateStaticBody(shapes=memory_fixed_shapes)
        
        test_data = self.world.CreateDynamicBody(
            position=(-10, 0),
            fixtures=[fixtureDef(
                shape=circleShape(radius=6.35/2, pos=(22,150)),
                density=1.0,
                filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))]

        )

        row_injector_fixtures = []
        for col in range(0,8):
            row_injector_fixtures.append(fixtureDef(shape=makeBox(7+22*col,0,3,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=makeBox(22*8+12,0,7,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
        injectors=[]
        for col in range(0,8):
            injector = self.world.CreateDynamicBody(position=(0, 7+14*col), fixtures=row_injector_fixtures)
            injectors.append(injector)
            self.slide_joint(injector, groundBody, (1,0), 7, 17)
                             
        row_ejector_fixtures = []
        for col in range(0,8):
            row_ejector_fixtures.append(fixtureDef(shape=makeBox(22*col,0,14,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_ejector_fixtures.append(fixtureDef(shape=makeBox(22*8+14,0,3,13), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        ejectors = []
        for col in range(0,8):
            ejector = self.world.CreateDynamicBody(
                position=(0, 14*col),
                fixtures=row_ejector_fixtures,
            )
            ejectors.append(ejector)
            self.slide_joint(ejector, groundBody, (1,0), 0, 14)


        for selector_no in range(0,selector_rods):
            row_selector_fixtures = []
            for row in range(0,8):
                enabled = (row >> selector_no) & 1
                row_selector_fixtures.append(
                    fixtureDef(shape=makeBox(14+25*selector_no,14*row+7*enabled,14,7),             density=1.0,
                               filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))
                )
            
            row_selector = self.world.CreateDynamicBody(
                position=(200, 0),
                fixtures=row_selector_fixtures,
            )
            self.slide_joint(row_selector, groundBody, (0,1), -7, 0)

        # Row followrs
        followers = []
        for row in range(0,8):
            row_follower_fixtures = []
            for selector_no in range(0,selector_rods+1):
                row_follower_fixtures.append(fixtureDef(shape=circleShape(radius=6.35/2, pos=(selector_no*25+35,14*row+10)), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
            follower = self.world.CreateDynamicBody(
                position=(200, 0),
                fixtures=row_follower_fixtures)
            followers.append(follower)
            self.slide_joint(follower, groundBody, (1,0), limit1=0, limit2=20)

        row_holdoff = self.world.CreateDynamicBody(
            position=(200,0),
            fixtures = fixtureDef(shape=makeBox(14+25*selector_rods,0,3,14*memory_rows+20), density=1.0,
                                  filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))
            )
        row_holdoff_supports = []
        for i in range(0,2):
            pos = (14+25*selector_rods,(14*row)*i)
            support = self.world.CreateDynamicBody( position=(200,0),
                                                    fixtures = fixtureDef(shape=makeBox(pos[0], pos[1],3,30), density=1.0,
                                                                          filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF)))
            row_holdoff_supports.append(support)
            self.world.CreateRevoluteJoint(bodyA=support, bodyB=groundBody, anchor=(200+pos[0], pos[1]))
            self.world.CreateRevoluteJoint(bodyA=support, bodyB=row_holdoff, anchor=(200+pos[0]+1.5, pos[1]+30))

        # Rods which connect row selectors to ejectors
        for r in range(0,memory_rows):
            self.world.CreateDistanceJoint(bodyA=ejectors[r],
	                              bodyB=followers[r],
	                                anchorA=(200, 14*row+10),
	                              anchorB=(200+selector_no*25+35,14*row+10),
	                                   collideConnected=False)
            
    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

