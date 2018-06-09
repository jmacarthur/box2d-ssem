#!/usr/bin/env python3

from framework import (Framework, main)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)

def makeBox(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x+width/2, y+height/2), 0))


# Circle example
#shape=circleShape(radius=6.35,pos=(0,10)),

selector_rods = 3
memory_rows = 1<<selector_rods
pitch = 22
class Memory (Framework):
    name = ""


    def slide_joint(self, body1, body2, axis, limit1, limit2,friction=True):
        if friction:
            return self.world.CreatePrismaticJoint(
                bodyA=body1, bodyB=body2,
                anchor=body2.worldCenter,
                axis=axis, maxMotorForce=10000.0,
                enableMotor=True, lowerTranslation=limit1,
                upperTranslation=limit2, enableLimit=True)
        else:
            return self.world.CreatePrismaticJoint(
                bodyA=body1, bodyB=body2,
                anchor=body2.worldCenter,
                axis=axis, lowerTranslation=limit1,
                upperTranslation=limit2, enableLimit=True)

    def rotating_bar(self, xpos, ypos, height, attachment_body):
        row_holdoff = self.world.CreateDynamicBody(
            position=(xpos,0),
            fixtures = fixtureDef(shape=makeBox(0,0,3,height), density=1.0,
                                  filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))
            )
        row_holdoff_supports = []
        for i in range(0,2):
            pos = (0,(14*6)*i)
            support = self.world.CreateDynamicBody( position=(xpos,0),
                                                    fixtures = fixtureDef(shape=makeBox(pos[0], pos[1],3,30), density=1.0,
                                                                          filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF)))
            row_holdoff_supports.append(support)
            self.world.CreateRevoluteJoint(bodyA=support, bodyB=attachment_body, anchor=(xpos, ypos+pos[1]))
            self.world.CreateRevoluteJoint(bodyA=support, bodyB=row_holdoff, anchor=(xpos+1.5+pos[0], ypos+30+pos[1]))

    def diverter_set(self, xpos, ypos, attachment_body):
        conrod = self.world.CreateDynamicBody(
            position=(xpos,ypos),
            fixtures = fixtureDef(shape=makeBox(0,15,pitch*8,2), density=1.0,
                                  filter=filter(groupIndex=2, categoryBits=0x0002, maskBits=0x0000)) # Never collides with anything
        )
        for c in range(0,8):
            diverter = self.world.CreateDynamicBody(
                position=(xpos,ypos),
                fixtures = fixtureDef(shape=makeBox(c*pitch,0,3,20), density=1.0,
                                      filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))
            )
            self.world.CreateRevoluteJoint(bodyA=diverter, bodyB=attachment_body, anchor=(xpos+c*pitch, ypos))
            self.world.CreateRevoluteJoint(bodyA=diverter, bodyB=conrod, anchor=(xpos+c*pitch, ypos+15))

        for c in range(0,8):
            self.world.CreateStaticBody(shapes=makeBox(c*pitch+xpos, -12+ypos,2,10))
            self.world.CreateStaticBody(shapes=makeBox(c*pitch+xpos+11, -12+ypos,2,25))
            self.world.CreateStaticBody(shapes=makeBox(c*pitch+xpos+2, -12+ypos,11,3))
        
        self.world.CreateStaticBody(shapes=circleShape(radius=5, pos=(xpos-10, ypos+15)))
        self.world.CreateStaticBody(shapes=circleShape(radius=5, pos=(xpos+pitch*8-5, ypos+15)))

    def regenerator(self, xpos, ypos, attachment_body):
        regen_parts = []
        pusher_parts = []
        for c in range(0,8):
            self.world.CreateStaticBody(shapes=makeBox(c*pitch+xpos-11, -12+ypos,11,10))

            pusher = fixtureDef(shape=makeBox(c*pitch+xpos-11,-12+ypos+11,2,10), density=1.0,
                                      filter=filter(groupIndex=1))
            pusher_parts.append(pusher)

            bellcrank_shape = [ fixtureDef(shape=makeBox(c*pitch+xpos, -12+ypos+20, 10, 3), density=1.0),
                                fixtureDef(shape=makeBox(c*pitch+xpos, -12+ypos+8, 3, 12), density=1.0) ]
            bellcrank = self.world.CreateDynamicBody(fixtures = bellcrank_shape)
            self.world.CreateRevoluteJoint(bodyA=bellcrank, bodyB=attachment_body, anchor=(xpos+c*pitch, -12+ypos+20))
            
        pusher_body = self.world.CreateDynamicBody(fixtures = pusher_parts)
        self.slide_joint(pusher_body, attachment_body, (1,0), -8,0)
        
    def __init__(self):
        super(Memory, self).__init__()


        memory_fixed_shapes = []
        for row in range(0,8):
            for col in range(0,8):
                memory_fixed_shapes.append(makeBox(22*col,14*row,14,7))
                memory_fixed_shapes.append(makeBox(22*col+14-3+1,14*row+6,3,8))
        groundBox = makeBox(-20,-200,40,1)
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

        # Row followers
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

        self.rotating_bar(200+25*selector_rods+14,0,14*memory_rows+20, groundBody)
        self.rotating_bar(-20,0,14*memory_rows+20, groundBody)
            
        # Rods which connect row selectors to ejectors
        for r in range(0,memory_rows):
            self.world.CreateDistanceJoint(bodyA=ejectors[r],
	                              bodyB=followers[r],
	                                anchorA=(200, 14*row+10),
	                              anchorB=(200+selector_no*25+35,14*row+10),
	                                   collideConnected=False)

        self.diverter_set(0,-50, groundBody)
        self.regenerator(0,-80, groundBody)
        self.diverter_set(0,-120, groundBody)
        self.diverter_set(-5,-160, groundBody)
    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

