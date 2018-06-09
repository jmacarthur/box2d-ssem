#!/usr/bin/env python3

import math

from framework import (Framework, main)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)

def makeBox(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x+width/2, y+height/2), 0))

def radians(degrees):
    return 2*math.pi*float(degrees) / 360

def rotate_polygon(polygon, degrees):
    new_poly = []
    r = radians(degrees)
    return [(x*math.cos(r) - y*math.sin(r), y*math.cos(r) + x*math.sin(r)) for (x,y) in polygon]

# Circle example
#shape=circleShape(radius=6.35,pos=(0,10)),

selector_rods = 3
memory_rows = 1<<selector_rods
pitch = 22

filters = [filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF),
           filter(groupIndex=2, categoryBits=0x0002, maskBits=0x0000)]
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

    def regenerator(self, xpos, ypos, attachment_body, crank_list):
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
            anchorpos = (xpos+c*pitch, -12+ypos+20)
            self.world.CreateRevoluteJoint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos)
            crank_list.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            
        pusher_body = self.world.CreateDynamicBody(fixtures = pusher_parts)
        self.slide_joint(pusher_body, attachment_body, (1,0), -8,0)

    def toggle(self, xpos, ypos, attachment_body):
        toggle_shape = [ fixtureDef(shape=makeBox(xpos-10, ypos, 20, 3), density=1.0),
                            fixtureDef(shape=makeBox(xpos-1.5, ypos, 3, 10), density=1.0) ]
        toggle = self.world.CreateDynamicBody(fixtures = toggle_shape)
        self.world.CreateRevoluteJoint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos),
                                       maxMotorTorque = 10000.0,
                                       motorSpeed = 0.0,
                                       enableMotor = True)
        self.world.CreateStaticBody(shapes=makeBox(xpos-3, ypos-3,6,3))
        return toggle
        
    def subtractor_output_toggle(self, xpos, ypos, attachment_body):
        toggle_shape = [ fixtureDef(shape=makeBox(xpos-1.5, ypos, 3, 10), density=1.0, filter = filter(groupIndex=2,categoryBits=0x0002, maskBits=0x0000)) ]
        toggle = self.world.CreateDynamicBody(fixtures = toggle_shape )
        self.world.CreateRevoluteJoint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos))
        return toggle

    def translate_points(self, points, xpos, ypos):
        return [(x+xpos,y+ypos) for (x,y) in points]

    def subtractor(self, xpos, ypos, attachment_body):
        for c in range(0,8):
            input_toggle = self.toggle(xpos+c*pitch, ypos-150+20*c, attachment_body)
            output_toggle = self.subtractor_output_toggle(xpos+c*pitch-pitch, ypos-150+20*c, attachment_body)
            self.world.CreateDistanceJoint(bodyA=input_toggle,
	                                   bodyB=output_toggle,
	                                   anchorA=(xpos, ypos),
	                                   anchorB=(xpos-pitch,ypos),
	                                   collideConnected=False)
            divider_vertices = [ (0,0), (pitch-7,-5), (pitch-7,-20*(8-c)), (0,-20*(8-c)-20) ]
            divider_vertices = self.translate_points(divider_vertices, xpos+c*pitch-pitch+3.5, ypos+pitch+10)
            divider_shape = polygonShape(vertices=divider_vertices)
            self.world.CreateStaticBody(shapes=divider_shape)

    def ball_bearing_lift(self,xpos,ypos,attachment_body):
        plane = 0
        radius = 30
        offset = 80
        height=500
        pentagon_points = [(radius*math.cos(i*(math.pi*2)/5), radius*math.sin(i*(math.pi*2)/5)) for i in range(0,5)]
        
        base_roller = self.world.CreateDynamicBody(
            position=(xpos, ypos),
            fixtures=[fixtureDef(
                shape=polygonShape(vertices=pentagon_points),
                density=1.0,
                filter=filters[plane])]
        )
        top_roller = self.world.CreateDynamicBody(
            position=(xpos+offset, ypos+height),
            fixtures=[fixtureDef(
                shape=polygonShape(vertices=pentagon_points),
                density=1.0,
                filter=filters[plane])]
        )

        joint_length = 48
        chain_links = []
        link_pos_x = xpos+30
        link_pos_y = ypos
        link_angle = 80
        for i in range(0,30):
            link_polygon = [ (-5,-5), (joint_length-5,-5), (joint_length-5, 5), (-5,5) ]
            raiser_polygon = [ ((joint_length-5)/2-1.5,0), ((joint_length-5)/2+1.5,0), ((joint_length-5)/2+1.5,-20), ((joint_length-5)/2-1.5,-20) ]
            link_polygon = rotate_polygon(link_polygon, link_angle)
            raiser_polygon = rotate_polygon(raiser_polygon, link_angle)
            chain_link = self.world.CreateDynamicBody(
                position=(link_pos_x, link_pos_y),
                fixtures=[fixtureDef(
                    shape=polygonShape(vertices=link_polygon),
                    density=1.0,
                    filter=filters[plane]),
                          fixtureDef(
                    shape=polygonShape(vertices=raiser_polygon),
                    density=1.0,
                    filter=filters[plane]),
                ])
            chain_links.append(chain_link)
            if i>0: 
                self.world.CreateRevoluteJoint(bodyA=chain_link, bodyB=chain_links[i-1], anchor=(link_pos_x, link_pos_y))
            link_pos_x += math.cos(radians(link_angle))*(joint_length-5)
            link_pos_y += math.sin(radians(link_angle))*(joint_length-5)
            if(i>=10 and i<15): link_angle+=(180/5)
            if(i>=25 and i<30): link_angle+=(180/5)
        self.world.CreateRevoluteJoint(bodyA=chain_links[0], bodyB=chain_links[-1], anchor=(link_pos_x, link_pos_y))

        self.world.CreateRevoluteJoint(bodyA=base_roller, bodyB=attachment_body, anchor=(xpos,ypos))
        self.world.CreateRevoluteJoint(bodyA=top_roller, bodyB=attachment_body, anchor=(xpos+offset,ypos+height))
        idler = self.world.CreateDynamicBody(
            position=(xpos+offset/4, ypos+height/4),
            fixtures=[fixtureDef(
                shape=circleShape(radius=30, pos=(0,0)),
                density=10.0,
                filter=filters[0])]

        )
        self.world.CreateRevoluteJoint(bodyA=base_roller, bodyB=idler, anchor=(xpos,ypos))

    def injector(self, xpos, ypos, attachment_body):
        intake_angle = radians(10)
        height = 40
        crank_offset = pitch-10
        crank_y = 19
        for c in range(0,8):
            divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,height-c*pitch*math.sin(intake_angle)-(pitch-7)*math.sin(intake_angle)), (0,height-c*pitch*math.sin(intake_angle)) ]
            divider_vertices = self.translate_points(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            divider_shape = polygonShape(vertices=divider_vertices)
            self.world.CreateStaticBody(position=(0,0), shapes=divider_shape)
            
        self.injector_cranks = []
        for c in range(0,8):
            divider_vertices = [ (10,-20), (24,-20), (24,-13), (10,-15)]
            divider_vertices = self.translate_points(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            divider_shape = polygonShape(vertices=divider_vertices)
            self.world.CreateStaticBody(position=(0,0), shapes=divider_shape)

            bellcrank_shape = [ fixtureDef(shape=makeBox(c*pitch+xpos+crank_offset, ypos+crank_y+9, 10, 3), density=1.0, filter=filters[1]),
                                fixtureDef(shape=makeBox(c*pitch+xpos+crank_offset, ypos+crank_y, 3, 12), density=1.0, filter=filters[0]) ]

            bellcrank = self.world.CreateDynamicBody(fixtures = bellcrank_shape)
            anchorpos = (xpos+c*pitch+crank_offset, ypos+crank_y+10)
            self.world.CreateRevoluteJoint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos)
            self.injector_cranks.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))

            
        for c in range(0,9):
            divider_vertices = [ (10,-10), (12,-10), (12,-3), (10,-3)]
            divider_vertices = self.translate_points(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            divider_shape = polygonShape(vertices=divider_vertices)
            self.world.CreateStaticBody(position=(0,0), shapes=divider_shape)
            
        divider_vertices = [ (0,0), (9*pitch,-9*pitch*math.sin(intake_angle)), (9*pitch,height), (0,height) ]
        divider_vertices = self.translate_points(divider_vertices, xpos, ypos+height+45)
        divider_shape = polygonShape(vertices=divider_vertices)
        self.world.CreateStaticBody(position=(0,0), shapes=divider_shape)

        # End stop on the right
        divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,height), (0,height) ]
        divider_vertices = self.translate_points(divider_vertices, xpos+8*pitch, ypos+pitch+10)
        divider_shape = polygonShape(vertices=divider_vertices)
        self.world.CreateStaticBody(position=(0,0), shapes=divider_shape)
        
    def add_ball_bearing(self, xpos, ypos, plane):
        self.world.CreateDynamicBody(
            position=(xpos, ypos),
            fixtures=[fixtureDef(
                shape=circleShape(radius=6.35/2, pos=(22,150)),
                density=5.0,
                filter=filters[plane])]

        )

    def memory_module(self, xpos, ypos, groundBody):
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

        # Wall on the left of the memory to create the final channel
        
        memory_fixed = self.world.CreateStaticBody(shapes=makeBox(-10,0,3,14*8))

    def connect_regenerators(self):
        for i in range(0,8):
            (objectA, posA) = self.injector_cranks[i]
            (objectB, posB) = self.upper_regenerators[i]
            self.world.CreateDistanceJoint(bodyA=objectA,
	                                   bodyB=objectB,
	                                   anchorA=posA,
	                                   anchorB=posB,
	                                   collideConnected=False)
            (objectA, posA) = self.upper_regenerators[i]
            (objectB, posB) = self.lower_regenerators[i]
            self.world.CreateDistanceJoint(bodyA=objectA,
	                                   bodyB=objectB,
	                                   anchorA=posA,
	                                   anchorB=posB,
	                                   collideConnected=False)

            self.injector_cranks[i]
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
        
        test_data = self.add_ball_bearing(-10,0,0)


        self.injector(-32,110, groundBody)
        self.memory_module(0,0, groundBody)
        self.upper_regenerators = []
        self.diverter_set(0,-50, groundBody)
        self.regenerator(0,-80, groundBody, self.upper_regenerators)
        self.diverter_set(0,-120, groundBody)
        self.diverter_set(-5,-160, groundBody)
        self.subtractor(0,-210, groundBody)
        self.lower_regenerators = []
        self.regenerator(0,-380, groundBody, self.lower_regenerators)

        self.connect_regenerators()
        # gutter
        gutter_vertices = [ (0,0), (pitch*9,10), (pitch*9,-10), (0,-10) ]
        gutter_vertices = self.translate_points(gutter_vertices, -20, -420)
        gutter_shape = polygonShape(vertices=gutter_vertices)
        gutter = self.world.CreateStaticBody(shapes=gutter_shape)

        self.world.CreateStaticBody(shapes=polygonShape(vertices=[ (-300,-500),(200,-500), (200,-510), (-300,-510)]))
        wall_vertices = [ (0,-500), (0,-430), (10,-430), (10,-500) ]
        self.world.CreateStaticBody(shapes=polygonShape(vertices=wall_vertices))
        wall_vertices = self.translate_points(wall_vertices, -300, 0)
        self.world.CreateStaticBody(shapes=polygonShape(vertices=wall_vertices))

        self.ball_bearing_lift(-200,-400,groundBody)
        
        
    def Step(self, settings):
        super(Memory, self).Step(settings)

if __name__ == "__main__":
    main(Memory)

