#!/usr/bin/env python3

import math

from framework import (Framework, main, Keys)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)
from Box2D import b2CircleShape

def box_polygon(width,height):
    return [(0,0), (width,0), (width,height), (0,height)]

def makeBox(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x+width/2, y+height/2), 0))

def radians(degrees):
    return 2*math.pi*float(degrees) / 360

def rotate_polygon(polygon, degrees):
    new_poly = []
    r = radians(degrees)
    return [(x*math.cos(r) - y*math.sin(r), y*math.cos(r) + x*math.sin(r)) for (x,y) in polygon]

bb_diameter = 6.35

selector_rods = 3
memory_rows = 1<<selector_rods
pitch = 22
follower_spacing = 14

bar_gate_raisers = False
cam_speed = 0.1
filters = [filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF),
           filter(groupIndex=2, categoryBits=0x0002, maskBits=0x0000),
           filter(groupIndex=3, categoryBits=0x0004, maskBits=0x0000)]
class Memory (Framework):
    name = "SSEM"


    def vertical_rotating_bar(self, xpos, ypos, height, attachment_body, support_sep=50):
        if bar_gate_raisers:
            bar_body = self.add_dynamic_polygon(box_polygon(3,height), xpos+3, ypos, filters[0])
            bar_body_2 = self.add_dynamic_polygon(box_polygon(3,height), xpos, ypos-15, filters[0])
            for i in range(0,2):
                pos = (0,support_sep*i)
                support = self.add_dynamic_polygon(box_polygon(3,30), xpos+pos[0], ypos+pos[1], filters[0])
                self.revolving_joint(bodyA=support, bodyB=attachment_body, anchor=(xpos+pos[0], ypos+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body, anchor=(xpos+1.5+pos[0], ypos+30+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body_2, anchor=(xpos+1.5+pos[0], ypos+15+pos[1]))
            bar_body.attachment_point=(xpos+4.5, ypos+height/3)
            return bar_body
        else:
            body = self.add_dynamic_polygon(box_polygon(3,height), xpos,ypos, filters[0])
            body.attachment_point = (xpos+1.5,ypos+height/2)
            self.slide_joint(attachment_body, body, (1,0), 0, 20, friction=0)
            return body

    def horizontal_rotating_bar(self, xpos, ypos, height, attachment_body,support_sep=0):
        if bar_gate_raisers:
            if support_sep == 0: support_sep = height/2
            bar_body = self.add_dynamic_polygon(box_polygon(height,3), xpos, ypos+3, filters[0])
            bar_body_2 = self.add_dynamic_polygon(box_polygon(height,3), xpos+15, ypos, filters[0])
            for i in range(0,2):
                pos = (support_sep*i,0)
                support = self.add_dynamic_polygon(box_polygon(30,3), xpos+pos[0], ypos+pos[1], filters[0])
                self.revolving_joint(bodyA=support, bodyB=attachment_body, anchor=(xpos+1.5+pos[0]+30, ypos+1.5+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body, anchor=(xpos+pos[0], ypos+1.5+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body_2, anchor=(xpos+15+pos[0], ypos+1.5+pos[1]))
            bar_body.attachment_point=(xpos+height/2, ypos+4.5)
            return bar_body
        else:
            body = self.add_dynamic_polygon(box_polygon(height,3), xpos,ypos, filters[0])
            body.attachment_point = (xpos+height/2,ypos+1.5)
            self.slide_joint(attachment_body, body, (0,-1), -20, 0, friction=0)
            return body

            
    def diverter_set(self, xpos, ypos, attachment_body, discard = False, inverted = False, slope_x=200, slope_y=100):
        filterA = filters[0]
        filterB = filters[1]
        if inverted: (filterA, filterB) = (filterB, filterA)
        conrod = self.add_dynamic_polygon(box_polygon(pitch*8,2), xpos, ypos+15, filters[2])
        for c in range(0,8):
            diverter = self.add_dynamic_polygon(box_polygon(3,20), c*pitch+xpos, ypos, filterA)
            self.revolving_joint(bodyA=diverter, bodyB=attachment_body, anchor=(xpos+c*pitch, ypos))
            self.revolving_joint(bodyA=diverter, bodyB=conrod, anchor=(xpos+c*pitch, ypos+15))

        transfer_band_x = []

        for c in range(0,8):
            self.add_static_polygon(box_polygon(2,10),  c*pitch+xpos,    -12+ypos, filterA)
            self.add_static_polygon(box_polygon(2,25),  c*pitch+xpos+11, -12+ypos, filterA)
            self.add_static_polygon(box_polygon(11,3), c*pitch+xpos+2,  -12+ypos, filterA)
            transfer_band_x.append((c*pitch+xpos, c*pitch+xpos+11))

        if discard:
            self.add_static_polygon([(0,0), (170,-10), (170,-13), (0,-3) ], xpos, ypos-11, filterB)
        elif slope_x!=0:
            exit_transfer_band_x = []
            for c in range(0,8):
                exit_slope = polygonShape(vertices=[(0,0), (slope_x,-slope_y), (slope_x,-slope_y-3), (0,-3) ])
                self.add_static_polygon(exit_slope, c*pitch+xpos, ypos-10, filter=filterB)
                exit_transfer_band_x.append((c*pitch+xpos+slope_x, c*pitch+xpos+slope_x+pitch))
            self.transfer_bands.append((ypos-10-slope_y+10, ypos-10-slope_y, exit_transfer_band_x, 1))

        self.add_static_circle(xpos-10, ypos+15, 5, filterA)
        self.add_static_circle(xpos+pitch*8-5, ypos+15, 5, filterA)
        self.add_static_polygon(makeBox(xpos-10, ypos-10, 3,30))
        self.transfer_bands.append((-12+ypos+10, -12+ypos, transfer_band_x, 1 if inverted else 0))
        conrod.attachment_point = (xpos+pitch*8, ypos+15) 
        return conrod
    def regenerator(self, xpos, ypos, attachment_body, crank_list):
        regen_parts = []
        pusher_parts = []
        for c in range(0,8):
            self.add_static_polygon(box_polygon(10,10), c*pitch+xpos-11, -12+ypos)

            pusher = fixtureDef(shape=makeBox(c*pitch+xpos-11,-12+ypos+11,2,10), density=1.0,
                                      filter=filter(groupIndex=1))
            pusher_parts.append(pusher)

            bellcrank_fixtures = [ fixtureDef(shape=makeBox(c*pitch+xpos, -12+ypos+20, 10, 3), density=1.0, filter=filters[2]),
                                fixtureDef(shape=makeBox(c*pitch+xpos, -12+ypos+8, 3, 12), density=1.0) ]
            bellcrank = self.add_multifixture(bellcrank_fixtures)

            anchorpos = (xpos+c*pitch, -12+ypos+20)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos)
            crank_list.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            
        pusher_body = self.add_multifixture(pusher_parts)
        pusher_body.attachment_point = (xpos+8*pitch,ypos+10)
        self.slide_joint(pusher_body, attachment_body, (1,0), -8,0)
        return pusher_body

    def toggle(self, xpos, ypos, attachment_body):
        toggle_shape = [ fixtureDef(shape=makeBox(xpos-10, ypos, 20, 3), density=1.0),
                            fixtureDef(shape=makeBox(xpos-1.5, ypos, 3, 10), density=1.0) ]
        toggle = self.add_multifixture(toggle_shape)
        self.revolving_joint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos), friction=1.0)

        # Bit that goes under the toggle to stop it moving too far
        self.add_static_polygon(box_polygon(6,2), xpos-3, ypos-3)
        return toggle
        
    def subtractor_output_toggle(self, xpos, ypos, attachment_body):
        toggle = self.add_dynamic_polygon([(xpos-1.5,ypos), (xpos+1.5,ypos), (xpos, ypos+10)], 0,0)
        self.revolving_joint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos))
        return toggle

    def translate_points(self, points, xpos, ypos):
        return [(x+xpos,y+ypos) for (x,y) in points]

    def subtractor(self, xpos, ypos, attachment_body, lines = 8, output_offset_dir = -1):
        output_offset_x = pitch*(lines+1)*output_offset_dir
        for c in range(0,lines):
            input_toggle = self.toggle(xpos+c*pitch, ypos-150+20*c, attachment_body)
            output_toggle = self.subtractor_output_toggle(xpos+c*pitch+output_offset_x, ypos-150+20*c, attachment_body)
            self.world.CreateDistanceJoint(bodyA=input_toggle,
	                                   bodyB=output_toggle,
	                                   anchorA=(xpos, ypos+5),
	                                   anchorB=(xpos+output_offset_x,ypos+5),
	                                   collideConnected=False)
            # Large static bits that form input channels
            self.add_static_polygon([ (0,0), (pitch-7,-5), (pitch-7,-20*(8-c)), (0,-20*(8-c)-20) ],
                                    xpos+c*pitch-pitch+3.5, ypos+pitch+9)

            # More top-side channels, but for the output
            self.add_static_polygon([ (0,0), (pitch-7,-5), (pitch-7,-20*(8-c)), (0,-20*(8-c)-20) ],
                                    xpos+c*pitch-pitch+3.5+output_offset_x, ypos+pitch+9)
            # Bottom-side channels, for the output
            self.add_static_polygon([ (-1,0), (1,0), (1,20*c+10), (-1,20*c+10) ],
                                    xpos+c*pitch+output_offset_x, ypos+pitch-185)

            self.add_static_polygon([ (-1,0), (1,0), (1,20*c+50), (-1,20*c+50) ],
                                    xpos+(c+0.5)*pitch+output_offset_x, ypos+pitch-185)

            
    def ball_bearing_lift(self,xpos,ypos,attachment_body):
        plane = 0
        radius = 30
        offset = 80
        height=500
        pentagon_points = [(radius*math.cos(i*(math.pi*2)/5), radius*math.sin(i*(math.pi*2)/5)) for i in range(0,5)]
        
        base_roller = self.add_dynamic_polygon(pentagon_points, xpos, ypos, filters[plane])
        top_roller = self.add_dynamic_polygon(pentagon_points, xpos+offset, ypos+height, filters[plane])

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


            chain_fixtures = [fixtureDef(
                    shape=polygonShape(vertices=link_polygon),
                    density=1.0,
                    filter=filters[plane]),
                          fixtureDef(
                    shape=polygonShape(vertices=raiser_polygon),
                    density=1.0,
                    filter=filters[plane]),
                ]
            chain_link=self.add_multifixture(chain_fixtures, link_pos_x, link_pos_y)
            
            chain_links.append(chain_link)
            if i>0: 
                self.revolving_joint(bodyA=chain_link, bodyB=chain_links[i-1], anchor=(link_pos_x, link_pos_y))
            link_pos_x += math.cos(radians(link_angle))*(joint_length-5)
            link_pos_y += math.sin(radians(link_angle))*(joint_length-5)
            if(i>=10 and i<15): link_angle+=(180/5)
            if(i>=25 and i<30): link_angle+=(180/5)
        self.revolving_joint(bodyA=chain_links[0], bodyB=chain_links[-1], anchor=(link_pos_x, link_pos_y))

        self.revolving_joint(bodyA=base_roller, bodyB=attachment_body, anchor=(xpos,ypos))
        self.revolving_joint(bodyA=top_roller, bodyB=attachment_body, anchor=(xpos+offset,ypos+height))

        idler = self.add_dyanmic_circle(xpos+offset/4, ypos+height/4, 30, density=10, filter=filters[0])
        self.revolving_joint(bodyA=base_roller, bodyB=idler, anchor=(xpos,ypos))

    def injector(self, xpos, ypos, attachment_body, injector_crank_array, columns=8):
        intake_angle = radians(10)
        height = 40
        crank_offset = pitch-10
        crank_y = 19

        intake_vertices = [ (0,0), (100,0), (100,10), (0,10) ]
        intake_vertices = rotate_polygon(intake_vertices, -180 - 10)
        intake_vertices = self.translate_points(intake_vertices, xpos, ypos+55)
        self.add_static_polygon(intake_vertices)


        for c in range(0,columns):
            divider_height = 23
            height2 =        23
            divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,divider_height), (0,height2) ]
            divider_vertices = self.translate_points(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            self.add_static_polygon(divider_vertices)
            
        for c in range(0,columns):
            self.add_static_polygon([ (10,-20), (24,-20), (24,-13), (10,-15)], xpos+c*pitch, ypos+pitch+10)
            
            bellcrank_shape = [ fixtureDef(shape=makeBox(c*pitch+xpos+crank_offset, ypos+crank_y+9, 10, 3), density=1.0, filter=filters[1]),
                                fixtureDef(shape=makeBox(c*pitch+xpos+crank_offset, ypos+crank_y, 3, 12), density=1.0, filter=filters[0]) ]

            bellcrank = self.add_multifixture(bellcrank_shape)
            anchorpos = (xpos+c*pitch+crank_offset, ypos+crank_y+10)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos, friction=0)
            injector_crank_array.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            # All-inject bar
            raiser = self.add_dynamic_circle(xpos+c*pitch+15, ypos+130, radius=5, density=50.0)
            self.slide_joint(attachment_body, raiser, (0,1), 0,5, friction=0)
            self.world.CreateDistanceJoint(bodyA=bellcrank,
	                              bodyB=raiser,
                                      anchorA=((anchorpos[0]+5)*self.scale,(anchorpos[1]-10)*self.scale),
	                              anchorB=raiser.worldCenter,
	                                   collideConnected=False)
        raiser_bar = self.horizontal_rotating_bar(xpos,ypos+110, pitch*9, attachment_body, support_sep=pitch*8)
        
        for c in range(0,columns+1):
            
            # Backstop for swing arm - stops the swing arm falling back too far
            self.add_static_polygon([ (10,-12), (11,-12), (11,-3), (10,-3)], xpos+c*pitch, ypos+pitch+10)

            # Thing that stops all the ball bearings rolling over the one in the crank
            self.add_static_polygon([(22,-6), (23,-6), (23,-3), (22,-3)], xpos+c*pitch+1, ypos+pitch+10)

        # roof
        roof_height=-20
        self.add_static_polygon([ (0,roof_height), (9*pitch,roof_height), (9*pitch,0), (0,0) ],
                                xpos, ypos+height+45)

        # End stop on the right
        self.add_static_polygon([ (0,0), (pitch-7,0), (pitch-7,height), (0,height) ], xpos+columns*pitch, ypos+pitch+10)
        return raiser_bar
        
    def add_ball_bearing(self, xpos, ypos, plane):
        bearing = self.add_dynamic_circle(xpos, ypos, 6.35/2, density=5.0, filter=filters[plane])
        self.ball_bearings.append((bearing,plane))

    def add_row_decoder(self, xpos, ypos, groundBody, follower_array, selector_array):
        for selector_no in range(0,selector_rods):
            row_selector_fixtures = []
            for row in range(0,8):
                enabled = (row >> selector_no) & 1
                row_selector_fixtures.append(
                    fixtureDef(shape=makeBox(14+25*selector_no,follower_spacing*row+7*enabled,14,5), density=1.0,
                               filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF))
                )
            #Pegs which are used to raise the selector rods all at once
            row_selector_fixtures.append(
                    fixtureDef(shape=circleShape(radius=5, pos=(14+25*selector_no+7,follower_spacing*9+14)), density=1.0,
                               filter=filter(groupIndex=1, categoryBits=0x0001, maskBits=0xFFFF)))

            
            row_selector = self.add_multifixture(row_selector_fixtures, xpos+200, 0)
            selector_array.append(row_selector)
            self.slide_joint(row_selector, groundBody, (0,1), -8, 0, friction=0)

        selector_holdoff_bar = self.horizontal_rotating_bar(xpos+200, ypos+130, 80, groundBody, 50)
        selector_holdoff_bar.attachment_point = (xpos+220,ypos+130)
        # Row followers
        for row in range(0,8):
            row_follower_fixtures = []
            for selector_no in range(0,selector_rods+1):
                row_follower_fixtures.append(fixtureDef(shape=circleShape(radius=3, pos=(selector_no*25+32,14*row+10)), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
            follower = self.add_multifixture(row_follower_fixtures, xpos+200, 0)
            follower_array.append(follower)
            self.slide_joint(follower, groundBody, (1,0), limit1=0, limit2=20, friction=0)

        # Holdoff bar for all output rods
        follower_holdoff_bar = self.vertical_rotating_bar(xpos+200+25*selector_rods+14,0,14*memory_rows+5, groundBody)
        
        return (selector_holdoff_bar, follower_holdoff_bar)
        
    def memory_module(self, xpos, ypos, groundBody):
        row_injector_fixtures = []
        lift = 13
        for col in range(0,8):
            row_injector_fixtures.append(fixtureDef(shape=makeBox(7+22*col,0,3,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=makeBox(22*8+12,0,7,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=makeBox(-10,0,3,12), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
        injectors=[]
        for col in range(0,8):
            injector = self.add_multifixture(row_injector_fixtures, 0, 7+14*col)
            injectors.append(injector)
            self.slide_joint(injector, groundBody, (1,0), 7, 17, friction=0.1)
            
        row_ejector_fixtures = []
        for col in range(0,8):
            # Blocks which stop the ball bearing falling past
            ejector = fixtureDef(shape=makeBox(22*col+1,0,14,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE))
            row_ejector_fixtures.append(ejector)
        row_ejector_fixtures.append(fixtureDef(shape=makeBox(22*8+14,1,3,11), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
        ejectors = []
        for col in range(0,8):
            ejector = self.add_multifixture(row_ejector_fixtures, 0, 14*col)
            ejector.attachment_point = (22*col, 0)
            ejectors.append(ejector)
            self.slide_joint(ejector, groundBody, (1,0), 0, 14, friction=0)

        # Add weights which bias the rows
        for col in range(0,8):
            crank = self.crank_left_up(xpos-250+30*col, ypos+14*col-30, groundBody, output_length=40, weight=50)
            self.distance_joint(ejectors[col], crank, ejectors[col].attachment_point, crank.attachment_point)
                              

        self.memory_followers = []
        self.memory_selectors = []
        (selector_holdoff, follower_holdoff) = self.add_row_decoder(xpos, ypos, groundBody, self.memory_followers, self.memory_selectors)
        
        # Rods which connect row selectors to ejectors
        for r in range(0,memory_rows):
            self.distance_joint(ejectors[r], self.memory_followers[r], (200,14*r+10), (200+35, 14*r+10))

        # Gate returner to push all the memory rows back in
        self.memory_returning_gate = self.vertical_rotating_bar(xpos-30,0,14*memory_rows, groundBody)

        # Wall on the left of the memory to create the final channel
        memory_fixed = self.add_static_polygon(box_polygon(3,14*8), -10,0, filter=filter(groupIndex=0, categoryBits=0x0001, maskBits=0xFFFF))
        return (selector_holdoff, follower_holdoff)

    def connect_regenerators(self):
        for i in range(0,8):
            (objectA, posA) = self.injector_cranks[i]
            (objectB, posB) = self.upper_regenerators[i]
            self.world.CreateDistanceJoint(bodyA=objectA,
	                                   bodyB=objectB,
	                                   anchorA=(posA[0]*self.scale, posA[1]*self.scale),
	                                   anchorB=(posB[0]*self.scale, posB[1]*self.scale),
	                                   collideConnected=False)
            (objectA, posA) = self.upper_regenerators[i]
            (objectB, posB) = self.lower_regenerators[i]
            self.world.CreateDistanceJoint(bodyA=objectA,
	                                   bodyB=objectB,
	                                   anchorA=(posA[0]*self.scale, posA[1]*self.scale),
	                                   anchorB=(posB[0]*self.scale, posB[1]*self.scale),
	                                   collideConnected=False)

            self.injector_cranks[i]

    def crank_left_up(self, xpos, ypos, attachment_body, output_length=20, weight=10):
        crank_fixture_1 = fixtureDef(shape=makeBox(-20,0,20,3), density=1.0, filter=filters[0])
        crank_fixture_2 = fixtureDef(shape=makeBox(0,0,3,output_length), density=1.0, filter=filters[0])
        crank_fixture_3 = fixtureDef(shape=makeBox(-20,-5,10,10), density=weight, filter=filters[0]) # Heavy weight
        crank = self.add_multifixture([crank_fixture_1, crank_fixture_2, crank_fixture_3], xpos, ypos)
        self.revolving_joint(crank, attachment_body, (xpos,ypos))
        crank.attachment_point=(xpos,ypos+output_length)
        return crank
            
    def memory_sender(self, xpos, ypos, attachment_body):
        v1 = [ (0,0), (7,-4), (7,-17), (0,-13) ]
        entrace_poly = [ (-7,2), (0,0), (0,-5), (-7,0) ]
        sensor_poly = [ (0,0), (7,-4), (7,-11), (0,-7) ]
        blocker_poly = [ (7,0), (10,-4), (10,-8), (7,-4) ]
        self.memory_sensors = []
        blocker_fixtures = []
        for c in range(0,8):
            self.add_static_polygon(v1, xpos+c*pitch, ypos)
            self.add_static_polygon(entrace_poly, xpos+c*pitch, ypos)

            
            sensor = self.add_dynamic_polygon(sensor_poly, xpos+c*pitch, ypos+7, filter=filters[0])
            self.memory_sensors.append(sensor)
            self.slide_joint(attachment_body, sensor, (0,1), 0, 15, friction=False)
            blocker_fixtures.append(fixtureDef(shape=polygonShape(vertices=self.translate_points(blocker_poly,c*pitch,0)), density=1.0, filter=filters[0]))
        blocker_set = self.add_multifixture(blocker_fixtures, xpos, ypos)
        self.slide_joint(attachment_body, blocker_set, (1,0), 0, 15, friction=False)

        crank = self.crank_left_up(xpos-30, ypos-20, attachment_body)
        self.distance_joint(crank, blocker_set, crank.attachment_point, (xpos+10,ypos))

    # Interface functions to PyBox2D

    def distance_joint(self, bodyA, bodyB, posA=None, posB=None):
        if posA is None: posA = bodyA.attachment_point
        if posB is None: posB = bodyB.attachment_point
        self.world.CreateDistanceJoint(bodyA=bodyA,
	                               bodyB=bodyB,
	                               anchorA=(posA[0]*self.scale, posA[1]*self.scale),
	                               anchorB=(posB[0]*self.scale, posB[1]*self.scale),
	                               collideConnected=False)
        
    
    def add_static_polygon(self,vertices, xpos=0, ypos=0, filter=filters[0]):
        translated_vertices = self.translate_points(vertices, xpos, ypos)
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in translated_vertices])
        fixture = fixtureDef(shape=shape, density=1.0, filter=filter)
        return self.world.CreateStaticBody(fixtures=fixture)

    def add_static_circle(self, xpos, ypos, radius, filter=filters[0]):
        return self.world.CreateStaticBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale, ypos*self.scale)), filter=filter))

    def add_dynamic_circle(self, xpos, ypos, radius, density=1.0, filter=filters[0]):
        circle = self.world.CreateDynamicBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale,ypos*self.scale)), density=density, filter=filter))
        circle.attachment_point = (xpos*self.scale, ypos*self.scale)
        return circle
        
    def add_dynamic_polygon(self, vertices, xpos, ypos, filter=filters[0]):
        translated_vertices = self.translate_points(vertices, xpos, ypos)
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in translated_vertices])
        fixture = fixtureDef(shape=shape, density=1.0, filter=filter)
        return self.world.CreateDynamicBody(fixtures=fixture)

    def slide_joint(self, body1, body2, axis, limit1, limit2,friction=1.0):
        if friction==False: friction = 0
        
        if friction> 0:
            return self.world.CreatePrismaticJoint(
                bodyA=body1, bodyB=body2,
                anchor=body2.worldCenter,
                axis=axis, maxMotorForce=10000.0*friction,
                enableMotor=True, lowerTranslation=limit1*self.scale,
                upperTranslation=limit2*self.scale, enableLimit=True)
        else:
            return self.world.CreatePrismaticJoint(
                bodyA=body1, bodyB=body2,
                anchor=body2.worldCenter,
                axis=axis, lowerTranslation=limit1*self.scale,
                upperTranslation=limit2*self.scale, enableLimit=True)

    def pulley_joint(self, bodyA, bodyB, attachmentA, attachmentB, groundA, groundB):        
        self.world.CreatePulleyJoint(
            bodyA=bodyA, 
            bodyB=bodyB, 
            anchorA = (attachmentA[0]*self.scale, attachmentA[1]*self.scale),
            anchorB = (attachmentB[0]*self.scale, attachmentB[1]*self.scale),
            groundAnchorA = (groundA[0]*self.scale, groundA[1]*self.scale),
            groundAnchorB = (groundB[0]*self.scale, groundB[1]*self.scale),
            ratio=1.0)
        
    def add_multifixture(self, fixtures, xpos=0, ypos=0):
        # Multifixtures are a bit tricky - we have to go inside the shape data to scale it.
        new_fixtures = []
        for f in fixtures:
            if isinstance(f.shape, b2CircleShape):
                new_fixtures.append(fixtureDef(shape=circleShape(radius=f.shape.radius*self.scale,
                                                                 pos=(f.shape.pos.x*self.scale, f.shape.pos.y*self.scale)),
                                                                 filter=f.filter,
                                                                 density=f.density))
            else:
                new_fixtures.append(fixtureDef(shape=polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in f.shape.vertices]),
                                               filter=f.filter,
                                               density=f.density))
                                               
        return self.world.CreateDynamicBody(position=(xpos*self.scale, ypos*self.scale), fixtures=new_fixtures)

    def add_multipolygon(self, polygons, xpos=0, ypos=0, filter=filters[0]):
        fixtures = []
        for p in polygons:
            fixtures.append(fixtureDef(shape=polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in p]),
                                               filter=filter,
                                               density=1.0))
                                               
        return self.world.CreateDynamicBody(position=(xpos*self.scale, ypos*self.scale), fixtures=fixtures)

    
    def revolving_joint(self, bodyA, bodyB, anchor, friction=False, motor=0, force=1):
        (x,y) = anchor

        if motor!=0:
            j = self.world.CreateRevoluteJoint(bodyA=bodyA, bodyB=bodyB, anchor=(x*self.scale, y*self.scale),
                                           maxMotorTorque = 100000.0*force,
                                           motorSpeed = motor,
                                           enableMotor = True)
        elif friction:
            j = self.world.CreateRevoluteJoint(bodyA=bodyA, bodyB=bodyB, anchor=(x*self.scale, y*self.scale),
                                           maxMotorTorque = 10000.0*force,
                                           motorSpeed = 0.0,
                                           enableMotor = True)
        else:
            j = self.world.CreateRevoluteJoint(bodyA=bodyA, bodyB=bodyB, anchor=(x*self.scale, y*self.scale))
        return j

    # End of interface functions

    def connect_memory(self):
        """ Connects the memory selectors to the memory senders """
        for i in range(0,len(self.memory_selectors)):
            bodyA = self.memory_selectors[i]
            bodyB = self.memory_sensors[i]
            self.world.CreateDistanceJoint(bodyA=bodyA,
	                                   bodyB=bodyB,
	                                   anchorA=bodyA.worldCenter,
	                                   anchorB=bodyB.worldCenter,
	                                   collideConnected=False)

        for i in range(0,len(self.rom_selectors)):
            bodyA = self.rom_selectors[i]
            bodyB = self.memory_sensors[i+5]
            self.world.CreateDistanceJoint(bodyA=bodyA,
	                                   bodyB=bodyB,
	                                   anchorA=bodyA.worldCenter,
	                                   anchorB=bodyB.worldCenter,
	                                   collideConnected=False)

            
    def add_cam(self, xpos, ypos, attachment_body, follower_len, phase=0, horizontal=False, reverse_direction=False, axis_offset=0):
        """ Very basic function which just adds a motorised circle with a bump.
        phase is between 0 and 1 and adjusts initial rotation. """
        
        radius = 30
        disc_fixture = fixtureDef(shape=circleShape(radius=radius, pos=(0,0)),density=1.0,filter=filters[0])
        bump_polygon = self.translate_points([ (-10,-3), (-7,3), (7, 3), (10,-3) ], 0, radius)
        bump_polygon = rotate_polygon(bump_polygon, phase*360)
        bump_fixture = fixtureDef(shape=polygonShape(vertices=bump_polygon),density=1.0,filter=filters[0])        
        cam_body = self.add_multifixture([disc_fixture, bump_fixture], xpos, ypos)
        cam_driver = self.revolving_joint(attachment_body, cam_body, (xpos,ypos), motor=0.25, force=50)
        cam_driver.motorSpeed = 0

        axle_y = ypos+radius
        if horizontal:
            if reverse_direction:
                axle_x = xpos-radius-2.5-axis_offset
            else:
                axle_x = xpos+radius+2.5+axis_offset
            follower_body = self.add_dynamic_polygon(makeBox(axle_x-2.5, axle_y-follower_len, 5, follower_len), 0, 0)
        else:
            axle_x = xpos-radius
            follower_body = self.add_dynamic_polygon(makeBox(axle_x, axle_y+2.5, follower_len, 5), 0, 0)
        print("Creating cam: xpos= {}, ypos= {}, axle_x = {} ,axle_y= {}, follower_len={}".format(xpos, ypos, axle_x, axle_y, follower_len))

        if horizontal:
            follower_body.attachment_point=(axle_x, axle_y-follower_len)
        else:
            follower_body.attachment_point=(axle_x+follower_len, axle_y)
        print("Setting attachment point: (x,y)={}".format(follower_body.attachment_point))
        self.revolving_joint(attachment_body, follower_body, (axle_x+2.5,axle_y+2.5), friction=False)
        self.all_cam_drives.append(cam_driver)
        return follower_body

    def add_instruction_cranks(self, attachment_body, xpos, ypos):
        len1 = 25
        len2 = 25
        thickness = 2.5
        for i in range(0,8):
            crank_polygon1 = [ (-thickness,-thickness), (-thickness,-len1-thickness), (thickness,-len1-thickness), (thickness,-thickness) ]
            crank_polygon2 = [ (-thickness,-thickness), (-thickness,thickness), (len2+thickness,thickness), (len2+thickness,-thickness) ]
            crank = self.add_multipolygon([crank_polygon1, crank_polygon2], xpos+i*30, ypos-i*follower_spacing)
            self.revolving_joint(attachment_body, crank, (xpos+i*30,ypos-i*follower_spacing))
            self.world.CreateDistanceJoint(bodyA=crank, bodyB=self.rom_followers[7-i], anchorA=((xpos+i*30)*self.scale, (ypos-i*follower_spacing-len1)*self.scale), anchorB=self.rom_followers[7-i].worldCenter, collideConnected=False)
            block = self.add_dynamic_polygon(box_polygon(30,5), xpos+i*30, ypos-i*50-50)
            self.slide_joint(attachment_body, block, (0,1), -8,0)
                                             
            self.world.CreateDistanceJoint(bodyA=crank, bodyB=block, anchorA=((xpos+i*30+len2)*self.scale, (ypos-i*follower_spacing)*self.scale), anchorB=block.worldCenter, collideConnected=False)
            

    def __init__(self):
        super(Memory, self).__init__()
        self.camSpeed =0 
        self.all_cam_drives = []
        self.scale = 0.5
        self.transfer_bands = []
        self.ball_bearings = []
        memory_fixed_shapes = []
        for row in range(0,8):
            for col in range(0,8):
                memory_fixed_filter = filter(groupIndex=0, categoryBits=0x0001, maskBits=0xFFFF)
                self.add_static_polygon(box_polygon(14,7), 22*col, 14*row, filter=memory_fixed_filter)
                self.add_static_polygon(box_polygon(3,8), 22*col+14-3+1, 14*row+6, filter=memory_fixed_filter)
                pass
        groundBox = makeBox(-20,-500,1,1)
        groundBody = self.world.CreateStaticBody(shapes=groundBox)
        # Initial charge for main injector
        for r in range(0,8):
            for i in range(0,2):
                test_data = self.add_ball_bearing(-100+7*i,230+7*r,0)
        self.injector_cranks = []
        self.injector(-32,110, groundBody, injector_crank_array=self.injector_cranks)
        (memory_selector_holdoff, memory_follower_holdoff) = self.memory_module(0,0, groundBody)
        self.upper_regenerators = []
        self.diverter_set(-5,-25, groundBody, slope_x=-200) # Diverter 1. Splits to subtractor reader.
        self.diverter_set(-15,-57.5, groundBody, discard=True) # Diverter 2. Discards all output.
        upper_regen_control = self.regenerator(-15,-85, groundBody, self.upper_regenerators) # Regenerator 1. For regenning anything read from memory.
        diverter_3 = self.diverter_set(-10,-125, groundBody, slope_x=200) # Diverter 3; splits to instruction reg/PC

        # PC injector
        self.pc_injector_cranks = []
    
        mem_injector_raiser = self.injector(300,-200, groundBody, injector_crank_array=self.pc_injector_cranks, columns=5)
        # Initial charge for PC injector
        for r in range(0,8):
            for i in range(0,2):
                test_data = self.add_ball_bearing(250+7*i,-120+7*r,0)
        self.subtractor(0,-210, groundBody)
        self.lower_regenerators = []
        self.regenerator(-200,-390, groundBody, self.lower_regenerators)
        #Program counter
        self.subtractor(200,-310, groundBody, lines=5, output_offset_dir=1)

        self.connect_regenerators()
        # gutter
        gutter_vertices = [ (0,0), (pitch*9,10), (pitch*9,-10), (0,-10) ]
        gutter_vertices = self.translate_points(gutter_vertices, -20, -420)
        gutter = self.add_static_polygon(gutter_vertices)

        self.add_static_polygon([ (-300,-600),(500,-600), (500,-610), (-300,-610)])
        wall_vertices = [ (0,-500), (0,-430), (10,-430), (10,-500) ]
        self.add_static_polygon(wall_vertices)
        wall_vertices = self.translate_points(wall_vertices, -300, 0)
        self.add_static_polygon(wall_vertices)

        # Instruction decoder ROM
        self.rom_followers = []
        self.rom_selectors = []
        self.add_row_decoder(200, 0, groundBody, self.rom_followers, self.rom_selectors)

        self.add_instruction_cranks(groundBody, 550, 140)
       
        #self.ball_bearing_lift(-200,-400,groundBody)
        self.memory_sender(325+17,-500, groundBody)
        self.connect_memory()
        print("Scale is {}".format(self.scale))

        # Cams

        # Cam 1: Fires memory injector, reading PC into address reg.
        follower_body = self.add_cam(300,200, groundBody, 100, phase=0.45)
        self.distance_joint(follower_body, mem_injector_raiser, follower_body.attachment_point, mem_injector_raiser.attachment_point)

        # Cam 2: Main memory selector lifter
        follower_body = self.add_cam(150,300, groundBody, 150, phase=0.25)
        print("Attachemnt point on cam is {}".format(follower_body.attachment_point))
        self.distance_joint(follower_body, memory_selector_holdoff)

        # Cam 2: Memory returner
        follower_body = self.add_cam(600,300, groundBody, 60, horizontal=True)
        self.distance_joint(follower_body, self.memory_returning_gate)

        # Cam 4: Memory holdoff
        follower_body = self.add_cam(600,200, groundBody, 90, horizontal=True)
        self.distance_joint(follower_body, memory_follower_holdoff)

        # Cam 5: Regenerator 1
        follower_body = self.add_cam(800, 100, groundBody, 60, horizontal=True)
        self.distance_joint(follower_body, upper_regen_control)

        # Cam 5: Split to instruction counter/reg
        follower_body = self.add_cam(800,-100, groundBody, 60, horizontal=True, reverse_direction=True, axis_offset=3)
        self.distance_joint(follower_body, diverter_3)

    def Step(self, settings):
        super(Memory, self).Step(settings)
        for i in range(0,len(self.ball_bearings)):
            (b, plane) = self.ball_bearings[i]
            (x,y) = b.worldCenter
            x /= self.scale
            y /= self.scale
            for (top, bottom, xbands, source_plane) in self.transfer_bands:
                if y<top and y>bottom and plane == source_plane:
                    for (left, right) in xbands:
                        if x>left and x<right:
                            self.world.DestroyBody(b)
                            plane = 1-source_plane
                            print("Flipping ball bearing from plane %d to plane %d"%(source_plane, plane))
                            self.ball_bearings[i] = (self.world.CreateDynamicBody(
                                position=b.worldCenter,
                                fixtures=[fixtureDef(
                                    shape=circleShape(radius=6.35/2*self.scale, pos=(0,0)),
                                    density=5.0,
                                    filter=filters[plane])]
                                
                            ),plane)


    def Keyboard(self, key):
        print("Processing key: {}".format(key))
        if key == Keys.K_r:
            self.camSpeed = 0 if self.camSpeed == cam_speed else cam_speed
            
            for d in self.all_cam_drives:
                
                d.motorSpeed = self.camSpeed

if __name__ == "__main__":
    main(Memory)

    
