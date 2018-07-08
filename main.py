#!/usr/bin/env python3

import argparse
import copy
import math
import random

from framework import (Framework, main, Keys)
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)
from Box2D import b2CircleShape
from constants import *
from test_sets import test_set


def box_vertices(x, y, width,height):
    return [(0,0), (width,0), (width,height), (0,height)]

def box_polygon_shape(x, y, width, height):
    return polygonShape(box=(width/2, height/2, (x+width/2, y+height/2), 0))

def radians(degrees):
    return 2*math.pi*float(degrees) / 360

def rotate_polygon(polygon, degrees):
    return rotate_polygon_radians(polygon, radians(degrees))

def rotate_polygon_radians(polygon, r):
    return [(x*math.cos(r) - y*math.sin(r), y*math.cos(r) + x*math.sin(r)) for (x,y) in polygon]

def translate_polygon(points, xpos, ypos):
    return [(x+xpos,y+ypos) for (x,y) in points]

class Parts():
    """ This is a container class for parts within the SSEM. Parts are filled in gradually during setup. """
    def __init__(self):
        self.pc_injector_raiser = None
        self.memory_selector_holdoff = None
        self.instruction_selector_holdoff = None
        self.sender_eject = None
        self.main_injector_raiser = None
        self.accumulator_diverter_lever = None
        self.lower_regen_control = None
        self.pc_reset_lever = None
        self.cmd_injector = None

class Memory (Framework):
    name = "SSEM"

    def vertical_rotating_bar(self, xpos, ypos, height, attachment_body, support_sep=50):
        if bar_gate_raisers:
            bar_body = self.add_dynamic_polygon(box_vertices(0, 0, 3,height), xpos+3, ypos, filters[0])
            bar_body_2 = self.add_dynamic_polygon(box_vertices(0, 0, 3,height), xpos, ypos-15, filters[0])
            for i in range(0,2):
                pos = (0,support_sep*i)
                support = self.add_dynamic_polygon(box_vertices(0, 0, 3,30), xpos+pos[0], ypos+pos[1], filters[0])
                self.revolving_joint(bodyA=support, bodyB=attachment_body, anchor=(xpos+pos[0], ypos+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body, anchor=(xpos+1.5+pos[0], ypos+30+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body_2, anchor=(xpos+1.5+pos[0], ypos+15+pos[1]))
            bar_body.attachment_point=(xpos+4.5, ypos+height/3)
            return bar_body
        else:
            body = self.add_dynamic_polygon(box_vertices(0, 0, 3,height), xpos,ypos, filters[0])
            body.attachment_point = (xpos+1.5,ypos+height/2)
            self.slide_joint(attachment_body, body, (1,0), 0, 20, friction=0)
            return body

    def horizontal_rotating_bar(self, xpos, ypos, height, attachment_body,support_sep=0):
        if bar_gate_raisers:
            if support_sep == 0: support_sep = height/2
            bar_body = self.add_dynamic_polygon(box_vertices(0, 0, height,3), xpos, ypos+3, filters[0])
            bar_body_2 = self.add_dynamic_polygon(box_vertices(0, 0, height,3), xpos+15, ypos, filters[0])
            for i in range(0,2):
                pos = (support_sep*i,0)
                support = self.add_dynamic_polygon(box_vertices(0, 0, 30,3), xpos+pos[0], ypos+pos[1], filters[0])
                self.revolving_joint(bodyA=support, bodyB=attachment_body, anchor=(xpos+1.5+pos[0]+30, ypos+1.5+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body, anchor=(xpos+pos[0], ypos+1.5+pos[1]))
                self.revolving_joint(bodyA=support, bodyB=bar_body_2, anchor=(xpos+15+pos[0], ypos+1.5+pos[1]))
            bar_body.attachment_point=(xpos+height/2, ypos+4.5)
            return bar_body
        else:
            body = self.add_dynamic_polygon(box_vertices(0, 0, height,3), xpos,ypos, filters[0])
            body.attachment_point = (xpos+height/2,ypos+1.5)
            self.slide_joint(attachment_body, body, (0,-1), -20, 0, friction=0)
            return body

            
    def diverter_set(self, xpos, ypos, attachment_body, discard = 0, inverted = False, slope_x=200, slope_y=100, start_at=0, mirror=False, return_weight=50):
        """discard: nonzero used to indicate the right-side output form this
        is always discarded, so we can combine outputs. Value
        indicates the distance the discard plane runs for. Negative
        values means it discards to the left.
        
        inverted: Normal behaviour is to input data in plane 0 and output it in
        0 going through 1 temporarily. inverted means 1 to 1 via 0.
        
        slope_x, slope_y: How far the diverted output is offset in
        both directions. (slope_x can be negative; slope_y must be
        positive)
        
        start_at: Reduces the number of lanes e.g. for diversion only
        to the instruction register.

        mirror: Only affects discarders - if set, discards to the left
        instead of the usual right.

        """
        filterA = filters[0]
        filterB = filters[1]
        if inverted: (filterA, filterB) = (filterB, filterA)
        conrod = self.add_dynamic_polygon(box_vertices(0, 0, pitch*8,2), xpos, ypos+15, filters[2])
        for c in range(0,8):
            diverter_poly = [(0,0), (3,0), (3,17), (0,20)]
            diverter = self.add_dynamic_polygon(rotate_polygon(diverter_poly,-20), c*pitch+xpos, ypos, filterA)
            self.revolving_joint(bodyA=diverter, bodyB=attachment_body, anchor=(xpos+c*pitch, ypos))
            self.revolving_joint(bodyA=diverter, bodyB=conrod, anchor=(xpos+c*pitch, ypos+15))

        transfer_band_x = []

        for c in range(0,8):
            self.add_static_polygon(box_vertices(0, 0, 2,10),  c*pitch+xpos,    -12+ypos, filterA)
            self.add_static_polygon(box_vertices(0, 0, 2,35),  c*pitch+xpos+11, -12+ypos, filterA)
            self.add_static_polygon(box_vertices(0, 0, 11,3), c*pitch+xpos+2,  -12+ypos, filterA)
            transfer_band_x.append((c*pitch+xpos, c*pitch+xpos+11))

        if discard != 0:
            if discard>0:
                self.add_static_polygon([(0,0), (discard,-30), (discard,-33), (0,-3) ], xpos, ypos-11, filterB)
            else:
                self.add_static_polygon([(8*pitch,0), (discard,-30), (discard,-33), (8*pitch,-3) ], xpos, ypos-11, filterB)
        elif slope_x!=0:
            if slope_x < 0:
                offset = pitch
            else:
                offset = 0
        
            exit_transfer_band_x = []
            for c in range(start_at,8):
                exit_slope = polygonShape(vertices=[(0,0), (slope_x,-slope_y), (slope_x,-slope_y-3), (0,-3) ])
                self.add_static_polygon(exit_slope, c*pitch+xpos+offset, ypos-10, filter=filterB)
                exit_transfer_band_x.append((c*pitch+xpos+slope_x, c*pitch+xpos+slope_x+pitch))
            self.transfer_bands.append((ypos-10-slope_y+10, ypos-10-slope_y, exit_transfer_band_x, 1))

        self.add_static_circle(xpos-10, ypos+15, 5, filterA)
        self.add_static_circle(xpos+pitch*8-5, ypos+15, 5, filterA)
        self.add_static_polygon(box_polygon_shape(xpos-10, ypos-10, 3, 20))
        self.transfer_bands.append((-12+ypos+10, -12+ypos, transfer_band_x, 1 if inverted else 0))
        conrod.attachment_point = (xpos+pitch*8, ypos+15)

        crank_xpos = xpos+pitch*8+20
        if mirror:
            crank_xpos = xpos-50
        return_crank = self.crank_right_up(crank_xpos,ypos, attachment_body, weight=return_weight)
        self.distance_joint(return_crank, conrod)
        
        return conrod

    def regenerator(self, xpos, ypos, attachment_body, crank_list):
        regen_parts = []
        pusher_parts = []
        for c in range(0,8):
            self.add_static_polygon(box_vertices(0, 0, 7,10), c*pitch+xpos-11, -12+ypos)

            pusher = fixtureDef(shape=box_polygon_shape(c*pitch+xpos-11,-12+ypos+11,2,10), density=1.0,
                                      filter=filter(groupIndex=1))
            pusher_parts.append(pusher)

            bellcrank_fixtures = [ fixtureDef(shape=box_polygon_shape(c*pitch+xpos, -12+ypos+20, 10, 3), density=1.0, filter=filters[2]),
                                fixtureDef(shape=box_polygon_shape(c*pitch+xpos, -12+ypos+8, 3, 12), density=1.0) ]
            bellcrank = self.add_multifixture(bellcrank_fixtures)

            anchorpos = (xpos+c*pitch, -12+ypos+20)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos)
            crank_list.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            
        pusher_body = self.add_multifixture(pusher_parts)
        pusher_body.attachment_point = (xpos+8*pitch,ypos+10)
        self.slide_joint(pusher_body, attachment_body, (1,0), -8,0, friction=0)
        return pusher_body

    def toggle(self, xpos, ypos, attachment_body, toggle_joint_array=None):
        toggle_divider_polygon = [ (xpos-3, ypos), (xpos+3, ypos), (xpos, ypos+10) ]
        toggle_shape = [ fixtureDef(shape=box_polygon_shape(xpos-10, ypos, 20, 3), density=1.0, filter=filters[5]),
                         fixtureDef(shape=polygonShape(vertices=toggle_divider_polygon), density=1.0, filter=filters[5]) ]
        toggle = self.add_multifixture(toggle_shape)
        toggle_drive = self.revolving_joint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos), friction=1.0)
        self.all_toggle_drives.append(toggle_drive)
        if toggle_joint_array is not None:
            toggle_joint_array.append(toggle_drive)
        # Bit that goes under the toggle to stop it moving too far
        self.add_static_polygon(box_vertices(0, 0, 6,2), xpos-3, ypos-3)
        return toggle
        
    def subtractor_output_toggle(self, xpos, ypos, attachment_body):
        toggle = self.add_dynamic_polygon([(xpos-1.5,ypos), (xpos+1.5,ypos), (xpos, ypos+10)], 0,0, filter=filters[5])
        self.revolving_joint(bodyA=toggle, bodyB=attachment_body, anchor=(xpos,ypos))
        return toggle

    def translate_points(self, points, xpos, ypos):
        return [(x+xpos,y+ypos) for (x,y) in points]

    def subtractor(self, xpos, ypos, attachment_body, lines = 8, output_offset_dir = -1, discard_bands=False, toggle_joint_array=None, is_actually_adder=False, comparison_output=False):
        output_offset_x = pitch*(lines+1)*output_offset_dir
        sub_y_pitch = 20
        for c in range(0,lines):
            toggle_centre_x = xpos+c*pitch
            toggle_centre_y = ypos-(sub_y_pitch*lines)+10+sub_y_pitch*c
            input_toggle = self.toggle(toggle_centre_x, toggle_centre_y, attachment_body, toggle_joint_array)
            input_toggle.attachment_point = (toggle_centre_x, toggle_centre_y+10)
            output_toggle = self.subtractor_output_toggle(toggle_centre_x+output_offset_x, toggle_centre_y, attachment_body)
            output_toggle.attachment_point = (toggle_centre_x+output_offset_x, toggle_centre_y+10)
            self.distance_joint(input_toggle, output_toggle)

            # Dividers of output channels
            self.add_static_polygon([ (-1,0), (1,0), (1,sub_y_pitch*c+50), (-1,sub_y_pitch*c+50) ],
                                    xpos+(c+0.5)*pitch+output_offset_x, ypos+pitch-30-sub_y_pitch*lines, filter=filters[4])

            # Small triangular divider
            self.add_static_polygon([ (20,-6), (27,-3), (27,0) ],
                                    xpos+c*pitch-pitch+5, ypos+pitch+6-sub_y_pitch*(lines-c))

            # Baffles which slow down every even channel
            if c%2 == 0:
                self.add_static_polygon([ (pitch-10,-3), (pitch-7,-3), (pitch-7,-6), (pitch-10,-6) ],
                                        xpos+c*pitch-pitch+3.5, ypos+pitch+5, filter=filters[4])
                self.add_static_polygon([ (pitch-1,0), (pitch,0), (pitch,3), (pitch-1,3) ],
                                        xpos+c*pitch-pitch+3.5, ypos+pitch-10, filter=filters[4])

        for c in range(0,lines+1):
            # Large static bits that form input channels
            self.add_static_polygon([ (0,0), (pitch-7-2,-3), (pitch-7,-sub_y_pitch*(lines-c)-5), (0,-sub_y_pitch*(lines-c)-sub_y_pitch) ],
                                    xpos+c*pitch-pitch+3.5, ypos+pitch+9, filter=filters[4])


            
            # More top-side channels, but for the output
            self.add_static_polygon([ (0,0), (pitch-7,-5), (pitch-7,-sub_y_pitch*(lines-c)), (0,-sub_y_pitch*(lines-c)-sub_y_pitch) ],
                                    xpos+c*pitch-pitch+3.5+output_offset_x, ypos+pitch+9, filter=filters[4])
            # Bottom-side channels, for the output            
            if c != 0 or not comparison_output:
                self.add_static_polygon([ (-1,0), (1,0), (1,sub_y_pitch*c+10), (-1,sub_y_pitch*c+10) ],
                                        xpos+c*pitch+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines),filter=filters[4])

        # The leftmost output channel can be diverted to support 'negative detect' for the CMP operation.
        if comparison_output:
            leftmost_toggle = self.add_dynamic_polygon([ (-1,0), (1,0), (1,10), (-1,12) ],
                                                       xpos+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines),filter=filters[4])
            self.revolving_joint(attachment_body, leftmost_toggle, (xpos+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines)))
            leftmost_toggle.attachment_point=(xpos+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines)+10)
            self.comparison_diverter = leftmost_toggle
            self.add_static_polygon([ (-40,-20), (2,0), (2,2), (0,2) ],
                                    xpos+output_offset_x-3, ypos+pitch-30-sub_y_pitch*(lines)+10,filter=filters[4])
                
        # A reset bar
        reset_angle = math.atan2(sub_y_pitch, pitch)
        reset_len = math.sqrt((lines*pitch)**2 + (lines*sub_y_pitch)**2)
        reset_poly = rotate_polygon_radians(box_vertices(0, 0, reset_len, 5), reset_angle)
        if is_actually_adder:
            reset_lever = self.add_dynamic_polygon(polygonShape(vertices=reset_poly), xpos+10, ypos-sub_y_pitch*lines, filters[3])
        else:
            reset_lever = self.add_dynamic_polygon(polygonShape(vertices=reset_poly), xpos-216, ypos-sub_y_pitch*lines+10, filters[3])
        reset_lever.attachment_point=(xpos,ypos-180)
        self.slide_joint(attachment_body, reset_lever, (1,0), -20,20, friction=0)

        # Transfer bands in negative reader channels (discards)
        if discard_bands:
            transfer_band_x = [ (xpos+output_offset_x+pitch*x-12,xpos+output_offset_x+pitch*x) for x in range(1,lines) ]
            band_base_y = ypos-sub_y_pitch*lines
            self.transfer_bands.append((band_base_y+10, band_base_y, transfer_band_x, 0))

        return reset_lever
        
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

    def horizontal_injector(self, xpos, ypos, attachment_body):
        left = fixtureDef(shape=box_polygon_shape(0,0.5,10,6), density=1.0, filter=filters[0])
        right = fixtureDef(shape=box_polygon_shape(17,0.5,10,6), density=1.0, filter=filters[0])
        drive_rect = self.add_multifixture([left,right], xpos, ypos)
        drive_rect.attachment_point=(xpos+5, ypos+3.5)
        self.slide_joint(attachment_body, drive_rect, (1,0), 0, 10, friction=0)
        self.add_static_polygon(box_polygon_shape(0,-7,17,7), xpos, ypos)
        self.add_static_polygon(box_polygon_shape(0,7,10,20), xpos, ypos)# Blocks left
        self.add_static_polygon(box_polygon_shape(10+7,7,10,20), xpos, ypos)
        self.add_static_polygon(polygonShape(vertices=[(0,27), (10,27), (0,37)]), xpos, ypos)
        self.add_static_polygon(polygonShape(vertices=[(17,27), (27,27), (27,37)]), xpos, ypos)
        return drive_rect

    def single_injector(self, xpos, ypos, attachment_body, horizontal_drive=False):
        intake_angle = 30
        height = 8
        crank_offset = pitch-10
        crank_y = 19
        injector_bar_height = 90
        divider_height = 8
        height2 =        8
        divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,divider_height), (0,height2) ]
        divider_vertices = translate_polygon(divider_vertices, xpos, ypos+pitch+10)
        self.add_static_polygon(divider_vertices)
            
        # The base
        self.add_static_polygon([ (10,-20), (24,-20), (24,-15), (21,-13), (10,-15)], xpos, ypos+pitch+10)
            
        bellcrank_shape = [ fixtureDef(shape=box_polygon_shape(xpos+crank_offset, ypos+crank_y+9, 10, 3), density=1.0, filter=filters[1]),
                            fixtureDef(shape=box_polygon_shape(xpos+crank_offset, ypos+crank_y, 3, 12), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)) ]
        
        bellcrank = self.add_multifixture(bellcrank_shape)
        anchorpos = (xpos+crank_offset, ypos+crank_y+10)
        if horizontal_drive:
            bellcrank.attachment_point=(xpos+crank_offset, ypos+crank_y)
        else:
            bellcrank.attachment_point=(xpos+crank_offset+10, ypos+crank_y+10)
        self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos, friction=0)

        # Backstop for swing arm - stops the swing arm falling back too far
        self.add_static_polygon([ (10,-20), (11,-20), (11,-3), (10,-3)], xpos, ypos+pitch+10)
        
        # Thing that stops all the ball bearings rolling over the one in the crank
        self.add_static_polygon([(20.5,-6), (23,-6), (23,-3), (20.5,-3)], xpos+1, ypos+pitch+10)
        
        # roof
        #        roof_height=-20
        #        self.add_static_polygon([ (0,roof_height), (9*pitch,roof_height), (9*pitch,0), (0,0) ],
        #                                xpos, ypos+height+45)

        # End stop on the right
        self.add_static_polygon([ (0,0), (pitch-7,0), (pitch-7,height+30), (0,height) ], xpos+pitch, ypos+pitch+10)

        # End stop on the left
        self.add_static_polygon([ (0,0), (pitch-7,0), (pitch-7,height), (0,height+30) ], xpos, ypos+pitch+10)

        # Final channel guard on the right
        self.add_static_polygon([ (0,0), (3,0), (3,20), (0,20) ], xpos+pitch+10, ypos+pitch-10)
        
        return bellcrank

    def injector(self, xpos, ypos, attachment_body, injector_crank_array, columns=8):
        intake_angle = 30
        height = 8
        crank_offset = pitch-10
        crank_y = 19
        injector_bar_height = 90
        for c in range(0,columns):
            divider_height = 8
            height2 =        8
            divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,divider_height), (0,height2) ]
            divider_vertices = translate_polygon(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            self.add_static_polygon(divider_vertices)
            
        for c in range(0,columns):
            # The base
            self.add_static_polygon([ (10,-20), (24,-20), (24,-15), (21,-13), (10,-15)], xpos+c*pitch, ypos+pitch+10)
            
            bellcrank_shape = [ fixtureDef(shape=box_polygon_shape(c*pitch+xpos+crank_offset, ypos+crank_y+9, 10, 3), density=1.0, filter=filters[1]),
                                fixtureDef(shape=box_polygon_shape(c*pitch+xpos+crank_offset, ypos+crank_y, 3, 12), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)) ]

            bellcrank = self.add_multifixture(bellcrank_shape)
            anchorpos = (xpos+c*pitch+crank_offset, ypos+crank_y+10)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos, friction=0)
            injector_crank_array.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            # All-inject bar
            raiser = self.add_dynamic_circle(xpos+c*pitch+15, ypos+injector_bar_height+10, radius=5, density=50.0)
            self.slide_joint(attachment_body, raiser, (0,1), 0,5, friction=0)
            self.world.CreateDistanceJoint(bodyA=bellcrank,
	                              bodyB=raiser,
                                      anchorA=((anchorpos[0]+5)*self.scale,(anchorpos[1]-10)*self.scale),
	                              anchorB=raiser.worldCenter,
	                                   collideConnected=False)
        raiser_bar = self.horizontal_rotating_bar(xpos,ypos+injector_bar_height, pitch*9, attachment_body, support_sep=pitch*8)
        
        for c in range(0,columns+1):
            
            # Backstop for swing arm - stops the swing arm falling back too far
            self.add_static_polygon([ (10,-20), (11,-20), (11,-3), (10,-3)], xpos+c*pitch, ypos+pitch+10)

            # Thing that stops all the ball bearings rolling over the one in the crank
            self.add_static_polygon([(20.5,-6), (23,-6), (23,-3), (20.5,-3)], xpos+c*pitch+1, ypos+pitch+10)


        # roof
        #        roof_height=-20
        #        self.add_static_polygon([ (0,roof_height), (9*pitch,roof_height), (9*pitch,0), (0,0) ],
        #                                xpos, ypos+height+45)

        # End stop on the right
        self.add_static_polygon([ (0,0), (pitch-7,0), (pitch-7,height+30), (0,height) ], xpos+columns*pitch, ypos+pitch+10)

        # End stop on the left
        self.add_static_polygon([ (0,0), (pitch-7,0), (pitch-7,height), (0,height+30) ], xpos, ypos+pitch+10)

        return raiser_bar
        
    def add_ball_bearing(self, xpos, ypos, plane):
        bearing = self.add_dynamic_circle(xpos, ypos, 6.35/2, density=5.0, filter=filters[plane])
        self.ball_bearings.append((bearing,plane))

    def add_row_decoder(self, xpos, ypos, groundBody, follower_array, selector_array):
        for selector_no in range(0,selector_rods):
            row_selector_fixtures = []
            for row in range(0,8):
                enabled = (row >> (selector_rods-selector_no-1)) & 1
                row_selector_fixtures.append(
                    fixtureDef(shape=box_polygon_shape(14+25*selector_no,follower_spacing*row+7*enabled,14,5), density=1.0,
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
        memory_fixed_shapes = []
        for row in range(0,8):
            for col in range(0,8):
                memory_fixed_filter = filter(groupIndex=0, categoryBits=0x0001, maskBits=0xFFFF)
                large_block = [(0,0), (14,0), (14,7), (0,6)]
                self.add_static_polygon(large_block, 22*col, 14*row, filter=memory_fixed_filter)
                self.add_static_polygon(box_vertices(0, 0, 3,8), 22*col+14-3+1, 14*row+6, filter=memory_fixed_filter)
                pass

        row_injector_fixtures = []        
        
        for col in range(0,8):
            row_injector_fixtures.append(fixtureDef(shape=box_polygon_shape(7+22*col,1,1,5), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
            row_injector_fixtures.append(fixtureDef(shape=box_polygon_shape(7+22*col+10,1,3,5), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=box_polygon_shape(22*8+12,0,7,7), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        row_injector_fixtures.append(fixtureDef(shape=box_polygon_shape(-10,0,3,12), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        self.memory_col0_x = 7
        self.memory_row0_y = -30
        injectors=[]
        for col in range(0,8):
            injector = self.add_multifixture(row_injector_fixtures, 0, 7+14*col)
            injectors.append(injector)
            self.slide_joint(injector, groundBody, (1,0), 7, 17, friction=0.1)
            
        row_ejector_fixtures = []
        for col in range(0,8):
            # Blocks which stop the ball bearing falling past
            ejector = fixtureDef(shape=polygonShape(vertices=[(22*col+1,3),(22*col+15,3),(22*col+15,7), (22*col+2,7), (22*col+1,6)]), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE))
            row_ejector_fixtures.append(ejector)
        row_ejector_fixtures.append(fixtureDef(shape=box_polygon_shape(22*8+14,1,3,11), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
        
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
        (selector_holdoff, follower_holdoff) = self.add_row_decoder(xpos+50, ypos, groundBody, self.memory_followers, self.memory_selectors)
        
        # Rods which connect row selectors to ejectors
        for r in range(0,memory_rows):
            self.distance_joint(ejectors[r], self.memory_followers[r], (200,14*r+10), (200+35, 14*r+10))

        # Gate returner to push all the memory rows back in
        self.memory_returning_gate = self.vertical_rotating_bar(xpos-32,0,14*memory_rows, groundBody)

        # Wall on the left of the memory to create the final channel
        memory_fixed = self.add_static_polygon(box_vertices(0, 0, 3,14*8), -10,0, filter=filter(groupIndex=0, categoryBits=0x0001, maskBits=0xFFFF))
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
        crank_fixture_1 = fixtureDef(shape=box_polygon_shape(-20,0,20,3), density=1.0, filter=filters[0])
        crank_fixture_2 = fixtureDef(shape=box_polygon_shape(0,0,3,output_length), density=1.0, filter=filters[0])
        crank_fixture_3 = fixtureDef(shape=box_polygon_shape(-20,-5,10,10), density=weight, filter=filters[0]) # Heavy weight
        crank = self.add_multifixture([crank_fixture_1, crank_fixture_2, crank_fixture_3], xpos, ypos)
        self.revolving_joint(crank, attachment_body, (xpos,ypos))
        crank.attachment_point=(xpos,ypos+output_length)
        return crank

    def crank_right_up(self, xpos, ypos, attachment_body, output_length=20, weight=10):
        crank_fixture_1 = fixtureDef(shape=box_polygon_shape(0,0,20,3), density=1.0, filter=filters[0])
        crank_fixture_2 = fixtureDef(shape=box_polygon_shape(0,0,3,output_length), density=1.0, filter=filters[0])
        crank_fixture_3 = fixtureDef(shape=box_polygon_shape(10,-5,10,10), density=weight, filter=filters[0]) # Heavy weight
        crank = self.add_multifixture([crank_fixture_1, crank_fixture_2, crank_fixture_3], xpos, ypos)
        self.revolving_joint(crank, attachment_body, (xpos,ypos))
        crank.attachment_point=(xpos,ypos+output_length)
        return crank

    
    def memory_sender(self, xpos, ypos, attachment_body):
        v1 = [ (0,0), (7,-4), (7,-17), (0,-13) ]
        entrace_poly = [ (-7,2), (0,0), (0,-5), (-7,0) ]
        sensor_poly = [ (3,0), (7,-4), (7,-11), (0,-7) ]
        blocker_poly = [ (7,0), (10,-4), (10,-8), (7,-4) ]
        self.memory_sensors = []
        blocker_fixtures = []
        for c in range(0,8):
            self.add_static_polygon(v1, xpos+c*pitch, ypos)
            self.add_static_polygon(entrace_poly, xpos+c*pitch, ypos)

            
            sensor = self.add_dynamic_polygon(sensor_poly, xpos+c*pitch, ypos+7, filter=filters[0])
            self.memory_sensors.append(sensor)
            self.slide_joint(attachment_body, sensor, (0,1), 0, 15, friction=False)
            blocker_fixtures.append(fixtureDef(shape=polygonShape(vertices=translate_polygon(blocker_poly,c*pitch,0)), density=1.0, filter=filters[0]))
        blocker_set = self.add_multifixture(blocker_fixtures, xpos, ypos)
        self.slide_joint(attachment_body, blocker_set, (1,0), 0, 15, friction=False)
        
        crank = self.crank_left_up(xpos-30, ypos-20, attachment_body)
        self.distance_joint(crank, blocker_set, crank.attachment_point, (xpos+10,ypos))
        blocker_set.attachment_point=(xpos+c*pitch,ypos+7)
        return blocker_set
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
        translated_vertices = translate_polygon(vertices, xpos, ypos)
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in translated_vertices])
        fixture = fixtureDef(shape=shape, density=1.0, filter=filter)
        return self.world.CreateStaticBody(fixtures=fixture)

    def add_static_circle(self, xpos, ypos, radius, filter=filters[0]):
        return self.world.CreateStaticBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale, ypos*self.scale)), filter=filter))

    def add_dynamic_circle(self, xpos, ypos, radius, density=1.0, filter=filters[0]):
        circle = self.world.CreateDynamicBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale,ypos*self.scale)), density=density, filter=filter))
        circle.attachment_point = (xpos*self.scale, ypos*self.scale)
        return circle
        
    def add_dynamic_polygon(self, vertices, xpos, ypos, filter=filters[0], density=1.0):
        translated_vertices = translate_polygon(vertices, xpos, ypos)
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in translated_vertices])
        fixture = fixtureDef(shape=shape, density=density, filter=filter)
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
                                           maxMotorTorque = 10000000.0*force,
                                           motorSpeed = motor,
                                           enableMotor = True)
        elif friction:
            j = self.world.CreateRevoluteJoint(bodyA=bodyA, bodyB=bodyB, anchor=(x*self.scale, y*self.scale),
                                           maxMotorTorque = 10000.0*force,
                                           motorSpeed = 0.0,
                                           enableMotor = True)
        else:
            j = self.world.CreateRevoluteJoint(bodyA=bodyA, bodyB=bodyB, anchor=(x*self.scale, y*self.scale), enableMotor = True)
        return j

    # End of interface functions

    def connect_memory(self):
        """ Connects the memory selectors to the memory senders """
        for i in range(0,len(self.memory_selectors)):
            bodyA = self.memory_selectors[i]
            bodyB = self.memory_sensors[i+5]
            self.world.CreateDistanceJoint(bodyA=bodyA,
	                                   bodyB=bodyB,
	                                   anchorA=bodyA.worldCenter,
	                                   anchorB=bodyB.worldCenter,
	                                   collideConnected=False)

        for i in range(0,len(self.rom_selectors)):
            bodyA = self.rom_selectors[i]
            bodyB = self.memory_sensors[i]
            self.world.CreateDistanceJoint(bodyA=bodyA,
	                                   bodyB=bodyB,
	                                   anchorA=bodyA.worldCenter,
	                                   anchorB=bodyB.worldCenter,
	                                   collideConnected=False)

            
    def add_cam(self, xpos, ypos, follower_len, bumps=[], horizontal=False, reverse_direction=False, axis_offset=0, axis=True, bump_height=3):
        """ Very basic function which just adds a motorised circle with a bump.
        phase is between 0 and 1 and adjusts initial rotation. 
        horizontal/vertical: Vertical means the output moves in a vertical direction, which means the follower is on top of the cam.
        reverse_direction puts the cam on the other side and only makes sense for horizontal cams.
        """
        attachment_body = self.groundBody
        offset = 0
        if not horizontal:
            offset = -0.25
        if reverse_direction:
            offset = 0.5
        for b in range(0,len(bumps)):
            bumps[b] = (bumps[b][0] + offset, bumps[b][1])
        radius = 30
        bump_height = radius+bump_height
        disc_fixture = fixtureDef(shape=circleShape(radius=radius, pos=(0,0)),density=1.0,filter=filters[0])
        bump_fixtures = []
        for (start, length) in bumps:
            ang = start
            bump_points = [( radius*math.cos(-ang*math.pi*2), radius*math.sin(-ang*math.pi*2)) ]
            points = 1
            # Max points in a polygon is 15 at the moment.
            while ang < (start+length) and points < 15:
                ang += 0.02
                bump_points.append( ( bump_height*math.cos(-ang*math.pi*2), bump_height*math.sin(-ang*math.pi*2)) )
                points += 1
                if points == 15:
                    print("WARNING: Max points reached in cam bump; %2.2d%% of cam complete"%(100*(ang-start)/length))

            bump_points.append(( radius*math.cos(-(ang+0.01)*math.pi*2), radius*math.sin(-(ang+0.01)*math.pi*2)))
            bump_fixtures.append(fixtureDef(shape=polygonShape(vertices=bump_points),density=0.0,filter=filters[0]))
        cam_body = self.add_multifixture([disc_fixture] + bump_fixtures, xpos, ypos)
        cam_driver = self.revolving_joint(attachment_body, cam_body, (xpos,ypos), motor=1, force=50)
        cam_driver.motorSpeed = 0
        follower_filter = filters[1]
        if axis:
            if horizontal:
                axle_y = ypos+radius
                if reverse_direction:
                    axle_x = xpos-radius-axis_offset-5
                else:
                    axle_x = xpos+radius+axis_offset+2.5
                follower_body = self.add_dynamic_polygon(box_polygon_shape(axle_x, axle_y-follower_len, 5, follower_len), 0, 0, filter=follower_filter)
                follower_wheel = self.add_dynamic_circle(axle_x+2.5, axle_y-radius, 5)
                self.revolving_joint(follower_wheel, follower_body, (axle_x+2.5,axle_y-radius), friction=False)
            else:
                axle_y = ypos+radius+axis_offset+2.5
                axle_x = xpos-radius
                follower_body = self.add_dynamic_polygon(box_polygon_shape(axle_x, axle_y, follower_len, 5), 0, 0, filter=follower_filter)
                follower_wheel = self.add_dynamic_circle(axle_x+radius, axle_y+2.5, 5)
                self.revolving_joint(follower_wheel, follower_body, (axle_x+radius,axle_y+2.5), friction=False)

            if horizontal:
                follower_body.attachment_point=(axle_x, axle_y-follower_len)
            else:
                follower_body.attachment_point=(axle_x+follower_len, axle_y)
            print("Setting attachment point: (x,y)={}".format(follower_body.attachment_point))
            self.revolving_joint(attachment_body, follower_body, (axle_x+2.5,axle_y+2.5), friction=False)
            print("Creating cam: xpos= {}, ypos= {}, axle_x = {} ,axle_y= {}, follower_len={}".format(xpos, ypos, axle_x, axle_y, follower_len))

        self.all_cam_drives.append(cam_driver)
        if axis:
            return follower_body
        else:
            return None

    def add_instruction_cranks(self, attachment_body, xpos, ypos):
        len1 = 25
        len2 = 25
        thickness = 2.5
        andgate_spacing_y = 30
        self.instruction_inputs = []
        self.instruction_outputs = []
        # Reverse output for LDN and CMP
        reversed_outputs = [ False, True, False, False, False, True, False, False ]
        for i in range(0,8):
            crank_polygon1 = [ (-thickness,-thickness), (-thickness,-len1-thickness), (thickness,-len1-thickness), (thickness,-thickness) ]
            crank_polygon2 = [ (-thickness,-thickness), (-thickness,thickness), (len2+thickness,thickness), (len2+thickness,-thickness) ]
            crank = self.add_multipolygon([crank_polygon1, crank_polygon2], xpos+i*30, ypos-i*follower_spacing)
            self.revolving_joint(attachment_body, crank, (xpos+i*30,ypos-i*follower_spacing))
            self.world.CreateDistanceJoint(bodyA=crank, bodyB=self.rom_followers[7-i], anchorA=((xpos+i*30)*self.scale, (ypos-i*follower_spacing-len1)*self.scale), anchorB=self.rom_followers[7-i].worldCenter, collideConnected=False)
            block = self.add_dynamic_polygon([ (0,0), (30,0), (28,5), (2,5) ], xpos+i*30, ypos-i*andgate_spacing_y-50)
            offset = 30 if reversed_outputs[i] else 0
            # A hack: pushing 0 (JMP) also pushes 1 (JRE), but not vice versa.
            if i==7:
                block_slider_base = fixtureDef(shape=box_polygon_shape(0,0,30,5), filter=filters[0])
                block_slider_connector = fixtureDef(shape=box_polygon_shape(-5,0,10,20), filter=filters[0])
                block_slider = self.add_multifixture([block_slider_base, block_slider_connector], xpos+i*30-30, ypos-i*andgate_spacing_y-50)
            elif i==6:
                block_slider_base = fixtureDef(shape=box_polygon_shape(0,0,30,5), filter=filters[0])
                block_slider_connector = fixtureDef(shape=box_polygon_shape(15,-20,6,20), filter=filters[0])
                block_slider = self.add_multifixture([block_slider_base, block_slider_connector], xpos+i*30-30, ypos-i*andgate_spacing_y-50)
            else:
                block_slider = self.add_dynamic_polygon(box_vertices(0, 0, 30,5), xpos+i*30-30+offset*2, ypos-i*andgate_spacing_y-50)
            self.revolving_joint(block_slider, block, (xpos+i*30+offset, ypos-i*andgate_spacing_y-50))
            self.slide_joint(attachment_body, block_slider, (1,0), 0 if reversed_outputs[i] else -20,20 if reversed_outputs[i] else 0, friction=0)
            block_slider.attachment_point = (xpos+i*30-30, ypos-i*andgate_spacing_y-50)
            self.instruction_outputs.append(block_slider)
            pusher_slider = self.add_dynamic_polygon(box_vertices(0, 0, 30,5), xpos+i*30+30-offset*2, ypos-i*andgate_spacing_y-68)
            pusher_slider.attachment_point = (xpos+i*30+60, ypos-i*andgate_spacing_y-70)
            self.instruction_inputs.append(pusher_slider)
            self.slide_joint(attachment_body, pusher_slider, (1,0), 0 if reversed_outputs[i] else -20,20 if reversed_outputs[i] else 0, friction=0)
            self.world.CreateDistanceJoint(bodyA=crank, bodyB=block, anchorA=((xpos+i*30+len2)*self.scale, (ypos-i*follower_spacing)*self.scale), anchorB=block.worldCenter, collideConnected=False)
        self.instruction_inputs.reverse()
        self.instruction_outputs.reverse()

    def ball_bearing_block(self, xpos, ypos,cols):
        for r in range(0,5):
            for col in range(0,cols):
                test_data = self.add_ball_bearing(xpos+7*col+r%2,ypos+7*r,0)
        
    def set_initial_memory(self, memory_array):
        for x in range(0,8):
            for y in range(0,8):
                if (memory_array[y] & 1<<x):
                    self.add_ball_bearing(self.memory_col0_x + pitch*(7-x)+2, self.memory_row0_y + 14*y+41, 0)

    def setup_ssem(self):
        """ Sets up all the parts of the SSEM except cams. """
        
        groundBox = box_polygon_shape(-20,-500,1,1) # A tiny box that just acts as the ground body for everything else
        groundBody = self.world.CreateStaticBody(shapes=groundBox)
        self.groundBody = groundBody
        # Initial charge for main injector
        self.ball_bearing_block(0,190,cols=16)
        self.injector_cranks = []
        self.parts.main_injector_raiser = self.injector(-32,150, groundBody, injector_crank_array=self.injector_cranks)

        self.parts.accumulator_diverter_lever = self.diverter_set(0,130, groundBody, slope_x=-240, slope_y=180, return_weight=10) # Diverter 1. Splits to subtractor reader.

        (self.parts.memory_selector_holdoff, self.parts.memory_follower_holdoff) = self.memory_module(0,0, groundBody)
        self.upper_regenerators = []

        self.parts.discard_lever_2 = self.diverter_set(-5,-30, groundBody, discard=500) # Diverter 2a. Discard reader-pulse data.
        self.parts.upper_regen_control = self.regenerator(-5,-65, groundBody, self.upper_regenerators) # Regenerator 1. For regenning anything read from memory.
        self.parts.ip_diverter_lever = self.diverter_set(-5,-95, groundBody, slope_x=352, slope_y=170, start_at=3, return_weight=5) # Diverter 1. Splits to instruction counter.
        self.parts.diverter_3 = self.diverter_set(-10,-158, groundBody, slope_x=208, slope_y=310) # Diverter 3; splits to instruction reg/PC
        
        # PC injector
        self.pc_injector_cranks = []    
        self.parts.pc_injector_raiser = self.injector(230,-290, groundBody, injector_crank_array=self.pc_injector_cranks, columns=5)
        # Initial charge for PC injector
        self.ball_bearing_block(250,-240,cols=8)


        sub_pos_x = -15
        sub_pos_y = -200
        self.parts.accumulator_reset_lever = self.subtractor(sub_pos_x,sub_pos_y, groundBody, discard_bands=True, toggle_joint_array=self.accumulator_toggles, comparison_output=True)
        skip_lever_x = sub_pos_x - 200
        skip_lever_y = sub_pos_y - 200
        a = fixtureDef(shape=polygonShape(vertices=[(-50,-32), (0,-30), (0,-25), (-50,-30)]), filter=filters[0], density=1.0)
        b = fixtureDef(shape=box_polygon_shape(0,-30,5,30), filter=filters[0], density=1.0)
        c = fixtureDef(shape=box_polygon_shape(-30,-30,5,30), filter=filters[0], density=1.0)
        d = fixtureDef(shape=box_polygon_shape(0,0,300,5), filter=filters[2], density=1.0)
        e = fixtureDef(shape=box_polygon_shape(285,-15,15,15), filter=filters[2], density=2.10)
        f = fixtureDef(shape=box_polygon_shape(150,-50,5,50), filter=filters[0], density=1.0)
        skip_lever=self.add_multifixture([a,b,d,e,f], skip_lever_x, skip_lever_y)
        skip_lever.attachment_point = (skip_lever_x+150,skip_lever_y-50)
        #skip_lever = self.add_dynamic_polygon(polygonShape(vertices=box_vertices(0, 0, 300,5)), skip_lever_x, skip_lever_y, filter=filters[2])
        self.revolving_joint(groundBody, skip_lever, (skip_lever_x+150, skip_lever_y+2.5), friction=0)
        self.add_static_polygon(polygonShape(vertices=box_vertices(0, 0, 10,10)), skip_lever_x+270, skip_lever_y-15, filter=filters[2])
        self.parts.cmp_injector = self.horizontal_injector(skip_lever_x-48,skip_lever_y+257, groundBody)
        self.ball_bearing_block(skip_lever_x-30,skip_lever_y+280,cols=1)
        self.add_static_polygon(polygonShape(vertices=[(0,0), (20,0), (0,20)]), skip_lever_x-30,skip_lever_y+230)
                            
        self.lower_regenerators = []
        self.parts.lower_regen_control = self.regenerator(-203,-380, groundBody, self.lower_regenerators)
        #Program counter
        self.parts.pc_reset_lever = self.subtractor(400,-320, groundBody, lines=5, toggle_joint_array=self.ip_toggles, is_actually_adder=True)
        # Thing that adds one ball bearing to the PC
        pc_incrementer = self.horizontal_injector(457,-250, groundBody)
        self.ball_bearing_block(457+20,-250+30,cols=1)
        self.distance_joint(skip_lever, pc_incrementer)

        self.connect_regenerators()

        self.add_static_polygon([ (-300,-600),(500,-550), (500,-610), (-300,-610)])
        self.add_static_polygon([ (-400,-550),(-310,-600), (-310,-610), (-400,-610)])

        # Instruction decoder ROM
        self.rom_followers = []
        self.rom_selectors = []
        (self.parts.instruction_selector_holdoff, self.parts.instruction_follower_holdoff) = self.add_row_decoder(200, 0, groundBody, self.rom_followers, self.rom_selectors)

        self.add_instruction_cranks(groundBody, 550, 140)
       
        #self.ball_bearing_lift(-200,-400,groundBody)
        self.parts.sender_eject = self.memory_sender(200,-500, groundBody)
        self.connect_memory()

    def basic_cam(self, x, y, arm_length, bumps, axis_offset=0, attachment_part):
        follower_body = self.add_cam(x,y,arm_length, bumps=bumps, axis_offset=axis_offset)
        self.distance_joint(follower_body, attachment_part)
        
    def setup_cams(self):

        groundBody = self.groundBody


        # Cams

        # Cam 1: Fires PC injector, reading PC into address reg.
        basic_cam(300,200, 100, bumps=[(0.0,0.02)], axis_offset=1, self.parts.pc_injector_raiser)
        # Cam 2: Main memory selector lifter
        basic_cam(150,300, 150, bumps=[(0.1,0.02), (0.32, 0.06)], attachment_part=self.parts.memory_selector_holdoff)

        # Cam 2: Memory returner (left side)
        follower_body = self.add_cam(-400,120, 100, horizontal=True, bumps=[(0.05, 0.04), (0.31,0.1), (0.63,0.1), (0.93,0.05)], axis_offset=-1)
        self.distance_joint(follower_body, self.memory_returning_gate)

        # Cam 4: Memory holdoff (right side)
        follower_body = self.add_cam(-300,100, 100, horizontal=True, bumps=[(0.08,0.06), (0.17,0.05), (0.31,0.1), (0.48,0.05), (0.64,0.1), (0.94,0.05)], axis_offset=-1)
        self.distance_joint(follower_body, self.parts.memory_follower_holdoff)

        # Cam 5: Regenerator 1
        follower_body = self.add_cam(800, 100, 80, horizontal=True, bumps=[(0.25,0.05), (0.56,0.05)])
        self.distance_joint(follower_body, self.parts.upper_regen_control)

        # Cam 6: Split to instruction counter/reg
        follower_body = self.add_cam(400,-120, 60, horizontal=True, reverse_direction=True, axis_offset=2, bumps=[(0.18, 0.12)])
        self.distance_joint(follower_body, self.parts.diverter_3)

        # Cam 7: Instruction selector holdoff
        follower_body = self.add_cam(320, 300, 150, bumps=[(0.32,0.06)])
        self.distance_joint(follower_body, self.parts.instruction_selector_holdoff)

        # Cam 8: Sender eject.
        # Note timing hazard. We cannot raise selector and eject until
        # regenerated data is written back, so we delay for a few
        # seconds here.  If gravity or timing changes, expect this to
        # break.
        follower_body = self.add_cam(600, -430, 80, bumps=[(0.30,0.02)], horizontal=True)
        self.distance_joint(follower_body, self.parts.sender_eject)

        instruction_ready_point = 0.50
        
        # Cam 9: Resets accumulator on LDN.
        follower_body = self.add_cam(850, 0, 120, bumps=[(instruction_ready_point,0.05)], horizontal=True, reverse_direction=False, axis_offset=-1)
        self.distance_joint(follower_body, self.instruction_inputs[LDN])
        # Attach LDN instruction output to reset bar
        self.distance_joint(self.parts.accumulator_reset_lever, self.instruction_outputs[LDN])

        # Cam 11: Instruction follower holdoff
        follower_body = self.add_cam(1000, 100, 100, bumps=[(0.15,0.25)], horizontal=True, axis_offset=-1)
        self.distance_joint(follower_body, self.parts.instruction_follower_holdoff)
        
        # Cam 12: Fires main memory injector, injecting all 8 columns. If STO is on, this diverts to the subtractor reader. If not, it
        # will fall through the memory and be discarded.
        follower_body = self.add_cam(0,300, 100, bumps=[(0.62,0.02)], axis_offset=1)
        self.distance_joint(follower_body, self.parts.main_injector_raiser)

        # Cam 13: Discard everything after main injector fires for all cases.
        #follower_body = self.add_cam(900, 200, 100, bumps=[(0.6,0.2)], horizontal=True, reverse_direction=True, axis_offset=3)
        #self.distance_joint(follower_body, discard_lever_2)

        # Cam 14: Divert to subtractor reader on STO.
        # Also diverts the regenerator output on STO; we must separately discard that.
        follower_body = self.add_cam(1000, 0, 100, bumps=[(0.49,0.2)], horizontal=True, reverse_direction=True, axis_offset=2, bump_height=3.5)
        self.distance_joint(follower_body, self.instruction_inputs[STO])
        self.distance_joint(self.parts.accumulator_diverter_lever, self.instruction_outputs[STO])
        self.distance_joint(self.parts.discard_lever_2, self.instruction_outputs[STO])

        # Cam 15: Divert to instruction pointer, on JRP (and JMP via the same lever).
        follower_body = self.add_cam(1100, 0, 100, bumps=[(0.5,0.2)], horizontal=True, reverse_direction=True, axis_offset=2)
        self.distance_joint(follower_body, self.instruction_inputs[JRP])
        self.distance_joint(self.parts.ip_diverter_lever, self.instruction_outputs[JRP])

        # Cam 16: Secondary discard, of any data falling through the memory just after main inject
        follower_body = self.add_cam(-500,-150, 80, bumps=[(0.67,0.07)], reverse_direction=True, horizontal=True, axis_offset=2)
        self.distance_joint(follower_body, self.parts.discard_lever_2)

        # Cam 17: Fires bottom regenerator (usually empty, unless STO is on)
        follower_body = self.add_cam(-500,-300, 80, bumps=[(0.87,0.02)], horizontal=True, axis_offset=0)
        self.distance_joint(follower_body, self.parts.lower_regen_control)


        # Cam 18: Reset PC on JMP
        follower_body = self.add_cam(1230, 0, 140, bumps=[(0.5,0.1)], horizontal=True, reverse_direction=True, axis_offset=2)
        self.distance_joint(follower_body, self.instruction_inputs[JMP])
        self.distance_joint(self.parts.pc_reset_lever, self.instruction_outputs[JMP])

        # Cam 19: Runs CMP.
        follower_body = self.add_cam(900,200, 120, bumps=[(instruction_ready_point,0.05)], horizontal=True, reverse_direction=False, axis_offset=-1)
        self.distance_joint(follower_body, self.instruction_inputs[CMP])
        self.distance_joint(self.comparison_diverter, self.instruction_outputs[CMP])
        self.distance_joint(self.parts.cmp_injector, self.instruction_outputs[CMP])

        # Cam 20: Inc PC.
        follower_body = self.add_cam(-95,-450, 60, bumps=[(0.85,0.05)], horizontal=True, reverse_direction=False, axis=False, bump_height=5)

        
        # Notable timing points:
        # 0.31: Memory at PC has been read and regenerated

    def __init__(self, testmode, test_set_no):
        super(Memory, self).__init__()
        self.test_set_no = test_set_no
        self.test_set = test_set[self.test_set_no]
        self.parts = Parts()
        if testmode:
            self.test_mode = True
            print("Running test {}".format(test_set_no))
            settle_delay = 400
            self.start_point = random.randint(settle_delay,settle_delay+100)
            print("Running test {} starting at tick {}".format(testmode, self.start_point))
            self.name="SSEM - {}".format(self.test_set.get("name", "Automated test"))
        else:
            # Use the test data from the specified test, but don't go into testmode.
            self.test_mode = False
            print("Starting in interactive mode")
            self.start_point = 0
        self.initial_accumulator =  self.test_set["initial_accumulator"] if "initial_accumulator" in self.test_set else 0
        print("Setting initial accumulator to {}".format(self.initial_accumulator))
        self.accumulator_toggles = []
        self.ip_toggles = []
        self.cams_on = False
        self.all_cam_drives = []
        self.all_toggle_drives = []
        self.scale = 1.0
        self.transfer_bands = []
        self.ball_bearings = []
        self.sequence = 0 # Like step count but only increments when cams are on
        self.init_pulse = 0 # A small counter for use at startup to reset the toggles

        self.setup_ssem()
        self.setup_cams()
        
        self.set_initial_memory(self.test_set["initial_memory"])

    def read_pc_array(self):
        return [1 if i.angle>0 else 0 for i in self.ip_toggles]

    def read_accumulator_array(self):
        return [0 if i.angle>0 else 1 for i in self.accumulator_toggles]
        
    def read_pc_value(self):
        bits = reversed(self.read_pc_array())
        total = 0
        val = 1
        for b in bits:
            total += b * val
            val <<= 1
        return total
    
    def read_accumulator_value(self):
        bits = reversed(self.read_accumulator_array())
        total = 0
        val = 1
        for b in bits:
            total += b * val
            val <<= 1
        return total

    def read_memory_array(self):
        memory = []
        for row in range(0,8):
            row_val = 0
            for col in range(0,8):
                for i in range(0,len(self.ball_bearings)):
                    (b, plane) = self.ball_bearings[i]
                    (x,y) = b.worldCenter
                    x /= self.scale
                    y /= self.scale
                    
                    expected_pos_x = self.memory_col0_x + pitch*(7-col)+2
                    expected_pos_y = self.memory_row0_y + 14*row+41
                    dx = expected_pos_x - x
                    dy = expected_pos_y - y
                    if abs(dx)<5 and abs(dy)<5:
                        row_val += 1<<col
            memory.append(row_val)
        return memory
    
    def verify_results(self):
        expected_accumulator = self.test_set.get("expected_accumulator", self.initial_accumulator)
        expected_pc = self.test_set.get("expected_pc", 1)
        
        accumulator = self.read_accumulator_value()
        if expected_accumulator != accumulator:
            print("FAIL: Expected accumulator {}, actual result {}".format(expected_accumulator, accumulator))
            return
        expected_memory = copy.copy(self.test_set["initial_memory"])
        if "memory_update" in self.test_set:
            (address, value) = self.test_set["memory_update"]
            expected_memory[address] = value

        pc = self.read_pc_value()
        if expected_pc != pc:
            print("FAIL: Expected PC {}, actual result {}".format(expected_pc, pc))
            return

        memory = self.read_memory_array()
        for a in range(0,8):
            if expected_memory[a] != memory[a]:
                print("FAIL: At address {}, expected memory {} but found {}".format(a,expected_memory[a], memory[a]))
                return
        
        print("PASS")

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
        if self.init_pulse < 25:
            bit = 0
            for d in self.accumulator_toggles:
                d.motorSpeed = 10 if (self.initial_accumulator & 1<<(7-bit))==0 else -10
                bit += 1
            for d in self.ip_toggles:
                d.motorSpeed = -10
        elif self.init_pulse < 50:
            for d in self.all_toggle_drives:
                d.motorSpeed = 0
        elif self.init_pulse == 100:
            print("Initialization complete")
        elif self.init_pulse == self.start_point:
            self.cams_on = True
        self.init_pulse += 1
        
        if self.cams_on: self.sequence += 1
        angleTarget = (self.sequence*math.pi*2/10000.0)
        if self.sequence % 100 == 0 and self.cams_on:
            print("Sequence {} AngleTarget = {} degrees timing = {}".format(self.sequence, 360*angleTarget/(math.pi*2), angleTarget/(math.pi*2)))
            print("Accumulator = {} ({}) PC= {} ({})".format(",".join(map(str,self.read_accumulator_array())), self.read_accumulator_value(), self.read_pc_array(), self.read_pc_value()))
            print("Memory = {}".format(",".join(map(str, self.read_memory_array()))))
        if angleTarget >= (math.pi*2) and self.cams_on:
            angleTarget = math.pi*2
            self.cams_on = False
            print("Sequence complete; cams off")
            self.verify_results()
            if self.test_mode:
                self.stopFlag = True
            
        for d in self.all_cam_drives:
            angleError = d.angle - angleTarget
            gain = 1.0
            d.motorSpeed = (-gain * angleError)

    def Keyboard(self, key):
        print("Processing key: {}".format(key))
        if key == Keys.K_r:
            self.cams_on = not self.cams_on

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('testset', type=int)
    args = parser.parse_args()
    main(Memory(args.test, args.testset))
