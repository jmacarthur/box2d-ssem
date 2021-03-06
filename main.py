#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Jim MacArthur
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.


import argparse
import copy
import math
import random
import sys

from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, filter)
from Box2D import b2CircleShape
from constants import *
from test_sets import test_set
from emulator import SSEM_State
from settings import fwSettings
from cams import cams
def box_vertices(x, y, width,height):
    return [(x,y), (x+width,y), (x+width,y+height), (x,y+height)]

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

# Error codes - return value of main
SUCCESS = 0
WRONG_ACCUMULATOR = 1
WRONG_IP = 2
WRONG_MEMORY = 3
UNSUPPORTED_OP = 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--randomtest', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--overlay', action='store_true')
    parser.add_argument('--timelapse', type=int, default=0)
    parser.add_argument('testset', type=int, default=0, nargs='?')
    args = parser.parse_args()
    if args.headless:
        from backends.headless_framework import HeadlessFramework as Framework
    else:
        from backends.modified_pygame_framework import ModifiedPygameFramework as Framework

from framework import (main, Keys)
    
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
        self.memory_follower_holdoff = None

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
            bar_body.attachment_point=(4.5, height/3)
            bar_body.origin=(xpos, ypos)
            return bar_body
        else:
            body = self.add_dynamic_polygon(box_vertices(0, 0, 3,height), xpos,ypos, filters[0])
            body.attachment_point = (1.5,height/2)
            body.origin = (xpos, ypos)
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
            body.attachment_point = (height/2,1.5)
            body.origin = (xpos, ypos)
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
                self.add_static_polygon([(0,0), (5,0), (5,10), (0,10) ], xpos+discard+10, ypos-43, filterB)
                self.transfer_bands.append((ypos-43+10, ypos-43, [(xpos+discard+3, xpos+discard+13)], 1))
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
                exit_transfer_band_x.append((c*pitch+xpos+slope_x, c*pitch+xpos+slope_x+pitch-5))
            self.transfer_bands.append((ypos-10-slope_y, ypos-10-slope_y-3, exit_transfer_band_x, 1))

        self.add_static_circle(xpos-10, ypos+15, 5, filterA)
        self.add_static_circle(xpos+pitch*8-5, ypos+15, 5, filterA)
        self.add_static_polygon(box_polygon_shape(xpos-12, ypos-10, 3, 20))
        self.transfer_bands.append((-12+ypos+10, -12+ypos, transfer_band_x, 1 if inverted else 0))
        conrod.attachment_point = (pitch*8, 0.5)
        conrod.origin=(xpos,ypos+15)

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
            self.add_static_polygon([(0,0), (7,0), (7,8), (5,10), (0,10)], c*pitch+xpos-11, -12+ypos)

            pusher_poly = [(0,0), (2,0), (2,10), (0,12)]
            pusher_poly = translate_polygon(pusher_poly, c*pitch-11, -12+11)
            pusher = fixtureDef(shape=polygonShape(vertices=pusher_poly), density=1.0,
                                      filter=filter(groupIndex=1))
            pusher_parts.append(pusher)

            bellcrank_fixtures = [ fixtureDef(shape=box_polygon_shape(c*pitch+xpos, -12+ypos+20, 10, 3), density=1.0, filter=filters[2]),
                                fixtureDef(shape=box_polygon_shape(c*pitch+xpos, -12+ypos+8, 3, 12), density=1.0) ]
            bellcrank = self.add_multifixture(bellcrank_fixtures)

            anchorpos = (xpos+c*pitch, -12+ypos+20)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos)
            crank_list.append((bellcrank, (anchorpos[0]+8,anchorpos[1])))
            
        pusher_body = self.add_multifixture(pusher_parts, xpos, ypos)
        pusher_body.attachment_point = (6.5*pitch,5)
        pusher_body.origin = (xpos, ypos)
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
        if is_actually_adder:
            self.labels.append(("Program counter", xpos+20, ypos-50,45))
        else:
            self.labels.append(("Subtractor/accumulator", xpos+20, ypos-50,45))

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


        # Final output channel divider on the left
        if not comparison_output:
            self.add_static_polygon([ (-1,0), (1,0), (1,50), (-1,50) ],
                                xpos+(-0.5)*pitch+output_offset_x, ypos+pitch-30-sub_y_pitch*lines, filter=filters[4])

        for c in range(0,lines+1):
            # Large static bits that form input channels
            self.add_static_polygon([ (0,0), (pitch-7-2,-3), (pitch-7,-sub_y_pitch*(lines-c)-5), (0,-sub_y_pitch*(lines-c)-sub_y_pitch) ],
                                    xpos+c*pitch-pitch+3.5, ypos+pitch+9, filter=filters[4])


            
            # More top-side channels, but for the output
            self.add_static_polygon([ (0,0), (pitch-7,-5), (pitch-7,-sub_y_pitch*(lines-c)), (0,-sub_y_pitch*(lines-c)-sub_y_pitch) ],
                                    xpos+c*pitch-pitch+3.5+output_offset_x, ypos+pitch+9, filter=filters[4])
            # Bottom-side channels, for the output            
            if c != 0 or not comparison_output:
                self.add_static_polygon([ (-1,0), (2,0), (2,sub_y_pitch*c+10), (-1,sub_y_pitch*c+10) ],
                                        xpos+c*pitch+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines),filter=filters[4])

        # The leftmost output channel can be diverted to support 'negative detect' for the CMP operation.
        if comparison_output:
            leftmost_toggle = self.add_dynamic_polygon([ (-1,0), (1,0), (1,10), (-1,12) ],
                                                       xpos+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines),filter=filters[4])
            self.revolving_joint(attachment_body, leftmost_toggle, (xpos+output_offset_x, ypos+pitch-30-sub_y_pitch*(lines)))
            leftmost_toggle.attachment_point=(0,10)
            leftmost_toggle.origin=(xpos+output_offset_x,ypos+pitch-30-sub_y_pitch*(lines))
            self.comparison_diverter = leftmost_toggle
            self.add_static_polygon([ (-40,-20), (2,0), (2,2), (0,2) ],
                                    xpos+output_offset_x-3, ypos+pitch-30-sub_y_pitch*(lines)+10,filter=filters[4])
                
        # A reset bar
        reset_angle = math.atan2(sub_y_pitch, pitch)
        reset_len = math.sqrt((lines*pitch)**2 + (lines*sub_y_pitch)**2)
        reset_poly = rotate_polygon_radians(box_vertices(0, 0, reset_len, 5), reset_angle)
        if is_actually_adder:
            reset_lever = self.add_dynamic_polygon(polygonShape(vertices=reset_poly), xpos+10, ypos-sub_y_pitch*lines, filters[3])
            reset_lever.attachment_point=(reset_len/2,reset_len/2)
            reset_lever.origin=(xpos,ypos)
        else:
            reset_lever = self.add_dynamic_polygon(polygonShape(vertices=reset_poly), xpos-216, ypos-sub_y_pitch*lines+10, filters[3])
            reset_lever.attachment_point=(reset_len/2,reset_len/2)
            reset_lever.origin=(xpos,ypos)
            return_crank = self.crank_left_up(xpos-300,ypos+10, attachment_body, weight=10)
            self.distance_joint(reset_lever, return_crank)

        self.slide_joint(attachment_body, reset_lever, (1,0), -20,20, friction=0.01)

        # Transfer bands in negative reader channels (discards)
        if discard_bands:
            transfer_band_x = [ (xpos+output_offset_x+pitch*x-12,xpos+output_offset_x+pitch*x) for x in range(1,lines) ]
            band_base_y = ypos-sub_y_pitch*lines
            self.transfer_bands.append((band_base_y+10, band_base_y, transfer_band_x, 0))

        return reset_lever

    def horizontal_injector(self, xpos, ypos, attachment_body):
        left = fixtureDef(shape=box_polygon_shape(0,0.5,10,6), density=1.0, filter=filters[0])
        right = fixtureDef(shape=box_polygon_shape(17,0.5,10,6), density=1.0, filter=filters[0])
        drive_rect = self.add_multifixture([left,right], xpos, ypos)
        drive_rect.attachment_point=(5, 3.5)
        drive_rect.origin=(xpos,ypos)
        self.slide_joint(attachment_body, drive_rect, (1,0), 0, 10, friction=0)
        self.add_static_polygon(box_polygon_shape(0,-7,17,7), xpos, ypos)
        self.add_static_polygon(box_polygon_shape(0,7,10,20), xpos, ypos) # Blocks left
        self.add_static_polygon(box_polygon_shape(10+7,7,10,20), xpos, ypos)
        self.add_static_polygon(polygonShape(vertices=[(0,27), (10,27), (0,37)]), xpos, ypos)
        self.add_static_polygon(polygonShape(vertices=[(17,27), (27,27), (27,37)]), xpos, ypos)
        return drive_rect


    def injector(self, xpos, ypos, attachment_body, injector_crank_array, columns=8):
        intake_angle = 30
        height = 8
        crank_offset = pitch-10
        crank_y = 19
        injector_bar_height = 90
        for c in range(0,columns):
            divider_height = 6.5
            height2 =        6.5
            divider_vertices = [ (0,0), (pitch-7,0), (pitch-7,divider_height), (0,height2) ]
            divider_vertices = translate_polygon(divider_vertices, xpos+c*pitch, ypos+pitch+10)
            self.add_static_polygon(divider_vertices)
            
        for c in range(0,columns):
            # The base
            self.add_static_polygon([ (10,-20), (24,-20), (24,-15), (21.5,-12.5), (10,-15)], xpos+c*pitch, ypos+pitch+10)
            
            bellcrank_shape = [ fixtureDef(shape=box_polygon_shape(c*pitch+xpos+crank_offset, ypos+crank_y+9, 10, 3), density=1.0, filter=filters[1]),
                                fixtureDef(shape=box_polygon_shape(c*pitch+xpos+crank_offset+0.1, ypos+crank_y, 2.9, 12), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)) ]

            bellcrank = self.add_multifixture(bellcrank_shape)
            anchorpos = (xpos+c*pitch+crank_offset, ypos+crank_y+10)
            self.revolving_joint(bodyA=bellcrank, bodyB=attachment_body, anchor=anchorpos, friction=0)
            # leverage adjusts the amount of distance the injector will move. Higher number = less distance but more power
            leverage = 8
            injector_crank_array.append((bellcrank, (anchorpos[0]+leverage,anchorpos[1])))
            # All-inject bar
            raiser = self.add_dynamic_circle(xpos+c*pitch+15, ypos+injector_bar_height+10, radius=5, density=50.0)
            self.slide_joint(attachment_body, raiser, (0,1), 0,5, friction=0)
            self.world.CreateDistanceJoint(bodyA=bellcrank,
	                              bodyB=raiser,
                                      anchorA=((anchorpos[0]+6)*self.scale,(anchorpos[1]-10)*self.scale),
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
        bearing = self.add_dynamic_circle(xpos, ypos, 6.35/2, density=5.0, filter=filters[plane], color=(255,0,0) if plane==0 else (127,0,0))
        self.ball_bearings.append((bearing,plane))
        return bearing

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
        # Row followers
        for row in range(0,8):
            row_follower_fixtures = []
            for selector_no in range(0,selector_rods+1):
                row_follower_fixtures.append(fixtureDef(shape=circleShape(radius=3, pos=(selector_no*25+32,0)), density=1.0, filter=filter(groupIndex=1, categoryBits=0x0002, maskBits=0xFFFE)))
            follower = self.add_multifixture(row_follower_fixtures, xpos+200, 14*row+10, color=(0,255,255))
            follower.attachment_point = (30,0)
            follower.origin = (xpos, 0)
            follower_array.append(follower)
            self.slide_joint(follower, groundBody, (1,0), limit1=0, limit2=20, friction=0)

        # Holdoff bar for all output rods
        follower_holdoff_bar = self.vertical_rotating_bar(xpos+200+25*selector_rods+14,0,14*memory_rows+5, groundBody)
        
        return (selector_holdoff_bar, follower_holdoff_bar)
        
    def memory_module(self, xpos, ypos, groundBody):
        self.labels.append(("Memory", xpos+20, ypos,0))
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
        injectors=[]
        for col in range(0,8):
            injector = self.add_multifixture(row_injector_fixtures, 0, 7+14*col, color=(200,200,0))
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
            ejector = self.add_multifixture(row_ejector_fixtures, 0, 14*col, color=(200,0,200))
            ejector.attachment_point = (22*col, 0)
            ejectors.append(ejector)
            self.slide_joint(ejector, groundBody, (1,0), 0, 14, friction=0)

        # Add weights which bias the rows
        for col in range(0,8):
            crank = self.crank_left_up(xpos-250+30*col, ypos+14*col-30, groundBody, output_length=40, weight=50)
            self.distance_joint(ejectors[col], crank)
                              
        self.memory_followers = []
        self.memory_selectors = []
        (selector_holdoff, follower_holdoff) = self.add_row_decoder(xpos+50, ypos, groundBody, self.memory_followers, self.memory_selectors)
        self.labels.append(("Address decoder", xpos+250,ypos,0))
        # Rods which connect row selectors to ejectors
        for r in range(0,memory_rows):
            self.distance_joint(ejectors[r], self.memory_followers[r])

        # Gate returner to push all the memory rows back in
        self.memory_returning_gate = self.vertical_rotating_bar(xpos-32,0,14*memory_rows, groundBody)

        # Wall on the left of the memory to create the final channel
        memory_fixed = self.add_static_polygon([ (0,0), (3,0), (3,14*8), (0,14*8.5)], -10,0, filter=filter(groupIndex=0, categoryBits=0x0001, maskBits=0xFFFF))
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
        crank.attachment_point=(0,output_length)
        crank.origin=(xpos,ypos)
        return crank

    def crank_right_up(self, xpos, ypos, attachment_body, output_length=20, weight=10):
        crank_fixture_1 = fixtureDef(shape=box_polygon_shape(0,0,20,3), density=1.0, filter=filters[0])
        crank_fixture_2 = fixtureDef(shape=box_polygon_shape(0,0,3,output_length), density=1.0, filter=filters[0])
        crank_fixture_3 = fixtureDef(shape=box_polygon_shape(10,-5,10,10), density=weight, filter=filters[0]) # Heavy weight
        crank = self.add_multifixture([crank_fixture_1, crank_fixture_2, crank_fixture_3], xpos, ypos)
        self.revolving_joint(crank, attachment_body, (xpos,ypos))
        crank.attachment_point=(0,output_length)
        crank.origin=(xpos,ypos)
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
        blocker_set.attachment_point=(5,3)
        blocker_set.origin=(xpos,ypos)
        self.slide_joint(attachment_body, blocker_set, (1,0), 0, 15, friction=False)
        
        crank = self.crank_left_up(xpos-30, ypos-20, attachment_body)
        self.distance_joint(crank, blocker_set)
        blocker_set.attachment_point=(c*pitch+5,3)
        blocker_set.origin=(xpos, ypos)
        return blocker_set
    # Interface functions to PyBox2D

    def distance_joint(self, bodyA, bodyB, posA=None, posB=None):
        if 'origin' not in bodyA.__dict__:
            bodyA.origin = (0,0)
        if 'origin' not in bodyB.__dict__:
            bodyB.origin = (0,0)
        if posA is None: posA = bodyA.attachment_point
        if posB is None: posB = bodyB.attachment_point
        
        self.world.CreateDistanceJoint(bodyA=bodyA,
	                               bodyB=bodyB,
	                               anchorA=((posA[0]+bodyA.origin[0])*self.scale, (posA[1]+bodyA.origin[1])*self.scale),
	                               anchorB=((posB[0]+bodyB.origin[0])*self.scale, (posB[1]+bodyB.origin[1])*self.scale),
	                               collideConnected=False)
        self.distance_links.append((bodyA, posA, bodyB, posB))

    
    def add_static_polygon(self,vertices, xpos=0, ypos=0, filter=filters[0]):
        translated_vertices = translate_polygon(vertices, xpos, ypos)
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in translated_vertices])
        fixture = fixtureDef(shape=shape, density=1.0, filter=filter)
        self.static_polygons.append(shape)
        return self.world.CreateStaticBody(fixtures=fixture)

    def add_static_circle(self, xpos, ypos, radius, filter=filters[0]):
        return self.world.CreateStaticBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale, ypos*self.scale)), filter=filter))

    def add_dynamic_circle(self, xpos, ypos, radius, density=1.0, filter=filters[0],color=(127,127,127)):
        circle = self.world.CreateDynamicBody(fixtures=fixtureDef(shape=circleShape(radius=radius*self.scale, pos=(xpos*self.scale,ypos*self.scale)), density=density, filter=filter), userData=color)
        circle.attachment_point = (xpos*self.scale, ypos*self.scale)
        self.dynamic_bodies.append(circle)
        return circle
        
    def add_dynamic_polygon(self, vertices, xpos, ypos, filter=filters[0], density=1.0):
        shape = polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in vertices])
        fixture = fixtureDef(shape=shape, density=density, filter=filter)
        body = self.world.CreateDynamicBody(fixtures=fixture, position=(xpos,ypos))
        self.dynamic_bodies.append(body)
        return body

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
        
    def add_multifixture(self, fixtures, xpos=0, ypos=0, color=(127,127,127)):
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
                                               
        body = self.world.CreateDynamicBody(position=(xpos*self.scale, ypos*self.scale), fixtures=new_fixtures, userData=color)
        self.dynamic_bodies.append(body)
        return body

    def add_multipolygon(self, polygons, xpos=0, ypos=0, filter=filters[0]):
        fixtures = []
        for p in polygons:
            fixtures.append(fixtureDef(shape=polygonShape(vertices=[(x*self.scale, y*self.scale) for (x,y) in p]),
                                               filter=filter,
                                               density=1.0))
                                               
        body = self.world.CreateDynamicBody(position=(xpos*self.scale, ypos*self.scale), fixtures=fixtures)
        self.dynamic_bodies.append(body)
        return body

    
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

    def add_cam(self, xpos, ypos, follower_len, bumps=[], horizontal=False, reverse_direction=False, axis_offset=0, axis=True, bump_height=3, slow_rise=False):
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

        if slow_rise:
            ang=bumps[0][0]
            bump_points = [( radius*math.cos(-ang*math.pi*2), radius*math.sin(-ang*math.pi*2)) ]
            height = radius
            for point in range(0,14):
                ang += 0.015
                height += 0.3
                bump_points.append( ( height*math.cos(-ang*math.pi*2), height*math.sin(-ang*math.pi*2)) )
            bump_points.append(( radius*math.cos(-ang*math.pi*2), radius*math.sin(-ang*math.pi*2)))
            bump_fixtures.append(fixtureDef(shape=polygonShape(vertices=bump_points),density=0.0,filter=filters[0]))
        else:
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
                f = fixtureDef(shape=polygonShape(vertices=bump_points),density=0.0,filter=filters[0], userData=(0,0,255))
                bump_fixtures.append(f)
        cam_body = self.add_multifixture(bump_fixtures + [disc_fixture], xpos, ypos, (0,255,0))
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

            # Each instruction line has three parts - 'block' is the
            # revolving bit which drops into place when an instruction
            # is selected. 'block_slider' is connected to that and
            # drives the relevant parts of the machine.  'pusher' is
            # the input, also a sliding part usually attached to a
            # cam, which will engage and push the 'block' when that
            # instruction is selected.
            self.labels.append((instruction_opcodes[7-i], xpos+i*30, ypos-i*follower_spacing+10,0))
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
            block_slider.attachment_point = (15, 2.5)
            block_slider.origin=(xpos+i*30+offset*2,ypos-i*andgate_spacing_y-50)
            self.instruction_outputs.append(block_slider)
            pusher_slider = self.add_dynamic_polygon(box_vertices(0, 0, 30,5), xpos+i*30+30-offset*2, ypos-i*andgate_spacing_y-68)
            pusher_slider.attachment_point = (15, 2.5)
            pusher_slider.origin=(xpos+i*30-offset*2+30, ypos-i*andgate_spacing_y-70)
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
                    self.add_ball_bearing(memory_col0_x + pitch*(7-x)+2, memory_row0_y + 14*y+41, 0)

    def setup_ssem(self):
        """ Sets up all the parts of the SSEM except cams. """
        
        groundBox = box_polygon_shape(-20,-500,1,1) # A tiny box that just acts as the ground body for everything else
        groundBody = self.world.CreateStaticBody(shapes=groundBox)
        self.memory_sender_y = -500
        self.groundBody = groundBody
        # Initial charge for main injector
        self.ball_bearing_block(0,190,cols=16)
        self.add_static_polygon([ (0,20), (100,0), (100,5), (0,25)], -132, 220)
        self.add_static_polygon([ (0,0), (3,0), (3,20), (0,20)], -132, 240)
        self.injector_cranks = []
        self.parts.main_injector_raiser = self.injector(-32,150, groundBody, injector_crank_array=self.injector_cranks)

        (self.parts.memory_selector_holdoff, self.parts.memory_follower_holdoff) = self.memory_module(0,0, groundBody)
        self.upper_regenerators = []
        self.parts.accumulator_diverter_lever = self.diverter_set(0,130, groundBody, slope_x=-240, slope_y=180, return_weight=10) # Diverter 1. Splits to subtractor reader.

        self.parts.discard_lever_2 = self.diverter_set(-5,-30, groundBody, discard=470) # Diverter 2a. Discard reader-pulse data.
        self.parts.upper_regen_control = self.regenerator(-5,-65, groundBody, self.upper_regenerators) # Regenerator 1. For regenning anything read from memory.
        self.parts.ip_diverter_lever = self.diverter_set(-5,-95, groundBody, slope_x=320, slope_y=170, start_at=3, return_weight=5) # Diverter 1. Splits to instruction counter.
        self.parts.diverter_3 = self.diverter_set(-10,-158, groundBody, slope_x=200, slope_y=310) # Diverter 3; splits to instruction reg/PC
        
        # PC injector
        pc_injector_x = 230
        pc_injector_y = -290
        self.pc_injector_cranks = []    
        self.parts.pc_injector_raiser = self.injector(pc_injector_x,pc_injector_y, groundBody, injector_crank_array=self.pc_injector_cranks, columns=5)
        # Initial charge for PC injector
        self.ball_bearing_block(250,-240,cols=8)


        sub_pos_x = -15
        sub_pos_y = -220
        self.parts.accumulator_reset_lever = self.subtractor(sub_pos_x,sub_pos_y, groundBody, discard_bands=True, toggle_joint_array=self.accumulator_toggles, comparison_output=True)
        self.dropper = self.slow_drop_unit(groundBody, sub_pos_x-18, sub_pos_y+40)

        # The 'skip lever' - the large balance arm which injects one
        # into the program counter when comparison (CMP) succeeds
        skip_lever_x = sub_pos_x - 200
        skip_lever_y = sub_pos_y - 200
        a = fixtureDef(shape=polygonShape(vertices=[(-50,-32), (0,-30), (0,-25), (-50,-30)]), filter=filters[0], density=1.0)
        b = fixtureDef(shape=box_polygon_shape(0,-30,5,30), filter=filters[0], density=1.0)
        c = fixtureDef(shape=box_polygon_shape(-30,-30,5,30), filter=filters[0], density=1.0)
        d = fixtureDef(shape=box_polygon_shape(0,0,300,5), filter=filters[2], density=1.0)
        e = fixtureDef(shape=box_polygon_shape(285,-15,15,15), filter=filters[2], density=2.10)
        f = fixtureDef(shape=box_polygon_shape(150,-50,5,50), filter=filters[0], density=1.0)
        skip_lever=self.add_multifixture([a,b,d,e,f], skip_lever_x, skip_lever_y)
        skip_lever.attachment_point = (150,-50)
        skip_lever.origin = (skip_lever_x,skip_lever_y)

        self.revolving_joint(groundBody, skip_lever, (skip_lever_x+150, skip_lever_y+2.5), friction=0)
        self.add_static_polygon(polygonShape(vertices=box_vertices(0, 0, 10,10)), skip_lever_x+270, skip_lever_y-15, filter=filters[2])
        self.parts.cmp_injector = self.horizontal_injector(skip_lever_x-48,skip_lever_y+257, groundBody)
        self.ball_bearing_block(skip_lever_x-30,skip_lever_y+280,cols=1)
        self.add_static_polygon(polygonShape(vertices=[(0,0), (20,0), (0,20)]), skip_lever_x-30,skip_lever_y+230)
                            
        self.lower_regenerators = []
        self.parts.lower_regen_control = self.regenerator(-203,-400, groundBody, self.lower_regenerators)
        #Program counter
        self.parts.pc_reset_lever = self.subtractor(400,-320, groundBody, lines=5, toggle_joint_array=self.ip_toggles, is_actually_adder=True)
        # Thing that adds one ball bearing to the PC
        pc_incrementer_x = 457
        pc_incrementer = self.horizontal_injector(pc_incrementer_x,-240, groundBody)
        # Overspill loose ball bearings from the PC incrementer to the PC reader
        overspill_width = pc_incrementer_x - pc_injector_x - 125
        self.add_static_polygon([ (0, 0), (overspill_width,15), (overspill_width,20), (0,5) ], pc_injector_x+125,pc_injector_y+65)

        # Block to stop right-side overspill on incrementer
        self.add_static_polygon([ (0, 0), (5,0), (5,20), (0,20) ], pc_incrementer_x+27,-240+40)


        self.ball_bearing_block(457+20,-250+30,cols=1)
        self.distance_joint(skip_lever, pc_incrementer)

        self.connect_regenerators()

        # Large collection plates at the bottom
        self.add_static_polygon([ (-300,-600),(700,-550), (700,-610), (-300,-610)])
        self.add_static_polygon([ (600,-610),(700,-610), (850,-400), (800,-400)])
        self.add_static_polygon([ (-500,-400),(-450,-400), (-310,-610), (-400,-610)])

        # Instruction decoder ROM
        self.rom_followers = []
        self.rom_selectors = []
        (self.parts.instruction_selector_holdoff, self.parts.instruction_follower_holdoff) = self.add_row_decoder(200, 0, groundBody, self.rom_followers, self.rom_selectors)
        self.labels.append(("Instruction decoder", 400,0,0))

        self.add_instruction_cranks(groundBody, 550, 140)
        self.parts.sender_eject = self.memory_sender(198,self.memory_sender_y, groundBody)
        self.connect_memory()

        # A guard which stops waste data from the subtractor falling into the instruction register
        self.add_static_polygon([ (0,0), (120,120), (120,123), (0,3)], 120, self.memory_sender_y-10)

        # Add one final transfer band to move everything back into band 0
        self.transfer_bands.append((-550+10, -550, [ (-300,800)], 1))

    def basic_cam(self, x, y, arm_length, bumps, axis_offset=0, attachment_part=None, horizontal=False, reverse_direction=False, bump_height=3, slow_rise=False):
        follower_body = self.add_cam(x,y,arm_length, bumps=bumps, axis_offset=axis_offset,
                                     horizontal=horizontal, reverse_direction=reverse_direction,
                                     bump_height=bump_height, slow_rise=slow_rise, axis=(attachment_part is not None))
        if attachment_part is not None: self.distance_joint(follower_body, attachment_part)

    def rake_cam(self, xpos, ypos):
        attachment_body = self.groundBody
        radius = 30
        slider_length = 80
        disc_fixture = fixtureDef(shape=circleShape(radius=radius, pos=(0,0)),density=1.0,filter=filters[0])
        cam_body = self.add_multifixture([disc_fixture], xpos, ypos)
        cam_driver = self.revolving_joint(attachment_body, cam_body, (xpos,ypos), motor=1, force=50)
        cam_driver.motorSpeed = 0.5

        crank_fixture = fixtureDef(shape=polygonShape(vertices=box_vertices(0,0,50,3)),density=1.0,filter=filters[2])
        crank_body = self.add_multifixture([crank_fixture], xpos+radius, ypos)
        self.revolving_joint(cam_body, crank_body, (xpos+radius,ypos))

        slider_fixture = fixtureDef(shape=polygonShape(vertices=box_vertices(0,0,slider_length,3)),density=1.0,filter=filters[2])
        pusher_vertices = [(0,0), (10,0), (8,5), (2,5)]
        pusher_fixture = fixtureDef(shape=polygonShape(vertices=translate_polygon(pusher_vertices, slider_length, 0)),density=1.0,filter=filters[0])
        slider_body = self.add_multifixture([slider_fixture,pusher_fixture], xpos+radius+50, ypos)
        self.revolving_joint(crank_body, slider_body, (xpos+radius+50,ypos))
        self.slide_joint(attachment_body, slider_body, (1,0), -60, 60, friction=0)

    def slow_drop_unit(self, attachment_body, xpos, ypos):
        dropper_fixtures = []
        for i in range(0,9):
            self.add_static_polygon([ (0,0), (15,0), (15,3), (0,3)], xpos+pitch*i, ypos)
            if(i<8):
                dropper_fixtures.append(fixtureDef(shape=polygonShape(vertices=box_vertices(pitch*i,-4,4+i,3)),density=1.0,filter=filters[0]))

        dropper_body = self.add_multifixture(dropper_fixtures, xpos, ypos)
        self.slide_joint(attachment_body, dropper_body, (1,0), -60, 60, friction=0.1)
        dropper_body.attachment_point=(100,-2.5)
        dropper_body.origin=(xpos,ypos)
        # End stop
        self.add_static_polygon([ (0,0), (15,0), (15,3), (0,3)], xpos+pitch*8+8, ypos-3)

        # Return lever
        return_crank = self.crank_right_up(xpos+200,ypos-20, attachment_body, weight=10)
        self.distance_joint(return_crank, dropper_body)
        return dropper_body

    def setup_cams(self):

        groundBody = self.groundBody

        # Important timing points:
        ip_ready_point = 0.15 # Address should be set up for instruction fetch at this point
        instruction_fetched = 0.4 # Instruction should be in instruction register
        instruction_ready_point = 0.50 # Instruction decoder should be set up, ready for cams to use

        # Cam mapping from signal name to computer part and arm length
        cam_mapping = {
            "PC INJECTOR": (self.parts.pc_injector_raiser, 100),
            "MEMORY DECODER INPUT HOLDOFF": (self.parts.memory_selector_holdoff, 150),
            "MEMORY RETURN": (self.memory_returning_gate,100),
            "MEMORY DECODER OUTPUT HOLDOFF": (self.parts.memory_follower_holdoff, 100),
            "SENDER EJECT": (self.parts.sender_eject,80),
            "UPPER REGEN": (self.parts.upper_regen_control, 80),
            "TO INSTRUCTION REGISTER": (self.parts.diverter_3, 60),
            "INSTRUCTION OUTPUT HOLDOFF": (self.parts.instruction_selector_holdoff, 150),
            "LDN TRIGGER": (self.instruction_inputs[LDN], 120),
            "IP OUTPUT HOLDOFF": (self.parts.instruction_follower_holdoff, 100),
            "MAIN INJECTOR": (self.parts.main_injector_raiser, 100),
            "STO TRIGGER": (self.instruction_inputs[STO], 100),
            "JRP TRIGGER": (self.instruction_inputs[JRP],100),
            "DISCARD": (self.parts.discard_lever_2, 80),
            "LOWER REGEN": (self.parts.lower_regen_control, 80),
            "JMP TRIGGER": (self.instruction_inputs[JMP], 140),
            "CMP TRIGGER": (self.instruction_inputs[CMP], 120),
            "INC PC": (None, 60)
        }

        for c in cams:
            if c.signal_name in cam_mapping:
                (attachment_part, arm_length) = cam_mapping[c.signal_name]
                self.basic_cam(c.xpos, c.ypos, arm_length, c.steps, c.offset, attachment_part, horizontal=c.horizontal, bump_height=c.bump_height, reverse_direction=c.reverse_direction)
            else:
                raise Exception("Can't find a part to attach signal '{}' to ".format(c.signal_name))

        self.distance_joint(self.parts.accumulator_reset_lever, self.instruction_outputs[LDN])
        self.distance_joint(self.parts.accumulator_diverter_lever, self.instruction_outputs[STO])
        self.distance_joint(self.parts.discard_lever_2, self.instruction_outputs[STO])
        self.distance_joint(self.parts.ip_diverter_lever, self.instruction_outputs[JRP])
        self.distance_joint(self.parts.pc_reset_lever, self.instruction_outputs[JMP])
        self.distance_joint(self.comparison_diverter, self.instruction_outputs[CMP])
        self.distance_joint(self.parts.cmp_injector, self.instruction_outputs[CMP])

        # Cam 20: Slow dropper
        self.basic_cam(-300,-100, 80, [(0.65,0)], 8, self.dropper, horizontal=True, reverse_direction=True, bump_height=5, slow_rise=True)

        # Notable timing points:
        # 0.31: Memory at PC has been read and regenerated

    def __init__(self, testmode, randomtest, test_set_no, headless, overlay=False, timelapse=0):
        super(Memory, self).__init__()
        self.labels = []
        self.stopFlag = False
        self.timelapse = timelapse
        self.settings.drawOverlay = overlay
        self.test_set_no = test_set_no
        self.instruction_tested = False
        self.parts = Parts()
        self.prewritten_test = False
        self.random_test = False
        self.cycles_complete = 0
        self.phasetext = "Initializing"
        settle_delay = 400
        if randomtest:
            # Start a random test with test set 0 as the base
            self.test_set = test_set[0]
            self.random_test = True
            self.auto_test_mode = True
            randomseed = test_set_no
            if randomseed > 0:
                random.seed(randomseed)
            self.start_point = random.randint(settle_delay,settle_delay+100)
            self.name="SSEM - Random test mode"
            self.test_set['cycles'] = 3
        elif testmode:
            # Start a prewritten test
            self.auto_test_mode = True
            self.prewritten_test = True
            print("Running test {}".format(test_set_no))
            self.start_point = random.randint(settle_delay,settle_delay+100)
            print("Running test {} starting at tick {}".format(testmode, self.start_point))
            self.test_set = test_set[self.test_set_no]
            self.name="SSEM - {}".format(self.test_set.get("name", "Automated test"))
        else:
            # Use the test data from the specified test, but don't go into testmode.
            self.auto_test_mode = False
            print("Starting in interactive mode")
            self.start_point = 0
            self.test_set = test_set[self.test_set_no]

        self.accumulator_toggles = []
        self.ip_toggles = []
        self.cams_on = False
        self.all_cam_drives = []
        self.all_toggle_drives = []
        self.scale = 1.0
        self.transfer_bands = []
        self.ball_bearings = []
        self.static_polygons = []
        self.dynamic_bodies = []
        self.distance_links = []
        self.sequence = 0 # Like step count but only increments when cams are on
        self.init_pulse = 0 # A small counter for use at startup to reset the toggles

        self.setup_ssem()
        self.setup_cams()

        # Additional parts:
        self.rake_cam(-80,190)

        
        self.initial_state = SSEM_State()
        if self.random_test:
            while True:
                self.initial_state.set_random()
                self.final_state = copy.deepcopy(self.initial_state)
                for i in range(0,self.test_set.get("cycles",1)):
                    self.final_state.advance()
                if self.final_state.unsupported_flag:
                    print("Regenerating initial state because unsupported operations were performed in the emulator.")
                else:
                    break
        else:
            self.initial_state.accumulator = self.test_set.get("initial_accumulator", 0)
            self.initial_state.pc = self.test_set.get("initial_pc", 0)
            self.initial_state.mem = self.test_set.get("initial_memory")
            self.final_state = copy.copy(self.initial_state)
            for i in range(0,self.test_set.get("cycles",1)):
                self.final_state.advance()
            if self.final_state.unsupported_flag:
                print("Exiting because unsupported operations were performed in the emulator.")
                sys.exit(0)

        self.expected_state = self.final_state
        self.set_initial_memory(self.initial_state.mem)
        self.initial_accumulator =  self.initial_state.accumulator
        self.initial_pc = self.initial_state.pc
        self.instruction_text="Fetching..."

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
                    
                    expected_pos_x = memory_col0_x + pitch*(7-col)
                    expected_pos_y = memory_row0_y + 14*row+40
                    dx = expected_pos_x - x
                    dy = expected_pos_y - y
                    if abs(dx)<5 and abs(dy)<5:
                        row_val += 1<<col
            memory.append(row_val)
        return memory

    def instruction_test(self):
        # Early test to see if instruction has been read correctly. If not, no point continuing with test.
        columns = []
        val = 0
        for i in range(0,8):
            (x,y) = self.memory_sensors[i].worldCenter
            x /= self.scale
            y /= self.scale
            relative_pos = y - self.memory_sender_y
            columns.append(1 if relative_pos > 2 else 0)
            val += (1<<(7-i) if relative_pos>2 else 0)
        print("Instruction register value = {}".format(",".join(map(str,columns))))
        instruction_op = (val >> 5) & 0x7
        instruction_address = val & 0x1f
        self.instruction_text="Executing {}: {} on address {} (wrapped to {})".format(val, instruction_opcodes[instruction_op], instruction_address, instruction_address % memory_rows)

        self.instruction_tested = True
    
    def verify_results(self):
        expected_accumulator = self.test_set.get("expected_accumulator", self.initial_accumulator)
        expected_pc = self.test_set.get("expected_pc", 1)
        
        accumulator = self.read_accumulator_value()
        pc = self.read_pc_value()
        memory = self.read_memory_array()

        if self.prewritten_test:
            if expected_accumulator != accumulator:
                print("FAIL: Expected accumulator {}, actual result {}".format(expected_accumulator, accumulator))
                return WRONG_ACCUMULATOR
                return
            expected_memory = copy.copy(self.test_set["initial_memory"])
            if "memory_update" in self.test_set:
                (address, value) = self.test_set["memory_update"]
                expected_memory[address] = value

            if expected_pc != pc:
                print("FAIL: Expected PC {}, actual result {}".format(expected_pc, pc))
                return WRONG_IP

            for a in range(0,8):
                if expected_memory[a] != memory[a]:
                    print("FAIL: At address {}, expected memory {} but found {}".format(a,expected_memory[a], memory[a]))
                    return WRONG_MEMORY

        # Verify against emulator
        if accumulator != self.expected_state.accumulator:
            print("FAIL: Emulated accumulator {}, actual result {}".format(self.expected_state.accumulator, accumulator))
            return WRONG_ACCUMULATOR
        if (pc%memory_rows) != self.expected_state.pc:
            print("FAIL: Emulated PC {}, actual result {}".format(self.expected_state.pc, (pc%memory_rows)))
            return WRONG_IP
        for a in range(0,8):
            if self.expected_state.mem[a] != memory[a]:
                print("FAIL: At address {}, emulated memory {} but found {}".format(a,self.expected_state.mem[a], memory[a]))
                return WRONG_MEMORY
            
        print("PASS")
        return SUCCESS

    def update_state(self):
        local_sequence = self.sequence % 10000
        if local_sequence < 1500:
            self.phasetext = "Setup instruction address"
            self.instruction_text="Fetching..."
        elif (local_sequence < 3600):
            self.phasetext = "Instruction fetch"
        elif (local_sequence < 4500):
            self.phasetext = "Instruction decode"
        elif (local_sequence < 8000):
            self.phasetext = "Execute"
        else:
            self.phasetext = "Writeback"
    
    def Step(self, settings):
        super(Memory, self).Step(settings)
        new_bearings = []
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
                            #print("Flipping ball bearing from plane %d to plane %d"%(source_plane, plane))
                            self.ball_bearings[i] = (self.world.CreateDynamicBody(
                                position=b.worldCenter,
                                fixtures=[fixtureDef(
                                    shape=circleShape(radius=6.35/2*self.scale, pos=(0,0)),
                                    density=5.0,
                                    filter=filters[plane])],
                                
                                userData=(255,0,0) if plane==0 else (127,0,0)),plane)
            # Fake ball lift - returns falling ball bearings to the top
            if y<-650:
                self.world.DestroyBody(b)
                new_bearings.append((self.add_ball_bearing(-50, 250, 0), plane))
            else:
                new_bearings.append((b, plane))
        self.ball_bearings = new_bearings

        if self.init_pulse < 25:
            bit = 0
            for d in self.accumulator_toggles:
                d.motorSpeed = 10 if (self.initial_accumulator & 1<<(7-bit))==0 else -10
                bit += 1
            bit = 0
            for d in self.ip_toggles:
                d.motorSpeed = -10 if (self.initial_pc & 1<<(4-bit))==0 else 10
                bit += 1
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
        simulation_time = ((self.sequence%10000)/10000.0)
        if self.sequence % 100 == 0 and self.cams_on:
            self.update_state()
            print("Step {} degrees timing = {} ACC= {} ({}) PC= {} ({}) Mem= {}".format(
                self.sequence, simulation_time,"".join(map(str,self.read_accumulator_array())),
                self.read_accumulator_value(), "".join(map(str,self.read_pc_array())), self.read_pc_value(),
                format(",".join(map(str, self.read_memory_array())))))
        if simulation_time > 0.41 and not self.instruction_tested:
            self.instruction_test()
        if simulation_time > 0.9:
            self.instruction_tested = False
        if angleTarget >= (math.pi*2*self.test_set.get("cycles",1)) and self.cams_on:
            angleTarget -= math.pi*2
            self.cams_on = False
            print("Sequence complete; cams off")
            result = self.verify_results()
            if result>0:
                sys.exit(result)
            if self.auto_test_mode:
                self.stopFlag = True
            
        for d in self.all_cam_drives:
            angleError = d.angle - angleTarget
            gain = 1.0
            d.motorSpeed = (-gain * angleError)

    def Keyboard(self, key):
        print("Processing key: {}".format(key))
        if key == Keys.K_r:
            self.cams_on = not self.cams_on

if __name__=="__main__":
    main(Memory(args.test, args.randomtest, args.testset, args.headless, args.overlay, args.timelapse))
