
# This is a derivative version of pygame_framework which adds code suitable
# for Box2D-SSEM. Several functions have code copied from pygame_framework.
#
# Box2D C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version of pygame_framework Copyright (c) 2010 kne / sirkne at gmail dot com
# Modified version for Box2D-SSEM Copyright (c) 2018 Jim MacArthur
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

import os
import pygame

from Box2D import (b2DrawExtended, b2Vec2, b2CircleShape)
from settings import fwSettings
from .pygame_framework import PygameDraw, PygameFramework
from framework import Keys

try:
    from .pygame_gui import (fwGUI, gui)
    GUIEnabled = True
except Exception as ex:
    print('Unable to load PGU; menu disabled.')
    print('(%s) %s' % (ex.__class__.__name__, ex))
    GUIEnabled = False

class ModifiedPygameDraw(PygameDraw):
    def DrawSegment(self, p1, p2, color):
        """
        Draw the line segment from p1-p2 with the specified color.
        """
        pygame.draw.aaline(self.surface, color, p1, p2)

    def DrawCircle(self, center, radius, color, drawwidth=1):
        """
        Draw a wireframe circle given the center, radius, axis of orientation
        and color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, (255,255,255),
                           (int(center[0]), int(center[1])), radius, drawwidth)

    def DrawSolidCircle(self, center, radius, axis, color):
        """
        Draw a solid circle given the center, radius, axis of orientation and
        color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)
        center = (int(center[0]), int(center[1]))
        if type(color)!=tuple:
            color = (color/2).bytes+[127]

        pygame.draw.circle(self.surface, color,
                           center, radius, 0)
        line_col = [x/2 for x in color]
        pygame.draw.circle(self.surface, line_col, center, radius, 1)

    def DrawPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes,
                               vertices[0], vertices)
        else:
            pygame.draw.polygon(self.surface, color.bytes, vertices, 0)

    def DrawSolidPolygon(self, vertices, color):
        """
        Draw a filled polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return
        if type(color)!=tuple:
            color = (color/2).bytes+[127]
        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color,
                               vertices[0], vertices[1])
        else:
            pygame.draw.polygon(
                self.surface, color, vertices, 0)
            line_col = [x/2 for x in color]
            pygame.draw.polygon(self.surface, line_col, vertices, 1)

    # the to_screen conversions are done in C with b2DrawExtended, leading to
    # an increase in fps.
    # You can also use the base b2Draw and implement these yourself, as the
    # b2DrawExtended is implemented:
    def to_screen(self, point):
        """
        Convert from world to screen coordinates.
        In the class instance, we store a zoom factor, an offset indicating where
        the view extents start at, and the screen size (in pixels).
        """
        x=(point[0] * self.zoom)-self.offset.x
        if self.flipX:
            x = self.screenSize.x - x
        y=(point[1] * self.zoom)-self.offset.y
        if self.flipY:
            y = self.screenSize.y-y
        return (x, y)

class ModifiedPygameFramework(PygameFramework):
    def __init__(self):
        super(ModifiedPygameFramework, self).__init__()
        self.renderer = ModifiedPygameDraw(surface=self.screen, test=self)
        self.world.renderer = self.renderer

    def __reset(self):
        super(ModifiedPygameFramework, self).__init__()
        self._fontSize = 50
        self._viewZoom = 1
        self._viewCenter = (100,0)
                        
    def __init__(self):
        super(PygameFramework, self).__init__()

        self.__reset()
        if fwSettings.onlyInit:  # testing mode doesn't initialize pygame
            return

        print('Initializing pygame framework...')
        # Pygame Initialization
        pygame.init()
        caption = "Python Box2D Testbed - " + self.name
        pygame.display.set_caption(caption)

        # Screen and debug draw
        self.screen = pygame.display.set_mode((1800,1100))
        self.screenSize = b2Vec2(*self.screen.get_size())

        self.renderer = ModifiedPygameDraw(surface=self.screen, test=self)
        self.world.renderer = self.renderer

        try:
            self.font = pygame.font.Font(None, self._fontSize)
            self.labelfont = pygame.font.Font(None, 20)
        except IOError:
            try:
                self.font = pygame.font.Font("freesansbold.ttf", self._fontSize)
            except IOError:
                print("Unable to load default font or 'freesansbold.ttf'")
                print("Disabling text drawing.")
                self.Print = lambda *args: 0
                self.DrawStringAt = lambda *args: 0

        # GUI Initialization
        if GUIEnabled:
            self.gui_app = gui.App()
            self.gui_table = fwGUI(self.settings)
            container = gui.Container(align=1, valign=-1)
            container.add(self.gui_table, 0, 0)
            self.gui_app.init(container)

        self.viewCenter = (360, -100.0)
        self.groundbody = self.world.CreateBody()

    def run(self):
        """
        Main loop.

        Continues to run while checkEvents indicates the user has
        requested to quit.

        Updates the screen and tells the GUI to paint itself.
        """

        # If any of the test constructors update the settings, reflect
        # those changes on the GUI before running
        if GUIEnabled:
            self.gui_table.updateGUI(self.settings)

        running = True
        clock = pygame.time.Clock()
        while running and not self.stopFlag:
            running = self.checkEvents()
            self.screen.fill((0, 0, 0))

            # Check keys that should be checked every loop (not only on initial
            # keydown)
            self.CheckKeys()

            # Run the simulation loop
            self.SimulationLoop()
            if GUIEnabled and self.settings.drawMenu:
                self.gui_app.paint(self.screen)

            self.draw_distance_links()
            if self.timelapse <= 1 or self.sequence%self.timelapse == 1:
                self.Draw(self.settings)
                if self.settings.drawOverlay:
                    self.overlay_draw()

                self.draw_labels()
                pygame.display.flip()

            if self.timelapse >= 1 and self.sequence%self.timelapse == 1:
                os.makedirs("images", exist_ok=True)
                pygame.image.save(self.renderer.surface, os.path.join("images", "img{:05d}.png".format(int(self.sequence/self.timelapse))))

            clock.tick(self.settings.hz)
            self.fps = clock.get_fps()

        self.world.contactListener = None
        self.world.destructionListener = None
        self.world.renderer = None

    def overlay_draw(self):
        for body in self.dynamic_bodies:
            # Colour is only per-body at the moment, since userdata doesn't seem to propogate in fixtures.
            color = body.userData
            if color is None: color = (127,127,127)
            for fixture in body.fixtures:
                if isinstance(fixture.shape, b2CircleShape):
                    self.renderer.DrawSolidCircle(self.renderer.to_screen(body.GetWorldPoint(fixture.shape.pos)), fixture.shape.radius, axis=0, color=color)
                else:
                    self.renderer.DrawSolidPolygon(vertices=[ self.renderer.to_screen(body.GetWorldPoint(x)) for x in fixture.shape], color=color)

    def draw_distance_links(self):
        for link in self.distance_links:
            (bodyA, posA, bodyB, posB) = link
            a = self.renderer.to_screen(bodyA.GetWorldPoint(posA))
            b = self.renderer.to_screen(bodyB.GetWorldPoint(posB))
            self.renderer.DrawSegment(a,b,(0,255,0))

    def draw_labels(self):
        for l in self.labels:
            (text, x, y, angle) = l
            (x, y) = self.renderer.to_screen((x,y))
            text_surface = self.labelfont.render(text, True, (255,255,255))
            if(angle!=0): text_surface = pygame.transform.rotate(text_surface, angle)
            self.screen.blit(text_surface, (x,y))

    def CheckKeys(self):
        """
        Check the keys that are evaluated on every main loop iteration.
        I.e., they aren't just evaluated when first pressed down
        """

        pygame.event.pump()
        self.keys = keys = pygame.key.get_pressed()
        if keys[Keys.K_LEFT]:
            self.viewCenter -= (2, 0)
        elif keys[Keys.K_RIGHT]:
            self.viewCenter += (2, 0)

        if keys[Keys.K_UP]:
            self.viewCenter += (0, 2)
        elif keys[Keys.K_DOWN]:
            self.viewCenter -= (0, 2)

        if keys[Keys.K_HOME]:
            self.viewZoom = 1.0
            self.viewCenter = (0.0, 20.0)

    def Print(self, str, color=(229, 153, 153, 255)):
        """
        Draw some text at the top status lines
        and advance to the next line.
        """
        self.screen.blit(self.font.render(
            str, True, color), (5, self.textLine))
        self.textLine += self._fontSize*0.8
