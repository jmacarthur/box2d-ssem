# External box2d renderer by Jim MacArthur

import pygame

class ExternalRenderer():
    def __init__(self,surface):
        self.surface = surface

    def draw_polygon(self,vertices):
        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, (0,255,0),
                               vertices[0], vertices[1])
        else:
            pygame.draw.polygon(
                self.surface, (255,255,255), vertices, 0)

    def draw_circle(self, pos, radius):
        pygame.draw.circle(
                self.surface, (255,255,255), (int(pos[0]), int(pos[1])), int(radius), 0)
