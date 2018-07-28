# External box2d renderer by Jim MacArthur

import pygame

class ExternalRenderer():
    def __init__(self,surface):
        self.surface = surface

    def draw_polygon(self,vertices):
        print("Rendering polygon")
        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, (0,255,0),
                               vertices[0], vertices[1])
        else:
            pygame.draw.polygon(
                self.surface, (255,255,255), vertices, 0)
