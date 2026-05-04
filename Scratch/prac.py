from Pygame import user_gui 
import pygame

#this is used for testing the pygame gui
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Kinect Skeleton (Stable)")
font = pygame.font.SysFont("consolas", 20)
clock = pygame.time.Clock()

s = [
        ["HEAD", 0, -90, 0],
        ["SPINE_SHOULDER", 0, 0, 0],
        ["SPINE_BASE", 0, 180, 0],

        ["ELBOW_RIGHT", -110, 40, 0],
        ["WRIST_RIGHT", -170, 80, 0],

        ["ELBOW_LEFT", 110, 40, 0],
        ["WRIST_LEFT", 170, 80, 0],

        ["KNEE_RIGHT", -45, 340, 0],
        ["ANKLE_RIGHT", -55, 500, 0],

        ["KNEE_LEFT", 45, 340, 0],
        ["ANKLE_LEFT", 55, 500, 0],
    ]  

while True:
    user_gui.draw(screen, s)