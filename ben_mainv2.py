from pykinect2024 import PyKinect2024
from pykinect2024.PyKinect2024 import *
from pykinect2024.PyKinectRuntime import PyKinectRuntime

from Pygame import user_gui

import pygame

kinect = PyKinectRuntime(FrameSourceTypes_Body)

print("Kinect open — stand in front of it!")

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Kinect Skeleton (Stable)")
font = pygame.font.SysFont("consolas", 20)
clock = pygame.time.Clock()

running = True
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((0, 0, 0))  # Clear screen

        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()
            if bodies is not None:
                for i in range(0, kinect.max_body_count):
                    body = bodies.bodies[i]
                    if not body.is_tracked:
                        continue
                    joints = body.joints
                    wrist = joints[PyKinect2024.JointType_HandRight]
                    positionRH = wrist.Position

                    s = [
                        ["HEAD", 0, -90, 0],
                        ["SPINE_SHOULDER", 0, 0, 0],
                        ["SPINE_BASE", 0, 180, 0],
                        ["ELBOW_RIGHT", -110, 40, 0],
                        ["WRIST_RIGHT", (positionRH.x)*100, (positionRH.y)*100, (positionRH.z)*100],
                        ["ELBOW_LEFT", 110, 40, 0],
                        ["WRIST_LEFT", 170, 80, 0],
                        ["KNEE_RIGHT", -45, 340, 0],
                        ["ANKLE_RIGHT", -55, 500, 0],
                        ["KNEE_LEFT", 45, 340, 0],
                        ["ANKLE_LEFT", 55, 500, 0],
                    ]
                    user_gui.draw(screen, s)  # Draw skeleton

        pygame.display.flip()
        clock.tick(60)
finally:
    kinect.close()
    pygame.quit()