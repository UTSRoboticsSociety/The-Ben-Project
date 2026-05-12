from pykinect2024 import PyKinect2024
from pykinect2024.PyKinect2024 import *
from pykinect2024.PyKinectRuntime import PyKinectRuntime

from Pygame_stuff import user_gui

import pygame

kinect = PyKinectRuntime(FrameSourceTypes_Body)

print("Kinect open — stand in front of it!")

pygame.init()
screen = pygame.display.set_mode((800, 600))
#screen_unflip = pygame.display.set_mode((800, 600))
#screen = pygame.transform.flip(screen_unflip, False, True)
pygame.display.set_caption("Kinect Skeleton (Stable)")
font = pygame.font.SysFont("consolas", 20)
clock = pygame.time.Clock()

running = True
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        #screen.fill((0, 0, 0))  # Clear screen

        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()
            if bodies is not None:
                for i in range(0, kinect.max_body_count):
                    body = bodies.bodies[i]
                    if not body.is_tracked:
                        continue
                    #====== Joint ======
                    joints = body.joints

                    
                    head = joints[PyKinect2024.JointType_Head].Position

                    spineShoulder = joints[PyKinect2024.JointType_SpineShoulder].Position
                    spineBase = joints[PyKinect2024.JointType_SpineBase].Position

                    elbowRight = joints[PyKinect2024.JointType_ElbowRight].Position
                    wristRight = joints[PyKinect2024.JointType_WristRight].Position

                    elbowLeft =joints[PyKinect2024.JointType_ElbowLeft].Position
                    wristLeft = joints[PyKinect2024.JointType_WristLeft].Position

                    kneeRight = joints[PyKinect2024.JointType_KneeRight].Position
                    ankleRight = joints[PyKinect2024.JointType_AnkleRight].Position

                    kneeLeft = joints[PyKinect2024.JointType_KneeLeft].Position
                    ankleLeft = joints[PyKinect2024.JointType_AnkleLeft].Position

                    s = [
                        ["SPINE_BASE", spineBase.x, spineBase.y, spineBase.z], 
                        ["HEAD", head.x, head.y, head.z],
                        ["SPINE_SHOULDER", spineShoulder.x, spineShoulder.y, spineShoulder.z],
                    

                        ["ELBOW_RIGHT", elbowRight.x, elbowRight.y, elbowRight.z],
                        ["WRIST_RIGHT", wristRight.x, wristRight.y, wristRight.z],

                        ["ELBOW_LEFT", elbowLeft.x, elbowLeft.y, elbowLeft.z],
                        ["WRIST_LEFT", wristLeft.x, wristLeft.y, wristLeft.z],

                        ["KNEE_RIGHT", kneeRight.x, kneeRight.y, kneeRight.z],
                        ["ANKLE_RIGHT", ankleRight.x, ankleRight.y, ankleRight.z],

                        ["KNEE_LEFT", kneeLeft.x, kneeLeft.y, kneeLeft.z],
                        ["ANKLE_LEFT", ankleLeft.x, ankleLeft.y, ankleLeft.z],
                    ]
                    user_gui.draw(screen, s)  # Draw skeleton

        pygame.display.flip()
        clock.tick(60)
finally:
    kinect.close()
    pygame.quit()