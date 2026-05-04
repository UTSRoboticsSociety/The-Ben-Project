from pykinect2024 import PyKinect2024
from pykinect2024.PyKinect2024 import *
from pykinect2024.PyKinectRuntime import PyKinectRuntime
from Pygame import user_gui

import time
import serial

import pygame

#=================VARIABLES =================
window_width = 800
window_height = 600

#=================SERIAL =================
ser = serial.Serial("COM9", 115200, timeout=0.05)
time.sleep(2)
print("Serial connected")

# ================= CONSTANTS =================
SEND_INTERVAL = 0.1       # seconds (10 Hz)
DEADBAND = 3              # degrees

def run():

    #======Pygame sutff======
    pygame.init()
    screen = pygame.display.set_mode((window_width, window_height))
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

            #====== Kinect======
            kinect = KinectTracking()
            tracking = kinect.skeleton_frame_ready()
            jointPointArray = kinect.jointPositionList
            user_gui.draw(screen, jointPointArray)  # Draw skeleton

            pygame.display.flip() #updating the frame
            clock.tick(60)
    

    finally:
        kinect.close()
        pygame.quit()

# ================= TRACKER =================
class KinectTracking(object):

    def __init__(self):
        self.last_send_time = 0

    def jointPositionList(
        self, head,
        spineShoulder,
        spineBase,
        elbowRight,
        wristRight,
        elbowLeft,
        wristLeft,
        kneeRight,
        ankleRight,
        kneeLeft,
        ankleLeft,
    ):
        s = [
            ["HEAD", head.x, head.y, head.z],
            ["SPINE_SHOULDER", spineShoulder.x, spineShoulder.y, spineShoulder.z],
            ["SPINE_BASE", spineBase.x, spineBase.y, spineBase.z],

            ["ELBOW_RIGHT", elbowRight.x, elbowRight.y, elbowRight.z],
            ["WRIST_RIGHT", wristRight.x, wristRight.y, wristRight.z],

            ["ELBOW_LEFT", elbowLeft.x, elbowLeft.y, elbowLeft.z],
            ["WRIST_LEFT", wristLeft.x, wristLeft.y, wristLeft.z],

            ["KNEE_RIGHT", kneeRight.x, kneeRight.y, kneeRight.z],
            ["ANKLE_RIGHT", ankleRight.x, ankleRight.y, ankleRight.z],

            ["KNEE_LEFT", kneeLeft.x, kneeLeft.y, kneeLeft.z],
            ["ANKLE_LEFT", ankleLeft.x, ankleLeft.y, ankleLeft.z],
        ]
        return s


    def skeleton_frame_ready(self, skeleton_frame):
        now = time.time()
        if now - self.last_send_time < SEND_INTERVAL:
            return

        kinect = PyKinectRuntime(FrameSourceTypes_Body)

        print("Kinect open — stand in front of it!")
        


        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()
            if bodies is not None:
                for i in range(0, kinect.max_body_count):
                    body = bodies.bodies[i]
                    if not body.is_tracked:
                        continue
                    
                    #====== Joint ======
                    joints = body.joints

                    head = joints[PyKinect2024.JointType_Head].position

                    spineShoulder = joints[PyKinect2024.JointType_SpineShoulder].position
                    spineBase = joints[PyKinect2024.JointType_SpineBase].position

                    elbowRight = joints[PyKinect2024.JointType_ElbowRight].position
                    wristRight = joints[PyKinect2024.JointType_WristRight].position

                    elbowLeft =joints[PyKinect2024.JointType_ElbowLeft].position
                    wristLeft = joints[PyKinect2024.JointType_WristLeft].position

                    kneeRight = joints[PyKinect2024.JointType_KneeRight].position
                    ankleRight = joints[PyKinect2024.JointType_AnkleRight].position

                    kneeLeft = joints[PyKinect2024.JointType_KneeLeft].position
                    ankleLeft = joints[PyKinect2024.JointType_AnkleLeft].position

                    # From the captured psositions, we store them in a list
                    self.jointPositionList(head, spineShoulder, spineBase, elbowRight, wristRight, elbowLeft, wristLeft, kneeRight, ankleRight, kneeLeft, ankleLeft)
                    
                    #====== Inverse Kinematics ======

                    # need to intergrate inverse kinatmic solver that takes a x,y,z argument and estamites the joint angles needed reach that point
                    # this will be needed for only four specific joints, that being the hands and feet.
                    # resultant angles from the inverser kinematic solver will be retunred in an array as radians which will need to be converted to degrees
                    
                    
                    # Build single message with all servo commands
                    angles = { #will need to update the angles and the servo ID
                        # #left
                        # 13: left_angles_deg[],
                        # 14: left_angles_deg[],
                        # 8: left_angles_deg[],

                        # #right
                        # 16: right_angles_deg[],
                        # 17: right_angles_deg[],
                        # 18: right_angles_deg[],

                        # 10: head_angle
                    }

                    self.send_all_servos(angles)
                    self.last_send_time = now

    def send_all_servos(self, angles):
        """Send all servo commands in a single serial message"""
        # Check which servos need updating
        to_send = {}
        for servo_id, angle in angles.items():
            last = self.last_angles.get(servo_id)
            
            if last is None or abs(angle - last) >= DEADBAND:
                to_send[servo_id] = angle
                self.last_angles[servo_id] = angle
        
        # If nothing changed, don't send
        if not to_send:
            return
        
        # Send all commands in ONE message: "13,pitch:14,roll:8,elbow\n"
        try:
            msg = ":".join(["%d,%d" % (sid, ang) for sid, ang in sorted(to_send.items())])
            msg += "\n"
            ser.write(msg.encode("utf-8"))
        except Exception as e:
            print("Serial write error:", e)




if __name__ == "__main__":
    run()