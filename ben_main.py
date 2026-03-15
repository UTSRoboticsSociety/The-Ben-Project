import numpy as np
import time
import math
import serial
from URDF import benURDFV3
from collections import deque

from pykinect2 import nui
from pykinect2.nui import JointId, TransformSmoothParameters

#14 = shoulder pitch
#13 = arm raise
#  

#=================VARIABLES =================

window_width = 800
window_height = 600

# ================= SERIAL =================
ser = serial.Serial("COM3", 115200, timeout=0.05) #be sure to change the COM port to what evre you are using
time.sleep(2)
print("Serial connected")

# ================= CONSTANTS =================

#Used for servo communiction
SEND_INTERVAL = 0.1       # seconds (10 Hz)
DEADBAND = 3              # degrees

ben = benURDFV3.Ben()


    

# ================= MAIN =================
def run():
    #kinect 
    kinect = nui.Runtime()
    kinect.skeleton_engine.enabled = True


    # Enable smoothing with default parameters
    try:
        kinect.skeleton_engine.smooth_parameters = nui.SkeletonEngine.TransformSmoothParameters
    except:
        pass  # Smoothing not available or already enabled
    

    tracker = KinectTracking()
    kinect.skeleton_frame_ready += tracker.skeleton_frame_ready

    print("Kinect started")

    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("stopping")
        kinect.close()
        ser.close()

    


# ================= TRACKER =================
class KinectTracking(object):

    def __init__(self):
        self.last_send_time = 0
        self.last_angles = {}
        self.current_skeleton = None #store for visualization
        self.roll = 0
        self.right_target = 0
        self.inital_left_ik = [0, 0, 0, 0, 0]
        self.inital_right_ik = [0, 0, 0, 0, 0]
        self.was_tracked = False
        self.frames_not_tracked = 0
    

    def send_default_pose(self):
        default_angles = {
            #left
            13: 30,
            14: 0,
            8: 0,
            #right
            16: 0,
            17: 30,
            18: 0,

            10: 0
        }      
        self.send_all_servos(default_angles, force=True)
        self.inital_left_ik = [0, 0, 0, 0, 0]
        self.inital_right_ik = [0, 0, 0, 0, 0]


    def skeleton_frame_ready(self, skeleton_frame):
        now = time.time()
        skeletons = skeleton_frame.SkeletonData
        tracked = None

        for s in skeletons:
            if s and s.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                tracked = s
                break

        if not tracked:
            self.current_skeleton =None 
            print("hello")
            if self.was_tracked:
                print("going home") 
                self.send_default_pose()
                self.was_tracked = False
                self.last_send_time = now
        
            return
        
        self.frames_not_tracked = 0

        if now - self.last_send_time < SEND_INTERVAL:
            return


        self.was_tracked = True
        self.current_skeleton = tracked

        # ----------- JOINTS -----------
        left_end_effector = tracked.SkeletonPositions[JointId.WristLeft]
        left_origin = tracked.SkeletonPositions[JointId.ShoulderLeft]

        right_end_effector = tracked.SkeletonPositions[JointId.WristRight]
        right_origin = tracked.SkeletonPositions[JointId.ShoulderRight]

        head_end_effector = tracked.SkeletonPositions[JointId.Head]
        shoulderCentre = tracked.SkeletonPositions[JointId.ShoulderCenter]

        mapped_left_end_eff = [
            (left_end_effector.x - left_origin.x),
            (left_end_effector.y - left_origin.y),
            (left_end_effector.z - left_origin.z)
        ]

        mapped_right_end_eff = [
            (right_end_effector.x - right_origin.x),
            (right_end_effector.y - right_origin.y),
            (right_end_effector.z - right_origin.z)
        ]

        #head tricking just using trig 
        head_position_x = (head_end_effector.x - shoulderCentre.x)
        head_position_y = (head_end_effector.y - shoulderCentre.y)
        head_angle_raw = math.degrees(math.atan(head_position_x/head_position_y))

        if head_end_effector.x < shoulderCentre.x:
            head_angle = -abs(head_angle_raw-30)
        
        elif head_end_effector.x > shoulderCentre.x:
            head_angle = abs(head_angle_raw)
            



        self.left_target = np.array([-mapped_left_end_eff[2], mapped_left_end_eff[0], mapped_left_end_eff[1]])
        
        target_frame = np.eye(4)
        target_frame[:3, 3] = self.left_target
        left_ik_sol = ben.left_arm.inverse_kinematics(target_frame, initial_position=self.inital_left_ik)
        self.inital_left_ik = left_ik_sol
        left_angles_deg = np.degrees(left_ik_sol)



        self.right_target = np.array([mapped_right_end_eff[2], mapped_right_end_eff[0], mapped_right_end_eff[1]])
        target_frame_right = np.eye(4)
        target_frame_right[:3, 3] = self.right_target
        right_ik_sol = ben.right_arm.inverse_kinematics(target_frame_right, initial_position=self.inital_right_ik)
        self.inital_right_ik = right_ik_sol
        right_angles_deg = np.degrees(right_ik_sol)


        # Build single message with all servo commands
        angles = {
            #left
            13: left_angles_deg[2],
            14: left_angles_deg[1],
            8: left_angles_deg[3],

            #right
            16: right_angles_deg[1],
            17: right_angles_deg[2],
            18: right_angles_deg[3],

            10: head_angle
        }
        self.send_all_servos(angles)
        self.last_send_time = now

    def send_all_servos(self, angles, force=False):
        """Send all servo commands in a single serial message"""
        # Check which servos need updating
        to_send = {}
        for servo_id, angle in angles.items():
            last = self.last_angles.get(servo_id)
            
            if force or last is None or abs(angle - last) >= DEADBAND:
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


# ================= RUN =================
if __name__ == "__main__":
    run()
