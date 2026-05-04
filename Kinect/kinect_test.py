from pykinect2024 import PyKinect2024
from pykinect2024.PyKinect2024 import *
from pykinect2024.PyKinectRuntime import PyKinectRuntime

kinect = PyKinectRuntime(FrameSourceTypes_Body)

print("Kinect open — stand in front of it!")

try:
    while True:
        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()

            if bodies is not None:
                for i in range(0,kinect.max_body_count):
                    body = bodies.bodies[i]
                    if not body.is_tracked:
                        continue
                    
                    elif body.is_tracked:
                        joints = body.joints
                        
                        right_hand = joints[PyKinect2024.JointType_HandRight]
                        positionRH = right_hand.Position

                        print(positionRH.x)
                        
                        

                    # body.joint_orientations[JointType_HandRight]  ← quaternion
                    # body.tracking_id        ← unique ID per person

finally:
    kinect.close()