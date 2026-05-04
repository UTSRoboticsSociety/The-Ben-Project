from pykinect2024 import PyKinect2024
from pykinect2024.PyKinect2024 import *
from pykinect2024.PyKinectRuntime import PyKinectRuntime

#This code is the barebones basics in getting an output from the xbox kinect v2

kinect = PyKinectRuntime(FrameSourceTypes_Body)

print("Kinect open — stand in front of it!")

try:
    while True:
        if kinect.has_new_body_frame(): #checks if the kinect has recieved new body frame (new person walks into frame)
            bodies = kinect.get_last_body_frame() #from the detected body in frame, we obtain the joint positions and tracking status 

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