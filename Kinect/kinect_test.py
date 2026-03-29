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
                for i in range(kinect.max_body_count):
                    body = bodies.bodies[i]
                    if body.is_tracked:
                        joints = body.joints
                        # Example: grab the right hand joint
                        right_hand = joints[JointType_HandRight]
                        positionRH = right_hand.Position

                        print(positionRH)

                    elif not body.is_tracked:
                        print("no body found")

                    # Also available per body:
                    # body.joint_orientations[JointType_HandRight]  ← quaternion
                    # body.hand_right_state   ← open/closed/lasso
                    # body.tracking_id        ← unique ID per person

finally:
    kinect.close()