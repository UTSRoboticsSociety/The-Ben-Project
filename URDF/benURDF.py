import os
os.environ['TCL_LIBRARY'] = r'C:\Python27\tcl\tcl8.5'
os.environ['TK_LIBRARY'] = r'C:\Python27\tcl\tk8.5'

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# =============================
# Create the kinematic chain
# =============================

class Ben():
    def __init__(self):
        self.torso, self.left_arm, self.right_arm = self.benURDF()
    def benURDF(self):
        torso = Chain(name="toseo", links=[
            OriginLink(),
            
        ])
                    

        left_arm = Chain(name="left_arm", links=[
            OriginLink(),
            #14
            # left_shoulder (fixed to torso)
            URDFLink(
                name="left_shoulder_pitch",
                translation_vector=[0, -0.0588, 0],
                orientation=[0, 0, -np.pi/2],
                rotation=[1,0,0],
                bounds=(np.radians(-20), np.radians(150))
            ),

            #13
            #left shoulder (yaw)
            URDFLink(
                name="left_shoulder_yaw",
                translation_vector=[0.0232, 0, -0.015],
                orientation=[0, np.pi, np.pi],
                rotation=[0, 1, 0],  
                bounds=(np.radians(30), np.radians(90))
            ),
            #8
            #Left elbow pitch
            URDFLink(
                name="elbow",
                translation_vector=[0, 0, 0.0935],
                orientation=[0, 0, np.pi],
                rotation=[0, 1, 0],   # Y axis
                bounds=(np.radians(0), np.radians(60))
            ),
            URDFLink(
                name="wrist",
                translation_vector=[0, 0, 0.0381],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],   # Y axis
                bounds=(np.radians(-120), np.radians(150))
            ),


        ])

        right_arm = Chain(name="right_arm", links=[
            OriginLink(),
            # right_shoulder (fixed to torso)
            URDFLink(
                name="right_shoulder_pitch",
                translation_vector=[0, 0.0588, 0],
                orientation=[0,0,np.pi/2],
                rotation=[1,0,0],
                bounds=(np.radians(-20), np.radians(150))
            ),

            #right shoulder (yaw)
            URDFLink(
                name="right_shoulder_yaw",
                translation_vector=[0.0232, 0, -0.015],
                orientation=[0, np.pi, np.pi],
                rotation=[0, 1, 0],   # Z axis
                bounds=(np.radians(30), np.radians(90))
            ),

            #Right elbow pitch
            URDFLink(
                name="elbow",
                translation_vector=[0, 0, 0.0935],
                orientation=[0, 0, -np.pi],
                rotation=[0, 1, 0],   # Y axis
                bounds=(np.radians(0), np.radians(60))
            ),
            URDFLink(
                name="wrist",
                translation_vector=[0, 0, 0.0381],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],   # Y axis
                bounds=(np.radians(-120), np.radians(150))
            ),


        ])
        return torso, left_arm, right_arm
    
    def plot_joint_frames(self, plot_axis, chain, angles, axis_length = 0.03):
        transforms = chain.forward_kinematics(angles, full_kinematics=True)
        for i, transform in enumerate(transforms):
            origin = transform[:3, 3]
            
    # Extract rotation matrix columns (these are the axis directions)
            x_axis = transform[:3, 0]  # Red - X axis
            y_axis = transform[:3, 1]  # Green - Y axis  
            z_axis = transform[:3, 2]  # Blue - Z axis
            
            # Plot X axis (red)
            plot_axis.quiver(origin[0], origin[1], origin[2],
                    x_axis[0], x_axis[1], x_axis[2],
                    length=axis_length, color='r', arrow_length_ratio=0.3)
            
            # Plot Y axis (green)
            plot_axis.quiver(origin[0], origin[1], origin[2],
                    y_axis[0], y_axis[1], y_axis[2],
                    length=axis_length, color='g', arrow_length_ratio=0.3)
            
            # Plot Z axis (blue)
            plot_axis.quiver(origin[0], origin[1], origin[2],
                    z_axis[0], z_axis[1], z_axis[2],
                    length=axis_length, color='b', arrow_length_ratio=0.3)



    def debug(self, left_angles, right_angles):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        
        self.left_arm.plot(left_angles, ax)
        self.right_arm.plot(right_angles, ax)
        self.plot_joint_frames(ax, self.left_arm, left_angles)
        self.plot_joint_frames(ax, self.right_arm, right_angles)

        # Axes formatting
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim(-0.3,0.3)
        ax.set_ylim(-0.3,0.3)
        ax.set_zlim(0,0.4)

        ax.set_title("Ben")
        plt.show()
    

    
    def ik_target(self, target_position, arm):
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_position

        num_links = len(arm.links)
        inital_position = [0] * num_links

        ik_solution = arm.inverse_kinematics(
            target_frame,
            initial_position=inital_position
        )

        print("Joint angles (rad):")
        print(ik_solution)

        print("Joint angles (deg):")
        print(np.degrees(ik_solution))

        # =============================
        # Plot the robot
        # =============================
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')


        # Draw the robot
        arm.plot(ik_solution, ax, target=target_position)


        # Set equal aspect ratio
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-0.3,0.3)
        ax.set_ylim(-0.3,0.3)
        ax.set_zlim(0,0.4)
        ax.set_title("3-DOF Robot Arm Visualization")
        plt.show()

        return ik_solution
    

if __name__ == "__main__":
    ben = Ben()
    
    left = 1
    left_angles = [0, np.radians(0), np.radians(90), np.radians(0),np.radians(0)]
    right_angles = [0, np.radians(0), np.radians(90), np.radians(0),np.radians(0)]
    #ben.debug(left_angles,right_angles)
    
    if left == 1:


    
        #ben.debug(left_angles,right_angles)
        target_position = [0.1, -0.1, 0.02]
        ben.ik_target(target_position, ben.left_arm)
    else:

        #ben.debug(left_angles,right_angles)
        target_position = [0.1, 0.1, 0.02]
        ben.ik_target(target_position, ben.right_arm)
    

    
    
    
    

