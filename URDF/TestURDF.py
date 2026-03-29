import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink, OriginLink

# ── 1. Build the chain manually (equivalent to loading a URDF) ──────────────
arm_chain = Chain(name='robot_arm', links=[
    OriginLink(),                          # fixed world anchor

    URDFLink(
        name="base_rotation",
        origin_translation=[0, 0, 0.1],   # 10 cm up from world origin
        origin_orientation=[0, 0, 0],      # no initial rotation
        rotation=[0, 0, 1],               # rotates around Z axis (yaw)
    ),

    URDFLink(
        name="shoulder",
        origin_translation=[0, 0, 0.2],   # 20 cm up from base
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],               # rotates around Y axis (pitch)
    ),

    URDFLink(
        name="elbow",
        origin_translation=[0.3, 0, 0],   # 30 cm along X (upper arm length)
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],               # pitch
    ),

    URDFLink(
        name="wrist",
        origin_translation=[0.25, 0, 0],  # 25 cm along X (forearm length)
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],               # pitch
    ),

    URDFLink(
        name="end_effector",
        origin_translation=[0.1, 0, 0],   # 10 cm tool length
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],               # no active rotation (fixed tip)
    ),
])

# ── 2. Forward kinematics — where is the tip given joint angles? ─────────────
# One angle per active link (OriginLink and the fixed end-effector are passive)
joint_angles = [0, np.pi/4, -np.pi/4, np.pi/6, 0]  # radians
fk_matrix = arm_chain.forward_kinematics(joint_angles)

tip_position = fk_matrix[:3, 3]           # XYZ translation column
print("FK tip position:", np.round(tip_position, 4))

# ── 3. Inverse kinematics — what angles reach a target position? ─────────────
target_position = [0.5, 0.1, 0.3]        # desired XYZ in metres

ik_angles = arm_chain.inverse_kinematics(
    target_position=target_position,
    initial_position=joint_angles,        # seed angles (warm start)
    # target_orientation=...,            # optional 3×3 rotation matrix
    # orientation_mode="X",              # "X", "Y", "Z", or "all"
)

print("IK joint angles (rad):", np.round(ik_angles, 4))
print("IK joint angles (deg):", np.round(np.degrees(ik_angles), 2))

# Verify by running FK on the IK solution
verify_fk = arm_chain.forward_kinematics(ik_angles)
print("Achieved position:    ", np.round(verify_fk[:3, 3], 4))
print("Target  position:     ", target_position)

# ── 4. Load from an actual URDF file (alternative to building manually) ──────
# from ikpy.chain import Chain
#
# arm_chain = Chain.from_urdf_file(
#     "my_robot.urdf",
#     base_elements=["base_link"],        # link to start the chain from
#     last_link_vector=[0.1, 0, 0],       # tip offset past the last joint
#     active_links_mask=[False, True, True, True, True, False],
# )