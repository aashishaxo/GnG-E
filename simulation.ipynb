import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- Link Dimensions (in cm) ---
# Using your 10cm arm linkages, plus added base and wrist dimensions
L1_BASE = 5.0      # Height of the base
L2_ARM1 = 10.0     # Shoulder to Elbow
L3_ARM2 = 10.0     # Elbow to Wrist
L4_WRIST = 4.0     # Wrist to Gripper

# --- Linear Algebra: Transformation Matrices ---
# These functions create 4x4 matrices that rotate or translate points in 3D space.
def trans(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def rot_x(theta_deg):
    t = np.radians(theta_deg)
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(t), -np.sin(t), 0],
                     [0, np.sin(t), np.cos(t), 0],
                     [0, 0, 0, 1]])

def rot_y(theta_deg):
    t = np.radians(theta_deg)
    return np.array([[np.cos(t), 0, np.sin(t), 0],
                     [0, 1, 0, 0],
                     [-np.sin(t), 0, np.cos(t), 0],
                     [0, 0, 0, 1]])

def rot_z(theta_deg):
    t = np.radians(theta_deg)
    return np.array([[np.cos(t), -np.sin(t), 0, 0],
                     [np.sin(t), np.cos(t), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# --- Forward Kinematics Calculation ---
def update_kinematics(t1, t2, t3, t4, t5, jaw_width):
    # Origin
    H0 = np.eye(4)
    
    # Joint 1: Base Yaw (Spins around Z axis) + Move up to shoulder
    H1 = H0 @ rot_z(t1) @ trans(0, 0, L1_BASE)
    
    # Joint 2: Shoulder Pitch (Bends around Y axis) + Move up arm 1
    H2 = H1 @ rot_y(t2) @ trans(0, 0, L2_ARM1)
    
    # Joint 3: Elbow Pitch + Move up arm 2
    H3 = H2 @ rot_y(t3) @ trans(0, 0, L3_ARM2)
    
    # Joint 4 & 5: Wrist Pitch and Roll + Move to gripper
    H4 = H3 @ rot_y(t4) @ rot_x(t5) @ trans(0, 0, L4_WRIST)
    
    # Jaws: Two separate points branching off the wrist
    jaw_left = H4 @ trans(0, jaw_width, 2)
    jaw_right = H4 @ trans(0, -jaw_width, 2)
    
    # Extract just the X, Y, Z coordinates from the 4x4 matrices
    points = [H0, H1, H2, H3, H4]
    x = [p[0, 3] for p in points]
    y = [p[1, 3] for p in points]
    z = [p[2, 3] for p in points]
    
    return x, y, z, jaw_left, jaw_right, H4

# --- Setup the UI and Plot ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.45) # Make lots of room for 6 sliders!

# Draw function
def draw_robot(val=None):
    ax.cla() # Clear previous frame
    
    # Get current slider values
    t1, t2, t3 = s_base.val, s_shoulder.val, s_elbow.val
    t4, t5, jaw = s_wrist_p.val, s_wrist_r.val, s_jaw.val
    
    x, y, z, jl, jr, end_effector = update_kinematics(t1, t2, t3, t4, t5, jaw)
    
    # Plot the main arm bones
    ax.plot(x, y, z, 'o-', color='#2c3e50', linewidth=6, markersize=8)
    
    # Plot the gripper base
    hx, hy, hz = end_effector[0,3], end_effector[1,3], end_effector[2,3]
    
    # Plot Jaw Left
    ax.plot([hx, jl[0,3]], [hy, jl[1,3]], [hz, jl[2,3]], 'r-', linewidth=4)
    # Plot Jaw Right
    ax.plot([hx, jr[0,3]], [hy, jr[1,3]], [hz, jr[2,3]], 'r-', linewidth=4)
    
    # Lock the camera and limits so the graph doesn't jump around
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(0, 30)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis (Height)')
    ax.set_title('6-DOF Robotic Arm Simulator')

# --- Create Sliders ---
axcolor = 'lightgoldenrodyellow'
axes = [plt.axes([0.15, 0.35 - i*0.05, 0.65, 0.03], facecolor=axcolor) for i in range(6)]

s_base      = Slider(axes[0], 'Base Yaw', -180, 180, valinit=45)
s_shoulder  = Slider(axes[1], 'Shoulder Pitch', -90, 90, valinit=30)
s_elbow     = Slider(axes[2], 'Elbow Pitch', -90, 90, valinit=45)
s_wrist_p   = Slider(axes[3], 'Wrist Pitch', -90, 90, valinit=-30)
s_wrist_r   = Slider(axes[4], 'Wrist Roll', -180, 180, valinit=0)
s_jaw       = Slider(axes[5], 'Jaw Open', 0, 3, valinit=2)

# Attach the update function to all sliders
for s in [s_base, s_shoulder, s_elbow, s_wrist_p, s_wrist_r, s_jaw]:
    s.on_changed(draw_robot)

# Draw the initial state
draw_robot()
plt.show()