import raylibpy as rl
import numpy as np
import math

def draw_ellipse_shadow(center, radius_x, radius_z, color, segments=36):
    points = []
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = center[0] + math.cos(angle) * radius_x
        z = center[2] + math.sin(angle) * radius_z
        points.append(rl.Vector3(x, center[1], z))
    for i in range(segments):
        rl.draw_triangle3d(
            rl.Vector3(center[0], center[1], center[2]),
            points[i],
            points[(i+1)%segments],
            color
        )
    for i in range(segments):
        rl.draw_triangle3d(
            rl.Vector3(center[0], center[1], center[2]),
            points[(i+1)%segments],
            points[i],
            color
        )

def draw_shadow_rect(start, end, width, color):
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.sqrt(dx*dx + dz*dz)
    if length == 0:
        return
    nx = -dz / length
    nz = dx / length
    half_w = width / 2
    p1 = rl.Vector3(start[0] + nx * half_w, start[1], start[2] + nz * half_w)
    p2 = rl.Vector3(start[0] - nx * half_w, start[1], start[2] - nz * half_w)
    p3 = rl.Vector3(end[0] - nx * half_w, end[1], end[2] - nz * half_w)
    p4 = rl.Vector3(end[0] + nx * half_w, end[1], end[2] + nz * half_w)
    rl.draw_triangle3d(p1, p2, p3, color)
    rl.draw_triangle3d(p1, p3, p4, color)
    rl.draw_triangle3d(p3, p2, p1, color)
    rl.draw_triangle3d(p4, p3, p1, color)

def draw_gripper_shadow(shadow_pos, color):
    finger_width = 0.08
    finger_length = 0.25
    gap = 0.1
    
    p1 = rl.Vector3(shadow_pos[0] - finger_width - gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
    p2 = rl.Vector3(shadow_pos[0] - gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
    p3 = rl.Vector3(shadow_pos[0] - gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
    p4 = rl.Vector3(shadow_pos[0] - finger_width - gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
    
    p5 = rl.Vector3(shadow_pos[0] + gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
    p6 = rl.Vector3(shadow_pos[0] + finger_width + gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
    p7 = rl.Vector3(shadow_pos[0] + finger_width + gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
    p8 = rl.Vector3(shadow_pos[0] + gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
    
    rl.draw_triangle3d(p1, p2, p3, color)
    rl.draw_triangle3d(p1, p3, p4, color)
    rl.draw_triangle3d(p3, p2, p1, color)
    rl.draw_triangle3d(p4, p3, p1, color)
    
    rl.draw_triangle3d(p5, p6, p7, color)
    rl.draw_triangle3d(p5, p7, p8, color)
    rl.draw_triangle3d(p7, p6, p5, color)
    rl.draw_triangle3d(p8, p7, p5, color)

def draw_shadows(robot, primitive, light_dir):
    light_dir = np.array(light_dir)
    
    shadow_pos = (primitive.position.x, 0.01, primitive.position.z)
    shadow_radius_x = primitive.radius * 0.5 * (1 + abs(light_dir[0]))
    shadow_radius_z = primitive.radius * 0.5 * (1 + abs(light_dir[2]))
    draw_ellipse_shadow(shadow_pos, shadow_radius_x, shadow_radius_z, rl.fade(rl.BLACK, 0.45))

    kin = robot.compute_forward_kinematics()
    joints = [kin["base"], kin["joint1"], kin["joint2"], kin["joint3"]]
    for i, joint in enumerate(joints):
        joint_pos = np.array([joint.x, joint.y, joint.z])
        t = joint_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        shadow_pos = joint_pos + light_dir * t
        shadow_pos[1] = 0.01
        
        if i == len(joints)-1:
            draw_gripper_shadow(shadow_pos, rl.fade(rl.BLACK, 0.5))
        else:
            shadow_radius_x = 0.2 * (1 + abs(light_dir[0]))
            shadow_radius_z = 0.28 * (1 + abs(light_dir[2]))
            draw_ellipse_shadow(tuple(shadow_pos), shadow_radius_x, shadow_radius_z, rl.fade(rl.BLACK, 0.35))

    for i in range(len(joints)-1):
        start_pos = np.array([joints[i].x, joints[i].y, joints[i].z])
        end_pos = np.array([joints[i+1].x, joints[i+1].y, joints[i+1].z])
        t_start = start_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        t_end = end_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        shadow_start = start_pos + light_dir * t_start
        shadow_end = end_pos + light_dir * t_end
        shadow_start[1] = 0.01
        shadow_end[1] = 0.01
        shadow_width = 0.15 * (1 + 0.5 * (abs(light_dir[0]) + abs(light_dir[2])))
        draw_shadow_rect(tuple(shadow_start), tuple(shadow_end), shadow_width, rl.fade(rl.BLACK, 0.35)) 