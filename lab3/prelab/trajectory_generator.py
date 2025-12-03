import numpy as np
import matplotlib.pyplot as plt
import os

def generate_profile(S_total, F_des, A_des, D_des, Ti):
    # Calculate initial times
    Ta = F_des / A_des
    Td = F_des / D_des
    S_acc = 0.5 * F_des * Ta
    S_dec = 0.5 * F_des * Td
    
    if S_acc + S_dec > S_total:
        # Triangular profile
        # S_total = 0.5 * V_peak * (Ta + Td) = 0.5 * V_peak^2 * (1/A + 1/D)
        # V_peak = sqrt(2 * S_total / (1/A + 1/D))
        V_peak = np.sqrt(2 * S_total / (1/A_des + 1/D_des))
        Ta = V_peak / A_des
        Td = V_peak / D_des
        Tc = 0
    else:
        S_const = S_total - S_acc - S_dec
        Tc = S_const / F_des
        V_peak = F_des

    # Quantize times
    Na = int(np.ceil(Ta / Ti))
    Nc = int(np.ceil(Tc / Ti))
    Nd = int(np.ceil(Td / Ti))
    
    Ta_new = Na * Ti
    Tc_new = Nc * Ti
    Td_new = Nd * Ti
    
    # Recalculate kinematics
    # S_total = V_new * (0.5 * Ta_new + Tc_new + 0.5 * Td_new)
    V_new = S_total / (0.5 * Ta_new + Tc_new + 0.5 * Td_new)
    A_new = V_new / Ta_new if Ta_new > 0 else 0
    D_new = V_new / Td_new if Td_new > 0 else 0
    
    # Generate profile
    N_total = Na + Nc + Nd
    t = np.arange(N_total + 1) * Ti
    s = np.zeros_like(t)
    s_dot = np.zeros_like(t)
    s_ddot = np.zeros_like(t)
    
    for k in range(len(t)):
        time = t[k]
        if time <= Ta_new:
            s_ddot[k] = A_new
            s_dot[k] = A_new * time
            s[k] = 0.5 * A_new * time**2
        elif time <= Ta_new + Tc_new:
            s_ddot[k] = 0
            s_dot[k] = V_new
            s[k] = 0.5 * A_new * Ta_new**2 + V_new * (time - Ta_new)
        else:
            t_dec = time - (Ta_new + Tc_new)
            s_ddot[k] = -D_new
            s_dot[k] = V_new - D_new * t_dec
            s[k] = 0.5 * A_new * Ta_new**2 + V_new * Tc_new + V_new * t_dec - 0.5 * D_new * t_dec**2
            
    return t, s, s_dot, s_ddot

def linear_interp(P_start, P_end, F, A, D, Ti):
    dist = np.linalg.norm(np.array(P_end) - np.array(P_start))
    t, s, s_dot, s_ddot = generate_profile(dist, F, A, D, Ti)
    
    # Interpolate positions
    unit_vec = (np.array(P_end) - np.array(P_start)) / dist
    
    pos = np.zeros((len(t), 2))
    vel = np.zeros((len(t), 2))
    acc = np.zeros((len(t), 2))
    
    for k in range(len(t)):
        pos[k] = np.array(P_start) + s[k] * unit_vec
        vel[k] = s_dot[k] * unit_vec
        acc[k] = s_ddot[k] * unit_vec
        
    return t, pos, vel, acc, s, s_dot, s_ddot

def circular_interp(P_start, Center, F, A, D, Ti, direction, angle_total):
    # direction: 1 for CCW, -1 for CW
    # angle_total: radians (positive)
    
    radius = np.linalg.norm(np.array(P_start) - np.array(Center))
    arc_length = radius * angle_total
    
    t, s, s_dot, s_ddot = generate_profile(arc_length, F, A, D, Ti)
    
    # Determine start angle
    vec_start = np.array(P_start) - np.array(Center)
    theta_start = np.arctan2(vec_start[1], vec_start[0])
    
    pos = np.zeros((len(t), 2))
    vel = np.zeros((len(t), 2))
    acc = np.zeros((len(t), 2))
    
    for k in range(len(t)):
        # Current angle
        theta = theta_start + direction * (s[k] / radius)
        
        # Position
        pos[k] = np.array(Center) + radius * np.array([np.cos(theta), np.sin(theta)])
        
        # Velocity (tangential)
        # v_vec = [-sin(theta), cos(theta)] * v_mag * direction
        vel[k] = s_dot[k] * direction * np.array([-np.sin(theta), np.cos(theta)])
        
        # Acceleration
        # a_vec = a_tangential + a_normal
        # a_tan = s_ddot * direction * [-sin, cos]
        # a_norm = - (v^2 / R) * [cos, sin]
        tan_vec = direction * np.array([-np.sin(theta), np.cos(theta)])
        norm_vec = np.array([-np.cos(theta), -np.sin(theta)]) # Pointing to center? No, -cos, -sin points to center from surface
        
        acc[k] = s_ddot[k] * tan_vec + (s_dot[k]**2 / radius) * norm_vec

    return t, pos, vel, acc, s, s_dot, s_ddot

def save_plots(prefix, t, pos, vel, acc, s, s_dot, s_ddot):
    # Toolpath
    plt.figure(figsize=(6, 6))
    plt.plot(pos[:, 0], pos[:, 1], 'b-')
    plt.plot(pos[0, 0], pos[0, 1], 'go', label='Start')
    plt.plot(pos[-1, 0], pos[-1, 1], 'ro', label='End')
    plt.xlabel('X [mm]')
    plt.ylabel('Y [mm]')
    plt.title(f'{prefix} Toolpath')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.savefig(f'{prefix}_toolpath.png')
    plt.close()
    
    # S profiles
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(t, s)
    plt.ylabel('s [mm]')
    plt.title('Displacement Profile')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(t, s_dot)
    plt.ylabel('s_dot [mm/s]')
    plt.title('Feedrate Profile')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(t, s_ddot)
    plt.ylabel('s_ddot [mm/s^2]')
    plt.title('Tangential Acceleration Profile')
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.tight_layout()
    plt.savefig(f'{prefix}_s_profiles.png')
    plt.close()
    
    # Axis profiles
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(t, pos[:, 0], label='X')
    plt.plot(t, pos[:, 1], label='Y')
    plt.ylabel('Position [mm]')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(t, vel[:, 0], label='Vx')
    plt.plot(t, vel[:, 1], label='Vy')
    plt.ylabel('Velocity [mm/s]')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(t, acc[:, 0], label='Ax')
    plt.plot(t, acc[:, 1], label='Ay')
    plt.ylabel('Acceleration [mm/s^2]')
    plt.legend()
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.tight_layout()
    plt.savefig(f'{prefix}_axis_profiles.png')
    plt.close()

def main():
    # --- Part A: Handout Trajectory ---
    Ti = 0.0001 # 0.1 ms
    F = 200
    A = 1000
    D = 1000
    
    P1 = [0, 0]
    P2 = [40, 30]
    P3 = [60, 30]
    P4 = [90, 30] # Center
    
    # Segment 1: P1 -> P2
    t1, pos1, vel1, acc1, s1, sd1, sdd1 = linear_interp(P1, P2, F, A, D, Ti)
    
    # Segment 2: P2 -> P3
    t2, pos2, vel2, acc2, s2, sd2, sdd2 = linear_interp(P2, P3, F, A, D, Ti)
    
    # Segment 3: P3 -> P3 (Circle)
    # Full circle CCW
    t3, pos3, vel3, acc3, s3, sd3, sdd3 = circular_interp(P3, P4, F, A, D, Ti, 1, 2*np.pi)
    
    # Combine
    # Need to shift time and s
    t2_shift = t2 + t1[-1] + Ti
    t3_shift = t3 + t2_shift[-1] + Ti
    
    s2_shift = s2 + s1[-1]
    s3_shift = s3 + s2_shift[-1]
    
    t_total = np.concatenate([t1, t2_shift, t3_shift])
    pos_total = np.concatenate([pos1, pos2, pos3])
    vel_total = np.concatenate([vel1, vel2, vel3])
    acc_total = np.concatenate([acc1, acc2, acc3])
    s_total = np.concatenate([s1, s2_shift, s3_shift])
    sd_total = np.concatenate([sd1, sd2, sd3])
    sdd_total = np.concatenate([sdd1, sdd2, sdd3])
    
    save_plots('handout', t_total, pos_total, vel_total, acc_total, s_total, sd_total, sdd_total)
    
    # Save data for Part C simulation
    np.savez('handout_traj.npz', t=t_total, pos=pos_total)

    # --- Part C.2: Custom Trajectory (M Shape) ---
    F_cust = 100
    A_cust = 250
    D_cust = 250
    
    # Points
    CP1 = [0, 0]
    CP2 = [10, 40]
    CP3 = [30, 40]
    CP4 = [50, 0]
    CP5 = [70, 40]
    CP6 = [90, 40]
    CP7 = [100, 0]
    
    Center1 = [20, 40] # For CP2->CP3
    Center2 = [80, 40] # For CP5->CP6
    
    # Seg 1: CP1 -> CP2
    ct1, cpos1, cvel1, cacc1, cs1, csd1, csdd1 = linear_interp(CP1, CP2, F_cust, A_cust, D_cust, Ti)
    
    # Seg 2: CP2 -> CP3 (CW arc)
    ct2, cpos2, cvel2, cacc2, cs2, csd2, csdd2 = circular_interp(CP2, Center1, F_cust, A_cust, D_cust, Ti, -1, np.pi)
    
    # Seg 3: CP3 -> CP4
    ct3, cpos3, cvel3, cacc3, cs3, csd3, csdd3 = linear_interp(CP3, CP4, F_cust, A_cust, D_cust, Ti)
    
    # Seg 4: CP4 -> CP5
    ct4, cpos4, cvel4, cacc4, cs4, csd4, csdd4 = linear_interp(CP4, CP5, F_cust, A_cust, D_cust, Ti)
    
    # Seg 5: CP5 -> CP6 (CW arc)
    ct5, cpos5, cvel5, cacc5, cs5, csd5, csdd5 = circular_interp(CP5, Center2, F_cust, A_cust, D_cust, Ti, -1, np.pi)
    
    # Seg 6: CP6 -> CP7
    ct6, cpos6, cvel6, cacc6, cs6, csd6, csdd6 = linear_interp(CP6, CP7, F_cust, A_cust, D_cust, Ti)
    
    # Combine
    ct_list = [ct1, ct2, ct3, ct4, ct5, ct6]
    cpos_list = [cpos1, cpos2, cpos3, cpos4, cpos5, cpos6]
    cvel_list = [cvel1, cvel2, cvel3, cvel4, cvel5, cvel6]
    cacc_list = [cacc1, cacc2, cacc3, cacc4, cacc5, cacc6]
    cs_list = [cs1, cs2, cs3, cs4, cs5, cs6]
    csd_list = [csd1, csd2, csd3, csd4, csd5, csd6]
    csdd_list = [csdd1, csdd2, csdd3, csdd4, csdd5, csdd6]
    
    ct_total = []
    cpos_total = []
    cvel_total = []
    cacc_total = []
    cs_total = []
    csd_total = []
    csdd_total = []
    
    current_time = 0
    current_s = 0
    
    for i in range(6):
        if i > 0:
            current_time += ct_list[i-1][-1] + Ti
            current_s += cs_list[i-1][-1]
            
        ct_total.append(ct_list[i] + current_time)
        cpos_total.append(cpos_list[i])
        cvel_total.append(cvel_list[i])
        cacc_total.append(cacc_list[i])
        cs_total.append(cs_list[i] + current_s)
        csd_total.append(csd_list[i])
        csdd_total.append(csdd_list[i])
        
    ct_final = np.concatenate(ct_total)
    cpos_final = np.concatenate(cpos_total)
    cvel_final = np.concatenate(cvel_total)
    cacc_final = np.concatenate(cacc_total)
    cs_final = np.concatenate(cs_total)
    csd_final = np.concatenate(csd_total)
    csdd_final = np.concatenate(csdd_total)
    
    save_plots('custom', ct_final, cpos_final, cvel_final, cacc_final, cs_final, csd_final, csdd_final)
    np.savez('custom_traj.npz', t=ct_final, pos=cpos_final)

if __name__ == "__main__":
    main()
