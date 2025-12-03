import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os

# --- MATLAB Style Configuration ---
def set_matlab_style():
    plt.style.use('default')
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.linestyle'] = ':'
    plt.rcParams['grid.color'] = '0.8'
    plt.rcParams['lines.linewidth'] = 1.5
    # MATLAB default colors (R2019a+):
    # 0: #0072BD (Blue)
    # 1: #D95319 (Orange)
    # 2: #EDB120 (Yellow)
    # 3: #7E2F8E (Purple)
    # 4: #77AC30 (Green)
    # 5: #4DBEEE (Cyan)
    # 6: #A2142F (Maroon)
    plt.rcParams['axes.prop_cycle'] = plt.cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F'])

def set_scope_style():
    plt.rcParams.update({
        'axes.facecolor': 'black',
        'figure.facecolor': 'gray', # Outer frame
        'axes.edgecolor': 'white',
        'axes.labelcolor': 'white',
        'xtick.color': 'white',
        'ytick.color': 'white',
        'grid.color': '0.4',
        'text.color': 'white',
        'axes.titlecolor': 'white'
    })

def reset_style():
    set_matlab_style()

# --- Trajectory Generation Functions ---

def generate_s_profile(S_total, F, A, D, Ti):
    Ta = F/A
    Td = F/D
    Sa = 0.5 * F * Ta
    Sd = 0.5 * F * Td
    
    if Sa + Sd > S_total:
        Vp = np.sqrt(2 * S_total / (1/A + 1/D))
        Ta = Vp/A
        Td = Vp/D
        Tc = 0
        Sa = 0.5 * Vp * Ta # Recalculate distances
        Sd = 0.5 * Vp * Td
    else:
        Sc = S_total - Sa - Sd
        Tc = Sc / F
        
    # Quantize time steps
    Na = int(np.ceil(Ta/Ti))
    Nc = int(np.ceil(Tc/Ti))
    Nd = int(np.ceil(Td/Ti))
    
    Ta_new = Na * Ti
    Tc_new = Nc * Ti
    Td_new = Nd * Ti
    
    # Recalculate V, A, D based on quantized time
    # S_total = 0.5*V*Ta + V*Tc + 0.5*V*Td = V * (0.5*Ta + Tc + 0.5*Td)
    V_new = S_total / (0.5*Ta_new + Tc_new + 0.5*Td_new)
    A_new = V_new / Ta_new if Ta_new > 0 else 0
    D_new = V_new / Td_new if Td_new > 0 else 0
    
    t = np.arange(Na + Nc + Nd + 1) * Ti
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
            td = time - (Ta_new + Tc_new)
            s_ddot[k] = -D_new
            s_dot[k] = V_new - D_new * td
            s[k] = 0.5 * A_new * Ta_new**2 + V_new * Tc_new + V_new * td - 0.5 * D_new * td**2
            
    return t, s, s_dot, s_ddot

def linear_interp(P_start, P_end, F, A, D, Ti):
    dist = np.linalg.norm(np.array(P_end) - np.array(P_start))
    t, s, s_dot, s_ddot = generate_s_profile(dist, F, A, D, Ti)
    
    u = (np.array(P_end) - np.array(P_start)) / dist
    pos = np.zeros((len(t), 2))
    vel = np.zeros((len(t), 2))
    acc = np.zeros((len(t), 2))
    
    for k in range(len(t)):
        pos[k] = np.array(P_start) + s[k] * u
        vel[k] = s_dot[k] * u
        acc[k] = s_ddot[k] * u
        
    return t, pos, vel, acc, s, s_dot, s_ddot

def circular_interp(P_start, Center, F, A, D, Ti, direction, angle):
    R = np.linalg.norm(np.array(P_start) - np.array(Center))
    arc_len = R * angle
    t, s, s_dot, s_ddot = generate_s_profile(arc_len, F, A, D, Ti)
    
    vec_start = np.array(P_start) - np.array(Center)
    theta_start = np.arctan2(vec_start[1], vec_start[0])
    
    pos = np.zeros((len(t), 2))
    vel = np.zeros((len(t), 2))
    acc = np.zeros((len(t), 2))
    
    for k in range(len(t)):
        theta = theta_start + direction * (s[k] / R)
        pos[k] = np.array(Center) + R * np.array([np.cos(theta), np.sin(theta)])
        
        tan_vec = direction * np.array([-np.sin(theta), np.cos(theta)])
        norm_vec = np.array([-np.cos(theta), -np.sin(theta)])
        
        vel[k] = s_dot[k] * tan_vec
        acc[k] = s_ddot[k] * tan_vec + (s_dot[k]**2 / R) * norm_vec
        
    return t, pos, vel, acc, s, s_dot, s_ddot

def combine_traj(segments, Ti):
    t_total = []
    pos_total = []
    vel_total = []
    acc_total = []
    s_total = []
    sd_total = []
    sdd_total = []
    
    curr_t = 0
    curr_s = 0
    
    for seg in segments:
        t, pos, vel, acc, s, sd, sdd = seg
        if len(t_total) > 0:
            curr_t = t_total[-1][-1] + Ti
            curr_s = s_total[-1][-1]
        
        t_total.append(t + curr_t)
        pos_total.append(pos)
        vel_total.append(vel)
        acc_total.append(acc)
        s_total.append(s + curr_s)
        sd_total.append(sd)
        sdd_total.append(sdd)
        
    return (np.concatenate(t_total), np.concatenate(pos_total), np.concatenate(vel_total), 
            np.concatenate(acc_total), np.concatenate(s_total), np.concatenate(sd_total), 
            np.concatenate(sdd_total))

# --- Controller Design Functions ---

def get_plant(axis):
    Ka = 1.0
    Kt = 0.49
    Ke = 1.59
    if axis == 'X':
        Je = 4.36e-4; B = 0.0094
    else:
        Je = 3.0e-4; B = 0.0091
    return signal.TransferFunction([Ka*Kt*Ke], [Je, B, 0])

def design_lead_lag(plant, wc_hz, PM_target):
    wc = 2 * np.pi * wc_hz
    w, mag, phase = signal.bode(plant, w=[wc])
    mag_plant = 10**(mag[0]/20)
    phase_plant = phase[0]
    
    phi_req = -180 + PM_target - phase_plant
    phi_rad = np.radians(phi_req)
    
    alpha = (1 + np.sin(phi_rad)) / (1 - np.sin(phi_rad))
    tau = 1 / (wc * np.sqrt(alpha))
    K = 1 / (mag_plant * np.sqrt(alpha))
    
    num_lead = [K * alpha * tau, K]
    den_lead = [tau, 1]
    lead = signal.TransferFunction(num_lead, den_lead)
    
    Ki = wc / 10
    integ = signal.TransferFunction([1, Ki], [1, 0])
    
    num_tot = np.convolve(num_lead, integ.num)
    den_tot = np.convolve(den_lead, integ.den)
    
    return signal.TransferFunction(num_tot, den_tot)

def main():
    set_matlab_style()
    Ti = 0.0001
    
    # --- Part A: Handout Traj ---
    F = 200; A = 1000; D = 1000
    P1=[0,0]; P2=[40,30]; P3=[60,30]; P4=[90,30]
    
    seg1 = linear_interp(P1, P2, F, A, D, Ti)
    seg2 = linear_interp(P2, P3, F, A, D, Ti)
    seg3 = circular_interp(P3, P4, F, A, D, Ti, 1, 2*np.pi)
    
    t_h, pos_h, vel_h, acc_h, s_h, sd_h, sdd_h = combine_traj([seg1, seg2, seg3], Ti)
    
    # Plot Handout
    plt.figure(figsize=(6,6))
    plt.plot(pos_h[:,0], pos_h[:,1], 'b-')
    plt.title('Handout Toolpath')
    plt.xlabel('X [mm]'); plt.ylabel('Y [mm]')
    plt.grid(True); plt.axis('equal')
    plt.savefig('handout_path.png')
    plt.close()
    
    plt.figure(figsize=(10,8))
    plt.subplot(3,1,1); plt.plot(t_h, s_h); plt.ylabel('s [mm]'); plt.grid(True)
    plt.subplot(3,1,2); plt.plot(t_h, sd_h); plt.ylabel('s_dot [mm/s]'); plt.grid(True)
    plt.subplot(3,1,3); plt.plot(t_h, sdd_h); plt.ylabel('s_ddot [mm/s^2]'); plt.grid(True)
    plt.xlabel('Time [s]')
    plt.savefig('handout_profiles.png')
    plt.close()
    
    plt.figure(figsize=(10,8))
    plt.subplot(3,1,1); plt.plot(t_h, pos_h[:,0], 'r', label='X'); plt.plot(t_h, pos_h[:,1], 'b', label='Y'); plt.ylabel('Pos'); plt.legend(); plt.grid(True)
    plt.subplot(3,1,2); plt.plot(t_h, vel_h[:,0], 'r'); plt.plot(t_h, vel_h[:,1], 'b'); plt.ylabel('Vel'); plt.grid(True)
    plt.subplot(3,1,3); plt.plot(t_h, acc_h[:,0], 'r'); plt.plot(t_h, acc_h[:,1], 'b'); plt.ylabel('Acc'); plt.grid(True)
    plt.savefig('handout_axis_cmd.png')
    plt.close()
    
    # --- Part B & C: Control & Sim ---
    cases = [
        {'name': 'LBW', 'X_wc': 20, 'Y_wc': 20},
        {'name': 'HBW', 'X_wc': 40, 'Y_wc': 40},
        {'name': 'Mismatch', 'X_wc': 40, 'Y_wc': 20}
    ]
    
    for c in cases:
        Gx = get_plant('X')
        Gy = get_plant('Y')
        
        Cx = design_lead_lag(Gx, c['X_wc'], 60)
        Cy = design_lead_lag(Gy, c['Y_wc'], 60)
        
        # Bode
        w = np.logspace(1, 4, 1000)
        
        # Open Loop
        sys_ol_x = signal.TransferFunction(np.convolve(Cx.num, Gx.num), np.convolve(Cx.den, Gx.den))
        sys_ol_y = signal.TransferFunction(np.convolve(Cy.num, Gy.num), np.convolve(Cy.den, Gy.den))
        
        wx, magx, phasex = signal.bode(sys_ol_x, w)
        wy, magy, phasey = signal.bode(sys_ol_y, w)
        
        plt.figure(figsize=(10, 6))
        plt.subplot(2,1,1)
        plt.semilogx(wx, magx, label='X'); plt.semilogx(wy, magy, label='Y')
        plt.ylabel('Mag [dB]'); plt.legend(); plt.grid(True); plt.title(f"Bode Plot - {c['name']}")
        plt.subplot(2,1,2)
        plt.semilogx(wx, phasex, label='X'); plt.semilogx(wy, phasey, label='Y')
        plt.ylabel('Phase [deg]'); plt.grid(True)
        plt.savefig(f"bode_{c['name']}.png")
        plt.close()
        
        # Simulation (Discrete)
        Gx_d = Gx.to_discrete(dt=Ti, method='zoh')
        Gy_d = Gy.to_discrete(dt=Ti, method='zoh')
        Cx_d = Cx.to_discrete(dt=Ti, method='bilinear')
        Cy_d = Cy.to_discrete(dt=Ti, method='bilinear')
        
        # Closed Loop
        # num_cl = C*G, den_cl = 1 + C*G
        num_ol_xd = np.convolve(Cx_d.num, Gx_d.num)
        den_ol_xd = np.convolve(Cx_d.den, Gx_d.den)
        # Pad
        if len(num_ol_xd) < len(den_ol_xd): num_ol_xd = np.pad(num_ol_xd, (len(den_ol_xd)-len(num_ol_xd), 0))
        sys_cl_x = signal.TransferFunction(num_ol_xd, den_ol_xd + num_ol_xd, dt=Ti)
        
        num_ol_yd = np.convolve(Cy_d.num, Gy_d.num)
        den_ol_yd = np.convolve(Cy_d.den, Gy_d.den)
        if len(num_ol_yd) < len(den_ol_yd): num_ol_yd = np.pad(num_ol_yd, (len(den_ol_yd)-len(num_ol_yd), 0))
        sys_cl_y = signal.TransferFunction(num_ol_yd, den_ol_yd + num_ol_yd, dt=Ti)
        
        # Sim Handout
        tout_x, yout_x = signal.dlsim(sys_cl_x, pos_h[:,0], t=t_h)
        tout_y, yout_y = signal.dlsim(sys_cl_y, pos_h[:,1], t=t_h)
        yout_x = np.squeeze(yout_x); yout_y = np.squeeze(yout_y)
        
        # Plot Path
        plt.figure(figsize=(6,6))
        plt.plot(pos_h[:,0], pos_h[:,1], 'k--', label='Ref')
        plt.plot(yout_x, yout_y, 'r-', label='Sim')
        plt.title(f"Simulated Path - {c['name']}")
        plt.legend(); plt.grid(True); plt.axis('equal')
        plt.savefig(f"sim_path_{c['name']}.png")
        plt.close()
        
        # Plot Tracking Error
        plt.figure()
        plt.plot(t_h, pos_h[:,0]-yout_x, label='Err X')
        plt.plot(t_h, pos_h[:,1]-yout_y, label='Err Y')
        plt.title(f"Tracking Error - {c['name']}")
        plt.legend(); plt.grid(True)
        plt.savefig(f"error_{c['name']}.png")
        plt.close()
        
        # Scope Plot (X, Y vs Time)
        with plt.rc_context(): # Use context manager to isolate scope style
            set_scope_style()
            plt.figure(facecolor='gray') # Match Simulink gray background
            plt.plot(t_h, yout_x, color='#7E2F8E', label='X') # Purple
            plt.plot(t_h, yout_y, color='#D95319', label='Y') # Red/Orange (Simulink uses Red for Y often)
            plt.plot(t_h, t_h*10, color='#EDB120', label='Clock') # Yellow
            plt.title(f"Scope Output - {c['name']}")
            plt.legend(facecolor='black', edgecolor='white', labelcolor='white')
            plt.grid(True)
            plt.savefig(f"scope_{c['name']}.png", facecolor='gray')
            plt.close()
        
    # --- Custom "G" Trajectory ---
    F_c = 100; A_c = 250; D_c = 250
    GP1=[80,80]; GP2=[40,80]; GP3=[40,20]; GP4=[80,20]; GP5=[80,50]; GP6=[60,50]
    Center_Arc = [40, 50]
    
    g_seg1 = linear_interp(GP1, GP2, F_c, A_c, D_c, Ti)
    g_seg2 = circular_interp(GP2, Center_Arc, F_c, A_c, D_c, Ti, 1, np.pi) # CCW 180
    g_seg3 = linear_interp(GP3, GP4, F_c, A_c, D_c, Ti)
    g_seg4 = linear_interp(GP4, GP5, F_c, A_c, D_c, Ti)
    g_seg5 = linear_interp(GP5, GP6, F_c, A_c, D_c, Ti)
    
    t_g, pos_g, vel_g, acc_g, s_g, sd_g, sdd_g = combine_traj([g_seg1, g_seg2, g_seg3, g_seg4, g_seg5], Ti)
    
    # Plot Custom Path
    plt.figure(figsize=(6,6))
    plt.plot(pos_g[:,0], pos_g[:,1], 'b-')
    plt.title('Custom "G" Toolpath')
    plt.grid(True); plt.axis('equal')
    plt.savefig('custom_path.png')
    plt.close()
    
    plt.figure(figsize=(10,8))
    plt.subplot(3,1,1); plt.plot(t_g, s_g); plt.ylabel('s'); plt.grid(True)
    plt.subplot(3,1,2); plt.plot(t_g, sd_g); plt.ylabel('s_dot'); plt.grid(True)
    plt.subplot(3,1,3); plt.plot(t_g, sdd_g); plt.ylabel('s_ddot'); plt.grid(True)
    plt.savefig('custom_profiles.png')
    plt.close()
    
    # Sim Custom (LBW only)
    # Re-use LBW sys_cl_x/y from last loop iteration? No, last was Mismatch.
    # Re-create LBW
    Gx = get_plant('X'); Gy = get_plant('Y')
    Cx = design_lead_lag(Gx, 20, 60); Cy = design_lead_lag(Gy, 20, 60)
    Gx_d = Gx.to_discrete(dt=Ti, method='zoh'); Gy_d = Gy.to_discrete(dt=Ti, method='zoh')
    Cx_d = Cx.to_discrete(dt=Ti, method='bilinear'); Cy_d = Cy.to_discrete(dt=Ti, method='bilinear')
    
    num_ol_xd = np.convolve(Cx_d.num, Gx_d.num); den_ol_xd = np.convolve(Cx_d.den, Gx_d.den)
    if len(num_ol_xd) < len(den_ol_xd): num_ol_xd = np.pad(num_ol_xd, (len(den_ol_xd)-len(num_ol_xd), 0))
    sys_cl_x = signal.TransferFunction(num_ol_xd, den_ol_xd + num_ol_xd, dt=Ti)
    
    num_ol_yd = np.convolve(Cy_d.num, Gy_d.num); den_ol_yd = np.convolve(Cy_d.den, Gy_d.den)
    if len(num_ol_yd) < len(den_ol_yd): num_ol_yd = np.pad(num_ol_yd, (len(den_ol_yd)-len(num_ol_yd), 0))
    sys_cl_y = signal.TransferFunction(num_ol_yd, den_ol_yd + num_ol_yd, dt=Ti)
    
    tout_xg, yout_xg = signal.dlsim(sys_cl_x, pos_g[:,0], t=t_g)
    tout_yg, yout_yg = signal.dlsim(sys_cl_y, pos_g[:,1], t=t_g)
    yout_xg = np.squeeze(yout_xg); yout_yg = np.squeeze(yout_yg)
    
    plt.figure(figsize=(6,6))
    plt.plot(pos_g[:,0], pos_g[:,1], 'k--', label='Ref')
    plt.plot(yout_xg, yout_yg, 'r-', label='Sim')
    plt.title('Custom Trajectory Sim (LBW)')
    plt.legend(); plt.grid(True); plt.axis('equal')
    plt.savefig('custom_sim_path.png')
    plt.close()
    
    plt.figure()
    plt.plot(t_g, pos_g[:,0]-yout_xg, label='Err X')
    plt.plot(t_g, pos_g[:,1]-yout_yg, label='Err Y')
    plt.title('Custom Tracking Error (LBW)')
    plt.legend(); plt.grid(True)
    plt.savefig('custom_error.png')
    plt.close()
    
    with plt.rc_context():
        set_scope_style()
        plt.figure(facecolor='gray')
        plt.plot(t_g, yout_xg, color='#7E2F8E', label='X')
        plt.plot(t_g, yout_yg, color='#D95319', label='Y')
        plt.plot(t_g, t_g*10, color='#EDB120', label='Clock')
        plt.title('Custom Scope Output')
        plt.legend(facecolor='black', edgecolor='white', labelcolor='white')
        plt.grid(True)
        plt.savefig('custom_scope.png', facecolor='gray')
        plt.close()

if __name__ == "__main__":
    main()
