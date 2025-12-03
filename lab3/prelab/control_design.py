import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os

def get_plant(axis):
    # Parameters from Table 1
    Ka = 1.0
    Kt = 0.49
    Ke = 1.59 # mm/rad
    
    if axis == 'X':
        Je = 4.36e-4
        B = 0.0094
    elif axis == 'Y':
        Je = 3.0e-4
        B = 0.0091
    
    # G(s) = Ka * Kt * Ke / (s * (Je * s + B))
    # Numerator: Ka*Kt*Ke
    # Denominator: Je*s^2 + B*s
    num = [Ka * Kt * Ke]
    den = [Je, B, 0]
    
    return signal.TransferFunction(num, den)

def design_lead(plant, wc_hz, PM_target):
    wc = 2 * np.pi * wc_hz
    w, mag, phase = signal.bode(plant, w=[wc])
    
    mag_plant = 10**(mag[0]/20)
    phase_plant = phase[0]
    
    # Required phase from controller
    # Angle(GC) = -180 + PM
    # Angle(G) + Angle(C) = -180 + PM
    # Angle(C) = -180 + PM - Angle(G)
    
    # Ensure phase is in [-360, 0] range for G usually starts at -90 or -180
    # Plant is Type 1, so phase starts at -90 and goes to -180.
    # At wc, phase should be between -90 and -180.
    
    phi_req = -180 + PM_target - phase_plant
    
    # Lead compensator angle phi_max
    # alpha = (1 + sin(phi)) / (1 - sin(phi))
    # phi must be in radians
    phi_rad = np.radians(phi_req)
    if phi_rad <= 0:
        alpha = 1
        tau = 0
        K = 1/mag_plant
    else:
        alpha = (1 + np.sin(phi_rad)) / (1 - np.sin(phi_rad))
        tau = 1 / (wc * np.sqrt(alpha))
        # Magnitude of C at wc is K * sqrt(alpha) ?
        # |C(jw)| = K * |(j w alpha tau + 1) / (j w tau + 1)|
        # = K * sqrt( (alpha w tau)^2 + 1 ) / sqrt( (w tau)^2 + 1 )
        # At w = 1/(tau sqrt(alpha)):
        # |C| = K * sqrt( (alpha/sqrt(alpha))^2 + 1 ) / sqrt( (1/sqrt(alpha))^2 + 1 )
        # = K * sqrt( alpha + 1 ) / sqrt( 1/alpha + 1 )
        # = K * sqrt( alpha + 1 ) / ( sqrt(1+alpha)/sqrt(alpha) )
        # = K * sqrt(alpha)
        
        # We want |G||C| = 1 => |C| = 1/|G|
        # K * sqrt(alpha) = 1/mag_plant
        K = 1 / (mag_plant * np.sqrt(alpha))
        
    num = [K * alpha * tau, K]
    den = [tau, 1]
    
    return signal.TransferFunction(num, den), K, alpha, tau

def add_integrator(lead_sys, wc_hz):
    wc = 2 * np.pi * wc_hz
    Ki = wc / 10
    # C_I(s) = (s + Ki) / s
    num = [1, Ki]
    den = [1, 0]
    integrator = signal.TransferFunction(num, den)
    
    # Series: Lead * Integrator
    total_num = np.convolve(lead_sys.num, num)
    total_den = np.convolve(lead_sys.den, den)
    
    return signal.TransferFunction(total_num, total_den)

def discretize(sys, Ts, method='zoh'):
    # Scipy doesn't have c2d with 'tustin' directly in TransferFunction.to_discrete?
    # It has to_discrete.
    # method: 'gbt', 'bilinear', 'euler', 'backward_diff', 'zoh', 'foh', 'impulse'
    # 'bilinear' is Tustin.
    if method == 'tustin':
        dt_sys = sys.to_discrete(dt=Ts, method='bilinear')
    else:
        dt_sys = sys.to_discrete(dt=Ts, method=method)
    return dt_sys

def get_step_info(sys_d, T_sim=1.0):
    t, y = signal.dstep(sys_d, n=int(T_sim/sys_d.dt))
    # y is tuple (t, y) or just y? dstep returns t, y
    # y is (n_samples, n_outputs). Squeeze it.
    y = np.squeeze(y[0]) # dstep returns (t, y) where y is list of arrays?
    # Wait, signal.dstep returns (t, y). y has shape (n,).
    
    # Rise time: 10% to 90%
    final_val = y[-1]
    t10 = t[np.where(y >= 0.1 * final_val)[0][0]]
    t90 = t[np.where(y >= 0.9 * final_val)[0][0]]
    tr = t90 - t10
    
    # Overshoot
    peak_val = np.max(y)
    os = (peak_val - final_val) / final_val * 100
    
    return tr, os, t, y

def main():
    Ts = 0.0001
    
    # Cases
    cases = [
        {'name': 'LBW', 'X': {'wc': 20, 'PM': 60}, 'Y': {'wc': 20, 'PM': 60}},
        {'name': 'HBW', 'X': {'wc': 40, 'PM': 60}, 'Y': {'wc': 40, 'PM': 60}},
        {'name': 'Mismatch', 'X': {'wc': 40, 'PM': 60}, 'Y': {'wc': 20, 'PM': 60}}
    ]
    
    results = []
    
    for case in cases:
        case_res = {'name': case['name']}
        
        for axis_name in ['X', 'Y']:
            params = case[axis_name]
            plant = get_plant(axis_name)
            
            # Design Lead
            lead, K, alpha, tau = design_lead(plant, params['wc'], params['PM'])
            
            # Add Integrator
            controller_c = add_integrator(lead, params['wc'])
            
            # Discretize
            plant_d = discretize(plant, Ts, 'zoh')
            controller_d = discretize(controller_c, Ts, 'tustin')
            
            # Closed Loop
            # G_cl = (C_d * G_d) / (1 + C_d * G_d)
            # Use feedback function if available, or algebra
            # Scipy doesn't have feedback function easily for TF?
            # num = num_c*num_g, den = den_c*den_g + num_c*num_g
            
            num_open = np.convolve(controller_d.num, plant_d.num)
            den_open = np.convolve(controller_d.den, plant_d.den)
            
            # Pad to same length
            if len(num_open) < len(den_open):
                num_open = np.pad(num_open, (len(den_open)-len(num_open), 0), 'constant')
            
            den_closed = den_open + num_open
            sys_cl = signal.TransferFunction(num_open, den_closed, dt=Ts)
            
            # Step Info
            tr, os_val, t_step, y_step = get_step_info(sys_cl)
            
            case_res[axis_name] = {
                'sys_cl': sys_cl,
                'tr': tr,
                'os': os_val,
                'poles': sys_cl.poles,
                'zeros': sys_cl.zeros,
                'controller_d': controller_d,
                'plant_d': plant_d
            }
            
            # Plot Bode Open Loop
            # Open Loop = controller_c * plant
            # Need to combine num/den
            num_ol_c = np.convolve(controller_c.num, plant.num)
            den_ol_c = np.convolve(controller_c.den, plant.den)
            sys_ol_c = signal.TransferFunction(num_ol_c, den_ol_c)
            
            w = np.logspace(0, 4, 1000)
            w, mag, phase = signal.bode(sys_ol_c, w)
            
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.semilogx(w, mag)
            plt.title(f'Bode Plot {case["name"]} Axis {axis_name}')
            plt.grid(True)
            plt.ylabel('Magnitude [dB]')
            
            plt.subplot(2, 1, 2)
            plt.semilogx(w, phase)
            plt.grid(True)
            plt.ylabel('Phase [deg]')
            plt.xlabel('Frequency [rad/s]')
            plt.savefig(f'bode_{case["name"]}_{axis_name}.png')
            plt.close()
            
        results.append(case_res)
        
    # Save results to text file for report
    with open('controller_results.txt', 'w') as f:
        for res in results:
            f.write(f"Case: {res['name']}\n")
            for axis in ['X', 'Y']:
                data = res[axis]
                f.write(f"  Axis {axis}:\n")
                f.write(f"    Rise Time: {data['tr']*1000:.2f} ms\n")
                f.write(f"    Overshoot: {data['os']:.2f} %\n")
                f.write(f"    Bandwidth: {res['name']} (Target)\n") # Calculated BW is tricky with dstep
                f.write(f"    Poles: {data['poles']}\n")
                f.write(f"    Zeros: {data['zeros']}\n")
                
    # Simulate Tracking (Part C.1)
    # Load trajectory
    traj = np.load('handout_traj.npz')
    t = traj['t']
    pos = traj['pos']
    ref_x = pos[:, 0]
    ref_y = pos[:, 1]
    
    # Use Case 1 (LBW) for simulation as per handout?
    # Handout C.1 says "Verify that your trajectories and controllers result in stable contouring".
    # Usually we use the designed controller. Let's use LBW (Case 1).
    
    case1 = results[0]
    sys_cl_x = case1['X']['sys_cl']
    sys_cl_y = case1['Y']['sys_cl']
    
    # dlsim
    # dlsim(system, u, t=None, x0=None)
    # u must have shape (n_samples,)
    
    # Interpolate trajectory to match Ts?
    # The trajectory was generated with Ti=0.1ms, which matches Ts.
    # So we can use it directly.
    
    tout_x, yout_x = signal.dlsim(sys_cl_x, ref_x, t=t)
    tout_y, yout_y = signal.dlsim(sys_cl_y, ref_y, t=t)
    
    # Squeeze outputs
    yout_x = np.squeeze(yout_x)
    yout_y = np.squeeze(yout_y)
    
    # Plot Simulated Path vs Reference
    plt.figure(figsize=(6, 6))
    plt.plot(ref_x, ref_y, 'b--', label='Reference')
    plt.plot(yout_x, yout_y, 'r-', label='Simulated')
    plt.xlabel('X [mm]')
    plt.ylabel('Y [mm]')
    plt.title('Simulated Contouring Performance (LBW)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.savefig('simulated_contour_lbw.png')
    plt.close()
    
    # Plot Tracking Errors
    err_x = ref_x - yout_x
    err_y = ref_y - yout_y
    
    plt.figure()
    plt.plot(t, err_x, label='Error X')
    plt.plot(t, err_y, label='Error Y')
    plt.xlabel('Time [s]')
    plt.ylabel('Error [mm]')
    plt.title('Tracking Errors (LBW)')
    plt.legend()
    plt.grid(True)
    plt.savefig('tracking_errors_lbw.png')
    plt.close()

    # Simulate Custom Trajectory (Part C.2)
    traj_cust = np.load('custom_traj.npz')
    t_cust = traj_cust['t']
    pos_cust = traj_cust['pos']
    ref_x_cust = pos_cust[:, 0]
    ref_y_cust = pos_cust[:, 1]
    
    tout_x_cust, yout_x_cust = signal.dlsim(sys_cl_x, ref_x_cust, t=t_cust)
    tout_y_cust, yout_y_cust = signal.dlsim(sys_cl_y, ref_y_cust, t=t_cust)
    
    yout_x_cust = np.squeeze(yout_x_cust)
    yout_y_cust = np.squeeze(yout_y_cust)
    
    # Plot Custom Simulated Path
    plt.figure(figsize=(6, 6))
    plt.plot(ref_x_cust, ref_y_cust, 'b--', label='Reference')
    plt.plot(yout_x_cust, yout_y_cust, 'r-', label='Simulated')
    plt.xlabel('X [mm]')
    plt.ylabel('Y [mm]')
    plt.title('Simulated Custom Trajectory (LBW)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.savefig('simulated_contour_custom.png')
    plt.close()
    
    # Plot Custom Tracking Errors
    err_x_cust = ref_x_cust - yout_x_cust
    err_y_cust = ref_y_cust - yout_y_cust
    
    plt.figure()
    plt.plot(t_cust, err_x_cust, label='Error X')
    plt.plot(t_cust, err_y_cust, label='Error Y')
    plt.xlabel('Time [s]')
    plt.ylabel('Error [mm]')
    plt.title('Tracking Errors Custom (LBW)')
    plt.legend()
    plt.grid(True)
    plt.savefig('tracking_errors_custom.png')
    plt.close()

if __name__ == "__main__":
    main()
