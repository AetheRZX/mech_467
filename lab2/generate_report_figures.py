
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control
import math
from pathlib import Path

# --- System Parameters (Same as generate_figures.py) ---
KA = 0.887
KT = 0.72
JE = 7e-4
BE = 0.00612
KE = 20 / (2 * math.pi)
T_S = 0.0002  # 0.2 ms

def get_plant():
    num = [KA * KT * KE]
    den = [JE, BE, 0.0]
    Gs = control.TransferFunction(num, den)
    Gz = control.c2d(Gs, T_S, method='zoh')
    return Gs, Gz

def get_controllers(Gz):
    # P Controller (Q4)
    # Kp from prelab
    Kp = 1.253
    C_P = Kp
    
    # Lead-Lag (Q5)
    # C_LL(z) = (155.5 z - 152.3) / (z - 0.763)
    # From prelab text/code
    num_ll = [155.5, -152.3]
    den_ll = [1.0, -0.763]
    C_LL = control.TransferFunction(num_ll, den_ll, dt=T_S)
    
    # Lead-Lag-Integrator (Q5)
    # C_LLI(z) = (156.1 z^2 - 307.8 z + 151.7) / (z^2 - 1.763 z + 0.763)
    num_lli = [156.1, -307.8, 151.7]
    den_lli = [1.0, -1.763, 0.763]
    C_LLI = control.TransferFunction(num_lli, den_lli, dt=T_S)
    
    return C_P, C_LL, C_LLI

def load_data(path):
    # Header at line 7 (0-indexed) means row 7 is header.
    # Data units at line 8.
    # Real data starts after.
    # We'll read with header=7, then drop units row (row 0 of df).
    try:
        # Try modern pandas
        try:
            df = pd.read_csv(path, header=7, encoding='utf-8', on_bad_lines='skip')
        except TypeError:
            # Older pandas
            df = pd.read_csv(path, header=7, encoding='utf-8', error_bad_lines=False)
            
        # Drop units row (first row usually) and any text rows
        # We can just force coerce Time to numeric and drop NaNs
        
        # Map columns
        # Based on user image:
        # 0: Name, 1: Ref, 2: Enc1_P, 3: Enc2_P, 4: Enc1_V, 5: Enc2_V, 6: Vout
        
        # We want Ref (1) and Enc2_P (3)
        # Note: In previous read, Ref was 4? Let's be careful.
        # If the file structure matches the image "Name, RefTraj..." 
        # then Ref is 1, Enc2 is 3.
        
        # Let's try to detect by name if possible, or fall back to indices 1 and 3.
        # But wait, my previous run with index 4/5 gave reasonable plots (Step tracking).
        # If I change to 1/3, I must be sure.
        
        # Let's inspect the columns by name in the script
        cols = df.columns
        # Clean names
        cols = [c.strip() for c in cols]
        
        ref_idx = 1
        act_idx = 3
        
        # Heuristic search
        for i, c in enumerate(cols):
            if "Ref" in c:
                ref_idx = i
            if "Enc2" in c and "ActP" in c:
                act_idx = i
        
        # Fallback if explicit names not found (e.g. LabScope weirdness)
        # If previous Ref was at 4, maybe the header provided in image is different?
        # User image 0 header: Name(0), RefTraj(1), Enc1_ActP(2), Enc2_ActP(3)...
        
        time_col = df.columns[0]
        ref_col = df.columns[ref_idx]
        act_col = df.columns[act_idx]
        
        data = pd.DataFrame()
        data['Time'] = pd.to_numeric(df[time_col], errors='coerce')
        data['Ref'] = pd.to_numeric(df[ref_col], errors='coerce')
        data['Act'] = pd.to_numeric(df[act_col], errors='coerce')
        
        data = data.dropna()
        data['Time'] = data['Time'] / 1000.0  # ms -> s
        
        # Shift time to start at 0
        if len(data) > 0:
            data['Time'] = data['Time'] - data['Time'].iloc[0]
            
        return data
    except Exception as e:
        print(f"FAILED to load {path.resolve()}")
        print(f"Error details: {e}")
        return None

# --- Metrics Calculation Functions ---

def calculate_step_metrics(t_exp, y_exp, t_sim, y_sim, duration):
    """
    Calculate Rise Time (10-90%) and Overshoot.
    Using python-control step_info logic or custom.
    Bobsy uses stepinfo() on sliced data.
    """
    # Slice data to the relevant duration for calculation if needed, 
    # but stepinfo usually handles the whole signal. 
    # However, Bobsy sliced it. Let's slice for consistency.
    
    # Experimental
    mask_exp = (t_exp >= 0) & (t_exp <= duration)
    # Ensure we have data
    if not np.any(mask_exp):
        tr_exp, os_exp = np.nan, np.nan
    else:
        # Shift time to start at 0 for the slice
        t_e = t_exp[mask_exp] - t_exp[mask_exp].iloc[0]
        y_e = y_exp[mask_exp]
        
        # We need the final value to be 1.0 approximately.
        # If steady state isn't reached, stepinfo might fail.
        # Assuming normalized step input of 1mm.
        try:
            # Settle value usually last value or 1.0
            inf_val = 1.0
            # Rise time: time to go from 10% to 90% of final value
            # Overshoot: (max - final) / final * 100
            
            # Simple custom implementation to avoid library dependency nuances
            final_val = y_e.iloc[-1] 
            # Or assume 1.0 if we trust the reference
            final_val = 1.0 
            
            max_val = np.max(y_e)
            os_exp = (max_val - final_val) / final_val * 100 if final_val != 0 else 0
            
            # Rise time
            tgt_10 = 0.1 * final_val
            tgt_90 = 0.9 * final_val
            
            t_10_idx = np.where(y_e >= tgt_10)[0]
            t_90_idx = np.where(y_e >= tgt_90)[0]
            
            if len(t_10_idx) > 0 and len(t_90_idx) > 0:
                 tr_exp = t_e.iloc[t_90_idx[0]] - t_e.iloc[t_10_idx[0]]
            else:
                 tr_exp = np.nan
        except:
             tr_exp, os_exp = np.nan, np.nan

    # Simulation
    mask_sim = (t_sim >= 0) & (t_sim <= duration)
    if not np.any(mask_sim):
        tr_sim, os_sim = np.nan, np.nan
    else:
        t_s = t_sim[mask_sim]
        y_s = y_sim[mask_sim]
        try:
            final_val = 1.0
            max_val = np.max(y_s)
            os_sim = (max_val - final_val) / final_val * 100
            
            t_10_idx = np.where(y_s >= 0.1 * final_val)[0]
            t_90_idx = np.where(y_s >= 0.9 * final_val)[0]
            
            if len(t_10_idx) > 0 and len(t_90_idx) > 0:
                 tr_sim = t_s[t_90_idx[0]] - t_s[t_10_idx[0]]
            else:
                 tr_sim = np.nan
        except:
            tr_sim, os_sim = np.nan, np.nan
            
    return tr_exp, os_exp, tr_sim, os_sim

def calculate_ramp_metrics(t_exp, y_exp, t_sim, y_sim, slope=10): # Report says 10mm/s
    """
    Calculate Steady State Error for Ramp.
    SS Error = mean(abs(Response - Reference)) over [1s, 2s].
    Reference = slope * t
    """
    # Time window [1, 2]
    t_start, t_end = 1.0, 2.0
    
    # Experimental
    err_exp = np.nan
    if t_exp is not None and len(t_exp) > 0:
        mask_exp = (t_exp >= t_start) & (t_exp <= t_end)
        if np.any(mask_exp):
            t_e = t_exp[mask_exp]
            y_e = y_exp[mask_exp]
            ref_e = slope * t_e
            err_exp = np.mean(np.abs(y_e - ref_e))

    # Simulation
    err_sim = np.nan
    if t_sim is not None and len(t_sim) > 0:
        mask_sim = (t_sim >= t_start) & (t_sim <= t_end)
        if np.any(mask_sim):
            t_s = t_sim[mask_sim]
            y_s = y_sim[mask_sim]
            ref_s = slope * t_s
            err_sim = np.mean(np.abs(y_s - ref_s))
        
    return err_exp, err_sim

def simulate_system(controller_name, input_type, t_vec):
    gs, gz = get_plant()
    cp, cll, clli = get_controllers(gz)

    sys_cl = None
    if controller_name == 'P':
        sys_cl = control.feedback(cp * gz, 1)
    elif controller_name == 'LeadLag':
        sys_cl = control.feedback(cll * gz, 1)
    elif controller_name == 'LeadLagIntegrator':
        sys_cl = control.feedback(clli * gz, 1)
    else:
        raise ValueError(f"Unknown controller name: {controller_name}")

    r_vec = None
    if input_type == 'step':
        r_vec = np.ones_like(t_vec) * 1.0
    elif input_type == 'ramp':
        r_vec = 10.0 * t_vec # Assuming 10 mm/s ramp
    else:
        raise ValueError(f"Unknown input type: {input_type}")

    t_out, y_out = control.forced_response(sys_cl, T=t_vec, U=r_vec)
    return t_out, y_out


def main():
    # Load data
    data_dir = Path("lab2_ryan/data_ryan")
    
    # 1. Simulate
    # Define T_final long enough for Ramp
    T_final = 2.0 
    dt = 0.0002
    T = np.arange(0, T_final + dt, dt)
    
    sys_cl_P_step = simulate_system('P', 'step', T)
    sys_cl_LL_step = simulate_system('LeadLag', 'step', T)
    sys_cl_LLI_step = simulate_system('LeadLagIntegrator', 'step', T)
    
    sys_cl_P_ramp = simulate_system('P', 'ramp', T)
    sys_cl_LL_ramp = simulate_system('LeadLag', 'ramp', T)
    sys_cl_LLI_ramp = simulate_system('LeadLagIntegrator', 'ramp', T)

    # Load Experiments
    # Filenames
    files = {
        'P_Step': 'proportional_step.csv',
        'P_Ramp': 'proportional_ramp.csv',
        'LL_Step': 'LeadLag_step.csv',
        'LL_Ramp': 'LeadLag_ramp.csv',
        'LLI_Step': 'LeadLagIntegrator_step.csv',
        'LLI_Ramp': 'LeadLagIntegrator_ramp.csv'
    }
    
    exp_data = {}
    for key, fname in files.items():
        exp_data[key] = load_data(data_dir / fname)

    # --- Plotting & Metric Config ---
    # Follow Bobsy's Limits:
    # Kp: Step [0, 1], Ramp [0, 2]
    # LL: Step [0, 0.5], Ramp [0, 0.5]
    # LLI: Step [0, 0.25], Ramp [0, 0.25]
    
    configs = [
        {
            'name': 'P Controller',
            'step_key': 'P_Step', 'ramp_key': 'P_Ramp',
            'sim_step': sys_cl_P_step, 'sim_ramp': sys_cl_P_ramp,
            'step_xlim': 1.0, 'ramp_xlim': 2.0,
            'step_met_dur': 1.0, 'ramp_met_slope': 10 # Experiment was 10mm/s
        },
        {
            'name': 'Lead-Lag',
            'step_key': 'LL_Step', 'ramp_key': 'LL_Ramp',
            'sim_step': sys_cl_LL_step, 'sim_ramp': sys_cl_LL_ramp,
            'step_xlim': 0.5, 'ramp_xlim': 0.5,
            'step_met_dur': 0.5, 'ramp_met_slope': 10
        },
        {
            'name': 'Lead-Lag-Integrator',
            'step_key': 'LLI_Step', 'ramp_key': 'LLI_Ramp',
            'sim_step': sys_cl_LLI_step, 'sim_ramp': sys_cl_LLI_ramp,
            'step_xlim': 0.25, 'ramp_xlim': 0.25,
            'step_met_dur': 0.25, 'ramp_met_slope': 10
        }
    ]
    
    metrics_list = []

    # Create Figures folder
    output_dir = Path("report2/figures")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Plot separately to match Bobsy's style? 
    # Bobsy had Figure 33, 34, 35. 
    # Let's create `compare_P.png`, `compare_LL.png`, `compare_LLI.png`.
    
    for cfg in configs:
        fig, axes = plt.subplots(2, 1, figsize=(8, 10))
        
        # Step
        ax1 = axes[0]
        # Exp
        df_step = exp_data[cfg['step_key']]
        tr_e, os_e, tr_s, os_s = np.nan, np.nan, np.nan, np.nan # Initialize
        if df_step is not None:
            ax1.plot(df_step['Time'], df_step['Act'], 'r', label='Experimental Results')
            # Calculate metrics
            tr_e, os_e, tr_s, os_s = calculate_step_metrics(
                df_step['Time'], df_step['Act'], 
                cfg['sim_step'][0], cfg['sim_step'][1], 
                cfg['step_met_dur']
            )
        else:
            # If no experimental data, still calculate sim metrics
            _, _, tr_s, os_s = calculate_step_metrics(
                pd.Series([]), pd.Series([]), # Empty series for exp
                cfg['sim_step'][0], cfg['sim_step'][1], 
                cfg['step_met_dur']
            )
            
        # Sim
        ax1.plot(cfg['sim_step'][0], cfg['sim_step'][1], 'b', label='Simulink Results')
        
        # Style
        ax1.set_xlim([0, cfg['step_xlim']])
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Step Response (mm)')
        ax1.set_title(f"{cfg['name']} Step Response of Experimental vs. Simulated Data")
        ax1.legend()
        ax1.grid(True)
        
        # Ramp
        ax2 = axes[1]
        # Exp
        df_ramp = exp_data[cfg['ramp_key']]
        err_e, err_s = np.nan, np.nan # Initialize
        if df_ramp is not None:
            ax2.plot(df_ramp['Time'], df_ramp['Act'], 'r', label='Experimental Results')
            # Calculate metrics
            err_e, err_s = calculate_ramp_metrics(
                df_ramp['Time'], df_ramp['Act'], 
                cfg['sim_ramp'][0], cfg['sim_ramp'][1], 
                cfg['ramp_met_slope']
            )
        else:
            # If no experimental data, still calculate sim metrics
            _, err_s = calculate_ramp_metrics(
                pd.Series([]), pd.Series([]), # Empty series for exp
                cfg['sim_ramp'][0], cfg['sim_ramp'][1], 
                cfg['ramp_met_slope']
            )
            
        # Sim
        ax2.plot(cfg['sim_ramp'][0], cfg['sim_ramp'][1], 'b', label='Simulink Results')
        
        # Style
        ax2.set_xlim([0, cfg['ramp_xlim']])
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Ramp Response (mm)')
        ax2.set_title(f"{cfg['name']} Ramp Response of Experimental vs. Simulated Data")
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        safe_name = cfg['name'].replace(' ', '_').replace('-', '')
        plt.savefig(output_dir / f"compare_{safe_name}.png", dpi=300)
        plt.close()
        
        # Store metrics
        metrics_list.append({
            'Controller': cfg['name'],
            'Tr_Exp': tr_e, 'Tr_Sim': tr_s,
            'OS_Exp': os_e, 'OS_Sim': os_s,
            'Ess_Exp': err_e, 'Ess_Sim': err_s
        })

    # Save metrics
    res_df = pd.DataFrame(metrics_list)
    res_df.to_csv("report2/metrics.csv", index=False)
    print("Metrics saved to report2/metrics.csv")
    print(res_df)
    
if __name__ == "__main__":
    main()
