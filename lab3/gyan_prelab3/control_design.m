% MECH 467 Lab 3 - Controller Design & Simulation
% Gyan Edbert Zesiro (ID: 38600060)

function control_design()
    % Plant Parameters
    % X Axis
    Ka_x = 1; Kt_x = 0.49; Ke_x = 1.59; Je_x = 4.36e-4; B_x = 0.0094;
    % Y Axis
    Ka_y = 1; Kt_y = 0.49; Ke_y = 1.59; Je_y = 3e-4; B_y = 0.0091;

    s = tf('s');
    Gx = (Ka_x * Kt_x * Ke_x) / (s * (Je_x * s + B_x));
    Gy = (Ka_y * Kt_y * Ke_y) / (s * (Je_y * s + B_y));

    Ts = 0.0001;

    % Cases
    cases = struct('name', {'LBW', 'HBW', 'Mismatch'}, ...
                   'X_wc', {20*2*pi, 40*2*pi, 40*2*pi}, ...
                   'Y_wc', {20*2*pi, 40*2*pi, 20*2*pi});

    for i = 1:3
        c = cases(i);
        
        % Design Controllers
        Cx = design_lead_lag(Gx, c.X_wc, 60);
        Cy = design_lead_lag(Gy, c.Y_wc, 60);
        
        % Discretize
        Cx_d = c2d(Cx, Ts, 'tustin');
        Cy_d = c2d(Cy, Ts, 'tustin');
        Gx_d = c2d(Gx, Ts, 'zoh');
        Gy_d = c2d(Gy, Ts, 'zoh');
        
        % Closed Loop
        sys_cl_x = feedback(Cx_d * Gx_d, 1);
        sys_cl_y = feedback(Cy_d * Gy_d, 1);
        
        % Simulate Handout Trajectory
        load('traj_data.mat', 'traj');
        t = traj.handout.t;
        ref_x = traj.handout.x;
        ref_y = traj.handout.y;
        
        [y_x, ~] = lsim(sys_cl_x, ref_x, t);
        [y_y, ~] = lsim(sys_cl_y, ref_y, t);
        
        % Plotting (Placeholder for MATLAB execution)
        figure;
        plot(ref_x, ref_y, 'k--', y_x, y_y, 'r-');
        title(['Contouring Performance - ' c.name]);
        legend('Reference', 'Simulated');
        
        % Custom Trajectory Simulation (Only for LBW usually)
        if strcmp(c.name, 'LBW')
            t_cust = traj.custom.t;
            ref_x_cust = traj.custom.x;
            ref_y_cust = traj.custom.y;
            
            [y_x_cust, ~] = lsim(sys_cl_x, ref_x_cust, t_cust);
            [y_y_cust, ~] = lsim(sys_cl_y, ref_y_cust, t_cust);
            
            figure;
            plot(ref_x_cust, ref_y_cust, 'k--', y_x_cust, y_y_cust, 'b-');
            title('Custom Trajectory Contouring (LBW)');
        end
    end
end

function C_tot = design_lead_lag(G, wc, PM_target)
    [mag, phase] = bode(G, wc);
    phi_req = -180 + PM_target - phase;
    
    alpha = (1 + sind(phi_req))/(1 - sind(phi_req));
    tau = 1 / (wc * sqrt(alpha));
    K = 1 / (mag * sqrt(alpha));
    
    s = tf('s');
    C_lead = K * (alpha * tau * s + 1) / (tau * s + 1);
    
    Ki = wc / 10;
    C_int = (s + Ki) / s;
    
    C_tot = C_lead * C_int;
end
