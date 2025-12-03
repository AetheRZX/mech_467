% MECH 467 Lab 3 - Trajectory Generation
% Gyan Edbert Zesiro (ID: 38600060)

function [traj] = traj_gen()
    % Parameters
    Ti = 0.0001; % 0.1 ms
    F = 200;     % mm/s
    A = 1000;    % mm/s^2
    D = 1000;    % mm/s^2

    %% Part A: Handout Trajectory
    P1 = [0, 0];
    P2 = [40, 30];
    P3 = [60, 30];
    P4 = [90, 30]; % Center of circle

    % Linear P1 -> P2
    [t1, x1, y1] = linear_interp(P1, P2, F, A, D, Ti);
    
    % Linear P2 -> P3
    [t2, x2, y2] = linear_interp(P2, P3, F, A, D, Ti);
    
    % Circular P3 -> P3 (Full Circle)
    [t3, x3, y3] = circular_interp(P3, P4, F, A, D, Ti, 1, 2*pi);
    
    % Combine
    traj.handout = combine_traj({t1, t2, t3}, {x1, x2, x3}, {y1, y2, y3}, Ti);
    
    %% Part C: Custom "G" Trajectory
    % Define points for "G"
    % Start top right: (80, 80)
    % Arc CCW to (80, 20) via (50, 50) center? No, let's do segments.
    
    % 1. Top Bar: (80, 80) to (40, 80)
    % 2. Left Arc: (40, 80) to (40, 20) -> Semi-circle centered at (40, 50)
    % 3. Bottom Bar: (40, 20) to (80, 20)
    % 4. Up Bar: (80, 20) to (80, 50)
    % 5. In Bar: (80, 50) to (60, 50)
    
    F_cust = 100;
    A_cust = 250;
    D_cust = 250;
    
    GP1 = [80, 80];
    GP2 = [40, 80];
    GP3 = [40, 20];
    GP4 = [80, 20];
    GP5 = [80, 50];
    GP6 = [60, 50];
    
    Center_Arc = [40, 50];
    
    [gt1, gx1, gy1] = linear_interp(GP1, GP2, F_cust, A_cust, D_cust, Ti);
    [gt2, gx2, gy2] = circular_interp(GP2, Center_Arc, F_cust, A_cust, D_cust, Ti, 1, pi); % CCW 180 deg
    [gt3, gx3, gy3] = linear_interp(GP3, GP4, F_cust, A_cust, D_cust, Ti);
    [gt4, gx4, gy4] = linear_interp(GP4, GP5, F_cust, A_cust, D_cust, Ti);
    [gt5, gx5, gy5] = linear_interp(GP5, GP6, F_cust, A_cust, D_cust, Ti);
    
    traj.custom = combine_traj({gt1, gt2, gt3, gt4, gt5}, ...
                               {gx1, gx2, gx3, gx4, gx5}, ...
                               {gy1, gy2, gy3, gy4, gy5}, Ti);
                           
    % Save Handout Traj
    traj_handout = traj.handout;
    save('HandoutTraj.mat', 'traj_handout');
    
    % Save Custom Traj for Lab
    traj_custom = traj.custom;
    % Rename to 'traj' as expected by machine
    traj = traj_custom; 
    save('GyanTraj.mat', 'traj');
end

function [t, x, y] = linear_interp(P_start, P_end, F, A, D, Ti)
    dist = norm(P_end - P_start);
    [t, s] = generate_s_profile(dist, F, A, D, Ti);
    
    u = (P_end - P_start) / dist;
    x = P_start(1) + s * u(1);
    y = P_start(2) + s * u(2);
end

function [t, x, y] = circular_interp(P_start, Center, F, A, D, Ti, dir, angle)
    R = norm(P_start - Center);
    arc_len = R * angle;
    [t, s] = generate_s_profile(arc_len, F, A, D, Ti);
    
    theta_start = atan2(P_start(2)-Center(2), P_start(1)-Center(1));
    theta = theta_start + dir * (s / R);
    
    x = Center(1) + R * cos(theta);
    y = Center(2) + R * sin(theta);
end

function [t, s] = generate_s_profile(S_total, F, A, D, Ti)
    Ta = F/A;
    Td = F/D;
    Sa = 0.5 * F * Ta;
    Sd = 0.5 * F * Td;
    
    if Sa + Sd > S_total
        Vp = sqrt(2 * S_total / (1/A + 1/D));
        Ta = Vp/A;
        Td = Vp/D;
        Tc = 0;
    else
        Sc = S_total - Sa - Sd;
        Tc = Sc / F;
    end
    
    t = 0:Ti:(Ta+Tc+Td);
    s = zeros(size(t));
    
    for k = 1:length(t)
        time = t(k);
        if time <= Ta
            s(k) = 0.5 * A * time^2;
        elseif time <= Ta + Tc
            s(k) = Sa + F * (time - Ta);
        else
            td = time - Ta - Tc;
            s(k) = Sa + F * Tc + F * td - 0.5 * D * td^2;
        end
    end
end

function combined = combine_traj(t_cells, x_cells, y_cells, Ti)
    t_total = [];
    x_total = [];
    y_total = [];
    
    curr_t = 0;
    
    for i = 1:length(t_cells)
        t_seg = t_cells{i};
        if i > 1
            % Shift time
            t_seg = t_seg + curr_t + Ti;
        end
        
        t_total = [t_total, t_seg];
        x_total = [x_total, x_cells{i}];
        y_total = [y_total, y_cells{i}];
        
        curr_t = t_total(end);
    end
    
    combined.t = t_total;
    combined.x = x_total;
    combined.y = y_total;
end
