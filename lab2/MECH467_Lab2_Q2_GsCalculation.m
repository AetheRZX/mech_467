s=tf('s');
% Define symbolic variables
syms s Ka Kt Ke Je Be Ki

% Define the open-loop transfer function G_ol(s)
G_ol = (Ka * Kt * Ke) / (s * (Je * s + Be));

% Define the controller transfer function C(s)
C = (Ki + s) / s;

% Calculate the closed-loop transfer function G(s)
G = simplify(G_ol * C / (1 + G_ol * C));

% Display the result
disp('Closed-loop transfer function G(s):');
disp(G);