%% Control system for computed torque:

% Transfer functions:
G_open = tf([0 0 16],[1 8 0])
G_closed = feedback(G_open, 1)

% Closed loop unit step response:
%figure(1)
%step(G_closed)

% Open loop frequency response:
figure(2)
margin(G_open)

% Finding K gain:
syms val
eq = 20 * log10(val) == 15
K = double(solve(eq, val))

% Open loop frequency response with added gain:
figure(3)
margin(K*G_open)