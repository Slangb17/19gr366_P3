% Script for symbolic derivation of the dynamics for the CrustCrawler
clc; clear all; close all;

%% Define symbolic variables
% Generalized coordinates and velocity/acceleration of them
syms t real 
q1 = sym(str2sym('q1(t)'));
q2 = sym(str2sym('q2(t)'));
q3 = sym(str2sym('q3(t)'));
dq1 = diff(q1,t);
dq2 = diff(q2,t);
dq3 = diff(q3,t);
ddq1 = diff(dq1,t);
ddq2 = diff(dq2,t);
ddq3 = diff(dq3,t);

q_sym = {q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3};
q_char = {'q1','q2','q3','dq1','dq2','dq3','ddq1','ddq2','ddq3'};
 
% Mass of links in grams
m1 = 0.22501;
m2 = 0.21286;
m3 = 0.28598;
 
% Principal axes of inertia and principal moments of inertia: ( grams *
% square millimeters):

I1 = [0.08 1.00 0.00;
     -1.00 0.08 0.00;
     0.00 0.00 1.00] / 1e+9;
        
I2 = [0.01 0.01 1.00;
      1.00 0.00 -0.01;
      0.00 1.00 -0.01] / 1e+9;
           
I3 = [0.03 0.02 1.00;
      1.00 0.00 -0.03;
      0.00 1.00 -0.02] / 1e+9;

% Gravity
%gv = [0 -9.80665 0 0]';
syms g real
gv = [0 g 0 0]';
 
% Transformation matrices
T_01 = troty(q1)*transl(0, 0.250, 0);
    
T_12 = trotz(pi/2) * troty(q2)*transl(0, 0, 0.220);

T_23 = troty(q3)*transl(0, 0, 0.276);

T_02 = T_01*T_12;
T_02 = simplify(T_02,'Steps',50);
T_03 = T_01*T_12*T_23;
T_03 = simplify(T_03,'Steps',50);
 
% Rotation matrices
R_01 = T_01(1:3,1:3);
R_12 = T_12(1:3,1:3);
R_23 = T_23(1:3,1:3);
 
R_02 = R_01*R_12;
R_03 = R_01*R_12*R_23;
 
%% Step 3: Define Center of Mass Positions (in local link coordinates)
P_c11 = [0.00040 -0.02060 0.00 1].';
P_c22 = [0.00036 -0.00052 -0.03859 1].';
P_c33 = [-0.00195 -0.00156 -0.13656 1].';
 
% Transform center of mass point to base frame
P_c01 = T_01*P_c11;
P_c02 = T_02*P_c22;
P_c03 = T_03*P_c33;
 
%% Step 4: Take time derivatives of position with respect to base frame
v_c01 = diff(P_c01, t);
v_c02 = diff(P_c02, t);
v_c03 = diff(P_c03, t);
 
%% Step 5: Compute angular velocity of each link
omega_01 = R_01*[0; 0; dq1];
omega_02 = omega_01 + R_02*[0; 0; dq2];
omega_03 = omega_02 + R_03*[0; 0; dq3];
 
%% Step 6: Define inertia tensors for each link according to base frame
% I_global = A I? AT
II1 = R_01*I1*R_01.';
II2 = R_02*I2*R_02.';
II3 = R_03*I3*R_03.';
 
%% Step7: Translational and rotational kinetic energy of the links
T1 = (1/2)*m1*(v_c01.'*v_c01) + (1/2)*omega_01.'*II1*omega_01;
T2 = (1/2)*m2*(v_c02.'*v_c02) + (1/2)*omega_02.'*II2*omega_02;
T3 = (1/2)*m3*(v_c03.'*v_c03) + (1/2)*omega_03.'*II3*omega_03;
 
%% Step 8: Compute the potential energy
V1 = m1*gv.'*P_c01;
V2 = m2*gv.'*P_c02;
V3 = m3*gv.'*P_c03;
 
%% Computing actuation torques - derivatives of the Lagrangian

L = (T1 - V1) + (T2 - V2) + (T3 - V3);
L = subs(L, q_sym, q_char);

% Partial diffs of theta dots:
par_dq1 = diff(L, 'dq1');
par_dq2 = diff(L, 'dq2');
par_dq3 = diff(L, 'dq3');

% Partial diffs of theta:
par_q1 = diff(L, 'q1');
par_q2 = diff(L, 'q2');
par_q3 = diff(L, 'q3');

% Diffs of partial diffs of theta dots:
par_dq1 = subs(par_dq1, q_char, q_sym);
par_dq2 = subs(par_dq2, q_char, q_sym);
par_dq3 = subs(par_dq3, q_char, q_sym);

dt_par_dq1 = diff(par_dq1, t);
dt_par_dq2 = diff(par_dq2, t);
dt_par_dq3 = diff(par_dq3, t);

dt_par_dq1 = subs(dt_par_dq1, q_sym, q_char);
dt_par_dq2 = subs(dt_par_dq2, q_sym, q_char);
dt_par_dq3 = subs(dt_par_dq3, q_sym, q_char);

% Computing tau1, tau2 and tau3:
tau1 = dt_par_dq1 - par_q1;
tau2 = dt_par_dq2 - par_q2;
tau3 = dt_par_dq3 - par_q3;

tau1 = simplify(tau1, 'Steps', 50);
tau2 = simplify(tau2, 'Steps', 50);
tau3 = simplify(tau3, 'Steps', 50);

%% Compute Mass matrix
M_11 = subs(tau1,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 1, 0, 0, 0});
M_12 = subs(tau1,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 1, 0, 0});
M_13 = subs(tau1,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 0, 1, 0});
M_21 = subs(tau2,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 1, 0, 0, 0});
M_22 = subs(tau2,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 1, 0, 0});
M_23 = subs(tau2,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 0, 1, 0});
M_31 = subs(tau3,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 1, 0, 0, 0});
M_32 = subs(tau3,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 1, 0, 0});
M_33 = subs(tau3,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3','g'},{0, 0, 0, 0, 0, 1, 0});
 
M = [M_11   M_12    M_13;
     M_21   M_22    M_23;
     M_31   M_32    M_33];
M = simplify(M,'Steps',50)

%% Compute Gravety vector 
G_1 = subs(tau1,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3'},{0, 0, 0, 0, 0, 0});
G_2 = subs(tau2,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3'},{0, 0, 0, 0, 0, 0});
G_3 = subs(tau3,{'dq1', 'dq2', 'dq3', 'ddq1', 'ddq2', 'ddq3'},{0, 0, 0, 0, 0, 0});

G = [G_1;G_2;G_3];
G = simplify(G,'Steps',50)
 
%% Compute Coriolis vector
M_ddq = M * [ddq1 ddq2 ddq3].';
M_ddq_ = subs(M_ddq,{ddq1, ddq2, ddq3},{'ddq1', 'ddq2', 'ddq3'});
M_ddq_ = simplify(M_ddq_,'Steps',10);
 
V = [tau1;tau2;tau3] - M_ddq_ - G;
V = simplify(V,'Steps',50)