% DC Optimal Power Flow (DCOPF) in MATLAB using YALMIP and Gurobi
% --- Adds export of bus voltages (angles) and line currents ---

clc; clear all;

%-----------------------------
% Decision variables (YALMIP)
%-----------------------------
P1 = sdpvar(1,1,'full'); % Generator 1 power (MW)
P2 = sdpvar(1,1,'full'); % Generator 2 power (MW)
P3 = sdpvar(1,1,'full'); % Generator 3 power (MW)

t1 = 0;                   % Ref bus angle (rad)
t2 = sdpvar(1,1,'full');  % Bus 2 angle (rad)
t3 = sdpvar(1,1,'full');  % Bus 3 angle (rad)

%-----------------------------
% Network data
%-----------------------------
% Line susceptances (p.u.) -- DC model uses b_ij
b12 = 1; b13 = 1; b23 = 1;

% Three-phase line-to-line voltages for current conversion (kV)
% (set these to your case; same on all three lines here)
V12_kV = 230; V13_kV = 230; V23_kV = 230;
pf = 1.0;  % assume cos(phi) ~ 1 (DC approx / near-unity PF)

% Loads (MW)
PL1 = 80; PL2 = 0; PL3 = 100;

%-----------------------------
% Objective: generation cost
%-----------------------------
Objective = 10*P1 + 20*P2 + 100*P3;

%-----------------------------
% Constraints
%-----------------------------
Constraints = [];

% Generator limits
Constraints = [Constraints, P1 >= 0,  P1 <= 1200];    % G1
Constraints = [Constraints, P2 >= 0,  P2 <= 1200];    % G2
Constraints = [Constraints, P3 >= 0];                 % G3 (if you have an upper bound, add it)

% Nodal power balance (MW) under DC model
Constraints = [Constraints, P1 - PL1 == (t1 - t2)*b12 + (t1 - t3)*b13]; % Bus 1
Constraints = [Constraints, P2 - PL2 == (t2 - t1)*b12 + (t2 - t3)*b23]; % Bus 2
Constraints = [Constraints, P3 - PL3 == (t3 - t1)*b13 + (t3 - t2)*b23]; % Bus 3

% Line flow (angle-difference) limits: here only on line 1-3 as in original
Constraints = [Constraints, (t1 - t3) <= 10];
Constraints = [Constraints, (t3 - t1) <= 10];

%-----------------------------
% Solve (Gurobi)
%-----------------------------
options = sdpsettings('solver','gurobi','verbose',0);
diagnostics = optimize(Constraints, Objective, options);

if diagnostics.problem == 0
    fprintf('Problem status: optimal\n');
else
    fprintf('Problem status: %s\n', diagnostics.info);
end

%-----------------------------
% Values
%-----------------------------
P1_val = value(P1);
P2_val = value(P2);
P3_val = value(P3);
t2_val = value(t2);
t3_val = value(t3);

% LMPs (duals of nodal balances; sign per YALMIP)
LMP1 = -dual(Constraints(6));
LMP2 = -dual(Constraints(7));
LMP3 = -dual(Constraints(8));

% Congestion duals for the two angle-diff limits on line 1-3
cong13_pos = -dual(Constraints(9));  % (t1 - t3) <= 10
cong31_pos = -dual(Constraints(10)); % (t3 - t1) <= 10

%-----------------------------
% Power flows (MW) on lines
%-----------------------------
P12 = b12*(t1      - t2_val); % 1->2
P13 = b13*(t1      - t3_val); % 1->3
P23 = b23*(t2_val  - t3_val); % 2->3

%-----------------------------
% Bus voltages (angles) export
%-----------------------------
busID   = [1; 2; 3];
Vmag_pu = [1; 1; 1];                     % DC model: |V| ~ 1 p.u.
Vang_rad= [0; t2_val; t3_val];
Vang_deg= rad2deg(Vang_rad);

bus_table = table(busID, Vmag_pu, Vang_rad, Vang_deg, ...
    'VariableNames', {'Bus','Vmag_pu','Vang_rad','Vang_deg'});

%-----------------------------
% Line currents (p.u. and kA/A)
%-----------------------------
% In DC approx: I_pu = b_ij*(theta_i - theta_j)
I12_pu = b12*(t1      - t2_val);
I13_pu = b13*(t1      - t3_val);
I23_pu = b23*(t2_val  - t3_val);

% Convert MW -> kA via P = sqrt(3) * V_LL(kV) * I(kA) * pf   (pf≈1)
I12_kA = abs(P12) / (sqrt(3)*V12_kV*pf);
I13_kA = abs(P13) / (sqrt(3)*V13_kV*pf);
I23_kA = abs(P23) / (sqrt(3)*V23_kV*pf);

I12_A = 1e3 * I12_kA;
I13_A = 1e3 * I13_kA;
I23_A = 1e3 * I23_kA;

line_from = [1; 1; 2];
line_to   = [2; 3; 3];
P_MW      = [P12; P13; P23];
I_pu      = [I12_pu; I13_pu; I23_pu];
I_kA      = [I12_kA; I13_kA; I23_kA];
I_A       = [I12_A;  I13_A;  I23_A];
V_kV      = [V12_kV; V13_kV; V23_kV];

line_table = table(line_from, line_to, V_kV, P_MW, I_pu, I_kA, I_A, ...
    'VariableNames', {'From','To','V_LL_kV','P_MW','I_pu','I_kA','I_A'});

%-----------------------------
% Print summary
%-----------------------------
fprintf('\nLoad at bus 1: %d MW. LMP: $ %.2f . Pays: $ %.2f\n', PL1, LMP1, PL1*LMP1);
fprintf('Load at bus 2: %d MW. LMP: $ %.2f . Pays: $ %.2f\n', PL2, LMP2, PL2*LMP2);
fprintf('Load at bus 3: %d MW. LMP: $ %.2f . Pays: $ %.2f\n', PL3, LMP3, PL3*LMP3);

fprintf('\nTotal system operating cost : $ %.2f\n', value(Objective));

fprintf('\nPg1: %.2f MW, gets paid: $ %.2f\n', P1_val, P1_val*LMP1);
fprintf('Pg2: %.2f MW, gets paid: $ %.2f\n', P2_val, P2_val*LMP2);
fprintf('Pg3: %.2f MW, gets paid: $ %.2f\n', P3_val, P3_val*LMP3);

fprintf('\nPower flows (MW):\n');
fprintf('Line 1-2: %.2f MW\n', P12);
fprintf('Line 1-3: %.2f MW\n', P13);
fprintf('Line 2-3: %.2f MW\n', P23);

fprintf('\nBus voltage angles (deg):  [Bus1 %.2f] [Bus2 %.2f] [Bus3 %.2f]\n', ...
    0, Vang_deg(2), Vang_deg(3));

fprintf('\nLine currents (p.u.):  I12=%.4f  I13=%.4f  I23=%.4f\n', I12_pu, I13_pu, I23_pu);
fprintf('Line currents (kA):    I12=%.4f  I13=%.4f  I23=%.4f\n', I12_kA, I13_kA, I23_kA);
fprintf('Line currents (A):     I12=%.1f   I13=%.1f   I23=%.1f\n', I12_A, I13_A, I23_A);

% Supply-demand check
total_load = PL1 + PL2 + PL3;
total_gen  = P1_val + P2_val + P3_val;
fprintf('\nSupply meets demand?  load = %d MW, gen = %.2f MW\n', total_load, total_gen);

%-----------------------------
% Export to CSV
%-----------------------------
writetable(bus_table,  'bus_voltage.csv');
writetable(line_table, 'line_flows_currents.csv');
fprintf('\nCSV exported: bus_voltage.csv , line_flows_currents.csv\n');
