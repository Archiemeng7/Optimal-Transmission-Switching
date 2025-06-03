% DC Optimal Power Flow (DCOPF) calculation in MATLAB using YALMIP and Gurobi

% Clear workspace
clc;
clear all;

% Define YALMIP variables
P1 = sdpvar(1, 1, 'full'); % Generator 1 power
P2 = sdpvar(1, 1, 'full'); % Generator 2 power
P3 = sdpvar(1, 1, 'full'); % Generator 3 power
t1 = 0;                    % Reference bus angle (fixed at 0)
t2 = sdpvar(1, 1, 'full'); % Bus 2 angle
t3 = sdpvar(1, 1, 'full'); % Bus 3 angle

% Line susceptances
b12 = 1;
b13 = 1;
b23 = 1;

% Loads
PL1 = 80;
PL2 = 0;
PL3 = 100;

% Objective function: Minimize total generation cost
Objective = 10*P1 + 20*P2 + 100*P3;

% Constraints
Constraints = [];

% Generator limits (same order as CVXPY)
Constraints = [Constraints, P1 >= 0];       % Constraint 1
Constraints = [Constraints, P2 >= 0];       % Constraint 2
Constraints = [Constraints, P3 >= 0];       % Constraint 3
Constraints = [Constraints, P1 <= 1200];    % Constraint 4
Constraints = [Constraints, P2 <= 1200];    % Constraint 5

% Power balance constraints
Constraints = [Constraints, P1 - PL1 == (t1 - t2)*b12 + (t1 - t3)*b13]; % Constraint 6 (Bus 1)
Constraints = [Constraints, P2 - PL2 == (t2 - t1)*b12 + (t2 - t3)*b23]; % Constraint 7 (Bus 2)
Constraints = [Constraints, P3 - PL3 == (t3 - t1)*b13 + (t3 - t2)*b23]; % Constraint 8 (Bus 3)

% Line flow limits
Constraints = [Constraints, (t1 - t3) <= 10]; % Constraint 9
Constraints = [Constraints, (t3 - t1) <= 10]; % Constraint 10

% Solver settings for Gurobi
options = sdpsettings('solver', 'gurobi', 'verbose', 0);

% Solve the problem
diagnostics = optimize(Constraints, Objective, options);

% Check solver status
if diagnostics.problem == 0
    fprintf('Problem status: optimal\n');
else
    fprintf('Problem status: %s\n', diagnostics.info);
end

% Extract variable values
P1_val = value(P1);
P2_val = value(P2);
P3_val = value(P3);
t2_val = value(t2);
t3_val = value(t3);

% Extract LMPs (dual values of power balance constraints)
LMP1 = -dual(Constraints(6)); % Power balance at bus 1
LMP2 = -dual(Constraints(7)); % Power balance at bus 2
LMP3 = -dual(Constraints(8)); % Power balance at bus 3
congestion1 = -dual(Constraints(9)); % Line Power at line 1-3
congestion2 = -dual(Constraints(10)); % Line Power at line 3-1

% Calculate power flows for all lines
P12 = b12 * (t1 - t2_val); % Flow from Bus 1 to Bus 2
P13 = b13 * (t1 - t3_val); % Flow from Bus 1 to Bus 3
P23 = b23 * (t2_val - t3_val); % Flow from Bus 2 to Bus 3

% Display results
fprintf('\nLoad at bus 1: %d MWh. LMP: $ %.1f . Pays: $ %.1f\n', PL1, round(LMP1, 2), round(PL1*LMP1, 2));
fprintf('Load at bus 2: %d MWh. LMP: $ %.1f . Pays: $ %.1f\n', PL2, round(LMP2, 2), round(PL2*LMP2, 2));
fprintf('Load at bus 3: %d MWh. LMP: $ %.1f . Pays: $ %.1f\n', PL3, round(LMP3, 2), round(PL3*LMP3, 2));

% Total system operating cost
fprintf('\nTotal system operating cost : $ %.1f\n', round(value(Objective), 3));

% Generator outputs and payments
fprintf('\nPg1:  %.1f  MWh, gets paid: $ %.1f\n', round(abs(P1_val), 2), round(P1_val*LMP1, 2));
fprintf('Pg2:  %.1f  MWh, gets paid: $ %.1f\n', round(abs(P2_val), 2), round(P2_val*LMP2, 2));
fprintf('Pg3:  %.1f  MWh, gets paid: $ %.1f\n', round(abs(P3_val), 2), round(P3_val*LMP3, 2));

% Power flow distribution results
fprintf('\nPower flow distribution:\n');
fprintf('Line 1-2: %.1f MW\n', round(P12, 2));
fprintf('Line 1-3: %.1f MW\n', round(P13, 2));
fprintf('Line 2-3: %.1f MW\n', round(P23, 2));

% Check if supply meets demand
total_load = PL1 + PL2 + PL3;
total_gen = P1_val + P2_val + P3_val;
fprintf('\n supply meets demand? :  %d load and  %.1f  gen\n', total_load, total_gen);