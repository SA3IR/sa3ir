
% ------------------
% Context Parameters
% ------------------

int: 	ctx_battery;
int: 	ctx_waitingTimeCM_A;
int: 	ctx_waitingTimeCM_B;
float: 	ctx_distanceCM_A;
float: 	ctx_distanceCM_B;
float:	ctx_maxAllowedVelocity;

% --------------------
% Auxiliary Parameters
% --------------------

float: priority_efficiency = (exp(-5.0/15.0) - exp(-1.0 * int2float(ctx_battery) / 15.0)) / (exp(-5.0/15.0) - exp(-100.0/15.0));
float: priority_power = (exp(-1.0 * int2float(ctx_battery) / 15.0) - exp(-100.0 / 15.0)) / (exp(-5.0 / 15.0) - exp(-100.0 / 15.0));
int: timeCM_A = ctx_waitingTimeCM_A + floor(ctx_distanceCM_A/ctx_maxAllowedVelocity);
int: timeCM_B = ctx_waitingTimeCM_B + floor(ctx_distanceCM_B/ctx_maxAllowedVelocity);

% ------------------
% Variantion points
% ------------------
var 1..2: coffeemachine;

% ------------------
% Constraints
% ------------------

constraint priority_efficiency < priority_power /\ ctx_distanceCM_A < ctx_distanceCM_B -> coffeemachine = 1;
constraint priority_efficiency < priority_power /\ ctx_distanceCM_A >= ctx_distanceCM_B -> coffeemachine = 2;
constraint priority_efficiency >= priority_power /\ timeCM_A < timeCM_B -> coffeemachine = 1;
constraint priority_efficiency >= priority_power /\ timeCM_A >= timeCM_B -> coffeemachine = 2;

% ------------------
% Solver mode
% ------------------
solve satisfy;

% ------------------
% Output
% ------------------
output ["Selected Coffee Machine (A=1, B=2) = ", show(coffeemachine), "\n"];


