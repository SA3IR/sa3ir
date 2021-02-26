
% ------------------
% Context Parameters
% ------------------

int: 	ctx_battery;
int: 	ctx_ambientNoise;

% --------------------
% Auxiliary Parameters
% --------------------

float: priority_efficiency = (exp(-5.0/15.0) - exp(-1.0 * int2float(ctx_battery) / 15.0)) / (exp(-5.0/15.0) - exp(-100.0/15.0));
float: priority_power = (exp(-1.0 * int2float(ctx_battery) / 15.0) - exp(-100.0 / 15.0)) / (exp(-5.0 / 15.0) - exp(-100.0 / 15.0));


% ------------------
% Variantion points
% ------------------
var 100.0..600.0: maxVelocity;
var 0..100: speakerVolume;


% ------------------
% Constraints
% ------------------

constraint ctx_ambientNoise < 20 -> speakerVolume = 35;
constraint ctx_ambientNoise >= 20 /\ ctx_ambientNoise < 70 -> speakerVolume = 55;
constraint ctx_ambientNoise >= 70 -> speakerVolume = 85;

var 0.0..100.0: aux;
constraint maxVelocity <= 100.0 			-> aux = 1.768/100.0 * maxVelocity;
constraint maxVelocity > 100.0 /\ maxVelocity <= 200.0 	-> aux = 3.444/100.0 * maxVelocity - 1.676;
constraint maxVelocity > 200.0 /\ maxVelocity <= 300.0 	-> aux = 6.708/100.0 * maxVelocity - 8.204;
constraint maxVelocity > 300.0 /\ maxVelocity <= 400.0 	-> aux = 13.07/100.0 * maxVelocity - 27.29;
constraint maxVelocity > 400.0 /\ maxVelocity <= 500.0 	-> aux = 25.44/100.0 * maxVelocity - 76.77;
constraint maxVelocity > 500.0 /\ maxVelocity <= 600.0 	-> aux = 49.57/100.0 * maxVelocity - 197.42;


% ------------------
% Solver mode
% ------------------
solve minimize priority_efficiency * ( -1.0 * 100.0 * (maxVelocity - 100.0) / (600.0 - 100.0) ) + priority_power * aux;

% ------------------
% Output
% ------------------
output ["Speaker Volume = ", show(speakerVolume), "\n", "Max Velocity = ", show(maxVelocity), "\n"];


