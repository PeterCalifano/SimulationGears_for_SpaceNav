clear
close all
clc

%% Predator-Prey Model
% Initial conditions
Initial_predator_population = 15;
Initial_Prey_population = 20;

% Model parameters
Prey_Birth_rate = 4; % r
Interaction_Prey_Predator = 5; % a
env_capacity = 100; % k
Prey_rate_control_parameter = 100; % c
Predator_growth_rate = 1.1; % b
Predator_death_rate = 0.8; % d

r = Prey_Birth_rate;
a = Interaction_Prey_Predator;
b = Predator_growth_rate;
c = Prey_rate_control_parameter;
d = Predator_death_rate;
k = env_capacity;

H_star = (Prey_rate_control_parameter*Predator_death_rate)/(Interaction_Prey_Predator*Predator_growth_rate - Predator_death_rate);
% L_Star =  (Prey_Birth_rate * H_star * (Prey_rate_control_parameter + H_star)/(H_star*Interaction_Prey_Predator))*(1- H_star/env_capacity);
% L_star = b*c*r*(a*b*k - c*d - d*k)./(k*(a*b - d).^2);
L_star = (r/a)*(1 - H_star/k)*(c + H_star);
out = sim('PredatorPreyDynamics.slx');

H = out.H;
L = out.L;

figure;
plot(H, L, 'g.-');
hold on;

plot(H(1), L(1), 'ro');
plot(H(end), L(end), 'ko');

plot([0,env_capacity, H_star], [0, 0, L_star], 'bo')

hold off
legend('Trajectory - State Space', 'Initial condition', 'Final condition', 'Eq. points')


