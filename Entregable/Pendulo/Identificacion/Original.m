
clear all;
close all;
clc;


% Load data for system identification
load('dataPendulo.mat'); % This should contain input-output data as well as a time vector
shortF = true;
sampleTime = 0.01; % Sample time in seconds
if shortF == false
    U = U./max(U);
    Y = Y*(pi/180);
    data = iddata(Y, U, sampleTime, ...
        'Name', 'Robot Motion Data', ...
        'InputName', 'PWM', 'InputUnit', 'V', ...
        'OutputName', 'Pitch', 'OutputUnit', 'rad', ...
        'Tstart', 0, 'TimeUnit', 's');
else
    Ushort = [U(1:25)];
    Yshort = [Y(1:25)];
    % Ushort = [U(1:25);U(58:74)];
    % Yshort = [Y(1:25);Y(58:74)];
    Ushort = Ushort./max(Ushort);
    Yshort = Yshort*(pi/180);
    data = iddata(Yshort, Ushort, sampleTime, ...
        'Name', 'Robot Motion Data', ...
        'InputName', 'PWM', 'InputUnit', 'V', ...
        'OutputName', 'Pitch', 'OutputUnit', 'rad', ...
        'Tstart', 0, 'TimeUnit', 's');
end


% Time vector and input data
t = linspace(0, sampleTime*length(Yshort), length(Yshort))';
u = Ushort;  % Example input (adjust according to your actual data)
y_real = Yshort;  % Example output with noise

% Initial parameter guesses [L, m, K_theta, K_x, b]
initial_params = [0.143, 0.55, 0.1, 0.05, 0.01];  % Example initial guesses

% Optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Run optimization
[opt_params, fval] = fmincon(@(params) objective_function(params, t, y_real, u), initial_params, [], [], [], [], [0.1, 0.5, 0, 0, 0], [1, 3, 1, 1, 0.1], [], options);

% Display results
disp('Optimized Parameters:');
disp(opt_params);
disp('Maximized R² Score:');
disp(-fval);

%% State space estimated

g = 9.81;
L = opt_params(1);
m = opt_params(2);
K_theta = opt_params(3);
K_x = opt_params(4);
b = opt_params(5);

% theta = x(1);
% dtheta = x(2);
% xpos = x(3);
% dxpos = x(4);
A = [0 1 0 0;(g/L) -(K_theta/(m*L^2)) 0 0;...
    0 0 0 1; 0 0 0 -(b/m)];
B = [0;(K_theta/(m*L^2));0; (K_x/m)];
C = [1 0 0 0;0 0 1 0];
D = [0;0];

estSys = ss(A,B,C,D);
figure,step(estSys);


[yEst,tOut,x] = lsim(estSys,Ushort,t);

figure,
plot(t,Yshort);
hold on
plot(t,yEst(:,1));


disp(rank(ctrb(A,B)))
disp(rank(obsv(A,C)))


K = place(A,B,[-3+0.5i;-3-0.5i;-5;-6]);
contEstSys = ss(A-B*K,B,C,D);
figure,step(contEstSys);