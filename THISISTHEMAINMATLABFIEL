%Kode lånt fra forelesning 5/2 og modifisert.
clear;
close all;
syms s % Define the variable

A = [0, 1; 0, -10.05];
B = [0; 239.25];
C = [1, 0]; % Assuming you want to output both states, otherwise define as needed
D = [0]; % Here when D is 0, there is no direct feedback

transpose([0; 0; 1])
%My state and b matrices is 2x1 and my therefore my feedback gain matrix must -
%have 1 row
%% This is for finding a polynomial with our desired attributes and it's poles

% Given POS and Ts values
POS = 5; % Percentage Overshoot
Ts = 2; % Settling Time

% Solving for zeta using the overshoot formula
% This requires a numerical solution, using fzero for example
zeta_fun = @(zeta) 100*exp(-zeta*pi/sqrt(1-zeta^2)) - POS;
zeta_guess = 0.5; % Initial guess for zeta
zeta = fzero(zeta_fun, zeta_guess);

% Solving for omega_n
omega_n = 4 / (zeta * Ts);

% Display calculated zeta and omegan
fprintf('Damping Ratio (zeta): %.4f\n', zeta);
fprintf('Natural Frequency (omegan): %.4f rad/s\n', omega_n);

% Construct the system's denominator polynomial
% For a second-order system: deng = [1 2*zeta*omegan omegan^2]
dennominator = [1, 2*zeta*omega_n, omega_n^2];
poles = roots(dennominator)
%% Finding state feedback vector based on the poles we just found in section above
K = place(A,B,poles);
%For the state feedback system we have made so far, when completing this -
%section is Css = 1 -T(0) when T(s) = C(sI-A)^(-1)B+D
%For removing this steady state error we add integral control
%A good rule of thumb is place it 5 times as far away from the imaginary axis
%as the two others(could of cource be more)



%%
size(A)
size(B)
size(K_IntegralControl)

K_IntegralControl(1:2)
K_IntegralControl(2)
K_IntegralControl(3)
%% This section is for finding integral control
%Integral control adds an other pole to the system
%We want the pole of the integrator to be far away so that -
%the last pole does not effect the characteristics off the system -
% and the poles, mirrored abaout the real axis, would be the dominant ones

new_pole = real(poles(1))*5;  % Multiplying the real part of the first pole by 5. 
% This operation assumes that both poles share the same real value, which is a common scenario.
% It's crucial to verify this assumption in practice
% System's stability and response characteristics are heavily influenced by the real parts of its poles.

% Append the new pole to the existing poles array
new_poles = [poles; new_pole];

% Generate a our new desired polynomial with integral control
new_denominator = poly(new_poles);


%To include integral action, i have to expand the original system matrices to accommodate the integral state.
%I wil use my augmented system matrices A_ic and B_ic

A_ic = [A, zeros(size(A, 1), 1); -C, 0];
B_ic = [B; 0];
C_ic = [C, 0]; % Assuming the output does not directly include the integral state.


%A_ic = [A-(B*K_IntegralControl(1:2)) (B*K_IntegralControl(3));
 %       -C 0]

K_IntegralControl = place(A_ic, B_ic, new_poles);
K = K_IntegralControl(1:2)
Ki = K_IntegralControl(3);
%K_IntegralControl = place(A, B, new_poles);
%We will get a k matrix in this form: [k_1 K_2 K_i]

%New state matrix with integral control (again this is only in scenarios of
%3x3


%size(A)
%Now we would have a transfer function, with integrator control, of:
%T(s) = C(sI-Ai)^(-1)B

%from here we have an integral state variable feedback system


%% Code for finding the system observer gain
%
%With the integral control we will get this new 
%Our A_new matrix is going to be:
%A_new = [A-(B*K_IntegralControl(1:2)) (B*K_IntegralControl(3));
  %      -[C, 0] 0]

% Verify the result of this operation is a column vector with the correct dimensions
%A = transpose(A)
%B =[1250; 0]
%C_observ =[0; 0.1]
%tempColumn = B * K_IntegralControl(3);
%disp(size(tempColumn));


if size(A, 1) == size(A, 1) 
    
    if (size(A) == size(A))
        
        
    
    
    
        pos=20 % This is my desired overshoot(%OS)
        Ts=2 %Here i set my desired setteling time
        z=(-log(pos/100))/(sqrt(pi^2 + log(pos/100)^2));
        wn=4/(z*Ts);
        r=roots([1,2*z*wn, wn^2]);
          
        r
        takenPoles=[r']
        observerPoles = 7*real(poles) %+ imag(poles)  
        L = acker(A',C',observerPoles)
        L(1)
        L(2) 
        size(L) %L is 1x2
    else
        error('Dimension mismatch. size(tempColumn, 1) != size(A, 1)');

    end
else
    error('Dimension mismatch. size(tempColumn, 1) != size(A, 1)');
end



%% Run this section and the ones above to test the program in simulink
% If you not are going to run the simulink program you can skip this
A = [0, 1; 0, -10];
B = [0; 1250];
C = [0.1, 0]; 
D = [0];

%% Step-response section
% Source1: https://se.mathworks.com/matlabcentral/answers/1940104-state-feedback-control-and-observer
% source2: https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace

% Combine state-feedback controller and observer
%We sould might add expression for integral control
Aco  = [A-B*K B*K;
        zeros(size(A)) A-L*C];
Bco  = [B*Nbar;
       zeros(size(B))];
Cco  = [C zeros(size(C))];
Dco  = 0;
% Closed-loop observer-based control system
clco = ss(Aco, Bco, Cco, Dco)

step(clco, 6), grid on

stepinfo(clco)


%We are using this to see if there is any steady state error
sse = fix(1 - dcgain(clco)) 

%% 
%checking if the observer actually is copying our design
%This is the observer with this closed loop system
%We are using the separation principple
% possible soure: https://se.mathworks.com/matlabcentral/answers/1728095-designing-an-observer-control-system
%https://ceid.utsa.edu/ataha/wp-content/uploads/sites/38/2017/10/Observers_Intro2-1.pdf
%Our real system
realSys = ss(A,B,C,D)
%USE THE SECTION UNDER



%Our observer estimator system
Ao = A-(L'*C)
Bo = [B L']
Co = eye(size(A),'like',A)
Do =[0]

% [start : step : finalTime]
timeV=0:0.1:50
sysInput= 10*ones(size(timeV))


% [start : step : finalTime]
timeV=0:0.1:10
sysInput= 10*ones(size(timeV))

%initial state matrix
x0Initial = [1;1]

%simulating real system
[yTrue,timeTime,xTrue] = lsim(realSys,sysInput,timeV,x0Initial)
%yTrue is the simulated system output

figure(1)
hold on
plot(timeTime,xTrue(:,1)) % plotting the first state
plot(timeTime,xTrue(:,2)) % plotting the secound state

%Defining our observer system
sysObs = ss(Ao, Bo, Co, Do)
size(yTrue)
%We need to transpose our output yTrue
inputObsrv=[sysInput; yTrue']
%The initial state of the observer:
x0ObsInitial=[0,0]
[obsOutput,timeObserver,xObserver]=lsim(sysObs,inputObsrv,timeV,x0ObsInitial)

%Compearing the state of the observer to the state of our real sysmtem
figure(2)
hold on
plot(timeV,xTrue(:,2),'r')
plot(timeV,xObserver(:,2),'b')
legend('Real System State 2', 'Observer State 2', 'Location', 'best')
title('Comparison of Real System and Observer State 2')
xlabel('Time (seconds)')
ylabel('State 2 Value')
hold off % 

figure(3) % Creating a new figure
hold on
plot(timeV, xTrue(:,1), 'r') % Plotting State 1 of the real system in red
plot(timeV, xObserver(:,1), 'b') % Plotting State 1 of the observer in blue
legend('Real System State 1', 'Observer State 1', 'Location', 'best')
title('Comparison of Real System and Observer State 1')
xlabel('Time (seconds)')
ylabel('State 1 Value')
hold off % Release hold if no more plots will be added

%% This is for testing our closed loop estimation to check if the observer is actually is cop
% Define your system matrices, K and L here
%use a little transport delay
% Observer design
Ao = A - L'*C;
Bo = [B, L']; % Assuming direct feedthrough for simplicity
Co = eye(size(A)); % Observer output matrix
Do = zeros(size(Co, 1), size(Bo, 2)); % Observer direct transition matrix


sysObs = ss(Ao, Bo, Co, Do)


% Closed-loop system for simulation purposes
% Since the control action is based on the estimated state, you simulate
% the closed-loop dynamics with the observer integrated
Acl = [A-B*K          B*K;
       zeros(size(A)) A-L'*C];
Bcl = [B; zeros(size(B))];
Ccl = [C zeros(size(C))];
Dcl = D;

realSys = ss(A,B,C,D)

timeV=0:0.1:10
sysInput= ones(size(timeV))




% Check if there are at least 10 elements
%if length(sysInput) >= 10
    % Set elements from index 10 onwards to 1
%    sysInput(10:end) = 1;
%end


x0Initial = [1;1]

% Initial conditions for both real system and observer
initialConditions = [x0Initial; zeros(size(x0Initial))]; 

% Simulate the combined system
[yTrue,timeTime,xTrue] = lsim(ss(Acl, Bcl, Ccl, Dcl), sysInput, timeV, initialConditions);

x0ObsInitial=[1,1]
inputObsrv=[sysInput; yTrue']

% Extract the true and estimated states from the combined state vector
%xTrue = xCombined(:, 1:size(A,1));
%xEstimated = xCombined(:, size(A,1)+1:end);


[obsOutput,timeObserver,xObserver]=lsim(sysObs,inputObsrv,timeV,x0ObsInitial)


%Compearing the state of the observer to the state of our real sysmtem
figure(2)
hold on
plot(timeV,xTrue(:,2),'r')
plot(timeV,xObserver(:,2),'b')
legend('Real System State 2', 'Observer State 2', 'Location', 'best')
title('Comparison of Real System and Observer State 2')
xlabel('Time (seconds)')
ylabel('State 2 Value')
hold off % if you want to add more plots later, remove this line

% Step response comparison in the same figure but different subplot
%subplot(1,2,2); % Second subplot in a 1x2 grid
%step(sysObs);
%hold on;
%step(ss(Acl, Bcl, Ccl, Dcl));
%legend('Observer Step Response', 'Real System Step Response', 'Location', 'best');
%title('Step Response Comparison');
%xlabel('Time (seconds)');
%ylabel('Response');

figure; 

% Subplot 1: Display Step Input
subplot(1, 3, 1); % First subplot in a 1x3 grid
t = 0:0.01:5; % Define a time vector, adjust the range as needed
u = ones(size(t)); % Step input
plot(t, u, 'k--'); % Plotting step input as a dashed black line
ylim([0, 1.2]); % Adjust y-axis limits to clearly show the step
title('Step Input');
xlabel('Time (seconds)');
ylabel('Input Value');

% This is for step Response
subplot(1, 3, 2:3); 
step(sysObs, t(end));
hold on;
step(ss(Acl, Bcl, Ccl, Dcl), t(end));
legend('Observer Step Response', 'Real System Step Response', 'Location', 'best');
title('Step Response Comparison');
xlabel('Time (seconds)');
ylabel('Response');


figure(3) % Creating a new figure
hold on
plot(timeV, xTrue(:,1), 'r') % Plotting State 1 of the real system in red
plot(timeV, xObserver(:,1), 'b') % Plotting State 1 of the observer in blue
legend('Real System State 1', 'Observer State 1', 'Location', 'best')
title('Comparison of Real System and Observer State 1')
xlabel('Time (seconds)')
ylabel('State 1 Value')
hold off % Release hold if no more plots will be added

%%
sysInput
%% Simulate system with transport delay
% Defining the delay in secounds
delayTime = 0.000001;

% Obtain the numerator and denominator of the Pade approximation for the delay
[numDelay, denDelay] = pade(delayTime, 3); 

% Create a transfer function model of the delay
delaySystem = tf(numDelay, denDelay);

% Assuming sysObs is your observer system without delay
% Add delay to the observer system
sysObsDelayed = series(delaySystem, sysObs); % This adds the delay to the input of the observer system

% Now you can use sysObsDelayed in your simulations
size(sysObsDelayed)
% Assuming you have a real system defined as realSys and an input signal sysInput
[yTrue, timeTrue, xTrue] = lsim(realSys, sysInput, timeV, [-1,-1]); % Simulate real system

% For the observer with delay, you might need to adjust the input if it depends on the output of the real system
inputObsrv = [sysInput; yTrue']; % Adjust based on your system's specifics

timeV=0:0.1:10
inputObsrv= ones(size(timeV))


% Simulate the observer system with delay
[obsOutput, timeObserver, xObserver] = lsim(sysObsDelayed, inputObsrv, timeV, [-1,-1,-1,-1,-1,-1,-1,-1]);


figure;
subplot(2,1,1); % Plot real system state
plot(timeV, xTrue(:,1), 'r'); % Adjust index as per your state of interest
xline(9, '--k', 'x = 9');
title('Real System State 1 (Angular Position)');
xlabel('Time (seconds)');
ylabel('State Value');

subplot(2,1,2); % Plot observer state with delay
plot(timeV, xObserver(:,1), 'b'); % Adjust index as per your state of interest
xline(9, '--k', 'x = 9');
title('Observer State 1 (Angular Position)');
xlabel('Time (seconds)');
ylabel('State Value');

%% Compare the first state
figure;
subplot(2,1,1); % Plot real system state
plot(timeV, xTrue(:,2), 'k'); % Adjust index as per your state of interest
yline(24, '--k', 'y = 24');
xline(9, '--k', 'x = 9');
title('Real System State 2 (Angular Velocity)');
xlabel('Time (seconds)');
ylabel('State Value');

subplot(2,1,2); % Plot observer state with delay
plot(timeV, xObserver(:,2), 'g'); % Adjust index as per your state of interest
yline(24, '--k', 'y = 24');
xline(9, '--k', 'x = 9');
title('Observer State with Delay 2 (Angular Velocity)');
xlabel('Time (seconds)');
ylabel('State Value');


%% Simulating Sampled system, with zero order hold
Ts = 0.5; % Sampling time

% Discretize the real system
realSysD = c2d(realSys, Ts, 'zoh');

% Discretize the observer system
sysObsD = c2d(sysObs, Ts, 'zoh');

% If handling delay explicitly is necessary
% Assuming delaySystem is your delay modeled in continuous time
delaySystemD = c2d(delaySystem, Ts, 'zoh');
sysObsDelayedD = series(delaySystemD, sysObsD); % Now both in discrete-time

timeV = 0:0.1:10; % Time vector
inputObsrv = ones(length(timeV), size(sysObsDelayed.B, 2)); % Ensure correct size

% Simulate using discrete-time systems
%[yTrueD, timeTrueD, xTrueD] = lsim(realSysD, sysInput, timeV);
[obsOutputD, timeObserverD, xObserverD] = lsim(sysObsDelayedD, inputObsrv, timeV);

[obsOutput, timeObserver, xObserver] = lsim(sysObsDelayed, inputObsrv, timeV, [-1,-1,-1,-1,-1,-1,-1,-1]);

figure;
subplot(2,1,1); % Plot real system state
plot(timeV, xTruex(:,1), 'r'); % Adjust index as per your state of interest
xline(9, '--k', 'x = 9');
title('Real System State 1 (Angular Position)');
xlabel('Time (seconds)');
ylabel('State Value');

subplot(2,1,2); % Plot observer state with delay
plot(timeV, xObserver(:,1), 'b'); % Adjust index as per your state of interest
xline(9, '--k', 'x = 9');
title('Observer State 1 (Angular Position)');
xlabel('Time (seconds)');
ylabel('State Value');




%% This section is for testing the observer

% Define system matrices
A = A_new;
B = [B; 0];
C = [0 0.1 0];
D = 0;

% The observer gain matrix L, created in the section above 
L_obs = L;

% The initial state and the initial state estimate is being defined here
%[initial_rpm; initial_position; OSV];
%investigate later
x0 = [0 0 0];

%This is the estimate
%There is no prior knowladge
x_hat0 = [0 0 0]; 

% Define input signal u(t) as a function handle, e.g., a step function
u = @(t) 1; % Example: constant input of 1

% Simulation time span
tspan = [0, 10]; % From 0 to 10 seconds

% Simulate system
[t, x] = ode45(@(t, x) A*x + B*u(t), tspan, x0);

% Output simulation
y = C*x' + D*u(t); % Assuming D is zero for simplicity

% Simulate observer/estimator
[t, x_hat] = ode45(@(t, x_hat) A*x_hat + B*u(t) + L_obs*(y(t)' - C*x_hat), tspan, x_hat0);

% Plotting
figure;
plot(t, x, 'r', 'LineWidth', 2); hold on; % Actual state
plot(t, x_hat, 'b--', 'LineWidth', 2); % Estimated state
xlabel('Time (s)');
ylabel('State and Estimate');
legend('Actual State', 'Estimated State');
title('State vs. Estimated State');




%% Creating a transfeer function of the system we just made.

% Define controller and observer matrices
%K = [0.1 1]; % State feedback gain
%L = [2, 3]; % Observer gain

% Form the augmented system (which is the full-state intagral feedback and full-order observer)
A_new = [A-(B*K_IntegralControl(1:2)), tempColumn; -C 0]
%A_aug = [A-(B*K), B*K;
%         zeros(size(A)), (A-L*C)];
B_aug = [B; 0];
C_aug = [0 0.1 0];
mySizeA = size(A_new)
mySizeB = size(B_aug)
mySizeC = size(C_aug)
D = 0;

% Create the state-space model
sys = ss(A_new, B_aug, C_aug, D)

% Use ss2tf for converting the state - space model to a transfer function
[tfNum , tfDen] = ss2tf(A_new , B_aug , C_aug , D);
transferFunction = tf(tfNum, tfDen)


symTF = poly2sym(tfNum, s) / poly2sym(tfDen, s); 
pretty(symTF) 



%%



w_c = 400*pi; %Kantfrekvens i rad/s
f_c = w_c / (2*pi); %Kantfrekvens i Hz
f_s = 500; %Samplingsfrekvens i Hz
t_s = 1/f_s; %Sampletid

%The numerator of the analogue system we just created
num_analog = tfNum

%The denominator of the analogue system we just created
den_analog = tfDen
sys_c = tf(tfNum, tfDen); %analogue system transfer function

[num_digital, den_digital] = bilinear(num_analog, den_analog, f_s);
sys1 = tf(num_digital, den_digital, t_s); %Diskre system 

[num_digital2, den_digital2] = bilinear(num_analog, den_analog, f_s, f_c);
sys2 = tf(num_digital2, den_digital2, t_s); %Diskre system med forhåndsfordreining

plotoptions = bodeoptions;
plotoptions.Grid = 'on';
plotoptions.FreqScale = 'linear';
plotoptions.FreqUnits = 'Hz';
plotoptions.YLimMode = 'manual';
plotoptions.YLim = {[-60, 10], [-90,360]};
plotoptions.XLim = [0, f_s*0.66];
hold on;
bode(sys_c, plotoptions);
bode(sys1, plotoptions)
bode(sys2, plotoptions)
legend(["Kontinuerlig system","Tustins metode", "Tustins metode med prewarping"])


figure;
step(sys_c), hold on
step(sys2), hold off
grid on
legend('Continuous', 'Discrete', 'Location', 'Best')
title('Step Response Comparison')
ylabel('Amplitude')
xlabel('Time (s)')
