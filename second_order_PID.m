%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written By: Mohammad Y. Saadeh, on 02/15/2012  %
% University of Nevada Las Vegas {UNLV}          %
% Modified By: Andy Zelenak, 2/12/2016            %
% The University of Texas at Austin              %
%                                                %%%%%%%%%%%%%%%%%%%%%%%
% PID controller and simulation of second-order system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear;close all
tic                 % start timer to calculate CPU time

desired = 1;        % desired output, or reference point

Kp = 120.6;             % proportional term Kp. Ku for ZN method is 201
Ki = 385.92;           % Integral term Ki
Kd = 9.42;           % derivative term Kd

if (sign(Kp)~=sign(Ki) | sign(Ki)~=sign(Kd))
    warning('All three gains should have the same sign for stability.');
end

dt = 0.01;          % sampling time
Time = 2;          % total simulation time in seconds
n = round(Time/dt); % number of samples

% pre-assign all the arrays to optimize simulation time
Prop(1:n+1) = 0; Der(1:n+1) = 0; Int(1:n+1) = 0; I(1:n+1) = 0;
u(1:n+1) = 0;
Output(1:n+1) = 0;
Error(1:n+1) = 0;

% Initial conditions
x1(1:n+1) = 0;
x2(1:n+1) = 0;
x2_dot(1:n+1) = 0;

for i = 1:n
    Error(i+1) = desired - x1(i); % error entering the PID controller
    
    Prop(i+1) = Error(i+1);% error of proportional term
    Der(i+1)  = (Error(i+1) - Error(i))/dt; % derivative of the error
    Int(i+1)  = Error(i+1)*dt; % integration of the error
    I(i+1)    = sum(Int); % the sum of the integration of the error
    
    u(i+1)  = Kp*Prop(i) + Ki*I(i+1)+ Kd*Der(i); % the three PID terms
    
    %% You can replace the following lines with your system/hardware/model
    x2_dot(i+1) = -x2(i)-x1(i)+u(i);
    
    % Integrate the x_dots to calculate the present state
    x2(i+1) = x2(i)+x2_dot(i+1)*dt;
    x1(i+1) = x1(i)+x2(i+1)*dt;
end

tsim = toc % simulation time

% plot results
T = 0:dt:Time;
Reference = desired*ones(1,i+1);
plot(T,Reference,'r.',T,x1,'bo','MarkerSize',2)
xlabel('Time (sec)')
legend('Desired','Simulated')

