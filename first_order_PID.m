%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written By: Mohammad Y. Saadeh, on 02/15/2012  %
% University of Nevada Las Vegas {UNLV}          %
% Modified By: Andy Zelenak, 2/12/2016            %
% The University of Texas at Austin              %
%                                                %%%%%%%%%%%%%%%%%%%%%%%
% PID controller and simulation of first-order system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear;close all
tic                 % start timer to calculate CPU time

setpoint = 1;        % desired output, or reference point


Kp = -100.0;        % proportional term Kp
Ki = -1;           % Integral term Ki
Kd = 0;           % derivative term Kd

if (sign(Kp)~=sign(Ki) | sign(Ki)~=sign(Kd))
    warning('All three gains should have the same sign for stability.');
end

dt = 0.001;          % sampling time
Time = 1;          % total simulation time in seconds
n = round(Time/dt); % number of samples

% pre-assign all the arrays to optimize simulation time
Prop(1:n+1) = 0; Der(1:n+1) = 0; Int(1:n+1) = 0; I(1:n+1) = 0;
u(1:n+1) = 0;
Output(1:n+1) = 0;
Error(1:n+1) = 0;

% Initial conditions
x1(1:n+1) = 0;
x1_dot(1:n+1) = 0;

for i = 1:n
    Error(i+1) = setpoint - x1(i); % error entering the PID controller
    
    Prop(i+1) = Error(i+1);% error of proportional term
    Der(i+1)  = (Error(i+1) - Error(i))/dt; % derivative of the error
    Int(i+1)  = Error(i+1)*dt; % integration of the error
    I(i+1)    = sum(Int); % the sum of the integration of the error
    
    u(i+1)  = Kp*Prop(i) + Ki*I(i+1)+ Kd*Der(i); % the three PID terms
    
    %% You can replace the following lines with your system/hardware/model
    x1_dot(i+1) = x1(i)- u(i);
    
    % Integrate the x_dots to calculate the present state
    x1(i+1) = x1(i)+x1_dot(i+1)*dt;
end

tsim = toc % simulation time

% plot results
T = 0:dt:Time;
Reference = setpoint*ones(1,i+1);
plot(T,Reference,'r.',T,x1,'bo','MarkerSize',2)
xlabel('Time [unitless]')
ylabel('State/setpoint [unitless]')
legend('Setpoint','x_1')
set(gcf,'color','w')

