u0 = 10; % initial velocity parameter

%% try sigmoid function, but it doesn't take advantage of using a 
% max decceleration to slow down
syms x 
sigmoid = matlabFunction(exp(x)/(exp(x)+1));


tf = 6;
t  = linspace(0,2*tf);   % get time span
ud = u0-u0*sigmoid(t-6); % get ud
a  = diff(ud)./diff(t);% get acc for comfort



figure(1);clf;hold on;
plot(t, ud,t(1:end-1), a);

%% try constant deceleration
amax = 3;
tf = u0/amax;
t = linspace(0,tf);
ud = u0-amax.*t;

figure(1);clf;hold on;
plot(t,ud);