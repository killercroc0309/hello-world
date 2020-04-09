clear all;
close all;

NumSteps = 1000;

Cd = 0.5;
A = 2;
v_wind=20;
m=900;
theta = -pi/6;
gamma_friction=100;
g = 9.81;

Fmax = 20000; %max motor force
Fmin = -20000; %min motor force

v_target = 120/3.6;

tfinal = 50;
t=linspace(0,tfinal, NumSteps);
tStep = tfinal/NumSteps; %timestep size

%preallocate variables for speed:
v = t;
a = t;
x = t;
error = t;
error_integral = t;
error_derivative = t;
Fmotor = t*0;

ProportionalGain = -1000;
IntegralGain = -1.5;
DerivativeGain = -10000;
DerivativeControlThreshold = 50; %velocity error to be within before applying derivative control; this is an example of "fuzzy logic"

v(1) = 0; %start at the cruise control velocity
a(1) = (Fmotor(1)-0.5*Cd*A*(v(1)-v_wind)*abs(v(1)-v_wind)-m*g*sin(theta)-gamma_friction*v(1))/m;
error(1) = v(1)-v_target;
tDelay = 2; %Number of seconds the response is delayed from the signal

iDelay=tDelay/tStep;  %number of steps that the force is behind due to time delays in the engine

for i=1:(NumSteps-1)
    theta=0;%sin(t(i)/20);
    a(i+1) = (Fmotor(i)-0.5*Cd*A*(v(i)-v_wind)*abs(v(i)-v_wind)-m*g*sin(theta)-gamma_friction*v(i))/m; %compute next acceleration value
    v(i+1) = v(i) + a(i+1)*tStep; %compute next velocity from the integral
    x(i+1) = x(i) + v(i+1)*tStep; %compute next position value
    error(i+1) = v(i+1)-v_target;
    error_integral(i+1) = error_integral(i)+error(i+1);
    error_derivative(i+1) = (error(i+1)-error(i));
    if i>iDelay
        Fmotor(i+1) = ProportionalGain*error(i-iDelay) + IntegralGain*error_integral(i-iDelay) + (abs(error(i-iDelay))<DerivativeControlThreshold)*DerivativeGain*error_derivative(i-iDelay);
        if Fmotor(i+1) > Fmax
            Fmotor(i+1) =Fmax;
        elseif Fmotor(i+1) < Fmin
            Fmotor(i+1) = Fmin;
        end
    end
end

%Plot results
subplot(2,2,1);
plot(t, v)
xlabel('time')
ylabel('velocity')

subplot(2,2,2);
plot(t, a, 'r')
xlabel('time')
ylabel('acceleration')

subplot(2,2,3);
plot(t, error, 'r')
xlabel('time')
ylabel('velocity error')

subplot(2,2,4);
plot(t, Fmotor, 'g')
xlabel('time')
ylabel('Output Motor Force')
