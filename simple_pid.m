function u = simple_pid(error)

%Declare static value
persistent previous_error;
if isempty (previous_error)
    previous_error = 0;
end

persistent previous_integral;
if isempty (previous_integral)
    previous_integral = 0;
end

%Constant values
dt = 1;
Kp = 1.54872967509146;
Ki = 0.0184338237431332;
Kd = 0;
N = 100;

%proportional part
P = Kp*error;

%integral part
integral = previous_integral + (error + previous_error); %numerical integrator without anti-windup
previous_integral = integral;
I = Ki*integral*(dt/2);

%derivative part
derivative = (error - previous_error)/dt; %numerical derivative without filter
previous_error = error;
D = Kd*derivative;


%sum of all parts
u = P + I + D; %without saturation
