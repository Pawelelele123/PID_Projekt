temperature = load('Pomiary_modelowanie_obiektu_użyte_do_modelowania.txt');
dt=1; %one second
number_of_samples = length (temperature);
t = (0:number_of_samples-1)*dt;

%Input signal
input_amplitude = 0.90; %0.09 -> 9% of PWM duty (PWM1=90;)
input = input_amplitude*ones (1 ,number_of_samples);

%LTI model (linear time-invariant model)
s = tf('s');
k = 6.5/input_amplitude;	%model gain
T = 290;	%model time constant
delay = 15;	%model delay
H = k/(1 + s*T) *exp(-s*delay);	%model
disp(sprintf('Model parameters k=%.2g, T=%g, delay=%g\n', k, T, delay));

%Model response
model_response = lsim (H, input, t);
model_response = model_response + 22.4; %add offset

%Model error
residuum = temperature - model_response';
error_abs_sum = sum(abs(residuum));
disp(sprintf('Model error sum(abs(residuum)) = %g\n', error_abs_sum));

figure(1);
hold on;
plot(t, temperature', '.r', 'MarkerSize', 10, 'DisplayName', 'próbki pomiarowe');
plot(t, model_response, '.b', 'MarkerSize', 10, 'DisplayName', 'odpowiedź modelu grzałki + LM35');
title('Odpowiedź skokowa');
xlabel('czas [s]');
ylabel('temperatura [°C]');
legend('Location', 'Best');
axis tight;
hold off;

figure(2);
plot(t, residuum', '.m', 'MarkerSize', 10);
title('Residuum (próbki pomiarowe - odpowiedź modelu)');
xlabel('czas [s]');
ylabel('temperatura [°C]');
axis tight;
