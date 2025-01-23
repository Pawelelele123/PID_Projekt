temperature = load('Pomiary_zadanie_temperatury_użyte_do_weryfikacji.txt');
dt=1; %one second
number_of_samples = length (temperature);
t = (0:number_of_samples-1)*dt;
set_point = 26;

figure(1);
hold on;
plot(t, temperature', '.r', 'MarkerSize', 10, 'DisplayName', 'próbki pomiarowe');
line([0 t(end)], [set_point, set_point], 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 1);
title('Odpowiedź skokowa');
xlabel('czas [s]');
ylabel('temperatura [°C]');
legend;
axis tight;
hold off;
