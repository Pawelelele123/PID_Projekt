import serial
import matplotlib.pyplot as plt
from time import sleep

# Włącz tryb interaktywny dla matplotlib
plt.ion()

# Inicjalizacja portu szeregowego
hSerial = serial.Serial('COM3', 115200, timeout=1, parity=serial.PARITY_NONE)

# Wysyłanie początkowych komend do urządzenia
hSerial.write(b'PWM1=90;')

# Czyszczenie bufora wejściowego i wyjściowego
hSerial.reset_input_buffer()
hSerial.flush()

# Inicjalizacja list do przechowywania danych
temperature_samples = []
t = []
t_value = 0

# Ustawienie wykresu
plt.figure()
plt.title('Odpowiedź skokowa')
plt.xlabel('czas [s]')
plt.ylabel('temperatura [°C]')
line, = plt.plot(t, temperature_samples, 'r-')
plt.xlim(0, 2000)
plt.ylim(20, 85)  # Ustawienie zakresu temperatury, można dostosować
plt.grid()

# przygotowanie pliku
open('Pomiary_modelowanie_obiektu.txt', 'w').close()
f = open('Pomiary_modelowanie_obiektu.txt', 'a')

# Główna pętla programu
while True:
    # Odczytanie linii z portu szeregowego
    text = hSerial.readline().decode('utf-8').strip()
    # Przetwarzanie danych
    try:
        # Zakładając, że dane to temperatura w formacie "temp=XX"
        if text.startswith('temp='):
            temperature = float(text.split('=')[1])
            print(temperature)
            temperature_samples.append(temperature)
            t.append(t_value)
            t_value += 1

            # Ograniczenie liczby próbek do 1000, aby nie zaśmiecać pamięci
            if len(temperature_samples) > 1000:
                temperature_samples.pop(0)
                t.pop(0)

            # Aktualizacja wykresu
            line.set_xdata(t)
            line.set_ydata(temperature_samples)
            plt.pause(0.1)  # Pauza, aby wykres mógł się zaktualizować

            # Aktualizacja pliku
            f.write(str(temperature)+" ")
        else:
            raise Exception(text)

    except Exception as e:
        print(f'Błąd przetwarzanych danych: {e}')

    # Odczekanie chwilę przed kolejnym pomiarem
    sleep(0.1)
