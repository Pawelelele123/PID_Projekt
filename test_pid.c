#include <stdio.h>
#include <stdint.h>

typedef float float32_t;

// Struktura do przechowywania parametrów regulatora
typedef struct
{
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;
    float32_t dt;
} pid_parameters_t;

typedef struct
{
    pid_parameters_t p;
    float32_t previous_error, previous_integral;
} pid_t;

/**
Calculation of discrete PID
* @param [in, out] s A pointer to PID parameters and history
* @param [in] setpoint Input setpoint value
* @param [in] measured Input measured value
* @return PID output value
**/
// Algorytm regulatora
float32_t calculate_discrete_pid(pid_t* pid, float32_t setpoint, float32_t measured) {
    float32_t u = 0.0f, P, I, D, error, integral, derivative;
    
    error = setpoint - measured;
    
    // proportional part
    P = pid->p.Kp * error;
    
    // integral part
    integral = pid->previous_integral + (error + pid->previous_error); // numerical integrator without anti-windup
    pid->previous_integral = integral;
    I = pid->p.Ki * integral * (pid->p.dt / 2.0f);
    
    // derivative part
    derivative = (error - pid->previous_error) / pid->p.dt; // numerical derivative without filter
    pid->previous_error = error;
    D = pid->p.Kd * derivative;
    
    // sum of all parts
    u = P + I + D; // without saturation
    
    return u;
}

//prosta i szybka weryfikacja implementacji algorytmu
int main() {
    uint16_t number_of_samples = 1000;
    float32_t dt = 0.01f, setpoint = 1.0f, measured = 0, pid_output;
    pid_t pid1 = { .p.Kp = 3.0f, .p.Ki = 0.01f, .p.Kd = 0.0f, .p.dt = dt, .previous_error = 0, .previous_integral = 0 };
    
    printf("Generated step response for display in Scilab or Matlab\r\n");
    // Generate response with the same sampling time
    printf("dt=%f; t=[0:%d-1]*dt; \r\n", dt, number_of_samples);
    printf("response=[");
    while (number_of_samples--)
	{
        setpoint = 1.0f; // like step response
        measured = 0.0f; // in step response measured is equal 0
        pid_output = calculate_discrete_pid(&pid1, setpoint, measured);
        printf("%f", pid_output);
        if (number_of_samples > 0)
		{
            printf(",");
        }
    }
    printf("];\r\n");
    printf("figure(1); plot(t, response, '.r', 'MarkerSize', 20); title('Odpowiedź skokowa'); xlabel('czas [s]'); ylabel('temperatura [°C]'); \r\n");
    printf("s=tf('s'); H_P=%f; H_I=%f*c2d(1/s, dt, 'tustin'); H_D=%f*c2d_euler(s ,dt, 'backward'); hold on; \r\n", pid1.p.Kp, pid1.p.Ki, pid1.p.Kd);
    printf("H = H_P + H_I + H_D; st=step(H, t); plot(t, st, 'b', 'LineWidth', 2); legend('odpowiedź cyfrowego PID', 'odpowiedź Matlaba'); grid on; grid minor; hold off; \r\n");
    
    return 0;
}
