#include "ControladorPosicion.h"

ControladorPosicion::ControladorPosicion() {
    _kp = 4.0f;
    _deadband = 65.0f;   // PWM mínimo medido para vencer fricción en rosca M8
    _err_tol = 0.20f;    // Banda de tolerancia para evitar chattering por holgura
}

void ControladorPosicion::inicializar(float kp, float deadband_pwm, float err_tol_deg) {
    _kp = kp;
    _deadband = deadband_pwm;
    _err_tol = err_tol_deg;
}

int16_t ControladorPosicion::calcular(float theta_ref, float theta_real) {
    float error = theta_ref - theta_real;

    // Zona de descanso
    if (fabs(error) <= _err_tol) {
        return 0;
    }

    // Ganancia proporcional escalada a PWM (255 = 12V)
    // Kp en V/deg -> PWM/deg = Kp * (255 / 12) = Kp * 21.25
    float gain_pwm = _kp * 21.25f;
    float u_p = gain_pwm * fabs(error);

    // Compensación directa de fricción de husillo
    float u_total = _deadband + u_p;

    // Saturación a 8 bits
    if (u_total > 255.0f) u_total = 255.0f;

    // Aplicar dirección
    if (error > 0) {
        return (int16_t)u_total;
    } else {
        return -(int16_t)u_total;
    }
}
