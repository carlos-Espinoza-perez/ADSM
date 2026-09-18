/*
 * =========================================================================
 * UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
 * CONTROLADOR DE POSICIÓN EN TIEMPO DISCRETO CON COMPENSACIÓN DE ZONA MUERTA
 * =========================================================================
 */

#ifndef CONTROLADOR_POSICION_H
#define CONTROLADOR_POSICION_H

#include <Arduino.h>

class ControladorPosicion {
public:
    ControladorPosicion();
    void inicializar(float kp, float deadband_pwm, float err_tol_deg);
    
    // Calcula la señal de control en PWM (-255 a +255)
    int16_t calcular(float theta_ref, float theta_real);

    void setKp(float kp) { _kp = kp; }
    void setDeadband(float deadband) { _deadband = deadband; }

private:
    float _kp;
    float _deadband;
    float _err_tol;
};

#endif
