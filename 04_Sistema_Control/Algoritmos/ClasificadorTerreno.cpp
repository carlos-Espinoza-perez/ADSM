/*
 * =========================================================================
 * UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
 * Implementación de ClasificadorTerreno.cpp
 * =========================================================================
 */

#include "ClasificadorTerreno.h"

ClasificadorTerreno::ClasificadorTerreno() {
    _modoActivo = MODO_HISTERESIS;
    _clasePrevia = TERRENO_PLANO;
}

void ClasificadorTerreno::inicializar() {
    _clasePrevia = TERRENO_PLANO;
}

float ClasificadorTerreno::claseAAngulo(ClaseTerreno c) {
    switch(c) {
        case TERRENO_DESCENSO_ALTO: return -10.0f;
        case TERRENO_DESCENSO_BAJO: return  -5.0f;
        case TERRENO_PLANO:         return   0.0f;
        case TERRENO_ASCENSO_BAJO:  return  +5.0f;
        case TERRENO_ASCENSO_ALTO:  return +10.0f;
        default:                    return   0.0f;
    }
}

const char* ClasificadorTerreno::claseANombre(ClaseTerreno c) {
    switch(c) {
        case TERRENO_DESCENSO_ALTO: return "Descenso Alto (-10 deg)";
        case TERRENO_DESCENSO_BAJO: return "Descenso Bajo (-5 deg)";
        case TERRENO_PLANO:         return "Plano (0 deg)";
        case TERRENO_ASCENSO_BAJO:  return "Ascenso Bajo (+5 deg)";
        case TERRENO_ASCENSO_ALTO:  return "Ascenso Alto (+10 deg)";
        default:                    return "Desconocido";
    }
}

float ClasificadorTerreno::pertenenciaTrapezoidal(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

float ClasificadorTerreno::pertenenciaTriangular(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

ResultadoClasificacion ClasificadorTerreno::clasificar(float pitch_tibial, bool sw_talon, bool sw_punta) {
    // Fase de apoyo activo: detectado si cualquiera de los dos switches hace contacto
    bool en_apoyo = (sw_talon || sw_punta);

    if (_modoActivo == MODO_HISTERESIS) {
        return clasificarPorHisteresis(pitch_tibial, en_apoyo);
    } else {
        return clasificarPorLogicaDifusa(pitch_tibial, en_apoyo);
    }
}

// -------------------------------------------------------------------------
// ESTRATEGIA A: Umbrales con Histéresis
// -------------------------------------------------------------------------
ResultadoClasificacion ClasificadorTerreno::clasificarPorHisteresis(float pitch_tibial, bool en_apoyo) {
    uint32_t t_inicio = micros();
    ClaseTerreno nueva_clase = _clasePrevia;

    // Si no está en apoyo o contacto, preserva la última clase válida
    if (!en_apoyo) {
        ResultadoClasificacion res;
        res.clase = _clasePrevia;
        res.angulo_objetivo_deg = claseAAngulo(_clasePrevia);
        res.pertenencia_max = 1.0f;
        res.tiempo_computo_us = micros() - t_inicio;
        return res;
    }

    const float H = BANDA_HISTERESIS; // 1.5 deg

    // Lógica determinista de conmutación con bandas de guarda
    switch (_clasePrevia) {
        case TERRENO_PLANO:
            if (pitch_tibial > (2.5f + H))       nueva_clase = TERRENO_ASCENSO_BAJO;
            else if (pitch_tibial < (-2.5f - H))  nueva_clase = TERRENO_DESCENSO_BAJO;
            break;

        case TERRENO_ASCENSO_BAJO:
            if (pitch_tibial > (7.5f + H))       nueva_clase = TERRENO_ASCENSO_ALTO;
            else if (pitch_tibial < (2.5f - H))   nueva_clase = TERRENO_PLANO;
            break;

        case TERRENO_ASCENSO_ALTO:
            if (pitch_tibial < (7.5f - H))       nueva_clase = TERRENO_ASCENSO_BAJO;
            break;

        case TERRENO_DESCENSO_BAJO:
            if (pitch_tibial < (-7.5f - H))      nueva_clase = TERRENO_DESCENSO_ALTO;
            else if (pitch_tibial > (-2.5f + H))  nueva_clase = TERRENO_PLANO;
            break;

        case TERRENO_DESCENSO_ALTO:
            if (pitch_tibial > (-7.5f + H))      nueva_clase = TERRENO_DESCENSO_BAJO;
            break;
    }

    _clasePrevia = nueva_clase;

    ResultadoClasificacion res;
    res.clase = nueva_clase;
    res.angulo_objetivo_deg = claseAAngulo(nueva_clase);
    res.pertenencia_max = 1.0f;
    res.tiempo_computo_us = micros() - t_inicio;
    return res;
}

// -------------------------------------------------------------------------
// ESTRATEGIA B: Lógica Difusa (Fuzzy Logic - Sugeno orden 0)
// -------------------------------------------------------------------------
ResultadoClasificacion ClasificadorTerreno::clasificarPorLogicaDifusa(float pitch_tibial, bool en_apoyo) {
    uint32_t t_inicio = micros();

    if (!en_apoyo) {
        ResultadoClasificacion res;
        res.clase = _clasePrevia;
        res.angulo_objetivo_deg = claseAAngulo(_clasePrevia);
        res.pertenencia_max = 1.0f;
        res.tiempo_computo_us = micros() - t_inicio;
        return res;
    }

    // 1. Fuzzificación: cálculo de grados de pertenencia
    float mu[5];
    mu[0] = pertenenciaTrapezoidal(pitch_tibial, -30.0f, -20.0f, -7.5f, -5.0f); // Descenso Alto
    mu[1] = pertenenciaTriangular(pitch_tibial, -7.5f, -5.0f, -2.5f);            // Descenso Bajo
    mu[2] = pertenenciaTriangular(pitch_tibial, -2.5f,  0.0f,  2.5f);            // Plano
    mu[3] = pertenenciaTriangular(pitch_tibial,  2.5f,  5.0f,  7.5f);            // Ascenso Bajo
    mu[4] = pertenenciaTrapezoidal(pitch_tibial,  5.0f,  7.5f, 20.0f, 30.0f);    // Ascenso Alto

    // 2. Inferencia y Defuzzificación por Sugeno Singleton (promedio ponderado)
    float targets[5] = { -10.0f, -5.0f, 0.0f, 5.0f, 10.0f };
    float sum_ponderada = 0.0f;
    float sum_pesos = 0.0f;
    int idx_max = 2;
    float val_max = -1.0f;

    for (int i = 0; i < 5; i++) {
        sum_ponderada += mu[i] * targets[i];
        sum_pesos += mu[i];
        if (mu[i] > val_max) {
            val_max = mu[i];
            idx_max = i;
        }
    }

    float angulo_continuo = 0.0f;
    if (sum_pesos > 0.001f) {
        angulo_continuo = sum_ponderada / sum_pesos;
    } else {
        angulo_continuo = claseAAngulo(_clasePrevia);
        idx_max = (int)_clasePrevia;
        val_max = 1.0f;
    }

    // Cuantización a la clase discreta para la comparación metodológica formal
    ClaseTerreno clase_emitida = (ClaseTerreno)idx_max;
    _clasePrevia = clase_emitida;

    ResultadoClasificacion res;
    res.clase = clase_emitida;
    res.angulo_objetivo_deg = angulo_continuo; // Salida suave difusa
    res.pertenencia_max = val_max;
    res.tiempo_computo_us = micros() - t_inicio;
    return res;
}
