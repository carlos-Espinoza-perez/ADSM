/*
 * =========================================================================
 * UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
 * Facultad de Ingeniería · Carrera de Ingeniería Mecatrónica
 * Proyecto: Tobillo Protésico Activo Escalado
 * Autor: Carlos Espinoza · Revisor: MSc. Kevin Josué Flores Carvajal
 * 
 * MÓDULO DE CLASIFICACIÓN DE TERRENO (C++ para ESP32)
 * Implementa las dos estrategias comparadas de forma intercambiable:
 *   1. ESTRATEGIA A: Umbrales con Histéresis
 *   2. ESTRATEGIA B: Lógica Difusa (Mamdani / Sugeno orden 0)
 * =========================================================================
 */

#ifndef CLASIFICADOR_TERRENO_H
#define CLASIFICADOR_TERRENO_H

#include <Arduino.h>

// Clases discretas de terreno evaluadas en el protocolo
enum ClaseTerreno {
    TERRENO_DESCENSO_ALTO = 0, // -10 grados
    TERRENO_DESCENSO_BAJO = 1, //  -5 grados
    TERRENO_PLANO         = 2, //   0 grados
    TERRENO_ASCENSO_BAJO  = 3, //  +5 grados
    TERRENO_ASCENSO_ALTO  = 4  // +10 grados
};

// Modos de algoritmo seleccionables por comando o MQTT
enum ModoClasificador {
    MODO_HISTERESIS = 0,
    MODO_LICA_DIFUSA = 1
};

// Estructura de salida del clasificador
struct ResultadoClasificacion {
    ClaseTerreno clase;
    float angulo_objetivo_deg;
    float pertenencia_max;
    uint32_t tiempo_computo_us;
};

class ClasificadorTerreno {
public:
    ClasificadorTerreno();
    void inicializar();
    
    // Configuración de modo
    void setModo(ModoClasificador modo) { _modoActivo = modo; }
    ModoClasificador getModo() const { return _modoActivo; }

    // Ejecución de clasificación recibiendo los 3 sensores vigentes
    ResultadoClasificacion clasificar(float pitch_tibial, bool sw_talon, bool sw_punta);

    // Métodos específicos
    ResultadoClasificacion clasificarPorHisteresis(float pitch_tibial, bool en_apoyo);
    ResultadoClasificacion clasificarPorLogicaDifusa(float pitch_tibial, bool en_apoyo);

    // Mapeo auxiliar de clase a ángulo nominal
    static float claseAAngulo(ClaseTerreno c);
    static const char* claseANombre(ClaseTerreno c);

private:
    ModoClasificador _modoActivo;
    ClaseTerreno _clasePrevia;

    // Parámetros de Histéresis
    const float BANDA_HISTERESIS = 1.5f; // grados

    // Funciones de pertenencia para Lógica Difusa
    float pertenenciaTrapezoidal(float x, float a, float b, float c, float d);
    float pertenenciaTriangular(float x, float a, float b, float c);
};

#endif // CLASIFICADOR_TERRENO_H
