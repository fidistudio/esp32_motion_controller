#pragma once

class VelocityProfile
{
public:
    VelocityProfile(float V_max, float W_max, float radius_R, float radius_L, float b, float Ts = 0.01f);
    void updateMotionParameters(float x, float y);
    void step(float &wR_, float &wL_);

private:
    float v_max_;    // Velocidad lineal máxima [m/s]
    float w_max_;    // Velocidad angular máxima (giro del carrito) [rad/s]
    float radius_R_; // Radio de la rueda derecha [m]
    float radius_L_; // Radio de la rueda izquierda [m]
    float b_;        // Distancia de una rueda al eje central del carrito [m]
    float d_;        // Distancia hacia el objetivo [m]
    float theta_;    // Ángulo de giro para alinearse al objetivo [rad]
    float tf_v_;     // Tiempo final del movimiento lineal [s]
    float tf_w_;     // Tiempo final del giro [s]
    float Ts_;       // Periodo de muestreo [s]
    float t_;        // Tiempo acumulado [s]
    float w_;        // Velocidad angular [rad/s]
    float v_;        // Velocidad lineal [m/s]
};