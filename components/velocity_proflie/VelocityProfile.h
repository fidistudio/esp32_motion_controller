#pragma once

typedef enum
{
    LINEAR,
    ANGULAR
} velocity_type_t;

typedef enum
{
    TRIANGULAR_PROFILE,
    TRAPEZOIDAL_PROFILE
} profile_type_t;

typedef enum
{
    SLOPE_LIMITED,
    CONSTANT_ACCEL
} profile_method_t;

class VelocityProfile
{
public:
    VelocityProfile(float V_max, float W_max, float accel_max, float alpha_max, float radius_R, float radius_L, float b, float Ts = 0.01f);
    void updateMotionParameters(float x, float y, profile_method_t method);
    void step(float &wR_, float &wL_);
    void changeVelMax(float v_max, float w_max);

private:
    float generate_slope_limited_profile(velocity_type_t type, float tf, float Vel_max);
    float generate_constant_accel_time_profile(velocity_type_t type,float dist,float Vel_max,float acc_max);

    float v_max_;              // Velocidad lineal máxima [m/s]
    float w_max_;              // Velocidad angular máxima (giro del carrito) [rad/s]
    float accel_max_;          // Aceleración lineal máxima [m/s^2]
    float alpha_max_;          // Aceleración angular máxima [rad/s^2]
    float radius_R_;           // Radio de la rueda derecha [m]
    float radius_L_;           // Radio de la rueda izquierda [m]
    float b_;                  // Distancia de una rueda al eje central del carrito [m]
    float d_;                  // Distancia hacia el objetivo [m]
    float theta_;              // Ángulo de giro para alinearse al objetivo [rad]
    float tf_v_;               // Tiempo final del movimiento lineal [s]
    float tf_w_;               // Tiempo final del giro [s]
    float Ts_;                 // Periodo de muestreo [s]
    float t_;                  // Tiempo acumulado [s]
    float w_;                  // Velocidad angular [rad/s]
    float v_;                  // Velocidad lineal [m/s]
    profile_type_t profile_v_; // Tipo de perfil
    profile_type_t profile_w_; // Tipo de perfil
    profile_method_t method_;  // Método de límite de aceleración o de aceleración fija
};