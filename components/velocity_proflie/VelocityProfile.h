#pragma once
#include <cstdint>

enum class VelocityType : uint8_t
{
    LINEAR,
    ANGULAR
};

enum class ProfileType : uint8_t
{
    TRIANGULAR,
    TRAPEZOIDAL
};

enum class ProfileMethod : uint8_t
{
    SLOPE_LIMITED,
    CONSTANT_ACCEL
};

class VelocityProfile
{
public:
    VelocityProfile(float v_max, float w_max, float accel_max, float alpha_max, float radius_r, float radius_l, float b, float sampling_period = 0.01f, ProfileMethod method);
    void updateMotionParameters(float x, float y);
    void step(float &wR_, float &wL_);
    void changeVelMax(float v_max, float w_max);
    void changeMethod(ProfileMethod method);
    bool isProfileFinished();

private:
    float generate_slope_limited_profile(VelocityType type, float tf, float Vel_max);
    float generate_constant_accel_time_profile(VelocityType type, float dist, float Vel_max, float acc_max);

    float v_max_;           // Velocidad lineal máxima [m/s]
    float w_max_;           // Velocidad angular máxima (giro del carrito) [rad/s]
    float accel_max_;       // Aceleración lineal máxima [m/s^2]
    float alpha_max_;       // Aceleración angular máxima [rad/s^2]
    float radius_r_;        // Radio de la rueda derecha [m]
    float radius_l_;        // Radio de la rueda izquierda [m]
    float b_;               // Distancia de una rueda al eje central del carrito [m]
    float d_;               // Distancia hacia el objetivo [m]
    float theta_;           // Ángulo de giro para alinearse al objetivo [rad]
    float tf_v_;            // Tiempo final del movimiento lineal [s]
    float tf_w_;            // Tiempo final del giro [s]
    float sampling_period_; // Periodo de muestreo [s]
    float t_;               // Tiempo acumulado [s]
    float w_;               // Velocidad angular [rad/s]
    float v_;               // Velocidad lineal [m/s]
    ProfileType profile_v_; // Tipo de perfil
    ProfileType profile_w_; // Tipo de perfil
    ProfileMethod method_;  // Método de límite de aceleración o de aceleración fija
};