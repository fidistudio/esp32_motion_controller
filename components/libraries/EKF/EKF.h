#pragma once
#include <cstdint>

/**
 * @brief Extended Kalman Filter para pose 2D de robot diferencial.
 *
 * Estado:   x = [x, y, θ]ᵀ
 * Control:  u = [v, ω]ᵀ   (velocidad lineal y angular)
 * Mediciones:
 *   - UWB  → [x_uwb, y_uwb]  (posición por trilateración)
 *   - IMU  → θ_imu           (giroscopio + acelerómetro, sin mag)
 *   - Odom → θ_odom          (odometría diferencial de encoders)
 *
 * Orden de fusión por ciclo:
 *   predict(v, ω)  →  correctIMU(θ)  →  correctOdom(θ)  →  [correctUWB si hay
 * dato nuevo]
 *
 * IMU y odometría tienen errores complementarios:
 *   IMU  → bueno a corto plazo, deriva lentamente (bias de giroscopio)
 *   Odom → bueno a largo plazo, ruidoso por slip de ruedas
 * El EKF los pondera automáticamente vía R_imu y R_odom.
 */
class EKF {
public:
  /**
   * @param dt_s      Periodo de predicción en segundos (ej. 0.05 para 20 Hz)
   * @param q_xy      Ruido de proceso en posición [m²]
   * @param q_theta   Ruido de proceso en orientación [rad²]
   * @param r_uwb     Ruido de medición UWB [m²]
   * @param r_imu     Ruido de medición IMU θ [rad²]  — más bajo = más confianza
   * @param r_odom    Ruido de medición odometría θ [rad²] — más alto = menos
   * confianza
   */
  EKF(float dt_s, float q_xy, float q_theta, float r_uwb, float r_imu,
      float r_odom);

  /* -------- API principal -------- */

  /** Paso de predicción — llamar a 20 Hz con odometría de ruedas */
  void predict(float v, float omega);

  /** Corrección con posición UWB [x_meas, y_meas] en metros */
  void correctUWB(float x_meas, float y_meas);

  /** Corrección con orientación IMU/AHRS en radianes (sin magnetómetro) */
  void correctIMU(float theta_meas);

  /**
   * Corrección con yaw calculado desde encoders (odometría diferencial).
   * Misma ecuación que correctIMU pero con r_odom_ como ruido de medición.
   * Actúa como ancla de largo plazo para compensar el drift del giroscopio.
   */
  void correctOdom(float theta_meas);

  /* -------- Getters de estado -------- */
  float getX() const { return x_[0]; }
  float getY() const { return x_[1]; }
  float getTheta() const { return x_[2]; }

  /** Reinicializa el estado y covarianza */
  void reset(float x0 = 0.f, float y0 = 0.f, float theta0 = 0.f);

private:
  /* -------- Parámetros -------- */
  float dt_;
  float q_xy_;
  float q_theta_;
  float r_uwb_;
  float r_imu_;
  float r_odom_; // NUEVO

  /* -------- Estado x = [x, y, θ] -------- */
  float x_[3];

  /* -------- Covarianza P (3×3) -------- */
  float P_[3][3];

  /* -------- Corrección escalar de θ genérica (interna) -------- */
  void correctTheta(float theta_meas, float r);

  /* -------- Helpers de álgebra matricial -------- */
  static void mat3x3_mul(const float A[3][3], const float B[3][3],
                         float C[3][3]);
  static void mat3x3_transpose(const float A[3][3], float At[3][3]);
  static void mat3x3_add(const float A[3][3], const float B[3][3],
                         float C[3][3]);
  static bool mat2x2_inv(const float M[2][2], float Minv[2][2]);
};
