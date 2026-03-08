#pragma once

#include <cstdint>

/**
 * @brief Extended Kalman Filter para pose 2D de robot diferencial.
 *
 * Estado:   x = [x, y, θ]ᵀ
 * Control:  u = [v, ω]ᵀ   (velocidad lineal y angular)
 * Medición: z = [x_uwb, y_uwb]ᵀ  (posición por trilateración UWB)
 *
 * Modelo cinemático diferencial (predicción):
 *   x_k+1 = x_k + v·cos(θ_k)·dt
 *   y_k+1 = y_k + v·sin(θ_k)·dt
 *   θ_k+1 = θ_k + ω·dt
 *
 * La corrección IMU (θ) se aplica directamente como corrección
 * sobre el estado θ usando un Kalman de una dimensión embebido.
 */
class EKF {
public:
  /**
   * @param dt_s        Periodo de predicción en segundos (ej. 0.05 para 20 Hz)
   * @param q_xy        Ruido de proceso en posición [m²]
   * @param q_theta     Ruido de proceso en orientación [rad²]
   * @param r_uwb       Ruido de medición UWB [m²]
   * @param r_imu       Ruido de medición IMU θ [rad²]
   */
  EKF(float dt_s, float q_xy, float q_theta, float r_uwb, float r_imu);

  /* -------- API principal -------- */

  /** Paso de predicción — llamar a 20 Hz con odometría de ruedas */
  void predict(float v, float omega);

  /** Corrección con posición UWB [x_meas, y_meas] en metros */
  void correctUWB(float x_meas, float y_meas);

  /** Corrección con orientación IMU/AHRS en radianes */
  void correctIMU(float theta_meas);

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

  /* -------- Estado x = [x, y, θ] -------- */
  float x_[3];

  /* -------- Covarianza P (3×3, almacenada row-major) -------- */
  float P_[3][3];

  /* -------- Helpers -------- */

  /** Multiplica A(3×3) · B(3×3) → C(3×3) */
  static void mat3x3_mul(const float A[3][3], const float B[3][3],
                         float C[3][3]);

  /** Transpone A(3×3) → At(3×3) */
  static void mat3x3_transpose(const float A[3][3], float At[3][3]);

  /** Suma A + B → C (3×3) */
  static void mat3x3_add(const float A[3][3], const float B[3][3],
                         float C[3][3]);

  /** Invierte matriz 2×2 in-place.  Retorna false si singular. */
  static bool mat2x2_inv(const float M[2][2], float Minv[2][2]);
};
