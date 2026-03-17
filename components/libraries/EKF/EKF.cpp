#include "EKF.h"

#include <cmath>
#include <cstring>

/* ============================================================
 *  Constructor
 *  CAMBIO: nuevo parámetro r_odom
 * ============================================================ */
EKF::EKF(float dt_s, float q_xy, float q_theta, float r_uwb, float r_imu,
         float r_odom)
    : dt_(dt_s), q_xy_(q_xy), q_theta_(q_theta), r_uwb_(r_uwb), r_imu_(r_imu),
      r_odom_(r_odom) {
  reset();
}

/* ============================================================
 *  reset — sin cambios
 * ============================================================ */
void EKF::reset(float x0, float y0, float theta0) {
  x_[0] = x0;
  x_[1] = y0;
  x_[2] = theta0;

  memset(P_, 0, sizeof(P_));
  P_[0][0] = 1.0f;
  P_[1][1] = 1.0f;
  P_[2][2] = 0.1f;
}

/* ============================================================
 *  predict — sin cambios
 * ============================================================ */
void EKF::predict(float v, float omega) {
  const float theta = x_[2];
  const float cos_t = cosf(theta);
  const float sin_t = sinf(theta);

  x_[0] += v * cos_t * dt_;
  x_[1] += v * sin_t * dt_;
  x_[2] += omega * dt_;

  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;

  float F[3][3] = {{1.f, 0.f, -v * sin_t * dt_},
                   {0.f, 1.f, v * cos_t * dt_},
                   {0.f, 0.f, 1.f}};

  float Q[3][3] = {{q_xy_, 0.f, 0.f}, {0.f, q_xy_, 0.f}, {0.f, 0.f, q_theta_}};

  float FP[3][3], FPFt[3][3], Ft[3][3];
  mat3x3_mul(F, P_, FP);
  mat3x3_transpose(F, Ft);
  mat3x3_mul(FP, Ft, FPFt);
  mat3x3_add(FPFt, Q, P_);
}

/* ============================================================
 *  correctUWB — sin cambios
 * ============================================================ */
void EKF::correctUWB(float x_meas, float y_meas) {
  const float inn[2] = {x_meas - x_[0], y_meas - x_[1]};

  float S[2][2] = {{P_[0][0] + r_uwb_, P_[0][1]},
                   {P_[1][0], P_[1][1] + r_uwb_}};

  float Sinv[2][2];
  if (!mat2x2_inv(S, Sinv))
    return;

  float K[3][2];
  for (int i = 0; i < 3; i++) {
    K[i][0] = P_[i][0] * Sinv[0][0] + P_[i][1] * Sinv[1][0];
    K[i][1] = P_[i][0] * Sinv[0][1] + P_[i][1] * Sinv[1][1];
  }

  for (int i = 0; i < 3; i++)
    x_[i] += K[i][0] * inn[0] + K[i][1] * inn[1];

  float KH[3][3] = {};
  for (int i = 0; i < 3; i++) {
    KH[i][0] = K[i][0];
    KH[i][1] = K[i][1];
  }

  float IminusKH[3][3] = {{1.f - KH[0][0], -KH[0][1], -KH[0][2]},
                          {-KH[1][0], 1.f - KH[1][1], -KH[1][2]},
                          {-KH[2][0], -KH[2][1], 1.f - KH[2][2]}};

  float Pnew[3][3];
  mat3x3_mul(IminusKH, P_, Pnew);
  memcpy(P_, Pnew, sizeof(P_));

  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;
}

/* ============================================================
 *  correctTheta  ← NUEVO método privado genérico
 *
 *  Corrección escalar de θ con ruido de medición configurable.
 *  Tanto correctIMU como correctOdom lo llaman con su propio R.
 *
 *  h(x) = θ     H = [0, 0, 1]
 *
 *  s  = P[2][2] + r
 *  k  = P[:,2] / s
 *  x += k · (θ_meas − θ)   con innovación normalizada a [-π, π]
 *  P  = (I − k·H) · P
 * ============================================================ */
void EKF::correctTheta(float theta_meas, float r) {
  float inn = theta_meas - x_[2];
  while (inn > M_PI)
    inn -= 2.f * M_PI;
  while (inn < -M_PI)
    inn += 2.f * M_PI;

  const float s = P_[2][2] + r;
  if (s < 1e-9f)
    return;

  float k[3] = {P_[0][2] / s, P_[1][2] / s, P_[2][2] / s};

  for (int i = 0; i < 3; i++)
    x_[i] += k[i] * inn;

  // P = (I - k·H) · P   donde H = [0, 0, 1]
  for (int i = 0; i < 3; i++) {
    P_[i][0] -= k[i] * P_[2][0];
    P_[i][1] -= k[i] * P_[2][1];
    P_[i][2] -= k[i] * P_[2][2];
  }

  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;
}

/* ============================================================
 *  correctIMU — ahora delega en correctTheta con r_imu_
 *  Comportamiento idéntico al original, código más limpio.
 * ============================================================ */
void EKF::correctIMU(float theta_meas) { correctTheta(theta_meas, r_imu_); }

/* ============================================================
 *  correctOdom  ← NUEVO método público
 *
 *  Misma ecuación que correctIMU pero con r_odom_ como ruido.
 *  r_odom > r_imu  →  el EKF confía menos en la odometría
 *  que en la IMU, pero la usa para corregir el drift de largo
 *  plazo del giroscopio cuando hay slip o superficies irregulares.
 * ============================================================ */
void EKF::correctOdom(float theta_meas) { correctTheta(theta_meas, r_odom_); }

/* ============================================================
 *  Helpers — sin cambios
 * ============================================================ */
void EKF::mat3x3_mul(const float A[3][3], const float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      C[i][j] = 0.f;
      for (int k = 0; k < 3; k++)
        C[i][j] += A[i][k] * B[k][j];
    }
}

void EKF::mat3x3_transpose(const float A[3][3], float At[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      At[i][j] = A[j][i];
}

void EKF::mat3x3_add(const float A[3][3], const float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C[i][j] = A[i][j] + B[i][j];
}

bool EKF::mat2x2_inv(const float M[2][2], float Minv[2][2]) {
  const float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
  if (fabsf(det) < 1e-9f)
    return false;
  const float inv_det = 1.f / det;
  Minv[0][0] = M[1][1] * inv_det;
  Minv[0][1] = -M[0][1] * inv_det;
  Minv[1][0] = -M[1][0] * inv_det;
  Minv[1][1] = M[0][0] * inv_det;
  return true;
}
