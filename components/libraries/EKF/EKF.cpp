#include "EKF.h"

#include <cmath>
#include <cstring>

/* ============================================================
 *  Constructor
 * ============================================================ */

EKF::EKF(float dt_s, float q_xy, float q_theta, float r_uwb, float r_imu)
    : dt_(dt_s), q_xy_(q_xy), q_theta_(q_theta), r_uwb_(r_uwb), r_imu_(r_imu) {
  reset();
}

/* ============================================================
 *  reset
 * ============================================================ */

void EKF::reset(float x0, float y0, float theta0) {
  x_[0] = x0;
  x_[1] = y0;
  x_[2] = theta0;

  // Covarianza inicial: incertidumbre moderada
  memset(P_, 0, sizeof(P_));
  P_[0][0] = 1.0f; // σ²_x
  P_[1][1] = 1.0f; // σ²_y
  P_[2][2] = 0.1f; // σ²_θ  (~18°)
}

/* ============================================================
 *  predict
 *
 *  Modelo cinemático diferencial:
 *    f(x,u) = [ x + v·cos(θ)·dt ]
 *             [ y + v·sin(θ)·dt ]
 *             [ θ + ω·dt        ]
 *
 *  Jacobiano F = ∂f/∂x:
 *    F = [ 1  0  -v·sin(θ)·dt ]
 *        [ 0  1   v·cos(θ)·dt ]
 *        [ 0  0   1           ]
 *
 *  P_pred = F·P·Fᵀ + Q
 * ============================================================ */

void EKF::predict(float v, float omega) {
  const float theta = x_[2];
  const float cos_t = cosf(theta);
  const float sin_t = sinf(theta);

  /* --- 1. Propagar estado --- */
  x_[0] += v * cos_t * dt_;
  x_[1] += v * sin_t * dt_;
  x_[2] += omega * dt_;

  /* Normalizar θ a [-π, π] */
  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;

  /* --- 2. Jacobiano F --- */
  float F[3][3] = {{1.f, 0.f, -v * sin_t * dt_},
                   {0.f, 1.f, v * cos_t * dt_},
                   {0.f, 0.f, 1.f}};

  /* --- 3. Ruido de proceso Q --- */
  float Q[3][3] = {{q_xy_, 0.f, 0.f}, {0.f, q_xy_, 0.f}, {0.f, 0.f, q_theta_}};

  /* --- 4. P_pred = F·P·Fᵀ + Q --- */
  float FP[3][3], FPFt[3][3], Ft[3][3];
  mat3x3_mul(F, P_, FP);
  mat3x3_transpose(F, Ft);
  mat3x3_mul(FP, Ft, FPFt);
  mat3x3_add(FPFt, Q, P_);
}

/* ============================================================
 *  correctUWB
 *
 *  Modelo de observación (lineal en x,y):
 *    h(x) = [ x ]    H = [ 1 0 0 ]
 *           [ y ]        [ 0 1 0 ]
 *
 *  S   = H·P·Hᵀ + R          (2×2)
 *  K   = P·Hᵀ · S⁻¹          (3×2)
 *  x  += K · (z - h(x))
 *  P   = (I - K·H) · P
 * ============================================================ */

void EKF::correctUWB(float x_meas, float y_meas) {
  /* Innovación */
  const float inn[2] = {x_meas - x_[0], y_meas - x_[1]};

  /* S = H·P·Hᵀ + R  →  H extrae filas 0 y 1 de P */
  /* S[0][0] = P[0][0] + r_uwb */
  /* S[0][1] = P[0][1]         */
  /* S[1][0] = P[1][0]         */
  /* S[1][1] = P[1][1] + r_uwb */
  float S[2][2] = {{P_[0][0] + r_uwb_, P_[0][1]},
                   {P_[1][0], P_[1][1] + r_uwb_}};

  float Sinv[2][2];
  if (!mat2x2_inv(S, Sinv))
    return; // Sistema degenerado, saltar

  /* K = P·Hᵀ · S⁻¹
   * P·Hᵀ  →  primeras dos columnas de P  (H selecciona x e y)
   * PHt[i][j] = P[i][j]  para j=0,1
   */
  float K[3][2];
  for (int i = 0; i < 3; i++) {
    K[i][0] = P_[i][0] * Sinv[0][0] + P_[i][1] * Sinv[1][0];
    K[i][1] = P_[i][0] * Sinv[0][1] + P_[i][1] * Sinv[1][1];
  }

  /* x += K · inn */
  for (int i = 0; i < 3; i++) {
    x_[i] += K[i][0] * inn[0] + K[i][1] * inn[1];
  }

  /* P = (I - K·H) · P
   * K·H  →  K·H[i][j] = K[i][0]*δ(j,0) + K[i][1]*δ(j,1)
   *        es decir: fila i de KH = K[i][0]·e0 + K[i][1]·e1
   */
  float KH[3][3] = {};
  for (int i = 0; i < 3; i++) {
    KH[i][0] = K[i][0];
    KH[i][1] = K[i][1];
    // KH[i][2] = 0
  }

  float IminusKH[3][3] = {{1.f - KH[0][0], -KH[0][1], 0.f - KH[0][2]},
                          {-KH[1][0], 1.f - KH[1][1], 0.f - KH[1][2]},
                          {-KH[2][0], -KH[2][1], 1.f - KH[2][2]}};

  float Pnew[3][3];
  mat3x3_mul(IminusKH, P_, Pnew);
  memcpy(P_, Pnew, sizeof(P_));

  /* Normalizar θ */
  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;
}

/* ============================================================
 *  correctIMU
 *
 *  Medición escalar de θ:
 *    h(x) = θ     H = [0, 0, 1]
 *
 *  s   = P[2][2] + r_imu
 *  k   = P[:,2] / s          (3×1 gain)
 *  x  += k · (θ_meas − θ)
 *  P   = (I − k·H) · P
 * ============================================================ */

void EKF::correctIMU(float theta_meas) {
  /* Innovación angular normalizada */
  float inn = theta_meas - x_[2];
  while (inn > M_PI)
    inn -= 2.f * M_PI;
  while (inn < -M_PI)
    inn += 2.f * M_PI;

  const float s = P_[2][2] + r_imu_;
  if (s < 1e-9f)
    return;

  /* k = P[:,2] / s */
  float k[3] = {P_[0][2] / s, P_[1][2] / s, P_[2][2] / s};

  /* x += k · inn */
  for (int i = 0; i < 3; i++)
    x_[i] += k[i] * inn;

  /* P = (I - k·H) · P   donde H = [0,0,1] */
  for (int i = 0; i < 3; i++) {
    P_[i][0] -= k[i] * P_[2][0];
    P_[i][1] -= k[i] * P_[2][1];
    P_[i][2] -= k[i] * P_[2][2];
  }

  /* Normalizar θ */
  while (x_[2] > M_PI)
    x_[2] -= 2.f * M_PI;
  while (x_[2] < -M_PI)
    x_[2] += 2.f * M_PI;
}

/* ============================================================
 *  Helpers de álgebra matricial (sin heap, todo en stack)
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
