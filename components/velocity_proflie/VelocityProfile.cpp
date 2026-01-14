#include "VelocityProfile.h"
#include <cmath>

VelocityProfile::VelocityProfile(float V_max, float W_max, float accel_max, float alpha_max, float radius_R, float radius_L, float b, float Ts) : v_max_(V_max), w_max_(W_max), accel_max_(accel_max), alpha_max_(alpha_max), radius_R_(radius_R), radius_L_(radius_L), b_(b), Ts_(Ts) {}

void VelocityProfile::updateMotionParameters(float x, float y, profile_method_t method)
{
    d_ = std::sqrt(x * x + y * y);
    theta_ = std::atan2(y, x);
    t_ = 0.0f;
    method_ = method;
    if (method_ == profile_method_t::SLOPE_LIMITED) // Método de pendiente limitada
    {
        tf_v_ = (3 * d_) / (2 * v_max_);
        if (3 * v_max_ / tf_v_ > accel_max_)
        {
            profile_v_ = profile_type_t::TRIANGULAR_PROFILE;
            tf_v_ = 2 * d_ / v_max_;
        }
        else
            profile_v_ = profile_type_t::TRAPEZOIDAL_PROFILE;

        tf_w_ = (3 * theta_) / (2 * w_max_);
        if (3 * w_max_ / tf_w_ > alpha_max_)
        {
            profile_w_ = profile_type_t::TRIANGULAR_PROFILE;
            tf_w_ = 2 * theta_ / w_max_;
        }
        else
            profile_w_ = profile_type_t::TRAPEZOIDAL_PROFILE;
    }
    else // Método de pendiente fija
    {
        if (d_ > v_max_ * v_max_ / accel_max_)
        {
            profile_v_ = profile_type_t::TRAPEZOIDAL_PROFILE;
            tf_v_ = (d_ / v_max_) + (v_max_ / accel_max_);
        }
        else
        {
            profile_v_ = profile_type_t::TRIANGULAR_PROFILE;
            tf_v_ = 2 * std::sqrt(d_ / accel_max_);
        }

        if (theta_ > w_max_ * w_max_ / alpha_max_)
        {
            profile_w_ = profile_type_t::TRAPEZOIDAL_PROFILE;
            tf_w_ = (theta_ / w_max_) + (w_max_ / alpha_max_);
        }
        else
        {
            profile_w_ = profile_type_t::TRIANGULAR_PROFILE;
            tf_w_ = 2 * std::sqrt(theta_ / alpha_max_);
        }
    }
}

void VelocityProfile::step(float &wR_, float &wL_)
{
    if (t_ < tf_w_)
    {
        if (method_ == profile_method_t::SLOPE_LIMITED)
            w_ = generate_slope_limited_profile(velocity_type_t::ANGULAR, tf_w_, w_max_);

        else
            w_ = generate_constant_accel_time_profile(velocity_type_t::ANGULAR, theta_, w_max_, alpha_max_);

        wR_ = (b_ * w_) / radius_R_;
        wL_ = -(b_ * w_) / radius_L_;
    }
    else if (t_ < tf_w_ + tf_v_)
    {
        if (method_ == profile_method_t::SLOPE_LIMITED)
            v_ = generate_slope_limited_profile(velocity_type_t::LINEAR, tf_v_, v_max_);
        else
            v_ = generate_constant_accel_time_profile(velocity_type_t::LINEAR, d_, v_max_, accel_max_);

        wR_ = v_ / radius_R_;
        wL_ = v_ / radius_L_;
    }
    else
    {
        wR_ = 0.0f;
        wL_ = 0.0f;
    }
    t_ += Ts_;
}

void VelocityProfile::changeVelMax(float v_max, float w_max)
{
    v_max_ = v_max;
    w_max_ = w_max;
}

float VelocityProfile::generate_slope_limited_profile(velocity_type_t type, float tf, float vel_max)
{
    float t;
    float v;
    if (type == velocity_type_t::ANGULAR)
    {
        v = copysign(1, theta_);
        t = t_;
    }
    else
    {
        v = 1;
        t = t_ - tf_w_;
    }
    if (type == velocity_type_t::ANGULAR && profile_w_ == profile_type_t::TRAPEZOIDAL_PROFILE || type == velocity_type_t::LINEAR && profile_v_ == profile_type_t::TRAPEZOIDAL_PROFILE)
    {
        if (t < tf / 3)
            v *= (3 * vel_max * t) / tf;
        else if (t < (2 * tf) / 3)
            v *= vel_max;
        else
            v *= 3 * vel_max * (tf - t) / tf;
    }
    else if (type == velocity_type_t::ANGULAR && profile_w_ == profile_type_t::TRIANGULAR_PROFILE || type == velocity_type_t::LINEAR && profile_v_ == profile_type_t::TRIANGULAR_PROFILE)
    {
        if (t < tf / 2)
            v *= 2 * vel_max * t / tf;
        else
            v *= 2 * vel_max * (tf - t) / tf;
    }
    return v;
}

float VelocityProfile::generate_constant_accel_time_profile(velocity_type_t type, float dist, float vel_max, float acc_max)
{
    float t;
    float v;
    if (type == velocity_type_t::ANGULAR)
    {
        v = copysign(1, theta_);
        t = t_;
    }
    else
    {
        v = 1;
        t = t_ - tf_w_;
    }
    if (type == velocity_type_t::ANGULAR && profile_w_ == profile_type_t::TRAPEZOIDAL_PROFILE || type == velocity_type_t::LINEAR && profile_v_ == profile_type_t::TRAPEZOIDAL_PROFILE)
    {
        if (t < vel_max / acc_max)
            v *= acc_max * t;
        else if (t < dist / vel_max)
            v *= vel_max;
        else
            v *= vel_max - acc_max * ((dist / vel_max) - t);
    }
    else if (type == velocity_type_t::ANGULAR && profile_w_ == profile_type_t::TRIANGULAR_PROFILE || type == velocity_type_t::LINEAR && profile_v_ == profile_type_t::TRIANGULAR_PROFILE)
    {
        if (t < std::sqrt(dist / acc_max))
            v *= acc_max * t;
        else
            v *= acc_max * (2 * std::sqrt(dist / acc_max) - t);
    }
    return v;
}