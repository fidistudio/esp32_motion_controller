#include "VelocityProfile.h"
#include <cmath>

VelocityProfile::VelocityProfile(float v_max, float w_max, float accel_max, float alpha_max, float radius_r, float radius_l, float b, float sampling_period, ProfileMethod method) : v_max_(v_max), w_max_(w_max), accel_max_(accel_max), alpha_max_(alpha_max), radius_r_(radius_r), radius_l_(radius_l), b_(b), sampling_period_(sampling_period), method_(method) {}

void VelocityProfile::updateMotionParameters(float x, float y)
{
    d_ = std::sqrt(x * x + y * y);
    theta_ = std::atan2(y, x);
    t_ = 0.0f;
    if (method_ == ProfileMethod::SLOPE_LIMITED) // Método de pendiente limitada
    {
        tf_v_ = (3 * d_) / (2 * v_max_);
        if (3 * v_max_ / tf_v_ > accel_max_)
        {
            profile_v_ = ProfileType::TRIANGULAR;
            tf_v_ = 2 * d_ / v_max_;
        }
        else
            profile_v_ = ProfileType::TRAPEZOIDAL;

        tf_w_ = (3 * theta_) / (2 * w_max_);
        if (3 * w_max_ / tf_w_ > alpha_max_)
        {
            profile_w_ = ProfileType::TRIANGULAR;
            tf_w_ = 2 * theta_ / w_max_;
        }
        else
            profile_w_ = ProfileType::TRAPEZOIDAL;
    }
    else // Método de pendiente fija
    {
        if (d_ > v_max_ * v_max_ / accel_max_)
        {
            profile_v_ = ProfileType::TRAPEZOIDAL;
            tf_v_ = (d_ / v_max_) + (v_max_ / accel_max_);
        }
        else
        {
            profile_v_ = ProfileType::TRIANGULAR;
            tf_v_ = 2 * std::sqrt(d_ / accel_max_);
        }

        if (theta_ > w_max_ * w_max_ / alpha_max_)
        {
            profile_w_ = ProfileType::TRAPEZOIDAL;
            tf_w_ = (theta_ / w_max_) + (w_max_ / alpha_max_);
        }
        else
        {
            profile_w_ = ProfileType::TRIANGULAR;
            tf_w_ = 2 * std::sqrt(theta_ / alpha_max_);
        }
    }
}

void VelocityProfile::step(float &wR_, float &wL_)
{
    if (t_ < tf_w_)
    {
        if (method_ == ProfileMethod::SLOPE_LIMITED)
            w_ = generate_slope_limited_profile(VelocityType::ANGULAR, tf_w_, w_max_);

        else
            w_ = generate_constant_accel_time_profile(VelocityType::ANGULAR, theta_, w_max_, alpha_max_);

        wR_ = (b_ * w_) / radius_r_;
        wL_ = -(b_ * w_) / radius_l_;
    }
    else if (t_ < tf_w_ + tf_v_)
    {
        if (method_ == ProfileMethod::SLOPE_LIMITED)
            v_ = generate_slope_limited_profile(VelocityType::LINEAR, tf_v_, v_max_);
        else
            v_ = generate_constant_accel_time_profile(VelocityType::LINEAR, d_, v_max_, accel_max_);

        wR_ = v_ / radius_r_;
        wL_ = v_ / radius_l_;
    }
    else
    {
        wR_ = 0.0f;
        wL_ = 0.0f;
    }
    t_ += sampling_period_;
}

void VelocityProfile::changeVelMax(float v_max, float w_max)
{
    v_max_ = v_max;
    w_max_ = w_max;
}

void VelocityProfile::changeMethod(ProfileMethod method){
    method_=method;
}

bool VelocityProfile::isProfileFinished(){
    return t_>=tf_w_+tf_v_;
}

float VelocityProfile::generate_slope_limited_profile(VelocityType type, float tf, float vel_max)
{
    float t;
    float v;
    if (type == VelocityType::ANGULAR)
    {
        v = copysign(1, theta_);
        t = t_;
    }
    else
    {
        v = 1;
        t = t_ - tf_w_;
    }
    if (type == VelocityType::ANGULAR && profile_w_ == ProfileType::TRAPEZOIDAL || type == VelocityType::LINEAR && profile_v_ == ProfileType::TRAPEZOIDAL)
    {
        if (t < tf / 3)
            v *= (3 * vel_max * t) / tf;
        else if (t < (2 * tf) / 3)
            v *= vel_max;
        else
            v *= 3 * vel_max * (tf - t) / tf;
    }
    else if (type == VelocityType::ANGULAR && profile_w_ == ProfileType::TRIANGULAR || type == VelocityType::LINEAR && profile_v_ == ProfileType::TRIANGULAR)
    {
        if (t < tf / 2)
            v *= 2 * vel_max * t / tf;
        else
            v *= 2 * vel_max * (tf - t) / tf;
    }
    return v;
}

float VelocityProfile::generate_constant_accel_time_profile(VelocityType type, float dist, float vel_max, float acc_max)
{
    float t;
    float v;
    if (type == VelocityType::ANGULAR)
    {
        v = copysign(1, theta_);
        t = t_;
    }
    else
    {
        v = 1;
        t = t_ - tf_w_;
    }
    if (type == VelocityType::ANGULAR && profile_w_ == ProfileType::TRAPEZOIDAL || type == VelocityType::LINEAR && profile_v_ == ProfileType::TRAPEZOIDAL)
    {
        if (t < vel_max / acc_max)
            v *= acc_max * t;
        else if (t < dist / vel_max)
            v *= vel_max;
        else
            v *= vel_max - acc_max * ((dist / vel_max) - t);
    }
    else if (type == VelocityType::ANGULAR && profile_w_ == ProfileType::TRIANGULAR || type == VelocityType::LINEAR && profile_v_ == ProfileType::TRIANGULAR)
    {
        if (t < std::sqrt(dist / acc_max))
            v *= acc_max * t;
        else
            v *= acc_max * (2 * std::sqrt(dist / acc_max) - t);
    }
    return v;
}

