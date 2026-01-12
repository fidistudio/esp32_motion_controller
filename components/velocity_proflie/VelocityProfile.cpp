#include "VelocityProfile.h"
#include <cmath>

VelocityProfile::VelocityProfile(float V_max, float W_max, float radius_R, float radius_L, float b, float Ts = 0.01f) : v_max_(V_max), w_max_(W_max), radius_R_(radius_R), radius_L_(radius_L), b_(b), Ts_(Ts) {}

void VelocityProfile::updateMotionParameters(float x, float y)
{
    d_ = std::sqrt(x * x + y * y);
    theta_ = std::atan2(y, x);
    tf_v_ = (3 * d_) / (2 * v_max_);
    tf_w_ = (3 * theta_) / (2 * w_max_);
    t_ = 0.0f;
}

void VelocityProfile::step(float &wR_, float &wL_)
{
    t_ += Ts_;
    if (t_ < tf_w_)
    {
        w_ = copysign(1, theta_);
        if (t_ < tf_w_ / 3)
            w_ *= (3 * w_max_ * t_) / tf_w_;
        else if (t_ < (2 * tf_w_) / 3)
            w_ *= w_max_;
        else
            w_ *= w_max_ * (2 * tf_w_ - 3 * t_) / tf_w_;
        wR_ = (b_ * w_) / radius_R_;
        wL_ = -(b_ * w_) / radius_L_;
    }
    else if (t_ < tf_w_ + tf_v_)
    {
        if (t_ - tf_w_ < tf_v_)
            v_ = (3 * v_max_ * (t_ - tf_w_)) / tf_v_;
        else if (t_ - tf_w_ < (2 * tf_v_) / 3)
            v_ = v_max_;
        else
            v_ = v_max_ * (2 * tf_v_ - 3 * (t_ - tf_w_)) / tf_v_;
        wR_ = v_ / radius_R_;
        wL_ = v_ / radius_L_;
    }
    else
    {
        wR_ = 0.0f;
        wL_ = 0.0f;
    }
}