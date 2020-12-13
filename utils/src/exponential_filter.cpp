#include "exponential_filter.hpp"

namespace utils
{
    ExponentialFilter::ExponentialFilter(double a, const Eigen::VectorXd &y_0) : a_{a}, y_last_{y_0}
    {
    }

    void ExponentialFilter::predict(Eigen::VectorXd &x)
    {
        x = (1 - a_) * y_last_ + a_ * x;
        y_last_ = x;
    }
} // namespace utils