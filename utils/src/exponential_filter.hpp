#include <Eigen/Dense>
#include <mutex>

#ifndef UTILS_EXPONENTIAL_FILTER_H
#define UTILS_EXPONENTIAL_FILTER_H
//https://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Exponential-Filter/exponential-filter.htm

namespace utils
{
    class ExponentialFilter
    {
    private:
        Eigen::VectorXd y_last_;
        const double a_;

    public:
        /**
         * Constructor for filter
         * @param a: The filter value (usually between 0.8 and 0.99)
         * @param y_0: The init value of the filter (usually 0)
         */
        ExponentialFilter(double a, const Eigen::VectorXd &y_0);

        /**
         * Predict the next value of the signal
         * @param x: The to be filtered vector; return value is stored in same vector
         */
        void predict(Eigen::VectorXd &x);
    };
} // namespace utils

#endif
