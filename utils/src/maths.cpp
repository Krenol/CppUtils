#include "maths.hpp"
namespace utils
{
    double Maths::mean(const std::vector<double>& vec)
    {
        double sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
        double m =  sum / vec.size();
        return m;
    }
    
    double Maths::deviation(const std::vector<double>& vec)
    {
        auto m = mean(vec);
        double accum = 0.0;
        std::for_each (std::begin(vec), std::end(vec), [&](const double d) {
            accum += (d - m) * (d - m);
        });

        double dev = sqrt(accum / (vec.size()-1));
        return dev;
    }
}