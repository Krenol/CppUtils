#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

#ifndef UTILS_MATHS_H
#define UTILS_MATHS_H

namespace utils {

    
    class Maths {
        public:
            static double mean(const std::vector<double>& vec);

            static double deviation(const std::vector<double>& vec);

    };
}

#endif