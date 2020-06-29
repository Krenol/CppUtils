#include "utils/utils.hpp"
#include <iostream>



int main() {
    std::vector<double> test = {0.0, 1.2, 1.6, 1,9, 2.7, 3.1, 2.2};
    auto m = utils::Maths::mean(test);
    auto d = utils::Maths::deviation(test);
    std::cout << "mean: " << m << "\tdeviation: " << d << std::endl; 
    utils::Waiter::SleepMillis(10000);
    std::cout << "waited 10000ms" << std::endl; 
    return 0;
}