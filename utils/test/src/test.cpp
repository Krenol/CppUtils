#include "utils/utils.hpp"
#include <iostream>

class K : public utils::Kalmann {
protected:
    void updateA() {
        auto dt = getDt();
        A_ << 1, -dt, 0, 1;
    }

    void updateB() {
        auto dt = getDt();
        B_ << dt, 0;
    }

    void updateQ() {
        auto dt = getDt();
        Q_ << 0.001 * dt, 0, 0, 0.003 * dt;
    }

public:
    K(const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& P_0,
        const Eigen::MatrixXd& R) : Kalmann(C, P_0, R)
    {

    }
};

int main() {
    Eigen::MatrixXd C(1,2), P_0(2,2), R(1,1);
    C << 1, 0;
    P_0.setZero();
    R << 0.03;
    std::cout << "R: " << R << std::endl << "P_0: " << P_0 << std::endl << "C: " << C << std::endl;
    K kal(C, P_0, R);
    // std::vector<double> test ={ 0.0, 1.2, 1.6, 1, 9, 2.7, 3.1, 2.2 };
    // auto m = utils::Maths::mean(test);
    // auto d = utils::Maths::deviation(test);
    // std::cout << "mean: " << m << "\tdeviation: " << d << std::endl;
    Eigen::VectorXd x(1);
    x << 2;
    auto y = kal.predict(x);
    std::cout << "y=" << y << std::endl;
    x << 4;
    y = kal.predict(x);
    std::cout << "y=" << y << std::endl;
    std::cout << "TEST ENV VAR = " << utils::Environment::getVar("TEST", "OK") << std::endl << "waiting 10000ms...\n";
    utils::Waiter::SleepMillis(10000);
    std::cout << "waited 10000ms" << std::endl;
    return 0;
}