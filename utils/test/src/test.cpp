#include "utils/utils.hpp"
#include <iostream>
#include <unistd.h>

class K : public utils::Kalman {
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
        const Eigen::MatrixXd& R) : Kalman(C, P_0, R)
    {

    }
};

void old(){
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
    Eigen::VectorXd x(1), y;
    x << 2;
    kal.predict(y, x);
    std::cout << "y=" << y << std::endl;
    kal.predict(y, x);
    std::cout << "y=" << y << std::endl;
}

void test(){
    usleep(1000000);
    std::cout << "waited 1s" << std::endl;
}

int main() {
    // utils::StoppableThread th(SCHED_RR, 90);
    // std::function<void(void)> f = test;
    // th.run(f);
    Eigen::VectorXd z(3);
    utils::PRINT_MATRIX_SIZES(Eigen::MatrixXd::Zero(6, 6), z);
    std::cin.get();
    return 0;
}