#include "utils/utils.hpp"
#include <iostream>
#include <unistd.h>

class K : public utils::Kalman<std::chrono::milliseconds> {
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

struct mpu_kalman_vel_conf {
        double r, q11, q22, q33, q44, q55, q66;

        mpu_kalman_vel_conf() {
            this->r = 0.03;
            this->q11 = 0.02;
            this->q22 = 0.02;
            this->q33 = 0.02;
            this->q44 = 0.02;
            this->q55 = 0.02;
            this->q66 = 0.02;
        }

        mpu_kalman_vel_conf(double r, double q11, double q22, double q33, double q44, double q55, double q66){
            this->r = r;
            this->q11 = q11;
            this->q22 = q22;
            this->q33 = q33;
            this->q44 = q44;
            this->q55 = q55;
            this->q66 = q66;
        }

        mpu_kalman_vel_conf(const mpu_kalman_vel_conf& conf) {
            r = conf.r;
            q11 = conf.q11;
            q22 = conf.q22;
            q33 = conf.q33;
            q44 = conf.q44;
            q55 = conf.q55;
            q66 = conf.q66;
        }
    };

    class MPU6050_Kalman_Vel : public utils::Kalman<std::chrono::milliseconds> {
    private:
        mpu_kalman_vel_conf conf_;


    protected:
        void updateA() {
            A_.setIdentity();
        }

        void updateQ() {
            Q_ << conf_.q11, 0, 0, 0, 0, 0, 
                0, conf_.q22, 0, 0, 0, 0,
                0, 0, conf_.q33, 0, 0, 0,
                0, 0, 0, conf_.q44, 0, 0,
                0, 0, 0, 0, conf_.q55, 0,
                0, 0, 0, 0, 0, conf_.q66;
                
        }

    public:
        MPU6050_Kalman_Vel() : MPU6050_Kalman_Vel(mpu_kalman_vel_conf())
        {

        }

        MPU6050_Kalman_Vel(const MPU6050_Kalman_Vel&) : MPU6050_Kalman_Vel() {

        }

        MPU6050_Kalman_Vel(const mpu_kalman_vel_conf& conf) : Kalman((Eigen::MatrixXd(3, 6) << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0).finished(), Eigen::MatrixXd::Zero(6,6), (Eigen::MatrixXd(3,3) << conf.r, 0, 0, 0, conf.r, 0, 0, 0, conf.r).finished()), conf_{conf}
        {

        }

        MPU6050_Kalman_Vel(const mpu_kalman_vel_conf& conf, const Eigen::VectorXd& x_0) : Kalman((Eigen::MatrixXd(3, 6) << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0).finished(), Eigen::MatrixXd::Zero(6,6), (Eigen::MatrixXd(3,3) << conf.r, 0, 0, 0, conf.r, 0, 0, 0, conf.r).finished(), x_0), conf_{conf}
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
    kal.predict(y,x);
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
    Eigen::VectorXd z(3), x;
    MPU6050_Kalman_Vel k;
    z << 1, 1, 1;
    k.predict(x, z);
    std::cin.get();
    return 0;
}