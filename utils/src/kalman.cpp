#include "kalman.hpp"
#include <iostream>
namespace utils
{
    void Kalman::setDt() 
    {
        auto now = std::chrono::steady_clock::now();
        if(first_call_){
            dt_ = 0;
        } else {
            dt_ = std::chrono::duration_cast<std::chrono::microseconds>(now - last_call_).count();
        }
        last_call_ = now;
    }

    void Kalman::init() 
    {
        
        //init empty A, B and Q
        A_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        B_ = Eigen::MatrixXd::Zero(P_.rows(), 1);
        Q_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());

        // init Eigenmatrix
        I_= Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        I_.setIdentity();
    }
    
    void Kalman::updateStep(const Eigen::VectorXd& z, const Eigen::VectorXd& u) 
    {
        std::lock_guard<std::mutex> guard(mtx_);
        setDt();
        updateA();
        updateB();
        updateQ();
        x_ = A_ * x_ + B_ * u;
        P_ = A_ * P_ * A_.transpose() + Q_;
        K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
        x_ = x_ + K_ * (z - C_ * x_);
        P_ = (I_ - K_ * C_) * P_;
    }

    Kalman::Kalman(const Eigen::MatrixXd& C, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& R,
                    const Eigen::VectorXd& x0) : 
                    C_{C}, P_{P_0}, R_{R}, x_{x0}
    {
        
        init();
    }
    
    Kalman::Kalman(
                    const Eigen::MatrixXd& C, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& R) :
                    C_{C}, P_{P_0}, R_{R}
    {
        init();
        x_ = Eigen::VectorXd::Zero(A_.rows());
    }


    const Eigen::VectorXd& Kalman::predict(const Eigen::VectorXd& z) 
    {
        Eigen::VectorXd u(B_.cols());
        u.setZero();
        updateStep(z, u);
        return x_;
    } 
    
    const Eigen::VectorXd& Kalman::predict(const Eigen::VectorXd& z, const Eigen::VectorXd& u) 
    {
        updateStep(z, u);
        return x_;
    }
    
    double Kalman::getDt() 
    {
        return dt_;
    }
}