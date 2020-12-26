#include "kalman.hpp"

namespace utils
{
    void Kalman::setDt() 
    {
        auto now = std::chrono::steady_clock::now();
        if(first_call_){
            dt_ = 1;
            first_call_ = false;
        } else {
            dt_ = std::chrono::duration_cast<std::chrono::microseconds>(now - last_call_).count();
        }
        last_call_ = now;
    }

    void Kalman::init() 
    {
        
        //init empty A and Q
        A_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        Q_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());

        // init Eigenmatrix
        I_= Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        I_.setIdentity();
    }
    
    void Kalman::preupdate() 
    {
        setDt();
        updateA();
        updateB();
        updateC();
        updateQ();
        updateR();
    }
    
    void Kalman::updateStep(const Eigen::VectorXd& z) 
    {
        P_ = A_ * P_ * A_.transpose() + Q_;
        K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
        x_ = x_ + K_ * (z - C_ * x_);
        P_ = (I_ - K_ * C_) * P_;
    }
    
    void Kalman::updateA() 
    {
        
    }
    
    void Kalman::updateB() 
    {
        
    }
    
    void Kalman::updateC() 
    {
        
    }
    
    void Kalman::updateQ() 
    {

    }
    
    void Kalman::updateR() 
    {
        
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


    void Kalman::predict(Eigen::VectorXd& out, const Eigen::VectorXd& z) 
    {
        std::lock_guard<std::mutex> guard(mtx_);
        preupdate();
        x_ = A_ * x_;
        updateStep(z);
        out = x_;
    } 
    
    void Kalman::predict(Eigen::VectorXd& out, const Eigen::VectorXd& z, const Eigen::VectorXd& u) 
    {
        std::lock_guard<std::mutex> guard(mtx_);
        // we must know u for init of B
        if(first_call_) B_ = Eigen::MatrixXd::Zero(P_.rows(), u.size());
        preupdate();
        x_ = A_ * x_ + B_ * u;
        updateStep(z);
        out = x_;
    }
    
    double Kalman::getDt() 
    {
        return dt_;
    }
}