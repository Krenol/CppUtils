#include "kalmann.hpp"
namespace utils
{
    void Kalmann::init() 
    {
        auto size = A_.rows();
        I_(size, size);
        I_.setIdentity();
    }
    
    void Kalmann::updateStep(const Eigen::VectorXd& z) 
    {
        P_ = A_ * P_ * A_.transpose() + Q_;
        K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
        x_ += K_ * (z - H_ * x_);
        P_ = (I_ - K_ * H_) * P_;
    }

    Kalmann::Kalmann(const Eigen::MatrixXd& A, 
                    const Eigen::MatrixXd& H, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& Q, 
                    const Eigen::MatrixXd& R,
                    const Eigen::VectorXd& x0) : 
                    A_{A}, H_{H}, P_{P_0}, Q_{Q}, R_{R}, x_{x0}
    {
        init();
    }
    
    Kalmann::Kalmann(const Eigen::MatrixXd& A, 
                    const Eigen::MatrixXd& H, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& Q, 
                    const Eigen::MatrixXd& R) : 
                    A_{A},H_{H}, P_{P_0}, Q_{Q}, R_{R}, x_{A.rows()}
    {
        x_.setZero();
        init();
    }


    const Eigen::VectorXd& Kalmann::predict(const Eigen::VectorXd& z) 
    {
        x_ = A_ * x_;
        updateStep(z);
    } 
    
    const Eigen::VectorXd& Kalmann::predict(const Eigen::MatrixXd& B, const Eigen::VectorXd& u) 
    {
        x_ = A_ * x_ + B * u;
        updateStep(x_);
    }
}