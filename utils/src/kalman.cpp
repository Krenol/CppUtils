#include "kalman.hpp"

namespace utils
{    
    template<typename T>
    void Kalman<T>::setDt() 
    {
        auto now = std::chrono::steady_clock::now();
        if(first_call_){
            dt_ = 1;
            first_call_ = false;
        } else {
            dt_ = std::chrono::duration_cast<T>(now - last_call_).count();
        }
        last_call_ = now;
    }

    template<typename T>
    void Kalman<T>::init() 
    {
        
        //init empty A and Q
        A_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        Q_ = Eigen::MatrixXd::Zero(P_.rows(), P_.cols());

        // init Eigenmatrix
        I_= Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        I_.setIdentity();
    }
    
    template<typename T>
    void Kalman<T>::preupdate() 
    {
        setDt();
        updateA();
        updateB();
        updateC();
        updateQ();
        updateR();
    }
    
    template<typename T>
    void Kalman<T>::updateStep(const Eigen::VectorXd& z) 
    {
        auto CT = C_.transpose();
        P_ = A_ * P_ * A_.transpose() + Q_;
        K_ = P_ * CT * (C_ * P_ * CT + R_).inverse();
        x_ = x_ + K_ * (z - C_ * x_);
        P_ = (I_ - K_ * C_) * P_;
    }
    
    template<typename T>
    void Kalman<T>::updateA() 
    {
        
    }
    
    template<typename T>
    void Kalman<T>::updateB() 
    {
        
    }
    
    template<typename T>
    void Kalman<T>::updateC() 
    {
        
    }
    
    template<typename T>
    void Kalman<T>::updateQ() 
    {

    }
    
    template<typename T>
    void Kalman<T>::updateR() 
    {
        
    }
    
    Kalman::Kalman(const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B,
                const Eigen::MatrixXd& C,
                const Eigen::MatrixXd& P_0,
                const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R,
                const Eigen::VectorXd& x0) : 
                    C_{C}, P_{P_0}, R_{R}, x_{x0}, A_{A}, B_{B}, Q_{Q}
    {
        // init Eigenmatrix
        I_= Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        I_.setIdentity();
    }

    Kalman::Kalman(const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B,
                const Eigen::MatrixXd& C,
                const Eigen::MatrixXd& P_0,
                const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R) : 
                    C_{C}, P_{P_0}, R_{R}, x_{x0}, A_{A}, B_{B}, Q_{Q}
    {
        // init Eigenmatrix
        I_= Eigen::MatrixXd::Zero(P_.rows(), P_.cols());
        I_.setIdentity();
        x_ = Eigen::VectorXd::Zero(A_.rows());
    }

    template<typename T>
    Kalman<T>::Kalman(const Eigen::MatrixXd& C, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& R,
                    const Eigen::VectorXd& x0) : 
                    C_{C}, P_{P_0}, R_{R}, x_{x0}
    {   
        init();
    }
    
    template<typename T>
    Kalman<T>::Kalman(
                    const Eigen::MatrixXd& C, 
                    const Eigen::MatrixXd& P_0, 
                    const Eigen::MatrixXd& R) :
                    C_{C}, P_{P_0}, R_{R}
    {
        init();
        x_ = Eigen::VectorXd::Zero(A_.rows());
    }

    template<typename T>
    void Kalman<T>::predict(Eigen::VectorXd& out, const Eigen::VectorXd& z) 
    {
        std::lock_guard<std::mutex> guard(mtx_);
        preupdate();
        x_ = A_ * x_;
        updateStep(z);
        out = x_;
    } 
    
    template<typename T>
    void Kalman<T>::predict(Eigen::VectorXd& out, const Eigen::VectorXd& z, const Eigen::VectorXd& u) 
    {
        std::lock_guard<std::mutex> guard(mtx_);
        // we must know u for init of B
        if(first_call_) B_ = Eigen::MatrixXd::Zero(P_.rows(), u.size());
        preupdate();
        x_ = A_ * x_ + B_ * u;
        updateStep(z);
        out = x_;
    }
    
    template<typename T>
    double Kalman<T>::getDt() 
    {
        return dt_;
    }

    template class Kalman<std::chrono::hours>;
    template class Kalman<std::chrono::minutes>;
    template class Kalman<std::chrono::seconds>;
    template class Kalman<std::chrono::milliseconds>;
    template class Kalman<std::chrono::microseconds>;
    template class Kalman<std::chrono::nanoseconds>;
}