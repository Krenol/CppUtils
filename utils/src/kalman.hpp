#include <Eigen/Dense>
#include <mutex>
#include <chrono> 
#include <atomic>
#include <iostream>

#ifndef UTILS_KALMAN_H
#define UTILS_KALMAN_H

namespace utils {

    //https://stackoverflow.com/a/28876046
    template <typename T>
    struct is_chrono_duration
    {
        static constexpr bool value = false;
    };

    template <typename Rep, typename Period>
    struct is_chrono_duration<std::chrono::duration<Rep, Period>>
    {
        static constexpr bool value = true;
    };

    static void PRINT_MATRIX_SIZES(const Eigen::MatrixXd& P_0, const Eigen::VectorXd& z, const Eigen::VectorXd& u) {
        int m = z.size(), l = u.size(), n = P_0.rows();
        std::cout << "\n\n\n--------------------------" << std::endl;
        std::cout << "m = z.size() = " << m << std::endl;
        std::cout << "l = u.size() = " << l << std::endl;
        std::cout << "n = P.rows() = " << n << std::endl;
        std::cout << "A(nxn) = " << n << "x" << n << std::endl; 
        std::cout << "B(nxl) = " << n << "x" << l << std::endl; 
        std::cout << "C(mxn) = " << m << "x" << n << std::endl; 
        std::cout << "Q(nxn) = " << n << "x" << n << std::endl; 
        std::cout << "R(mxm) = " << m << "x" << m << std::endl; 
        std::cout << "K(nxm) = " << n << "x" << m << std::endl; 
        std::cout << "x(n) = " << n << std::endl; 
        std::cout << "--------------------------\n\n\n" << std::endl;
    }

    static void PRINT_MATRIX_SIZES(const Eigen::MatrixXd& P_0, const Eigen::VectorXd& z) {
        int m = z.size(), n = P_0.rows();
        std::cout << "\n\n\n--------------------------" << std::endl;
        std::cout << "m = z.size() = " << m << std::endl;
        std::cout << "n = P.rows() = " << n << std::endl;
        std::cout << "A(nxn) = " << n << "x" << n << std::endl; 
        std::cout << "C(mxn) = " << m << "x" << n << std::endl; 
        std::cout << "Q(nxn) = " << n << "x" << n << std::endl; 
        std::cout << "R(mxm) = " << m << "x" << m << std::endl; 
        std::cout << "K(nxm) = " << n << "x" << m << std::endl; 
        std::cout << "x(n) = " << n << std::endl; 
        std::cout << "--------------------------\n\n\n" << std::endl;
    }
    template <typename T = std::chrono::milliseconds>
    class Kalman {
        static_assert(is_chrono_duration<T>::value, "T not derived from std::chrono::duration");
    private:
        Eigen::VectorXd x_;
        std::mutex mtx_;
        std::chrono::steady_clock::time_point last_call_;
        Eigen::MatrixXd P_, K_, I_;
        std::atomic<double> dt_{ 0 };
        std::atomic_bool first_call_ {true};

        /*
        Method to set dt_ for the integral / derivate term
        */
        void setDt();

        /**
         * Initialize the component
         */
        void init();
        
        /**
         * Update all matrices and dt
         */
        void preupdate();


        /**
         * Update the matrixes and calculate the values based on new x
         * @param z The measured / calculated system state
         */
        void updateStep(const Eigen::VectorXd& z);

    protected:
        Eigen::MatrixXd A_, Q_, B_, R_, C_;
        

        /**
         * Method to update the A matrix (state transition model) of the Kalman Filter
         */
        virtual void updateA();

        /**
         * Method to update the B matrix (control-input) of the Kalman Filter
         */
        virtual void updateB();

        /**
         * Method to update the C matrix (uutput matrix) of the Kalman Filter
         */
        virtual void updateC();

        /**
         * Method to update the Q matrix (process noise covariance) of the Kalman Filter
         */
        virtual void updateQ();

        /**
         * Method to update the R matrix (measurement noise covariance) of the Kalman Filter
         */
        virtual void updateR();

    public:
        /**
         * Constructor
         * @param A State transition model
         * @param B Control input matrix
         * @param C Output matrix
         * @param P_0 Initial error covariance estiamtion
         * @param Q Process noise covariance
         * @param R Measurement noise covariance
         * @param x0 Inital state guess
         */
        Kalman(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& P_0,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::VectorXd& x0
        );

        /**
         * Constructor
         * @param A State transition model
         * @param B Control input matrix
         * @param C Output matrix
         * @param P_0 Initial error covariance estiamtion
         * @param Q Process noise covariance
         * @param R Measurement noise covariance
         */
        Kalman(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& P_0,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R
        );


        /**
         * Constructor
         *
         * @param C Output matrix
         * @param P_0 Initial error covariance estiamtion
         * @param R Measurement noise covariance
         * @param x0 Inital state guess
         */
        Kalman(
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& P_0,
            const Eigen::MatrixXd& R,
            const Eigen::VectorXd& x0
        );

        /**
         * Constructor
         *
         * @param C Output matrix
         * @param P_0 Initial error covariance estiamtion
         * @param R Measurement noise covariance
         */
        Kalman(
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& P_0,
            const Eigen::MatrixXd& R
        );

        /**
         * Predict the state of the system based on observed state
         * @param out Vector to store prediction
         * @param z The observed state
         *
         */
        void predict(Eigen::VectorXd& out, const Eigen::VectorXd& z);

        /**
         * Predict the state of the system based on control input
         * @param out Vector to store prediction
         * @param z The observed state
         * @param u control input
         */
        void predict(Eigen::VectorXd& out, const Eigen::VectorXd& z, const Eigen::VectorXd& u);
        
        /**
         * Method to get dt value
         * @returns delta t between last steps
         */
        double getDt();
    };
}

#endif