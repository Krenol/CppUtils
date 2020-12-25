#include <Eigen/Dense>
#include <mutex>
#include <chrono> 
#include <atomic>

#ifndef UTILS_KALMAN_H
#define UTILS_KALMAN_H

namespace utils {
    static void PRINT_MATRIX_SIZES(const Eigen::MatrixXd& P_0, const Eigen::VectorXd& z, const Eigen::VectorXd& u);
    class Kalman {
    private:
        Eigen::VectorXd x_;
        std::mutex mtx_;
        std::chrono::steady_clock::time_point last_call_;
        Eigen::MatrixXd C_, P_, R_, K_, I_;
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
         * Update the matrixes and calculate the values based on new x
         * @param z The measured / calculated system state
         */
        void updateStep(const Eigen::VectorXd& z, const Eigen::VectorXd& u);

    protected:
        Eigen::MatrixXd A_, Q_, B_;
        

        /**
         * Method to update the A matrix (state transition model) of the Kalman Filter
         */
        virtual void updateA() = 0;

        /**
         * Method to update the B matrix (control-input) of the Kalman Filter
         */
        virtual void updateB() = 0;

        /**
         * Method to update the Q matrix (process noise covariance) of the Kalman Filter
         */
        virtual void updateQ() = 0;

    public:
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
         * @param z The observed state
         * @returns Predicted system state
         *
         */
        const Eigen::VectorXd& predict(const Eigen::VectorXd& z);

        /**
         * Predict the state of the system based on control input
         * @param z The observed state
         * @param u control input
         * @returns Predicted system state
         */
        const Eigen::VectorXd& predict(const Eigen::VectorXd& z, const Eigen::VectorXd& u);
        
        /**
         * Method to get dt value
         * @returns delta t between last steps
         */
        double getDt();
    };
}

#endif