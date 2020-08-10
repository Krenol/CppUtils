#include <Eigen/Dense>

#ifndef UTILS_KALMANN_H
#define UTILS_KALMANN_H

namespace utils {

    
    class Kalmann {
        private:
            Eigen::MatrixXd A_, H_, P_, Q_, R_, K_, I_;
            Eigen::VectorXd x_;

            /**
             * Initialize the component
             */

            void init();

            /**
             * Update the matrixes and calculate the values based on new x
             * @param z The measured / calculated system state
             */
            void updateStep(const Eigen::VectorXd& z);

        public:
            /** 
             * Constructor
             * 
             * @param A System dynamics matrix
             * @param H Output matrix
             * @param P_0 Initial error covariance estiamtion
             * @param Q Process noise covariance
             * @param R Measurement noise covariance
             * @param x0: Inital state guess
             */
            Kalmann(
                const Eigen::MatrixXd& A, 
                const Eigen::MatrixXd& H, 
                const Eigen::MatrixXd& P_0, 
                const Eigen::MatrixXd& Q, 
                const Eigen::MatrixXd& R,
                const Eigen::VectorXd& x0
            );

            /** 
             * Constructor
             * 
             * @param A System dynamics matrix
             * @param H Output matrix
             * @param P_0 Initial error covariance estiamtion
             * @param Q Process noise covariance
             * @param R Measurement noise covariance
             */
            Kalmann(
                const Eigen::MatrixXd& A, 
                const Eigen::MatrixXd& H, 
                const Eigen::MatrixXd& P_0, 
                const Eigen::MatrixXd& Q, 
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
             * @param B the control-input model
             * @param u control input
             * @returns Predicted system state
             */
            const Eigen::VectorXd& predict(const Eigen::MatrixXd& B, const Eigen::VectorXd& u);

    };
}

#endif