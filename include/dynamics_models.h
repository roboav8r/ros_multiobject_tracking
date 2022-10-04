#ifndef DYNAMICS_MODELS_H_
#define DYNAMICS_MODELS_H_

#include <Eigen/Dense>

/*
Namespace for dynamics model classes and data types
*/
namespace DynamicsModels {

    // Available dynamics model types
    enum DynamicsTypes {linear};

    /*
    Base Linear Dynamics Class
    TODO: Instead of 2D, make a general template
    */
    class LinearDynamics2D
    {
        public:
            /*
            Constructors
            */
            LinearDynamics2D() : _dt(0.1), _procNoise(0.1) {
                _transMatrix.setIdentity();
                _covMatrix.setZero();
                _updateTransMatrix();
                _updateCovMatrix();
            };

            LinearDynamics2D(double deltaT, double sigmaP) : _dt(deltaT), _procNoise(sigmaP) {
                _transMatrix.setIdentity();
                _covMatrix.setZero();
                _updateTransMatrix();
                _updateCovMatrix();
            };

            /* 
            Mutators
            */
            void SetTimestep(double deltaT) { 
                this->_dt=deltaT;
            };

            void ProbSurvival(double pS) {this->_pSurvival = pS; };

            void ProcNoise(double sigmaP) {this->_procNoise = sigmaP; };

            /*
             Accessors
            */
            // Get probability of object survival
            double ProbSurvival() {return _pSurvival; };
            
            // Get transition matrix for constant timestep
            Eigen::Matrix<double, 4, 4> TransMatrix() {return _transMatrix;}
            
            // Get transition matrix for variable timestep
            Eigen::Matrix<double, 4, 4> TransMatrix(double deltaT) {
                SetTimestep(deltaT);
                _updateTransMatrix();
                return _transMatrix;
            }
            
            // Get covariance matrix for constant timestep
            Eigen::Matrix<double, 4, 4> CovMatrix() {return _covMatrix;}
            
            // Get covariance matrix for variable timestep
            Eigen::Matrix<double, 4, 4> CovMatrix(double deltaT) {
                SetTimestep(deltaT);
                _updateCovMatrix();
                return _covMatrix;
            }

        private:
            double _pSurvival;      // Object survival probability
            double _dt;             // Timestep
            double _procNoise;      // Process noise, sigma_p
            Eigen::Matrix<double, 4, 4> _transMatrix;
            Eigen::Matrix<double, 4, 4> _covMatrix;

            // TODO: replace _update functions with more clean matrix algebra
            void _updateTransMatrix() {
                _transMatrix(0,2) = _dt;
                _transMatrix(1,3) = _dt;
            }
            void _updateCovMatrix(){
                _covMatrix(0,0) = pow(_dt,4)/4;
                _covMatrix(1,1) = pow(_dt,4)/4;
                _covMatrix(0,2) = pow(_dt,3)/2;
                _covMatrix(1,3) = pow(_dt,3)/2;
                _covMatrix(2,0) = pow(_dt,3)/2;
                _covMatrix(3,1) = pow(_dt,3)/2;
                _covMatrix(2,2) = pow(_dt,2);
                _covMatrix(3,3) = pow(_dt,2);
                _covMatrix*=pow(_procNoise,2);                
            }

    };


}


#endif // TRACKERS_H_