#include "trackers.h"

/*
Namespace for Tracker-specific classes and data types
*/
namespace MultiObjectTrackers {

    /*
    Base Tracker Class
    S = # of spatial dimensions
    D = dimension of motion (1 = position, 2 = position + velocity)
    */
    template<size_t S, size_t D>
    class Tracker 
    {
        public:
            // Default Constructor
            Tracker() : _stateDim(S*D) {};
            Tracker(ros::Time timestamp) : _stateDim(S*D), _lastUpdated(timestamp) {};

            // Accessors
            int StateDim() {return _stateDim;};

            // Mutators
            void LastUpdated(ros::Time timestamp) { _lastUpdated = timestamp;};

        protected:
            int         _stateDim;      // Number of state variables to track
            ros::Time   _lastUpdated;   // ROS timestamp when state was last updated
    };

    /*
    Gaussian Mixture - Probability Hypothesis Density (GM-PHD) class
    */
    template<size_t S, size_t D>
    class GmPhdFilter : Tracker<S,D>
    {
        public:
            // Default constructor
            GmPhdFilter() {};

            // Construct with initial state at a given time
            GmPhdFilter(GaussianDataTypes::GaussianMixture<S*D> initState) : Tracker<S,D>(ros::Time::now()), _state(initState) {};

        private:
            GaussianDataTypes::GaussianMixture<S*D> _state;

    };


}