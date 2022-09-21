#ifndef TRACKERS_H_
#define TRACKERS_H_

#include "trackers.h"

/*
Namespace for Tracker-specific classes and data types
*/
namespace MultiObjectTrackers {

    // Available tracker types
    enum TrackerTypes {gm_phd};

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
            Tracker() : _state_dim(S*D) {};

            // Accessors
            int StateDim() {return _state_dim;};

        protected:
            // Number of state variables to track
            int _state_dim;
    };

    /*
    Gaussian Mixture - Probability Hypothesis Density (GM-PHD) class
    */
    template<size_t S, size_t D>
    class GmPhdFilter: public Tracker<S,D>
    {
        public:
            GmPhdFilter(){};
        private:

    };


}


#endif // TRACKERS_H_