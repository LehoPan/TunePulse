#ifndef MOTION_PLAN_TRAPEZOIDAL_H
#define MOTION_PLAN_TRAPEZOIDAL_H

#include "generic_block.h"

class MotionPlanTRAPEZOIDAL : public Block {
    BLOCK_INPUT(float, position);       // input for the target position
    BLOCK_INPUT(float, max_vel);        // input for the maximum desired velocity
    BLOCK_INPUT(float, max_accel);      // input for the maximum desired acceleration
    BLOCK_INPUT(float, dt);             // input for the timestep for instantaneous values
    BLOCK_OUTPUT(float, current_pos);   // output for the current position
    BLOCK_OUTPUT(float, current_vel);   // output for the current velocity
    BLOCK_OUTPUT(float, current_accel); // output for the current acceleration
    BLOCK_OUTPUT(float, current_jerk);  // output for the current jerk

    public:

        /**
         * @brief Updates the instantaneous position, velocity, acceleration, and jerk values.
         * Updates the instantaneous position, velocity, acceleration, and jerk values per timestep inputted.
         */
        void tick() override;
};



#endif MOTION_PLAN_TRAPEZOIDAL_H