#ifndef MOTION_PLAN_SCURVE_H
#define MOTION_PLAN_SCURVE_H

#include "generic_block.h"

class MotionPlanSCURVE : public Block {
    BLOCK_INPUT(float, position);       // input for the target position
    BLOCK_INPUT(float, max_vel);        // input for the maximum desired velocity
    BLOCK_INPUT(float, dt);             // input for the timestep for instantaneous values
    BLOCK_OUTPUT(float, current_pos);   // output for the current position
    BLOCK_OUTPUT(float, current_vel);   // output for the current velocity
    BLOCK_OUTPUT(float, current_accel); // output for the current acceleration

    public:
    
};

#endif MOTION_PLAN_SCURVE_H