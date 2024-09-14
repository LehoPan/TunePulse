#ifndef MOTION_PLAN_TRAPEZOIDAL_H
#define MOTION_PLAN_TRAPEZOIDAL_H

#include "generic_block.h"

// struct for inputting vel/accel/jerk limiters
struct inputTarget {
    float pos;   // target distance
    float dt;           // desired timestep
    float vel;      // maxiimum velocity
    float accel;    // maximum acceleration
    float jerk;     // maximum jerk
};

// struct for inputting the current instantaneous values
struct outputInstant {
    float pos = 0;      // current position
    float time = 0;     // instantaneous time
    float vel = 0;      // instantaneous velocity
    float accel = 0;    // instantaneous acceleration
    float jerk = 0;     // instantaneous jerk
};

/**
 * @brief Creates a Triangular motion profile.
 * This class outputs values to plot a triangular motion profile.
 * @param tick Update the motion calculations.
 * @param target given target limiters for vel/accel/jerk
 */
class MotionPlanTrapezoidal : public Block {
    BLOCK_INPUT(struct inputTarget, target);        // input limiters for vel/accel/jerk
    BLOCK_OUTPUT(float, current_pos);               // output for the current position
    BLOCK_OUTPUT(float, current_vel);               // output for the current velocity
    BLOCK_OUTPUT(float, current_accel);             // output for the current acceleration
    BLOCK_OUTPUT(float, current_jerk);              // output for the current jerk
    BLOCK_OUTPUT(float, current_time);              // output for the current time elapsed

    private:
        outputInstant instant;

    public:
        /**
         * @brief Trapezoidal motion plan constructor.
         * @param target maximum limits to vel/accel/jerk

         */
        MotionPlanTrapezoidal(const struct inputTarget target)
            : target_(target){}

        /**
         * @brief Updates the instantaneous position, velocity, acceleration, and jerk values.
         * Updates the instantaneous position, velocity, acceleration, and jerk values per timestep inputted.
         */
        void tick() override;
};

void MotionPlanTrapezoidal::tick() {
    if (instant.pos < target_.pos) {
        // increment time
        instant.time += target_.dt;

        // changes velocity, checks if it needs to slow down or speed up by if its halway to the target position
        if (instant.pos <= target_.pos / 2) {
            instant.vel = (instant.vel < target_.vel) ? instant.vel + (target_.accel * target_.dt) : target_.vel;
        } else {
            // calculates how long much distance it would take to decelerate from maximum velocity, and decelerates if it is under the distance
            instant.vel = ((target_.pos - instant.pos) < (target_.vel * (target_.vel / target_.accel)) / 2) ? instant.vel - (target_.accel * target_.dt) : target_.vel;
        }
        
        instant.pos += instant.vel * target_.dt;
        instant.accel = target_.accel;
        instant.jerk = 0;

    }
    // output values
    current_pos_ = instant.pos;
    current_vel_ = instant.vel;
    current_accel_ = instant.accel;
    current_jerk_ = instant.jerk;
    current_time_ = instant.time;
}





#endif MOTION_PLAN_TRAPEZOIDAL_H