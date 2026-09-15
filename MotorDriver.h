#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// Drives both motors on the L298N from a single signed correction value
// (both wheels get the same speed/direction, since balancing is a
// single-axis pitch problem -- turning logic can be added later as a
// separate offset once balancing is stable).

void motorInit();

// speed: -255..255. Positive = one direction, negative = the other.
// Call this every loop iteration with the fuzzy controller's output.
void motorSetSpeed(float speed);

// Immediately stops both motors. Used when the robot has fallen past
// FALL_ANGLE_LIMIT, so it doesn't sit there fighting a lost cause.
void motorStop();

#endif
