#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

// Mamdani-style fuzzy PD controller.
//
// Inputs : error (setpoint - filtered angle), error rate (angular velocity)
// Output : motor drive signal, -255..255 (sign = direction, magnitude = PWM)
//
// Each input has 5 linguistic levels: NB, NS, Z, PS, PB
// (Negative Big, Negative Small, Zero, Positive Small, Positive Big).
// The output uses 5 matching singleton values instead of full triangular
// output sets -- this keeps defuzzification to a simple weighted average
// instead of a numerically-integrated centroid, which matters on a
// microcontroller running a 200Hz control loop. This is a very common,
// accepted simplification (sometimes called "Mamdani with singleton
// consequents") and won't be visibly different from a full centroid
// version for a control task like this.

class FuzzyController {
public:
  FuzzyController();

  // error      : degrees, setpoint - angle
  // errorRate  : degrees/sec
  // Returns the motor drive value, -255..255.
  float compute(float error, float errorRate);

private:
  enum Level { NB = 0, NS, Z, PS, PB, NUM_LEVELS };

  // Triangular membership function: 0 outside [a,c], peaks at b.
  float triangular(float x, float a, float b, float c);

  // Fills membership[NUM_LEVELS] with this variable's degree of
  // membership in each of the 5 levels, for a given crisp input and range.
  void fuzzify(float x, float range, float membership[NUM_LEVELS]);

  // The 5x5 rule table: ruleTable[errorLevel][errorRateLevel] = outputLevel
  Level ruleTable[NUM_LEVELS][NUM_LEVELS];

  // Crisp output value assigned to each output level (singleton consequent)
  float outputSingleton[NUM_LEVELS];
};

#endif
