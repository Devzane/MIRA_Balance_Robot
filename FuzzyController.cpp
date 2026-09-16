#include <Arduino.h>
#include <cmath> // Added to securely use fmin() for the float math
#include "FuzzyController.h"
#include "Config.h"


FuzzyController::FuzzyController() {
  // Standard diagonal fuzzy-PD rule table. Reading it: if the robot is
  // leaning hard one way (error = POS_BIG) AND still rotating that way
  // (errorRate = POS_BIG), the output should be a hard correction (POS_BIG).
  // If it's leaning one way but already rotating back (error = POS_BIG,
  // errorRate = NEG_BIG), the correction can be gentler (ZERO) since it's
  // already recovering on its own.
  Level table[NUM_LEVELS][NUM_LEVELS] = {
    /*              errorRate: NEG_BIG    NEG_SMALL  ZERO       POS_SMALL  POS_BIG  */
    /* error NEG_BIG */      { NEG_BIG,   NEG_BIG,   NEG_BIG,   NEG_SMALL, ZERO },
    /* error NEG_SMALL */    { NEG_BIG,   NEG_BIG,   NEG_SMALL, ZERO,      POS_SMALL },
    /* error ZERO  */        { NEG_BIG,   NEG_SMALL, ZERO,      POS_SMALL, POS_BIG },
    /* error POS_SMALL */    { NEG_SMALL, ZERO,      POS_SMALL, POS_BIG,   POS_BIG },
    /* error POS_BIG */      { ZERO,      POS_SMALL, POS_BIG,   POS_BIG,   POS_BIG },
  };
  for (int i = 0; i < NUM_LEVELS; i++)
    for (int j = 0; j < NUM_LEVELS; j++)
      ruleTable[i][j] = table[i][j];

  outputSingleton[NEG_BIG]   = -FUZZY_OUTPUT_MAX;
  outputSingleton[NEG_SMALL] = -FUZZY_OUTPUT_MAX * 0.5f;
  outputSingleton[ZERO]      = 0.0f;
  outputSingleton[POS_SMALL] =  FUZZY_OUTPUT_MAX * 0.5f;
  outputSingleton[POS_BIG]   =  FUZZY_OUTPUT_MAX;
}

float FuzzyController::triangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

void FuzzyController::fuzzify(float x, float range, float membership[NUM_LEVELS]) {
  // Clamp to the defined universe so inputs beyond it still saturate
  // at "fully NEG_BIG" or "fully POS_BIG" instead of returning all zeros.
  if (x < -range) x = -range;
  if (x >  range) x =  range;

  float half = range * 0.5f;

  // Centers: -range, -half, 0, +half, +range. Each set's "shoulders"
  // extend past the universe edge so NEG_BIG/POS_BIG saturate to 1.0 at the ends.
  membership[NEG_BIG]   = triangular(x, -range - 1, -range, -half);
  membership[NEG_SMALL] = triangular(x, -range, -half, 0);
  membership[ZERO]      = triangular(x, -half, 0, half);
  membership[POS_SMALL] = triangular(x, 0, half, range);
  membership[POS_BIG]   = triangular(x, half, range, range + 1);
}

float FuzzyController::compute(float error, float errorRate) {
  float mErr[NUM_LEVELS];
  float mRate[NUM_LEVELS];
  fuzzify(error, FUZZY_ERROR_RANGE, mErr);
  fuzzify(errorRate, FUZZY_ERROR_RATE_RANGE, mRate);

  float numerator = 0.0f;
  float denominator = 0.0f;

  for (int i = 0; i < NUM_LEVELS; i++) {
    if (mErr[i] <= 0.0f) continue;
    for (int j = 0; j < NUM_LEVELS; j++) {
      if (mRate[j] <= 0.0f) continue;

      // Mamdani AND = min of the two membership degrees (rule firing strength)
      // Using fmin() safely handles the float comparison without scope errors
      float strength = fmin(mErr[i], mRate[j]);
      Level outLevel = ruleTable[i][j];

      numerator   += strength * outputSingleton[outLevel];
      denominator += strength;
    }
  }

  if (denominator <= 0.0001f) return 0.0f; // no rule fired (shouldn't normally happen)

  float output = numerator / denominator;

  if (output >  FUZZY_OUTPUT_MAX) output =  FUZZY_OUTPUT_MAX;
  if (output < -FUZZY_OUTPUT_MAX) output = -FUZZY_OUTPUT_MAX;

  return output;
}