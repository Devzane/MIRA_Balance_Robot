#include "FuzzyController.h"
#include "Config.h"

FuzzyController::FuzzyController() {
  // Standard diagonal fuzzy-PD rule table. Reading it: if the robot is
  // leaning hard one way (error = PB) AND still rotating that way
  // (errorRate = PB), the output should be a hard correction (PB).
  // If it's leaning one way but already rotating back (error = PB,
  // errorRate = NB), the correction can be gentler (Z) since it's
  // already recovering on its own.
  Level table[NUM_LEVELS][NUM_LEVELS] = {
    /*              errorRate: NB   NS    Z    PS   PB   */
    /* error NB */          { NB,  NB,  NB,  NS,   Z },
    /* error NS */          { NB,  NB,  NS,   Z,  PS },
    /* error Z  */          { NB,  NS,   Z,  PS,  PB },
    /* error PS */          { NS,   Z,  PS,  PB,  PB },
    /* error PB */          {  Z,  PS,  PB,  PB,  PB },
  };
  for (int i = 0; i < NUM_LEVELS; i++)
    for (int j = 0; j < NUM_LEVELS; j++)
      ruleTable[i][j] = table[i][j];

  outputSingleton[NB] = -FUZZY_OUTPUT_MAX;
  outputSingleton[NS] = -FUZZY_OUTPUT_MAX * 0.5f;
  outputSingleton[Z]  = 0.0f;
  outputSingleton[PS] =  FUZZY_OUTPUT_MAX * 0.5f;
  outputSingleton[PB] =  FUZZY_OUTPUT_MAX;
}

float FuzzyController::triangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

void FuzzyController::fuzzify(float x, float range, float membership[NUM_LEVELS]) {
  // Clamp to the defined universe so inputs beyond it still saturate
  // at "fully NB" or "fully PB" instead of returning all zeros.
  if (x < -range) x = -range;
  if (x >  range) x =  range;

  float half = range * 0.5f;

  // Centers: -range, -half, 0, +half, +range. Each set's "shoulders"
  // extend past the universe edge so NB/PB saturate to 1.0 at the ends.
  membership[NB] = triangular(x, -range - 1, -range, -half);
  membership[NS] = triangular(x, -range, -half, 0);
  membership[Z]  = triangular(x, -half, 0, half);
  membership[PS] = triangular(x, 0, half, range);
  membership[PB] = triangular(x, half, range, range + 1);
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
      float strength = min(mErr[i], mRate[j]);
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
