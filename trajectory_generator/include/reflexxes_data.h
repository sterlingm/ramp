#ifndef REFLEXXES_DATA
#define REFLEXXES_DATA

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

struct ReflexxesData {
  ReflexxesAPI *rml;
  RMLPositionInputParameters *inputParameters;
  RMLPositionOutputParameters *outputParameters;
  RMLPositionFlags flags;
  unsigned int NUMBER_OF_DOFS;
  int resultValue;

  const ReflexxesData clone() {return *this;}
};

#endif
