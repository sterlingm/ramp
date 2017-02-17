#include "circle.h"


/************************************************************************
      BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor

CircleFit::CircleFit()
{
  a=0.; b=0.; r=1.; s=0.; i=0; j=0;
}

// Constructor with assignment of the circle parameters only

CircleFit::CircleFit(reals aa, reals bb, reals rr)
{
  a=aa; b=bb; r=rr;
}

// Printing routine

void CircleFit::print(void)
{
  std::cout << std::endl;
  std::cout << std::setprecision(10) << "center (" <<a <<","<< b <<")  radius "
     << r << "  sigma " << s << "  gradient " << g << "  iter "<< i << "  inner " << j << std::endl;
}
