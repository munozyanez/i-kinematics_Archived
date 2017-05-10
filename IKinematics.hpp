#ifndef IKINEMATICS_H
#define IKINEMATICS_H


#include <vector>
#include <fstream>      // std::fstream
#include <algorithm>    // std::min_element, std::max_element

#include <iostream>

#define IKFAST_HAS_LIBRARY

#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE kinLeftLeg
#include "ikfast.h"
#undef IKFAST_NAMESPACE

#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE kinRightLeg
#include "ikfast.h"
#undef IKFAST_NAMESPACE

class IKinematics
{
public:
    IKinematics();



private:

kinLeftLeg::IkReal j;

};

#endif // IKINEMATICS_H
