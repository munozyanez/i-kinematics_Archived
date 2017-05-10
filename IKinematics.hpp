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

    long RightLegFwdKin(const std::vector<double> &q, std::vector<double> &pose);
    long LeftLegFwdKin(const std::vector<double> &q, std::vector<double> &pose);

    long LeftLegInvKin(const std::vector<double> &pose, std::vector<double> &q);
    long RightLegInvKin(const std::vector<double> &pose, std::vector<double> &q);
private:

kinLeftLeg::IkReal j;
double rightLegFParam,leftLegFParam;
ikfast::IkSolutionList<double> rightLegIKList, leftLegIKList;
ikfast::IkSolution<double> *rightLegIKSol, * leftLegIKSol;
};

#endif // IKINEMATICS_H
