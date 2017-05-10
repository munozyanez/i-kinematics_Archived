#include "IKinematics.hpp"
#include <iostream>

IKinematics::IKinematics()
{


}


long IKinematics::LeftLegFwdKin(const std::vector<double>& q, std::vector<double> & pose)
{
    kinLeftLeg::ComputeFk(q.data(),&pose[0], &pose[3]);

    return 0;
}

long IKinematics::RightLegFwdKin(const std::vector<double>& q, std::vector<double> & pose)
{
    kinRightLeg::ComputeFk(q.data(),&pose[0], &pose[3]);

    return 0;
}

long IKinematics::LeftLegInvKin(const std::vector<double> & pose, std::vector<double>& q)
{

    //CAREFUL THIS

    kinLeftLeg::ComputeIk(&pose[0], &pose[3], 0, leftLegIKList);
    //leftLegIKSol = (ikfast::IkSolution<double>*)&leftLegIKList.GetSolution(0);
    return 0;
}

long IKinematics::RightLegInvKin(const std::vector<double> & pose, std::vector<double>& q)
{

    kinRightLeg::ComputeIk(&pose[0], &pose[3], &rightLegFParam, rightLegIKList);
    return 0;
}
