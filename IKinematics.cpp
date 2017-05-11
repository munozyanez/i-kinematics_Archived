#include "IKinematics.hpp"
#include <iostream>

IKinematics::IKinematics()
{
leftLegFreeQVals.resize(kinLeftLeg::GetNumFreeParameters() );
rightLegFreeQVals.resize(kinRightLeg::GetNumFreeParameters() );

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

    kinLeftLeg::ComputeIk(&pose[0], &pose[3], 0, leftLegIKList);
    leftLegIKSol = &leftLegIKList.GetSolution(0);
    for (long i = 0; i< leftLegFreeQVals.size(); i++)
    {
        std::cout << leftLegFreeQVals[i] << "," << std::endl;
    }
    leftLegIKSol->GetSolution(q,leftLegFreeQVals);
    return 0;
}

long IKinematics::RightLegInvKin(const std::vector<double> & pose, std::vector<double>& q)
{

    kinRightLeg::ComputeIk(&pose[0], &pose[3], &rightLegFParam, rightLegIKList);
    return 0;
}
