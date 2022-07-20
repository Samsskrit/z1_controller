#ifndef ENDCIRCLETRAJ_H
#define ENDCIRCLETRAJ_H

#include "model/ArmDynKineModel.h"
#include "trajectory/EndHomoTraj.h"

class EndCircleTraj: public EndHomoTraj{
public:
    EndCircleTraj(CtrlComponents *ctrlComp);
    EndCircleTraj(ArmDynKineModel *armModel);
    ~EndCircleTraj(){}

    void setEndRoundTraj(HomoMat startHomo, Vec3 axisPointFromInit, 
            Vec3 axisDirection, double maxSpeed, double angle,
            bool keepOrientation = true);
    void setEndRoundTraj(std::string stateName, Vec3 axisPointFromInit, 
            Vec3 axisDirection, double maxSpeed, double angle,
            bool keepOrientation = true);
private:
    bool _getEndTraj(HomoMat &homo, Vec6 &twist);
    Vec3 _center;
    double _radius;
    HomoMat _centerHomo;
    HomoMat _initHomoToCenter;
    RotMat _initOri;
    double _maxSpeed, _goalAngle, _speed, _angle;
    Vec3 _omegaAxis;

    bool _keepOrientation;
};

#endif  // ENDCIRCLETRAJ_H