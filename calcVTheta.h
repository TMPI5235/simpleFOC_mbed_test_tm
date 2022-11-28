#ifndef CALCVTHETA_H
#define CALCVTHETA_H

#include <math.h>

class CalcVTheta
{
public:
    CalcVTheta(float theta_wheel);   //コンストラクタ
    double calcV();
    double calcTheta();
    void canTransmit();

private:
    void CAN_recieve();
    double theta_feeld;
    double vx, vy, vtheta;
};

#endif