/*  独ステの制御用
*   各ユニットに1枚マイコンを使う
*   参考：
*/

#ifndef DOKUSUTECONTROLLER_H
#define DOKUSUTECONTROLLER_H

#include <math.h>

class DokusuteController
{
public:
    DokusuteController();   //コンストラクタ
    void CAN_recieve(int target_v, int target_stearAngle);
    double calcV();
    double calcTheta();
    void moveStear();
    void moveWheel();

private:
    double target_v;
    double target_stearAngle;
    double now_v, now_stearAngle;
    double delta_v, delta_stearAngle;
};

#endif