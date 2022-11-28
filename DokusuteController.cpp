#include "DokusuteController.h"
#include "mbed.h"
#include "EC.h"
#include "CalPID.h"
#include "MotorController.h"

#define CAN_Hz 1000000

CAN can(PA_11,PA_12,CAN_Hz); //CAN_RD, CAN_TD, CAN_Hzの順

DokusuteController::DokusuteController(){
    //初期化事項あれば
}

double calcV();
double calcTheta();
void moveStear();
void moveWheel(){

}
void CAN_recieve(int target_v, int target_stearAngle){
    CANMessage msg; // 送られてきたデータを入れる箱
    can.read(msg); //CANデータの読み取り
    target_v = (msg.data[0] - 128) * 4; //<-128~127>に変換
    target_stearAngle = (msg.data[1] - 128) * 4;
}