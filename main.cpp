/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "./library/SimpleFOC_Mbed/SimpleFOC.h"
#include <cstdio>
#include "FastPWM.h"
//

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms
#define PP 7

//#define VEL_EXAMPLE       //角度制御する場合はここをコメントアウト

#ifndef VEL_EXAMPLE
#define ANG_EXAMPLE         //速度制御でない場合、角度制御モードにする
#endif

//BufferedSerial pc(USBTX, USBRX);

DigitalOut led(LED1);

//F401
/*PwmOut pwm_A(D7);
PwmOut pwm_B(D8);
PwmOut pwm_C(D2);
FastPWM pwm_A(D7);
FastPWM pwm_B(D8);
FastPWM pwm_C(D2);
DigitalOut en(D4);
InterruptIn in_A(A2);
InterruptIn in_B(A4);
InterruptIn in_C(A5);*/

//F303
/*PwmOut pwm_A(D9);
PwmOut pwm_B(D10);
PwmOut pwm_C(D11);
DigitalOut en(D8);
InterruptIn in_A(D4);
InterruptIn in_B(D5);
InterruptIn in_C(D7);*/

//F446  2回路テスト基板右系統
//
/*PwmOut pwm_A(PA_9);
PwmOut pwm_B(PA_8);
PwmOut pwm_C(PA_10);*/
FastPWM pwm_A(PA_9);
FastPWM pwm_B(PA_8);
FastPWM pwm_C(PA_10);
DigitalOut en(PB_3);
InterruptIn in_A(PC_7);
InterruptIn in_B(PB_10);
InterruptIn in_C(PB_5);


BufferedSerial serial(USBTX,USBRX, 115200);

BLDCMotor motor = BLDCMotor(PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(&pwm_A, &pwm_B, &pwm_C, &en);
HallSensor sensor = HallSensor(in_A, in_B, in_C, PP);

Commander command = Commander(serial);  //

void doA() {sensor.handleA();}
void doB() {sensor.handleB();}
void doC() {sensor.handleC();}

////////////////////////        角度制御のとき      //////////////////////////////
#ifdef ANG_EXAMPLE
void hello(char *cmd);
void disableMotor(char* cmd);
void enableMotor(char* cmd);
void setTarget_ang(char* cmd);
void onPID_vel(char* cmd);
void onPID_ang(char* cmd);
void getMotorValue(char* cmd);

float target_angle = 0;

int main()
{
    command.add('a', hello, "hello world");
    command.add('d', disableMotor, "disable motor");
    command.add('e', enableMotor, "enable motor");
    command.add('t', setTarget_ang, "set target angle");
    command.add('p', onPID_vel, "velocoty PID setting");
    command.add('P', onPID_ang, "angle P setting");
    command.add('m', getMotorValue, "motor value");

    command.echo = true;

    //NEO550
    driver.pwm_frequency = 20000;
    
    sensor.pullup = Pullup::USE_INTERN;
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;   //ドライバ電圧
    
    driver.init();
    
    motor.linkDriver(&driver);
    
    motor.voltage_sensor_align = 0.35;  //センサアライメント????
    motor.velocity_index_search = 5;    //

    motor.controller = MotionControlType::angle;    //動作モード設定：角度制御

    motor.PID_velocity.P = 0.08;        //モル
    motor.PID_velocity.I = 0.06;
    motor.PID_velocity.D = 0;//.01;
    motor.PID_velocity.limit = 50;
    motor.voltage_limit = 10;
    motor.PID_velocity.output_ramp = 100;
    motor.LPF_velocity.Tf = 0.1;

    motor.P_angle.P =12;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.P_angle.output_ramp = 700;     //角加速度制限
    motor.velocity_limit = 300;         //回転速度リミット
    motor.useMonitoring(serial);
    
    motor.init();                       //モータ・モード設定を適用
    motor.initFOC(3.14,Direction::CCW); //センサチェック(引数指定で回転方向、センサオフセットのチェックを回避可能(既知値の場合、指定する))

    //センサチェックの判定
    if (motor.initFOC())  printf("FOC init success!\n");
    else{
        printf("FOC init failed!\n");
        return 99;      //なにかしらのエラーを返す
    }

    ThisThread::sleep_for(1s);
    printf("control start\n");
    printf("%d\n", pwm_A.read_period_us());
    

    led = 0;
    while (true) {
        
        motor.loopFOC();                //現在角度を取得して電圧を計算する(なるべく高速で回すこと)(nucleoなら~100usくらいで回せるはず)
        motor.move(target_angle);       //この引数に目標値を渡すとモータが動く(これはそんなに高速で回さなくていい)
        command.run();                  //シリアル指令用、本番機不要
        //led = 0;
        
    }
}

void hello(char* cmd) {
    /*if(led) led = 0;
    else led = 1;*/
}

void disableMotor(char* cmd) {
    motor.disable();
    printf("motor disabled\n");
}
void enableMotor(char* cmd) {
    motor.enable();
    printf("motor enabled\n");
}
void setTarget_ang(char* cmd) {
    command.scalar(&target_angle, cmd);
}
void getMotorValue(char* cmd) {
    command.motor(&motor, cmd);
}
void onPID_vel(char* cmd) {
    command.pid(&motor.PID_velocity, cmd);
}
void onPID_ang(char* cmd) {
    command.pid(&motor.P_angle, cmd);
}
#endif




//////////////////////////      速度制御のとき      //////////////////////////////
#ifdef VEL_EXAMPLE
void hello(char* cmd);
void onPID_vel(char* cmd);
void setTarget_vel(char* cmd);
void getMotorValue(char* cmd);
void setTf_vel(char* cmd);
void setTf_ang(char* cmd);

float target_velocity = 0;

int main()
{
    command.add('a', hello, "hello world");
    command.add('p', onPID_vel, "velocity PID setting");
    command.add('t', setTarget_vel, "set target velocity");
    command.add('g', getMotorValue, "get motor parameter");
    command.add('l', setTf_vel, "set velocity Tf");
    command.add('L', setTf_ang, "set angle Tf");
    command.echo = true;

    
    driver.pwm_frequency = 20000;
    
    sensor.pullup = Pullup::USE_INTERN;
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;
    
    driver.init();
    
    motor.linkDriver(&driver);
    
    motor.voltage_sensor_align = 0.3;

    motor.controller = MotionControlType::velocity;
    //PwmOut parameters
    /* 
    motor.PID_velocity.P = 0.048;
    motor.PID_velocity.I = 0.04;
    motor.PID_velocity.D = 0;//.01;
    motor.LPF_velocity.Tf = 0.1;
    motor.LPF_angle.Tf = 0.00001;
    */

    //FastPWM parameters
    motor.PID_velocity.P = 0.072;
    motor.PID_velocity.I = 0.06;
    motor.PID_velocity.D = 0;//.01;
    motor.PID_velocity.limit = 10;
    motor.voltage_limit = 10;
    motor.PID_velocity.output_ramp = 100;
    motor.LPF_velocity.Tf = 0.1;

    motor.velocity_limit = 50;
    motor.useMonitoring(serial);
    
    motor.init();
    motor.initFOC();

    ThisThread::sleep_for(1s);
    printf("control start\n");
    printf("%d\n", pwm_A.read_period_us());
    
    int loop_count = 0;
    led = 0;
    bool PlusMinus = 1;
    while (true) {
        
        motor.loopFOC();
        motor.move(target_velocity);        //この引数に目標値を渡すと回る
        
        command.run();
        
        loop_count++;
        if(loop_count > 10000) {
            led = !led;
            loop_count = 0;
            
            if(target_velocity > 200.0){
                PlusMinus = 0;
            }else if(target_velocity < 20.0){
                PlusMinus = 1;
            }

            if(PlusMinus == 1){
            target_velocity += 1.0;
            } else{
                target_velocity -= 1.0;
            }
            //printf("%f\n", target_velocity);
        }
        
    }
}



void hello(char* cmd) {
    /*if(led) led = 0;
    else led = 1;*/
}

void onPID_vel(char* cmd) {
    command.pid(&motor.PID_velocity, cmd);
}
void setTarget_vel(char* cmd) {
    command.scalar(&target_velocity, cmd);
}

void getMotorValue(char* cmd) {
    command.motor(&motor, cmd);
}

void setTf_vel(char* cmd) {
    command.scalar(&motor.LPF_velocity.Tf, cmd);
}

void setTf_ang(char* cmd) {
    command.scalar(&motor.LPF_angle.Tf, cmd);
}


#endif