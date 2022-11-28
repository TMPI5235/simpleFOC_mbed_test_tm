#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

class MotorController
{
private:
    float delta_t;
    float angle;
    float angle_offset;
    float duty_offset;

    float slope_forward_rotation;
    float slope_backward_rotation;
    float intercept_forward_rotation;
    float intercept_backward_rotation;

    float duty_max;
    float omega_max;
    float acceleration_max;
    float pre_target_omega;

    Ec *ec_;
    CalPID *sc_pid;
    CalPID *ac_pid;

    //
    float limitValue(float value, float max, float min);
    float calSc(float target_omega_input); //速度制御

protected:
    PwmOut motor_p;
    PwmOut motor_n;

public:
    MotorController(PinName motor_p_, PinName motor_n_, float dt, Ec &ec, CalPID &sc, CalPID &ac); //引数は下行
    //モーター正転、逆転、周期[s]、エンコーダ、速度制御用のPID、角度制御のPID

    //////////////////////////////////////////各クラスのコンストラクタ引数で設定はされているので呼び出しは不要。変更したい場合などに
    void period(float s);                               // pwm周期設定用関数
    void period_ms(int ms);                              // pwm周期設定用関数
    void period_us(int us);                              // pwm周期設定用関数
    void setDutyLimit(float duty_limit);                // 最終的な出力(duty)の最大値設定
    void setMaxScPID(float pid_max_sc);                   // CalPIDによるduty比の最大値設定
    void setMaxAcPD(float pid_max_ac);                   // CalPIDによる角速度[rad/s]の最大値設定
    void setPIDParamSc(float kp, float ki, float kd); //速度制御のPDゲイン設定用関数
    void setPDParamAc(float kp, float kd);             //角度制御のPDゲイン設定用関数
    void setDeltaTime(float dt);                        //制御周期の設定用関数
    void setAccelMax(float a_max);
    /////////////
    void setDutyOffset(float duty_off);
    void setAngleOffset(float angle_calibration);

    //////////////////////////////////////////実際に使うときに呼び出す関数
    float getAngle();
    void setEquation(float slope_f, float intercept_f, float slope_b, float intercept_b); //速度制御の設定用。設定が必要
    void turn(float duty);                                                                   //モーターにduty比入力する
    void Ac(float target_angle);                                                             // PD算出された角速度による角度制御。速度制御を利用している
    void Sc(float target_omega_input);                                                       //速度制御
    void stop();                                                                              // duty=0を入力
    void reset();
};

#endif