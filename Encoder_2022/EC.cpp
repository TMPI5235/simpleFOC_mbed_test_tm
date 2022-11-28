#include "mbed.h"
#include "EC.h"

Ec::Ec(int res, int multi) : count_(0), pre_omega_(0), pre_count_(0), resolution_(res), multiplication_(multi)
{
    timer_.start();
    setGearRatio(1);
}

int Ec::getCount() const
{
    return count_;
}

double Ec::getRad() const
{
    return count_ * 2.0f * M_PI / (multiplication_ * resolution_ * gear_ratio_);
}
double Ec::getDeg() const
{
    return count_ * 2.0f * 180.0 / (multiplication_ * resolution_ * gear_ratio_);
}
void Ec::calOmega()
{
    double t = timer_.read();
    double delta_time = t - ptw_;
    acceleration_ = (pre_omega_ - pre2_omega_) / delta_time;
    pre2_omega_ = pre_omega_;
    pre_omega_ = omega_;
    omega_ = (count_ - pre_count_) * 2.0f * M_PI / (multiplication_ * resolution_ * delta_time);
    omega_ /= gear_ratio_;
    pre_count_ = count_;
    ptw_ = t;
}

double Ec::getOmega() const
{
    return omega_;
}
double Ec::getAcceleration() const
{
    return acceleration_;
}
void Ec::setResolution(int res)
{
    resolution_ = res;
}

/*reset関数の定義*/
/*エンコーダを初期状態に戻すことができる*/
void Ec::reset()
{
    count_ = 0;
    pre_count_ = 0, omega_ = 0;
    ptw_ = 0;
    timer_.stop();
    timer_.reset();
    timer_.start();
}
void Ec::setGearRatio(double gear_r)
{
    gear_ratio_ = gear_r;
}

////////////////////////////////////////////////////1逓倍//////////////////////////////////////////////////////////////////
Ec1multi::Ec1multi(PinName signalA, PinName signalB, int res) : Ec(res, 1), signalA_(signalA), signalB_(signalB)
{
    signalA_.rise(callback(this, &Ec1multi::upA));
}

//ピン変化割り込み関数の定義
void Ec1multi::upA()
{
    if (signalB_.read())
        count_++;
    else
        count_--;
}
////////////////////////////////////////////////////2逓倍//////////////////////////////////////////////////////////////////
Ec2multi::Ec2multi(PinName signalA, PinName signalB, int res) : Ec(res, 2), signalA_(signalA), signalB_(signalB)
{
    signalA_.rise(callback(this, &Ec2multi::upA));
    signalA_.fall(callback(this, &Ec2multi::downA));
}

//ピン変化割り込み関数の定義
void Ec2multi::upA()
{
    if (signalB_.read())
        count_++;
    else
        count_--;
}
void Ec2multi::downA()
{
    if (signalB_.read())
        count_--;
    else
        count_++;
}

////////////////////////////////////////////////////4逓倍//////////////////////////////////////////////////////////////////
Ec4multi::Ec4multi(PinName signalA, PinName signalB, int res) : Ec(res, 4), signalA_(signalA), signalB_(signalB), pa_(0), pb_(0)
{
    signalA_.rise(callback(this, &Ec4multi::upA));
    signalA_.fall(callback(this, &Ec4multi::downA));
    signalB_.rise(callback(this, &Ec4multi::upB));
    signalB_.fall(callback(this, &Ec4multi::downB));
}
void Ec4multi::upA()
{
    pa_ = 1;
    if (pb_ == 1)
        count_++;
    else
        count_--;
}
void Ec4multi::downA()
{
    pa_ = 0;
    if (pb_ == 1)
        count_--;
    else
        count_++;
}
void Ec4multi::upB()
{
    pb_ = 1;
    if (pa_ == 1)
        count_--;
    else
        count_++;
}
void Ec4multi::downB()
{
    pb_ = 0;
    if (pa_ == 1)
        count_++;
    else
        count_--;
}