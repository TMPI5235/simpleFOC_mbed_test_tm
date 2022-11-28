#ifndef _INCLUDED_EC_H_
#define _INCLUDED_EC_H_

#ifndef M_PI
#define M_PI 3.14159265359f
#endif


    /** @section SAMPLE
     * @code
     * //プログラム例
     * #include "mbed.h"
     * #include "EC.h"
     *
     * #define RESOLUTION 500
     * Ticker ticker;
     * Serial pc(USBTX,USBRX);
     *
     * //          (A層,B層,分解能)
     * Ec1multi EC(p7,p8,RESOLUTION);  //1逓倍用class
     * //            or
     * //Ec2multi EC(p7,p8,RESOLUTION);  //2逓倍用class
     * //            or
     * //Ec4multi EC(p7,p8,RESOLUTION);  //4逓倍用class
     * void calcOmega();
     *
     * int main()
     * {
     *     int count=0;
     *     double omega;
     *     ticker.attach(&calcOmega,0.05);
     *
     *     while(1) {
     *         count=EC.getCount();
     *         omega=EC.getOmega();
     *         pc.printf("count=%d,",count);
     *         pc.printf("omega=%f\r\n",omega);
     *     }
     * }
     * void calcOmega()
     * {
     *     EC.calOmega();
     * }
     * @endcode
     */

    /**
     * @brief increment型エンコーダ用class
     * @details 1逓倍：Ec1multi
     * 2逓倍：Ec2multi
     * 4逓倍：Ec4multi
     **/
    class Ec
{
protected:
    int count_;         //カウント数
    double omega_;      //角速度(rad/s)
    double pre_omega_;  //角速度(rad/s)
    double pre2_omega_; //角速度(rad/s)
    double acceleration_;
    int pre_count_;  //一つ前のカウント
    int resolution_; //分解能
    int multiplication_;
    double ptw_;
    double gear_ratio_;

public:
    /*コンストラクタの定義
    1,2,4逓倍の元となるclass

    @param res エンコーダの分解能
    @param multi エンコーダの逓倍

    @remarks このclassは各逓倍のclassに継承されるため、使用者が宣言する必要はない
    */
    Ec(int res, int multi);
    /** @details エンコーダのcountを返す関数
     * 1周のcount=分解能×逓倍
     * @return count
     */
    int getCount() const;
    /**
     *   軸の回転角度を返す関数
     *   @return θ[rad]
     */
    double getRad() const;
    double getDeg() const;
    /**
     * 軸の角速度を返す関数
     * @return ω [rad/s]
     */

    double getOmega() const;
    /**
     * 角速度を計算するための関数
     * 微分を微小時間の変位として計算しているので、タイマー割込などで回さなければいけない
     */
    double getAcceleration() const;
    /**
     * 軸の角加速度を返す関数
     * @return a [rad/s^2]
     */
    void calOmega();
    /**
     * 分解能を指定するための関数
     * @param res エンコーダの分解能
     */
    void setResolution(int res);
    ///エンコーダのcountやωをリセットするための関数
    void reset();
    ///ギア比を設定する関数。デフォルトの値は1(減速無し)
    void setGearRatio(double gear_r);
    
    Timer timer_;
};
///@brief increment型エンコーダ用class(1逓倍)
class Ec1multi : public Ec
{
private:
    InterruptIn signalA_;
    DigitalIn signalB_;
    void upA();

public:
    /**
    * @brief コンストラクタの定義
    * @details main関数の前に必ず一度宣言する
    * 使うエンコーダの数だけ設定する必要がある

    @ param signalA エンコーダのA相のピン名
    @ param signalB エンコーダのB相のピン名
    @ param res エンコーダの分解能
    @ remark 2,4逓倍も同様
    */
    Ec1multi(PinName signalA, PinName signalB, int res);
};
///@brief increment型エンコーダ用class(2逓倍)
class Ec2multi : public Ec
{
private:
    InterruptIn signalA_;
    DigitalIn signalB_;
    void upA();
    void downA();

public:
    /**
    * @brief コンストラクタの定義
    * @details main関数の前に必ず一度宣言する
    * 使うエンコーダの数だけ設定する必要がある

    @ param signalA エンコーダのA相のピン名
    @ param signalB エンコーダのB相のピン名
    @ param res エンコーダの分解能
    */
    Ec2multi(PinName signalA, PinName signalB, int res);
};
///@brief increment型エンコーダ用class(4逓倍)
class Ec4multi : public Ec
{
private:
    InterruptIn signalA_;
    InterruptIn signalB_;
    void upA();
    void downA();
    void upB();
    void downB();
    int pa_, pb_;

public:
    /**
    * @brief コンストラクタの定義
    * @details main関数の前に必ず一度宣言する
    * 使うエンコーダの数だけ設定する必要がある

    @ param signalA エンコーダのA相のピン名
    @ param signalB エンコーダのB相のピン名
    @ param res エンコーダの分解能
    */
    Ec4multi(PinName signalA, PinName signalB, int res);
};

#endif