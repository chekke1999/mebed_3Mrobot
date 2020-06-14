#include "mbed.h"
//モーターxyzからRPMに変換
class Operation{
    public:
        int s3axis(double xy_axis,double z_axis){
            int stage_rpm = 0;
            if( 0 < z_axis && z_axis < 0.48){
                stage_rpm = (0.35 - xy_axis)*100;
            }
            return stage_rpm*20;
        }
};
//センサー
class Sensor{
    private:
        AnalogIn inS;
    public:
        Sensor(PinName _s):inS(_s){}
        bool detect(){
            float val = inS.read();
            return (val>0.39f)?true:false;
        }

};
//フィードバックモータ制御クラス
class Fbmo{
    private:
        Ticker time;
        PwmOut out1;
        PwmOut out2;
        InterruptIn HLin1;
        InterruptIn HLin2;
        //クラス内グローバル変数 staticがインスタンス間で値を共有して不具合発生
        int i,j,cnt,Gppm;
        float pwA,pwB,pps,pr_scan;
        //デューティー比制御
        float Duty(float pw,bool HL){
            if(HL == true && pw < 1.00f){
                pw = pw + 0.01f;
            }else if(HL == false && pw > 0.00f){
                pw = pw - 0.01f;
            }
            return pw;
        }
        //立下り割り込み回数カウント用
        void low() {
            i++;
        }
        void scan(){//1216 = 64*19
            pps = (float)(1 / pr_scan) * i;
            ppm = pps*60;
            i=0;
        }
    public:
        int GRpm,ppm;
        Fbmo(PinName PWout1,PinName PWout2,PinName Din1,PinName Din2,double scan_rpm)
        :out1(PWout1),out2(PWout2),HLin1(Din1),HLin2(Din2),i(0),j(0),cnt(0),Gppm(0),
         pwA(0.0),pwB(0.0),GRpm(0),ppm(0){
            if(scan_rpm > 1){
                scan_rpm = 1;
            }else if(0.001 > scan_rpm){
                scan_rpm = 0.001;
            }
            pr_scan = scan_rpm;
            time.attach(callback(this,&Fbmo::scan), scan_rpm);
            HLin1.fall(callback(this,&Fbmo::low));
            HLin2.fall(callback(this,&Fbmo::low));
        }
        int setRpm(int iRPM) {
            GRpm = iRPM;
            Gppm = abs(GRpm*1216);
            if(GRpm > 0){
                pwB = 0.0f;
                if(ppm < Gppm)
                    pwA = Duty(pwA,true);
                    //pwA = 1.0f;
                else if(ppm > Gppm)
                    pwA = Duty(pwA,false);
                    //pwA= 0.0f;
            }else if(GRpm < 0){
                pwA = 0.0f;
                if(ppm < Gppm)
                    pwB = Duty(pwB,true);
                    //pwB = 1.0f;
                else if(ppm > Gppm)
                    pwB = Duty(pwB,false);
                    //pwB = 0.0f;
            }else if(GRpm == 0){
                pwA = 0.0f;
                pwB = 0.0f;
            }
            out1.write(pwA);
            out2.write(pwB);
            return 0;
        }
        int tesMo(double pwm1,double pwm2){
            out1.write(pwm1);
            out2.write(pwm2);
            return 0;
        }
};

