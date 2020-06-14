#include "mbed.h"
#include "MoterCTL.h"
#define CLEAR(x) sprintf(x,"");
Sensor sensor[] = {
    Sensor(PA_0),
    Sensor(PA_1)
};
Fbmo moter1(
    /*モータ出力1のピン名*/ PC_9,
    /*モータ出力2のピン名*/ PB_6,
    /*エンコーダ入力1のピン名*/ PB_2,
    /*エンコーダ入力2のピン名*/ PB_1,
    /*スキャン周期(現在強制0.1)*/ 0.1
);
Fbmo moter2(
    /*モータ出力1のピン名*/ PB_10,
    /*モータ出力2のピン名*/ PB_15,
    /*エンコーダ入力1のピン名*/ PB_14,
    /*エンコーダ入力2のピン名*/ PB_4,
    /*スキャン周期(現在強制0.1)*/ 0.1
);
Fbmo moter3(
    /*モータ出力1のピン名*/ PB_5,
    /*モータ出力2のピン名*/ PB_13,
    /*エンコーダ入力1のピン名*/ PB_3,
    /*エンコーダ入力2のピン名*/ PB_4,
    /*スキャン周期(現在強制0.1)*/ 0.1
);
Operation opX;
Operation opY;
Serial xbee(PA_9,PA_10);
InterruptIn SSBT(USER_BUTTON);//ボタン割り込み
//ISコマンド
const char IS[19]= {0x7E,0x00,0x0F,0x17,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFE,0x02,0x49,0x53,0x4E};
//グローバル変数
int c=0,d=0,BT=0,cnt=0,RPM = 0,size = 0;
double x=0,y=0,z=0;
float oldT = 0,newT = 0,sum = 0,vtimer[100],avg = 0;
bool flag = false;
char val[10]="";char val1g[10]="";char val2g[10]="";char val3g[10]="";char val300[300]="";

//電圧変換
double vconv(double val){
   double ans;
   ans=1.17*val/1024;
   return ans;
}
//リセット処理
void reset(){
    CLEAR(val1g) CLEAR(val2g) CLEAR(val3g) CLEAR(val300)
    size = 0;
    flag=false;    
}
//ボタン割り込み(デバック用)
void BTH(){
    printf("z = %f[mV],y = %f[mV],x = %f[mV], iRPM = %d\r\n",z,y,x,RPM);
}

int main(){
    xbee.baud(9600);
    SSBT.rise(&BTH);
    int a=0,b=0,k=0,timeout=0,Turning=0;
    SSBT.rise(&BTH);
    while(1){
        //通信
        if(xbee.readable() == 0 && flag == false){
            for(int i=0;i<19;i++){
                xbee.putc(IS[i]);
            }timeout=0;
            flag = true;
        }else if (xbee.readable() == 1 && flag == true){
            timeout=0;
            size += sprintf(val,"%02x",xbee.getc());
            strcat(val300,val);
        }else if(xbee.readable() == 0 && flag == true){
            timeout++;
            if(timeout > 250000 && size < 58){
                reset();
                timeout = 0;
            }else if(timeout < 250000 && size == 58){
                for(a=44;a < (size - 2);){
                    for(b=0;b<4;b++){
                        if(b==0)
                            sprintf(val,"0x%c",val300[a]);
                        else
                            sprintf(val,"%c",val300[a]);
                        if(k==0){strcat(val1g,val);}
                        else if(k==1){strcat(val2g,val);}
                        else if(k==2){strcat(val3g,val);}
                        a++;
                    }k++;
                }k=0;
                z = vconv(atof(val1g));y = vconv(atof(val2g));x = vconv(atof(val3g));
                reset();
            }else if(size>58){
                reset();
            }
        }
        //モーター
        RPM = opX.s3axis(x,z);
        Turning = opY.s3axis(y,z);
        if(Turning>0 && abs(RPM) < abs(Turning)){//旋回
            moter1.setRpm(Turning);
            moter2.setRpm(Turning);
            moter3.setRpm(Turning);
            // moter1.tesMo(0.0,0.25);
            // moter2.tesMo(0.0,0.25);
            // moter3.tesMo(0.0,0.25);
        }else if(Turning<0 && abs(RPM)<abs(Turning)){
            moter1.setRpm(Turning);
            moter2.setRpm(Turning);
            moter3.setRpm(Turning);
            // moter1.tesMo(0.25,0.0);
            // moter2.tesMo(0.25,0.0);
            // moter3.tesMo(0.25,0.0);
        }else if(RPM > 0 && abs(RPM)>abs(Turning)){//前進
            if(sensor[0].detect() == false){
                moter2.setRpm(RPM);
                moter3.setRpm(-RPM);
                // moter2.tesMo(0.0,0.3);
                // moter3.tesMo(0.25,0.0);
            }else{
               moter1.setRpm(0);
               moter2.setRpm(0);
               moter3.setRpm(0);
                // moter1.tesMo(0.0,0.0);
                // moter2.tesMo(0.0,0.0);
                // moter3.tesMo(0.0,0.0);
            }
        }else if(RPM < 0 && abs(RPM)>abs(Turning)){//後進
            if(sensor[1].detect() == false){
                moter2.setRpm(RPM);
                moter3.setRpm(-RPM);
                // moter2.tesMo(0.25,0.0);
                // moter3.tesMo(0.0,0.3);
            }else{
               moter1.setRpm(0);
               moter2.setRpm(0);
               moter3.setRpm(0);
                // moter1.tesMo(0.0,0.0);
                // moter2.tesMo(0.0,0.0);
                // moter3.tesMo(0.0,0.0);
            }
        }else if(RPM == 0 && Turning == 0){
           moter1.setRpm(0);
           moter2.setRpm(0);
           moter3.setRpm(0);
            // moter1.tesMo(0.0,0.0);
            // moter2.tesMo(0.0,0.0);
            // moter3.tesMo(0.0,0.0);
        }
    }
}
