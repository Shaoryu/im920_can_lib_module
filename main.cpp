#include "mbed.h"
#define CANID 0x10 //omuniなら0x20

BufferedSerial im920(PA_9,PA_10,19200);
CAN can(PA_11,PA_12,1000000);
BufferedSerial pc(USBTX,USBRX,115200);
DigitalOut a(PA_0);
DigitalOut b(PA_1);
DigitalOut c(PB_4);
DigitalOut d(PB_5);
Timer t,limit;

int data[8];
int input[50];
int input_proc[50];
int nodeNUM;
void get_data();
CANMessage msg,msg_again,msg_receive;
    
int val=0;
int main()
{
    can.mode(CAN::Normal);
    msg.len=8;
    int nodeid=0;
    int node_led , node_led_counter=0 ,node_led_timer=125;
    int i=0;
    const char TXDA[] = {"TXDA"};//broad cast
    const char RDNN[] = {"RDNN"};//node read
    const char NEWLINE[] = {"\r\n"};
    char buf[17];
    buf[0]='X';
    int high,low,num,flag;
    // bool rflag=false,tflag=false,rflagflag=true;
    t.start();
    limit.start();
    b=0;

    ThisThread::sleep_for(50ms);
    //RDNN<CR><LF>
    im920.write(RDNN,sizeof(RDNN)-1);
    im920.write(NEWLINE,sizeof(NEWLINE));
    // ThisThread::sleep_for(500ms);
    do{
        im920.read(&input[i],1);
        input_proc[i]=input[i];
        if(input[i]>='0'&&input[i]<='9')input_proc[i]-='0';
        else if(input[i]>='A'&&input[i]<='F') input_proc[i]=input_proc[i]-'A'+10;  
    }while(input[i++]!='\n');
    node_led=input_proc[0]*pow(16,3)+input_proc[1]*pow(16,2)+input_proc[2]*pow(16,1)+input_proc[3];
    if(node_led<=0||node_led>0xFFFF||i!=6)system_reset();

    while (true) {
        c=!c;
        if(std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count()>node_led_timer){
            a=!a;
            if(++node_led_counter==node_led*2){
                node_led_counter=0;
                node_led_timer=1000;
            }else{
                node_led_timer=150;
            }
            t.reset();
        }


        if(can.read(msg_receive)){
            if(msg_receive.id==CANID)nodeid=msg_receive.data[0];//メインプログラムが受信したいコントローラノードを保存
            else if(msg_receive.id==0x50+node_led){//メインマイコンから送信したいデータを受信
                //コントローラのモジュールとロボット間通信のモジュールのノードは異なるものにする
                // while(std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count()<60);
                // c=!c;
                for(int i=0;i<8;i++){
                    high=msg_receive.data[i]/16;
                    low=msg_receive.data[i]%16;
                    buf[2*i+1]=high+'0';
                    buf[2*i+2]=low+'0';
                }
                //TXDA [data]<CR><LF> コマンドとデータの間にスペース
                im920.write(TXDA,sizeof(TXDA));
                im920.write(buf,sizeof(buf));
                im920.write(NEWLINE,sizeof(NEWLINE));
                // if(rflag&&!tflag&&rflagflag){t.reset();rflagflag=false;}
                // tflag=true;
            }
            
        }
        if(im920.readable()){
            i=0;
            flag=0;
            do{
                im920.read(&input[i],1);
                input_proc[i]=input[i];
                if(input[i]>='0'&&input[i]<='9')input_proc[i]-='0';
                else if(input[i]>='A'&&input[i]<='F') input_proc[i]=input_proc[i]-'A'+10;  
                if(input[i]==':'&&flag==0){flag=i;}
            }while(input[i++]!='\n');

            if(input[0]=='N'||input[0]=='O');
            else if(input[flag+2]=='X'){
                // rflag=true;
                // t.reset();
                for(int j=0;j<8;j++){
                    num=(input[flag+2*j+3]-'0')*16+input[flag+2*j+4]-'0';
                    // num=(input_proc[flag+2*j+3])*16+input_proc[flag+2*j+4];
                    msg.data[j]=num;
                }
                msg.id=0x50;
                if(can.write(msg))b=!b;

            }else{// if(input[flag+2]=='Y')
                b=!b;
                d=1;
                nodeNUM=input_proc[5]*16+input_proc[6];
                msg.id=CANID+nodeNUM;//msg idでノード番号判別
                nodeid=msg.id;//脳筋解決だから改善
                // if(msg.id==nodeid)d=1;
                // else d=0;
                for(int j=0;j<8;j++)data[j]=input_proc[j*2+flag+2]*16+input_proc[j*2+flag+3];//データ変換
                for(int l=0;l<6;l++)msg.data[l]=(char)data[l];

                msg.data[6]=((data[6]&0x7F)|((data[7]<<7)&0x80))&0xFF;
                msg.data[7]=((data[7]>>1)&0x3F)|((input[flag+1]<<6)&0xC0);

                can.write(msg);
                msg_again=msg;
                limit.reset();
            }
        }
        if(std::chrono::duration_cast<std::chrono::milliseconds>(limit.elapsed_time()).count()>210){
            d=0;//最後に
            msg_again.id=nodeid;
            // for(int i=0;i<8;i++)msg_again.data[i]=0;
            msg_again.data[0]=0;
            msg_again.data[1]=0;
            msg_again.data[2]=128;
            msg_again.data[3]=128;
            msg_again.data[4]=128;
            msg_again.data[5]=128;
            msg_again.data[6]=0;
            msg_again.data[7]=0;//非常停止が解除の状態なので注意
            can.write(msg_again);//データを初期値に戻す
        }else {
            can.write(msg_again);
        }
        ThisThread::sleep_for(3ms);
        // b=!b;
    }
}
