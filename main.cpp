#include "mbed.h"

BufferedSerial uart0(P1_7, P1_6,9600);
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
I2C i2c(P0_5,P0_4); //SDA,SCL P0_5,P0_4
DigitalOut LE1(P1_4);   //dds1
DigitalOut LE2(P1_1);
DigitalOut LE3(P0_2);
DigitalOut LE4(P0_7);
DigitalOut CS1(P1_5);   //dac1
DigitalOut CS2(P1_2);
DigitalOut CS3(P1_9);
DigitalOut CS4(P0_3);
AnalogIn ain0(P0_11);
AnalogIn ain1(P1_0);

//cs control func.
void cs_hi(uint8_t num);    //dac
void cs_lo(uint8_t num);
void le_hi(uint8_t num);    //dds
void le_lo(uint8_t num);
uint8_t buf;

//DDS control
#define res_inv 4           //res=67108864/2^28
uint8_t i;
void waveset(uint8_t ch, uint32_t freq, uint16_t pha, uint16_t ampl);    //waveset func.
uint32_t freq1,freq2,freq3,freq4;              //Hz
uint16_t pha1,pha2,pha3,pha4,ampl1,ampl2,ampl3,ampl4;          //deg. loaded mV

//DAC control
#define dac_fs 2500     //DAC full scale Vout
#define dac_res 4096    //dac resolution 2^12
#define g 7             //driver amp gain
#define dac_fs2 3300     //DAC full scale Vout. i2c
const uint8_t dac_addr=0xc0;    //i2c dac addr

//analog out
uint16_t a1,a2;

//parser
char char_read();
float char2flac();
void parser();
float pars[4];
uint32_t a,b,c,d;

int main(){
    for(i=1;i<=4;++i) cs_hi(i); //CS init
    for(i=1;i<=4;++i) le_hi(i); //LE init
    spi.format(16,2);   //spi mode setting. 2byte transfer, mode 2
    thread_sleep_for(100);

    while (true){
        buf=char_read();
        if(buf=='S'){
            parser();
        }else if(buf=='?'){
            a1=ain0.read_u16()*3300/65535;  //mV unit
            a2=ain1.read_u16()*3300/65535;  //mV unit
            printf("%d,%d,%04d,%04d\n\r",0,0,a1,a2);
        }
        a=(uint32_t)pars[0];
        b=(uint32_t)pars[1];
        c=(uint32_t)pars[2];
        d=(uint32_t)pars[3];

        if(a==1){
            freq1=b;
            pha1=c;
            ampl1=d;
        }else if(a==2){
            freq2=b;
            pha2=c;
            ampl2=d;
        }else if(a==3){
            freq3=b;
            pha3=c;
            ampl3=d;
        }else if(a==4){
            freq4=b;
            pha4=c;
            ampl4=d;
        }
        waveset(1,freq1,pha1,ampl1);
        waveset(2,freq2,pha2,ampl2);
        waveset(3,freq3,pha3,ampl3);
        waveset(4,freq4,pha4,ampl4);
        //accum rest
        for(i=1;i<=4;++i) le_lo(i);
        spi.write(0x2000);      //accum. reset
        for(i=1;i<=4;++i) le_hi(i);
    }
}

//cs control func.
void cs_hi(uint8_t num){
    if(num==1) CS1=1;
    else if(num==2) CS2=1;
    else if(num==3) CS3=1;
    else if(num==4) CS4=1;
}
void cs_lo(uint8_t num){
    if(num==1) CS1=0;
    else if(num==2) CS2=0;
    else if(num==3) CS3=0;
    else if(num==4) CS4=0;
}
void le_hi(uint8_t num){
    if(num==1) LE1=1;
    else if(num==2) LE2=1;
    else if(num==3) LE3=1;
    else if(num==4) LE4=1;
}
void le_lo(uint8_t num){
    if(num==1) LE1=0;
    else if(num==2) LE2=0;
    else if(num==3) LE3=0;
    else if(num==4) LE4=0;
}

//wave set func.
void waveset(uint8_t ch, uint32_t freq, uint16_t pha, uint16_t ampl){
    uint16_t buf;
    char set[2];
    if(freq>30000000)freq=30000000;
    if(pha>360)pha=360;
    if(ampl>2100)ampl=2100;

    le_lo(ch);
    spi.write(0x2100);
    buf=((res_inv*freq)&0x3FFF)+0x4000;
    spi.write(buf);
    buf=((res_inv*freq)>>14)+0x4000;
    spi.write(buf);
    buf=(4096*pha/360)+0xC000;
    spi.write(buf);
    le_hi(ch);
    
    //spi dac
    cs_lo(ch);
    buf=((1200-4*ampl/g)*dac_res/dac_fs)<<2;    //(1/res)*(1200/3)*(3-ampl*2/(200*g))
    spi.write(buf);
    cs_hi(ch);

    //i2c dac
    buf=((1200-4*ampl/g)*dac_res/dac_fs2);    //(1/res)*(1200/3)*(3-ampl*2/(200*g))
    set[0]=buf>>8;
    set[1]=buf&0xff;
    if(ch==1){
        i2c.write(dac_addr,set,2);  //ch1 dac
    }else{
        i2c.write(dac_addr+0x2,set,2);  //ch2 dac
    }
}

char char_read(){
    char local_buf[1];          //local buffer
    uart0.read(local_buf,1);    //1-char read
    return local_buf[0];        //return 1-char
}
float char2flac(){
    char temp[1],local_buf[20];          //local buffer
    uint8_t i;
    for(i=0;i<sizeof(local_buf);++i) local_buf[i]='\0'; //init local buf
    i=0;
    while(true){
        temp[0]=char_read();
        if(temp[0]==',') break; //',' is delimiter
        local_buf[i]=temp[0];
        ++i;
    }
    return atof(local_buf);
}
void parser(){
    uint8_t i=0;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
}
