#include<avr\io.h>
#include<util\delay.h>
#include"lcd.h"
#define powerl OCR1A
#define powerr OCR1B
#define Kp 200
#define Ki 0.2
#define Kd 120
int sensor[8];
long sensor_avarage=0;
int sensor_sum=0;
int position=0;
int proportional=0;
int integral=0;
int last_proportional=0;
int derivative=0;
int error_value=0;
int right_speed=0;
int max_speed=200;
int left_speed=0;
int set_point=0;

int motor(int pl,int pr)
{
  powerr=pr;
  powerl=pl;
}
void motor_init()
{
  ICR1=20000;
  DDRD|=(1<<PD4)|(1<<PD5);
  //  the pins pd4 and pd5 for the PWM
  DDRD|=(1<<PD0)|(1<<PD1)|(1<<PD2)|(1<<PD3);
  PORTD=(1<<PD0)|(0<<PD1)|(1<<PD2)|(0<<PD3);
  PORTD|=(0<<PD4)|(0<<PD5);
  TCCR1A|=(0<<COM1A0)|(1<<COM1A1)|(0<<COM1B0)|(1<<COM1B1)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(1<<WGM10);
  TCCR1B|=(0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
}
void sensor_init()
{
  //Port A pins as input for sensors
  DDRA=0X00;
  //Enable internal pull ups
  PORTA=0XFF;
}
void lcd_init()
{
  LCDInit(LS_BLINK|LS_ULINE);
  LCDClear();
}
int sensor_read(int j)
{
 if(j==0)
 {

 return PINA0;
 }
 else if(j==1)
 {
     return(PINA1);
}
else if(j==2)
{
    return(PINA2);
}
else if(j==3)
{
    return(PINA3);
}
else if(j==4)
{
    return(PINA4);
}
else if(j==5)
{
    return(PINA5);
}
else if(j==6)
{
    return(PINA6);
}
else if(j==7)
{
    return(PINA7);
}
}
void sensor_cal()
{
    int i=0;
    for(i=0;i<8;i++)
    {
        sensor[i]=sensor_read(i);
        sensor_avarage=(int)(sensor_avarage+sensor[i]*i*1000);
        sensor_sum+=(int)(sensor[i]);



    }


}
 void pid_calc()
    {
        position=(int)(sensor_avarage/sensor_sum);

    }

void main()
{

  sensor_init();
  motor_init();
  lcd_init();
  while(1)
  {
      sensor_cal();
      pid_calc();
	  LCDWriteStringXY(3,0,"SET POINT");
	  LCDWriteIntXY(4,1,position,5);



  }
}
