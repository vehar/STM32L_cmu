#include "gps.h"

unsigned char standby[]={"$PMTK161,0*28\x0D\x0A"};
unsigned char always_locate_mode[]={"$PMTK255,8*23\x0D\x0A"};
unsigned char return_locate_mode[]={"$PMTK001,255,3*35\x0D\x0A"};
unsigned char normal_mode[]={"$PMTK255,0*2B\x0D\x0A"};
unsigned char return_normal_mode[]={"$PMTK001,255,3*35\x0D\x0A"};
unsigned char mode[]={"$PMTK251,9600*17\x0D\x0A"};

unsigned char pereodic_mode[]={"$PMTK225,2,3000,12000,18000,72000*15\x0D\x0A"};
unsigned char rmc_mode[]={"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\x0D\x0A"};//
unsigned char ret_rmc[18]={"$PMTK001,314,3*36"};
unsigned char gps_buf[13][11];
unsigned char gps_xor[17];



unsigned char gps_time[6];
unsigned char gps_status;
unsigned char gps_lat[9];
unsigned char gps_n_s;
unsigned char gps_long[10];
unsigned char gps_e_w;
unsigned char gps_speed[4];
unsigned char gps_course[4];
unsigned char gps_date[6];
unsigned char gps_mode;
unsigned char gps_check[3];
unsigned char gps_end[2];



uint8_t gps_state=0, gps_fl;


void gps_on()
{
  GPIO_InitTypeDef gpio_cfg;
	
  GPIO_StructInit(&gpio_cfg);


  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
	gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpio_cfg.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &gpio_cfg);	
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);	
}



uint8_t lat_return(uint8_t i)
{
return 	gps_lat[i];
}
uint8_t n_s_return()
{
return 	gps_n_s;
}
uint8_t long_return(uint8_t i)
{
return 	gps_long[i];
}
uint8_t e_w_return()
{
return 	gps_e_w;
}




uint8_t time_return(uint8_t i)
{
return 	gps_time[i];
}

void gps_parser()
{
uint8_t i;

//************
for(i=0;i<6;i++)
{
gps_time[i]= gps_buf[0][i];	
}	
//************

gps_status = gps_buf[1][0];

//************
for(i=0;i<9;i++)
{
gps_lat[i]= gps_buf[2][i];	
}	
//************
gps_n_s = gps_buf[3][0];

//************
for(i=0;i<10;i++)
{
gps_long[i]= gps_buf[4][i];	
}	
//************
gps_e_w = gps_buf[5][0];
//************
for(i=0;i<4;i++)
{
	gps_speed[i]= gps_buf[6][i];	
}	
//************
for(i=0;i<4;i++)
{
	gps_course[i]= gps_buf[7][i];	
}	
//************
for(i=0;i<6;i++)
{
	gps_date[i]= gps_buf[8][i];	
}	
//************
gps_mode = gps_buf[11][0];
//************
gps_end[0] = gps_buf[12][0];
gps_end[1] = gps_buf[12][1];
//************

//************
}

void gps_init()
{
uint8_t i;
gps_fl=0;
	
	Usart1_Send_String(rmc_mode);
	gps_state=1;	
	delay();
}

void gps()
{ 
uint8_t x,y;
	
while(reciev_uart()!='$');	
while(reciev_uart()!='G');		
while(reciev_uart()!='P');		
while(reciev_uart()!='R');		
while(reciev_uart()!='M');		
while(reciev_uart()!='C');		
while(reciev_uart()!=',');	

x=0;
y=0;

	
while(gps_buf[12][2]!=0x0d)	
{
gps_buf[x][y]=reciev_uart();	
	
	if(gps_buf[x][y]==','||gps_buf[x][y]=='*')
	  {
	  gps_buf[x][y]=0;
	  x++;	
	  y=0;		
	  }
		
  else
    {
	  y++;
    }				
}	
	
gps_buf[12][2]=0x00;
gps_parser();
}





