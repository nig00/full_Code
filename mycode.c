 #include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>

#include <wiringPi.h>           //WiringPi headers
#include <lcd.h>                //LCD headers from WiringPi
#include <wiringPiSPI.h>
//Pin numbers below are the WiringPi pin numbers
#define LCD_RS  27               //Register select pin
#define LCD_E   28               //Enable Pin
#define LCD_D4  21              //Data pin 4
#define LCD_D5  22               //Data pin 5
#define LCD_D6  23               //Data pin 6
#define LCD_D7  24               //Data pin 7
#define BAUDRATE1 B115200
#define BAUDRATE2 B9600

#include <stdio.h>
#include <wiringPi.h>
#define	heart	29
#define	BUZZ	26
////////////////////////////////////////////CHANEL0/////////////////////////////
#define CS_MCP3208  8       // BCM_GPIO8

#define SPI_CHANNEL 0
#define SPI_SPEED   100000  // !! Start low here and if all works try to increase if needed on a breadboard I could go upto about 750000 
/////////////////////     READ ADC VALUES  ///////////////////////////////////
int read_mcp3208_adc(unsigned char adcChannel)
{
unsigned char buff[3];
int adcValue = 0;
buff[0] = 0x06 | ((adcChannel & 0x07) >> 7);
buff[1] = ((adcChannel & 0x07) << 6);
buff[2] = 0x00;
digitalWrite(CS_MCP3208, 0);  // Low : CS Active
wiringPiSPIDataRW(SPI_CHANNEL, buff, 3);
buff[1] = 0x0F & buff[1];
adcValue = ( buff[1] << 8) | buff[2];
digitalWrite(CS_MCP3208, 1);  // High : CS Inactive
return adcValue;
}
/////////////////////////////////////////////////////////////////////////////// 
int main (void)
{

char ch2[100];
int fd;
int lcd; 
int a=0; 


int jj=0;
int kk=0;
int kkk=0;

struct termios newtio;
int adc1Channel = 0;
int adc1Value   = 0;

int adc2Channel = 1;
int adc2Value   = 0;
 
int adc3Channel = 2;
int adc3Value   = 0;

int adc4Channel = 3;
int adc4Value   = 0;
  
if(wiringPiSetup() == -1)
{
fprintf (stdout, "Unable to start wiringPi: %s\n", strerror(errno));
return 1 ;
}

if(wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1)
{
fprintf (stdout, "wiringPiSPISetup Failed: %s\n", strerror(errno));
return 1 ;
}

pinMode (heart, INPUT) ;


pinMode (BUZZ, OUTPUT) ;


digitalWrite (BUZZ, HIGH) ;	// Off



pinMode(CS_MCP3208, OUTPUT);
//Handle for LCD
//Initialise LCD(int rows, int cols, int bits, int rs, int enable, int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7)
if (lcd = lcdInit (2, 16,4, LCD_RS, LCD_E ,LCD_D4 , LCD_D5, LCD_D6,LCD_D7,0,0,0,0))
{
printf ("lcdInit failed! \n");
return -1 ;
}
//////////////////////////////////////////////////////       
fd = open("/dev/ttyAMA0",O_RDWR | O_NONBLOCK);
printf("Port Opened1\n");
bzero(&newtio,sizeof(newtio));
newtio.c_cflag = BAUDRATE1 | CS8 | CLOCAL | CREAD;
newtio.c_cflag &= ~PARENB;
newtio.c_cflag &= ~PARODD;
newtio.c_cflag &= ~CSTOPB;
newtio.c_iflag = IGNPAR;
newtio.c_oflag = 0;
newtio.c_lflag = 0;       
newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
newtio.c_cc[VMIN]     = 255;   /* blocking read until 255 chars received */
tcflush(fd,TCIFLUSH);
tcsetattr(fd,TCSANOW,&newtio);
write(fd,"AT\r\n",4);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT+CIOBAUD=9600\r\n",17);usleep(600000);usleep(600000);usleep(600000);usleep(600000);

//Clear the display
lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "PATIENT HEALTH");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column
lcdPuts(lcd, "MONITORING ");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdClear(lcd);  
lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "SYSTEM USING ");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column
lcdPuts(lcd, "RASPBERRY PI");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdClear(lcd);  
lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "AND IOT");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column
lcdPuts(lcd, "WIFI MODEM");  //Print the text on the LCD at the current cursor postion
usleep(600000);usleep(600000);usleep(600000);
lcdClear(lcd);

lcdClear(lcd);  
lcdPosition(lcd,0,0);           
lcdPuts(lcd, "plz connect the");
usleep(600000);usleep(600000);usleep(600000);
lcdPosition(lcd,0,1);           
lcdPuts(lcd, "SENSORS TO BODY");
usleep(600000);usleep(600000);usleep(600000);
usleep(600000);usleep(600000);usleep(600000);
lcdClear(lcd);
//////////////////////////////////////////////////////       
fd = open("/dev/ttyAMA0",O_RDWR | O_NONBLOCK);
printf("Port Opened2\n");
bzero(&newtio,sizeof(newtio));
newtio.c_cflag = BAUDRATE2 | CS8 | CLOCAL | CREAD;
newtio.c_cflag &= ~PARENB;
newtio.c_cflag &= ~PARODD;
newtio.c_cflag &= ~CSTOPB;
newtio.c_iflag = IGNPAR;
newtio.c_oflag = 0;
newtio.c_lflag = 0;       
newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
newtio.c_cc[VMIN]     = 255;   /* blocking read until 255 chars received */
tcflush(fd,TCIFLUSH);
tcsetattr(fd,TCSANOW,&newtio);
//////////////////////////////////////////////////////////////////////////////
system("clear");
printf("\n\nPATIENT HELTH MNTR USING RASPBERRYPI--IOT.\n\n");
lcdPosition(lcd,0,0);          
lcdPuts(lcd, "WIFI START..."); 
write(fd,"AT\r\n",4);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT\r\n",4);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT\r\n",4);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"ATE0\r\n",6);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT+CIPMUX=0\r\n",13);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT+CWMODE=1\r\n",13);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
write(fd,"AT+CWJAP=\"UPC556503\",\"PAPUERYB\"\r\n",33);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);
usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);usleep(600000);

while(1)
{
a=a+1;jj=0;
lcdClear(lcd);  
/////////////////////TEMP//////////////////////////////////////////
adc1Value = read_mcp3208_adc(adc1Channel);
printf ("BODY TEMPERATURE\n") ;
printf("\tTEMP = %.3f\n", (((3.3/4096) * adc1Value)*100));
lcdPosition(lcd,0,0);
adc1Value=(((3.3/4096) * adc1Value)*100);
lcdPrintf(lcd,"TEMP= %02u",adc1Value);  
lcdPosition(lcd,9,0);
lcdPuts(lcd, "deg");
usleep(20000);
usleep(600000);usleep(600000);usleep(600000);
////////////////////////////////////////
if(adc1Value>50)
{
digitalWrite (BUZZ, LOW) ;// On
}

if(adc1Value<50)
{
digitalWrite (BUZZ, HIGH) ;// Off
}

/////////////////////////////////////////////////
////////////////////ECG//////////////////////////////////////////
adc2Value = read_mcp3208_adc(adc2Channel);
printf ("ECG MONITORING\n") ;
printf("\tECG = %.3f\n", ((3.3/4096) * adc2Value)*100 );
lcdPosition(lcd,0,1);
adc2Value=(((3.3/4096) * adc2Value)*100);
lcdPrintf(lcd,"ECG = %02u",adc2Value);
usleep(200000);

lcdClear(lcd); 

printf("\n\n MEMS SENOSR\n\n");

adc3Value = read_mcp3208_adc(adc3Channel);
lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
adc3Value=((((3.3/4096) * adc3Value)*100));
lcdPrintf(lcd,"x=%03u",adc3Value);  //Print the text on the LCD at the current cursor postion
printf("adc3Value = %03u", adc3Value);
usleep(200000);

adc4Value = read_mcp3208_adc(adc4Channel);
lcdPosition(lcd,0,1);           //Position cursor on the first line in the first column
adc4Value=((((3.3/4096) * adc4Value)*100));
lcdPrintf(lcd,"y=%03u",adc4Value);  //Print the text on the LCD at the current cursor postion
printf("adc4Value = %03u", adc4Value);
usleep(200000);



if( ( (adc4Value >= 155) & (adc4Value <= 170 )) )
{
printf ("NORMAL \n") ;

lcdPosition(lcd,9,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "NORMAL");  //Print the text on the LCD at the current cursor postion
usleep(600000);
digitalWrite (BUZZ, LOW) ;	// Off
}


if( ( (adc4Value >= 125) & (adc4Value <= 140 )) )
{
printf ("FRONT FALLEN \n") ;
lcdPosition(lcd,9,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "FORNT ");  //Print the text on the LCD at the current cursor postion
usleep(60000);
lcdPosition(lcd,7,1);           //Position cursor on the first line in the first column
lcdPuts(lcd, "FALLEN");  //Print the text on the LCD at the current cursor postion
digitalWrite (BUZZ, HIGH) ;	// Off
usleep(600000);
}

if( ( (adc4Value >= 175) & (adc4Value <= 198 )) )
{
printf ("BACK FALLEN \n") ;
lcdPosition(lcd,9,0);           //Position cursor on the first line in the first column
lcdPuts(lcd, "BACK");  //Print the text on the LCD at the current cursor postion
usleep(60000);
lcdPosition(lcd,7,1);           //Position cursor on the first line in the first column
lcdPuts(lcd, "FALLEN");  //Print the text on the LCD at the current cursor postion
digitalWrite (BUZZ, HIGH) ;	// Off
usleep(600000);
}



/////////////////////////////////////////////////////////////////////////////
printf ("heartbeat checking \n") ;
if(digitalRead (heart) == HIGH)	
{
for(kk=0;kk<=6800000;kk++)
{
if(digitalRead (heart) == LOW)	
{
jj=jj+1;
printf("\tbeats check = %02u\n", (jj));
usleep(600000);
}
}
if(jj==0)
{
printf ("heart beat clip not connected\n") ;
lcdClear(lcd);  
lcdPosition(lcd,0,0);           
lcdPuts(lcd, "plz connect the"); 
usleep(600000);usleep(600000);usleep(600000);
lcdPosition(lcd,0,1);          
lcdPuts(lcd, "HEARTBEAT SENSOR");  
usleep(600000);
usleep(6000);
lcdClear(lcd);  
lcdPosition(lcd,0,0);
lcdPrintf(lcd,"heart beat= %02u",jj);  
printf("\tHEARTBEAT = %02u\n", (jj));
usleep(600000);
jj=0;
lcdClear(lcd);
}
if(jj>0)
{
jj=70+jj;
printf("\tHEART BEAT = %02u\n", (jj));
usleep(6000);
lcdClear(lcd);  
lcdPosition(lcd,0,0);
lcdPrintf(lcd,"heart beat= %02u",jj);  
usleep(600000);
}
}
///////////////////////////////////////////////////////////////////////////
if(a==2)
{
printf("\n\n  patient data sending to webpage \n\n");
lcdPosition(lcd,0,0);           
lcdPuts(lcd, "sending data");  
usleep(600000);

write(fd,"AT+CIPSTART=\"TCP\",\"www.rpihealthsystem.com\",80\r\n",48);usleep(600000);
write(fd,"AT+CIPSEND=93\r\n",15);usleep(600000);
write(fd,"GET /health/put_data.php",24);usleep(600000);

write(fd,"?temp=",6);usleep(600000);
serialPutchar(fd,(adc1Value/100) + 0x30,1);usleep(30000);
serialPutchar(fd,((adc1Value/10)%10) + 0x30,1);usleep(30000);
serialPutchar(fd,(adc1Value%10) + 0x30,1);usleep(30000);

write(fd,"&hbt=",5);usleep(600000);
serialPutchar(fd,(jj/100) + 0x30,1);usleep(30000);
serialPutchar(fd,((jj/10)%10) + 0x30,1);usleep(30000);
serialPutchar(fd,(jj%10) + 0x30,1);usleep(30000);

write(fd,"&ecg=",5);usleep(600000);
serialPutchar(fd,(adc2Value/100) + 0x30,1);usleep(30000);
serialPutchar(fd,((adc2Value/10)%10) + 0x30,1);usleep(30000);
serialPutchar(fd,(adc2Value%10) + 0x30,1);usleep(30000);

write(fd," HTTP/1.1\r\n",11);usleep(30000);usleep(30000);usleep(30000);

write(fd,"Host: www.rpihealthsystem.com\r\n",31);usleep(30000);
usleep(30000);usleep(30000);
write(fd,"\r\n",2);usleep(30000);usleep(30000);usleep(30000);
usleep(30000);usleep(30000);usleep(30000);
usleep(30000);usleep(30000);usleep(30000);
usleep(30000);usleep(30000);usleep(30000);
usleep(30000);usleep(30000);usleep(30000);
lcdClear(lcd);
lcdPosition(lcd,0,0);          
lcdPuts(lcd, "WIFI SENT...."); 
usleep(60000);
usleep(60000);
lcdClear(lcd);
lcdPosition(lcd,0,0);          
lcdPuts(lcd, "WIFI CLOSE...."); 
a=0;
}
usleep(100000);
}
return 0;
}