/*

Radio wave clock Adjuster 
   By Hideaki YOKOKAWA (JP1BHH)
-----
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>

------------------------------

 Radio-wave clock adjuster(LF 40Khz or 60Khz in Japan)
 Protocol for its adjuster.
 Radio Modulation sign: A1D

change dates..

2019-10-26 to  2019-11-17

Addtional commnents

to compile

gcc -Wall -pthread -o prog prog.c -lpigpio -lrt

using pigpio library.

http://abyz.me.uk/rpi/pigpio/cif.html

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <pigpio.h> 
#include <unistd.h> 
#include <ctype.h> 
#include <dirent.h> 
#include <math.h> 
#include <fcntl.h> 
#include <assert.h> 
#include <sys/mman.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <signal.h> 
#include <malloc.h>
#include <sys/time.h>
#include <time.h>

#define MARKER 200 
#define BIT1 500 
#define BIT0 800 
#define OUT 4 
#define FREQ 40000 


void key_on()
{
int ret;
ret=gpioPWM(OUT, 128);

}

void key_off()
{
int ret;
ret=gpioPWM(OUT, 0);

}


  void dsend_wait(long int msec)
{
 long int wait_s,wait_l;
struct timeval s,l;
gettimeofday(&s, NULL);
   wait_s=((long int)s.tv_sec*1000 +(long int) s.tv_usec/1000);
for (;;) {sleep(0);
gettimeofday(&l, NULL);
   wait_l=((long int)l.tv_sec*1000 + (long int) l.tv_usec/1000);
if (msec <= (wait_l-wait_s) ) break;
//   printf("%d %d ",msec,wait_l-wait_s);
     }
  }

void dsend_1bits(long int msec)
{
 long int wait_1sec;
    wait_1sec =1000 - msec;
       key_on();
     dsend_wait(msec);
   key_off();
 dsend_wait(wait_1sec);
}

// send 1sec (1bit)
void sendbit(unsigned char b)
{  
    if (b == 3) {
     dsend_1bits(MARKER);
  //   printf("M\n");
    }else if (b == 1) {
       dsend_1bits(BIT1);
 //    printf("1\n");
    } else  {
     dsend_1bits(BIT0);
//   printf("0\n");
    }
}


// time code (JJY)

// detect Bits

char detect_bit(int *data,char cmp)
{
  int dbit;
      dbit= *data/cmp;
        *data=*data-dbit*cmp;
        if (dbit) return 1; else return 0;
}

//send code to Radio-wave-clock
//
//minutes
void send_min(char bit[])
{
  char i;
 sendbit(3);
    for (i=0;i<3;i++)sendbit(bit[i]);
 sendbit(0);
     for (i=3;i<7;i++)sendbit(bit[i]);

}

//hours
void send_hours(char bit[])
{
  char i;
 sendbit(3);
  sendbit(0);
  sendbit(0);
    for (i=0;i<2;i++)sendbit(bit[i]);
 sendbit(0);
     for (i=2;i<6;i++)sendbit(bit[i]);
 
}

//year of day
void send_yday(char bit[],char par[])
{
   char i;
 sendbit(3);
  sendbit(0);
  sendbit(0);
    for (i=0;i<2;i++)sendbit(bit[i]);
 sendbit(0);
     for (i=2;i<6;i++)sendbit(bit[i]);
  sendbit(3);
      for (i=6;i<10;i++)sendbit(bit[i]);
   sendbit(0);
  sendbit(0);
 
 //Parity
   sendbit(par[0]);
  sendbit(par[1]);
 //temp bit
   sendbit(0);
}
 

//year
void send_year(char bit[])
{
   char i;
 sendbit(3);
  sendbit(0);
    for (i=0;i<8;i++)sendbit(bit[i]);

}

//weekday

void send_wday(char bit[])
{
    char i;
 sendbit(3);
    for (i=0;i<3;i++)sendbit(bit[i]);
   sendbit(0);
   sendbit(0);
   sendbit(0);
   sendbit(0);
   sendbit(0);
   sendbit(0);
   sendbit(3);
}


void parity(char bmin[],char bhour[],char par[])
{
   char i;
      par[0]=0;
    for (i=0;i<7;i++)par[0]=par[0]^bmin[i];
      par[1]=0;
   for (i=0;i<6;i++)par[1]=par[1]^bhour[i];

}



void detect(char bit[],char start,int data)
{
      char ret;
         char i,k;
 k=0;
       int cmp[]={200,100,80,40,20,10,8,4,2,1};
   for (i=start;i<10;i++){
       bit[k]=detect_bit(&data,cmp[i]);
       k++; }
}


///get year yday,wday,hour,min
int get_daytime (int *year,int *yday,int *wday,int *hour, int *min,char ttime[])
{
       time_t now;
    struct tm *tm;
    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        printf ("Error extracting time stuff\n");
        return -1;
    }
 /// year of 20xx only
              *year=tm->tm_year-100;
              *yday=tm->tm_yday;
              *wday=tm->tm_wday,
              *hour=tm->tm_hour;
              *min=tm->tm_min;
     sprintf(ttime,"%02d-%03d %02d:%02d %d",*year,*yday,*hour,*min,*wday);
    return 0;
}


 void loop0sec(void)
 {
         time_t now;
    struct tm *tm;
     int sec;
  for (;;){
    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        printf ("Error extracting time stuff\n");
        return ;
    }
    sleep(0);
             sec=tm->tm_sec;
               if (sec == 0) break;
      }
 }

int main(int argc, char *argv[])
{
static  int year,yday,wday,hour,min;
static  int ret;
static  char byear[9],byday[11],bwday[4],bhour[7],bmin[8];
static  char par[2];
static char otime[32];

ret=gpioInitialise();
  if (ret < 0) exit(-1);

  ret=gpioSetMode(OUT, PI_OUTPUT);
ret=gpioWrite(OUT,0);
ret=gpioSetPWMfrequency(OUT, FREQ);
ret=gpioPWM(OUT, 0);
 // waiting time for 1st 0sec
for (;;){
 printf("Waiting loop \n");
 loop0sec();
 // get paras
ret= get_daytime (&year,&yday,&wday,&hour,&min,otime);
        detect(byear,2,(char)year);
	          detect(byday,0,(char)yday);
        detect(bwday,7,(char)wday);
      detect(bhour,4,(char)hour);
   detect(bmin,3,(char)min);
 
 parity(bmin,bhour,par);

   printf("Starting send %s\n",otime);
   //send 1 min.
    send_min(bmin);
   send_hours(bhour);
    send_yday(byday,par);
   send_year(byear);
   send_wday(bwday);
   }
gpioTerminate();
  return 0;
}
