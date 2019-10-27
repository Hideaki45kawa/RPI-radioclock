/*

Radio wave clock Adjuster 
   By Hideaki YOKOKAWA (JP1BHH)

License:
    This program is free software: you can redistribute it and/or modify

    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Old credits:
  Credits goes to Oliver Mattos and Oskar Weigl who implemented PiFM [1]
  based on the idea of exploiting RPi DPLL as FM transmitter. Dan MD1CLV
  combined this effort with WSPR encoding algorithm from F8CHK, resulting  
  in WsprryPi a WSPR beacon for LF and MF bands. Guido PE1NNZ extended 
  this effort with DMA based PWM modulation of fractional divider that was 
  part of PiFM, allowing to operate the WSPR beacon also on HF and VHF bands.
  In addition time-synchronisation and double amount of power output was 
  implemented. 

Code Writter Add.(Credits) 
  Credis goes to James Peroulas,wrirter of WsprrPi[2],based on idea of frequency generarion and frequeency shift.
  [1] PiFM code from http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
  [2] WsprrPi Code from  https://github.com/JamesP6000/WsprryPi

Reference documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
  http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
  http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
  https://github.com/mgottschlag/vctools/blob/master/vcdb/cm.yaml
  https://www.kernel.org/doc/Documentation/vm/pagemap.txt

------------------------------
Change Log. *For GPL lisence* 

 Radio-wave clock adjuster(LF 40Khz or 60Khz in Japan)
 Protocol for its adjuster.
 Radio Modulation sign: A1D

change dates..

2019-10-26 to  2019-10-**
 
Addtional commnents 

to compile

gcc 
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

#define F_XTAL       (19229581.050215044276577479844352)             // calibrated 19.2MHz XTAL frequency 
#define F_PLLD_CLK   (26.0 * F_XTAL)                                 // 500MHz PLLD reference clock 

#define N_ITER  1400  // number of PWM operations per symbol; larger values gives less spurs at the cost of frequency resolution; e.g. use 22500 for HF usage up to 30MHz, 12000 up to 50MHz, 1600 for VHF usage up to 144 Mhz, F_PWM_CLK needs to be adjusted when changing N_ITER 
//#define F_PWM_CLK    (31500000.0)   // 31.5MHz PWM clock   use with N_ITER=22500
#define F_PWM_CLK    (33970588.235294117647058823529413)   // 31.5MHz calibrated PWM clock   use with N_ITER=1400

#define WSPR_SYMTIME (8192.0/12000.0)  // symbol time

#define POLYNOM_1 0xf2d05351    // polynoms for
#define POLYNOM_2 0xe4613c47    // parity generator

/* RF code: */

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)


// MARKER BIT1 BIT0

#define MARKER 200
#define BIT1 500
#define BIT0 800

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio = NULL;
volatile unsigned *allof7e = NULL;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13) // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)
#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define PADS_GPIO_0_27  (0x7e10002c)
#define CM_GP0DIV (0x7e101074)
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)
#define PWMBASE  (0x7e20C000) /* PWM controller */

struct GPCTL {
    char SRC         : 4;
    char ENAB        : 1;
    char KILL        : 1;
    char             : 1;
    char BUSY        : 1;
    char FLIP        : 1;
    char MASH        : 2;
    unsigned int     : 13;
    char PASSWD      : 8;
};


void getRealMemPage(void** vAddr, void** pAddr) {
    void* a = (void*)valloc(4096);

    ((int*)a)[0] = 1;  // use page to force allocation.

    mlock(a, 4096);  // lock into ram.

    *vAddr = a;  // yay - we know the virtual address

    unsigned long long frameinfo;

    int fp = open("/proc/self/pagemap", 'r');
    lseek(fp, ((int)a)/4096*8, SEEK_SET);
    read(fp, &frameinfo, sizeof(frameinfo));

    *pAddr = (void*)((int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr) {

    munlock(vAddr, 4096);  // unlock ram.

    free(vAddr);
}

struct CB {
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int RES1;
    volatile unsigned int RES2;

};

struct DMAregs {
    volatile unsigned int CS;
    volatile unsigned int CONBLK_AD;
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int DEBUG;
};

struct PageInfo {
    void* p;  // physical address
    void* v;   // virtual address
};

struct PageInfo constPage;
struct PageInfo instrPage;
struct PageInfo instrs[1024];

double fracs[1024];

void txon()
{
    if(allof7e == NULL){
      allof7e = (unsigned *)mmap(
                  NULL,
                  0x01000000,  //len
                  PROT_READ|PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  0x20000000  //base
              );
      if ((int)allof7e==-1) exit(-1);
    }

    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

    // Set GPIO drive strength, more info: http://www.scribd.com/doc/101830961/GPIO-Pads-Control2 
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 0;  //2mA -3.4dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 1;  //4mA +2.1dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 2;  //6mA +4.9dBm
//      ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 3;  //8mA +6.6dBm(default)
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 4;  //10mA +8.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 5;  //12mA +9.2dBm
    //ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 6;  //14mA +10.0dBm
      ACCESS(PADS_GPIO_0_27) = 0x5a000018 + 7;  //16mA +10.6dBm

    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 1,0x5a};
    ACCESS(CM_GP0CTL) = *((int*)&setupword);
}

void txoff()
{
    struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a}; 
    ACCESS(CM_GP0CTL) = *((int*)&setupword); 
}

void setfreq(long freq)
{
    ACCESS(CM_GP0DIV) = (0x5a << 24) + freq;
}


void unSetupDMA(){
    printf("exiting\n");
    struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
    DMA0->CS =1<<31;  // reset dma controller
    txoff(); 
}

void handSig() {
  exit(0);
}
void setupDMATab( double centerFreq, double symOffset, double tsym, int nsym ){
   // make data page contents - it's essientially 1024 different commands for the
   // DMA controller to send to the clock module at the correct time.
  int i;
  for(i=1; i<1023; i+=3){
     double freq = centerFreq + ((double)(-511 + i))*symOffset/3.0;
     double divisor = F_PLLD_CLK/freq;
     unsigned long integer_part = (unsigned long) divisor;
     unsigned long fractional_part = (divisor - integer_part) * (1 << 12);
     unsigned long tuning_word = (0x5a << 24) + integer_part * (1 << 12) + fractional_part;
     if(fractional_part == 0 || fractional_part == 1023){
       if((-511 + i) >= 0 && (-511 + i) <= (nsym * 3)) 
         printf("warning: symbol %u unusable because fractional divider is out of range, try near frequency.\n", i/3);
     }
     ((int*)(constPage.v))[i-1] = tuning_word - 1;
     ((int*)(constPage.v))[i] = tuning_word;
     ((int*)(constPage.v))[i+1] = tuning_word + 1;
     double actual_freq = F_PLLD_CLK/((double)integer_part + (double)fractional_part/(double)(1<<12));
     double freq_corr = freq - actual_freq;
     double delta = F_PLLD_CLK/((double)integer_part + (double)fractional_part/(double)(1<<12)) - F_PLLD_CLK/((double)integer_part + ((double)fractional_part+1.0)/(double)(1<<12));
     int clocksPerIter = (int)((F_PWM_CLK/((double)N_ITER)) * tsym);
     double resolution = 2.0 * delta / ((double)clocksPerIter);
     if(resolution > symOffset ){
       printf("warning: PWM/PLL fractional divider has not enough resolution: %fHz while %fHz is required, try lower frequency or decrease N_ITER in code to achieve more resolution.\n", resolution, symOffset);
       exit(0);
     }
     fracs[i] = freq_corr/delta;
     //printf("i=%u f=%f fa=%f corr=%f delta=%f percfrac=%f int=%u frac=%u tuning_word=%u resolution=%fmHz\n", i, freq, actual_freq, freq_corr, delta, fracs[i], integer_part, fractional_part, tuning_word, resolution *1000);
   }
}

void setupDMA(){
   atexit(unSetupDMA);
   signal (SIGINT, handSig);
   signal (SIGTERM, handSig);
   signal (SIGHUP, handSig);
   signal (SIGQUIT, handSig);

   // allocate a few pages of ram
   getRealMemPage(&constPage.v, &constPage.p);
 
   int instrCnt = 0;
  
   while (instrCnt<1024) {
     getRealMemPage(&instrPage.v, &instrPage.p);
    
     // make copy instructions
     struct CB* instr0= (struct CB*)instrPage.v;
     int i; 
     for (i=0; i<4096/sizeof(struct CB); i++) {
       instrs[instrCnt].v = (void*)((int)instrPage.v + sizeof(struct CB)*i);
       instrs[instrCnt].p = (void*)((int)instrPage.p + sizeof(struct CB)*i);
       instr0->SOURCE_AD = (unsigned int)constPage.p+2048;
       instr0->DEST_AD = PWMBASE+0x18 /* FIF1 */;
       instr0->TXFR_LEN = 4;
       instr0->STRIDE = 0;
       //instr0->NEXTCONBK = (int)instrPage.p + sizeof(struct CB)*(i+1);
       instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
       instr0->RES1 = 0;
       instr0->RES2 = 0;

       if (i%2) {
         instr0->DEST_AD = CM_GP0DIV;
         instr0->STRIDE = 4;
         instr0->TI = (1<<26/* no wide*/) ;
       }

       if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (int)instrs[instrCnt].p;
       instr0++;
       instrCnt++;
     }
   }
   ((struct CB*)(instrs[1023].v))->NEXTCONBK = (int)instrs[0].p;

   // set up a clock for the PWM
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;  // Source=PLLD and disable
   usleep(1000);
//   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002000;  // set PWM div to 2, for 250MHz 
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;  // Source=PLLD and enable 
   usleep(1000);

   // set up pwm
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = 0;
   usleep(1000);
   ACCESS(PWMBASE + 0x4 /* status*/) = -1;  // clear errors
   usleep(1000);
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
   usleep(1000);
   ACCESS(PWMBASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;

   //activate dma
   struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
   DMA0->CS =1<<31;  // reset
   DMA0->CONBLK_AD=0;
   DMA0->TI=0;
   DMA0->CONBLK_AD = (unsigned int)(instrPage.p);
   DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
}


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit (-1);
    }

    /* mmap GPIO */

    // Allocate MAP block
    if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
        printf("allocation error \n");
        exit (-1);
    }

    // Make sure pointer is on 4K boundary
    if ((unsigned long)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

    // Now map it
    gpio_map = (unsigned char *)mmap(
                   gpio_mem,
                   BLOCK_SIZE,
                   PROT_READ|PROT_WRITE,
                   MAP_SHARED|MAP_FIXED,
                   mem_fd,
                   GPIO_BASE
               );

    if ((long)gpio_map < 0) {
        printf("mmap error %d\n", (int)gpio_map);
        exit (-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


}

void setup_gpios()
{
   int g;
   // Switch GPIO 7..11 to output mode

    /************************************************************************\
     * You are about to change the GPIO settings of your computer.          *
     * Mess this up and it will stop working!                               *
     * It might be a good idea to 'sync' before running this program        *
     * so at least you still have your code changes written to the SD-card! *
    \************************************************************************/

    // Set GPIO pins 7-11 to output
    for (g=7; g<=11; g++) {
        INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
        //OUT_GPIO(g);
    }

}

void strupr(char *str) 
{   while(*str) 
    { 
        *str = toupper(*str); 
        str++; 
    }
}

// For Radio-wave-clock adjuster..
// 800 msec is 0
// 500 msec is 1
// 200 msec is stopper

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
     }
  }

void dsend_1bits(long int msec)
{
 long int wait_1sec;
    wait_1sec =1000 - msec; 
    txon();
     dsend_wait(msec);
   txoff();
 dsend_wait(wait_1sec);
}
     
// send 1sec (1bit)
void sendbit(unsigned char b)
{  
    if (b == 3) {
     dsend_1bits(MARKER);
    }else if (b == 1) {
       dsend_1bits(BIT1);
    } else  {
     dsend_1bits(BIT0);
    }
}

 void dsetfreq(double centerfreq)
{
      setupDMATab(centerfreq, 0.5, 2.0, 4);
}


// time code (JJY)

// detect Bits

int detect_bit(char *data,char cmp)
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
    for (i=0;i<7;i++)sendbit(bit[i]);

}

//weekday

void send_wday(char bit[])
{
    char i;
 sendbit(3);
    for (i=0;i<2;i++)sendbit(bit[i]);
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



void detect(char bit[],char start,char data)
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
int get_daytime (int *year,int *yday,int *wday,int *hour, int *min)
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
  double centerfreq;  
 int year,yday,wday,hour,min;
  int ret;
 char byear[9],byday[11],bwday[4],bhour[7],bmin[8];
 char par[2];
    
  if (argc < 1){
   printf("Usage: radclock <send frequency(Hz)> \n");
exit(0);
}
  setup_io();
  setup_gpios(); 
  setupDMA();
  txoff();
    centerfreq = atof(argv[1]);
   dsetfreq(centerfreq);
 
 // waiting time for 1st 0sec
 loop0sec();
 printf("Start send Adjust signal\n");
   
 // get paras
ret= get_daytime (&year,&yday,&wday,&hour,&min);
        detect(byear,2,(char)year);
          detect(byday,0,(char)yday);
        detect(bwday,7,(char)wday);
      detect(bhour,4,(char)hour);
   detect(bmin,3,(char)min);
 
 parity(bmin,bhour,par);
    printf("Start send Adjust signal\n");
   
   //send 1 min.
    send_min(bmin);
   send_hours(bhour);
    send_yday(byday,par);
   send_year(byear);
   send_wday(bwday);
   
  return 0;
}
