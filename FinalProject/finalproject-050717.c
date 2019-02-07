#include <std.h>
#include <log.h>
#include <clk.h>
#include <gbl.h>
#include <bcache.h>

#include <mem.h> // MEM_alloc calls
#include <que.h> // QUE functions
#include <sem.h> // Semaphore functions
#include <sys.h> 
#include <tsk.h> // TASK functions
#include <math.h> 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines


#include "projectinclude.h"
#include "c67fastMath.h" // sinsp,cossp, tansp
#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "evmomapl138_spi.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "ladar.h"
#include "xy.h"
#include "MatrixMath.h"

#include "pQueue.h"

#define FEETINONEMETER 3.28083989501312

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

volatile uint32_t index;

// test variables
extern float enc1;  // Left motor encoder
extern float enc2;  // Right motor encoder
extern float enc3;
extern float enc4;
extern float adcA0;  // ADC A0 - Gyro_X -400deg/s to 400deg/s  Pitch
extern float adcB0;  // ADC B0 - External ADC Ch4 (no protection circuit)
extern float adcA1;  // ADC A1 - Gyro_4X -100deg/s to 100deg/s  Pitch
extern float adcB1;  // ADC B1 - External ADC Ch1
extern float adcA2;  // ADC A2 -	Gyro_4Z -100deg/s to 100deg/s  Yaw
extern float adcB2;  // ADC B2 - External ADC Ch2
extern float adcA3;  // ADC A3 - Gyro_Z -400deg/s to 400 deg/s  Yaw
extern float adcB3;  // ADC B3 - External ADC Ch3
extern float adcA4;  // ADC A4 - Analog IR1
extern float adcB4;  // ADC B4 - USONIC1
extern float adcA5;  // ADC A5 -	Analog IR2
extern float adcB5;  // ADC B5 - USONIC2
extern float adcA6;  // ADC A6 - Analog IR3
extern float adcA7;  // ADC A7 - Analog IR4
extern float compass;
extern float switchstate;

float vref = 0;
float turn = 0;

int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float LVvalue1 = 0;
float LVvalue2 = 0;
int new_LV_data = 0;

int newnavdata = 0;
float newvref = 0;
float newturn = 0;

extern sharedmemstruct *ptrshrdmem;

float x_pred[3][1] = {{0},{0},{0}};					// predicted state

//more kalman vars
float B[3][2] = {{1,0},{1,0},{0,1}};			// control input model
float u[2][1] = {{0},{0}};			// control input in terms of velocity and angular velocity
float Bu[3][1] = {{0},{0},{0}};	// matrix multiplication of B and u
float z[3][1];							// state measurement
float eye3[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// 3x3 identity matrix
float K[3][3] = {{1,0,0},{0,1,0},{0,0,1}};		// optimal Kalman gain
#define ProcUncert 0.0001
#define CovScalar 10
float Q[3][3] = {{ProcUncert,0,ProcUncert/CovScalar},
				 {0,ProcUncert,ProcUncert/CovScalar},
				 {ProcUncert/CovScalar,ProcUncert/CovScalar,ProcUncert}};	// process noise (covariance of encoders and gyro)
#define MeasUncert 1
float R[3][3] = {{MeasUncert,0,MeasUncert/CovScalar},
				 {0,MeasUncert,MeasUncert/CovScalar},
				 {MeasUncert/CovScalar,MeasUncert/CovScalar,MeasUncert}};	// measurement noise (covariance of LADAR)
float S[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance
float S_inv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance matrix inverse
float P_pred[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// predicted covariance (measure of uncertainty for current position)
float temp_3x3[3][3];				// intermediate storage
float temp_3x1[3][1];				// intermediate storage
float ytilde[3][1];					// difference between predictions

// deadreckoning
float vel1 = 0,vel2 = 0;
float vel1old = 0,vel2old = 0;
float enc1old = 0,enc2old = 0;

// SETTLETIME should be an even number and divisible by 3
#define SETTLETIME 6000
int settlegyro = 0;
float gyro_zero = 0;
float gyro_angle = 0;
float old_gyro = 0;
float gyro_drift = 0;
float gyro = 0;
int gyro_degrees = 0;
float gyro_radians = 0.0;
float gyro_x = 0,gyro_y = 0;
float gyro4x_gain = 1;

int statePos = 1;	// index into robotdest
int robotdestSize = 8;	// number of positions to use out of robotdest
pose robotdest[8];	// array of waypoints for the robot

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
extern pose ROBOTps;
extern pose LADARps;
extern float newLADARdataX[LADAR_MAX_DATA_SIZE];
extern float newLADARdataY[LADAR_MAX_DATA_SIZE];
float LADARdataX[LADAR_MAX_DATA_SIZE];
float LADARdataY[LADAR_MAX_DATA_SIZE];
extern int newLADARdata;

// Optitrack Variables
int trackableIDerror = 0;
int firstdata = 1;
volatile int new_optitrack = 0;
volatile float previous_frame = -1;
int frame_error = 0;
volatile float Optitrackdata[OPTITRACKDATASIZE];
pose OPTITRACKps;
float temp_theta = 0.0;
float tempOPTITRACK_theta = 0.0;
volatile int temp_trackableID = -1;
int trackableID = -1;
int errorcheck = 1;

int counter = 0;
float var1 = 0;
float var2 = 0;
float var3 = 0;
float var4 = 0;


// A-star initializations
node_t neighbors[4];		//used to get a list of neighboring nodes
int currDist = 0;	//distance traveled from starting point
int pathLen = 0;			//used to keep track of number of points in reconstructed path
int pathRow[400];			//reconstructed path in reverse order
int pathCol[400];
//int astarPath[400][400];
char map[176];
int mapRowSize = 16;
int mapColSize = 11;
dictElem_t nodeTrack[176];		//to track location of nodes in the heap, and to track parents of nodes technically only needs to be mapRowSize*mapColsize long
heap_t openSet, closedSet;
char map[176] = 		//16x11
{	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0' 	};

typedef struct obs_t {
   int x;
   int y;
   int r1;
   int c1;
   int r2;
   int c2;
   int r3;
   int c3;
   int tally;
   int mapped;
} obs;

obs obs_t[57];

//{	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'x', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', 'x', 'x',
//	'x', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', 'x', 'x',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'0', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', '0',
//	'0', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', '0',
//	'0', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', '0',
//	'0', 'x', '0', '0', '0', '0', '0', '0', '0', 'x', '0',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
//	'0', 'x', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', '0',
//	'0', 'x', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', '0',
//	'0', 'x', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', '0',
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0' 	};

int obstaclemap[176] =
{	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	};

int indexpos = 0;

int current_index = 0;
int astarrunning = 0;
int startdelay = 500;
int firsttime = 0;
int endcol = 0;
int endrow = 0;
int startrow = 0;
int startcol = 0;
float obsx = 0;
float obsy = 0;

extern float blue_rcenter;
extern float blue_ccenter;
extern int blue_numpixs;
extern float orange_rcenter;
extern float orange_ccenter;
extern int orange_numpixs;

//extern int new_r_centroid;
//extern int new_c_centroid;
//extern int new_num_pixels;
//extern int new_vision_data;
extern volatile float object_x;
extern volatile float object_y;
extern volatile int new_coordata;
extern volatile int numpels;

extern int ifblue;

float distance_y = 0.0;

float cen_row = 0;
float cen_col = 0;
int num_pix = 0;
float kp = 0.05;

float brc = 0;
float bcc = 0;
int bpx = 0;
float grc = 0;
float gcc = 0;
int gpx = 0;

int remap = 0;
int seeball = 0;

int send_to_labview = 0;

float distance = 0.0;
int obj_x = 0;
int obj_y = 0;


float gateClose = 12.5;
float gateOpen = 9.0;
float switchBlue = 6.3; // Ready to collect or drop blue balls
float switchOrange = 8.4; // Ready to collect or drop orange balls
float Gate = gateClose; // PWM3
float Switch = switchBlue; // PWM4

pose UpdateOptitrackStates(pose localROBOTps, int * flag);


void ComWithLinux(void) {

	int i = 0;
	TSK_sleep(100);

	while(1) {

		BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
		
		if (GET_DATA_FROM_LINUX) {

			if (newnavdata == 0) {
				newvref = ptrshrdmem->Floats_to_DSP[0];
				newturn = ptrshrdmem->Floats_to_DSP[1];
				newnavdata = 1;
			}

			CLR_DATA_FROM_LINUX;

		}

		if (GET_LVDATA_FROM_LINUX) {

			if (ptrshrdmem->DSPRec_size > 256) ptrshrdmem->DSPRec_size = 256;
				for (i=0;i<ptrshrdmem->DSPRec_size;i++) {
					fromLinuxstring[i] = ptrshrdmem->DSPRec_buf[i];
				}
				fromLinuxstring[i] = '\0';

				if (new_LV_data == 0) {
					sscanf(fromLinuxstring,"%f%f",&LVvalue1,&LVvalue2);
					new_LV_data = 1;
				}

			CLR_LVDATA_FROM_LINUX;

		}

		if ((tskcount%6)==0) {
			if (GET_LVDATA_TO_LINUX) {

				if(send_to_labview==1) {
//					var1 = ROBOTps.x;
//					var2 = ROBOTps.y;
//					var3 = (vel1+vel2)/2;
//					var4 = ROBOTps.theta;
					var1 = roundf(obsx);
					var2 = roundf(obsy);
					//var3 = (vel1+vel2)/2;
					var3 = 0;
					//var4 = ROBOTps.theta;
					var4 = 0;
					send_to_labview = 0;
				}
				// Default
				//ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"1.0 1.0 1.0 1.0");
				// you would do something like this
				ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",var1,var2,var3,var4);


				for (i=0;i<ptrshrdmem->DSPSend_size;i++) {
					ptrshrdmem->DSPSend_buf[i] = toLinuxstring[i];
				}

				// Flush or write back source
				BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

				CLR_LVDATA_TO_LINUX;

			}
		}
		
		if (GET_DATAFORFILE_TO_LINUX) {
			// First make sure all scratch elements are zero
			for (i=0;i<500;i++) {
				ptrshrdmem->scratch[i] = 0;
			}
			// Write LADARdataX to scratch
			for (i=0;i<228;i++) {
				ptrshrdmem->scratch[i] = LADARdataX[i];
			}
			// Write LADARdataY to scratch
			for (i=0;i<228;i++) {
				ptrshrdmem->scratch[228+i] = LADARdataY[i];
			}
			// Flush or write back source
			BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

			CLR_DATAFORFILE_TO_LINUX;
		}


		tskcount++;
		TSK_sleep(40);
	}


}


/*
 *  ======== main ========
 */
Void main()
{

	int i = 0;

	// unlock the system config registers.
	SYSCONFIG->KICKR[0] = KICK0R_UNLOCK;
	SYSCONFIG->KICKR[1] = KICK1R_UNLOCK;

	SYSCONFIG1->PUPD_SEL |= 0x10000000;  // change pin group 28 to pullup for GP7[12/13] (LCD switches)

	// Initially set McBSP1 pins as GPIO ins
	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x88888880);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13


	//Rick added for LCD DMA flagging test
	GPIO_setDir(GPIO_BANK0, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);

	GPIO_setDir(GPIO_BANK0, GPIO_PIN0, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN1, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN2, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN3, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN4, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN5, GPIO_INPUT);  
	GPIO_setDir(GPIO_BANK0, GPIO_PIN6, GPIO_INPUT);

	GPIO_setDir(GPIO_BANK7, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN12, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN13, GPIO_INPUT); 

	GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);  
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN9, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN10, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN11, OUTPUT_HIGH);  

	CLRBIT(SYSCONFIG->PINMUX[13], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[13], 0x88888811); //Set GPIO 6.8-13 to GPIOs and IMPORTANT Sets GP6[15] to /RESETOUT used by PHY, GP6[14] CLKOUT appears unconnected

	#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

	GPIO_setDir(GPIO_BANK6, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN12, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN13, GPIO_INPUT);   


   // on power up wait until Linux has initialized Timer1
	while ((T1_TGCR & 0x7) != 0x7) {
	  for (index=0;index<50000;index++) {}  // small delay before checking again

	}

	USTIMER_init();
	
	// Turn on McBSP1
	EVMOMAPL138_lpscTransition(PSC1, DOMAIN0, LPSC_MCBSP1, PSC_ENABLE);

    // If Linux has already booted It sets a flag so no need to delay
    if ( GET_ISLINUX_BOOTED == 0) {
    	USTIMER_delay(4*DELAY_1_SEC);  // delay allowing Linux to partially boot before continuing with DSP code
    }
	   
	// init the us timer and i2c for all to use.
	I2C_init(I2C0, I2C_CLK_100K);
	init_ColorVision();	
	init_LCD_mem(); // added rick

	EVTCLR0 = 0xFFFFFFFF;
	EVTCLR1 = 0xFFFFFFFF;
	EVTCLR2 = 0xFFFFFFFF;
	EVTCLR3 = 0xFFFFFFFF;	

	init_DMA();
	init_McBSP();

	init_LADAR();

	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x22222220);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[5], 0x00FF0FFF);
	SETBIT(SYSCONFIG->PINMUX[5], 0x00110111);  // This is enabling SPI pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13

	init_LCD();
    
	LADARps.x = 3.5/12; // 3.5/12 for front mounting
	LADARps.y = 0;
	LADARps.theta = 1;  // not inverted

	OPTITRACKps.x = 0;
	OPTITRACKps.y = 0;
	OPTITRACKps.theta = 0;

	for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
	{ LADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.

	// ROBOTps will be updated by Optitrack during gyro calibration
	// TODO: specify the starting position of the robot
	ROBOTps.x = 0;			//the estimate in array form (useful for matrix operations)
	ROBOTps.y = 0;
	ROBOTps.theta = 0;  // was -PI: need to flip OT ground plane to fix this
	x_pred[0][0] = ROBOTps.x; //estimate in structure form (useful elsewhere)
	x_pred[1][0] = ROBOTps.y;
	x_pred[2][0] = ROBOTps.theta;

	// TODO: defined destinations that moves the robot around and outside the course
	robotdest[0].x = 0; 	robotdest[0].y = -1;
	//Point 1
	robotdest[1].x = -5;	robotdest[1].y = -3;
	//Point 2
	robotdest[2].x = 3;		robotdest[2].y = 7;
	//Point 3
	robotdest[3].x = -3;	robotdest[3].y = 7;
	//Point 4
	robotdest[4].x = 5;		robotdest[4].y = -3;
	//Point 5
	robotdest[5].x = 0;		robotdest[5].y = 11;
	//Deposit Blue Balls
	robotdest[6].x = -2;	robotdest[6].y = -4;
	//Deposit Orange Balls
	robotdest[7].x = 2;		robotdest[7].y = -4;

	// flag pins
	GPIO_setDir(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(OPTITRACKDATA_FROM_LINUX_BANK, OPTITRACKDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATAFORFILE_TO_LINUX_BANK, DATAFORFILE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(LVDATA_FROM_LINUX_BANK, LVDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(LVDATA_TO_LINUX_BANK, LVDATA_TO_LINUX_FLAG, GPIO_OUTPUT);


	CLR_OPTITRACKDATA_FROM_LINUX;  // Clear = tell linux DSP is ready for new Opitrack data
	CLR_DATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new data
	CLR_DATAFORFILE_TO_LINUX;  // Clear = linux not requesting data
	SET_DATA_TO_LINUX;  // Set = put float array data into shared memory for linux
	SET_IMAGE_TO_LINUX;  // Set = put image into shared memory for linux
	CLR_LVDATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new LV data
	SET_LVDATA_TO_LINUX;  // Set = put LV char data into shared memory for linux

    // clear all possible EDMA 
	EDMA3_0_Regs->SHADOW[1].ICR = 0xFFFFFFFF;
	
    // Add your init code here

}
	

long timecount= 0;
int whichled = 0;
// This SWI is Posted after each set of new data from the F28335
void RobotControl(void) {


	int newOPTITRACKpose = 0;
	int i = 0;

	if (0==(timecount%1000)) {
		switch(whichled) {
		case 0:
			SETREDLED;
			CLRBLUELED;
			CLRGREENLED;
			whichled = 1;
			break;
		case 1:
			CLRREDLED;
			SETBLUELED;
			CLRGREENLED;
			whichled = 2;
			break;
		case 2:
			CLRREDLED;
			CLRBLUELED;
			SETGREENLED;
			whichled = 0;
			break;
		default:
			whichled = 0;
			break;
		}
	}
	
	if (GET_OPTITRACKDATA_FROM_LINUX) {

		if (new_optitrack == 0) {
			for (i=0;i<OPTITRACKDATASIZE;i++) {
				Optitrackdata[i] = ptrshrdmem->Optitrackdata[i];
			}
			new_optitrack = 1;
		}

		CLR_OPTITRACKDATA_FROM_LINUX;

	}

	if (new_optitrack == 1) {
		OPTITRACKps = UpdateOptitrackStates(ROBOTps, &newOPTITRACKpose);
		new_optitrack = 0;
	}

	// using 400deg/s gyro
	gyro = adcA3*3.0/4096.0;
	if (settlegyro < SETTLETIME) {
		settlegyro++;
		if (settlegyro < (SETTLETIME/3)) {
			// do nothing
		} else if (settlegyro < (2*SETTLETIME/3)) {
			gyro_zero = gyro_zero + gyro/(SETTLETIME/3);
		} else {
			gyro_drift += (((gyro-gyro_zero) + old_gyro)*.0005)/(SETTLETIME/3);
			old_gyro = gyro-gyro_zero;
		}
		if(settlegyro%500 == 0) {
			LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
			LCDPrintfLine(2,"");
		}
		enc1old = enc1;
		enc2old = enc2;
		newOPTITRACKpose = 0;

		SetRobotOutputs(0,0,0,0,0,0,0,0,0,0);
	} else {


		gyro_angle = gyro_angle - ((gyro-gyro_zero) + old_gyro)*.0005 + gyro_drift; 
		old_gyro = gyro-gyro_zero;
		gyro_radians = (gyro_angle * (PI/180.0)*400.0*gyro4x_gain);

		// Kalman filtering
		vel1 = (enc1 - enc1old)/(193.0*0.001);	// calculate actual velocities
		vel2 = (enc2 - enc2old)/(193.0*0.001);
		if (fabsf(vel1) > 10.0) vel1 = vel1old;	// check for encoder roll-over should never happen
		if (fabsf(vel2) > 10.0) vel2 = vel2old;
		enc1old = enc1;	// save past values
		enc2old = enc2;
		vel1old = vel1;
		vel2old = vel2;

		// Step 0: update B, u
		B[0][0] = cosf(ROBOTps.theta)*0.001;
		B[1][0] = sinf(ROBOTps.theta)*0.001;
		B[2][1] = 0.001;
		u[0][0] = 0.5*(vel1 + vel2);	// linear velocity of robot
		u[1][0] = (gyro-gyro_zero)*(PI/180.0)*400.0*gyro4x_gain;	// angular velocity in rad/s (negative for right hand angle)

		// Step 1: predict the state and estimate covariance
		Matrix3x2_Mult(B, u, Bu);					// Bu = B*u
		Matrix3x1_Add(x_pred, Bu, x_pred, 1.0, 1.0); // x_pred = x_pred(old) + Bu
		Matrix3x3_Add(P_pred, Q, P_pred, 1.0, 1.0);	// P_pred = P_pred(old) + Q
		// Step 2: if there is a new measurement, then update the state
		if (1 == newOPTITRACKpose) {
			newOPTITRACKpose = 0;
			z[0][0] = OPTITRACKps.x;	// take in the LADAR measurement
			z[1][0] = OPTITRACKps.y;
			// fix for OptiTrack problem at 180 degrees
			if (cosf(ROBOTps.theta) < -0.99) {
				z[2][0] = ROBOTps.theta;
			}
			else {
				z[2][0] = OPTITRACKps.theta;
			}
			// Step 2a: calculate the innovation/measurement residual, ytilde
			Matrix3x1_Add(z, x_pred, ytilde, 1.0, -1.0);	// ytilde = z-x_pred
			// Step 2b: calculate innovation covariance, S
			Matrix3x3_Add(P_pred, R, S, 1.0, 1.0);							// S = P_pred + R
			// Step 2c: calculate the optimal Kalman gain, K
			Matrix3x3_Invert(S, S_inv);
			Matrix3x3_Mult(P_pred,  S_inv, K);								// K = P_pred*(S^-1)
			// Step 2d: update the state estimate x_pred = x_pred(old) + K*ytilde
			Matrix3x1_Mult(K, ytilde, temp_3x1);
			Matrix3x1_Add(x_pred, temp_3x1, x_pred, 1.0, 1.0);
			// Step 2e: update the covariance estimate   P_pred = (I-K)*P_pred(old)
			Matrix3x3_Add(eye3, K, temp_3x3, 1.0, -1.0);
			Matrix3x3_Mult(temp_3x3, P_pred, P_pred);
		}	// end of correction step
	
		// set ROBOTps to the updated and corrected Kalman values.
		ROBOTps.x = x_pred[0][0];
		ROBOTps.y = x_pred[1][0];
		ROBOTps.theta = x_pred[2][0];

		if (startdelay > 0)
			startdelay--;
		else {
			if (firsttime == 0){
				startrow = 11-roundf(ROBOTps.y);
				startcol = roundf(ROBOTps.x)+5;
				endrow = 11-robotdest[statePos].y;
				endcol = robotdest[statePos].x+5;
				astarrunning = 1;
				SEM_post(&SEM_startAstar);
				firsttime = 1;

			}
		}

		if(new_coordata) {
			if(ifblue) {
				cen_row = blue_rcenter;
				cen_col = blue_ccenter;
				numpels = blue_numpixs;
			} else {
				cen_row = orange_rcenter;
				cen_col = orange_ccenter;
				numpels = orange_numpixs;
			}
			new_coordata = 0;
		}

		if(numpels > 10) {
//			distance = 0.00144*cen_row*cen_row + 0.194*cen_row + 8.07;
//			obj_x = ROBOTps.x + distance*cos(ROBOTps.theta);
//			obj_y = ROBOTps.y + distance*sin(ROBOTps.theta);
			if((ROBOTps.y<12.0 && ROBOTps.y>-0.5) && (ROBOTps.x>-6.0 && ROBOTps.x<6.0))
				seeball = 1;
		}
		else {
			seeball = 0;
		}

		switch(seeball) {
		case 0:
			if(astarrunning==0){
				// uses xy code to step through an array of positions
				if( xy_control(&vref, &turn, 1.0, ROBOTps.x, ROBOTps.y, (-5+pathCol[(pathLen - 1) - current_index]),(11 - (pathRow[(pathLen - 1) - current_index])), ROBOTps.theta, 0.25, 0.5))
				{
					//increment current_index
					current_index++;
					if(current_index == pathLen) {
						statePos = (statePos+1)%robotdestSize;
						//					startrow = 11-(int)(ROBOTps.y+0.5);
						//					startcol = (int)(ROBOTps.x+0.5)+5;
						startrow = 11-roundf(ROBOTps.y);
						startcol = roundf(ROBOTps.x)+5;
						endrow = 11-robotdest[statePos].y;
						endcol = robotdest[statePos].x+5;
						astarrunning = 1;
						SEM_post(&SEM_startAstar);
					}
				}
			}
			else{
				vref=0;
				turn = 0;
			}

			if (newLADARdata == 1) {
				newLADARdata = 0;
				for (i=0;i<228;i++) {
					LADARdistance[i] = newLADARdistance[i];
					LADARangle[i] = newLADARangle[i];
					LADARdataX[i] = newLADARdataX[i];
					LADARdataY[i] = newLADARdataY[i];
				}


				for(i=20; i<205; i+=3) {
					if((LADARdataY[i]<12.0 && LADARdataY[i]>-0.5) && (LADARdataX[i]>-6.0 && LADARdataX[i]<6.0) && LADARdistance[i]<500) {
						int rowmap = 11 - roundf(LADARdataY[i]);
						int colmap = 5 + roundf(LADARdataX[i]);

						obstaclemap[11*rowmap + colmap]++;
						if(obstaclemap[11*rowmap + colmap] > 10) {
							if(map[11*rowmap + colmap] == '0') {
								map[11*rowmap + colmap] = 'x';
								counter++;
								obsx = LADARdataX[i];
								obsy = LADARdataY[i];
								remap = 1;
								startrow = 11-roundf(ROBOTps.y);
								startcol = roundf(ROBOTps.x)+5;
								endrow = 11-robotdest[statePos].y;
								endcol = robotdest[statePos].x+5;
								astarrunning = 1;
								SEM_post(&SEM_startAstar);
							}
						}
					}
				}
			}
			break;
		case 1:
			for (i=0;i<228;i++) {
				if(LADARdistance[i]<350)
					seeball = 0;
			}
			if(seeball) {
				vref = 0.80;
				turn = kp*(vref-cen_col);
				Gate = gateOpen;
			}
			else {
				vref = 0;
				turn = 0;
				Gate = gateClose;
			}
			break;
		}


		send_to_labview = 1;


		if ((timecount%200)==0) {
			LCDPrintfLine(1,"x:%.2f,y:%.2f %i",ROBOTps.x,ROBOTps.y, numpels);
			LCDPrintfLine(2,"t:%.1f,f:%.1f",ROBOTps.theta,previous_frame);
		}


		SetRobotOutputs(vref,turn,Gate,Switch,0,0,0,0,0,0);
		//SetRobotOutputs(0,0,enc3/100,0,0,0,0,0,0,0);

		timecount++;
	}
}

pose UpdateOptitrackStates(pose localROBOTps, int * flag) {

	pose localOPTITRACKps;

	// Check for frame errors / packet loss
	if (previous_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
		frame_error++;
	}
	previous_frame = Optitrackdata[OPTITRACKDATASIZE-1];

	// Set local trackableID if first receive data
	if (firstdata){
		//trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
		trackableID = Optitrackdata[OPTITRACKDATASIZE-2];
		firstdata = 0;
	}

	// Check if local trackableID has changed - should never happen
	if (trackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
		trackableIDerror++;
		// do some sort of reset(?)
	}

	// Save position and yaw data
	if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
		// check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
		if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
			// save x,y
			// adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
			// This was the old way for Optitrack coordinates
			//localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
			//localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

			// This is the new coordinates for Motive
			localOPTITRACKps.x = -1.0*Optitrackdata[0]*FEETINONEMETER; 
			localOPTITRACKps.y = Optitrackdata[1]*FEETINONEMETER+4.0;

			// make this a function
			temp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
			tempOPTITRACK_theta = Optitrackdata[2];
			if (temp_theta > 0) {
				if (temp_theta < PI) {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA > 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta > (PI/2)) {
							// THETA > 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA > 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA > 0, kal in QIII, OT in QIII
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta > (3*PI/2)) {
							// THETA > 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA > 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			} else {
				if (temp_theta > -PI) {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA < 0, kal in QIII/IV, OT in QIII/IV
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta < (-PI/2)) {
							// THETA < 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA < 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA < 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta < (-3*PI/2)) {
							// THETA < 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA < 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			}
			*flag = 1;
		}
	}
	return localOPTITRACKps;
}


//preconditions: rowCurr, colCurr, rowGoal, and colGoal are feasible locations in the matrix
//returns the distance between two points as the sum of the row and column differences "Manhattan" distance on a square grid
int heuristic(int rowCurr, int colCurr, int rowGoal, int colGoal)
{
	return abs(rowGoal-rowCurr) + abs(colGoal-colCurr);
}

//assumes that canTravel is called with neighboring points of a valid starting point
//precondition: row, col must be valid indices for a matrix
//returns 0 for false and 1 for true
//(row, col) is considered an unacceptable traveling point if it is represented as an 'x' in map, or if it is out of bounds of the map
int canTravel(int row, int col)
{
	//check for out of bounds
	int mapIdx = row*mapColSize + col;		//map data stored as 1d array
	if(mapIdx >= (mapRowSize*mapColSize) || mapIdx < 0)		//index out of bounds
		return 0;		//0 for cannot travel
	else if(col >= mapColSize || col < 0)			//too many columns, will result in a location in the next row
		return 0;
	else if(map[mapIdx] == 'x')	//wall reached, cannot travel
		return 0;
	else
		return 1;
}

//parameters rowCurr and colCurr must be valid starting points
//returns the number of neighbors which are viable traveling points
//fills the node_t array called neighbors, with neighboring nodes to which the path can travel
//note: not using diagonal neighbors here, so there are only for possibilities for neighboring nodes
int getNeighbors(int rowCurr, int colCurr)
{
	node_t nodeToAdd;
	int numNeighbors = 0;
	if(canTravel(rowCurr-1, colCurr) == 1)	//can travel up
	{
		nodeToAdd.row = rowCurr-1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr, colCurr-1) == 1)	//can travel left
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr-1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr,colCurr+1) == 1)	//can travel right
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr+1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr+1, colCurr) == 1)	//can travel down
	{
		nodeToAdd.row = rowCurr+1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	return numNeighbors;
}

//helper function called inside astar to return the path from the goal point back to the starting point
//the list of points is put into the array pathRow and pathCol in reverse order, pathLen is the number of inserted points
void reconstructPath(int rowEnd, int colEnd, dictElem_t nodeTrack[])
{
	pathLen = 0;		//global variable, reset so start inserting at beginning of path array
	int currRow = rowEnd;
	int currCol = colEnd;
	while(currRow != 400 && currCol != 400)
	{
		//while node is not null "400", put it into path starting at end point
		pathRow[pathLen] = currRow;  //global array for Row
		pathCol[pathLen] = currCol;  //global array for Column
		pathLen++;


	//	printf("currPath: (%d, %d), %d\n", pathRow[pathLen-1], pathCol[pathLen-1], pathLen);
		int nodeTrackIdx = currRow*mapColSize+currCol;
		currRow = nodeTrack[nodeTrackIdx].parentRow;
		currCol = nodeTrack[nodeTrackIdx].parentCol;
	//	printf("next location: (%d, %d), %d\n", currRow, currCol, pathLen);
	}
//	printf("done with reconstruction\n");
}

//path planning algorithm
//parameters rowStart, colStart, rowEnd, and colEnd must be valid locations
//they must be both within the indices of the matrix size and must not be points where barriers ('x') exist
void astar(int rowStart, int colStart, int rowEnd, int colEnd)
{
	//pseudo code instruction: initialize open and closed sets
	//initialize a dictionary to keep track of parents of nodes for easy reconstruction and for keeping track of
	//  heap indexes for easy retrieval from heaps
	//set the values of the dictionary, open and closed sets to be zero
	// so that no old values sitting in memory will produce weird results
	int resetNodeCnt;
	for(resetNodeCnt=0; resetNodeCnt<176; resetNodeCnt++)
	{
		nodeTrack[resetNodeCnt].heapId = ' ';
		nodeTrack[resetNodeCnt].heapIdx = 0;
		nodeTrack[resetNodeCnt].parentRow = 0;
		nodeTrack[resetNodeCnt].parentCol = 0;
	}

	startHeap(&openSet);
	startHeap(&closedSet);
	currDist = 0;
	node_t start;

	/* initialize a start node to be the starting position
		since this is the start node, it should have zero distance traveled from the start, the normal predicted distance to the goal,
		and the total distance is the sum of the distance traveled from the start and the distance to the goal.*/
	/* update the dictionary, use row and column location as a key in the dictionary,
		use 400 for null parent values and don't forget to indicate which heap, open or closed set, the node is being placed*/
	/* put starting node on open set*/

	start.row = rowStart;
	start.col = colStart;
	start.distTravelFromStart = 0;
	start.distToGoal = heuristic(rowStart,colStart,rowEnd,colEnd);
	start.totalDist = start.distTravelFromStart+start.distToGoal;
	int startIdx = (start.row*mapColSize)+start.col;
	(nodeTrack[startIdx]).heapId = 'o';		//o for open set
	nodeTrack[startIdx].parentRow = 400;	//use 400 as NULL, if 400, know reached beginning in reconstruction
	nodeTrack[startIdx].parentCol = 400;	//no parent value = 400 since out of bounds
	if(rowStart == rowEnd && colStart == colEnd)		//if start point is the end point, don't do anything more!!!
		return;
	push(start, &openSet, nodeTrack, mapColSize); // put start node on the openSet

	char goalFound = 'f'; //false

	/*while open set not empty and goal not yet found*/
	while(openSet.numElems > 0 && goalFound != 't')
	{

		/*find the node with the least total distance (f_score) on the open list, call it q (called minDistNode in code)*/
		node_t minDistNode = peek(openSet);	//find node with least total cost, make that the current node

		/*set the current distance to the current node's distance traveled from the start.*/
		//1.  currDist is set to q's distance traveled from the Start.  Explain why this could be different from the Manhattan distance to the Start position
		//    This question is just asking for comments.
		currDist = minDistNode.distTravelFromStart;

		(nodeTrack[(minDistNode.row*mapColSize)+minDistNode.col]).heapId = 'r';		//r for removed from any set
		// Manhattan Distance - piecewise addition of vertical and horizontal distances (from q to destination)
		// Regular Distance - Pythagorean distance from q to destination

		//2.  pop q (which is currently the minimum) off which queue?
		// Choose one of these two lines of code
		// IF the Openset
		pop(&openSet, nodeTrack, mapColSize);
		// IF closedSet
		//pop(&closedSet, nodeTrack, mapColSize);

		/*generate q's 4 neighbors*/
		// 3.  Pass q's row and col to getNeighbors
		int numNeighbors = getNeighbors(minDistNode.row, minDistNode.col);	//get list of neighbors

		/*for each neighbor*/
		int cnt = 0;
		for(cnt = 0; cnt<numNeighbors; cnt++)	//for each found neighbor
		{
			// 4. Just add comments here.  Find where the structure node_t is defined and inside commments here copy its definition for
			// viewing reference.
			/*
			typedef struct node
{
	int row;
	int col;
	int distTravelFromStart;
	int distToGoal;
	int totalDist;
	int pushOrder;		//order that element was pushed onto the heap
} node_t;
			 */
			// All the answer for 4. will be commented.
			node_t next = neighbors[cnt];

			/*if neighbor is the goal, stop the search*/
				/*update the dictionary so this last node's parents are set to the current node*/
			if((next.row == rowEnd) && (next.col == colEnd))		//if neighbor is the goal, found the end, so stop the search
			{
				// 5.  set current neighbor's parents.  Set parentRow to q's row.  Set parentCol to q's col since q is the parent of this neighbor
				(nodeTrack[next.row*mapColSize+next.col]).parentRow = minDistNode.row;	 //set goal node's parent position to current position
				(nodeTrack[next.row*mapColSize+next.col]).parentCol = minDistNode.col;
				goalFound = 't';
				break;
			}

			/*neighbor.distTravelFromStart (g) = q.distTravelFromStart + distance between neighbor and q which is always 1 when search just top left bottom right*/
			// 6.  Set this neighbor's distance traveled from the start.  Remember you have the variable "currDist" that is the distance of q to Start
			next.distTravelFromStart = currDist + 1;

			/*neighbor.distToGoal (h) = distance from goal to neighbor, heuristic function	(estimated distance to goal)*/
			// 7.  Pass the correct parameters to "heuristic" to calculate the distance this neighbor is from the goal.
			//  Remember that we have the variables rowEnd and colEnd which are the grid coordinates of the goal
			next.distToGoal = heuristic(next.row, next.col, rowEnd, colEnd);

			/*neighbor.totalDist (f) = neighbor.distTravelFromStart + neighbor.distToGoal
				(total estimated distance as sum of distance traveled from start and distance to goal)*/
			// 8.  Find f, (totalDist) for this neighbor
			next.totalDist = next.distTravelFromStart + next.distToGoal;


			// 9.  Just comments for this question.
			// Explain in your own words (not copying the comments below) what the next 19 lines of C code are doing

			/*if a node with the same position as neighbor is in the OPEN list
				which has a lower total distance than neighbor, skip putting this neighbor onto the open set*/
			//check if node is on the open set already
			int nodeTrackIdx = (next.row*mapColSize)+next.col;
			char skipPush = 'f';
			if(nodeTrack[nodeTrackIdx].heapId == 'o')	//on the open set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromOpen = openSet.elems[heapIndex];
				if(fromOpen.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, a better one already on there
			}

			/*if a node with the same position as neighbor is in the CLOSED list
				which has a lower f than neighbor, skip putting this neighbor onto the open set*/
			else if(nodeTrack[nodeTrackIdx].heapId == 'c')		//on closed set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromClosed = closedSet.elems[heapIndex];
				if(fromClosed.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, already part of possible solution
			}

			/*if not skipping putting this node on the set, then push node onto the open set
				and update the dictionary to indicate the node is on the open set and set the parents of this node to the current node*/
				//(can't be an ordinary else to the things above because then it won't push onto the open set if already on open or closed set)
			if(skipPush != 't')
			{
				nodeTrack[nodeTrackIdx].heapId = 'o';		//mark in nodetrack that this is going onto the open set
				(nodeTrack[nodeTrackIdx]).parentRow = minDistNode.row;	 //set neighbor's parent position to current position
				(nodeTrack[nodeTrackIdx]).parentCol = minDistNode.col;
				//10.  push this neighbor on which queue?
				// Choose one of these two lines of code
				// IF openSet
				push(next, &openSet, nodeTrack, mapColSize);
				// IF closedSet
				//push(next, &closedSet, nodeTrack, mapColSize);

			}
		}/* end for loop*/

		/* These lines of code look through the open set and verify that the neighboring node is either
		 * lower or higher than the minimum f. If the total distance is higher, do not add this node as
		 * there is a better path. Compute the same analysis for the closed list. If it is minimum, add
		 * it to the open list.
		 */

		int nodeTrackIdx = minDistNode.row*mapColSize+minDistNode.col;
		nodeTrack[nodeTrackIdx].heapId = 'c';
		//11.  push q (current node) on which queue?
		// Choose one of these two lines of code
		// IF openSet
		//push(minDistNode, &openSet, nodeTrack, mapColSize);
		// IF closedSet
		push(minDistNode, &closedSet, nodeTrack, mapColSize);

	}  /*end while loop*/

	/*if a path was found from the start to the goal, then reconstruct the path*/
	if(goalFound == 't')
		// 12.  Pass the correct varaibles to "reconstructPath" in order for it to fill in the global arrays pathRow, pathCol
		//     and integer pathLen.  Note that the path is in reverse order in pathRow and pathCol.
		reconstructPath(rowEnd, colEnd, nodeTrack);
}

//SEM_pend(&SEM_startAstar);
void runAstar(void)  {

	while(1) {

		SEM_pend(&SEM_startAstar, SYS_FOREVER);
		//call astar
		astar(startrow, startcol, endrow, endcol);
		astarrunning = 0;
		current_index = 0;

	}
}

