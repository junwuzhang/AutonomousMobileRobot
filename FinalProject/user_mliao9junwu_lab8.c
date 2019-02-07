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

// Added to implement wall following
float convADCA3 = 0;
float convADCA2 = 0;
float minimum = 450;
float minimum2 = 400;
float ref_right_wall = 400.0;
float front_error_threshold = 2550.0;
float Kp_right_wall = 0.014;
float Kp_front_wall = 0.004;
float front_turn_velocity = 0.4;
float turn_command_saturation = 4.5;
float forward_velocity = 1.0;
float front_wall_error = 0.0;
float right_wall_error = 0.0;
float gyro_zero = 0.0;
float sumconvADCA2 = 0.0;
float sumconvADCA3 = 0.0;
float gyrogain = 1.0;
float enc1_old = 0.0;
float enc2_old = 0.0;
float gyrozero100 = 0.0;
float gyrozero400 = 0.0;
float gyrorate100 = 0.0;
float gyrorate400 = 0.0;
float integralGyro100 = 0.0;
float integralGyro400 = 0.0;
float oldGyrorate100 = 0.0;
float oldGyrorate400 = 0.0;
float oldIntegralGyro100 = 0.0;
float oldIntegralGyro400 = 0.0;
float x = 0.0;
float y = 0.0;
float Vl = 0.0;
float Vr = 0.0;
int count = 0;
float p1 = 0;
float p1_old = 0;
float p2 = 0;
float p2_old = 0;
float v1 = 0;
float v2 = 0;
int send_to_labview = 0;
float var1 = 0;
float var2 = 0;
float var3 = 0;
float var4 = 0;
// Finish adding variables for wall following

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

extern pose ROBOTps;
extern pose LADARps;

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
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

//extern int new_r_centroid;
//extern int new_c_centroid;
//extern int new_num_pixels;
//extern int new_vision_data;
extern volatile float object_x;
extern volatile float object_y;
extern volatile int new_coordata;
extern volatile int numpels;


// make local copies here
float cen_row = 0;
float cen_col = 0;
int num_pix = 0;
float kp = 0.05;

extern float blue_rcenter;
extern float blue_ccenter;
extern int blue_numpixs;
extern float green_rcenter;
extern float green_ccenter;
extern int green_numpixs;

float brc = 0;
float bcc = 0;
int bpx = 0;
float grc = 0;
float gcc = 0;
int gpx = 0;

int firstgreen = 0;
int secondgreen = 0;
int counttime = 0;
int counter = 0;

extern int ifblue;

float distance_y = 0.0;

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

				// Default
				ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"1.0 1.0 1.0 1.0");
				// you would do something like this
				//ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",var1,var2,var3,var4);

				if(send_to_labview==1) {
					var1 = x;
					var2 = y;
					var3 = (Vl+Vr)/2;
					var4 = integralGyro100;
					send_to_labview = 0;
				}
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

			// This is an example write to scratch
			// The Linux program SaveScratchToFile can be used to write the
			// ptrshrdmem->scratch[0-499] memory to a .txt file.
//			for (i=100;i<300;i++) {
//				ptrshrdmem->scratch[i] = (float)i;
//			}

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


//	if(new_vision_data) {
//		cen_row = new_r_centroid - (144/4);
//		cen_col = new_c_centroid - (176/2);
//		num_pix = new_num_pixels;
//		LCDPrintfLine(1, "%i %i", cen_row, cen_col);
//		LCDPrintfLine(2, "%i", num_pix);
//		new_vision_data = 0;
//	}

//	if(new_coordata) {
//		if(ifblue) {
//			cen_row = object_y;
//			cen_col = object_x;
//			blue_rcenter = cen_row;
//			blue_ccenter = cen_col;
//			blue_numpixs = numpels;
//		} else {
//			cen_row = object_y;
//			cen_col = object_x;
//			green_rcenter = cen_row;
//			green_ccenter = cen_col;
//			green_numpixs = numpels;
//		}

//		if(new_coordata) {
//			if(ifblue) {
//				cen_row = blue_rcenter;
//				cen_col = blue_ccenter;
//				numpels = blue_numpixs;
//			} else {
//				cen_row = green_rcenter;
//				cen_col = green_ccenter;
//				numpels = green_numpixs;
//			}

	send_to_labview = 1;


	if(new_coordata) {
//		LCDPrintfLine(1, "%.1f %.1f", cen_col, cen_row);
//		LCDPrintfLine(2, "%i", num_pix);
		bcc = blue_ccenter;
		brc = blue_rcenter;
		bpx = blue_numpixs;
		gcc = green_ccenter;
		grc = green_rcenter;
		gpx = green_numpixs;
		distance_y = 0.00144*cen_row*cen_row + 0.194*cen_row + 8.07;
//		LCDPrintfLine(1, "%.1f %.1f", blue_rcenter, blue_ccenter);
//		LCDPrintfLine(1, "%.1f %.1f", green_rcenter, green_ccenter);
		LCDPrintfLine(1, "%.1f %.1f %.1f", x, y, integralGyro100);
//		LCDPrintfLine(2, "%.1f %.1f Com: %.1f", gcc, grc, compass);
		LCDPrintfLine(2, "%i %i %i %i", firstgreen, secondgreen, gpx, counttime);
		new_coordata = 0;
	}

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

	if (newLADARdata == 1) {
		newLADARdata = 0;
		for (i=0;i<228;i++) {
			LADARdistance[i] = newLADARdistance[i];
			LADARangle[i] = newLADARangle[i];
			LADARdataX[i] = newLADARdataX[i];
			LADARdataY[i] = newLADARdataY[i];
		}
	}

	// Get rid of these two lines when implementing wall following
//	vref = 0.80;
//	turn = kp*(vref-cen_col);
	
	// Get rid of these two lines when implementing wall following
	int j = 0;

//	minimum = 5500.0;
//	for(j=111; j<116; j++) {
//		if(LADARdistance[j] < minimum)
//			minimum = LADARdistance[j];
//	}
//
//	minimum2 = 5500.0;
//	for(j=51; j<57; j++) {
//		if(LADARdistance[j] < minimum2)
//			minimum2 = LADARdistance[j];
//	}
//
//	for(j=105; j<122; j++) {
//		if(LADARdistance[j] < 400)
//			vref = 0.0;
//	}
//
//	front_wall_error = 3000 - minimum;
//	right_wall_error = ref_right_wall - minimum2;
//	if (fabsf(front_wall_error) > front_error_threshold){
//		// Change turn command according to proportional feedback control on front error
//		// use Kp_front_wall here…
//		turn = -Kp_front_wall * front_wall_error;
//		vref = front_turn_velocity;
//	} else {
//		// Change turn command according to proportional feedback control on right error
//		// use Kp_right_wall here
//		// vref = forward_velocity
//		turn = -Kp_right_wall * right_wall_error;
//		vref = forward_velocity;
//	}
//
//	// Add code here to saturate the turn command so that it is not larger
//	// than turn_command_saturation or less than –turn_command_saturation
//	if (turn > turn_command_saturation) {
//		turn = turn_command_saturation;
//	} else if (turn < (-1.0*turn_command_saturation)) {
//		turn = (-1.0*turn_command_saturation);
//	}

	minimum = 5500.0;
	for(j=111; j<116; j++) {
		if(LADARdistance[j] < minimum)
			minimum = LADARdistance[j];
	}

	minimum2 = 5500.0;
	for(j=51; j<57; j++) {
		if(LADARdistance[j] < minimum2)
			minimum2 = LADARdistance[j];
	}

	//	if(minimum < 400)
	//		vref = 0.0;

	count++;
	if(count < 3000) {
		vref = 0;
		turn = 0;
		enc1_old = enc1;
		enc2_old = enc2;
		sumconvADCA3 += adcA3*3.0/4095.0;
		sumconvADCA2 += adcA2*3.0/4095.0;
	} else if (count == 3000){
		gyrozero100 = (sumconvADCA2/3000.0);
		gyrozero400 = (sumconvADCA3/3000.0);
	} else {
		if (new_LV_data == 1) {
			gyrogain = LVvalue1;
			new_LV_data = 0;
		}

		gyrorate100 = gyrogain*100*(PI/180.0)*(adcA2*(3.0/4095.0) - gyrozero100);
		gyrorate400 = gyrogain*400*(PI/180.0)*(adcA3*(3.0/4095.0) - gyrozero400);

		gyrorate100 *= gyrogain;

		integralGyro100 = oldIntegralGyro100 + (gyrorate100 + oldGyrorate100)*0.5*0.001;
		integralGyro400 = oldIntegralGyro400 + (gyrorate400 + oldGyrorate400)*0.5*0.001;
		oldGyrorate100 = gyrorate100;
		oldGyrorate400 = gyrorate400;
		oldIntegralGyro100 = integralGyro100;
		oldIntegralGyro400 = integralGyro400;


		//convADCA3 = adcA3*3.0/4095.0;
		//convADCA3 = (convADCA3 - gyrozero400)*400;
		//convADCA2 = adcA2*3.0/4095.0;
		//convADCA2 = (convADCA2 - gyrozero100)*100;

		Vl = ((enc1-enc1_old)/193)/0.001;
		Vr = ((enc2-enc2_old)/193)/0.001;

		enc1_old = enc1;
		enc2_old = enc2;
		x = x + ((Vl + Vr)/2)*0.001*cosf(integralGyro100);
		y = y + ((Vl + Vr)/2)*0.001*sinf(integralGyro100);


		front_wall_error = 3000 - minimum;
		right_wall_error = ref_right_wall - minimum2;
		if (fabsf(front_wall_error) > front_error_threshold){
			// Change turn command according to proportional feedback control on front error
			// use Kp_front_wall here…
			turn = -Kp_front_wall * front_wall_error;
			vref = front_turn_velocity;

			if ((green_numpixs > 180) && ((compass > 3300 && compass < 3600) || (compass > 0 && compass < 360)) && counter) {
				x = 5.2;
				y = 9.5;
				integralGyro100 = 1.5;
				firstgreen++;
//				send_to_labview = 1;
				counter = 0;
			}

			if ((green_numpixs > 180) && (compass > 2200 && compass < 3300) && counter) {
				x = -3.7;
				y = 9.7;
				integralGyro100 = 3.0;
				secondgreen++;
//				send_to_labview = 1;
				counter = 0;
			}

			if(counter==0) {
				counttime++;
				if((counttime%100)==0)
					counter = 1;
			}


		} else {
			// Change turn command according to proportional feedback control on right error
			// use Kp_right_wall here
			// vref = forward_velocity
			turn = -Kp_right_wall * right_wall_error;
			vref = forward_velocity;
		}


		// Add code here to saturate the turn command so that it is not larger
		// than turn_command_saturation or less than –turn_command_saturation
		if (turn > turn_command_saturation) {
			turn = turn_command_saturation;
		} else if (turn < (-1.0*turn_command_saturation)) {
			turn = (-1.0*turn_command_saturation);
		}

	}


	SetRobotOutputs(vref,turn,0,0,0,0,0,0,0,0);

	timecount++;

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
			localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
			localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

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

