
/* ****************************************************** */
/* Based on an first implementation by Alex Mos           */
/* http://www.multiwii.com/forum/viewtopic.php?f=7&t=1413 */
/* In difference, to the original code this implementation*/
/* Uses standard SPi communication via the ISP port and   */
/* not digitalWriteFast.                                  */
/* ****************************************************** */

#include "Arduino.h"
#include "SPI.h"

#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "OptFlow.h"

#define	PRODUCT_ID					0x00 //   should return 0x17 if a working adns3080 is connected
#define	MOTION_REG					0x02
#define	DELTA_X_REG					0x03 
#define	DELTA_Y_REG					0x04 
#define	SQUAL_REG					0x05
#define EXTENDED_CONFIG                                 0x0b
#define FRAME_PERIOD_MAX_BOUND_LOWER                    0x19
#define FRAME_PERIOD_MAX_BOUND_UPPER                    0x1a
#define CONFIGURATION_BITS                              0x0a
#define MOTION_CLEAR_REG                                0x12
#define TSRAD_TIME 50 // try to set 75 if not working
#define OPTFLOW_PVAL 32
#define OPTFLOW_IVAL 20

//#if	!defined(digitalWriteFast)
//	#include "digitalWriteFast.h"
//#endif

/* Exponential moving average filter (optimized for integers) with factor = 2^n */
typedef struct avg_var16
{
int32_t buf; // internal bufer to store non-rounded average value
int16_t res; // result (rounded to int)
} t_avg_var16;
typedef struct avg_var8
{
int16_t buf; // internal bufer to store non-rounded average value
int8_t res; // result (rounded to int)
} t_avg_var8;
boolean _overflow=false;


/* n=(1..16) */
void average16(struct avg_var16 *avg, int16_t cur, int8_t n) {
        avg->buf+= cur - avg->res;
        avg->res = avg->buf >> n;
}
/* n=(1..8) */
void average8(struct avg_var8 *avg, int8_t cur, int8_t n) {
        avg->buf+= cur - avg->res;
        avg->res = avg->buf >> n;
}


static uint16_t	scale; // scale factor for raw sensor data
static int16_t sum_dx = 0, sum_dy = 0; // sensor's row data accumulators
static int16_t EstHVel[2] = {0,0}; // horizontal velocity, cm/sec (constrained -100, 100)
static int16_t optflow_pos[2] = {0,0};	//displacement (in mm*10 on height 1m)

int16_t optflow_angle[2] = { 0, 0 }; 

byte orig_spi_settings_spcr;
byte orig_spi_settings_spsr;


/* PID calculations. Outputs optflow_angle[ROLL], optflow_angle[PITCH] */
void	Optflow_update() {
	static int16_t optflowErrorI[2]	= {0,0};
	static int16_t prevHeading = 0;
	static int8_t	optflowUse = 0;
	int8_t axis;
	// enable OPTFLOW only in ANGLE	mode and if GPS	is not used
	if(f.ANGLE_MODE == 1 && f.OPTFLOW_MODE == 1 && f.GPS_mode == 0) {
		// init	first	time mode	enabled
		if(!optflowUse)	{
			optflowErrorI[0] = 0;	optflowErrorI[1] = 0;
			prevHeading = att.heading;
			optflow_start();
			optflowUse = 1;
                        sum_dx = sum_dy = 0;
			return;
		}
		
		// Read	sensors
		optflow_read();

		// Rotate I to follow global axis
		#ifdef OF_ROTATE_I
			int16_t	dif = att.heading - prevHeading;
			if (dif	<= - 180) dif += 360;
			else if	(dif >=	+ 180) dif -= 360;

			if(abs(dif) > 5) { //rotate by	5-degree steps
				rotate16(optflowErrorI,	dif*10);
				prevHeading = att.heading;
			}
		#endif
                
		// Use sensor	only inside	DEADBAND
		if(abs(rcCommand[ROLL])	< OF_DEADBAND && abs(rcCommand[PITCH]) < OF_DEADBAND)	{
			// calculate velocity

			optflow_get_vel();

			for(axis=0; axis<2; axis++) {
				// correction should be	less near deadband limits
				EstHVel[axis] = EstHVel[axis] *	(OF_DEADBAND - abs(rcCommand[axis])) / OF_DEADBAND;	// 16 bit ok: 100*100 =	10000
				optflowErrorI[axis]+= EstHVel[axis]; 
				optflowErrorI[axis] = constrain(optflowErrorI[axis], -20000, 20000);
				optflow_angle[axis] = EstHVel[axis] * conf.pid[PIDVEL].P8 / 50;	// 16 bit ok: 100 * 200	= 20000
			}					
		}	else {
			  optflow_angle[ROLL] = 0;	
                          optflow_angle[PITCH] = 0;
		}

		// Apply I-term	unconditionally
		for(axis=0; axis<2; axis++)	{
			int32_t tmp = optflow_angle[axis] + (int16_t)((int32_t)optflowErrorI[axis] * conf.pid[PIDVEL].I8 / 5000);
			optflow_angle[axis] = constrain(tmp, -300, 300);
		}

		#ifdef OF_DEBUG
			debug[2] = optflow_angle[ROLL]*10;
			debug[3] = optflow_angle[PITCH]*10;
		#endif
	}	else if(optflowUse)	{	// switch mode off
		  optflow_angle[ROLL] = 0;
        	  optflow_angle[PITCH] = 0;
		  optflowUse = 0;
	}
}


/* Calculate estimated velocity	from OF-sensor */
void	optflow_get_vel()	{
	int16_t	vel_of[2]; //	velocity from	OF-sensor, cm/sec
	static int16_t prevAngle[2]	=	{	0, 0 };
	static t_avg_var16 avgVel[2] = { {0,0},	{0,0}	}; 
	static t_avg_var8	avgSqual = {0,0};
	uint16_t my_alt;	// alt in	mm*10
	int8_t axis;

	static uint16_t	prevTime = 0;
	uint16_t tmpTime = micros();
	uint16_t dTime = tmpTime - prevTime;
	prevTime = tmpTime;
	
	// get normalized sensor values
	optflow_get();
	
	// read	and	average	surface	quality
	average8(&avgSqual, (int8_t)optflow_squal(), 5);

	if(cosZ	> 70 &&	avgSqual.res > 10) {
		// above 3m, freeze	altitude (it means less	stabilization	on high	altitude)
		// ..	and	reduce signal	if surface quality <50
                #ifdef SONAR || LIDAR_LITE 
                  if (sonarAlt <= 400) {
  		      my_alt = constrain((int16_t)sonarAlt, 30, 300) * min(avgSqual.res,50) * 2; // 16bit ok: 300 * 50 * 2 = 30000;
                  }
                  else {
                      my_alt = constrain((int16_t)alt.EstAlt, 30, 300) * min(avgSqual.res,50) * 2; // 16bit ok:	300 * 50 * 2 = 30000;
                  }
                #else 
		  my_alt = constrain((int16_t)alt.EstAlt, 30, 300) * min(avgSqual.res,50) * 2;	// 16 bit ok: 300 * 50 * 2 = 30000;
                #endif
	}	else {
  		  my_alt = 0;
	}
	
	for(axis=0;axis<2;axis++)	{
		// Get velocity	from OF-sensor only	in good	conditions
		if(my_alt !=	0) { 
			// remove	shift	in position	due	to inclination:	delta_angle	*	PI / 180 * 100
			// mm/sec(10m) * cm	/	us	 ->		 cm	/	sec
			vel_of[axis] = ((int32_t)optflow_pos[axis] + (att.angle[axis] - prevAngle[axis]) * 17) * my_alt	/ dTime;	
		}	else {
			vel_of[axis] = 0;
		}
		
		average16(&avgVel[axis], vel_of[axis], OF_LPF_FACTOR);
		EstHVel[axis]	= constrain(avgVel[axis].res,	-100,	100);
		prevAngle[axis]	= att.angle[axis];
	}
	
	#ifdef OF_DEBUG
                debug[0] = EstHVel[axis];
		debug[1] = optflow_pos[0];
		debug[2] = avgSqual.res;
		debug[3] = vel_of[0]*10;
	#endif
}



/* Convert row data to displacment (in mm*10 on	height 1m) since last call */
/* negated values (x,y) since sensor is mounted facing back */
void	optflow_get()	{

  	int32_t x = (int32_t)sum_dx * scale;
        x = -x;

	int32_t y = (int32_t)sum_dy * scale;
        y = -y;
        
	optflow_pos[ROLL] = constrain(x, -0x7FFF, 0x7FFF);
	optflow_pos[PITCH] = constrain(y, -0x7FFF, 0x7FFF);
	
        #ifdef OF_DEBUG
              debug[0] = x;
              debug[1] = y;
              debug[2] = optflow_pos[ROLL]; //sum_dx;
              debug[3] = optflow_pos[PITCH]; //sum_dy;
        #endif
        
	// clear accumulated displacement
	sum_dx = 0;	sum_dy = 0;	
}	


/* *************************************************** */
/* ADNS-3080																					 */
/* *************************************************** */

 boolean initOptflow()	{
   
        boolean health = false;
   
	pinMode(OF_MOSI, OUTPUT);
	pinMode(OF_MISO, INPUT);
	pinMode(OF_SCLK, OUTPUT);
	pinMode(OF_NCS, OUTPUT);

        //Since EZ-GUI does not really support to set VEL PID's here is a fallback
        //This is not the preferred solution but a necessary workaround 
        if (conf.pid[PIDVEL].P8 == 0)
          conf.pid[PIDVEL].P8 = 22;
        if (conf.pid[PIDVEL].D8 == 0)
          conf.pid[PIDVEL].D8 = 20;
    
        
	// reset device
	#if(OF_RESET>0)
		pinModeFast(OF_RESET,	OUTPUT);
		digitalWrite(OF_RESET, HIGH);
		delayMicroseconds(10);
		digitalWrite(OF_RESET, LOW);
		delayMicroseconds(500);
	#endif

		
	scale = (uint32_t)500000 / OF_FOCAL_DIST / 1600;
        SPI.begin();
        delay(100);
        for (int i = 0; i <3; i++){
           if( read_register(PRODUCT_ID) == 0x17 ){
             health = true;  // the sensor is of good health
             debug[3] = 777;
             break;
           }
           delay(5);
        }

        // set 1600 resolution bit
        byte regVal = read_register(CONFIGURATION_BITS);
        regVal |= 0x10; //80;//regVal | (1 << 4);
        delay(50);
        write_register(CONFIGURATION_BITS, regVal);
        delay(50);
         

      // set frame rate to 2000 FPS
        regVal = read_register(EXTENDED_CONFIG);
        delay(50);
        regVal = (regVal & ~0x01) | 0x01;
        write_register(EXTENDED_CONFIG, regVal);
        delay(50);
         // set frame period to 12000 (0x2EE0)
        write_register(FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
        delay(50);
        write_register(FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);
        delay(50);
     // set framerate to fixed value   
        regVal = 1;
        write_register(EXTENDED_CONFIG, regVal);
        
        return health;

}


/* Start motion	capture	*/
void	optflow_start()	{
	// reset motion	buffers
	write_register(MOTION_CLEAR_REG, 1);
}


/* Read	sensor values.	*/
void optflow_read() {
	byte motion;
	//do {
		motion = read_register(MOTION_REG);
		if((motion & 0x80) != 0)	{	// motion	detected
			if(motion & 0x01)	{	// 1600	cpi
				sum_dx+= (int16_t)((int8_t)read_register(DELTA_X_REG)) * 4;
				sum_dy+= (int16_t)((int8_t)read_register(DELTA_Y_REG)) * 4;
			}	else { //	400	cpi
				sum_dx+= (int8_t)read_register(DELTA_X_REG);
				sum_dy+= (int8_t)read_register(DELTA_Y_REG);
			}
		}
	//}	while(motion & 0x10);	 //	was 0x20 /; read internal buffers until overflow flag cleared

        #ifdef OF_DEBUG
           debug[0] = sum_dx;
           debug[1] = sum_dy;
           debug[2]=motion;
        #endif
}


/* get surface quality,	(0..169) */
uint8_t optflow_squal() {
	return read_register(SQUAL_REG);
}

/******************** SPI read/write routines ****************************/


/* Rotate vector V(x,y) to angle delta (in 0.1 degree) using small angle approximation and integers. */
/* (Not precise but fast) */
void rotate16(int16_t *V, int16_t delta) {
  int16_t tmp = V[0];
  V[0]-= (int16_t)( ((int32_t)delta) * V[1] / 573);
  V[1]+= (int16_t)( ((int32_t)delta) * tmp / 573);
}


// Read a register from the sensor
byte read_register(byte address)
{
  byte result = 0, junk = 0;

  backup_spi_settings();

  // take the chip select low to select the device
  digitalWrite(OF_NCS, LOW);

  // send the device the register you want to read:
  junk = SPI.transfer(address);

  // small delay
  delayMicroseconds(50); //was 50

  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);

  // take the chip select high to de-select:
  digitalWrite(OF_NCS, HIGH);

  restore_spi_settings();

  return result;
}



// write a value to one of the sensor's registers
void write_register(byte address, byte value)
{
  byte junk = 0;

  backup_spi_settings();

  // take the chip select low to select the device
  digitalWrite(OF_NCS, LOW);

  // send register address
  junk = SPI.transfer(address | 0x80 );

  // small delay
  delayMicroseconds(50);

  // send data
  junk = SPI.transfer(value);

  // take the chip select high to de-select:
  digitalWrite(OF_NCS, HIGH);

  restore_spi_settings();
}

//
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
//
byte backup_spi_settings()
{
  // store current spi values
  orig_spi_settings_spcr = SPCR & (DORD | CPOL | CPHA);
  orig_spi_settings_spsr = SPSR & SPI2X;
  
  // set the values that we need
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // sensor running at 2Mhz.  this is it's maximum speed
  
  return orig_spi_settings_spcr;
}


// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
byte restore_spi_settings()
{
  byte temp;

  // restore SPSR
  temp = SPSR;
  temp &= ~SPI2X;
  temp |= orig_spi_settings_spsr;
  SPSR = temp;

  // restore SPCR
  temp = SPCR;
  temp &= ~(DORD | CPOL | CPHA);   // zero out the important bits
  temp |= orig_spi_settings_spcr;  // restore important bits
  SPCR = temp;

  return temp;
}


