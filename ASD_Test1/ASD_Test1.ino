/*
 *       Proof of concept created by Arthur Lee
 *       2/1/2015
 */

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <UARTDriver.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Input channels
#define MAX_CHANNELS 4

static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc = &rc_1;
static uint16_t v[MAX_CHANNELS];

void setup()
{
  hal.console->printf("ASD T1 RC Channel test\n");
  // Enable channels outputs: 1 to 8
  // set type of output, symmetrical angles or a number range;
  rc_1.set_angle(4500);
  rc_1.set_default_dead_zone(80);
  rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

  rc_2.set_angle(4500);
  rc_2.set_default_dead_zone(80);
  rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

  rc_3.set_range(0,1000);
  rc_3.set_default_dead_zone(20);

  rc_4.set_angle(6000);
  rc_4.set_default_dead_zone(500);
  rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

  rc_5.set_range(0,1000);
  rc_6.set_range(200,800);

  rc_7.set_range(0,1000);

  rc_8.set_range(0,1000);
  for (int i=0; i<MAX_CHANNELS; i++) {
    rc[i].enable_out();
  }
}

void loop()
{
  // Read PWM
  RC_Channel::set_pwm_all();
  // Pass through
  for (int i=0; i<MAX_CHANNELS; i++) {
    v[i]=rc[i].control_in;
  }
  output();
  hal.scheduler->delay(20);
}

static void output(void){
  for (int i=0; i<MAX_CHANNELS; i++) {
//    hal.console->printf("CH%u: %4d ",
//     (unsigned)i+1,(unsigned)v[i]);
//     hal.console->println();
    rc[i].servo_out = v[i];
    rc[i].calc_pwm();
    rc[i].output();
  }
}

AP_HAL_MAIN();









