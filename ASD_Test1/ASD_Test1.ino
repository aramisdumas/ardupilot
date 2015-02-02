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
#define NUM_CHANNELS 8

static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc[NUM_CHANNELS];
static uint16_t val[NUM_CHANNELS];
// Mode flag true for vertical, false for horizontal
static bool mode;
void setup()
{
  hal.console->printf("ASD T1 RC Channel test\n");
  rc[CH_1]=&rc_1;
  rc[CH_2]=&rc_2;
  rc[CH_3]=&rc_3;
  rc[CH_4]=&rc_4;
  rc[CH_5]=&rc_5;
  rc[CH_6]=&rc_6;
  rc[CH_7]=&rc_7;
  rc[CH_8]=&rc_8;


  // Enable channels outputs: 1 to 8
  // Setup channel type for output
  rc[CH_1]->set_range(0,1000);
  rc[CH_1]->set_default_dead_zone(20);

  rc[CH_2]->set_range(0,1000);
  rc[CH_2]->set_default_dead_zone(20);

  rc[CH_3]->set_range(0,1000);
  rc[CH_3]->set_default_dead_zone(20);

  rc[CH_4]->set_range(0,1000);
  rc[CH_4]->set_default_dead_zone(20);
  //  // set type of output, symmetrical angles or a number range;
  //  rc[CH_1]->set_angle(4500);
  //  rc[CH_1]->set_default_dead_zone(80);
  //  rc[CH_1]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  //
  //  rc[CH_2]->set_angle(4500);
  //  rc[CH_2]->set_default_dead_zone(80);
  //  rc[CH_2]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  //
  //  rc[CH_3]->set_range(0,1000);
  //  rc[CH_3]->set_default_dead_zone(20);
  //
  //  rc[CH_4]->set_angle(6000);
  //  rc[CH_4]->set_default_dead_zone(80);
  //  rc[CH_4]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  rc[CH_5]->set_angle(4500);
  rc[CH_5]->set_default_dead_zone(80);
  rc[CH_5]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  rc[CH_6]->set_angle(4500);
  rc[CH_6]->set_default_dead_zone(80);
  rc[CH_6]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  rc[CH_7]->set_angle(4500);
  rc[CH_7]->set_default_dead_zone(80);
  rc[CH_7]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  rc[CH_8]->set_range(0,1000);
  for (int i=0; i<NUM_CHANNELS; i++) {
    rc[i]->enable_out();
  }
  if((unsigned) rc[CH_8]->control_in>750)
  {
    mode = false;
  } 
  else {
    mode = true;
  }
}

void loop()
{
  // Read PWM
  RC_Channel::set_pwm_all();
  //  // Store PWM values for pitch, roll, and yaw
  //  val[CH_1]=rc[CH_1]->control_in;
  //  val[CH_2]=rc[CH_2]->control_in;
  //  val[CH_4]=rc[CH_4]->control_in;
  // Display input
  hal.console->printf("C IN: ");
  for (int i=0; i<NUM_CHANNELS; i++) {
    hal.console->printf("CH%u: %u| ",
    (unsigned)i+1,
    rc[i]->control_in);
  }
  hal.console->println();

  //  // Pass through
  //  for (int i=0; i<NUM_CHANNELS; i++) {
  //    rc[i]->servo_out = rc[i]->control_in;
  //    rc[i]->calc_pwm();
  //    rc[i]->output();
  //  }

  // Change operating mode based on tilt servo position
  if((unsigned) rc[CH_8]->control_in>750){
    // 8 pulled high: vertical mode
    if (mode){
      setup_vert();
      mode = false;
    }
    // Read PWM
    RC_Channel::set_pwm_all();
    // Pass through
    for (int i=0; i<4; i++) {
      rc[i]->servo_out = rc[i]->control_in;
      rc[i]->calc_pwm();
      rc[i]->output();
    }
    // Assign Roll on CH_1 to Ailerons on CH_5
    rc[CH_5]->servo_out = rc[CH_5]->radio_trim;
    rc[CH_5]->calc_pwm();
    rc[CH_5]->output();
    // Assign Pitch on CH_2 to Elevators on CH_6
    rc[CH_6]->servo_out = rc[CH_6]->radio_trim;
    rc[CH_6]->calc_pwm();
    rc[CH_6]->output();
    // Assign Yaw CH_4 to Rudder on CH_7
    rc[CH_7]->servo_out = rc[CH_7]->radio_trim;
    rc[CH_7]->calc_pwm();
    rc[CH_7]->output();
    // Pass through tilt servo on CH_8
    rc[CH_8]->servo_out = rc[CH_8]->control_in;
    rc[CH_8]->calc_pwm();
    rc[CH_8]->output();
  }
  else{
    // 8 low: horizontal mode
    if (!mode){
      setup_hori();
      mode = true;
    }

    // Setup channel type for input
    rc[CH_1]->set_angle(4500);
    rc[CH_1]->set_default_dead_zone(80);
    rc[CH_1]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc[CH_2]->set_angle(4500);
    rc[CH_2]->set_default_dead_zone(80);
    rc[CH_2]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc[CH_3]->set_range(0,1000);
    rc[CH_3]->set_default_dead_zone(20);

    rc[CH_4]->set_angle(6000);
    rc[CH_4]->set_default_dead_zone(80);
    rc[CH_4]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    // Read PWM
    RC_Channel::set_pwm_all();
    // Setup channel type for output
    // Setup channel type for output
    rc[CH_1]->set_range(0,1000);
    rc[CH_1]->set_default_dead_zone(20);

    rc[CH_2]->set_range(0,1000);
    rc[CH_2]->set_default_dead_zone(20);

    rc[CH_3]->set_range(0,1000);
    rc[CH_3]->set_default_dead_zone(20);

    rc[CH_4]->set_range(0,1000);
    rc[CH_4]->set_default_dead_zone(20);
    // Slave motors together
    rc[CH_1]->servo_out = rc[CH_3]->control_in;
    rc[CH_1]->calc_pwm();
    rc[CH_1]->output();
    rc[CH_2]->servo_out = rc[CH_3]->control_in;
    rc[CH_2]->calc_pwm();
    rc[CH_2]->output();
    rc[CH_3]->servo_out = rc[CH_3]->control_in;
    rc[CH_3]->calc_pwm();
    rc[CH_3]->output();
    rc[CH_4]->servo_out = rc[CH_3]->control_in;
    rc[CH_4]->calc_pwm();
    rc[CH_4]->output();

    // Assign Roll on CH_1 to Ailerons on CH_5
    rc[CH_5]->servo_out = rc[CH_1]->control_in;
    rc[CH_5]->calc_pwm();
    rc[CH_5]->output();
    // Assign Pitch on CH_2 to Elevators on CH_6
    rc[CH_6]->servo_out = rc[CH_2]->control_in;
    rc[CH_6]->calc_pwm();
    rc[CH_6]->output();
    // Assign Yaw CH_4 to Rudder on CH_7
    rc[CH_7]->servo_out = rc[CH_4]->control_in;
    rc[CH_7]->calc_pwm();
    rc[CH_7]->output();
    // Pass through tilt servo on CH_8
    rc[CH_8]->servo_out = rc[CH_8]->control_in;
    rc[CH_8]->calc_pwm();
    rc[CH_8]->output();
  }


  hal.console->printf("S OUT: ");
  for (int i=0; i<NUM_CHANNELS; i++) {
    hal.console->printf("CH%u: %u| ",
    (unsigned)i+1,
    rc[i]->servo_out);
  }
  hal.console->println();
  hal.console->printf("R OUT: ");
  for (int i=0; i<NUM_CHANNELS; i++) {
    hal.console->printf("CH%u: %u| ",
    (unsigned)i+1,
    rc[i]->radio_out);
  }
  hal.console->println();
  hal.scheduler->delay(500);
}

static void setup_hori()
{
  //  // Setup channel type for output
  //  rc[CH_1]->set_angle(4500);
  //  rc[CH_1]->set_default_dead_zone(80);
  //  rc[CH_1]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  //
  //  rc[CH_2]->set_angle(4500);
  //  rc[CH_2]->set_default_dead_zone(80);
  //  rc[CH_2]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  //
  //  rc[CH_3]->set_range(0,1000);
  //  rc[CH_3]->set_default_dead_zone(20);
  //
  //  rc[CH_4]->set_angle(6000);
  //  rc[CH_4]->set_default_dead_zone(80);
  //  rc[CH_4]->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  //  hal.console->printf("Setup Hori\n");
}

static void setup_vert()
{

  //  // Setup channel type for output
  //  rc[CH_1]->set_range(0,1000);
  //  rc[CH_1]->set_default_dead_zone(20);
  //
  //  rc[CH_2]->set_range(0,1000);
  //  rc[CH_2]->set_default_dead_zone(20);
  //
  //  rc[CH_3]->set_range(0,1000);
  //  rc[CH_3]->set_default_dead_zone(20);
  //
  //  rc[CH_4]->set_range(0,1000);
  //  rc[CH_4]->set_default_dead_zone(20);
  //    hal.console->printf("Setup Vert\n");
}

AP_HAL_MAIN();



































