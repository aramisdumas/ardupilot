/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
#define ail 0
#define ele 1
#define rud 2
// Value to store trim
static uint16_t trim[3];

// stabilize_init - initialise stabilize controller
static bool stabilize_init(bool ignore_checks)
{
  // set target altitude to zero for reporting
  // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
  pos_control.set_alt_target(0);
  // Store trim values for aileron, elevator, and rudder
  trim[ail] = g.rc_1.radio_trim;
  trim[ele] = g.rc_2.radio_trim;
  trim[rud] = g.rc_4.radio_trim;
  // set aileron, elevator, and rudder to radio trim positions
  g.rc_5.servo_out = trim[ail];
  g.rc_5.calc_pwm();
  g.rc_5.output();
  g.rc_6.servo_out = trim[ele];
  g.rc_6.calc_pwm();
  g.rc_6.output();
  g.rc_7.servo_out = trim[rud];
  g.rc_7.calc_pwm();
  g.rc_7.output();

  // stabilize should never be made to fail
  return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
  int16_t target_roll, target_pitch;
  float target_yaw_rate;
  int16_t pilot_throttle_scaled;
  // Setup channels, strange that it needs to be run constantly but it does
  g.rc_5.set_angle(4500);
  g.rc_5.set_default_dead_zone(80);
  g.rc_5.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  g.rc_6.set_angle(4500);
  g.rc_6.set_default_dead_zone(80);
  g.rc_6.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
  g.rc_7.set_angle(4500);
  g.rc_7.set_default_dead_zone(80);
  g.rc_7.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

  // Check tilt servo input
  if(g.rc_8.control_in>=800){
    // if not armed set throttle to zero and exit immediately
    if(!motors.armed()) {
      attitude_control.relax_bf_rate_controller();
      attitude_control.set_yaw_target_to_current_heading();
      attitude_control.set_throttle_out(0, false);
      return;
    }
    // 8 pulled high: vertical mode
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
    // set aileron, elevator, and rudder to radio trim positions
    g.rc_5.servo_out = trim[ail];
    g.rc_5.calc_pwm();
    g.rc_5.output();
    g.rc_6.servo_out = trim[ele];
    g.rc_6.calc_pwm();
    g.rc_6.output();
    g.rc_7.servo_out = trim[rud];
    g.rc_7.calc_pwm();
    g.rc_7.output();

  }
  else{
    // 8 pulled low: horizontal mode

      // Output roll, pitch, and yaw to ailerons, elevator, and rudder
    RC_Channel::set_pwm_all();
    g.rc_5.servo_out = g.rc_1.control_in;
    g.rc_6.servo_out = g.rc_2.control_in;
    g.rc_7.servo_out = g.rc_4.control_in;
    g.rc_5.calc_pwm();
    g.rc_6.calc_pwm();
    g.rc_7.calc_pwm();
    g.rc_5.output();
    g.rc_6.output();
    g.rc_7.output();

//    // Failsafe (maybe) still have motor control with no smoothing if motors becomes disarmed
//    if(!motors.armed()) {
//      // Setup channel type for output
//      g.rc_1.set_range(g.throttle_min, g.throttle_max);
//      g.rc_1.set_default_dead_zone(30);
//      g.rc_2.set_range(g.throttle_min, g.throttle_max);
//      g.rc_2.set_default_dead_zone(30);
//      g.rc_4.set_range(g.throttle_min, g.throttle_max);
//      g.rc_4.set_default_dead_zone(30);
//      // Assign value to motor channels
//      g.rc_1.servo_out = g.rc_3.control_in;
//      g.rc_2.servo_out = g.rc_3.control_in;
//      g.rc_3.servo_out = g.rc_3.control_in;
//      g.rc_4.servo_out = g.rc_3.control_in;
//      g.rc_1.calc_pwm();
//      g.rc_2.calc_pwm();
//      g.rc_3.calc_pwm();
//      g.rc_4.calc_pwm();
//      // Setup channel type back
//      g.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
//      g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//      g.rc_1.set_default_dead_zone(30);
//      g.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
//      g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//      g.rc_2.set_default_dead_zone(30);
//      g.rc_4.set_angle(4500);
//      g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
//      g.rc_4.set_default_dead_zone(40);
//      // Output to motors
//      g.rc_1.output();
//      g.rc_3.output();
//      g.rc_2.output();
//      g.rc_4.output();
//    }
//    else{
//
//    }
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
  }
  // Pass through tilt servo on CH_8
  g.rc_8.servo_out = g.rc_8.control_in;
  g.rc_8.calc_pwm();
  g.rc_8.output();
}



























