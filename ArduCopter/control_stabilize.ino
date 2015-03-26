/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */
#define ail 0
#define ele 1
#define rud 2
// How many iterations at 100hz before the servos are moved
#define refresh 2
// Value to store trim
static uint16_t trim[3];
static uint8_t loop_count;
static bool vert;


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
  g.rc_5.control_in = trim[ail];
  g.rc_5.servo_out = trim[ail];
  g.rc_5.radio_trim = trim[ail];
  g.rc_5.calc_pwm();
  g.rc_5.output();
  g.rc_6.servo_out = trim[ele];
  g.rc_6.radio_trim = trim[ele];
  g.rc_6.calc_pwm();
  g.rc_6.output();
  g.rc_7.servo_out = trim[rud];
  g.rc_7.radio_trim = trim[rud];
  g.rc_7.calc_pwm();
  g.rc_7.output();

  // Set the update rate of CH5 to 50hz
  uint32_t mask = 0;
  mask |= 1U << 4;
  hal.rcout->set_freq(mask,50);
  // Initialize loop counter
  loop_count = 1;
  // Initial mode boolean
  if(g.rc_8.control_in>=800){
    vert = true;
  }
  else{
    vert = false;
  }
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

  // Set the update rate of CH5 to 50hz
  uint32_t mask = 0;
  mask |= 1U << 4;
  hal.rcout->set_freq(mask,50);

  // Check mode switch
  if(loop_count = refresh){
    if(g.rc_8.control_in>=800){
      vert = true;
    }
    else{
      vert = false;
    }
  }

  // Check tilt servo input
  if(vert){
    // if not armed set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
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

    if(loop_count = refresh){

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
  }

  else{
    if(loop_count = refresh){
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

      // Pass through tilt servo on CH_8
      g.rc_8.servo_out = g.rc_8.control_in;
      g.rc_8.calc_pwm();
      g.rc_8.output();
    }
    // Motor Output
    if(g.rc_3.control_in <= 0) {
      attitude_control.relax_bf_rate_controller();
      attitude_control.set_yaw_target_to_current_heading();
      attitude_control.set_throttle_out(0, false);
    }
    else
    {
      // output pilot's throttle
      motors.throttle_pass_through();
    }

  }
  // Reset Counter
  if(loop_count = refresh){
    loop_count = 1;
  }
  else{
    loop_count++;
  }
}






































