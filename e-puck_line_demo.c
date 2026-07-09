/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0  
#define REALITY 2     
#define TIME_STEP 32  

// Sensors & Actuators Setup
#define NB_DIST_SENS 8
#define NB_GROUND_SENS 3
#define NB_LEDS 8

// Sensor index mappings
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_LEFT_45 6
#define PS_LEFT_00 7
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
const int PS_OFFSET_REALITY[NB_DIST_SENS] = {480, 170, 320, 500, 600, 680, 210, 640};

WbDeviceTag ps[NB_DIST_SENS], gs[NB_GROUND_SENS], led[NB_LEDS], left_motor, right_motor;
int ps_value[NB_DIST_SENS] = {0};
unsigned short gs_value[NB_GROUND_SENS] = {0};

// Module Speed Arrays & Control Flags
int lfm_speed[2], oam_speed[2], ofm_speed[2], lem_speed[2];
int oam_active, oam_reset, oam_side = NO_SIDE;
int llm_active = FALSE, llm_inibit_ofm_speed, llm_past_side = NO_SIDE, lem_reset;
int lem_active, lem_state, lem_black_counter, cur_op_gs_value, prev_op_gs_value;

// Thresholds & Coefficients
#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4
#define OAM_OBST_THRESHOLD 100
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.2
#define OAM_K_PS_45 0.9
#define OAM_K_PS_00 1.2
#define OAM_K_MAX_DELTAS 600
#define LLM_THRESHOLD 550 
#define OFM_DELTA_SPEED 150
#define LEM_FORWARD_SPEED 100
#define LEM_K_GS_SPEED 0.5
#define LEM_THRESHOLD 500

#define LEM_STATE_STANDBY 0
#define LEM_STATE_LOOKING_FOR_LINE 1
#define LEM_STATE_LINE_DETECTED 2
#define LEM_STATE_ON_LINE 3

//------------------------------------------------------------------------------
//    BEHAVIORAL MODULES
//------------------------------------------------------------------------------

void LineFollowingModule(void) {
  int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];
  lfm_speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
  lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
}

void ObstacleAvoidanceModule(void) {
  int max_ds_value = 0, i, Activation[] = {0, 0};

  if (oam_reset) { oam_active = FALSE; oam_side = NO_SIDE; }
  oam_reset = 0;

  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) {
    if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
    Activation[RIGHT] += ps_value[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
    Activation[LEFT] += ps_value[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD) oam_active = TRUE;

  if (oam_active && oam_side == NO_SIDE) {
    oam_side = (Activation[RIGHT] > Activation[LEFT]) ? RIGHT : LEFT;
  }

  oam_speed[LEFT] = oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  if (oam_active) {
    int DeltaS = 0;
    if (oam_side == LEFT) {
      DeltaS -= (int)(OAM_K_PS_90 * ps_value[5] + OAM_K_PS_45 * ps_value[6] + OAM_K_PS_00 * ps_value[7]);
    } else {  
      DeltaS += (int)(OAM_K_PS_90 * ps_value[2] + OAM_K_PS_45 * ps_value[1] + OAM_K_PS_00 * ps_value[0]);
    }
    if (DeltaS > OAM_K_MAX_DELTAS) DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS) DeltaS = -OAM_K_MAX_DELTAS;

    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;
  }
}

void LineLeavingModule(int side) {
  if (!llm_active && side != NO_SIDE && llm_past_side == NO_SIDE) llm_active = TRUE;
  llm_past_side = side;

  if (llm_active) {  
    // CONCISE FIX: Merged identical LEFT and RIGHT evaluation blocks into a dynamic lookup
    int target_gs = (side == LEFT) ? GS_LEFT : GS_RIGHT;
    if ((gs_value[GS_CENTER] + gs_value[target_gs]) / 2 > LLM_THRESHOLD) {  
      llm_active = FALSE;
      llm_inibit_ofm_speed = FALSE;
      lem_reset = TRUE;
    } else {  
      llm_inibit_ofm_speed = TRUE;
    }
  }
}

void ObstacleFollowingModule(int side) {
  if (side != NO_SIDE) {
    ofm_active = TRUE;
    ofm_speed[LEFT] = (side == LEFT) ? -OFM_DELTA_SPEED : OFM_DELTA_SPEED;
    ofm_speed[RIGHT] = (side == LEFT) ? OFM_DELTA_SPEED : -OFM_DELTA_SPEED;
  } else {  
    ofm_active = FALSE;
    ofm_speed[LEFT] = ofm_speed[RIGHT] = 0;
  }
}

void LineEnteringModule(int side) {
  int Side = (side == LEFT) ? RIGHT : LEFT;      
  int OpSide = (side == LEFT) ? LEFT : RIGHT;
  int GS_Side = (side == LEFT) ? GS_RIGHT : GS_LEFT;
  int GS_OpSide = (side == LEFT) ? GS_LEFT : GS_RIGHT;

  if (lem_reset) lem_state = LEM_STATE_LOOKING_FOR_LINE;
  lem_reset = FALSE;

  lem_speed[LEFT] = lem_speed[RIGHT] = LEM_FORWARD_SPEED;

  switch (lem_state) {
    case LEM_STATE_STANDBY:
      lem_active = FALSE;
      break;
    case LEM_STATE_LOOKING_FOR_LINE:
      if (gs_value[GS_Side] < LEM_THRESHOLD) {
        lem_active = TRUE;
        lem_state = LEM_STATE_LINE_DETECTED;
        cur_op_gs_value = (gs_value[GS_OpSide] < LEM_THRESHOLD) ? BLACK : WHITE;
        lem_black_counter = (cur_op_gs_value == BLACK) ? 1 : 0;
        prev_op_gs_value = cur_op_gs_value;
      }
      break;
    case LEM_STATE_LINE_DETECTED:
      cur_op_gs_value = (gs_value[GS_OpSide] < LEM_THRESHOLD) ? BLACK : WHITE;
      if (cur_op_gs_value == BLACK) lem_black_counter++;
      
      if (prev_op_gs_value == BLACK && cur_op_gs_value == WHITE) {
        lem_state = LEM_STATE_ON_LINE;
        lem_speed[OpSide] = lem_speed[Side] = 0;
      } else {
        prev_op_gs_value = cur_op_gs_value;
        lem_speed[OpSide] = LEM_FORWARD_SPEED + LEM_K_GS_SPEED * (GS_WHITE - gs_value[GS_Side]);
        lem_speed[Side] = LEM_FORWARD_SPEED - LEM_K_GS_SPEED * (GS_WHITE - gs_value[GS_Side]);
      }
      break;
    case LEM_STATE_ON_LINE:
      oam_reset = TRUE;
      oam_side = NO_SIDE; 
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      break;
  }
}

//------------------------------------------------------------------------------
//    MAIN CONTROLLER
//------------------------------------------------------------------------------
int main() {
  int ps_offset[NB_DIST_SENS] = {0}, i, speed[2], Mode = 1, oam_ofm_speed[2];
  char name[20];

  wb_robot_init();

  // Unified Hardware Initializations
  for (i = 0; i < NB_LEDS; i++) { sprintf(name, "led%d", i); led[i] = wb_robot_get_device(name); }
  for (i = 0; i < NB_DIST_SENS; i++) { 
    sprintf(name, "ps%d", i); ps[i] = wb_robot_get_device(name); 
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  for (i = 0; i < NB_GROUND_SENS; i++) { 
    sprintf(name, "gs%d", i); gs[i] = wb_robot_get_device(name); 
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); wb_motor_set_velocity(right_motor, 0.0);

  for (;;) {  
    wb_robot_step(TIME_STEP);

    // Mode Switches (Simulation vs Real Hardware Contexts)
    if (Mode != wb_robot_get_mode()) {
      oam_reset = TRUE; llm_active = FALSE; llm_past_side = NO_SIDE;
      ofm_active = lem_active = FALSE; lem_state = LEM_STATE_STANDBY;
      Mode = wb_robot_get_mode();
      for (i = 0; i < NB_DIST_SENS; i++) {
        ps_offset[i] = (Mode == SIMULATION) ? PS_OFFSET_SIMULATION[i] : PS_OFFSET_REALITY[i];
      }
      wb_motor_set_velocity(left_motor, 0); wb_motor_set_velocity(right_motor, 0);
      wb_robot_step(TIME_STEP);  
    }

    // Process Sensors
    for (i = 0; i < NB_DIST_SENS; i++) {
      int val = (int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i];
      ps_value[i] = (val < 0) ? 0 : val;
    }
    for (i = 0; i < NB_GROUND_SENS; i++) gs_value[i] = wb_distance_sensor_get_value(gs[i]);

    // Subsumption Priority Evaluation
    LineFollowingModule();
    speed[LEFT] = lfm_speed[LEFT]; speed[RIGHT] = lfm_speed[RIGHT];

    ObstacleAvoidanceModule();
    LineLeavingModule(oam_side);
    ObstacleFollowingModule(oam_side);

    if (llm_inibit_ofm_speed) ofm_speed[LEFT] = ofm_speed[RIGHT] = 0;

    oam_ofm_speed[LEFT] = oam_speed[LEFT] + ofm_speed[LEFT];
    oam_ofm_speed[RIGHT] = oam_speed[RIGHT] + ofm_speed[RIGHT];

    if (oam_active || ofm_active) {
      speed[LEFT] = oam_ofm_speed[LEFT]; speed[RIGHT] = oam_ofm_speed[RIGHT];
    }

    LineEnteringModule(oam_side);
    if (lem_active) {
      speed[LEFT] = lem_speed[LEFT]; speed[RIGHT] = lem_speed[RIGHT];
    }

    // Status Logs
    if (oam_active) {
      printf("[STATUS] Obstacle Detected on %s! -> Steering Away.\n", (oam_side == RIGHT) ? "RIGHT" : "LEFT");
    } else if (lem_active) {
      printf("[STATUS] %s\n", (lem_state == LEM_STATE_LOOKING_FOR_LINE) ? "Searching for track..." : "Re-aligning onto line.");
    } else {
      printf("[STATUS] Track Clear. Safely Following Line.\n");
    }

    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
  }
  return 0;
}