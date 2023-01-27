/*  ---------------------------
    TURTLEBOT 3 SLAM 
    turtlebot3_controller.h
    by Dhonan Nabil Hibatullah
    --------------------------- */


#ifndef TURTLEBOT3_CONTROLLER_H
#define TURTLEBOT3_CONTROLLER_H


/* C STD LIBRARIES */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* WEBOTS LIBRARIES */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/inertial_unit.h>

/* CUSTOM LIBRARY */
#include "custom_datatype.h"

/* DEFINITIONS */
#define PI                          3.14159265358979323846
#define TIME_STEP                   32
#define TIME_STEP_S                 0.032

// turtlebot3 specs
#define TURTLEBOT3_WIDTH            0.178
#define TURTLEBOT3_WHEEL_RADIUS     0.033
#define TURTLEBOT3_LIDAR_MAX_RANGE  3.5
#define TURTLEBOT3_LIDAR_WIDTH      360
#define TURTLEBOT3_MAX_SPEED        3
#define IMU_TRUST_CONST             0.8
#define IMU_SUDDEN_JUMP_CONST       0.2

// for turtlebot3_mapping_braitenberg()
#define FORWARD             0
#define TURN_RIGHT          1
#define PERP_RIGHT          2
#define TURN_LEFT           3
#define PERP_LEFT           4
#define ROTATE              5
#define DIST_THRESHOLD      0.23
#define NARROW_THRESHOLD    0.3
#define FRONT               180
#define FRONT_LEFT          150
#define LEFT                90
#define BACK_LEFT           45
#define BACK                0
#define BACK_RIGHT          315
#define RIGHT               270
#define FRONT_RIGHT         210

// for turtlebot3_go_to_target()
#define HEADING         0
#define DIRECT_WISE     1
#define RIGHT_WISE      2
#define LEFT_WISE       3
#define STOP            4
#define TURNING_GAIN    0.7
#define MAX_DIST        0.3
#define MIN_DIST        1.5


/* ---------------------- TURTLEBOT3 STRUCTURE ---------------------- */
void turtlebot3_init(Turtlebot3* robot) {
    
    /* initiate all the value */
    robot->pose.x     = 0.0;
    robot->pose.y     = 0.0;
    robot->pose.theta = 0.0;
    robot->vel_r      = 0.0;
    robot->vel_l      = 0.0;

    /* get devices */
    robot->imu                   = wb_robot_get_device("inertial unit");
    robot->lidar                 = wb_robot_get_device("LDS-01");
    robot->lidar_main_motor      = wb_robot_get_device("LDS-01_main_motor");
    robot->lidar_secondary_motor = wb_robot_get_device("LDS-01_secondary_motor");
    robot->motor_r               = wb_robot_get_device("right wheel motor");
    robot->motor_l               = wb_robot_get_device("left wheel motor");

    /* initiate imu */
    wb_inertial_unit_enable(robot->imu, TIME_STEP);

    /* initiate lidar */
    wb_lidar_enable(robot->lidar, TIME_STEP);
    wb_lidar_enable_point_cloud(robot->lidar);
    for (int i = 0; i < 360; ++i) {
        robot->lidar_angle_offset[i] = (-1.0)*((float)i)*2.0*PI/360.0;
    }

    /* initiate motors */
    wb_motor_set_position(robot->lidar_main_motor, INFINITY);
    wb_motor_set_position(robot->lidar_secondary_motor, INFINITY);
    wb_motor_set_position(robot->motor_r, INFINITY);
    wb_motor_set_position(robot->motor_l, INFINITY);
    wb_motor_set_velocity(robot->lidar_main_motor, 20.0);
    wb_motor_set_velocity(robot->lidar_secondary_motor, 40.0);
    wb_motor_set_velocity(robot->motor_r, 0.0);
    wb_motor_set_velocity(robot->motor_l, 0.0);
}


void turtlebot3_set_velocity(Turtlebot3* robot, float vel_r, float vel_l) {
    /* set motor speed */
    robot->vel_r = vel_r;
    robot->vel_l = vel_l;
}


void turtlebot3_get_lidar(Turtlebot3* robot) {
    /* get lidar data */
    robot->lidar_data_raw = wb_lidar_get_range_image(robot->lidar);
}


/* ---------------------- KINEMATICS AND CONTROL ---------------------- */
void turtlebot3_mapping_braitenberg(Turtlebot3* robot) {
    /* last state */
    static int      movement_state  = FORWARD,
                    action          = 0;

    static float    starting_angle  = 0;

    /* finite state */
    if (action == 1) {
        starting_angle = robot->pose.theta;
        action = 0;
    }
    switch (movement_state) {
        
        case FORWARD:
            robot->vel_r = TURTLEBOT3_MAX_SPEED;
            robot->vel_l = TURTLEBOT3_MAX_SPEED;
            if (robot->lidar_data_raw[FRONT]       < DIST_THRESHOLD ||
                robot->lidar_data_raw[FRONT_LEFT]  < DIST_THRESHOLD ||
                robot->lidar_data_raw[FRONT_RIGHT] < DIST_THRESHOLD) {
                if (robot->lidar_data_raw[LEFT] < NARROW_THRESHOLD && robot->lidar_data_raw[RIGHT] < NARROW_THRESHOLD) {
                    movement_state = ROTATE;
                }
                else if (fabs(robot->lidar_data_raw[FRONT_RIGHT] - robot->lidar_data_raw[FRONT_LEFT]) < 0.3) {
                    if (robot->lidar_data_raw[RIGHT] > robot->lidar_data_raw[LEFT]) {
                        movement_state = PERP_RIGHT;
                    }
                    else if (robot->lidar_data_raw[LEFT] > robot->lidar_data_raw[RIGHT]) {
                        movement_state = PERP_LEFT;
                    }
                }  
                else if (robot->lidar_data_raw[RIGHT] > NARROW_THRESHOLD && robot->lidar_data_raw[RIGHT] > robot->lidar_data_raw[LEFT]) {
                    movement_state = TURN_RIGHT;
                    action = 1;
                }
                else if (robot->lidar_data_raw[LEFT] > NARROW_THRESHOLD && robot->lidar_data_raw[LEFT] > robot->lidar_data_raw[RIGHT]) {
                    movement_state = TURN_LEFT;
                    action = 1;
                }
            }
            break;

        case TURN_RIGHT:
            robot->vel_r = -TURTLEBOT3_MAX_SPEED;
            robot->vel_l = TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/8.0) ? FORWARD : TURN_RIGHT;
            break;

        case PERP_RIGHT:
            robot->vel_r = -TURTLEBOT3_MAX_SPEED;
            robot->vel_l = TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/2.0) ? FORWARD : PERP_RIGHT;
            break;
        
        case TURN_LEFT:
            robot->vel_r = TURTLEBOT3_MAX_SPEED;
            robot->vel_l = -TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/8.0) ? FORWARD : TURN_LEFT;
            break;

        case PERP_LEFT:
            robot->vel_r = TURTLEBOT3_MAX_SPEED;
            robot->vel_l = -TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/2.0) ? FORWARD : PERP_LEFT;
            break;
        
        case ROTATE:
            robot->vel_r    = -TURTLEBOT3_MAX_SPEED;
            robot->vel_l    = TURTLEBOT3_MAX_SPEED;
            movement_state  = (fabs(robot->pose.theta - starting_angle) > PI) ? FORWARD : ROTATE;
            break;
    }
}


void turtlebot3_go_to_target(Turtlebot3* robot, Map* map, Coor2D* target) {
    /* last state */
    static int   movement_state = HEADING,
                 state_choosen  = LEFT_WISE,
                 case_start     = 1;

    static float target_angle   = 0.0,
                 starting_angle = 0.0;
    
    /* determining target point */
    map_m2w(target);
    float   target_x = target->x_world,
            target_y = target->y_world;

    /* path finding algorithm */
    switch (movement_state) {
        
        case HEADING:
            /* find heading direction */
            if (case_start == 1) {
                float   direction_x = target_x - robot->pose.x,
                        direction_y = target_y - robot->pose.y;
                target_angle   = atan2(direction_y, direction_x);
                target_angle   = (target_angle < 0) ? 2*PI + target_angle : target_angle;
                starting_angle = robot->pose.theta;
                
                if (starting_angle - target_angle < 0) {
                    robot->vel_r = TURTLEBOT3_MAX_SPEED;
                    robot->vel_l = -TURTLEBOT3_MAX_SPEED;
                }
                else {
                    robot->vel_r = -TURTLEBOT3_MAX_SPEED;
                    robot->vel_l = TURTLEBOT3_MAX_SPEED;
                }
                case_start = 0;
                state_choosen = HEADING;
            }

            /* heading to target */
            if (fabs(robot->pose.theta - target_angle) < 0.15) {
                movement_state = DIRECT_WISE;
            }
            return;
            break;

        case DIRECT_WISE:
            robot->vel_r = TURTLEBOT3_MAX_SPEED;
            robot->vel_l = TURTLEBOT3_MAX_SPEED;
            
            if (state_choosen == LEFT_WISE && robot->lidar_data_raw[RIGHT] > MIN_DIST) {
                movement_state = HEADING;
                case_start = 1;
            }

            else if (state_choosen == RIGHT_WISE && robot->lidar_data_raw[RIGHT] > MIN_DIST) {
                movement_state = HEADING;
                case_start = 1;
            }

            if (robot->lidar_data_raw[FRONT]       < MAX_DIST ||
                robot->lidar_data_raw[FRONT_LEFT]  < MAX_DIST ||
                robot->lidar_data_raw[FRONT_RIGHT] < MAX_DIST) {
                    if (robot->lidar_data_raw[FRONT_LEFT] > robot->lidar_data_raw[FRONT_RIGHT])
                        movement_state = LEFT_WISE;
                    else
                        movement_state = RIGHT_WISE;
                    state_choosen = movement_state;
                    case_start = 1;
            }

            break;

        case RIGHT_WISE:
            if (case_start == 1) {
                starting_angle = robot->pose.theta;
                case_start = 0;
            }
            robot->vel_l = TURTLEBOT3_MAX_SPEED;
            robot->vel_r = -TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/3.6) ? DIRECT_WISE : RIGHT_WISE;
            break;

        case LEFT_WISE:
            if (case_start == 1) {
                starting_angle = robot->pose.theta;
                case_start = 0;
            }
            robot->vel_l = -TURTLEBOT3_MAX_SPEED;
            robot->vel_r = TURTLEBOT3_MAX_SPEED;
            movement_state = (fabs(robot->pose.theta - starting_angle) > PI/3.6) ? DIRECT_WISE : LEFT_WISE;
            break;

        case STOP:
            robot->vel_r = 0;
            robot->vel_l = 0;
            break;
    }

    if (fabs(target_x - robot->pose.x) < 0.1 && fabs(target_y - robot->pose.y) < 0.1)
        movement_state = STOP;
}


void turtlebot3_kinematics(Turtlebot3* robot) {
    /* calculate velocity based from kinematics */
    float  v_x, v_y, omega;
    v_x     = (robot->vel_r + robot->vel_l)*cos(robot->pose.theta)*TURTLEBOT3_WHEEL_RADIUS/2.0;
    v_y     = (robot->vel_r + robot->vel_l)*sin(robot->pose.theta)*TURTLEBOT3_WHEEL_RADIUS/2.0;
    omega   = (robot->vel_r - robot->vel_l)*TURTLEBOT3_WHEEL_RADIUS/TURTLEBOT3_WIDTH;

    /* get yaw angle from imu */
    const double *roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(robot->imu);
    double yaw = roll_pitch_yaw[2];

    /* initialize offset */
    static int offset_imu = 0;
    if (offset_imu == 0) {
        robot->imu_offset = yaw;
        offset_imu = 1;
    }

    /* normalize angle */
    yaw -= robot->imu_offset;
    yaw  = (yaw < 0.0) ? 2*PI + yaw : yaw;

    /* calculate result */
    robot->pose.x = robot->pose.x + v_x*TIME_STEP_S;
    robot->pose.y = robot->pose.y + v_y*TIME_STEP_S;
    if (fabs(yaw - robot->pose.theta) > IMU_SUDDEN_JUMP_CONST) {
        robot->pose.theta = fmod(robot->pose.theta + omega*TIME_STEP_S, 2.0*PI);
    }
    else {
        robot->pose.theta = (1.0 - IMU_TRUST_CONST)*fmod(robot->pose.theta + omega*TIME_STEP_S, 2.0*PI) + (IMU_TRUST_CONST)*yaw;
    }
}


void turtlebot3_run(Turtlebot3* robot) {
    /* run the motors */
    wb_motor_set_velocity(robot->motor_r, robot->vel_r);
    wb_motor_set_velocity(robot->motor_l, robot->vel_l);

    /* run the kinematics */
    turtlebot3_kinematics(robot);
}





#endif