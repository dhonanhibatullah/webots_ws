/*  ---------------------------
    TURTLEBOT 3 SLAM 
    map_slam_display.h
    by Dhonan Nabil Hibatullah
    --------------------------- */


#ifndef SLAM_DISPLAY_H
#define SLAM_DISPLAY_H


/* C STD LIBRARIES */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* WEBOTS LIBRARIES */
#include <webots/display.h>

/* CUSTOM LIBRARY */
#include "custom_datatype.h"

/* DEFINITIONS */
#define PI                  3.14159265358979323846
#define TIME_STEP           32
#define TIME_STEP_S         0.032

#define MAX_LIDAR_DATA      360

#define CORRELATION_CONST   0.95


/* ---------------------- MAP STRUCTURE ---------------------- */
void map_init(Map* map) {
    /* initiate values */
    map->robot_pos.x_world  = 0.0;
    map->robot_pos.y_world  = 0.0;
    map->robot_pos.x_map    = 0;
    map->robot_pos.y_map    = 0;

    /* initiate display with a white color */
    map->display        = wb_robot_get_device("display");
    map->display_height = wb_display_get_width(map->display);
    map->display_width  = wb_display_get_height(map->display);
    wb_display_fill_rectangle(map->display, 0, 0, map->display_width, map->display_height);
    map->background     = wb_display_image_copy(map->display, 0, 0, map->display_width, map->display_height);
}


double map_correlate(int map1[MAP_SIZE][MAP_SIZE], int map2[MAP_SIZE][MAP_SIZE]) {
    /* do the correlate operation */
    double  correlation_res = 360.0;
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            if (map1[i][j] != map2[i][j])
                correlation_res -= 1.0;
        }
    }
    correlation_res /= 360.0;
    return correlation_res;
}


void map_slam(Map* map, Turtlebot3* robot, int slam_state) {
    /* get the latest mapping state */
    wb_display_image_paste(map->display, map->background, 0, 0, false);

    /* for move straight only */
    if (robot->vel_r == robot->vel_l) {
        for (int i = 0; i < MAX_LIDAR_DATA; ++i) {
            if (robot->lidar_data_raw[i] != INFINITY) {
                robot->lidar_data[i].x_world = robot->lidar_data_raw[i]*cos(robot->pose.theta + robot->lidar_angle_offset[i]) - robot->pose.x;
                robot->lidar_data[i].y_world = robot->lidar_data_raw[i]*sin(robot->pose.theta + robot->lidar_angle_offset[i]) - robot->pose.y;
            }
            else {
                robot->lidar_data[i].x_world = 0;
                robot->lidar_data[i].y_world = 0;
            }
            /* transform into map coordinate */
            map_w2m(robot->lidar_data + i);
            robot->lidar_data[i].x_map = map->display_width - robot->lidar_data[i].x_map;
            map->map[robot->lidar_data[i].x_map][robot->lidar_data[i].y_map] = 1;
        }
    }

    /* simultaneous vehicle and map angle correction */
    double  correction_angle = 0.0,
            correlation_val  = 0.0;
    int     new_map[MAP_SIZE][MAP_SIZE];
    while (1) {
        if (robot->vel_r == robot->vel_l) break;

        /* copy the last map into new map */
        memcpy(new_map, map->map, sizeof(int)*MAP_SIZE*MAP_SIZE);

        /* rotating map */
        for (int i = 0; i < MAX_LIDAR_DATA; ++i) {
            if (robot->lidar_data_raw[i] != INFINITY) {
                robot->lidar_data[i].x_world = robot->lidar_data_raw[i]*cos(robot->pose.theta + robot->lidar_angle_offset[i]) - robot->pose.x;
                robot->lidar_data[i].y_world = robot->lidar_data_raw[i]*sin(robot->pose.theta + robot->lidar_angle_offset[i]) - robot->pose.y;
            }
            else {
                robot->lidar_data[i].x_world = 0;
                robot->lidar_data[i].y_world = 0;
            }
            /* transform into map coordinate */
            map_w2m(robot->lidar_data + i);
            robot->lidar_data[i].x_map = map->display_width - robot->lidar_data[i].x_map;
            new_map[robot->lidar_data[i].x_map][robot->lidar_data[i].y_map] = 1;
        }

        /* take the correlation */
        correlation_val = map_correlate(map->map, new_map);
        if (correlation_val >= CORRELATION_CONST) {
            memcpy(map->map, new_map, sizeof(int)*MAP_SIZE*MAP_SIZE);
            break;
        }
        else if (fabs(correction_angle) >= PI/45.) {
            break;
        }

        /* add the correction angle */
        if (robot->vel_r > robot->vel_l) 
            correction_angle -= PI/180.;
        else
            correction_angle += PI/180.;
        robot->pose.theta -= correction_angle;
    }

    /* display the processed map on display */
    if (slam_state == 0) {
    wb_display_set_color(map->display, 0x000000);
        for (int i = 0; i < MAP_SIZE; ++i) {
            for (int j = 0; j < MAP_SIZE; ++j) {
                if (map->map[i][j] == 1)
                    wb_display_draw_rectangle(map->display, i, j, 1, 1);
            }
        }
        /* delete previous robot location */
        wb_display_set_color(map->display, 0xFFFFFF);
        wb_display_draw_rectangle(map->display, map->robot_pos.x_map, map->robot_pos.y_map, 1, 1);
    }

    /* plot robot position */
    map->robot_pos.x_world = robot->pose.x;
    map->robot_pos.y_world = robot->pose.y;
    map_w2m(&(map->robot_pos));
    map->robot_pos.y_map = map->display_height - map->robot_pos.y_map;
    wb_display_set_color(map->display, 0xFF0000);
    wb_display_draw_rectangle(map->display, map->robot_pos.x_map, map->robot_pos.y_map, 1, 1);

    /* refresh the map */
    wb_display_image_delete(map->display, map->background);
    map->background = wb_display_image_copy(map->display, 0, 0, map->display_width, map->display_height);
}


// /* update-filter with hough transform */
    // int         used_point[MAX_LIDAR_DATA] = {0},
    //             hough_iter = 0;
    // HoughBase   hough_param[100];
    // for (int i = 0; i < MAX_LIDAR_DATA; ++i) {
    //     /* if point already used */
    //     if (used_point[i] == 1) continue;

    //     /* define line equation */
    //     double x_0, y_0, g, theta = 0;

    //     /* checks some angles */
    //     while (theta <= PI) {
    //         x_0 = data[i].x_map;
    //         y_0 = data[i].y_map;
    //         if (fabs(theta - PI) < 0.001) g = tan(theta - 0.01);
    //         else                          g = tan(theta);

    //         for (int j = 0; j < MAX_LIDAR_DATA; ++j) {
    //             /* continue the unnecessary points */
    //             if (i == j) continue;
    //             if (used_point[j] == 1) continue;

    //             /* calculate the distance */
    //             double dist = fabs(data[j].y_map - g*data[j].x_map + (x_0*g - y_0))/sqrt(pow(g, 2) + 1.0);
    //             if (dist < 1.2) {
    //                 /* take the gain */
    //                 hough_param[hough_iter].gain += 1;
    //                 /* mark the used point */
    //                 used_point[j] = 1;
    //             }
    //         }
    //         if (hough_param[hough_iter].gain > 40) {
    //             hough_param[hough_iter].x     = x_0;
    //             hough_param[hough_iter].y     = y_0;
    //             hough_param[hough_iter].theta = theta;
    //             ++hough_iter;
    //         }
    //         else hough_param[hough_iter].gain = 0;

    //         /* increment degree */
    //         theta += PI/40.0;
    //     }
    // }

    // /* convert the hough basis to the image space */
    // wb_display_set_color(map->display, 0x0000FF);
    // for (int i = 0; i < hough_iter; ++i) {
    //     double  x_0     = hough_param[i].x,
    //             y_0     = hough_param[i].y,
    //             g;
    //     if (fabs(hough_param[i].theta - PI) < 0.1)  g = tan(hough_param[i].theta - 0.01);
    //     else                                        g = tan(hough_param[i].theta);

    //     for (int j = 0; j < MAP_SIZE; ++j) {
    //         int x_val = j,
    //             y_val = (int)(g*((double)x_val - x_0) + y_0);

    //         if (0 > y_val || y_val >= MAP_SIZE) continue;
    //         else {
    //             map->hough_map[x_val][y_val] = 1;
    //             wb_display_draw_rectangle(map->display, x_val, y_val, 1, 1);
    //         }
    //     }
    // }


#endif