/*  ---------------------------
    TURTLEBOT 3 SLAM 
    custom_datatype.h
    by Dhonan Nabil Hibatullah
    --------------------------- */


#ifndef CUSTOM_DATATYPE_H
#define CUSTOM_DATATYPE_H

#define MAP_SIZE        256
#define WORLD_MAP_RATIO 0.037


typedef struct pose2d {

    float  x,
           y,
           theta;

} Pose2D;


typedef struct coor2d {
    
    float   x_world,
            y_world;

    int     x_map,
            y_map;

} Coor2D;


typedef struct houghbase {
    int x,
        y,
        theta,
        gain;
} HoughBase;


typedef struct map {

    int             map[MAP_SIZE][MAP_SIZE],
                    // hough_map[MAP_SIZE][MAP_SIZE],
                    display_height,
                    display_width;

    Coor2D          robot_pos;

    WbDeviceTag     display;

    WbImageRef      background;

} Map;


typedef struct turtlebot_3 {

    Pose2D          pose;

    Coor2D          lidar_data[360];

    float*          lidar_data_raw;

    float           vel_r,
                    vel_l,
                    lidar_angle_offset[360];

    double          imu_offset;

    WbDeviceTag     imu,
                    lidar,
                    lidar_main_motor,
                    lidar_secondary_motor,
                    motor_r,
                    motor_l;

} Turtlebot3;


typedef struct path {

} Path;


void map_w2m(Coor2D* pos) {
    pos->x_map = 127 + (int)round(pos->x_world/WORLD_MAP_RATIO);
    pos->y_map = 127 + (int)round(pos->y_world/WORLD_MAP_RATIO);
}


void map_m2w(Coor2D* pos) {
    pos->x_world = ((float)(pos->x_map - 127))*WORLD_MAP_RATIO;
    pos->y_world = ((float)(pos->y_map - 127))*WORLD_MAP_RATIO;
}


#endif