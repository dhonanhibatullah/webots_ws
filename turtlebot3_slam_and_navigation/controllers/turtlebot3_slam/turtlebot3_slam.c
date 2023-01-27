/*  ---------------------------
    TURTLEBOT 3 SLAM 
    by Dhonan Nabil Hibatullah
    --------------------------- */

#include "turtlebot3_controller.h"
#include "map_slam_display.h"

#define MAP   0
#define SLAM  1

void stop_mapping_criteria(unsigned char* state) {
  if (wb_robot_get_time() > 220.0)
    *state = SLAM;
}

int main(int argc, char **argv) {
  /* calling the turtlebot and map */
  Turtlebot3 *turtlebot = malloc(sizeof(Turtlebot3));
  Map *map = malloc(sizeof(Map));
  Coor2D *target = malloc(sizeof(Coor2D));
  
  /* initiate simulation and objects */
  wb_robot_init();
  turtlebot3_init(turtlebot);
  map_init(map);
  
  /* state */
  unsigned char slam_state = MAP;
  target->x_map = 195;
  target->y_map = 97;
  
  /* loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* get surrounding data with lidar */
    turtlebot3_get_lidar(turtlebot);
    
    /* slam algorithm */
    map_slam(map, turtlebot, slam_state);
    
    /* move the robot */
    switch(slam_state) {
      
      case MAP:
        turtlebot3_mapping_braitenberg(turtlebot);
        stop_mapping_criteria(&slam_state);
        break;
        
      case SLAM:
        turtlebot3_go_to_target(turtlebot, map, target);
        break;
    }

    /* run the turtlebot */    
    turtlebot3_run(turtlebot);
  };

  /* cleanup */
  free(turtlebot);
  free(map);
  wb_robot_cleanup();

  return 0;
}