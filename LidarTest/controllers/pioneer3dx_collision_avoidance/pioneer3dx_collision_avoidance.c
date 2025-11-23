/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller moving the Pioneer 3-DX and avoiding obstacles.
                 3 LEDs are switched on and off periodically
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/display.h>
// maximal speed allowed
#define MAX_SPEED 5.24

// how many sensors are on the robot
#define MAX_SENSOR_NUMBER 16

// delay used for the blinking leds
#define DELAY 70

// maximal value returned by the sensors
#define MAX_SENSOR_VALUE 1024

// minimal distance, in meters, for an obstacle to be considered
#define MIN_DISTANCE 3.0

// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 100

// structure to store the data associated to one sensor
typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

int main() {
  // necessary to initialize Webots
  wb_robot_init();

  // stores simulation time step
  int time_step = wb_robot_get_basic_time_step();

  // stores device IDs for the wheels
  WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel");

  // stores device IDs for the LEDs
  WbDeviceTag red_led[3];
  red_led[0] = wb_robot_get_device("red led 1");
  red_led[1] = wb_robot_get_device("red led 2");
  red_led[2] = wb_robot_get_device("red led 3");

  char sensor_name[5] = "";
  int i;

  // sets up sensors and stores some info about them
  for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
    sprintf(sensor_name, "so%d", i);
    sensors[i].device_tag = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i].device_tag, time_step);
  }

  // sets up wheels
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  int j, led_number = 0, delay = 0;
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;

  // by default, the robot goes forward
  State state = FORWARD;
  
  // Lidar, gps, compass en display
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, time_step);
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, time_step);
  
  WbDeviceTag display = wb_robot_get_device("display");
  
  
  // run simulation
  while (wb_robot_step(time_step) != -1) {
    const float *ranges = wb_lidar_get_range_image(lidar);
    int res = wb_lidar_get_horizontal_resolution(lidar);
    float fov = wb_lidar_get_fov(lidar);
    
    if (ranges[0] == wb_lidar_get_max_range(lidar)) {
        printf("LiDAR ziet niks. Alle beams op maxRange.\n");
    } else {
        printf("Eerste range: %f\n", ranges[0]);
    }
    
    // clear display
    wb_display_set_color(display, 0x000000);
    wb_display_fill_rectangle(display, 0, 0, 128, 128);
    
    // get gps + heading
    const double *pos = wb_gps_get_values(gps);
    const double *north = wb_compass_get_values(compass);
    double yaw = atan2(north[0], north[2]);
    
    for (int i = 0; i < res; i++) {
        float r = ranges[i];
        if (r < 10.0) {
            float angle = -fov/2.0 + (float)i * (fov / res);
            float global_angle = angle + yaw;
    
            float wx = pos[0] + r * cos(global_angle);
            float wz = pos[2] + r * sin(global_angle);
    
            int px = (int)(64 + wx * 20);
            int pz = (int)(64 - wz * 20);
    
            if (px >= 0 && px < 128 && pz >= 0 && pz < 128) {
                wb_display_set_color(display, 0xFFFFFF);
                wb_display_draw_pixel(display, px, pz);
            }
        }
    }
  
    // initialize speed and wheel_weight_total arrays at the beginning of the loop
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);

    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);

      // if the sensor doesn't see anything, we don't use it for this round
      if (sensor_value == 0.0)
        speed_modifier = 0.0;
      else {
        // computes the actual distance to the obstacle, given the value returned by the sensor
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.

        // if the obstacle is close enough, we may want to turn
        // here we compute how much this sensor will influence the direction of the robot
        if (distance < MIN_DISTANCE)
          speed_modifier = 1 - (distance / MIN_DISTANCE);
        else
          speed_modifier = 0.0;
      }

      // add the modifier for both wheels
      for (j = 0; j < 2; ++j)
        wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
    }

    // (very) simplistic state machine to handle the direction of the robot
    switch (state) {
      // when the robot is going forward, it will start turning in either direction when an obstacle is close enough
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      // when the robot has started turning, it will go on in the same direction until no more obstacle are in sight
      // this will prevent the robot from being caught in a loop going left, then right, then left, and so on.
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
    }

    // the three red LEDs are swicthed on and off periodically
    ++delay;
    if (delay == DELAY) {
      wb_led_set(red_led[led_number], 0);
      ++led_number;
      led_number = led_number % 3;
      wb_led_set(red_led[led_number], 1);
      delay = 0;
    }
    // sets the motor speeds
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
