#include <stdio.h>
#include <string.h>
#include "PostEkf.h"

int main(int argc, char** argv)
{
   constexpr unsigned int MAX_PATH = 260u;
   char file_name[MAX_PATH] = { "../csv_data/log001_sensor_combined_0.csv" };
   char mag_name[MAX_PATH] = { "../csv_data/log001_vehicle_magnetometer_0.csv" };
   char baro_name[MAX_PATH] = { "../csv_data/log001_vehicle_air_data_0.csv" };
   char gps_name[MAX_PATH] = { "../csv_data/log001_vehicle_gps_position_0.csv" };
   char status_name[MAX_PATH] = { "../csv_data/log001_vehicle_status_0.csv" };
   // if (argc <= 1) {
      // printf("please input the px4 csv log file.\n");
      // fgets(file_name, MAX_PATH, stdin);
   // }
   // else {
      // strcpy(file_name, argv[1]);
   // }
   printf("start ecl main...\n");
   printf("if can't see the sensors data uotput, makesure the csv file in the path `csv_data`\n");
   PostEkf post(file_name, mag_name, baro_name, gps_name,status_name);
   post.update();

   printf("finished.\n");
    
   return 0;
}