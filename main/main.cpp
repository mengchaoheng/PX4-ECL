#include <stdio.h>
#include <string.h>
#include "PostEkf.h"

int main(int argc, char** argv)
{
   constexpr unsigned int MAX_PATH = 260u;
   char file_name[MAX_PATH] = { 0 };
   if (argc <= 1) {
      printf("please input the px4 csv log file.\n");
      fgets(file_name, MAX_PATH, stdin);
   }
   else {
      strcpy(file_name, argv[1]);
   }
   printf("start ecl main...\n");

   PostEkf post(file_name);
   post.update();

   printf("finished.\n");
    
   return 0;
}