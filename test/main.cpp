#include <iostream>

#include <string.h>

#include "LEDSync.h"

int main(int argc, char** args) {
  if (argc <= 1) {
    std::cout << "No arguments given." << std::endl;
    return -1;
  }

  LEDSync ls(std::string(args[1], args[1]+strlen(args[1])));
  ls.play(50, true, 256);
  return 0;
}
