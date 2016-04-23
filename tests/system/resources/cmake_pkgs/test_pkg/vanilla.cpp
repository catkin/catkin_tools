#include <string>
#include <cstdlib>
#include <iostream>

#include <vanilla.h>

int main(int argc, char **argv) {
  vanilla();
  std::string llp = "UNDEFINED";
  std::string dlp = "UNDEFINED";

  char* llp_c = std::getenv("LD_LIBRARY_PATH");
  char* dlp_c = std::getenv("DYLD_LIBRARY_PATH");

  if(llp_c) {
    llp = llp_c;
  }
  if(dlp_c) {
    dlp = dlp_c;
  }

  std::cout<<"LD_LIBRARY_PATH: "<<llp<<std::endl;
  std::cout<<"DYLD_LIBRARY_PATH: "<<dlp<<std::endl;

  return 0;
}
