#include <ctime>
#include <sstream>
#include <iomanip>
#include <fstream>
#include "config.hpp"


std::string get_timestamp(){
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::ostringstream oss;
    oss << "["
        << 1900 + ltm->tm_year 
        << "-"
        << std::setfill('0') << std::setw(2) << 1 + ltm->tm_mon 
        << "-"
        << std::setfill('0') << std::setw(2) << ltm->tm_mday 
        << " "  
        << std::setfill('0') << std::setw(2) << ltm->tm_hour 
        << ":"
        << std::setfill('0') << std::setw(2) << ltm->tm_min
        << ":"
        << std::setfill('0') << std::setw(2) << ltm->tm_sec
        << "]";
    std::string timestamp = oss.str();   
    return timestamp;
}
