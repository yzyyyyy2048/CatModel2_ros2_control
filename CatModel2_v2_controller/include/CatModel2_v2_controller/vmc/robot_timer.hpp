#ifndef ROBOT_TIMER_HPP  
#define ROBOT_TIMER_HPP  
  
#include <chrono>  
#include <ctime>  
#include <iomanip>  
#include <sstream>  
  
class Timer {  
public:  
    Timer() : start_(std::chrono::steady_clock::now()) {}  
  
    void start() {  
        start_ = std::chrono::steady_clock::now();  
    }  
  
    double get() const {  
        auto now = std::chrono::steady_clock::now();  
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);  
        return static_cast<double>(duration.count()) / 1000.0; // convert to milliseconds  
    }  
  
private:  
    std::chrono::steady_clock::time_point start_;  
};  
  
#endif // ROBOT_TIMER_HPP