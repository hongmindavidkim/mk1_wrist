#include <float.h>
#include <cstdint>

#define PI 3.1415926535
#define rad2pulse(x) float(x) * (4096.0f/(2*PI)) + 2048 // 0 rad = 2048 = upright pos // Motor -> CW - , CCW + // -pi < x < pi
#define radconv(x) float(x) * (4096.0f/(2*PI))
#define PULLEY_RATIO_1 (1614.0f/1574.0f) // MPP to MPR (M4/M1)
#define PULLEY_RATIO_2 (1124.0f/1574.0f) // PIP to MPR (M4/M2)
#define PULLEY_RATIO_3 (784.0f/1574.0f)  // DIP to MPR (M4/M3)

#define MOTOR_JOINT_RATIO (1364.0f/1524.0f) // Actuator pulley to Joint Pulley 

#define PULLEY_DEPENDENCY_21 -1*(944.0f/1364.0f)
#define PULLEY_DEPENDENCY_31  1*(524.0f/1364.0f)

#define PULLEY_DEPENDENCY_32 -1*(524.0f/1364.0f)

uint32_t* ActuatorTransformation(float mcp, float pip, float dip, float mcr);