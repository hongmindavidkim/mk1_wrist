#include <float.h>
#include <cstdint>

#define PI 3.1415926535f
#define rad2pulse(x) float(x) * (4096.0f/(2*PI)) + 2048 // 0 rad = 2048 = upright pos // Motor -> CW - , CCW + // -pi < x < pi
#define radconv(x) float(x) * (4096.0f/(2*PI))
#define PULLEY_RATIO_1 (1638.0f/1598.0f) // MPP to MPR (M4/M1)
#define PULLEY_RATIO_2 (1148.0f/1598.0f) // PIP to MPR (M4/M2)
#define PULLEY_RATIO_3 (808.0f/1598.0f)  // DIP to MPR (M4/M3)

#define MOTOR_JOINT_RATIO (1388.0f/1548.0f) // Actuator pulley to Joint Pulley 

#define PULLEY_DEPENDENCY_21 -1*(968.0f/1388.0f)
#define PULLEY_DEPENDENCY_31  1*(548.0f/1388.0f)

#define PULLEY_DEPENDENCY_32 -1*(548.0f/1388.0f)

void ActuatorTransformation(uint32_t (&transform)[4], float mcp, float pip, float dip, float mcr);
void PitchYawTransformation(uint32_t (&transform)[2], float pitch, float yaw);