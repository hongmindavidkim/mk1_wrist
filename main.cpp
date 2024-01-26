#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cmath>
#include <cstdint>
#include "actuator_transformation.h"

#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)

bool state_change = false;
bool servo_on = false;

RawSerial uart(PF_7, PF_6);
DigitalInOut RTS(PF_9);

uint8_t dxl_ID[] =  {1, 2};
uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);

// Debugging LEDs
DigitalOut led_pwr(LED1);
DigitalOut led_mot(LED2);
DigitalOut led_com(LED3);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 921600);
Timer t;
int loop_time;
int servo_time;
double dt = 0.002; //2ms
uint32_t time_step = 0;
Ticker motor_cmd;
int8_t motor_cmd_flag = 0;
int32_t dxl_time;
// Initial Positions

uint32_t multiHomePos[2] = {2048, 2048};
uint32_t goalPos1[2] = {2048, 2048};
uint32_t goalPos2[2] = {2048, 2048};
uint32_t goalPos3[2] = {2048, 2048};
uint32_t goalPos4[2] = {2048, 2048};
uint32_t goalPos5[2] = {2048, 2048};
uint32_t goalPos6[2] = {2048, 2048};
uint32_t goalPos7[2] = {2048, 2048};
uint32_t goalPos8[2] = {2048, 2048};

uint32_t rtPos[2];

float pulse_to_rad = (2.0f*PI)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*PI)/60.0f; // = 0.0239

double current_limit = 1000;

int32_t dxl_position[2];
int32_t dxl_velocity[2];
int16_t dxl_current[2];
double desired_current[2];

uint16_t current_command[2];

XM430_bus dxl_bus(2000000, PF_7, PF_6, PF_9); // baud, tx, rx, rts
float freq1 = 0.75; //9.5
float freq2 = freq1/2.0f;
float amp1 = (PI/180)*65;
float amp2 = (PI/180)*35;
float amp3 = (PI/180)*15;



void motorCommand() {
    if (time_step < 3*(1/freq1)*(1/dt)){
        motor_cmd_flag = 1;


PitchYawTransformation(rtPos, (amp1)*(cosf(freq1*2*PI*(float)(dt*time_step))), (amp1)*(sinf(freq1*2*PI*(float)(dt*time_step))));

time_step++;
    }

    else {
        wait_ms(200);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
    }
}

void MoveAndReset(std::uint32_t positions[]) {
    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, positions);
    wait_ms(350);
    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
    wait_ms(350);
}

int main() {

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);
        dxl_bus.SetRetDelTime(dxl_ID[i],0x05); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_ms(100);
        }
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 0); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 0); // 80(17166) rev/min^2
        }

    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
  
    wait_ms(2000);

    PitchYawTransformation(goalPos1, deg2rad(0), deg2rad(65));
    PitchYawTransformation(goalPos2, deg2rad(0), deg2rad(-65));
    PitchYawTransformation(goalPos3, deg2rad(65), deg2rad(0));
    PitchYawTransformation(goalPos4, deg2rad(-65), deg2rad(0));
    PitchYawTransformation(goalPos5, deg2rad(65), deg2rad(65));
    PitchYawTransformation(goalPos6, deg2rad(-65), deg2rad(65));
    PitchYawTransformation(goalPos7, deg2rad(65), deg2rad(-65));
    PitchYawTransformation(goalPos8, deg2rad(-65), deg2rad(-65));

    MoveAndReset(goalPos1);
    MoveAndReset(goalPos2);
    MoveAndReset(goalPos3);
    MoveAndReset(goalPos4);
    MoveAndReset(goalPos5);
    MoveAndReset(goalPos6);
    MoveAndReset(goalPos7);
    MoveAndReset(goalPos8);
    

    motor_cmd.attach_us(&motorCommand,2000);
    t.reset();
    t.start();
    while (true) {

     if (motor_cmd_flag){
            motor_cmd_flag = 0;
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, rtPos);
        dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
        pc.printf("%d, %d\n\r",dxl_position[0],dxl_position[1]);
        }
    }
    t.stop();
    dxl_time = t.read_ms();


}

