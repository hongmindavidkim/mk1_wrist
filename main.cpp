#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cmath>
#include <cstdint>
#include "actuator_transformation.h"

#define REST_MODE 0
#define RAPID_FIRE_MODE 1
#define SMOOTH_MODE 2
#define TEACHING_MODE 3
#define IMPEDANCE_CONTROL_MODE 4
#define SERVO_ON 5
#define SERVO_OFF 6
#define SWITCH_TO_CURRENT_CONTROL 7
uint8_t state = 0; // Operating State

// Initialize stuff for dynamixels
#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)

bool state_change = false;
bool servo_on = false;

RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];


uint8_t dxl_ID[] =  {1, 2, 3, 4};
uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);

uint8_t dxl_ID_f2[] =  {5, 6, 7, 8};
uint8_t idLength_f2 = sizeof(dxl_ID_f2) / sizeof(dxl_ID_f2[0]);


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
Ticker freq_change;
int8_t motor_cmd_flag = 0;
int32_t dxl_time;
// Initial Positions

uint32_t multiHomePos[4];
uint32_t mouseGoalPos1[] = {2640, 2060, 1752, 2050};
uint32_t mouseGoalPos2[] = {2245, 2146, 1690, 1972};
uint32_t mouseGoalPos3[] = {2383, 2073, 1806, 2090};
uint32_t mouseGoalPos4[] = {2425, 2198, 1790, 2154};

uint32_t wheelPosStart[] = {2251, 2073, 1887, 2072};
uint32_t wheelPosPull[] = {2459, 2260, 1886, 2086};
uint32_t wheelPosPush[] = {2289, 2120, 1861, 2079};
uint32_t rtPos_f1[4];
uint16_t multiGoalCurrent[4];



//uint32_t* tempPos1 = ActuatorTransformation(PI/4, -PI/4, -PI/4, PI/4);
//uint32_t* tempPos2 = ActuatorTransformation(PI/4, PI/4, PI/4, -PI/4);

uint32_t goalPos[4];
float currentPos;
float currentVel;
float currentCur;

// float Kp = 0.3f;
// float Kd = 0.01f;
// float Kj = 0.0f;
//double Kp[4] = {0.063, 0.063, 0.063, 0.063};
//double Kd[4] = {0.079, 0.079, 0.079, 0.079};

double Kp[4] = {0.2, 0.15, 0.1, 0.3};
double Kd[4] = {0.02, 0.02, 0.02, 0.02};

float Kt = 2.0f * (3.7f / 2.7f);
float pulse_to_rad = (2.0f*PI)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*PI)/60.0f; // = 0.0239
//float desired_current;

double current_limit = 1000;

int32_t dxl_position[4];
int32_t dxl_velocity[4];
int16_t dxl_current[4];
double desired_current[4];

uint16_t current_command[4];
// uint32_t tempPos1[4];
// int32_t tempCPos1[4];
// uint32_t tempPos2[4];
// int32_t tempCPos2[4];

// uint32_t* goalPos1 = ActuatorTransformation(0, 0, 0, PI/4);
// uint32_t* goalPos2 = ActuatorTransformation(0, 0, 0, -PI/4);
// uint32_t* goalPos3 = ActuatorTransformation(0, 0, PI/4, 0);
// uint32_t* goalPos4 = ActuatorTransformation(0, 0, -PI/4, 0);
// uint32_t* goalPos5 = ActuatorTransformation(0, PI/4, 0, 0);
// uint32_t* goalPos6 = ActuatorTransformation(0, -PI/4, 0, 0);
// uint32_t* goalPos7 = ActuatorTransformation(PI/4, 0, 0, 0);
// uint32_t* goalPos8 = ActuatorTransformation(-PI/4, 0, 0, 0);

void display_options(void) {
    pc.printf("\n\r\n\r\n\r");
    pc.printf("4DOF FINGER CONTROL v0.1 by HDK\n\r\n\r");
    pc.printf(" Commands:\n\r");
    wait_us(10);
    pc.printf(" p - Power On / Torque Enable\n\r");
    wait_us(10);
    pc.printf(" h - Home Position\n\r");
    wait_us(10);
    pc.printf(" r - Rapid Fire\n\r");
    wait_us(10);
    pc.printf(" s - Smooth Motion\n\r");
    wait_us(10);
    pc.printf(" i - Impedance Control\n\r");
    wait_us(10);
    pc.printf(" t - Teaching Mode\n\r");
    wait_us(10);
    pc.printf(" c - Switch to Current Control\n\r");
    wait_us(10);
    pc.printf(" space - Abort / Torque Disable\n\r");
    wait_us(10);
}

void serial_interrupt(void){
    while(pc.readable()) {
        char input = pc.getc();
        if(input == 32) { //esc
            state = SERVO_OFF;
        }
        else if(input == 104) { //h
            if (state != REST_MODE) state_change = !state_change;
            else state_change = false;
            state = REST_MODE;
        }
        else if(input == 114) { //r
            if (state != RAPID_FIRE_MODE) state_change = !state_change;
            else state_change = false;
            state = RAPID_FIRE_MODE;
        }
        else if(input == 115) { //s
            state = SMOOTH_MODE;
            if (state != SMOOTH_MODE) state_change = !state_change;
            else state_change = false;
        }
        else if(input == 116) { //t
            state = TEACHING_MODE;
            if (state != TEACHING_MODE) state_change = !state_change;
            else state_change = false;
        }
        else if(input == 105) { //i
            if (state != IMPEDANCE_CONTROL_MODE) state_change = !state_change;
            else state_change = false;
            state = IMPEDANCE_CONTROL_MODE;
        }
        else if(input == 112) { //p
            if (state != SERVO_ON) state_change = !state_change;
            else state_change = false;
            state = SERVO_ON;
        }
        else if(input == 99) { //c
            if (state != SERVO_ON) state_change = !state_change;
            else state_change = false;
            state = SWITCH_TO_CURRENT_CONTROL;
        }

    }
}
XM430_bus dxl_bus(4000000, D1, D0, D2); // baud, tx, rx, rts
float freq1 = 1; //9.5
float freq2 = freq1/2.0f;
float amp1 = (PI/180)*15;
float amp2 = (PI/180)*15;
float freq[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};
uint8_t idx_freq = 0;
float c_freq_1 = 0;
float c_freq_2 = 0;;
uint8_t peak_time = 5;
float max_freq = 5;

void motorCommand() {
motor_cmd_flag = 1;
// if (dt*time_step < peak_time) {
//     c_freq_1 = 0.5*(dt*time_step);
//     c_freq_2 = 0.5*c_freq_1;
// }
// else if (dt*time_step >= 2*peak_time) {
//     c_freq_1 = 0;
//     c_freq_2 = 0;
// }
// else {
//     c_freq_1 = float(peak_time) - 0.5*((dt*time_step)-peak_time);
//     c_freq_2 = 0.5*c_freq_1;
// }

// c_freq_1 = 0.5*max_freq*(-cosf(PI*(1/(max_freq*2.0f))*dt*time_step)+1.0f);
// c_freq_2 = c_freq_1 * 0.5f;

// ActuatorTransformation(rtPos, 0.5*(PI/6)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f),  0.5*(PI/6)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(PI/6)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(PI/6)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f));
ActuatorTransformation(rtPos_f1, 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), (amp2)*(sinf(freq2*PI*(float)(dt*time_step))));
// ActuatorTransformation(rtPos_f1, 0.5*(amp1)*(-1*cosf(c_freq_1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(c_freq_1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(c_freq_1*PI*(float)(dt*time_step))+1.0f), (amp2)*(sinf(c_freq_2*PI*(float)(dt*time_step))));
// ActuatorTransformation(rtPos, 0, 0, 0, 0);
time_step++;
}

void cycleFreq() {
    
    *(&freq1) = freq[idx_freq];
    *(&freq2) = freq1/2.0f;
    if (idx_freq < (sizeof(freq)/sizeof(freq[0])-1)) {
        idx_freq++;
    }
    else idx_freq = 0;
}

uint32_t multiGoalPos1[4], multiGoalPos2[4];
// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    //pc.attach(&serial_interrupt);
    ActuatorTransformation(multiHomePos, 0, 0, 0, 0);
    ActuatorTransformation(goalPos, 0, 0, 0, 0);
    ActuatorTransformation(multiGoalPos1, deg2rad(10), deg2rad(10), deg2rad(10), deg2rad(10));
    ActuatorTransformation(multiGoalPos2, deg2rad(10), deg2rad(10), deg2rad(10), deg2rad(-10));
    // ActuatorTransformation(tempPos1, 0, 0, 0, PI/4);
    // ActuatorTransformation(tempPos2, 0, 0, 0, -PI/4);
    

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
        dxl_bus.SetVelocityProfile(dxl_ID[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 100); // 80(17166) rev/min^2
        }

    for (int i=0; i<idLength_f2; i++) {
        dxl_bus.SetTorqueEn(dxl_ID_f2[i],0x00);
        dxl_bus.SetRetDelTime(dxl_ID_f2[i],0x05); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID_f2[i], POSITION_CONTROL);
        wait_ms(100);
        dxl_bus.TurnOnLED(dxl_ID_f2[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID_f2[i],0x01); //to be able to move 
        wait_ms(100);
        }
    for (int i=0; i<idLength_f2; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID_f2[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID_f2[i], 100); // 80(17166) rev/min^2
        }

    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
    dxl_bus.SetMultGoalPositions(dxl_ID_f2, idLength_f2, multiHomePos);
    wait_ms(1000);

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 0); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 0); // 80(17166) rev/min^2
        }

    for (int i=0; i<idLength_f2; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID_f2[i], 0); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID_f2[i], 0); // 80(17166) rev/min^2
        }
        
    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    motor_cmd.attach_us(&motorCommand,2000);
    freq_change.attach_us(&cycleFreq,4000000);
    
    
    while (true) {
    t.reset();
    t.start();
        if (motor_cmd_flag){
            motor_cmd_flag = 0;
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, rtPos_f1);
        dxl_bus.SetMultGoalPositions(dxl_ID_f2, idLength_f2, rtPos_f1);
        // dxl_bus.GetMultVelocities(dxl_velocity, dxl_ID, idLength);
        dxl_bus.GetMultCurrents(dxl_current, dxl_ID, idLength);
        // dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
        // dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
        }
        
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, rtPos);
  
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, mouseGoalPos1);
        // dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
        // pc.printf("MCP: %d, MCR: %d, PIP: %d, DIP: %d\n\r", dxl_position[3], dxl_position[0], dxl_position[1], dxl_position[2]);
        // wait_ms(800);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, mouseGoalPos2);
        // wait_ms(1000);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, mouseGoalPos3);
        // wait_ms(200);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, mouseGoalPos4);
        // wait_ms(800);

        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos1);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos1[0], goalPos1[1], goalPos1[2], goalPos1[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos2);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos2[0], goalPos2[1], goalPos2[2], goalPos2[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos3);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos3[0], goalPos3[1], goalPos3[2], goalPos3[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos4);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos4[0], goalPos4[1], goalPos4[2], goalPos4[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos5);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos5[0], goalPos5[1], goalPos5[2], goalPos5[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos6);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos6[0], goalPos6[1], goalPos6[2], goalPos6[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos7);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos7[0], goalPos7[1], goalPos7[2], goalPos7[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, goalPos8);
        // pc.printf("%d, %d, %d, %d\n\r", goalPos8[0], goalPos8[1], goalPos8[2], goalPos8[3]);
        // wait_ms(500);
        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos1);
        // pc.printf("GOAL:    %d, %d, %d, %d\n\r", tempPos1[0], tempPos1[1], tempPos1[2], tempPos1[3]);
        // wait_ms(1000);
        // dxl_bus.GetMultPositions(tempCPos1, dxl_ID, idLength);
        // pc.printf("REACHED: %d, %d, %d, %d\n\n\r", tempCPos1[0], tempCPos1[1], tempCPos1[2], tempCPos1[3]);

        // dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos2);
        // pc.printf("GOAL:    %d, %d, %d, %d\n\r", tempPos2[0], tempPos2[1], tempPos2[2], tempPos2[3]);
        // wait_ms(1000);
        // dxl_bus.GetMultPositions(tempCPos2, dxl_ID, idLength);
        // pc.printf("REACHED: %d, %d, %d, %d\n\n\r", tempCPos2[0], tempCPos2[1], tempCPos2[2], tempCPos2[3]);
        
        // dxl_position = dxl_bus.GetPosition(dxl_ID[0]);
        // dxl_velocity = dxl_bus.GetVelocity(dxl_ID[0]);
        // dxl_current = dxl_bus.GetCurrent(dxl_ID[0]);
        // t.reset();
        // t.start();
        // dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
        // dxl_bus.GetMultVelocities(dxl_velocity, dxl_ID, idLength);
        // dxl_bus.GetMultCurrents(dxl_current, dxl_ID, idLength);

        //pc.printf("dxl velocity Term: %d\n\r",dxl_velocity);
        // convert states
        // currentPos = pulse_to_rad*(float)dxl_position;
        // currentVel = rpm_to_rads*(float)dxl_velocity;
        // currentCur = 0.001f*(float)dxl_current;

        // currentPos = dxl_position;
        // currentVel = dxl_velocity;
        // currentCur = dxl_current;
            
        // for(int i = 0;i<idLength;i++){
        // desired_current[i] = Kp[i]*((double)goalPos[i]-(double)dxl_position[i]) + Kd[i]*(0.0f-(int32_t)dxl_velocity[i]);
        // desired_current[i] = fmaxf(fminf(desired_current[i],current_limit),-current_limit);
        // current_command[i] = (int16_t)desired_current[i];
        // }

        //current_command = (int16_t)(desired_current*1000.0f);
        
        //pc.printf("Kp Term: %f\n\r",Kp*(goalPosC-dxl_position));
        // pc.printf("Kd Term: %f\n\r",Kd*(0.0f-(double)dxl_velocity));
        // pc.printf("Desired   Current: %f\n\r",desired_current);
        // pc.printf("Commanded Current: %d\n\r",current_command[3]);
        // pc.printf("Current Position: %d\n\n\r",dxl_position);
        // dxl_bus.SetMultGoalCurrents(dxl_ID, idLength, current_command);
        // servo_time = t.read_us();
        //pc.printf("%d\n\r",servo_time);
        dxl_time = t.read_us();
        // pc.printf("loop time: %d\n\r", dxl_time);
        // pc.printf("Velocity M1: %d, Velocity M2: %d, Velocity M3: %d, Velocity M4: %d\n\r", dxl_velocity[0], dxl_velocity[1], dxl_velocity[2], dxl_velocity[3]);
        pc.printf("Current M1: %d, Current M2: %d, Current M3: %d, Current M4: %d\n\r", dxl_current[0], dxl_current[1], dxl_current[2], dxl_current[3]);
    }

}

