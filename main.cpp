#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cstdint>
#include "actuator_transformation.h"

// Initialize stuff for dynamixels
#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))

bool state_change = false;
bool servo_on = false;

RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

uint8_t dxl_ID[] =  {1,2,3,4};
int8_t idLength = sizeof(dxl_ID) / sizeof(uint8_t);

// Debugging LEDs
DigitalOut led_pwr(LED1);
DigitalOut led_mot(LED2);
DigitalOut led_com(LED3);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 921600);
Timer t;
int loop_time;

// Initial Positions

uint32_t multiHomePos[] = {rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0)};
uint16_t multiGoalCurrent[] = {1193,1193,1193,1193};

uint32_t* tempPos1 = ActuatorTransformation(PI/4, -PI/4, -PI/4, PI/4);
uint32_t* tempPos2 = ActuatorTransformation(PI/4, PI/4, PI/4, -PI/4);

// ToF sensor i2c busses
/**
I2C i2c1(PF_0, PF_1); //(PB_9, PB_8); // SDA, SCL
// initialize sensors
VL6180X tof1; // right finger inner

int range[1];
float range_m[1]; // range in m
int range_status[1];
uint16_t range_period = 30;
**/

// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    
    /**
    // ToF Setup
    i2c1.frequency(400000);
    pc.printf("Sensor 1...\n\r");
    wait_us(100);
    if(!tof1.begin(&i2c1)){
        pc.printf("Sensor 1 init failed.\n\r");
    }
    wait_us(100);
    tof1.stopRangeContinuous();
    wait_us(100);
    tof1.startRangeContinuous(range_period);
    wait_us(1000);
    **/

    // Set up dynamixel
    wait_ms(300);
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RTS.mode(OpenDrainNoPull);
    RTS.output();
    uart.baud(1000000);
    RTS = 0;
    wait_ms(500);
    
    XM430_bus dxl_bus(1000000, D1, D0, D2); // baud, tx, rx, rts

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_ID[i],0x32); 
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);    
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); 
        wait_ms(100);
    }
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 80); // 17166 rev/min^2
    }
    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos); 
    wait_ms(2000);
    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    while (true) {
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos1);
        pc.printf("%d, %d, %d, %d\n\r", tempPos1[0], tempPos1[1], tempPos1[2], tempPos1[3]);
        wait_ms(500);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos2);
        pc.printf("%d, %d, %d, %d\n\r", tempPos2[0], tempPos2[1], tempPos2[2], tempPos2[3]);
        wait_ms(500);
        /**
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        **/
    }

}
