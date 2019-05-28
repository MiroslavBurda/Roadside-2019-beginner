// GY-652 - magnetometr lepsi,   dalsi je GY 531

#include <Arduino.h>
#include "RBControl_manager.hpp"
#include <Servo.h>
#include <Wire.h>
#include "time.hpp"
#include <stdint.h>
#include "stopwatch.hpp"
#include "nvs_flash.h"
#include "BluetoothSerial.h"

#include "ini.h"
#include "function.h"
bool kalibrace(); // definice dole pod hlavnim programem
void test_startu();


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
int stav_0 = 0;
int stav_1 = 0;

void setup() 
{
    Serial.begin (115200);
    rbc().initSmartServoBus(2, UART_NUM_1, GPIO_NUM_32);
    rbc().servoBus().limit(0, Angle::deg(25), Angle::deg(130)); // stav 0 = 25/130
    rbc().servoBus().limit(1, Angle::deg(80), Angle::deg(180)); // stav 0 = 25/130
}

void loop() 
{
    if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 'w':
                stav_0++;
                Serial.print("stav_0  ");
                Serial.println(stav_0);
                break;
            case 's':
                stav_0--;
                Serial.print("stav_0  ");
                Serial.println(stav_0);
                break;
            case 'a':
                stav_1++;
                Serial.print("stav_1  ");
                Serial.println(stav_1);
                break;
            case 'd':
                stav_1--;
                Serial.print("stav_1  ");
                Serial.println(stav_1);
                break;
            default:
                Serial.write(c);
        } 
    }
    rbc().servoBus().set(0,stav_0,180.f,1.5f); // stav 0 = 25/130
    rbc().servoBus().set(1,stav_1,180.f,1.5f); // stav 1 = 180/80
}
