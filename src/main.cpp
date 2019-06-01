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

using namespace rb;

void start();
void testovani_serv();
void testovani_motoru();
bool read_joystick(); // definice dole pod hlavnim programem
void ultrasounic_pinmode();
int read_ultrasounic();

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Stream* serial = nullptr; 

void setup() {
    Serial.begin (1000000); 
    Serial1.begin(1000000, SERIAL_8N1, 16, 17, false, 2000000UL ); // trida HardwareSerial (speed, config, Rx, Tx, invert, timeout )
    if (!SerialBT.begin("K2_robot")) //Bluetooth device name - na pocitaci Burda COM 9
    {                                // zjisteni portu: Ovládací panely -> zařízení a tiskárny 
        Serial.println("!!! Bluetooth initialization failed!");
        serial = &Serial;
    }
    else
    {
        serial = &SerialBT;
        SerialBT.println("!!! Bluetooth work!");
        Serial.println("!!! Bluetooth work!");
    }
    Serial.print ("Starting...\n");
    rbc().install(rb::ManagerInstallFlags::MAN_DISABLE_BATTERY_MANAGEMENT | rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE);
    auto& batt = rbc().battery();
    batt.setCoef(100.0);  
    rbc().initSmartServoBus(2, UART_NUM_2, GPIO_NUM_4); // na portu 4 a 13 nefunguje, klasické servo na portu 4 taky nefunguje 
    rbc().servoBus().limit(0, 25_deg, 130_deg); // stav 0 = 25/130
    rbc().servoBus().limit(1, 80_deg, 180_deg); // stav 1 = 180/80
    Serial.print ("RBC initialized\n");
    pinMode( 5, INPUT_PULLUP); 
    pinMode(16, INPUT_PULLUP);
    pinMode(17, OUTPUT);
    pinMode(25, INPUT_PULLUP); // POZOR, v souboru RBControl_piezo.cpp musi na r. 14 byt return; ( Piezo::Piezo() {  return; ) aby se vypnulo to, ze piezzo trvale nastavuje pin 25 jako OUTPUT 
    pinMode(26, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    pinMode(33, INPUT_PULLUP);
    ultrasounic_pinmode();

    Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
    Serial.print('\t');
    Serial.println(rbc().motor(RIGHT_MOTOR)->encoder()->value());
    rbc().leds().green(1); // indikace programu RA Beginner - pripraven
    printf("\nZKONTROLOVAL JSI BATERKU ?\n" );
    SerialBT.println("ZKONTROLOVAL JSI BATERKU ?");  
     printf("\nZKONTROLOVAL JSI STOP-TLAČÍTKO ?\n" );
    SerialBT.println("ZKONTROLOVAL JSI STOP-TLAČÍTKO ?"); 
    Serial.println("Push SW3 to Start Roadside Assistance Beginner - Vavrinec" );
    while (sw3() == 0) sleep(0); // ceka na stisk tlacitka SW3 - pokyn ke startu 
    rbc().leds().yellow(0);
    printf("Start\n" );
    SerialBT.println("Start\n" );
    printf("Enkodery\n" );
    Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
    Serial.print('\t');
    Serial.println(rbc().motor(RIGHT_MOTOR)->encoder()->value());
    rbc().servoBus().set(0, rb::Angle::deg(30), 150.f, 1.5f);
    rbc().servoBus().set(1, rb::Angle::deg(180), 150.f, 1.5f);
    delay(500);
    rbc().servoBus().set(0, rb::Angle::deg(130), 50.f, 1.5f);
    rbc().servoBus().set(1, rb::Angle::deg(80), 50.f, 1.5f);
    poloha_0 = 130;
    poloha_1 = 80;
    
}
    
timeout send_data { msec(500) }; // timeout zajistuje posilani dat do PC kazdych 500 ms

void loop() 
{
    rbc().leds().blue(SerialBT.hasClient());  // modrá LED sviti, kdyz jede bluetooth 
    rbc().leds().yellow(sw1()); // rucni zapinani zlute ledky - testovani odezvy desky 
    if (send_data) {
        send_data.ack();
        if (L_G_light) L_G_light = false; else  L_G_light = true;
        rbc().leds().green(L_G_light);
        SerialBT.println(millis());
    }
 //     testovani_serv();  // pozor, kdyz je zapnuto, rusi ostatni ovladani serv 
 //   testovani_motoru();

    if ( read_joystick() ) {
        float axis_0 = (abs(axis[0]) < 3) ? 0 : axis[0] /128.0; 
        axis_0 = axis_0*axis_0*axis_0;
        float axis_1 = (abs(axis[1]) < 3) ? 0 : axis[1] /128.0; 
        axis_1 = axis_1*axis_1*axis_1;
        int levy_m = (axis_1 - (axis_0 /2 )) * speed_coef;
        int pravy_m = (axis_1 + (axis_0 /2 )) * speed_coef;
        printf(" %i %i \n ", levy_m, pravy_m );
        rbc().setMotors().power(LEFT_MOTOR, levy_m)
                         .power(RIGHT_MOTOR, pravy_m)
                         .set();

        if (axis[5] > 100 ) // pouze zapnuto/vypnuto
        {
            rbc().servoBus().set(0, rb::Angle::deg(35), 300.f, 1.5f);
            rbc().servoBus().set(1, rb::Angle::deg(30), 300.f, 1.5f);
            poloha_0 = 35;
            poloha_1 = 175;
        }
        if (axis[5] < -100 ) // pouze zapnuto/vypnuto
        {
            rbc().servoBus().set(0, rb::Angle::deg(100), 300.f, 1.5f);
            rbc().servoBus().set(1, rb::Angle::deg(50), 300.f, 1.5f);
            poloha_0 = 100;
            poloha_1 = 110;
        }
        float rychlost = (-axis[2]+129)/128.0;
        if (btn[5] == 1 )
            rychlost_0 = rychlost;
        else if (btn[6] == 1 )
             rychlost_0 = -rychlost;
        else
            rychlost_0 =0;

        if (btn[8] == 1 )
            rychlost_1 = rychlost;
        else if (btn[7] == 1 )
             rychlost_1 = -rychlost;
        else
            rychlost_1 =0;

        Serial.print(poloha_0); Serial.print(" "); 
        Serial.print(poloha_1); Serial.print(" ");
        SerialBT.print(levy_m); SerialBT.print(" "); SerialBT.println(pravy_m);
    }  
    poloha_0 = trim(poloha_0 + rychlost_0, 25, 130);
    poloha_1 = trim(poloha_1 + rychlost_1, 80, 180);
    rbc().servoBus().set(0, rb::Angle::deg(poloha_0), 200.f, 1.5f);
    rbc().servoBus().set(1, rb::Angle::deg(poloha_1), 200.f, 1.5f);
    delay(10);
}
// ************************ definice, ktere jinde nefunguji 

void start()
{   
    vpred(1.1);
    Serial.println("vlevo");
    vlevo(1.1);
    Serial.println("vpred1");
    for (int x = 0; x != 4; ++x) {
        vpred(1);
    }
    Serial.println("vpred2");
    vpred(0.5);
    Serial.println("hotovo");
}

// ********************************************************************

// bool read_joystick()
// {
//     if ( SerialBT.available() == 0 )
//         return false;

//     int test = SerialBT.read();
//     if (test == 0x80)
//     {
//         int axis_count = SerialBT.read();
//         for (int x = 0; x < axis_count; x++)
//         {
//             while(SerialBT.available() < 1)
//             {
//                 // DO NOTHING - WAITING FOR PACKET
//                 delay(1);
//             }

//             int8_t tmp = SerialBT.read();
//             axis[x] = tmp;
//             Serial.print(x);  
//             Serial.print(": ");
//             Serial.print(axis[x], DEC);
//             Serial.print(" ");
//             SerialBT.print(x);
//             SerialBT.print(": ");
//             SerialBT.print(axis[x], DEC);
//             SerialBT.print(" ");

//         }
//         return true;
//     }
//     else if  ( test == 0x81 )
//     {
//         while(SerialBT.available() < 1) {
//             // DO NOTHING - WAITING FOR PACKET
//             delay(1);
//         }
//         byte a = SerialBT.read();
//         while(SerialBT.available() < 1) {
//             // DO NOTHING - WAITING FOR PACKET
//             delay(1);
//         }
//         btn_last[a] = btn[a];
//         btn[a] = SerialBT.read();
//         Serial.print(a, DEC); Serial.print(": "); Serial.print(btn[a], DEC); Serial.print("last: "); Serial.print(btn_last[a], DEC);
//         return true;
//     }
//     return false;
// }

bool read_joystick()
{
    if ( SerialBT.available() == 0 )
        return false;

    int test = SerialBT.read();
    if (test == 0x80)
    {   
        while(SerialBT.available() < 1)
                {
                    // DO NOTHING - WAITING FOR PACKET
                    delay(1);
                }
        int axis_count = SerialBT.read();
        Serial.print("axis_count: "); Serial.println(axis_count);
        if (axis_count >= AXIS_COUNT)
        {
            Serial.println("********* CHYBA V POCTU OS !!! ************");
        }
        else
        {
            for (int x = 0; x < axis_count; x++)
            {
                while(SerialBT.available() < 1)
                {
                    // DO NOTHING - WAITING FOR PACKET
                    delay(1);
                }

                int8_t tmp = SerialBT.read();
                axis[x] = tmp;
                Serial.print(x);  
                Serial.print(": ");
                Serial.print(axis[x], DEC);
                Serial.print(" ");
                SerialBT.print(x);
                SerialBT.print(": ");
                SerialBT.print(axis[x], DEC);
                

            }
            SerialBT.println(" ");
            return true;
        }
        
        
    }
    else if  ( test == 0x81 )
    {
        while(SerialBT.available() < 1) {
            // DO NOTHING - WAITING FOR PACKET
            delay(1);
        }
        byte a = SerialBT.read();
        if ( a >= BTN_COUNT )
        {
            Serial.println("********* CHYBA V POCTU TLACITEK !!! ************");
        }
        else 
        {
            while(SerialBT.available() < 1) {
                // DO NOTHING - WAITING FOR PACKET
                delay(1);
            }
            btn_last[a] = btn[a];
            btn[a] = SerialBT.read();
            Serial.print(a, DEC); Serial.print(": "); Serial.print(btn[a], DEC); Serial.print("last: "); Serial.print(btn_last[a], DEC);
            return true;
        }

    }
    return false;
}
// //******************************************
void testovani_serv()
{

     if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 'w':
                poloha_0++;
                Serial.print("poloha_0  ");
                Serial.println(poloha_0);
                break;
            case 's':
                poloha_0--;
                Serial.print("poloha_0  ");
                Serial.println(poloha_0);
                break;
            case 'a':
                poloha_1++;
                Serial.print("poloha_1  ");
                Serial.println(poloha_1);
                break;
            case 'd':
                poloha_1--;
                Serial.print("poloha_1  ");
                Serial.println(poloha_1);
                break;
            default:
                Serial.write(c);
        } 
    }
    rbc().servoBus().set(0,rb::Angle::deg(poloha_0),100.f,1.5f); // stav 0 = 25/130
    rbc().servoBus().set(1,rb::Angle::deg(poloha_0),180.f,1.5f); // stav 1 = 180/80
}
//*************************************************
void testovani_motoru()
{
   if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 'w':
                rbc().setMotors().power(LEFT_MOTOR, power_motor)
                                 .power(RIGHT_MOTOR, power_motor)
                                 .set();
                break;
            case 's':
                rbc().setMotors().power(LEFT_MOTOR, -power_motor)
                                 .power(RIGHT_MOTOR, -power_motor)
                                 .set();
                break;
            case 'a':
                rbc().setMotors().power(LEFT_MOTOR, -power_motor)
                                 .power(RIGHT_MOTOR, power_motor)
                                 .set();
                break;
            case 'd':
                rbc().setMotors().power(LEFT_MOTOR, power_motor)
                                 .power(RIGHT_MOTOR, -power_motor)
                                 .set();
                break;
            case '*':
                c = '0' + 10;
            case '0' ... '9':
                power_motor = (c - '0') * 10;
                Serial.println(power_motor);
                break;
                rbc().motor(LEFT_MOTOR)->drive(otacka * (c - '0'), 64, nullptr);
                rbc().motor(RIGHT_MOTOR)->drive(otacka * (c - '0'), 64, nullptr); //  tik; na otacku 
                break;

            case ' ':
                rbc().setMotors().stop(LEFT_MOTOR)
                                 .stop(RIGHT_MOTOR)
                                 .set();
                break; 

            case 'i': vpred(1);
                break;
            case 'k': vpravo(1);
                break;
            case 'm': vpred(-1);
                break;
            case 'j': vlevo(1);
                break;
            case 'p': vpravo_na_miste(1);
                break;
                
            default:
                Serial.write(c);
                break;
        } 
    }


}

void ultrasounic_pinmode()
{
    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
}
