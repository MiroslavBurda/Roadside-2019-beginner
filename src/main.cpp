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

using namespace rb;

bool kalibrace(); // definice dole pod hlavnim programem
void start();


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Stream* serial = nullptr;



void setup() {
    Serial.begin (1000000); 
    Serial1.begin(1000000, SERIAL_8N1, 16, 17, false, 2000000UL ); // trida HardwareSerial (speed, config, Rx, Tx, invert, timeout )
    if (!SerialBT.begin("RBControl")) //Bluetooth device name
    {
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

    rbc().install(rb::MAN_DISABLE_BATTERY_MANAGEMENT | rb::MAN_DISABLE_MOTOR_FAILSAFE);
    rbc().initSmartServoBus(2, UART_NUM_2, GPIO_NUM_14); // pin IO14 by default

    // ID je tady od 0, ale HW je od 1!
    rbc().servoBus().limit(0, 0_deg, 240_deg); // ID, minimalni, maximalni hodnota - toto se nastavuje pouze jednou

    rbc().servoBus().set(0, 120_deg, 150); // ID, cilova poloha, rychlost, [zrychleni a zpomaleni na zacatku a konci pohybu, 1.f toto vypina]


    Serial.print ("RBC initialized\n");
    auto& batt = rbc().battery();
    batt.setCoef(100.0);  // toto musí být napevno, aby si cip nemyslel, ze je nizke napeti na baterce, toto napeti se musi kontrolovat rucne  
    pinMode(5, INPUT_PULLUP); 
    pinMode(16, INPUT_PULLUP);
    pinMode(17, OUTPUT);
    pinMode(25, INPUT_PULLUP); // POZOR, v souboru RBControl_piezo.cpp musi na r. 14 byt return; ( Piezo::Piezo() {  return; ) aby se vypnulo to, ze piezzo trvale nastavuje pin 25 jako OUTPUT 
    pinMode(26, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    pinMode(33, INPUT_PULLUP);
    // rbc().schedule(200, vypis_IR);  // spusti casovac kazdych 200ms presne pomoci preruseni (ostatni veci pockaji)
    Serial.println ("Calibration begin after pushing SW2\n");
    SerialBT.println ("Calibration begin after pushing SW2\n");
    Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
    Serial.print('\t');
    Serial.println(rbc().motor(RIGHT_MOTOR)->encoder()->value());
    //while (sw2() == 0) sleep(0); // ceka na stisk tlacitka SW2 - začne kalibraci 
    //delay(1000);
    //kalibrace();
    SerialBT.println("Zkontroloval jsi IR senzory ?");
    Serial.println("Zkontroloval jsi IR senzory ?");
    printf("\nZKONTROLOVAL JSI BATERKU ?\n" );
    SerialBT.println("ZKONTROLOVAL JSI BATERKU ?");  
    Serial.println("Push SW3 to Start" );
    while (sw3() == 0) sleep(0); // ceka na stisk tlacitka SW3 - pokyn ke startu 
    rbc().leds().yellow(0);
    printf("Start\n" );
    SerialBT.println("Start\n" );
    printf("Enkodery\n" );
    Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
    Serial.print('\t');
    Serial.println(rbc().motor(RIGHT_MOTOR)->encoder()->value());
    
    start();
}
    
timeout send_data { msec(500) }; // timeout zajistuje posilani dat do PC kazdych 500 ms

void loop() 
{
    rbc().leds().yellow(sw1()); // rucni zapinani zlute ledky 
    if (send_data) {
        send_data.ack();
        if (L_G_light) L_G_light = false; else  L_G_light = true;
        rbc().leds().green(L_G_light);

        // Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
        // Serial.print('\t');
        // Serial.println(rbc().motor(RIGHT_MOTOR)->encoder()->value());

    }

    if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 't':
                if (position_servo >= 5)  position_servo = position_servo -5;               
                servo.write(position_servo);
                Serial.write(" 0: "); 
                Serial.print(position_servo);
                break;
            case 'u':
                if (position_servo <= 175)  position_servo = position_servo +5;               
                servo.write(position_servo);
                Serial.write(" 0: "); 
                Serial.print(position_servo);
                break;
            case '+':
                ++power_motor;
                Serial.println(power_motor);
                break;
            case '-':
                --power_motor;
                Serial.println(power_motor);
                break;
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

            // case 'l':
            //     rbc().motor(RIGHT_MOTOR)->drive(otacka, 45, nullptr); //  tik; na otacku 
            //     break;
            // case 'p':
            //     rbc().motor(LEFT_MOTOR)->drive(otacka, 45, nullptr);
            //     break;

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
// ************************ definice, ktere jinde nefunguji 

bool kalibrace() 
{
    Serial.println("Kalibrace...");
    byte qrd_extrem[12][2];     // pro vypocty minima a maxima kazdeho senzoru, 0 - minima, 1 - maxima
    bool test_ok =  read_qrd(); // pocatecni naplneni pole qrd_extrem[12][2]
    Serial.println("\tFirst readout");
    if (test_ok == true)
        for(byte a = 0; a<12; a++)
        {
            qrd_extrem[a][0]=qrd[a];
            qrd_extrem[a][1]=qrd[a];
        }

    end_L = false;
    end_R = false; 
    rbc().motor(LEFT_MOTOR)->drive(2*ctverec, power_motor, end_left);
    rbc().motor(RIGHT_MOTOR)->drive(2*ctverec, power_motor, end_right);
    uint32_t t = millis();
    uint32_t t_last_meas = micros();
    while (! (end_L and end_R) ) {
        test_ok = read_qrd();
        uint32_t tm = micros();
        //Serial.print(rbc().motor(LEFT_MOTOR)->encoder()->value()); // cteni enkoderu
        //Serial.print('\t');
        //Serial.print(rbc().motor(RIGHT_MOTOR)->encoder()->value());
        //Serial.print('\t');
        Serial.println(tm - t_last_meas);
        t_last_meas = tm;
        if (test_ok == true) {
            for(byte a = 0; a<12; a++) {
                if(qrd_extrem[a][0]>qrd[a])  qrd_extrem[a][0]=qrd[a];
                if(qrd_extrem[a][1]<qrd[a])  qrd_extrem[a][1]=qrd[a];
            }
        }
        if ((millis() - t) > 4000) {
            rbc().setMotors().stop(LEFT_MOTOR)
                             .stop(RIGHT_MOTOR)
                             .set();
            Serial.println("Calibration failed (timeout)");
            return false;
        }
    }
    
    for(byte b = 0; b<12; b++)
    {
        qrd_prumer[b] = (qrd_extrem[b][0]+qrd_extrem[b][1])/2;
        printf("\n# %i: Min: %i Max: %i Avg: %i", b, qrd_extrem[b][0], qrd_extrem[b][1], qrd_prumer[b]);
        SerialBT.print(b); SerialBT.print(": Min, Max, Avg: "); 
        SerialBT.print(qrd_extrem[b][0]); SerialBT.print(" "); SerialBT.print(qrd_extrem[b][1]); SerialBT.print(" "); SerialBT.print(qrd_prumer[b]); 
    }
    delay(300);
    rbc().motor(LEFT_MOTOR)->drive(-2*ctverec, power_motor, end_left);
    rbc().motor(RIGHT_MOTOR)->drive(-2*ctverec, power_motor, end_right);

    servo.attach(14); // otestovani serva 
    servo.write(servo_open); 
    delay(1000);
    servo.write(servo_close); 
    delay(200);
    
    rbc().leds().yellow(1); // rozsviti zlutou LED - pripraven ke startu 
  
    return true;
}

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

     

    // chybi zastaveni pri konci hry 

}
