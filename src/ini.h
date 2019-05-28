#include <atomic>

using rb::LED_GREEN;

#define RIGHT_MOTOR  rb::MotorId::M8
#define LEFT_MOTOR  rb::MotorId::M1


byte qrd[12];  // pole, do ktereho se nacitaji okamzite hodnoty QRD senzoru 
byte qrd_prumer[12];
std::atomic_bool end_L;
std::atomic_bool end_R;

Servo servo; 

int stav_0 = 80;
int stav_1 = 120;
int axis[7] = {5,6,7,8,9,10,11};
byte btn[8] = {0,0,0,0,0,0,0,0};
byte btn_last[8] = {0,0,0,0,0,0,0,0};
int speed_coef = -100; // nasobeni hodnoty, co leze z joysticku
int diff_coef = 1; // o kolik jede jeden motor rychleji nez druhy 


int servo_open = 100;
int servo_close = 180;
int position_servo = 100; // pro postupne krokovani serva pro kalibraci 
int power_motor = 50;
int otacka = 235; // pocet tiku na otacku - nevim jiste 
int ctverec = 482; // pocet tiku na ctverec - Praha
int zatoc = 945;  // pocet tiku na zatoceni o 90 stupnu
static const uint32_t i2c_freq = 400000;
bool L_G_light = false; // pro blikani zelene LED - indikuje, ze deska funguje 

struct Driven {
    int left_enc;
    int right_enc;
    bool ok;
};


// rb::Manager& rbc() 
// {
//    return rb::Manager::get();
// }

rb::Manager& rbc()
{
    static rb::Manager m(false,false);  // ve výchozím stavu se motory po puštění tlačítka vypínají, false zařídí, že pojedou, dokud nedostanou další pokyn
    return m;
}

bool sw1() { return !rbc().expander().digitalRead(rb::SW1); }
bool sw2() { return !rbc().expander().digitalRead(rb::SW2); }
bool sw3() { return !rbc().expander().digitalRead(rb::SW3); }

bool vypis_IR() {        // funkce pouzita do preruseni musi byt bool, musi vracet true, muze jich byt 10 a vice
    // vypis hodnot na IR zleva do prava 
    Serial.print("** ");
    Serial.print(digitalRead(33));
    Serial.print(" ");
    Serial.print(digitalRead(5));
    Serial.print(" ");
    Serial.print(digitalRead(27));
    Serial.print(" ");
    Serial.print(digitalRead(26));
    Serial.print(" ");
    Serial.print(digitalRead(25));
    Serial.print( !( digitalRead(33) and digitalRead(5) and digitalRead(27) and digitalRead(26) and digitalRead(25) ) ? " stop " : " ok " );
    Serial.println(" **");
    return true;
}
