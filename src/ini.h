#include <atomic>

using rb::LED_GREEN;

#define RIGHT_MOTOR  rb::MotorId::M8
#define LEFT_MOTOR  rb::MotorId::M1


byte qrd[12];  // pole, do ktereho se nacitaji okamzite hodnoty QRD senzoru 
byte qrd_prumer[12];
std::atomic_bool end_L;
std::atomic_bool end_R;

float poloha_0 = 80;
float poloha_1 = 120;
float rychlost_0 = 0;
float rychlost_1 = 0;
int axis[7] = {5,6,7,8,9,10,11};
byte btn[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
byte btn_last[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
int speed_coef = -100; // nasobeni hodnoty, co leze z joysticku
int diff_coef = 1; // o kolik jede jeden motor rychleji nez druhy 

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

rb::Manager& rbc()
{
    static rb::Manager m(false,false);  // ve výchozím stavu se motory po puštění tlačítka vypínají, false zařídí, že pojedou, dokud nedostanou další pokyn
    return m;
}

bool sw1() { return !rbc().expander().digitalRead(rb::SW1); }
bool sw2() { return !rbc().expander().digitalRead(rb::SW2); }
bool sw3() { return !rbc().expander().digitalRead(rb::SW3); }

float trim(float value, float min, float max)
{
    if (value < min)
        return min; 
    if (value > max)
        return max; 
    return value;
}

