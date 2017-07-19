#include <AFMotor.h> //Libraria Adafruit Motor Shield 
#include <QTRSensors.h> //Libraria Pololu QTR Sensor
AF_DCMotor motor1(1, MOTOR12_1KHZ ); //creaza motorul 1 folosind iesirea M1 din Motor Drive Shield, setat pe frecventa de 1kHz PWM
AF_DCMotor motor2(2, MOTOR12_1KHZ ); //creaza motorul 2 folosind iesirea M2 din Motor Drive Shield, setat pe frecventa de 1kHz PWM
#define KP 2 
#define KD 5 
#define M1_viteza_minima 150 //viteza min a motorului 1
#define M2_viteza_minima 150 //viteza min a motorului 2
#define M1_viteza_maxima 250 //viteza max a motorului 1
#define M2_viteza_maxima 250 //viteza max a motorului 2
#define senzor_mijloc 4 //numarul senzorului din mijloc
#define numar_senzori 5 //numarul de senzori folositi
#define TIMEOUT 2500 //asteapta 2500 ms pana iesirile senzorilor se opresc
#define EMITTER_PIN 2 //emitter_pin controleaza daca ledurile sunt on sau off
#define DEBUG 0
//senzorii de la 0 la 5 sunt conectati la intrarile analog de la 0 la 5 
QTRSensorsRC qtrrc((unsigned char[]) { A4,A3,A2,A1,A0} ,numar_senzori, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[numar_senzori];
void setup()
{
delay(1500);
manual_calibration();
set_motors(0,0);
}
int lastError = 0;
int last_proportional = 0;
int integral = 0;
void loop()
{
unsigned int sensors[5];
int position = qtrrc.readLine(sensors); 
int error = position - 2000;
int motorSpeed = KP * error + KD * (error - lastError);
lastError = error;
int leftMotorSpeed = M1_viteza_minima + motorSpeed;
int rightMotorSpeed = M2_viteza_minima - motorSpeed;

set_motors(leftMotorSpeed, rightMotorSpeed);
}
void set_motors(int motor1speed, int motor2speed)
{
if (motor1speed > M1_viteza_maxima ) motor1speed = M1_viteza_maxima;
if (motor2speed > M2_viteza_maxima ) motor2speed = M2_viteza_maxima;
if (motor1speed < 0) motor1speed = 0; 
if (motor2speed < 0) motor2speed = 0; 
motor1.setSpeed(motor1speed); 
motor2.setSpeed(motor2speed);
motor1.run(FORWARD); 
motor2.run(FORWARD);
}
void manual_calibration() {
int i;
for (i = 0; i < 250; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
if (DEBUG) {
Serial.begin(9600);
for (int i = 0; i < numar_senzori; i++)
{
Serial.print(qtrrc.calibratedMinimumOn[i]);
Serial.print(' ');
}
Serial.println();
for (int i = 0; i < numar_senzori; i++)
{
Serial.print(qtrrc.calibratedMaximumOn[i]);
Serial.print(' ');
}
Serial.println();
Serial.println();
}
}

