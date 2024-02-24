#include <HCSR04.h>
#include <VarSpeedServo.h>
#include <PID_v1.h>

const int servoPin = 9; // Pino do Servo

float Kp = 5.5; // Ganho Proporcional Inicial
float Ki = 1.84;   // Ganho Integral Inicial
float Kd = 2.75;   // Ganho Derivativo Inicial
double SetPoint, Input, Output, ServoOutput;

UltraSonicDistanceSensor distanceSensor(13, 12);
PID myPID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);
VarSpeedServo myServo;

const int numReadings = 10; // Número de leituras a serem suavizadas
float readings[numReadings];   // Array para armazenar as leituras
int index = 0;               // Índice atual do array
float total = 0;             // Soma total das leituras

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(20, 160);

  // Inicializa todas as leituras como 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  SetPoint = 25;
  Input = readPosition();
  myPID.Compute();

  ServoOutput = 85 + Output;
  myServo.write(ServoOutput, 100, false);
}

float readPosition() {
  delay(5);

  long cm = distanceSensor.measureDistanceCm();

  // Aplicar filtro passa-baixa
  total = total - readings[index] + cm;
  readings[index] = cm;
  index = (index + 1) % numReadings;
  cm = total / numReadings;
  Serial.println(cm);
  if (cm > 40) { // 30 cm é a posição máxima para a bola
    cm = 40;
  }

  if (cm < 10) {
    cm = 10;
  }

  

  return cm;
}
