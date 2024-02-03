// PROJETO DA DISCIPLINA SERVOMECANISMO
// DOCENTE: ANDRÉ BARBOSA
// DISCENTES: BENJAMIM CAMÊLO, KAUAN BENVENUTO, VICTOR LEITE.

#include <Ultrasonic.h>
#include <PID_v1.h>
#include <VarSpeedServo.h>

const int servoPin = 9;
//CONSTANTES KP, KI E KD
float Kp = 3.3;
float Ki = 1.8;
float Kd = 1.5;
double Setpoint, Input, Output, ServoOutput;
//OBJETO DA CLASSE PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//OBJETO DA CLASSE VARSPEEDSERVO 
//ESSA BIBLIOTECA PERMITE O AJUSTE DE VELOCIDADE DO SERVO MOTOR INDO DE 0 A 255
VarSpeedServo myServo;
Ultrasonic ultrasonic(13, 12); // (Trig PIN,Echo PIN)

void setup() {
  int b;
  Serial.begin(9600);
  myServo.attach(servoPin);

  int a = ultrasonic.read();
  //DEFININDO LIMITES DA PLATAFORMA UTILIZADA
  if(a >= 48){
    a = 48;
  }
  if(a <= 5){
    a=5;
  }
  if(a!=48){
    b=a;
  }
  Input = b;
  myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(180,0);
  
}

void loop() {
  int last_distance;
  int b;
  //SETPOINT É A DISTÂNCIA DESEJADA QUE A BOLINHA FIQUE COM RELAÇÃO AO 0 CM DA PLATAFORMA
  Setpoint = 15;
  int a = ultrasonic.read();
  Serial.println(a);
  delay(75);
  
  if(a >= 48){
    a = 48;
  }
  if(a <= 5){
    a=5;
  }
  if(a!=48){
    b=a;
  }else{
    b = last_distance;
  }
  Input = b;
  myPID.Compute();
  ServoOutput = Output;
  //VELOCIDADE AJUSTADA PARA 70 POIS FOI O VALOR QUE MAIS OBTEVE ESTABILIDADE
  myServo.write(ServoOutput, 70, false);
  last_distance = b;
}
