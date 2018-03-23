/*
 ****************CONTROL PID SEGUIDOR DE LINEA******************

   AUTOR: ARIEL ARTURO RÌOS SIERRA

   CORREO: arturi.marking@gmail.com

   FECHA: 12 de marzo del 2018

   VERSION: 1.2
 ***************************************************************
*/
#include <QTRSensors.h>

#define AIN_1 7
#define AIN_2 8

#define BIN_1 10
#define BIN_2 13

#define PWMA 6
#define PWMB 11

#define STBY 9

#define ENCO_A_IZQ 2
#define ENCO_B_IZQ 4
#define ENCO_A_DER 3
#define ENCO_B_DER 5

#define POS_OBJECTIVO 3500

#define BUTTON_STATUS 12

#define NUM_SENSOR_IR 8

int estadoBoton = HIGH;
int esperaTiempo = 560;
long tiempo = 0;
int previous = LOW;
bool estadoRobot = false;

const float velocidadBase = 50.0;

const float kP = 0.08;
const float kI = 0.0003;
const float kD = 0.12;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

float rpmIzq = 0;
float rpmDer = 0;
float velocidadIzq = 0;
float velocidadDer = 0;

unsigned long tiempoAnterior = 0;
unsigned int pulsosPorRevolucion = 8;

unsigned int diametroRueda = 20;
int relacionRuedas = 10;

unsigned int posicion_actual = 0;
unsigned int position = 0;
unsigned int posicionAnterior = 0;
unsigned int derivativo = 0;
unsigned int integral = 0;

QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
}, NUM_SENSOR_IR);

unsigned int valoresSensorIr[NUM_SENSOR_IR];

void setup() {

  pinMode(BUTTON_STATUS, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(ENCO_A_IZQ, INPUT);
  pinMode(ENCO_B_IZQ, INPUT);

  pinMode(ENCO_A_DER, INPUT);
  pinMode(ENCO_B_DER, INPUT);

  attachInterrupt(0, leftEncoderEvento, RISING);
  attachInterrupt(1, rightEncoderEvento, RISING);
}

void leftEncoderEvento() {
  if (digitalRead(ENCO_A_IZQ) == HIGH) {
    if (digitalRead(ENCO_B_IZQ) == LOW) {
      leftCount++;
    }
  } else {
    if (digitalRead(ENCO_B_IZQ) == HIGH) {
      leftCount++;
    }
  }
}

void rightEncoderEvento() {
  if (digitalRead(ENCO_A_DER) == HIGH) {
    if (digitalRead(ENCO_B_DER) == LOW) {
      rightCount++;
    }
  } else {
    if (digitalRead(ENCO_B_DER) == HIGH) {
      rightCount++;
    }
  }
}

void loop() {
  estadoBoton = digitalRead(BUTTON_STATUS);
  delay(40);

  if (estadoBoton == HIGH && previous == LOW && millis() - tiempo > esperaTiempo) {
    if (estadoRobot)
      estadoRobot = false;
    else
      estadoRobot = true;
    tiempo = millis();
  }

  if (!estadoRobot) {
    int motoresEncendidos = digitalRead(STBY);
    if (motoresEncendidos == 1) {
      digitalWrite(STBY, LOW);
    }
  }

  if (estadoRobot) {
    inicioRobot();
  }

  previous = estadoBoton;
}

void inicioRobot() {
  digitalWrite(STBY, HIGH);

  float errorPID = calculoPID(); // = -1.2

  leerVelocidades();

  float velocidadRPMIzq = rpmIzq + errorPID; // 1500 - (-233.90) = 1733.9 // antes  era ; - 0 - (-1.2) = 1.2
  float velocidadRPMDer = rpmDer - errorPID; // 1500 + (-233.90) = 1266.1 // antes era + ; 0 + (-1.2) = -1.2

  if (velocidadRPMIzq == 0 && velocidadRPMDer == 0) {
    primeraArrancada();
  }

  float potenciaPWMIzq = map(velocidadRPMIzq, 0, 3000, 0, 255); // 147.00
  float potenciaPWMDer = map(velocidadRPMDer, 0, 3000, 0, 255); // 107.00

  if (potenciaPWMIzq > velocidadBase) {
    potenciaPWMIzq = velocidadBase;
  }

  if (potenciaPWMDer > velocidadBase) {
    potenciaPWMDer = velocidadBase;
  }

  analogWrite(PWMA, potenciaPWMDer);
  analogWrite(PWMB, potenciaPWMIzq);
  adelante();

  posicionAnterior = position;
}

void leerVelocidades() {
  long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= 1000) {
    noInterrupts();
    rpmIzq = 60 * leftCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    rpmDer = 60 * rightCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    leftCount = 0;
    rightCount = 0;

    tiempoAnterior = millis();
    interrupts();
  }
}

unsigned int calculoPID() {
  position = leerPosicionError(); // 3500

  derivativo = position - posicionAnterior; // 3500 - 3500

  integral = position + posicionAnterior; // 14 . //

  return (kP * position + kI * integral + kD * derivativo); // kp = 0.02, ki = 0.00003, kd = 0.04 => -1.2  - 0.00036  - 0  = -1.2

}

unsigned int leerPosicionError() {

  qtra.read(valoresSensorIr);

  posicion_actual = qtra.readLine(valoresSensorIr);

  return (POS_OBJECTIVO - posicion_actual);
}

void primeraArrancada() {
  analogWrite(PWMA, velocidadBase);
  analogWrite(PWMB, velocidadBase);
  adelante();
}

void adelante() {
  digitalWrite(AIN_1, HIGH);
  digitalWrite(AIN_2, LOW);

  digitalWrite(BIN_1, LOW);
  digitalWrite(BIN_2, HIGH);
}
