/*
 ****************CONTROL PID SEGUIDOR DE LINEA******************

   AUTOR: ARIEL ARTURO RÌOS SIERRA

   CORREO: arturi.marking@gmail.com

   FECHA: 12 de marzo del 2018

   VERSION: 1.0
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

const float velocidadBase = 100.0;

const float kP = 0.02;
const float kI = 0.00003;
const float kD = 0.04;

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

  Serial.begin(9600);

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

  attachInterrupt(0, leftEncoderEvento, CHANGE);
  attachInterrupt(1, rightEncoderEvento, CHANGE);
}

void leftEncoderEvento() {
  if (digitalRead(ENCO_A_IZQ) == HIGH) {
    if (digitalRead(ENCO_B_IZQ) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(ENCO_B_IZQ) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}

void rightEncoderEvento() {
  if (digitalRead(ENCO_A_DER) == HIGH) {
    if (digitalRead(ENCO_B_DER) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(ENCO_B_DER) == LOW) {
      rightCount--;
    } else {
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
    Serial.println("Apagado");
  }

  if (estadoRobot) {
    inicioRobot();
  }

  previous = estadoBoton;
}

void inicioRobot() {
  digitalWrite(STBY, HIGH);

  leerVelocidades();

  int errorPID = calculoPID(); // = -1.2

  float velocidadActualIzq = velocidadIzq - errorPID; // antes  era -
  float velocidadActualDer = velocidadDer + errorPID; // antes era +

  float velocidadRPMIzq = rpmIzq - errorPID; // 1500 - (-233.90) = 1733.9 // antes  era ; - 0 - (-1.2) = 1.2
  float velocidadRPMDer = rpmDer + errorPID; // 1500 + (-233.90) = 1266.1 // antes era + ; 0 + (-1.2) = -1.2 

  Serial.println("Error PID" + String(errorPID));

  int potenciaPWMIzq = ceil(map(velocidadRPMIzq, 0, 3000, 0, 255)); // 147.00
  int potenciaPWMDer = ceil(map(velocidadRPMDer, 0, 3000, 0, 255)); // 107.00

  if (potenciaPWMIzq > velocidadBase) {
    potenciaPWMIzq = velocidadBase;
  }

  if (potenciaPWMDer > velocidadBase) {
    potenciaPWMDer = velocidadBase;
  }

  Serial.println("Potencia PWM Izq: " + String(potenciaPWMIzq) + "\n" + "Potencia PWM Der: " + String(potenciaPWMDer));
  delay(1000);

  //  analogWrite(PWMA, potenciaPWMDer);
  //  analogWrite(PWMB, potenciaPWMIzq);
  //  adelante();

  posicionAnterior = position;
}

void leerVelocidades() {
  long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= 1000) {
    noInterrupts();
    rpmIzq = 60 * leftCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    rpmDer = 60 * rightCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    velocidadIzq = rpmIzq / relacionRuedas * 3.1416 * diametroRueda * 60 / 1000000;

    velocidadIzq = rpmDer / relacionRuedas * 3.1416 * diametroRueda * 60 / 1000000;

    leftCount = 0;
    rightCount = 0;

    tiempoAnterior = millis();
    interrupts();
  }
}

unsigned int calculoPID() {
  position = leerPosicionError(); // -1550  // -6

  derivativo = position - posicionAnterior; // 14  // -5035  // - 6- (-6) = 0

  integral = integral + position; // 14 . // -5035 + 14 = -5021 // -6 - 6 = -12

  return (kP * position + kI * integral + kD * derivativo); // kp = 0.02, ki = 0.00003, kd = 0.04 => -1.2  - 0.00036  - 0  = -1.2

}

unsigned int leerPosicionError() {

  qtra.read(valoresSensorIr);

  posicion_actual = qtra.readLine(valoresSensorIr); // ant = 3485 // act = 5050

  for (unsigned char i = 0; i < NUM_SENSOR_IR; i++)
  {
    Serial.print(valoresSensorIr[i]);
    Serial.print('\t');
  }

  Serial.println("| "  + String(posicion_actual));

  delay(250);

  return (POS_OBJECTIVO - posicion_actual); // 3500 - 5050 = -1550
}

void adelante() {
  digitalWrite(AIN_1, HIGH);
  digitalWrite(AIN_2, LOW);

  digitalWrite(BIN_1, LOW);
  digitalWrite(BIN_2, HIGH);
}
