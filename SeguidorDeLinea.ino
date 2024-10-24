#include <PID_v1.h>
#include "SparkFun_TB6612.h"
#include <QTRSensors.h>
#include <JC_Button.h>

// Definir pines
#define AIN2 10
#define AIN1 9
#define BIN1 6
#define BIN2 5
#define PWMA 11
#define PWMB 3
#define STBY 7

#define BTN1_PIN 2
#define BTN2_PIN 4

// Pines de los sensores de línea
#define NUM_SENSORS 8
unsigned char sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Variables de los motores
Motor motorLeft = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, 1, STBY);

// Definir botones
Button btn1(BTN1_PIN);
Button btn2(BTN2_PIN);

// Parámetros del PID
double Kp = 0.2, Ki = 0.0, Kd = 0.0;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

class SeguidorDeLineaController {
  public:
    static QTRSensors qtr;
    static unsigned int sensorValues[NUM_SENSORS];

    static void inicializar() {
      Serial.begin(9600);
      btn1.begin();
      btn2.begin();
      qtr.setTypeAnalog();
      qtr.setSensorPins(sensorPins, NUM_SENSORS);
      Setpoint = 3500;  // Valor central ideal para mantener el carrito en línea
      myPID.SetMode(AUTOMATIC);
      myPID.SetOutputLimits(-255, 255);  // Limitar la salida del PID
      Serial.println("Sistema inicializado.");
    }

    static void calibrar() {
      Serial.println("Iniciando calibración...");
      for (int i = 0; i < 400; i++) {
        qtr.calibrate();
        delay(20);
      }
      Serial.println("Calibración completada.");
    }

    static void seguirLinea() {
      Serial.println("Iniciando seguimiento de línea...");
      while (true) {
        // Leer los sensores y obtener la posición
        int position = qtr.readLineBlack(sensorValues);

        // Establecer el valor de entrada del PID
        Input = position;
        myPID.Compute();

        // Ajustar la velocidad de los motores según la salida del PID
        int motorSpeedLeft = constrain(255 - Output, 0, 255);
        int motorSpeedRight = constrain(255 + Output, 0, 255);

        // Mover los motores
        motorLeft.drive(motorSpeedLeft);
        motorRight.drive(motorSpeedRight);

        delay(20);  // Pequeño retardo para estabilidad
      }
    }

    static void ejecutar() {
      Serial.println("Esperando acción del usuario...");

      while (true) {
        btn1.read();
        btn2.read();

        if (btn1.wasPressed()) {
          calibrar();
        } else if (btn2.wasPressed()) {
          Serial.println("Esperando 3 segundos para iniciar seguimiento de línea...");
          delay(3000);
          seguirLinea();
        }

        delay(50);  // Pequeño retardo para leer los botones con estabilidad
      }
    }
};

// Inicialización de los sensores de línea y sus valores
QTRSensors SeguidorDeLineaController::qtr;
unsigned int SeguidorDeLineaController::sensorValues[NUM_SENSORS];

void setup() {
  SeguidorDeLineaController::inicializar();
}

void loop() {
  SeguidorDeLineaController::ejecutar();
}
