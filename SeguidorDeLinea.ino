#include <JC_Button.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <PID_v1.h>

// Definir los pines de los botones y debounce
#define BTN1_PIN 2
#define BTN2_PIN 4
#define DEBOUNCE_MS 50

// Crear los objetos de botones con debounce
Button btn1(BTN1_PIN, DEBOUNCE_MS);
Button btn2(BTN2_PIN, DEBOUNCE_MS);

// Definir pines de sensores y motores (mismos que los mencionados anteriormente)
#define AIN2 10
#define AIN1 9
#define BIN1 6
#define BIN2 5
#define PWMA 11
#define PWMB 3
#define STBY 8

// Configuración de sensores QTR (8 sensores conectados a pines A0-A7)
QTRSensorsAnalog qtr((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, 8);

// Definir los motores
Motor motorIzquierdo = Motor(AIN1, AIN2, PWMA, STBY);
Motor motorDerecho = Motor(BIN1, BIN2, PWMB, STBY);

// Variables de control PID
double posicionLinea, setPoint, input, output;
double Kp = 0.2, Ki = 0, Kd = 0.1;
PID miPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// Clase que controla el seguidor de línea
class SeguidorDeLineaController {
public:
  static void inicializar() {
    Serial.begin(9600);
    btn1.begin();
    btn2.begin();
    qtr.setTypeAnalog();
    qtr.setEmitterPin(2); // Pin para los LEDs emisores

    // Configurar el PID
    setPoint = 3500;  // Punto central de los sensores (ajustable según tu configuración)
    miPID.SetMode(AUTOMATIC);
    miPID.SetOutputLimits(-255, 255); // Limitar salida del PID para control de motores
  }

  static void calibrar() {
    Serial.println("Calibrando sensores...");
    for (int i = 0; i < 400; i++) {  // Calibración durante 8 segundos (400 iteraciones)
      qtr.calibrate();
      delay(20);
    }
    Serial.println("Calibración completada.");
  }

  static void seguirLinea() {
    input = qtr.readLine((unsigned int[]) {0, 0, 0, 0, 0, 0, 0, 0}); // Leer línea
    miPID.Compute(); // Calcular salida PID

    // Controlar los motores basado en el error
    int velocidadBase = 150;  // Velocidad base del robot
    int velocidadIzquierda = velocidadBase - output;
    int velocidadDerecha = velocidadBase + output;

    motorIzquierdo.drive(velocidadIzquierda);
    motorDerecho.drive(velocidadDerecha);
  }

  static void ejecutar() {
    while (true) {
      btn1.read();
      btn2.read();

      if (btn1.wasPressed()) {
        calibrar();  // Iniciar calibración
      }

      if (btn2.wasPressed()) {
        Serial.println("Siguiendo línea en 3 segundos...");
        delay(3000);  // Esperar 3 segundos antes de comenzar a seguir la línea
        while (true) {
          seguirLinea();  // Seguir la línea
        }
      }
    }
  }
};

void setup() {
  SeguidorDeLineaController::inicializar();
}

void loop() {
  SeguidorDeLineaController::ejecutar();
}
