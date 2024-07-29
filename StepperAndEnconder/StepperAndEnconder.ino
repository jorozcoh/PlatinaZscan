#include <AccelStepper.h>
#include <Separador.h>
#include <Arduino.h>

// Pin definitions
int stopSensor1 = 13; // final de carrera 1
int stopSensor2 = 32; // final de carrera 2
int LED = 2; // LED interno para las interrupciones

// Encoder pins
#define encoderPinA 25
#define encoderPinB 26

// Encoder properties
const int pulsesPerRevolution = 20000;
const int quadratureSteps = 4; // Número de pasos por ciclo (A y B subiendo y bajando)

// Variables
volatile bool CrashSensor1 = false; // bandera para final de carrera 1
volatile bool CrashSensor2 = false; // bandera para final de carrera 2
volatile bool lastState1 = HIGH; // último estado del sensor 1
volatile bool lastState2 = HIGH; // último estado del sensor 2
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50; // tiempo de debounce en milisegundos
bool homing = false; // bandera para indicar si el motor está en modo Home

// Encoder variables
volatile int counter = 0;
volatile int lastEncoderState = 0;
hw_timer_t *timer = NULL;
const int timerInterval = 200;  // Intervalo del timer en microsegundos (1 ms)

// Serial communication variables
Separador s; // estructura para separar del serial
String elemento1; // instrucción de motor/posición
int motion; // posición a la que moverse

// Motor setup
AccelStepper stepper(AccelStepper::DRIVER, 33, 14); // pin 33 pulso menos y 14 dir menos

// Function declarations
void IRAM_ATTR buttonInterrupt1(); // Funcion interrupcion final de carrera 1 (el del encoder)
void IRAM_ATTR buttonInterrupt2();// Funcion interrupcion final de carrera 2
void serialEvent(); // funcion para separar los comandos del serial cuando se mandan los :
void executeCommand(String cmd);
void stopMotorAndMove(int steps, bool resetPosition); // funcion para detener el motor en los finales de carrera y que haga el rebote
void handleCrashSensors();
void IRAM_ATTR onTimer(); //interrupcion timer encoder
void handleEncoderState(); // funcion de conteo de pulsos encoder

void setup() {
  // Pin mode setup
  pinMode(stopSensor1, INPUT_PULLUP);
  pinMode(stopSensor2, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Interrupts setup
  attachInterrupt(digitalPinToInterrupt(stopSensor1), buttonInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stopSensor2), buttonInterrupt2, CHANGE);

  // Encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(encoderPinA), [] {
    timerAlarmEnable(timer);
  }, CHANGE);

  // Timer setup
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timerInterval, true);

  // Serial initialization
  Serial.begin(921600);

  // Motor initial setup
  stepper.setMaxSpeed(8000); //maxima aceleracion definida
  stepper.setAcceleration(70); // ideal 50
}

void loop() { // en el loop se revisan constantemente los finales de carrera, se deserializan comandos y se usa el homing para mover el motor a posicion o a velocidad especifica
  handleCrashSensors();

  // Check serial for commands
  if (Serial.available()) {
    serialEvent();
    executeCommand(elemento1);
  }

  // Run the motor in homing mode or normal mode
  if (homing) {
    stepper.runSpeed();
  } else {
    stepper.run();
  }

  // Print encoder counter value
  static int lastCounter = 0;
  if (counter != lastCounter) {
    Serial.print("Counter: ");
    Serial.println(counter);
    lastCounter = counter;
  }
}

void handleCrashSensors() {
  unsigned long currentTime = millis();

  // Handle crash sensor 1
  if (CrashSensor1 && (currentTime - lastDebounceTime1 > debounceDelay)) {
    lastDebounceTime1 = currentTime;
    CrashSensor1 = false;
    if (digitalRead(stopSensor1) == LOW) {
      digitalWrite(LED, HIGH);
      stopMotorAndMove(3000, true);
      Serial.println("Crash Sensor 1 Triggered");
    } else {
      digitalWrite(LED, LOW);
    }
  }

  // Handle crash sensor 2
  if (CrashSensor2 && (currentTime - lastDebounceTime2 > debounceDelay)) {
    lastDebounceTime2 = currentTime;
    CrashSensor2 = false;
    if (digitalRead(stopSensor2) == LOW) {
      digitalWrite(LED, HIGH);
      stopMotorAndMove(-3000, false);
      Serial.println("Crash Sensor 2 Triggered");
    } else {
      digitalWrite(LED, LOW);
    }
  }
}

/* Serial event handler */
void serialEvent() {
  String datosrecibidos = Serial.readString();
  elemento1 = s.separa(datosrecibidos, ':', 0);
  String elemento2 = s.separa(datosrecibidos, ':', 1);
  motion = elemento2.toFloat();

  Serial.println("el elemento 1 es: " + elemento1);
  Serial.println("el elemento 2 es: " + elemento2);
}

/* Crash sensor 1 interrupt handler */
void IRAM_ATTR buttonInterrupt1() {
  bool currentState1 = digitalRead(stopSensor1);
  if (currentState1 != lastState1) {
    lastState1 = currentState1;
    CrashSensor1 = true;
  }
}

/* Crash sensor 2 interrupt handler */
void IRAM_ATTR buttonInterrupt2() {
  bool currentState2 = digitalRead(stopSensor2);
  if (currentState2 != lastState2) {
    lastState2 = currentState2;
    CrashSensor2 = true;
  }
}

void stopMotorAndMove(int steps, bool resetPosition) {
  stepper.setSpeed(0); // velocidad a 0 para freno abrupto
  stepper.runSpeedToPosition(); // Asegurar que el motor se detenga inmediatamente

  delay(1000); // Esperar 1 segundo

  stepper.move(steps); // Move the motor by the specified number of steps
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    handleCrashSensors(); // Ensure crash sensors are still checked during movement
  }

  if (resetPosition) {
    stepper.setCurrentPosition(0); // Establecer la posición de inicio en 0
    counter = 0; // Reset encoder counter
  }

  stepper.stop(); // Stop the motor completely
  digitalWrite(LED, LOW); // Apagar el LED cuando el motor se detiene
}

void executeCommand(String cmd) {
  cmd.trim(); // Eliminar espacios en blanco al inicio y al final

  if (cmd.startsWith("Motor")) {
    // Moverse a una posición especificada usando un método no bloqueante
    homing = false; // Ensure we are not in homing mode
    stepper.moveTo(motion);
  } else if (cmd.startsWith("Home")) {
    // Mover el motor a una velocidad constante hacia el crash sensor 1
    homing = true; // Enter homing mode
    stepper.setSpeed(-6000);
  }
}

/* Timer interrupt handler */
void IRAM_ATTR onTimer() {
  handleEncoderState();
}

/* Encoder state handler */
void handleEncoderState() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  int currentState = (stateA << 1) | stateB;

  if (currentState != lastEncoderState) {
    if ((currentState == 1 && lastEncoderState == 0) || (currentState == 2 && lastEncoderState == 3) ||
        (currentState == 3 && lastEncoderState == 1) || (currentState == 0 && lastEncoderState == 2)) {
      counter--;
    } else {
      counter = max(0, counter + 1);
    }
  }

  lastEncoderState = currentState;
}
