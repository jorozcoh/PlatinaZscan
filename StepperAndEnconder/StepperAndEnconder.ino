#include <AccelStepper.h>
#include <Separador.h>
#include <Arduino.h>

// Pin definitions
int stopSensor1 = 13; // final de carrera 1
int stopSensor2 = 32; // final de carrera 2
int LED = 2; // LED interno para las interrupciones

// Variables
volatile bool CrashSensor1 = false; // bandera para final de carrera 1
volatile bool CrashSensor2 = false; // bandera para final de carrera 2
volatile bool lastState1 = HIGH; // último estado del sensor 1
volatile bool lastState2 = HIGH; // último estado del sensor 2
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50; // tiempo de debounce en milisegundos
bool homing = false; // bandera para indicar si el motor está en modo Home
bool moveComplete = false; // bandera para indicar si el movimiento ha sido completado

// Serial communication variables
Separador s; // estructura para separar del serial
String elemento1; // instrucción de motor/posición
float motion_mm; // posición a la que moverse en mm
String receivedData = ""; // buffer to store incoming serial data

// Motor setup
AccelStepper stepper(AccelStepper::DRIVER, 33, 14); // pin 33 pulso menos y 14 dir menos

// Function declarations
void IRAM_ATTR buttonInterrupt1();
void IRAM_ATTR buttonInterrupt2();
void serialEvent();
void executeCommand(String cmd);
void stopMotorAndMove(int steps, bool resetPosition);
void handleCrashSensors();
void checkMovementComplete();

void setup() {
  // Pin mode setup
  pinMode(stopSensor1, INPUT_PULLUP);
  pinMode(stopSensor2, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // Interrupts setup
  attachInterrupt(digitalPinToInterrupt(stopSensor1), buttonInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stopSensor2), buttonInterrupt2, CHANGE);

  // Serial initialization
  Serial.begin(115200);

  // Motor initial setup
  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(70); // ideal 50
}

void loop() {
  handleCrashSensors();
  checkMovementComplete();

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

  // Print motor position while moving
  static long lastPosition = -1;
  long currentPosition = stepper.currentPosition();
  if (currentPosition != lastPosition) {
    float position_mm = (currentPosition - 180.15) / 398.77; // Convert steps to mm
    Serial.print("Position: ");
    Serial.println(position_mm);
    lastPosition = currentPosition;
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
    } else {
      digitalWrite(LED, LOW);
    }
  }
}

/* Serial event handler */
void serialEvent() {
  while (Serial.available()) {
    char received = Serial.read();
    if (received == '\n') { // end of command
      elemento1 = s.separa(receivedData, ':', 0);
      String elemento2 = s.separa(receivedData, ':', 1);
      motion_mm = elemento2.toFloat();
      receivedData = ""; // clear buffer
    } else {
      receivedData += received; // add to buffer
    }
  }
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
    Serial.println("HomeComplete");
    
  }

  stepper.stop(); // Stop the motor completely
  digitalWrite(LED, LOW); // Apagar el LED cuando el motor se detiene
}

void executeCommand(String cmd) {
  cmd.trim(); // Eliminar espacios en blanco al inicio y al final

  if (cmd.startsWith("MoveAbs")) {
    // Convert mm to steps
    int motion_steps = 398.77 * motion_mm + 180.15;
    // Moverse a una posición especificada usando un método no bloqueante
    homing = false; // Ensure we are not in homing mode
    moveComplete = false;
    stepper.moveTo(motion_steps);
  } else if (cmd.startsWith("MoveRelative")) {
    // Convert mm to steps for relative movement
    int motion_steps = 398.77 * motion_mm;
    // Move a specified distance from the current position
    homing = false; // Ensure we are not in homing mode
    moveComplete = false;
    stepper.move(motion_steps);
  } else if (cmd.startsWith("Home")) {
    // Mover el motor a una velocidad constante hacia el crash sensor 1
    homing = true; // Enter homing mode
    stepper.setSpeed(-2000);
  } else if (cmd.startsWith("Stop")) {
    // Stop the motor immediately
    homing = false; // Ensure we are not in homing mode
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition()); // Retain current position
    moveComplete = true; // Indicate the move is complete
    Serial.println("MoveComplete");
  }
}

void checkMovementComplete() {
  if (!moveComplete && stepper.distanceToGo() == 0) {
    moveComplete = true;
    Serial.println("MoveComplete");
  }
}
