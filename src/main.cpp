#include <Arduino.h>
#include <PicoMQTT.h>
#include <AccelStepper.h>

#define END_CAPSIZING 34
#define END_CARRIAGE_CLOSE 35
#define END_CARRIAGE_FAR 32
#define END_CARRIAGE_HOOK 33

#define MOTOR_F 23
#define MOTOR_B 22

#define STEP_CARRIAGE_1 21
#define STEP_CARRIAGE_2 19
#define STEP_CARRIAGE_3 18
#define STEP_CARRIAGE_4 5

AccelStepper CarriageStepper(8, STEP_CARRIAGE_1, STEP_CARRIAGE_2, STEP_CARRIAGE_3, STEP_CARRIAGE_4);


#define STEP_TURN_1 17
#define STEP_TURN_2 16
#define STEP_TURN_3 4
#define STEP_TURN_4 2

AccelStepper TurnStepper(8, STEP_TURN_1, STEP_TURN_2, STEP_TURN_3, STEP_TURN_4);


#define STEP_CABLE_1 13
#define STEP_CABLE_2 12
#define STEP_CABLE_3 14
#define STEP_CABLE_4 27

AccelStepper CableStepper(8, STEP_CABLE_1, STEP_CABLE_2, STEP_CABLE_3, STEP_CABLE_4);


const int maxSpeed = 1600;
const int acceleration = 500;
PicoMQTT::Server mqtt;

const char* ssid     = "Rockus_PortCrane";
const char* password = "Kochegar";

void MoveMotor(const char* topic, const char* payload);
void Turn(const char* topic, const char* payload);
void MoveCarriage(const char* topic, const char* payload);
void MoveHook(const char* topic, const char* payload);

double motor_speed = 0;

void setup() {
  pinMode(MOTOR_F, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(STEP_CARRIAGE_1, OUTPUT);
  pinMode(STEP_CARRIAGE_2, OUTPUT);
  pinMode(STEP_CARRIAGE_3, OUTPUT);
  pinMode(STEP_CARRIAGE_4, OUTPUT);
  pinMode(STEP_TURN_1, OUTPUT);
  pinMode(STEP_TURN_2, OUTPUT);
  pinMode(STEP_TURN_3, OUTPUT);
  pinMode(STEP_TURN_4, OUTPUT);
  pinMode(STEP_CABLE_1, OUTPUT);
  pinMode(STEP_CABLE_2, OUTPUT);
  pinMode(STEP_CABLE_3, OUTPUT);
  pinMode(STEP_CABLE_4, OUTPUT);

  pinMode(END_CAPSIZING, INPUT);
  pinMode(END_CARRIAGE_CLOSE, INPUT);
  pinMode(END_CARRIAGE_FAR, INPUT);
  pinMode(END_CARRIAGE_HOOK, INPUT);

  CarriageStepper.setMaxSpeed(maxSpeed);
  CarriageStepper.setAcceleration(acceleration);
  TurnStepper.setMaxSpeed(maxSpeed);
  TurnStepper.setAcceleration(acceleration);
  CableStepper.setMaxSpeed(maxSpeed);
  CableStepper.setAcceleration(acceleration);

  CarriageStepper.setSpeed(0);
  TurnStepper.setSpeed(0);
  CableStepper.setSpeed(0);

  Serial.begin(115200);

  Serial.print("Запуск точки доступа");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Адрес: ");
  Serial.println(IP);

  mqtt.subscribe("move_carriage", MoveCarriage);
  mqtt.subscribe("turn", Turn);
  mqtt.subscribe("move_motor", MoveMotor);
  mqtt.subscribe("move_hook", MoveHook);

  mqtt.begin();
}

void loop() {
  mqtt.loop();

  TurnStepper.runSpeed();
  CarriageStepper.runSpeed();
  CableStepper.runSpeed();
}

void MoveMotor(const char* topic, const char* payload) {
  Serial.println("Moving motor: " + String(payload));
  motor_speed = String(payload).toDouble();

  if (motor_speed > 0) {
    analogWrite(MOTOR_F, motor_speed);
    digitalWrite(MOTOR_B, LOW);
  } else if(motor_speed < 0) {
    analogWrite(MOTOR_F, -motor_speed);
    digitalWrite(MOTOR_B, HIGH);
  } else {
    analogWrite(MOTOR_F, LOW);
    digitalWrite(MOTOR_B, LOW);
  }
}

void Turn(const char* topic, const char* payload) {
  Serial.println("Turning: " + String(payload));
  double turn_speed = String(payload).toDouble();

  TurnStepper.setSpeed(turn_speed);
}

void MoveCarriage(const char* topic, const char* payload) {
  Serial.println("Moving carriage: " + String(payload));
  double move_speed = String(payload).toDouble();

  if(analogRead(END_CARRIAGE_CLOSE) > 0 || analogRead(END_CARRIAGE_FAR) > 0) {
    return;
  }

  CarriageStepper.setSpeed(move_speed);
}

void MoveHook(const char* topic, const char* payload) {
  Serial.println("Moving hook: " + String(payload));
  double move_speed = String(payload).toDouble();

  if(analogRead(END_CARRIAGE_HOOK) > 0) {
    return;
  }

  CableStepper.setSpeed(move_speed);
}