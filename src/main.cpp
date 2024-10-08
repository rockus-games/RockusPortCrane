#include <Arduino.h>
#include <PicoMQTT.h>
#include <GyverStepper2.h>

#define END_CAPSIZING 14
#define END_CARRIAGE_CLOSE 27
#define END_CARRIAGE_FAR 32
#define END_CARRIAGE_HOOK 33

#define MOTOR_F 23
#define MOTOR_B 22

#define STEP_CARRIAGE_1 21
#define STEP_CARRIAGE_2 19
#define STEP_CARRIAGE_3 18
#define STEP_CARRIAGE_4 5

GStepper2<STEPPER4WIRE> CarriageStepper(2038, STEP_CARRIAGE_1, STEP_CARRIAGE_3, STEP_CARRIAGE_2, STEP_CARRIAGE_4);

// AccelStepper CarriageStepper(8, STEP_CARRIAGE_1, STEP_CARRIAGE_2, STEP_CARRIAGE_3, STEP_CARRIAGE_4);


#define STEP_TURN_1 17
#define STEP_TURN_2 16
#define STEP_TURN_3 4
#define STEP_TURN_4 2

GStepper2<STEPPER4WIRE> TurnStepper(2038, STEP_TURN_1, STEP_TURN_3, STEP_TURN_2, STEP_TURN_4);
// AccelStepper TurnStepper(8, STEP_TURN_1, STEP_TURN_2, STEP_TURN_3, STEP_TURN_4);


#define STEP_CABLE_1 13
#define STEP_CABLE_2 12
#define STEP_CABLE_3 25
#define STEP_CABLE_4 26

GStepper2<STEPPER4WIRE> CableStepper(2038, STEP_CABLE_1, STEP_CABLE_3, STEP_CABLE_2, STEP_CABLE_4);
// AccelStepper CableStepper(8, STEP_CABLE_1, STEP_CABLE_2, STEP_CABLE_3, STEP_CABLE_4);


const int maxSpeed = 1600;
const int acceleration = 50;
PicoMQTT::Server mqtt;

const char* ssid     = "Rockus_PortCrane";
const char* password = "Kochegar";

void MoveMotor(const char* topic, const char* payload);
void Turn(const char* topic, const char* payload);
void MoveCarriage(const char* topic, const char* payload);
void MoveHook(const char* topic, const char* payload);

double motor_speed = 0;

bool far_end = false;
bool close_end = false;

bool hook_end = false;

bool cap_end = false;

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

  pinMode(END_CAPSIZING, INPUT_PULLUP);
  pinMode(END_CARRIAGE_CLOSE, INPUT_PULLUP);
  pinMode(END_CARRIAGE_FAR, INPUT_PULLUP);
  pinMode(END_CARRIAGE_HOOK, INPUT_PULLUP);

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



  // Serial.printf("%d %d %d %d\n", digitalRead(END_CAPSIZING), digitalRead(END_CARRIAGE_CLOSE), digitalRead(END_CARRIAGE_FAR), digitalRead(END_CARRIAGE_HOOK));

  if(digitalRead(END_CAPSIZING) == 0) {
    if(!cap_end) {
      CarriageStepper.setSpeed(0);
      CableStepper.setSpeed(0);
      CarriageStepper.disable();
      CableStepper.disable();
    }
    cap_end = true;
  } else {
    cap_end = false;
  }

  if(digitalRead(END_CARRIAGE_CLOSE) == 0) {
    if(!close_end) {
      CarriageStepper.setSpeed(0);
      CarriageStepper.disable();
    }
    close_end = true;
  } else {
    close_end = false;
  }

  if(digitalRead(END_CARRIAGE_FAR) == 0) {
    if(!far_end) {
      CarriageStepper.setSpeed(0);
      CarriageStepper.disable();
    }
    far_end = true;
  } else {
    far_end = false;
  }

  if(digitalRead(END_CARRIAGE_HOOK) == 0) {
    if(!hook_end) {
      CableStepper.setSpeed(0);
      CarriageStepper.setSpeed(0);
      CableStepper.disable();
      CarriageStepper.disable();
    }
    hook_end = true;
  } else {
    hook_end = false;
  }

  TurnStepper.tick();
  CarriageStepper.tick();
  CableStepper.tick();
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

double carriage_speed = 0;
double hook_speed = 0;
double turn_speed = 0;

void Turn(const char* topic, const char* payload) {
  Serial.println("Turning: " + String(payload));
  turn_speed = String(payload).toDouble();

  TurnStepper.setTarget(turn_speed*10);
  TurnStepper.setSpeed(turn_speed);
  if(turn_speed != 0) {
    TurnStepper.enable();
  } else {
    TurnStepper.disable();
  }
}

void MoveCarriage(const char* topic, const char* payload) {
  Serial.println("Moving carriage: " + String(payload));
  carriage_speed = String(payload).toDouble();

  if(close_end) {
    carriage_speed = carriage_speed < 0 ? 0 : carriage_speed;
  }

  if(hook_end) {
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
    hook_speed = hook_speed > 0 ? 0 : hook_speed;
  }

  if(far_end || cap_end) {
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
  }

  CarriageStepper.setTarget(carriage_speed*10);
  CarriageStepper.setSpeed(carriage_speed);
  if(carriage_speed != 0) {
    CarriageStepper.enable();
  } else {
    CarriageStepper.disable();
  }
}

void MoveHook(const char* topic, const char* payload) {
  Serial.println("Moving hook: " + String(payload));
  hook_speed = String(payload).toDouble();


  if(hook_end || cap_end) {
    hook_speed = hook_speed > 0 ? 0 : hook_speed;
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
  }

  CableStepper.setTarget(hook_speed*10);
  CableStepper.setSpeed(hook_speed);
  if(hook_speed != 0) {
    CableStepper.enable();
  } else {
    CableStepper.disable();
  }
}