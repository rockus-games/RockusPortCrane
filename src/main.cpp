#include <Arduino.h>
#include <PicoMQTT.h>
#include <GyverStepper2.h>
#include "I2Cdev.h"

// Пины, к которым подключены концевые выключатели
#define END_CARRIAGE_CLOSE 16
#define END_CARRIAGE_FAR 32

#define MOVE_STEP 2
#define MOVE_DIR 4
#define MOVE_EN 15

// Шаговый двигатель движения
GStepper2<STEPPER2WIRE> MoveStepper(2038, MOVE_STEP, MOVE_DIR, MOVE_EN);

// Пины, к которым подключены шаговые двигатели
#define CARRIAGE_STEP 14
#define CARRIAGE_DIR 12
#define CARRIAGE_EN 27

// Шаговый двигатель каретки
GStepper2<STEPPER2WIRE> CarriageStepper(2038, CARRIAGE_STEP, CARRIAGE_DIR, CARRIAGE_EN);

#define CABLE_STEP 25
#define CABLE_DIR 26
#define CABLE_EN 33 

// Шаговый двигатель крюка
GStepper2<STEPPER2WIRE> CableStepper(2038, CABLE_STEP, CABLE_DIR, CABLE_EN);

// Максимальная скорость и ускорение двигателей
const int maxSpeed = 1600;
const int acceleration = 50;

// MQTT-сервер для обмена данными с приложением
PicoMQTT::Server mqtt;

// Настройки подключения к WiFi
const char* ssid     = "Rockus_ContainerCrane";
const char* password = "Kochegar";

// Функции для обработки сигналов с приложения
void MoveMotor(const char* topic, const char* payload);
void MoveCarriage(const char* topic, const char* payload);
void MoveHook(const char* topic, const char* payload);

// Скорости двигателей
double move_speed = 0;

// Состояние нажатия концевиков
bool far_end = false;
bool close_end = false;

void setup() {
  // Разрешаем отладочный вывод
  Serial.begin(115200);

  // Настройка пинов на вывод сигнала
  pinMode(MOVE_STEP, OUTPUT);
  pinMode(MOVE_DIR, OUTPUT);
  pinMode(MOVE_EN, OUTPUT);
  pinMode(CARRIAGE_STEP, OUTPUT);
  pinMode(CARRIAGE_DIR, OUTPUT);
  pinMode(CARRIAGE_EN, OUTPUT);
  pinMode(CABLE_STEP, OUTPUT);
  pinMode(CABLE_DIR, OUTPUT);
  pinMode(CABLE_EN, OUTPUT);

  // Настройка пинов на ввод сигнала
  pinMode(END_CARRIAGE_CLOSE, INPUT_PULLUP);
  pinMode(END_CARRIAGE_FAR, INPUT_PULLUP);

  // Выставляем максимальную скорость и ускорение двигателей
  CarriageStepper.setMaxSpeed(maxSpeed);
  CarriageStepper.setAcceleration(acceleration);
  CableStepper.setMaxSpeed(maxSpeed);
  CableStepper.setAcceleration(acceleration);
  MoveStepper.setMaxSpeed(maxSpeed);
  MoveStepper.setAcceleration(acceleration);

  // Задаём начальную скорость двигателям  pinMode(MOVE_DIR, OUTPUT);

  CarriageStepper.setSpeed(0);
  CableStepper.setSpeed(0);
  MoveStepper.setSpeed(0);

  // Запуск MQTT-сервера и подключение к WiFi
  Serial.print("Запуск точки доступа");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Адрес: ");
  Serial.println(IP);

  // Подписываемся на MQTT-топики
  mqtt.subscribe("move_carriage", MoveCarriage);
  mqtt.subscribe("move_motor", MoveMotor);
  mqtt.subscribe("move_hook", MoveHook);

  mqtt.begin();
}

void loop() {
  // Обработка MQTT
  mqtt.loop();

  // Проверка концевика на основании крана
  if(digitalRead(END_CARRIAGE_CLOSE) == 0) {
    if(!close_end) {
      CarriageStepper.setSpeed(0);
      CarriageStepper.disable();
    }
    close_end = true;
  } else {
    close_end = false;
  }

  // Проверка концевика на краю крана
  if(digitalRead(END_CARRIAGE_FAR) == 0) {
    if(!far_end) {
      CarriageStepper.setSpeed(0);
      CarriageStepper.disable();
    }
    far_end = true;
  } else {
    far_end = false;
  }

  CarriageStepper.tick();
  CableStepper.tick();
  MoveStepper.tick();
}

// Функция движения двигателей
void MoveMotor(const char* topic, const char* payload) {
  // Сохраняем скорость двигателя
  move_speed = String(payload).toDouble();

  // Двигаем двигатели
  MoveStepper.setTarget(move_speed*10);
  MoveStepper.setSpeed(move_speed);
  if(move_speed != 0) {
    MoveStepper.enable();
  } else {
    MoveStepper.disable();
  }
  Serial.println("Moving motor: " + String(move_speed));

}

// Скорости шаговых двигателей
double carriage_speed = 0;
double hook_speed = 0;

// Функция движения крана
void MoveCarriage(const char* topic, const char* payload) {
  Serial.println("Moving carriage: " + String(payload));
  // Сохраняем скорость двигателя
  carriage_speed = String(payload).toDouble();

  // Если сработал концевик, то выключаем двигатели
  if(close_end) {
    carriage_speed = carriage_speed < 0 ? 0 : carriage_speed;
  }

  // Если сработал концевик, то выключаем двигатели
  if(far_end) {
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
  }

  // Двигаем двигатели
  CarriageStepper.setTarget(carriage_speed*10);
  CarriageStepper.setSpeed(carriage_speed);
  if(carriage_speed != 0) {
    CarriageStepper.enable();
  } else {
    CarriageStepper.disable();
  }
}

// Функция движения крюка
void MoveHook(const char* topic, const char* payload) {
  Serial.println("Moving hook: " + String(payload));
  // Сохраняем скорость двигателя
  hook_speed = String(payload).toDouble();

  // Двигаем двигатели
  CableStepper.setTarget(hook_speed*10);
  CableStepper.setSpeed(hook_speed);
  if(hook_speed != 0) {
    CableStepper.enable();
  } else {
    CableStepper.disable();
  }
}