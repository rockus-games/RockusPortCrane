#include <Arduino.h>
#include <PicoMQTT.h>
#include <GyverStepper2.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Пины, к которым подключены концевые выключатели
#define END_CARRIAGE_CLOSE 27
#define END_CARRIAGE_FAR 32
#define END_CARRIAGE_HOOK 33

// Пины, к которым подключены двигатели колёс
#define MOTOR_F 23
#define MOTOR_B 14

// Пины, к которым подключены шаговые двигатели
#define STEP_CARRIAGE_1 19
#define STEP_CARRIAGE_2 18
#define STEP_CARRIAGE_3 5
#define STEP_CARRIAGE_4 17

// Шаговый двигатель каретки
GStepper2<STEPPER4WIRE> CarriageStepper(2038, STEP_CARRIAGE_1, STEP_CARRIAGE_3, STEP_CARRIAGE_2, STEP_CARRIAGE_4);

#define STEP_TURN_1 16
#define STEP_TURN_2 4
#define STEP_TURN_3 2
#define STEP_TURN_4 15

// Шаговый двигатель поворота
GStepper2<STEPPER4WIRE> TurnStepper(2038, STEP_TURN_1, STEP_TURN_3, STEP_TURN_2, STEP_TURN_4);

#define STEP_CABLE_1 13
#define STEP_CABLE_2 12
#define STEP_CABLE_3 25
#define STEP_CABLE_4 26

// Шаговый двигатель крюка
GStepper2<STEPPER4WIRE> CableStepper(2038, STEP_CABLE_1, STEP_CABLE_3, STEP_CABLE_2, STEP_CABLE_4);

// Пины, к которым подключен гироскоп
#define GYRO_SDA 21
#define GYRO_SCL 22

// Стандартное положение гироскопа
#define GYRO_ZERO_VAL -92.5

// Минимальное и максимальное отклонение гироскопа
#define GYRO_MIN_VAL 3
#define GYRO_MAX_VAL 4

// Максимальная скорость и ускорение двигателей
const int maxSpeed = 1600;
const int acceleration = 50;

// MQTT-сервер для обмена данными с приложением
PicoMQTT::Server mqtt;

// Настройки подключения к WiFi
const char* ssid     = "Rockus_PortCrane";
const char* password = "Kochegar";

// Функции для обработки сигналов с приложения
void MoveMotor(const char* topic, const char* payload);
void Turn(const char* topic, const char* payload);
void MoveCarriage(const char* topic, const char* payload);
void MoveHook(const char* topic, const char* payload);

// Скорости двигателей
double motor_speed = 0;

// Состояние нажатия концевиков
bool far_end = false;
bool close_end = false;
bool hook_end = false;

// Состояние гироскопа
bool gyro_end = false;

// Переменные для гироскопа
MPU6050 mpu;
uint8_t fifoBuffer[45]; 

void setup() {
  // Разрешаем отладочный вывод
  Serial.begin(115200);

  // Настройка пинов на вывод сигнала
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

  // Настройка пинов на ввод сигнала
  pinMode(END_CARRIAGE_CLOSE, INPUT_PULLUP);
  pinMode(END_CARRIAGE_FAR, INPUT_PULLUP);
  pinMode(END_CARRIAGE_HOOK, INPUT_PULLUP);

  // Выставляем максимальную скорость и ускорение двигателей
  CarriageStepper.setMaxSpeed(maxSpeed);
  CarriageStepper.setAcceleration(acceleration);
  TurnStepper.setMaxSpeed(maxSpeed);
  TurnStepper.setAcceleration(acceleration);
  CableStepper.setMaxSpeed(maxSpeed);
  CableStepper.setAcceleration(acceleration);

  // Задаём начальную скорость двигателям
  CarriageStepper.setSpeed(0);
  TurnStepper.setSpeed(0);
  CableStepper.setSpeed(0);

  // Настройка гироскопа
  byte error, address;
  Wire.begin();

  delay(1000);

  // Сканируем устройства, чтобы найти гироскоп
  Serial.println("Scanning...");
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
  
    // Если нашли гироскоп, то инициализируем его
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      mpu = MPU6050(address);
    }
  }

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  // Запуск MQTT-сервера и подключение к WiFi
  Serial.print("Запуск точки доступа");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Адрес: ");
  Serial.println(IP);

  // Подписываемся на MQTT-топики
  mqtt.subscribe("move_carriage", MoveCarriage);
  mqtt.subscribe("turn", Turn);
  mqtt.subscribe("move_motor", MoveMotor);
  mqtt.subscribe("move_hook", MoveHook);

  mqtt.begin();
}

// Переменные для гироскопа
float ypr[3];
uint32_t tmr;

// Функция получения угла наклона крана
void getAngle() {
  // Опрашиваем 2 раза в секунду
  if (millis() - tmr >= 500) { 
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      
      // Считываем ускорение и угловую скорость
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      // Эта функция рассчитывает текущий угол наклона крана
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Обновляем таймер
      tmr = millis();
      // Serial.println(ypr[1] * 57.3);
    }
  }
}


void loop() {
  // Обработка MQTT
  mqtt.loop();

  // Получаем угол наклона крана
  getAngle();

  // Если угол наклона крана больше 4 градусов, то выключаем двигатели
  if(abs(abs(ypr[1] * 57.3) - abs(GYRO_ZERO_VAL)) > GYRO_MAX_VAL) {
    if(!gyro_end) {
      TurnStepper.setSpeed(0);
      TurnStepper.disable();

      CarriageStepper.setSpeed(0);
      CarriageStepper.disable();

      CableStepper.setSpeed(0);
      CableStepper.disable();

      Serial.println("Gyro angle error!");
    }
    gyro_end = true;
  }
  // Если угол наклона крана меньше 3 градусов, то выключаем двигатели
  else if(abs(abs(ypr[1] * 57.3) - abs(GYRO_ZERO_VAL)) < GYRO_MIN_VAL) {
    gyro_end = false;
  }

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

  // Проверка концевика на крюке крана
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

// Функция движения двигателей
void MoveMotor(const char* topic, const char* payload) {
  Serial.println("Moving motor: " + String(payload));
  // Сохраняем скорость двигателя
  motor_speed = String(payload).toDouble();

  // Если угол наклона крана больше 4 градусов, то выключаем двигатели
  if(gyro_end) {
    motor_speed = 0;
  }

  // Двигаем двигатели
  if (motor_speed > 0) {
    // Если скорость больше нуля, то двигаем вперед
    analogWrite(MOTOR_F, motor_speed);
    digitalWrite(MOTOR_B, LOW);
  } else if(motor_speed < 0) {
    // Если скорость меньше нуля, то двигаем назад
    analogWrite(MOTOR_F, -motor_speed);
    digitalWrite(MOTOR_B, HIGH);
  } else {
    // Если скорость равна нулю, то остановим двигатели
    analogWrite(MOTOR_F, LOW);
    digitalWrite(MOTOR_B, LOW);
  }
}

// Скорости шаговых двигателей
double carriage_speed = 0;
double hook_speed = 0;
double turn_speed = 0;

// Функция поворота крана
void Turn(const char* topic, const char* payload) {
  Serial.println("Turning: " + String(payload));
  // Сохраняем скорость поворота
  turn_speed = String(payload).toDouble();

  // Если угол наклона крана больше 4 градусов, то выключаем двигатели
  if(gyro_end) {
    turn_speed = 0;
  }

  // Поворачиваем кран
  TurnStepper.setTarget(turn_speed*10);
  TurnStepper.setSpeed(turn_speed);
  if(turn_speed != 0) {
    TurnStepper.enable();
  } else {
    TurnStepper.disable();
  }
}

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
  if(hook_end) {
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
    hook_speed = hook_speed > 0 ? 0 : hook_speed;
  }

  // Если сработал концевик или угол наклона крана больше 4 градусов, то выключаем двигатели
  if(far_end || gyro_end) {
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

  // Если сработал концевик или угол наклона крана больше 4 градусов, то выключаем двигатели
  if(hook_end || gyro_end) {
    hook_speed = hook_speed > 0 ? 0 : hook_speed;
    carriage_speed = carriage_speed > 0 ? 0 : carriage_speed;
  }

  // Двигаем двигатели
  CableStepper.setTarget(hook_speed*10);
  CableStepper.setSpeed(hook_speed);
  if(hook_speed != 0) {
    CableStepper.enable();
  } else {
    CableStepper.disable();
  }
}