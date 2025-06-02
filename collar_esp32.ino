#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h>
#include <HardwareSerial.h>
#include "DFRobotDFPlayerMini.h"
#include <ArduinoJson.h>

const char* ssid = "S23";
const char* password = "123456789";

const char* host = "192.168.134.120";
const int port = 8080;

MPU9250 IMU;
WiFiClient client;

HardwareSerial mySerial(2);
DFRobotDFPlayerMini myDFPlayer;

#define MOTOR_PIN 5

enum BehaviorType {
  IDLE = 0,
  BEHAVIOR_1 = 1,
  BEHAVIOR_2 = 2, 
  BEHAVIOR_3 = 3,
  BEHAVIOR_4 = 4
};

BehaviorType currentBehavior = IDLE;
unsigned long lastBehaviorAction = 0;
const unsigned long BEHAVIOR_COOLDOWN = 5000;

// Глобальные переменные для контроля поведений
unsigned long behaviorCycleStart = 0;
unsigned long motorStartTime = 0;
bool motorActive = false;
bool audioPlaying = false;
bool waitingForAudio = false;
unsigned long audioStartTime = 0;
const long behaviorInterval = 6000;
unsigned long previousBehaviorMillis = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float temperature;

unsigned long previousMillis = 0;
const long interval = 50;

String commandBuffer = "";
int bracketCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== DOG BEHAVIOR + IMU SYSTEM STARTING (ESP32) ===");
  Serial.println("STATUS: Initializing components...");
  
  pinMode(MOTOR_PIN, OUTPUT);
  ledcAttach(MOTOR_PIN, 1000, 8);
  ledcWrite(MOTOR_PIN, 0);
  Serial.println("STATUS: Motor initialized on pin 5 with LEDC PWM");
  
  Serial.println("STATUS: Initializing audio player...");
  delay(2000);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  
  if (!myDFPlayer.begin(mySerial, false)) {
    Serial.println("ERROR: DFPlayer initialization failed!");
    Serial.println("ERROR: Check SD card and connections");
  } else {
    myDFPlayer.setTimeOut(500);
    myDFPlayer.volume(25);
    Serial.println("STATUS: Audio player initialized (Volume: 25/30)");
  }
  
  Wire.begin(25, 26);
  
  Serial.println("Инициализация IMU...");
  IMU.setup(0x68);
  
  Serial.println("Калибровка акселерометра и гироскопа...");
  IMU.calibrateAccelGyro();
  // Serial.println("Калибровка магнитометра...");
  // IMU.calibrateMag();
  
  Serial.print("Подключение к WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi подключен!");
  Serial.print("IP адрес: ");
  Serial.println(WiFi.localIP());
  
  connectToServer();
  
  Serial.println("=== SYSTEM READY ===");
  Serial.println("========================");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (!client.connected()) {
    Serial.println("Подключение к серверу потеряно. Переподключение...");
    connectToServer();
  }
  
  // Отправка IMU данных каждые 50 мс
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendIMUData();
  }
  
  // Проверка команд сервера (максимальная частота)
  checkServerCommands();
  
  // Выполнение поведений (максимальная частота для правильной работы)
  if (currentMillis - previousBehaviorMillis >= behaviorInterval) {
    previousBehaviorMillis = currentMillis;
    executeBehavior();
  }
  
  delay(10);
}

void connectToServer() {
  Serial.print("Подключение к серверу ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);
  
  commandBuffer = "";
  bracketCount = 0;
  
  int attempts = 0;
  while (!client.connect(host, port) && attempts < 5) {
    Serial.print(".");
    delay(1000);
    attempts++;
  }
  
  if (client.connected()) {
    Serial.println("\nПодключено к серверу!");
    client.println("{\"type\":\"connection\",\"device\":\"ESP32_IMU_BEHAVIOR\"}");
  } else {
    Serial.println("\nНе удалось подключиться к серверу!");
  }
}

void sendIMUData() {
  IMU.update();
  
  ax = IMU.getAccX();
  ay = IMU.getAccY();
  az = IMU.getAccZ();
  gx = IMU.getGyroX();
  gy = IMU.getGyroY();
  gz = IMU.getGyroZ();
  mx = IMU.getMagX();
  my = IMU.getMagY();
  mz = IMU.getMagZ();
  temperature = IMU.getTemperature();
  
  String jsonData = "{";
  jsonData += "\"timestamp\":" + String(millis()) + ",";
  jsonData += "\"accel\":{\"x\":" + String(ax, 4) + ",\"y\":" + String(ay, 4) + ",\"z\":" + String(az, 4) + "},";
  jsonData += "\"gyro\":{\"x\":" + String(gx, 4) + ",\"y\":" + String(gy, 4) + ",\"z\":" + String(gz, 4) + "},";
  jsonData += "\"mag\":{\"x\":" + String(mx, 4) + ",\"y\":" + String(my, 4) + ",\"z\":" + String(mz, 4) + "},";
  jsonData += "\"temp\":" + String(temperature, 2);
  jsonData += "}\n";
  
  client.print(jsonData);
}

void checkServerCommands() {
  if (!client.available()) return;
  
  while (client.available()) {
    char c = client.read();
    
    if (c == '{' && commandBuffer.length() == 0) {
      commandBuffer = "{";
      bracketCount = 1;
    }
    else if (bracketCount > 0) {
      commandBuffer += c;
      
      if (c == '{') {
        bracketCount++;
      } else if (c == '}') {
        bracketCount--;
        
        if (bracketCount == 0) {
          processCommand(commandBuffer);
          commandBuffer = "";
        }
      }
    }
    else if (c == '{') {
      commandBuffer = "{";
      bracketCount = 1;
    }
    
    if (commandBuffer.length() > 2048) {
      Serial.println("WARNING: Command buffer overflow, clearing");
      commandBuffer = "";
      bracketCount = 0;
    }
  }
}

void processCommand(String command) {
  command.trim();
  
  Serial.println("=== Processing command ===");
  Serial.print("Command length: ");
  Serial.println(command.length());
  Serial.print("Command: ");
  Serial.println(command);
  
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    Serial.println("=== End processing ===");
    return;
  }
  
  const char* type = doc["type"];
  if (!type) {
    Serial.println("ERROR: No 'type' field in JSON");
    return;
  }
  
  if (strcmp(type, "behavior") == 0) {
    const char* behavior = doc["behavior"];
    
    if (!behavior) {
      Serial.println("ERROR: No 'behavior' field in JSON");
      return;
    }
    
    Serial.print("Behavior command received: ");
    Serial.println(behavior);
    
    if (strcmp(behavior, "rest") == 0) {
      activateBehavior(BEHAVIOR_1);
    } else if (strcmp(behavior, "run") == 0) {
      activateBehavior(BEHAVIOR_3);
    } else if (strcmp(behavior, "walk") == 0) {
      activateBehavior(BEHAVIOR_2);
    } else if (strcmp(behavior, "crazy") == 0) {
      activateBehavior(BEHAVIOR_4);
    } else {
      Serial.println("Unknown behavior: " + String(behavior));
    }
  }
  
  Serial.println("=== End processing ===");
}

void activateBehavior(BehaviorType behavior) {
  // Останавливаем текущее поведение
  // stopAllActions();
  
  currentBehavior = behavior;
  lastBehaviorAction = millis();
  Serial.println("ACK: BEHAVIOR_" + String(behavior) + " activated");
  
  // Сбрасываем все состояния
  // resetBehaviorStates();
}

void resetBehaviorStates() {
  Serial.println("DEBUG: Resetting behavior state variables");
  behaviorCycleStart = 0;
  motorStartTime = 0;
  motorActive = false;
  audioPlaying = false;
  waitingForAudio = false;
  audioStartTime = 0;
  ledcWrite(MOTOR_PIN, 0);
  myDFPlayer.stop();
}


void executeBehavior() {
  
  switch (currentBehavior) {
    case BEHAVIOR_1:
      executeBehavior1();
      break;
    case BEHAVIOR_2: 
      executeBehavior2();
      break;
    case BEHAVIOR_3:
      executeBehavior3();
      break;
    case BEHAVIOR_4:
      executeBehavior4();
      break;
    case IDLE:
    default:
      break;
  }
}

void executeBehavior1() {
  // Ничего не делать
}

void executeBehavior2() {
  // Ничего не делать
}

  
void executeBehavior3() {
  Serial.println("ПОВЕДЕНИЕ 1: rest - аудио + мотор на 1 сек");
  static unsigned long lastSound = 0;
  

  if (millis() - lastSound >= 9000) {
    myDFPlayer.playMp3Folder(1);
    Serial.println("ACTION: Playing sound file 2 (mp3/0002.mp3) + Motor ON");
    lastSound = millis();
  }

  
  // Запускаем мотор одновременно
  ledcWrite(MOTOR_PIN, 200);
  Serial.println("Мотор запущен");
  Serial.println("Аудио должно играть");
  
  delay(1000);
  
  ledcWrite(MOTOR_PIN, 0);
  Serial.println("Мотор остановлен");
}

void executeBehavior4() {
  Serial.println("ПОВЕДЕНИЕ 1: rest - аудио + мотор на 1 сек");
  static unsigned long lastSound = 0;
  

  if (millis() - lastSound >= 9000) {
    myDFPlayer.playMp3Folder(2);
    Serial.println("ACTION: Playing sound file 2 (mp3/0002.mp3) + Motor ON");
    lastSound = millis();
  }

  
  // Запускаем мотор одновременно
  ledcWrite(MOTOR_PIN, 200);
  Serial.println("Мотор запущен");
  Serial.println("Аудио должно играть");
  
  delay(1000);
  
  ledcWrite(MOTOR_PIN, 0);
  Serial.println("Мотор остановлен");
}



void stopAllActions() {
  ledcWrite(MOTOR_PIN, 0);
  myDFPlayer.stop();
  Serial.println("ACTION: All actions stopped");
}