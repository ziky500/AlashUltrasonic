#include "AlashUltrasonic.h"

// Конструктор для GPIO
AlashUltrasonic::AlashUltrasonic(uint8_t triggerPin, uint8_t echoPin) {
  _triggerPin = triggerPin;
  _echoPin = echoPin;
  _connectionType = GPIO_MODE;
  _lastReadTime = 0;
}

// Конструктор для I2C
AlashUltrasonic::AlashUltrasonic(uint8_t i2cAddress) {
  _i2cAddress = i2cAddress;
  _connectionType = I2C_MODE;
  _lastReadTime = 0;
}

// Конструктор для UART
AlashUltrasonic::AlashUltrasonic(uint8_t rxPin, uint8_t txPin, ConnectionType type) {
  _rxPin = rxPin;
  _txPin = txPin;
  _connectionType = type;
  _lastReadTime = 0;
  _serial = new SoftwareSerial(_rxPin, _txPin);
}

// Конструктор для 1-Wire
AlashUltrasonic::AlashUltrasonic(uint8_t oneWirePin, ConnectionType type) {
  _oneWirePin = oneWirePin;
  _connectionType = type;
  _lastReadTime = 0;
}

// Инициализация
void AlashUltrasonic::begin() {
  switch (_connectionType) {
    case I2C_MODE:
      Wire.begin();
      break;
    case UART_MODE:
      _serial->begin(9600);
      break;
    case ONEWIRE_MODE:
      pinMode(_oneWirePin, OUTPUT);
      break;
    case GPIO_MODE:
    default:
      pinMode(_triggerPin, OUTPUT);
      pinMode(_echoPin, INPUT);
      break;
  }
}

// Измерение расстояния
float AlashUltrasonic::getDistance() {
  if (millis() - _lastReadTime < 30) {
    delay(millis() - _lastReadTime);
  }
  _lastReadTime = millis();

  switch (_connectionType) {
    case I2C_MODE:
      return getDistanceI2C();
    case UART_MODE:
      return getDistanceUART();
    case ONEWIRE_MODE:
      return getDistanceOneWire();
    case GPIO_MODE:
    default:
      return getDistanceGPIO();
  }
}

// Измерение расстояния через GPIO
float AlashUltrasonic::getDistanceGPIO() {
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(_triggerPin, LOW);

  long duration = pulseIn(_echoPin, HIGH);
  return calculateDistance(duration);
}

// Измерение расстояния через I2C (robust ver.)
float AlashUltrasonic::getDistanceI2C() {
  // send "start measurement" and check ack  2  return -1 on i2c error
  if (Wire.beginTransmission(_i2cAddress), Wire.write((uint8_t)0x01), Wire.endTransmission() != 0) {
    return -1.0f;  // i2c error
  }

  // local params so other modes remain untouched  2  total wait ~10*15 ms = 150 ms
  const uint8_t tries = 10;        // number of polls while waiting for data
  const uint8_t poll_ms = 15;      // delay between polls
  const float min_cm = 2.0f;       // plausible lower bound (too close -> treat as timeout)
  const float max_cm = 450.0f;     // plausible upper bound (reject wild spikes)

  // try primary 24-bit frame: micrometers (big-endian b0 b1 b2)  2  convert to cm
  for (uint8_t i = 0; i < tries; i++) {
    delay(poll_ms);  // wait for conversion
    int n = Wire.requestFrom((int)_i2cAddress, 3);
    if (n == 3) {
      uint32_t b0 = Wire.read();
      uint32_t b1 = Wire.read();
      uint32_t b2 = Wire.read();
      uint32_t um = (b0 << 16) | (b1 << 8) | b2;  // 24-bit micrometers
      if (um == 0) continue;  // not ready yet
      float cm = (float)um / 10000.0f;  // µm -> cm
      if (cm >= min_cm && cm <= max_cm) return cm;  // valid reading
      return -1.0f;  // out of plausible range -> treat as timeout
    }
  }

  // fallback: some firmwares return 16-bit millimeters (big-endian)  2  convert to cm
  if (Wire.requestFrom((int)_i2cAddress, 2) == 2) {
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    uint16_t mm = (uint16_t)((hi << 8) | lo);
    if (mm == 0) return -1.0f;  // not ready
    float cm = (float)mm / 10.0f;  // mm -> cm
    if (cm >= min_cm && cm <= max_cm) return cm;  // valid
  }

  return -1.0f;  // timeout / invalid frame
}

// Измерение расстояния через UART
float AlashUltrasonic::getDistanceUART() {
  _serial->flush();
  _serial->write(0xA0);
  
  delay(150);
  
  byte response[3];
  for (byte i = 0; _serial->available() && (i < 3); i++) {
    response[i] = _serial->read();
  }
  
  float micrometers = ((response[0] * 65536UL) + (response[1] * 256UL) + response[2]);
  return micrometers / 1000000.0 * 100.0;
}

// Измерение расстояния через 1-Wire
float AlashUltrasonic::getDistanceOneWire() {
  pinMode(_oneWirePin, OUTPUT);
  digitalWrite(_oneWirePin, HIGH);
  delayMicroseconds(500);
  digitalWrite(_oneWirePin, LOW);

  pinMode(_oneWirePin, INPUT);
  long duration = pulseIn(_oneWirePin, HIGH);
  float distance = duration * 340.0 / 2.0 / 10000.0;
  return distance;
}

// Преобразование времени в расстояние
float AlashUltrasonic::calculateDistance(long duration) {
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

