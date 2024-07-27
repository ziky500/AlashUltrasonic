#ifndef AlashUltrasonic_H
#define AlashUltrasonic_H

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

class AlashUltrasonic {
  public:
    // Конструкторы для GPIO, I2C, UART и 1-Wire
    AlashUltrasonic(uint8_t triggerPin, uint8_t echoPin);
    AlashUltrasonic(uint8_t i2cAddress);
    AlashUltrasonic(uint8_t rxPin, uint8_t txPin, bool useUART);
    AlashUltrasonic(uint8_t oneWirePin, bool useOneWire);

    // Инициализация
    void begin();
    
    // Измерение расстояния
    float getDistance();
  
  private:
    // Переменные для GPIO
    uint8_t _triggerPin;
    uint8_t _echoPin;
    
    // Переменные для I2C
    uint8_t _i2cAddress;
    
    // Переменные для UART
    SoftwareSerial* _serial;
    uint8_t _rxPin;
    uint8_t _txPin;
    
    // Переменные для 1-Wire
    uint8_t _oneWirePin;
    bool _useOneWire;

    // Общие переменные
    bool _useI2C;
    bool _useUART;
    unsigned long _lastReadTime;
    
    // Внутренние функции для измерения расстояния
    float getDistanceGPIO();
    float getDistanceI2C();
    float getDistanceUART();
    float getDistanceOneWire();
    float calculateDistance(long duration);
};

#endif
