//
//  HMC5883L.cpp
//  
//
//  Created by Tobias Ebsen on 04/05/18.
//
//

#include "hmc5883l.h"

#if defined(CORE_TEENSY) && ( defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))       // Teensy 3.X
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

using namespace HMC5883L;

uint8_t Sensor::regs[13];
bool Sensor::dataRead;
uint16_t Sensor::gainCount[] = {1370, 1090, 820, 660, 440, 390, 330, 230};
float sensorRange[] = {0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1};

void Sensor::setup(int drdyPin) {
    Wire.begin();
    
    offsetX = 0;
    offsetY = 0;
    offsetZ = 0;
    
    for (int r=0; r<sizeof(regs); r++) {
        regs[r] = (uint8_t)0;
    }
    dataRead = false;

    if (drdyPin != -1) {
        pinMode(drdyPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(drdyPin), dataReady, FALLING);
    }
}

void Sensor::setConfigA(AVERAGE sampleAvg, RATE dataRate, BIAS bias) {
    regs[CRA] |= (sampleAvg & 0b11) << 5;
    regs[CRA] |= (dataRate & 0b111) << 2;
    regs[CRA] |= (bias & 0b11) << 0;
    write(CRA);
}

void Sensor::setConfigB(uint8_t gain) {
    regs[CRB] = (gain & 0b111) << 5;
    write(CRB);
}

void Sensor::setMode(MODE mode, bool highSpeedI2C) {
    regs[MR] = (highSpeedI2C & 0b1) << 7;
    regs[MR] |= (mode & 0b11);
    write(MR);
}

short Sensor::getX() {
    dataRead = false;
    return ((regs[DXRA] << 8) | regs[DXRB]) - offsetX;
}

short Sensor::getY() {
    dataRead = false;
    return ((regs[DYRA] << 8) | regs[DYRB]) - offsetY;
}

short Sensor::getZ() {
    dataRead = false;
    return ((regs[DZRA] << 8) | regs[DZRB]) - offsetZ;
}

float Sensor::getHeading() {
    return atan2(getY(), getX()) * 180.f * PI;
}

float Sensor::getStrength() {
    short x = getX();
    short y = getY();
    return sqrt(x*x + y*y);
}

void Sensor::beginCalibration() {
    minX = 2000;
    minY = 2000;
    minZ = 2000;
    maxX = -2000;
    maxY = -2000;
    maxZ = -2000;
    offsetX = 0;
    offsetY = 0;
    offsetZ = 0;
}

void Sensor::updateCalibration(short x, short y, short z) {
    minX = min(minX, x);
    minY = min(minY, y);
    minZ = min(minZ, z);
    maxX = max(maxX, x);
    maxY = max(maxY, y);
    maxZ = max(maxZ, z);
}

void Sensor::updateCalibration() {
    updateCalibration(getX(), getY(), getZ());
}

void Sensor::endCalibration() {
    offsetX = minX + (maxX - minX) / 2;
    offsetY = minY + (maxY - minY) / 2;
    offsetZ = minZ + (maxZ - minZ) / 2;
}

bool Sensor::withinLimits(short xyz) {
    uint8_t g = (regs[CRB] >> 5) & 0b111;
    short low = 243 * gainCount[g] / 390;
    short high = 575 * gainCount[g] / 390;
    return xyz > low && xyz < high;
}

uint8_t Sensor::getRegister(REGISTER reg) {
    return regs[reg];
}

void Sensor::write(REGISTER reg, uint8_t data) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write((uint8_t)reg);
    Wire.write(data);
    Wire.endTransmission();
}

void Sensor::write(REGISTER reg) {
    write(reg, regs[reg]);
}

void Sensor::read(REGISTER reg) {
    
    Wire.beginTransmission(I2C_ADDR);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    
    Wire.requestFrom(I2C_ADDR, 1);
    while (Wire.available() < 1);
    regs[reg] = Wire.read();
}

void Sensor::readAll() {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write((uint8_t)0);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR, (size_t)sizeof(regs));
    while (Wire.available() < sizeof(regs));
    for (int r=0; r<sizeof(regs); r++) {
        regs[r] = Wire.read();
    }
}

bool Sensor::isDataRead() {
    return dataRead;
}

void Sensor::dataReady() {

    Wire.requestFrom(I2C_ADDR, (uint8_t)6);

    if (Wire.available() == 6) {

        for (int r=DXRA; r<=DXRA+6; r++) {
            regs[r] = Wire.read();
        }
        dataRead = true;
        
        Wire.beginTransmission(I2C_ADDR);
        Wire.write((uint8_t)REGISTER::DXRA);
        Wire.endTransmission();
    }
    else
        dataRead = false;
}