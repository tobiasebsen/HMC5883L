//
//  HMC5883L.h
//  
//
//  Created by Tobias Ebsen on 04/05/18.
//
//

#ifndef HMC5883L_h
#define HMC5883L_h

#include <Arduino.h>

namespace HMC5883L {

#define I2C_ADDR    (uint8_t)(0x1E)

    typedef enum {
        CRA,
        CRB,
        MR,
        DXRA,DXRB,
        DZRA,DZRB,
        DYRA,DYRB,
        SR,
        IRA,IRB,IRC
    } REGISTER;
    
    typedef enum {
        AVG_1,
        AVG_2,
        AVG_4,
        AVG_8
    } AVERAGE;

    typedef enum {
        RATE_0_75HZ,
        RATE_1_5HZ,
        RATE_3HZ,
        RATE_7_5HZ,
        RATE_15HZ,
        RATE_30HZ,
        RATE_75HZ
    } RATE;
    
    typedef enum {
        NORMAL,
        POSITIVE,
        NEGATIVE
    } BIAS;

    typedef enum {
        CONTINUOUS,
        SINGLE,
        IDLE,
        IDLE2
    } MODE;

    class Sensor {
    public:
        void setup(int drdyPin = -1);
        
        void setConfigA(AVERAGE sampleAvg = AVG_8, RATE dataRate = RATE_15HZ, BIAS bias = NORMAL);
        void setConfigB(uint8_t gain = 1);
        void setMode(MODE mode = SINGLE, bool highSpeedI2C = false);

        void setRegister(REGISTER reg, uint8_t data);
        uint8_t getRegister(REGISTER reg);
        
        short getX();
        short getY();
        short getZ();
        float getHeading();
        float getStrength();
        
        void beginCalibration();
        void updateCalibration();
        void updateCalibration(short x, short y, short z);
        void endCalibration();
        
        bool withinLimits(short xyx);
        
        void write(REGISTER reg, uint8_t data);
        void write(REGISTER reg);
        void read(REGISTER reg);
        void readAll();
        void seek(REGISTER reg);
        
        bool isDataRead();

        static void dataReady();
        
        short offsetX, offsetY, offsetZ;

    private:
        
        short minX, maxX;
        short minY, maxY;
        short minZ, maxZ;
        
        static uint8_t regs[13];
        static bool dataRead;
        static uint16_t gainCount[8];
        static float fieldRange[8];
    };
    
}

#endif /* HMC5883L_h */
