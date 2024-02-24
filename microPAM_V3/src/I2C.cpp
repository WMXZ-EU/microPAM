/* microPAM 
 * Copyright (c) 2023/2024, Walter Zimmer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include "I2C.h"

    i2c_class::i2c_class(TwoWire *wire) 
    {   this->wire = wire;
        wire->begin();
        delay(100);
    }

    i2c_class::i2c_class(TwoWire *wire, uint32_t speed) 
    {   this->wire = wire;
        wire->begin();
        delay(100);
        wire->setClock(speed);
    }

    i2c_class::i2c_class(TwoWire *wire, uint32_t speed, uint8_t scl, uint8_t sda) 
    {   this->wire = wire;
        wire->begin();
        delay(100);
        wire->setClock(speed);
        wire->setSCL(scl);
        wire->setSDA(sda);
    }

    uint8_t i2c_class::exist(uint8_t addr)
    {
        wire->beginTransmission(addr);
        return (wire->endTransmission()==0);
    }

    uint8_t i2c_class::read(uint8_t addr, uint8_t reg) 
    { 
        unsigned int val;
        wire->beginTransmission(addr);
        wire->write(reg);
        if (wire->endTransmission(false) != 0) return 0;
        if (wire->requestFrom((int)addr, 1) < 1) return 0;
        val = wire->read();
        return val;
    }
    
    uint8_t i2c_class::write(uint8_t addr, uint8_t reg) 
    { 
        wire->beginTransmission(addr);
        wire->write(reg);
        return (wire->endTransmission() == 0) ;
    }

    uint8_t i2c_class::write(uint8_t addr, uint8_t reg, uint8_t val) 
    { 
        wire->beginTransmission(addr);
        wire->write(reg);
        wire->write(val);
        return (wire->endTransmission() == 0) ;
    }

    uint8_t *i2c_class::readData( uint8_t addr, uint8_t reg, uint8_t *data, uint8_t ndat, uint16_t dt)
    {
            wire->beginTransmission(addr);
            wire->write(reg);
            if (wire->endTransmission(false) != 0) return 0;
            if (dt>0) delay(dt);
            if (wire->requestFrom((int)addr, (int)ndat) < 1) return 0;
            for(int ii=0;ii<ndat;ii++) data[ii] = wire->read();
            return data;
    }    


#if 0
void test_wire0(void)
{
        Wire.begin();
        delay(100);
//        Wire.setSDA(18);
//        Wire.setSCL(19);
        Wire.setClock(100'000);

    for(int ii=0;ii<127;ii++)
    {
        Wire.beginTransmission(ii);
        delay(100);
        uint8_t error = Wire.endTransmission();
        if(error) 
            Serial.printf("I2C not found %x\n",ii);  
        else 
            Serial.printf("I2C found %x\n",ii);      
    }
    while(1);  
}

void test_wire1(void)
{
        Wire1.begin();
        delay(100);
        Wire1.setSDA(17);
        Wire1.setSCL(16);
        Wire1.setClock(100'000);

    for(int ii=0;ii<127;ii++)
    {
        Wire1.beginTransmission(ii);
        delay(100);
        uint8_t error = Wire1.endTransmission();
        if(error) 
            Serial.printf("I2C not found %x\n",ii);  
        else 
            Serial.printf("I2C found %x\n",ii);      
    }
    while(1);  
}
#endif