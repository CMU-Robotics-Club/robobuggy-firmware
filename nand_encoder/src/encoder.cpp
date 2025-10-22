#include <Arduino.h>
#include "encoder.h"

#define I2C_ADDRESS 0x36
#define RAW_REGISTER 0x0C
#define FIX_REGISTER 0x0E
#define QUANTITY 2
#define STOP true
#define BUFFER_SIZE 100
#define RAW_SCALE 4096.0
#define ANG_SCALE 360.0


namespace encoder {
    angles_t int_ang;
    angle_time ang_buffer[BUFFER_SIZE];
    size_t buf_index;

    void init() {
        Wire.begin();
        buf_index = 0;
        int_ang.angle = 0;
        int_ang.raw_angle = 0;
        for(size_t i=0;i<BUFFER_SIZE;++i) {
            ang_buffer->a.angle = 0;
            ang_buffer->a.raw_angle = 0;
            ang_buffer->time = 0;
        }
    }

    void write_buffer() {
        ang_buffer[buf_index].a = int_ang;
        ang_buffer[buf_index].time = millis();
        ++buf_index;
        if(buf_index >= BUFFER_SIZE) buf_index = 0;
    }

    angle_time *read_buffer(size_t index_) {
        size_t index = (index_ + BUFFER_SIZE) % BUFFER_SIZE;
        return &ang_buffer[index];
    }

    void get_pos() {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(RAW_REGISTER); // write register address
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP);
        int_ang.raw_angle = 0;
        int_ang.raw_angle |= (Wire.read()&0x0F)<<8; // RAW_ANGLE[11:8]
        int_ang.raw_angle |= Wire.read();           // RAW_ANGLE[7:0]
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(FIX_REGISTER); // write register address
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP);
        int_ang.angle = 0;
        int_ang.angle |= (Wire.read()&0x0F)<<8;     // ANGLE[11:8]
        int_ang.angle |= Wire.read();               // ANGLE[7:0]
        write_buffer();
    }

    void get_ang(angles_t *a) {
        if(a!=NULL) {
            memcpy((void *)a, (const void *)&int_ang, sizeof(int_ang));
        }
    }

    long double scale(int raw) {
        double s = (double)raw / RAW_SCALE;
        return s * ANG_SCALE;
    }

    double get_speed() {
        angle_time *cur = read_buffer(buf_index-1);
        angle_time *prev = read_buffer(buf_index);
        double dist = scale(cur->a.angle)-scale(prev->a.angle);
        double time = (double)(cur->time - prev->time) / 1000.0;
        return dist / time;
    }
}