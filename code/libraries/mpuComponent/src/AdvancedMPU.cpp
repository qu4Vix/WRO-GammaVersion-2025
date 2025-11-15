/***************************************************
 * 
 * THE MAIN REPOSITORY CAN BE FOUND AT https://github.com/qu4Vix/WRO-GammaVersion-2025
 * 
 * This code is under a GPL-3.0 license. More information can be found in the License file
 * in the repository.
 * 
****************************************************/

/***************************************************
 * 
 * AdvancedMPU.h - Library for controling a MPU9250 and getting the overall angle with Arduino Framework.
 * 
 * Created by the Gamma Version Team, 2025
 * 
****************************************************/

#include "AdvancedMPU.h"

MPU::MPU() {
    
}

void MPU::BeginWire(byte pinSDA, byte pinSCL, uint32_t freq) {
    _pinSDA = pinSDA;
    _pinSCL = pinSCL;
    while (!Wire1.begin(int(_pinSDA), int(_pinSCL), freq)) {
        // Wire Connection failed
        delay(200);
        digitalWrite(_pinLED, !digitalRead(_pinLED));
    }
}

void MPU::Setup() {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
    //setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    while(!_mpu.setup(0x68, setting, Wire1)) {  // change to your own address
        //Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(300);
        digitalWrite(_pinLED, !digitalRead(_pinLED));
    }

    //loadCalibration();
}

void MPU::WorkOffset() {
    uint16_t num = 0;
    uint32_t offset_time = micros();
    double tot = 0;
    while (num < 1000) {
        if (_mpu.update()) {
            num++;
            tot += _mpu.getGyroZ();
        }
        //delay(5);
    }
    _offset = tot / num; //1000;//((micros() - offset_time) / double(1000000));
    Serial.println("Offset = " + String(_offset*1000000));
}

/*void MPU::UpdateAngle() {
    if (_mpu.update()) {
        unsigned long sampleDuration = micros() - _prev_us_angle;
        float gyroZ = _mpu.getGyroZ();
        _prev_us_angle = micros();
        _angle += ((gyroZ - _offset) * sampleDuration / double(1000000));
        //Serial.println(_angle);
    }
}*/

void MPU::UpdateAngle() {
    if (_mpu.update()) {

        unsigned long now = micros();
        float dt = (now - _prev_us_angle) * 1e-6f; // segundos
        _prev_us_angle = now;

        float gyroZ = _mpu.getGyroZ() - _offset;

        _angle += gyroZ * dt;
    }
}

double MPU::GetAngle() {
    return _angle;
}

void MPU::MeasureFirstMicros() {
    if (!_firstMicrosRead) {
        _prev_us_angle = micros();
        _firstMicrosRead = true;
    }
}

void MPU::AddAngle(double _addedAngle) {
    _angle += _addedAngle;
}

void MPU::SetDebugLedPin(uint8_t _pin) {
    _pinLED = _pin;
}