#ifndef IMU_H
#define IMU_H

#include <inttypes.h>
#include <vector>
#include <stdio.h>
#include <string>
#include <sys/signal.h>
#include <sys/types.h>
#include <fcntl.h>
using namespace std;

#define Freq_5Hz 0x01
#define Freq_15Hz 0x02
#define Freq_25Hz 0x03
#define Freq_35Hz 0x04
#define Freq_50Hz 0x05
#define Freq_100Hz 0x06

#define BaudRate_9600 02
#define BaudRate_19200 03
#define BaudRate_38400 04
#define BaudRate_115200 05 //default
class IMU
{
    public:
        
        IMU():
        roll(0),
        pitch(0),
        yaw(0),
        x_acc(0),
        y_acc(0),
        z_acc(0),
        roll_vel(0),
        pitch_vel(0),
        yaw_vel(0), 
        timeinterval(40000){
            // EulerAngle.resize(3);
            // Acceleration.resize(3);
            // AngularVel.resize(3);
        }
        int fd;
        virtual ~IMU() {}
        int parse_readbuff(uint8_t* readbuff, uint32_t len);
        int parse_readbuff_test(uint8_t* readbuff, uint32_t len);
       // void operator <<(vector<float> p);
        uint8_t CRC_cal(uint8_t *buff, uint8_t len);
        void printall();
        int OpenRS232USBModule();
        int OpenRS232USBModule_Interrupt(void signal_handler_IO (int));
        bool SetComFreq(string freq);
        // bool SetComFreq_Interrupt(string freq);
        float ToAngle(uint8_t* readbuff);
        float ToAcc(uint8_t* readbuff);
        void ContituousRead();
        // void signal_handler_IO (int status); /*definition of the signal handler*/
    private:
        // vector<float> EulerAngle; //roll pitch yaw
        // vector<float> Acceleration;//x y z
        // vector<float> AngularVel;//
        float roll, pitch, yaw, x_acc, y_acc, z_acc, roll_vel, pitch_vel, yaw_vel;
        uint32_t timeinterval;
        


};
// void signal_handler_IO (int status);
#endif

