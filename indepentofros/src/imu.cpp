// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <inttypes.h>
#include <iomanip>
#include <vector>
// Linux headers
#include <fcntl.h>	 // Contains file controls like O_RDWR
#include <errno.h>	 // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>	 // write(), read(), close()
#include <sys/signal.h>
#include <sys/types.h>
#include "../include/imu.hpp"
using namespace std;

int IMU::OpenRS232USBModule()
{

	struct termios port_settings; // structure to store the port settings in
	this->fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (this->fd == -1)
	{
		printf("ERROR: Can't open /dev/ttyUSB0. \n");
		return -1;
	}
	else
	{
		printf("/dev/ttyUSB0 opened correctly.\n");
	};

	cfsetispeed(&port_settings, B115200); // set baud rates
	port_settings.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	port_settings.c_cc[VTIME] = 1; // inter-character timer used, in 1/10 seconds
	port_settings.c_cc[VMIN] = 0;
	tcsetattr(this->fd, TCSANOW, &port_settings); // apply the settings to the port
	return 1;
}
// void signal_handler_IO (int status)
// {
// 	tcflush(this->fd,TCIOFLUSH);

// 	uint8_t readbuff[50];

// 		int num_bytes = read(this->fd, readbuff, sizeof(readbuff) );
//         this->parse_readbuff(readbuff,num_bytes);
// 		this->printall();

// }
int IMU::OpenRS232USBModule_Interrupt(void signal_handler_IO(int))
{
	struct sigaction saio;
	struct termios port_settings; // structure to store the port settings in
	this->fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (this->fd == -1)
	{
		printf("ERROR: Can't open /dev/ttyUSB0. \n");
		return -1;
	}
	else
	{
		printf("/dev/ttyUSB0 opened correctly.\n");
	};

	saio.sa_handler = (signal_handler_IO);
	saio.sa_flags = 0;
	sigaction(SIGIO, &saio, NULL);
	//allow the process to receive SIGIO
	// cout<<getpid()<<endl;
	fcntl(this->fd, F_SETOWN, getpid());
	fcntl(this->fd, F_SETFL, FASYNC);
	cfsetispeed(&port_settings, B115200); // set baud rates
	port_settings.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	port_settings.c_cc[VTIME] = 1; // inter-character timer used, in 1/10 seconds
	port_settings.c_cc[VMIN] = 0;
	tcsetattr(this->fd, TCSANOW, &port_settings); // apply the settings to the port
	return 1;
}

float IMU::ToAngle(uint8_t *readbuff)
{
	if ((readbuff[0] >> 4) == 0)
		return (readbuff[0] & 0x0F) * 100 + (readbuff[1] >> 4) * 10 + (readbuff[1] & 0x0F) + (float)((readbuff[2] >> 4) / 10.0) + (float)((readbuff[2] & 0x0F) / 100.0);
	else if ((readbuff[0] >> 4) == 1)
		return -((readbuff[0] & 0x0F) * 100 + (readbuff[1] >> 4) * 10 + (readbuff[1] & 0x0F) + (float)((readbuff[2] >> 4) / 10.0) + (float)((readbuff[2] & 0x0F) / 100.0));
}

float IMU::ToAcc(uint8_t *readbuff)
{
	if ((readbuff[0] >> 4) == 0)
		return (readbuff[1] >> 4) + (readbuff[1] & 0x0F) / 10.0 + (readbuff[2] >> 4) / 100.0 + (readbuff[2] & 0x0F) / 1000.0;
	else if ((readbuff[0] >> 4) == 1)
		return -((readbuff[1] >> 4) + (readbuff[1] & 0x0F) / 10.0 + (readbuff[2] >> 4) / 100.0 + (readbuff[2] & 0x0F) / 1000.0);
}

int IMU::parse_readbuff(uint8_t *readbuff, uint32_t len)
{

	if (len != 0x20)
	{
		cout << "Error: Data Length Exception!" << endl;
		cout << "lenth: " << len << endl;
		for (int i = 0; i < len; i++)
			printf(" %02X", readbuff[i]);

		tcflush(this->fd, TCIOFLUSH);
		return -1;
	}
	else
	{
		if (readbuff[0] == 0x68 && readbuff[1] == 0x1F && readbuff[2] == 0x00 && readbuff[3] == 0x84)
		{
			this->roll = this->ToAngle(&readbuff[4]);
			this->pitch = this->ToAngle(&readbuff[7]);
			this->yaw = this->ToAngle(&readbuff[10]);
			this->x_acc = this->ToAcc(&readbuff[13]);
			this->y_acc = this->ToAcc(&readbuff[16]);
			this->z_acc = this->ToAcc(&readbuff[19]);
			this->roll_vel = this->ToAngle(&readbuff[22]);
			this->pitch_vel = this->ToAngle(&readbuff[25]);
			this->yaw_vel = this->ToAngle(&readbuff[28]);

			return 0;
		}
		else
		{
			cout << "Error:Invalid Read Data" << endl;
			return -2;
		}
	}
}

int IMU::parse_readbuff_test(uint8_t *readbuff, uint32_t len)
{
	if (len != 14)
	{
		cout << "Error: Data Length Exception!" << endl;
		tcflush(this->fd, TCIOFLUSH);
		return -1;
	}
	else
	{
		cout << "received number: " << len << "content" << readbuff;
		//  usleep(1000);
		//  tcflush(this->fd,TCIOFLUSH);
		//  usleep(1000);
		// for(int i = 0;i<len;i++)
		// cout<<readbuff[i];
		// cout<<"0x" << setfill('0') << setw(2)<<hex<< uppercase  <<(unsigned int)readbuff[i]<<endl;
		// return 0;
	}
}

void IMU::ContituousRead()
{
	// sleep(1);
	tcflush(this->fd, TCIOFLUSH);

	uint8_t readbuff[50];
	while (1)
	{
		int num_bytes = read(this->fd, readbuff, sizeof(readbuff));
		this->parse_readbuff(readbuff, num_bytes);
		this->printall();
		usleep(this->timeinterval);
	}
}
// void IMU::operator<<(vector<float> p)
// {
// 	cout<<"["<<p[0]<<", "<<p[1]<<", "<<p[2]<<"]"<<endl;
// }

void IMU::printall()
{
	cout << "Euler Angle:=[" << this->roll << ", " << this->pitch << ", " << this->yaw << "]" << endl;
	cout << "Acceleration:=[" << this->x_acc << ", " << this->y_acc << ", " << this->z_acc << "]" << endl;
	cout << "Angular Vel:=[" << this->roll_vel << ", " << this->pitch_vel << ", " << this->yaw_vel << "]" << endl;
}

uint8_t IMU::CRC_cal(uint8_t *buff, uint8_t len)
{
	uint8_t temp = 0;
	for (int i = 0; i < len; i++)
	{
		temp += buff[i];
	}
	return temp;
}

bool IMU::SetComFreq(string freq = "Freq_25Hz")
{
	uint8_t sendbuff[6] = {0x68, 0x05, 0x00, 0x0C};
	if (freq.compare("Freq_5Hz") == 0)
	{

		sendbuff[4] = Freq_5Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 200000;
		cout << "Wait to Set 5Hz..." << endl;
	}
	else if (freq.compare("Freq_15Hz") == 0)
	{
		sendbuff[4] = Freq_15Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 66667;
		cout << "Wait to Set 15Hz..." << endl;
	}
	else if (freq.compare("Freq_25Hz") == 0)
	{
		sendbuff[4] = Freq_25Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 40000;
		cout << "Wait to Set 25Hz..." << endl;
	}
	else if (freq.compare("Freq_35Hz") == 0)
	{
		sendbuff[4] = Freq_35Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 28572;
		cout << "Wait to Set 35Hz..." << endl;
	}
	else if (freq.compare("Freq_50Hz") == 0)
	{
		sendbuff[4] = Freq_50Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 20000;
		cout << "Wait to Set 50Hz..." << endl;
	}
	else if (freq.compare("Freq_100Hz") == 0)
	{
		sendbuff[4] = Freq_100Hz;
		sendbuff[5] = this->CRC_cal(sendbuff + 1, 4);
		this->timeinterval = 10000;
		cout << "Wait to Set 100Hz..." << endl;
	}
	else
	{
		cout << "Error: Invalid Freqency Input!" << endl;
		return false;
	}
	tcflush(this->fd, TCIOFLUSH);
	write(this->fd, sendbuff, sizeof(sendbuff));
	uint8_t readbuff[50];

	while (1)
	{

		int num_bytes = read(this->fd, readbuff, sizeof(readbuff));

		if (num_bytes < 0)
		{
			printf("Error reading: %s and retry...\r\n", strerror(errno));
			tcflush(this->fd, TCIOFLUSH);
			write(this->fd, sendbuff, sizeof(sendbuff));
			// close(this->fd);
			// sleep(1);
			// this->OpenRS232USBModule();
			// sleep(1);
		}
		else
		{
			if (readbuff[0] == 0x68 && readbuff[1] == 0x05 && readbuff[2] == 0x00 && readbuff[3] == 0x8C && readbuff[4] == 0x00)
			{
				cout << freq << " is set successfully!" << endl;
				return true;
			}
			if (readbuff[0] == 0x68 && readbuff[1] == 0x05 && readbuff[2] == 0x00 && readbuff[3] == 0x8C && readbuff[4] == 0xFF)
			{
				cout << freq << " fails!" << endl;
				return false;
			}
			uint8_t readbuff[50];
		}
		usleep(10000);
	}
}

//  bool IMU::SetComFreq_Interrupt(string freq)
//  {
// 	uint8_t sendbuff[6] = {0x68, 0x05, 0x00, 0x0C};
// 	if (freq.compare("Freq_5Hz")==0)
// 	{

// 		sendbuff[4] = Freq_5Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 200000;
// 		cout<<"Wait to Set 5Hz..."<<endl;
// 	}
// 	else if (freq.compare("Freq_15Hz")==0)
// 	{
// 		sendbuff[4] = Freq_15Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 66667;
// 		cout<<"Wait to Set 15Hz..."<<endl;
// 	}
// 	else if (freq.compare("Freq_25Hz")==0)
// 	{
// 		sendbuff[4] = Freq_25Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 40000;
//         cout<<"Wait to Set 25Hz..."<<endl;
// 	}
// 	else if (freq.compare("Freq_35Hz")==0)
// 	{
// 		sendbuff[4] = Freq_35Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 28572;
// 		cout<<"Wait to Set 35Hz..."<<endl;
// 	}
// 	else if (freq.compare("Freq_50Hz")==0)
// 	{
// 		sendbuff[4] = Freq_50Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 20000;
// 		cout<<"Wait to Set 50Hz..."<<endl;
// 	}
// 	else if (freq.compare("Freq_100Hz")==0)
// 	{
// 		sendbuff[4] = Freq_100Hz;
// 		sendbuff[5] = this->CRC_cal(sendbuff+1,4);
// 		this->timeinterval = 10000;
// 		cout<<"Wait to Set 100Hz..."<<endl;
// 	}
// 	else
// 	{
// 		cout<<"Error: Invalid Freqency Input!"<<endl;
// 		return false;
// 	}
// 	tcflush(this->fd,TCIOFLUSH);
// 	write(this->fd,sendbuff,sizeof(sendbuff));

//  }
