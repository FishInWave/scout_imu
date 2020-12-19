
#include "../include/imu.hpp"
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <iostream>
#include <stdio.h>
#include  <iomanip>

using namespace std;
IMU imu;
void signal_handler_IO (int status);
int main()
{
	int i =1;
	imu.OpenRS232USBModule_Interrupt(signal_handler_IO);
  
	// imu.OpenRS232USBModule();
	// uint8_t buff_test[]={0x0D,0x00,0x84,0x10,0x50,0x23,0x00,0x23,0x04,0x01,0x80,0x00};
	
	// cout<<"0x" << setfill('0') << setw(2)<<hex<< uppercase  <<(unsigned int)imu.CRC_cal(buff_test,sizeof(buff_test))<<endl;
	// imu.printall();
	// imu.SetComFreq("Freq_100Hz");
	// imu.ContituousRead();
	while(1);


}
void signal_handler_IO (int status)
{
	// cout<<status<<endl;

	tcflush(imu.fd,TCIOFLUSH);
	uint8_t readbuff[50]={0};

	int num_bytes = read(imu.fd, readbuff, sizeof(readbuff) );
	// cout<<"received number: "<<num_bytes<<" content: "<<readbuff<<endl;
	imu.parse_readbuff(readbuff,num_bytes);
	// imu.parse_readbuff_test(readbuff,num_bytes);
	//  tcflush(imu.fd,TCIOFLUSH);
	// imu.printall();
	for(int i = 0;i<num_bytes;i++)
	{
		cout<<"0x" << setfill('0') << setw(2)<<hex<< uppercase  <<(unsigned int)readbuff[i] << " ";
	}

}