#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h> 

#define Acc 59;
#define Gyro 67;

int spi_fd;
int mpu;
struc mpu{
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
}


void setupSPI(void){
	// load SPI driver
	spi_fd = open("/dev/spidev0.0", O_RDWR);
	if(spi_fd<0){
		printf("Can't Load SPI driver\n");
		exit(1);
	}
	// setup SPI operational mode
	int mode = SPI_MODE_0;
	if(ioctl(spi_fd,SPI_IOC_WR_MODE, &mode)<0){
		printf("Can't Set SPI Mode\n");
		exit(1);
	}
	int lsb_first = 0;
	if(ioctl(spi_fd,SPI_IOC_WR_LSB_FIRST, &lsb_first)<0){
		printf("Can't Set MSB first\n");
		exit(1);
	}
	int bpw = 8;
	if(ioctl(spi_fd,SPI_IOC_WR_BITS_PER_WORD, &bpw)<0){
		printf("Can't Set Bit per Word\n");
		exit(1);
	}
	int max_Speed = 10000000;
	if(ioctl(spi_fd,SPI_IOC_WR_MAX_SPEED_HZ, &max_Speed)<0){
		printf("Can't Set Max Speed\n");
		exit(1);
	}
}

void setupI2C(void){
	// load driver
	mpu = open("/dev/i2c-1", O_RDWR);
	// Set slave address
	int addr = 0x68;
	if(ioctl(mpu, I2C_SLAVE, addr)){
		printf("Can't Load I2C driver");
		exit(1);
	}
}

void sendData(uint8_t addr, uint8_t value){
	struc spi_ioc_transfer spi;
	memset(spi, 0, sizeof(spi));
	uint8_t data[2];
	data[0] = addr; data[1] = value;
	
	spi.txbuf 		= (unsigned long) data;
	spi.len 		= 2;
	spi.bits_per_word = 8;
	spi.speed_hz 	= 10000000;
	
	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi)<0){
		printf("Can't Send data via SPI\n");
		exit(1);
	}
}

void Init_max7219(void){
	sendData(0x0C, 0);
	...
	...	
}
void Init_mpu6050(void){
	i2c_smbus_write_word_data(mpu, 25, 4);
	i2c_smbus_write_word_data(mpu, 26, 2);
	i2c_smbus_write_word_data(mpu, 27, 0x10);
	i2c_smbus_write_word_data(mpu, 28, 0x18);
	i2c_smbus_write_word_data(mpu, 107, 1);
}

struct read_Acc(){
	struct mpu Acc;
	int16_t temp = i2c_smbus_read_word_data(mpu, Acc);
	Acc.Ax = temp/2048.0;
	temp = i2c_smbus_read_word_data(mpu, Acc+2);
	Acc.Ay = temp/2048.0;
	temp = i2c_smbus_read_word_data(mpu, Acc+4);
	Acc.Az = temp/2048.0;
	return Acc;
}

int main(void){
	setupSPI();
	setupI2C();
	Init_max7219();
	Init_mpu6050();
	
	


	return 0;
}	