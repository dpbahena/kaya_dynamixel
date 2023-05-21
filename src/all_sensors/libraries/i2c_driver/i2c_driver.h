
/* AsClinic System / I2C Driver for Linux */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>


/* Class definitions */

class I2C_Driver
{
	/* Enum */

	enum class I2C_State : int
	{
		closed = 0,
		open = 1,
	};


	/* Variables */

private:
	static const int MAX_DEVICE_NAME_LENGTH = 20;
	char m_device_name[20];
	I2C_Driver::I2C_State m_state;
	int m_file_descriptor;





	/* Constructor */
public:
	I2C_Driver(const char * device_name);





	/* Get and Set */
public:
	const char * get_device_name();
	int get_state();
	int get_file_descriptor();





/* Functions */

private:
	

public:
	bool open_i2c_device();
	bool close_i2c_device();

	bool write_data(uint8_t address, uint16_t num_write_bytes, uint8_t * write_data_array);
	bool write_data_then_read_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array, uint16_t num_read_btyes, uint8_t * read_data_array);

}; // END OF CLASS DEFINITION





#endif // I2C_DRIVER_H