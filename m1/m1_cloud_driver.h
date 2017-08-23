/*
 *  Copyright (c) 2016 Medium One, Inc
 *  www.mediumone.com
 *
 *  Portions of this work may be based on third party contributions.
 *  Medium One, Inc reserves copyrights to this work whose
 *  license terms are defined under a separate Software License
 *  Agreement (SLA).  Re-distribution of any or all of this work,
 *  in source or binary form, is prohibited unless authorized by
 *  Medium One, Inc under SLA.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef INCLUDE_M1_CLOUD_DRIVER_H_
#define INCLUDE_M1_CLOUD_DRIVER_H_

#include "sf_comms_api.h"
#include "sf_i2c_api.h"

enum {
	M1_SUCCESS_NO_DATA = 0,
	M1_SUCCESS_DATA,
	M1_ERROR_BAD_HEADER,
	M1_ERROR_BUFFER_OVERFLOW,
	M1_ERROR_INVALID_PORT_OR_COMMAND,
	M1_ERROR_BAD_COMMAND,
	M1_ERROR_PORT,
	M1_ERROR_JSON_SERIALIZE,
	M1_ERROR_UNSUPPORTED_COMMAND,
};

/*
 * Description:
 * 		Registers a pointer to a sf_comms_instance_t for later use when driven by cloud commands. 
 *              sf_comms_instance_t::p_api::open must have been previously called.
 * Parameters:
 * 		p_sf_comms: 		Communications Framework Driver
 */
void m1_initialize_comms(const sf_comms_instance_t * p_sf_comms);

/*
 * Description:
 * 		Registers a pointer to a sf_i2c_instance_t for later use when driven by cloud commands. 
 *              sf_i2c_instance_t::p_api::open must have been previously called.
 * Parameters:
 * 		p_sf_i2c: 		I2C Framework Device Driver
 * Returns:
 *              0 on success
 *              -1 if too many I2C devices have been registered. The current limit is 10 devices
 */
int m1_initialize_i2c(const sf_i2c_instance_t * p_sf_i2c);

/*
 * Description:
 * 		Registers a pointer to a ioport_instance_t for later use when driven by cloud commands. 
 *              ioport_instance_t::p_api::open must have been previously called.
 * Parameters:
 * 		p_ioport: 		IO Port Driver
 */
void m1_initialize_ioport(const ioport_instance_t * p_ioport);

/*
 * Description:
 * 		Processes a cloud driver command and performs any instructions contained in it. Cloud 
 *              driver commands are strings with the following formats:
 *              
 *              Header - Each command starts with a header:
 *              <PORT>;<COMMAND>;
 *
 *              Valid values are as follows:
 *
 *              <PORT> - 0: IO Port
 *                       1: Communications Framework
 *                       3: I2C Framework Device
 *              <COMMAND> - 0: Read (only supported with PORT 1)
 *                          2: Read & Write (only supported with PORT 3)
 *                          3: GPIO Read (only supported with PORT 0)
 *                          4: GPIO Write (only supported with PORT 0)
 *
 *              Body - The body of the command depends on the type of command
 *              0: Read - <READ_LENGTH>;<TAG_NAME>;
 *                        READ_LENGTH: unsigned integer value
 *                        TAG_NAME:    read data will be sent in an event to M1 with tag TAG_NAME. 
 *                                     must be less than 600 characters long. no "." or ";" or other 
 *                                     restricted characters
 *              2: Read & Write - <READ_LENGTH>;<WRITE_LENGTH>;<TAG_NAME>;<DATA>
 *                        READ_LENGTH:  unsigned integer value
 *                        WRITE_LENGTH: unsigned integer value
 *                        TAG_NAME:     read data will be sent in an event to M1 with tag TAG_NAME. 
 *                                      must be less than 600 characters long. no "." or ";" or other 
 *                                      restricted characters
 *                        DATA:         binary data to be written
 *              3: GPIO Read - <GPIO_PIN>;<LEVEL>;
 *                        GPIO_PIN: unsigned integer value, mapped to IO pins as follows
 *                                  0 - P02_05, 4 - P03_04, 5 - P03_13, 6 - P03_14, 7 - P03_15
 *                        LEVEL:    unsigned integer value, level to set pin to. 0 for low, 1 for high
 *              4: GPIO Read - <GPIO_PIN>;<TAG_NAME>;
 *                        GPIO_PIN: unsigned integer value, mapped to IO pins as follows
 *                                  0 - P02_05, 4 - P03_04, 5 - P03_13, 6 - P03_14, 7 - P03_15
 *                        TAG_NAME:    read data will be sent in an event to M1 with tag TAG_NAME. 
 *                                     must be less than 600 characters long. no "." or ";" or other 
 *                                     restricted characters
 *
 *              Note: For I2C commands, the first byte of <DATA> always specifies the I2C device address.
 *              Examples:
 *                        I2C read 7 bytes from i2c device 0x5a, upload as tag "air_quality":
 *                                     "3;2;7;1;air_quality;\x5a"
 *
 *                        I2C writes 1 byte of 0xff and reads 7 bytes from i2c device 0x5a, upload as tag "air_quality":
 *                                     "3;2;7;2;air_quality;\x5a\xff"
 * Parameters:
 * 		cloud_driver_command: 	pointer to the string containing the cloud driver command
 * 		rxBuf: 		        if a read command was performed and was successful, this buffer
 *                                      holds the data that was read
 * Returns:
 *      M1_SUCCESS_NO_DATA
 *	M1_SUCCESS_DATA
 *	M1_ERROR_BAD_HEADER
 *	M1_ERROR_BUFFER_OVERFLOW
 *	M1_ERROR_INVALID_PORT_OR_COMMAND
 *	M1_ERROR_BAD_COMMAND
 *	M1_ERROR_PORT
 *	M1_ERROR_JSON_SERIALIZE
 *	M1_ERROR_UNSUPPORTED_COMMAND
 */
int m1_handle_message(const char * cloud_driver_command, char * rxBuf);

#endif /* INCLUDE_M1_CLOUD_DRIVER_H_ */
