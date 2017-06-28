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

void m1_initialize_comms(const sf_comms_instance_t * p_sf_comms);
int m1_initialize_i2c(const sf_i2c_instance_t * p_sf_i2c);
void m1_initialize_ioport(const ioport_instance_t * p_ioport);
int m1_handle_message(const char * cloud_driver_command, char * rxBuf);

#endif /* INCLUDE_M1_CLOUD_DRIVER_H_ */
