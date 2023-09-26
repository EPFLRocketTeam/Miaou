/**
 * @file 		comunicator.c
 * @brief 		serial msv2 communicator
 *
 * @date 		12.08.2022
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	comunicator Comunicator
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include "comunicator.h"


/**********************
 *	CONSTANTS
 **********************/


/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/




/**********************
 *	VARIABLES
 **********************/




/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


/**
 * @brief 	initialize communicator device
 * @detail
 */
util_error_t comunicator_init(	comunicator_t * com,
								UART_HandleTypeDef * uart,
								void (*cb)(uint8_t, uint16_t, uint8_t *)) {
	msv2_init(&com->msv2);
	com->cb = cb;
	com->uart = uart;
	return ER_SUCCESS;
}

util_error_t comunicator_recv(comunicator_t * com, uint8_t c) {
	MSV2_ERROR_t ret = msv2_decode_fragment(&com->msv2, c);
	if(ret == MSV2_SUCCESS) {
		com->cb(com->msv2.rx.opcode, com->msv2.rx.data_len*2, com->msv2.rx.data);
	}
	return ER_SUCCESS;
}

util_error_t comunicator_send(	comunicator_t * com,
								uint8_t opcode,
								uint16_t len,
								uint8_t * data) {

	uint16_t length = msv2_create_frame(&com->msv2, opcode, len/2, data);
	HAL_UART_Transmit_IT(com->uart, com->msv2.tx.data, length);
	return ER_SUCCESS;
}



/** @} */


/* END */
