/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "i2c.h"
#include "tistdtypes.h"
#include "board_identification.h"
#include "string.h"


int i2c_daughter_card_detection(void) {
	struct eeprom_board_signature board_signature;
	int addr = DB_ADDR,daughter = 1;
	char board_name[BOARD_NAME_SIZE + 1];

	i2c_init(OMAP_I2C_STANDARD, OMAP_I2C_SLAVE);

	// Start searching Daughter cards.
	// if daughter card found returns.
	daughter  = i2c_probe(addr);
	if(!daughter){
		// Daughter crad detected

		i2c_read_byte (addr, 0x0, 2, (Uint8 *)&board_signature,
				sizeof(struct eeprom_board_signature));
		if(board_signature.header != EEPROM_AMM335X_HEADER)
			return -1;

		memcpy(board_name, board_signature.board_name,
				BOARD_NAME_SIZE);
		board_name[BOARD_NAME_SIZE] = '\0';

		if(strcmp(board_name, EEPROM_GP_BOARD_SIGNATURE))
				return GP_BOARD;
		else if(strcmp(board_name, EEPROM_IA_BOARD_SIGNATURE))
				return IA_BOARD;
		else if(strcmp(board_name, EEPROM_IP_BOARD_SIGNATURE))
				return IP_BOARD;
		else
			return -1;
	} else {
		addr = BB_ADDR;
		daughter  = i2c_probe(addr);

		i2c_read_byte (addr, 0x0, 2, (Uint8 *)&board_signature,
				sizeof(struct eeprom_board_signature));
		if(board_signature.header != EEPROM_AMM335X_HEADER)
			return -1;

		memcpy(board_name, board_signature.board_name,
				BOARD_NAME_SIZE);
		board_name[BOARD_NAME_SIZE] = '\0';

		if (strcmp(board_name, EEPROM_BB_BOARD_SIGNATURE))
			return BASE_BOARD;
		else
			return -1;
	}
}

int profile_identification(void){
	struct CPLD_Header cpld_header;
	Uint8  profile = 0, cpld;

	cpld = i2c_probe(CPLD_ADDR);
	if(cpld){
		exit(-1);
	} else {
		i2c_read_byte(CPLD_ADDR, 0x10, 1, (Uint8 *) cpld_header.CFG_Reg,
				sizeof(cpld_header.CFG_Reg));
		return (cpld_header.CFG_Reg[0] & 0x07);
	}
	return 0;
}
