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
