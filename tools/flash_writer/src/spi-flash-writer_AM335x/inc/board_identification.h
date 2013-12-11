enum boards
{
	BASE_BOARD,
	GP_BOARD,
	IA_BOARD,
	IP_BOARD
};

#define DB_ADDR            0x51
#define BB_ADDR            0x50
#define CPLD_ADDR          0x35

#define BOARD_NAME_SIZE    8

#define EEPROM_AMM335X_HEADER       0xEE3355AA
#define EEPROM_BB_BOARD_SIGNATURE   "A33155BB"
#define EEPROM_GP_BOARD_SIGNATURE   "A335GPBD"
#define EEPROM_IA_BOARD_SIGNATURE   "A335IAMC"
#define EEPROM_IP_BOARD_SIGNATURE   "A335IPPH"

#define PROFILE_0 0x1 << 0
#define PROFILE_1 0x1 << 1
#define PROFILE_2 0x1 << 2
#define PROFILE_3 0x1 << 3
#define PROFILE_4 0x1 << 4
#define PROFILE_5 0x1 << 5
#define PROFILE_6 0x1 << 6
#define PROFILE_7 0x1 << 7

struct eeprom_board_signature {
	Uint32 header;
	Uint8  board_name[8];
	Uint32 version;
	Uint8 configopt[32];
};

struct CPLD_Header {
	Uint32 Device_header;
	Uint8  Device_id[8];
	Uint8  Device_rev[4];
	Uint8  CFG_Reg[2];
};

extern Int32 i2c_daughter_card_detection(void);
extern Int32 profile_identification(void);
extern Int32 i2c_probe (Int32 chip);
extern Int32 i2c_read_byte (Int8 devaddr, Uint16 regoffset, Uint8 alen, Uint8 * buffer, Uint8 len);
extern void i2c_init(int speed, int slaveaddr);
