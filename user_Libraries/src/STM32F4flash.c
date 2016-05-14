#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "main.h"

bool bUseEEPROM = 0;
uint32_t startAddress = 0x080E0000;//starting from 896KB, the beginning of last sector
                                                                                                                                  //0x8010000 is 64KB
								  //0x8040000 is 256KB
								  //0x8080000 is 512KB
								  //0x80C0000 is 768KB
								  
// /* Base address of the Flash sectors */
// #define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
// #define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */								  
								  
void writeFlash(void)
{
	uint32_t i, j;
	//DI;//not to execute time consuming code in interrupt for controller
	setLeftPwm(0);
	setRightPwm(0);
	FLASH_Unlock();
	/* Clear All pending flags */
	FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
	
	//VoltageRange_1        ((uint8_t)0x00)  /*!< Device operating range: 1.8V to 2.1V */
	//VoltageRange_2        ((uint8_t)0x01)  /*!<Device operating range: 2.1V to 2.7V */
	//VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */
	//VoltageRange_4        ((uint8_t)0x03)  /*!<Device operating range: 2.7V to 3.6V + External Vpp */	
	
	
	for(i = 0; i < SIZE; i++)
		for(j = 0; j < SIZE; j++)              
			FLASH_ProgramHalfWord((startAddress + (i*SIZE+j)*4), cell[i][j]);

	FLASH_Lock();
}

void readFlash(void)
{
	u32 i, j;	
	for(i = 0; i < SIZE; i++)
		for(j = 0; j < SIZE; j++)       
			cell[i][j] = *(int16_t *)(startAddress + (i*SIZE+j)*4);
}
