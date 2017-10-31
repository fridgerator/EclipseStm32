/* Includes ------------------------------------------------------------------*/
#include "eeprom_flash.h"
#include "stm32f3xx_hal.h"

/** @addtogroup STM32F3xx_HAL_Examples
 * @{
 */

/** @addtogroup FLASH_WriteProtection
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	FAILED = 0, PASSED = !FAILED
} TestStatus;
/* Private define ------------------------------------------------------------*/

/* Uncomment this line to Enable Write Protection */
//#define WRITE_PROTECTION_ENABLE
/* Uncomment this line to Disable Write Protection */
#define WRITE_PROTECTION_DISABLE

#define FLASH_USER_START_ADDR       ADDR_FLASH_PAGE_24   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR         ADDR_FLASH_PAGE_28   /* End @ of user Flash area */
#define FLASH_PAGE_TO_BE_PROTECTED (OB_WRP_PAGES24TO25 | OB_WRP_PAGES26TO27)

#define DATA_32                     ((uint32_t)0x12345678)

/* Check the status of the switches */
/* Enable by default the disable protection */
#if !defined(WRITE_PROTECTION_ENABLE)&&!defined(WRITE_PROTECTION_DISABLE)
#define WRITE_PROTECTION_DISABLE
#endif /* !WRITE_PROTECTION_ENABLE && !WRITE_PROTECTION_DISABLE */

/* Both switches cannot be enabled in the same time */
#if defined(WRITE_PROTECTION_ENABLE)&&defined(WRITE_PROTECTION_DISABLE)
#error "Switches WRITE_PROTECTION_ENABLE & WRITE_PROTECTION_DISABLE cannot be enabled in the time!"
#endif /* WRITE_PROTECTION_ENABLE && WRITE_PROTECTION_DISABLE */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Address = 0;
uint32_t PageError = 0;
__IO TestStatus MemoryProgramStatus = PASSED;
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
/*Variable used to handle the Options Bytes*/
static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
void writeFlash(void) {
	/* STM32F3xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user
	 can eventually implement his proper time base source (a general purpose
	 timer for example or other time source), keeping in mind that Time base
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	 handled in milliseconds basis.
	 - Set NVIC Group Priority to 4
	 - Low Level Initialization
	 */
	//HAL_Init();
	/* Configure the system clock to 72 MHz */
	//SystemClock_Config();
	/* Initialize LED1, LED2 and LED3 */
	//BSP_LED_Init(LED1);
	//BSP_LED_Init(LED2);
	//BSP_LED_Init(LED3);
	/* Initialize test status */
	MemoryProgramStatus = PASSED;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	HAL_FLASH_OB_Unlock();

	/* Get pages write protection status ****************************************/
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

#ifdef WRITE_PROTECTION_DISABLE
	/* Check if desired pages are already write protected ***********************/
	if ((OptionsBytesStruct.WRPPage & FLASH_PAGE_TO_BE_PROTECTED) != FLASH_PAGE_TO_BE_PROTECTED) {
		/* Restore write protected pages */
		OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
		OptionsBytesStruct.WRPState = OB_WRPSTATE_DISABLE;
		OptionsBytesStruct.WRPPage = FLASH_PAGE_TO_BE_PROTECTED;
		if (HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK) {
			/* Error occurred while options bytes programming. **********************/
			while (1) {
				__WFI();
			}
		}

		/* Generate System Reset to load the new option byte values ***************/
		HAL_FLASH_OB_Launch();
	}
#elif defined WRITE_PROTECTION_ENABLE
	/* Check if desired pages are not yet write protected ***********************/
	if(((~OptionsBytesStruct.WRPPage) & FLASH_PAGE_TO_BE_PROTECTED )!= FLASH_PAGE_TO_BE_PROTECTED)
	{
		/* Enable the pages write protection **************************************/
		OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
		OptionsBytesStruct.WRPState = OB_WRPSTATE_ENABLE;
		OptionsBytesStruct.WRPPage = FLASH_PAGE_TO_BE_PROTECTED;
		if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
		{
			/* Error occurred while options bytes programming. **********************/
			while (1)
			{
				BSP_LED_On(LED3);
			}
		}

		/* Generate System Reset to load the new option byte values ***************/
		HAL_FLASH_OB_Launch();
	}
#endif /* WRITE_PROTECTION_DISABLE */

	/* Lock the Options Bytes *************************************************/
	HAL_FLASH_OB_Lock();

	/* The selected pages are not write protected *******************************/
	if ((OptionsBytesStruct.WRPPage & FLASH_PAGE_TO_BE_PROTECTED) != 0x00) {
		/* Fill EraseInit structure************************************************/
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
		EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
			/*
			 Error occurred while page erase.
			 User can add here some code to deal with this error.
			 PageError will contain the faulty page and then to know the code error on this page,
			 user can call function 'HAL_FLASH_GetError()'
			 */
			while (1) {
				__WFI();
			}
		}
		CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));

		/* FLASH Word program of DATA_32 at addresses defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR */
		Address = FLASH_USER_START_ADDR;
		while (Address < FLASH_USER_END_ADDR) {
			HAL_StatusTypeDef result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32);
			if (result == HAL_OK) {
				Address = Address + 4;
			} else {
				/* Error occurred while writing data in Flash memory.
				 User can add here some code to deal with this error */
				while (1) {
					__WFI();
				}
			}
			CLEAR_BIT(FLASH->CR, (FLASH_CR_PG));
		}

		/* Check the correctness of written data */
		Address = FLASH_USER_START_ADDR;

		while (Address < FLASH_USER_END_ADDR) {
			if ((*(__IO uint32_t*) Address) != DATA_32) {
				MemoryProgramStatus = FAILED;
			}
			Address += 4;
		}
	} else {
		/* The desired pages are write protected */
		/* Check that it is not allowed to write in this page */
		Address = FLASH_USER_START_ADDR;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) != HAL_OK) {
			/* Error returned during programmation. */
			/* Check that WRPERR flag is well set */
			if (HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP) {
				MemoryProgramStatus = FAILED;
			} else {
				/* Another error occurred.
				 User can add here some code to deal with this error */
				while (1) {
					__WFI();
				}
			}
		} else {
			/* Write operation is successful. Should not occur
			 User can add here some code to deal with this error */
			while (1) {
				__WFI();
			}
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	/*Check if there is an issue to program data*/
	if (MemoryProgramStatus == PASSED) {
		/* No error detected. Switch on LED1*/
		__WFI();
	} else {
		/* Error detected. Switch on LED2*/
		__WFI();
	}

}

uint32_t readFlash(uint32_t offset) {
	uint32_t Address = FLASH_USER_START_ADDR;
	uint32_t value = (*(__IO uint32_t*) (Address + offset * 4));
	return value;
}

