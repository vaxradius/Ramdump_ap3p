#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include  <limits.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "ios_fifo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ramdump_task.h"

#define START_ADDR 0x10000000
#define TOTAL_LENGTH (30*1024)
//*****************************************************************************
//
// Ramdump task handle.
//
//*****************************************************************************
TaskHandle_t Ramdump_task_handle;


//*****************************************************************************
//
// BUTTON0 pin configuration settings.
//
//*****************************************************************************
//
// Set up the configuration for BUTTON0.
//
const am_hal_gpio_pincfg_t g_button0 =
{
    .uFuncSel = 3,
    .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

//
// Max SRAM for Apollo is 64KB.
// Use an offset of 1MB as the value to cause the fault.
//
#define ILLEGAL_SRAM_ADDR   (0x10000000 + (1024 * 1024))

//*****************************************************************************
//
//! @brief Function to cause a hard fault.
//!
//! @return None.
//
//*****************************************************************************
void
force_fault(void)
{
    uint32_t *pCauseFault;
    volatile uint32_t uVal;

    pCauseFault = (uint32_t*)ILLEGAL_SRAM_ADDR;
    uVal = *pCauseFault;

    //
    // Use the variable uVal in order to avoid a warning from some compilers.
    // However, the fault will prevent us from getting here.
    //
    pCauseFault = (uint32_t*)uVal;
}


void BTN_detetion_init(void)
{
	    //
    // Configure the button pin.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_button0);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_BUTTON0));

    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_BUTTON0));

	//
    // Enable GPIO interrupts to the NVIC.
    //
    NVIC_EnableIRQ(GPIO_IRQn);
	NVIC_SetPriority(GPIO_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

}

//*****************************************************************************
//
// GPIO ISR
//
//*****************************************************************************
void
am_gpio_isr(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	//
    // Delay for debounce.
    //
    am_util_delay_ms(200);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_BUTTON0));

	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(Ramdump_task_handle, BTN_INT_NOTIFICATION, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RamdumpTask(void *pvParameters)
{
	uint32_t numWritten = 0;
	uint32_t ui32UsedSpace = 0;
	uint32_t ui32LeftSpace = 0;
	uint32_t ui32TotalSent = 0;
	uint32_t ulNotifiedValue;

	Ramdump_task_handle = xTaskGetCurrentTaskHandle();

	BTN_detetion_init();

	/*Dummy Patterns in SRAM for checking in Host*/
	for(int i=1; i<256; i++)
		*((uint8_t *)(START_ADDR+TOTAL_LENGTH-i))=i;
    //
    // Loop forever.
    //
    while(1)
    {

		xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
				ULONG_MAX, /* Reset the notification value to 0 on exit. */
				&ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
				portMAX_DELAY );  /* Block indefinitely. */

		if(ulNotifiedValue == IOS_INT_NOTIFICATION)
		{
			while(ui32TotalSent < TOTAL_LENGTH)
			{
				am_hal_ios_fifo_space_left(g_pIOSHandle, &ui32LeftSpace);

				if(ui32LeftSpace >= 512)
				{
					am_hal_ios_fifo_write(g_pIOSHandle, (uint8_t *)(START_ADDR+ui32TotalSent), (ui32TotalSent+512<=TOTAL_LENGTH? 512:(512-((ui32TotalSent+512)-TOTAL_LENGTH))), &numWritten);
					ui32TotalSent += numWritten;
				}
		        // If we were Idle - need to inform Host if there is new data
		        if (g_iosState == AM_IOSTEST_SLAVE_STATE_NODATA)
		        {
		            am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
		            if (ui32UsedSpace)
		            {
		                g_iosState = AM_IOSTEST_SLAVE_STATE_DATA;
		                inform_host();
		            }
		        }
			}
		}
		else if(ulNotifiedValue == BTN_INT_NOTIFICATION)
		{
			am_util_stdio_printf("force_fault\n");
			force_fault();
		}
    }
}

