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

//*****************************************************************************
//
// Ramdump task handle.
//
//*****************************************************************************
TaskHandle_t Ramdump_task_handle;

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
	uint32_t ulNotifiedValue;

	uint8_t u8data[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

	Ramdump_task_handle = xTaskGetCurrentTaskHandle();

    //
    // Loop forever.
    //
    while(1)
    {

		xTaskNotifyWait( 0x00,      /* Don't clear any notification bits on entry. */
				ULONG_MAX, /* Reset the notification value to 0 on exit. */
				&ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
				portMAX_DELAY );  /* Block indefinitely. */
		

		//am_hal_gpio_out_bit_clear(8);
		//am_hal_gpio_out_bit_set(8);
		
		am_hal_ios_fifo_write(g_pIOSHandle, (uint8_t *)u8data, 16, &numWritten);

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

