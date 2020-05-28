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
}

