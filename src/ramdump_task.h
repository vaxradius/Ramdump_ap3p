#ifndef RAMDUMP_TASK_H
#define RAMDUMP_TASK_H

#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    NONE_NOTIFICATION    = 0,
    IOS_INT_NOTIFICATION   = 1,
    BTN_INT_NOTIFICATION      = 2,
} RAMDUMP_TASK_NOTIFICATION_E;

//*****************************************************************************
//
// Audio task handle.
//
//*****************************************************************************
extern TaskHandle_t Ramdump_task_handle;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void RamdumpTask(void *pvParameters);

#endif // RAMDUMP_TASK_H

