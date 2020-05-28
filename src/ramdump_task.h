#ifndef RAMDUMP_TASK_H
#define RAMDUMP_TASK_H

#include "FreeRTOS.h"
#include "task.h"

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

