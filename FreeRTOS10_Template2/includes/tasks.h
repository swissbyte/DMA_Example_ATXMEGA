/*
 * tasks.h
 *
 * Created: 05.04.2019 09:05:34
 *  Author: Claudio Hediger
 */ 


#ifndef TASKS_H_
#define TASKS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

void vTask_DMAHandler(void *pvParameters);

#define DMA_EVT_GRP_BufferA  ( 1 << 0 )
#define DMA_EVT_GRP_BufferB  ( 1 << 1 )

EventGroupHandle_t xDMAProcessEventGroup;


#endif /* TASKS_H_ */