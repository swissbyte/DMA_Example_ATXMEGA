/*
 * tasks.c
 *
 * Created: 05.04.2019 08:50:00
 *  Author: Claudio Hediger
 */ 

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "tasks.h"
#include "dma.h"

EventGroupHandle_t xDMAProcessEventGroup;


void vTask_DMAHandler(void *pvParameters) 
{
	//Do things and Stuff with DMA!
	
	xDMAProcessEventGroup = xEventGroupCreate();
	EventBits_t uxBits;
	
	PORTF.DIRSET = PIN1_bm; /*LED1*/
	PORTF.DIRSET = PIN2_bm; /*LED2*/
	
	while(1)
	{
		uxBits = xEventGroupWaitBits(
		xDMAProcessEventGroup,   /* The event group being tested. */
		DMA_EVT_GRP_BufferA | DMA_EVT_GRP_BufferB, /* The bits within the event group to wait for. */
		pdTRUE,        /* Bits should be cleared before returning. */
		pdFALSE,       /* Don't wait for both bits, either bit will do. */
		portMAX_DELAY );/* Wait a maximum for either bit to be set. */
			
		//Check Event bits
		if(uxBits & DMA_EVT_GRP_BufferA)
		{
			//Do stuff with BufferA
			//buffer_a ....
			
			//Debug Output
			PORTF.OUT = (PORTF.OUT & (0xFF - 0x02));
			PORTF.OUT |= 0x04;
		}
		else //When it was not DMA_EVT_GRP_BufferA, then it was probably B. Since we only use two bits!
		{
			//Do stuff with BufferB
			//buffer_b ....
			
			//Debug Output
			PORTF.OUT = (PORTF.OUT & (0xFF - 0x04));
			PORTF.OUT |= 0x02;
		}

	}
	
}