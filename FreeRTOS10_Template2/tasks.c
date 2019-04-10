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

void qamSendByte(uint8_t byte)
{
	uint8_t qamLUT[4] = {0,8,16,24};
	uint8_t i = 0;
	
	for (i=0; i!=4; i ++)
	{
		qamSymbols[i] = qamLUT[(byte >> (2 * i))  & 0x03];
	}
	qamSymbolCount = 3;	
}

void vTask_DMAHandler(void *pvParameters) 
{
	//Do things and Stuff with DMA!
	
	xDMAProcessEventGroup = xEventGroupCreate();
	EventBits_t uxBits;
	
	PORTF.DIRSET = PIN1_bm; /*LED1*/
	PORTF.DIRSET = PIN2_bm; /*LED2*/
	
	
	qamSendByte(0xE4);
	vSetDMA_LUT_Offset();
	while(1)
	{
		uxBits = xEventGroupWaitBits(
		xDMAProcessEventGroup,   /* The event group being tested. */
		DMA_EVT_GRP_QAM_FINISHED, /* The bits within the event group to wait for. */
		pdTRUE,        /* Bits should be cleared before returning. */
		pdFALSE,       /* Don't wait for both bits, either bit will do. */
		portMAX_DELAY );/* Wait a maximum for either bit to be set. */
			
		//Check Event bits
		if(uxBits & DMA_EVT_GRP_QAM_FINISHED)
		{
			//Do stuff with BufferA
			//buffer_a ....
			
			//Debug Output
			PORTF.OUT = (PORTF.OUT & (0xFF - 0x02));
			PORTF.OUT |= 0x04;
			
			qamSendByte(0xCC);
			vStartQAMTransfer();			
		}

	}
	
}