
#include "stdint.h"
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"
#include "dma.h"
#include "tasks.h"

#include "FreeRTOS.h"
#include "task.h"

volatile uint8_t buffer_a[2048];
volatile uint8_t buffer_b[2048];

//Initialisiert den ADC im Freerunning Mode
void sys_InitADC(void)
{
	
	// Free Running mode: On
	// Conversion mode: Unsigned, 12Bit
	ADCB.CTRLB=(ADCB.CTRLB & (~(ADC_CONMODE_bm | ADC_FREERUN_bm | ADC_RESOLUTION_gm))) | ADC_RESOLUTION_12BIT_gc | ADC_FREERUN_bm;

	// Reference 1V and configuration of prescaler to 256
	ADCB.PRESCALER=(ADCB.PRESCALER & (~ADC_PRESCALER_gm)) | ADC_PRESCALER_DIV256_gc;
	ADCB.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm;;			//internal 1V

	// Read and save the ADC offset using channel 0
	ADCB.CH0.CTRL=(ADCB.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc ;	// PORTB:0
	
	ADCB.CH1.CTRL=(ADCB.CH1.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc ;	// PORTB:1	
	
	ADCB.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;
	ADCB.CH2.MUXCTRL = ADC_CH_MUXINT_TEMP_gc;  //Temp Mux
	
	ADCB.EVCTRL = ADC_SWEEP_012_gc;
	
	// Enable the ADC in order to read the offset
	ADCB.CTRLA|=ADC_ENABLE_bm;
}

void vInitDMA()
{
	uint8_t i = 0;
	
	//ADC8 PB0 Input
	PORTB.DIRCLR = PIN0_bm;
	PORTB.DIRCLR = PIN1_bm;
	
	sys_InitADC();

	// set TCC1 to 11024Hz overflow, actually 11019.2838Hz (-0.052% error)
	TCC1.CTRLA = 0; // stop if running
	TCC1.CNT = 0;
	TCC1.PER = 0x0FFF;

	EVSYS.CH0MUX = EVSYS_CHMUX_TCC1_OVF_gc; // trigger on timer overflow


	// reset DMA controller
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);
	
	DMA.CTRL			= DMA_CH_ENABLE_bm | DMA_DBUFMODE_CH01_gc; // double buffered with channels 0 and 1
	
	//Bei Double Buffering wird automatisch aus Channel 0 und 1 ein "Pair" gebildet. 
	//Siehe dazu AVR1304.P8
	
	// channel 0
	// **** TODO: reset dma channels
	DMA.CH0.REPCNT		= 0;
	DMA.CH0.CTRLA		= DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm; // ADC result is 2 byte 12 bit word
	DMA.CH0.CTRLB		= 0x1;
	DMA.CH0.ADDRCTRL	= DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc | // reload source after every burst
	DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc; // reload dest after every transaction
	DMA.CH0.TRIGSRC		= DMA_CH_TRIGSRC_TCC1_OVF_gc;
	DMA.CH0.TRFCNT		= 2048; // always the number of bytes, even if burst length > 1
	DMA.CH0.DESTADDR0	= (( (uint16_t) buffer_a) >> 0) & 0xFF;
	DMA.CH0.DESTADDR1	= (( (uint16_t) buffer_a) >> 8) & 0xFF;
	DMA.CH0.DESTADDR2	= 0;
	DMA.CH0.SRCADDR0	= (( (uint16_t) &ADCB.CH1.RES) >> 0) & 0xFF;
	DMA.CH0.SRCADDR1	= (( (uint16_t) &ADCB.CH1.RES) >> 8) & 0xFF;
	DMA.CH0.SRCADDR2	= 0;

	// channel 1
	DMA.CH1.REPCNT		= 0;
	DMA.CH1.CTRLA		= DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm; // ADC result is 2 byte 12 bit word
	DMA.CH1.CTRLB		= 0x1;
	DMA.CH1.ADDRCTRL	= DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc | // reload source after every burst
	DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc; // reload dest after every transaction
	DMA.CH1.TRIGSRC		= DMA_CH_TRIGSRC_TCC1_OVF_gc;
	DMA.CH1.TRFCNT		= 2048;
	DMA.CH1.DESTADDR0	= (( (uint16_t) buffer_b) >> 0) & 0xFF;
	DMA.CH1.DESTADDR1	= (( (uint16_t) buffer_b) >> 8) & 0xFF;
	DMA.CH1.DESTADDR2	= 0;
	DMA.CH1.SRCADDR0	= (( (uint16_t) &ADCB.CH1.RES) >> 0) & 0xFF;
	DMA.CH1.SRCADDR1	= (( (uint16_t) &ADCB.CH1.RES) >> 8) & 0xFF;
	DMA.CH1.SRCADDR2	= 0;

	DMA.CH0.CTRLA		|= DMA_CH_ENABLE_bm;
	TCC1.CTRLA			= TC_CLKSEL_DIV1024_gc; // start timer, and in turn ADC

		//Example for polling
// 		while (!(DMA.INTFLAGS & DMA_CH0TRNIF_bm));
// 		DMA.INTFLAGS = DMA_CH0TRNIF_bm;

}

ISR(DMA_CH0_vect)
{
	
	//Interrupt quittieren
	DMA.CH0.CTRLB |= 0x10;
	TCC1.INTFLAGS |= 0x01;
	//PORTF.OUTTGL = 0x01;

	
	
		
	BaseType_t xHigherPriorityTaskWoken, xResult;

	/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
	xHigherPriorityTaskWoken = pdFALSE;

	/* Set bit 0 and bit 4 in xEventGroup. */
	xResult = xEventGroupSetBitsFromISR(
								xDMAProcessEventGroup,   /* The event group being updated. */
								DMA_EVT_GRP_BufferA, /* The bits being set. */
								&xHigherPriorityTaskWoken );

	/* Was the message posted successfully? */
	if( xResult != pdFAIL )
	{
		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		switch should be requested.  The macro used is port specific and will
		be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		the documentation page for the port being used. */
		//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}	
}

ISR(DMA_CH1_vect)
{
	//Interrupt quittieren
	DMA.CH1.CTRLB |= 0x10;
	TCC1.INTFLAGS |= 0x01;

		
	BaseType_t xHigherPriorityTaskWoken, xResult;

	/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
	xHigherPriorityTaskWoken = pdFALSE;

	/* Set bit 0 and bit 4 in xEventGroup. */
	xResult = xEventGroupSetBitsFromISR(
								xDMAProcessEventGroup,   /* The event group being updated. */
								DMA_EVT_GRP_BufferB, /* The bits being set. */
								&xHigherPriorityTaskWoken );

	/* Was the message posted successfully? */
	if( xResult != pdFAIL )
	{
		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		switch should be requested.  The macro used is port specific and will
		be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		the documentation page for the port being used. */
		//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}	
}
