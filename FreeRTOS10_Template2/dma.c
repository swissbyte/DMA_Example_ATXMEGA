
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

#define SYMBOL_BUFFER_SIZE	32
#define DAC_IDLE_VOLTAGE 0

volatile uint8_t buffer_a[2048];
volatile uint8_t buffer_b[2048]; 

volatile uint8_t LUTOffset = 0; 
uint8_t nextLUTOffset = 0;
volatile uint8_t lutCount = 0;
volatile uint8_t qamSymbolCount = 0;
volatile uint8_t qamBlockTransfer = 0;

void vSetDMA_LUT_Offset();

//Two Sinusodial Periods are saved here to easily move around with the offset!

const uint16_t sineLUT[SYMBOL_BUFFER_SIZE * 2] =
{
	// 100%
	0x7f,0xb0,0xd9,0xf4,0xfe,0xf4,0xd9,0xb0,
	0x7f,0x4e,0x25,0xa,0x0,0xa,0x25,0x4e,
	0x7f,0xb0,0xd9,0xf4,0xfe,0xf4,0xd9,0xb0,
	0x7f,0x4e,0x25,0xa,0x0,0xa,0x25,0x4e,
	0x7f,0xb0,0xd9,0xf4,0xfe,0xf4,0xd9,0xb0,
	0x7f,0x4e,0x25,0xa,0x0,0xa,0x25,0x4e,
	0x7f,0xb0,0xd9,0xf4,0xfe,0xf4,0xd9,0xb0,
	0x7f,0x4e,0x25,0xa,0x0,0xa,0x25,0x4e
};

uint8_t qamSymbols[8] = {0,0,0,0,0,0,0};


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

void vInitDAC()
{
	DACB.CTRLA = DAC_CH0EN_bm;	// Enable CH0
	DACB.CTRLB = DAC_CH0TRIG_bm;	// AutoTrigger CH0
	DACB.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm; //0x0;	// AVcc as Refernece Voltage
	DACB.EVCTRL = DAC_EVSEL_0_gc;	// Event Channel 1
	DACB.CTRLA |= DAC_ENABLE_bm;
	PORTB.DIRSET = 0x04;
}


void vStartQAMTransfer(void)
{
	vSetDMA_LUT_Offset();
}


void vSetDMA_LUT_Offset()
{

	// Die Transaktion wurde gestartet und wir warten


	LUTOffset = qamSymbols[qamSymbolCount];

	if(qamBlockTransfer == 1)
	{
		qamBlockTransfer = 0;
		while ((DMA.CH0.CTRLB & (DMA_CH_CHBUSY_bm | DMA_CH_CHPEND_bm)));
	}

	if(qamSymbolCount != 0)
	{
		qamBlockTransfer = 2;
		qamSymbolCount --;		
	}
	else 
	{
		if(qamBlockTransfer == 2) qamBlockTransfer = 1;
	}	
	
	if(qamBlockTransfer)
	{	
		DMA.CH0.SRCADDR0	= ( (uint16_t) (&sineLUT[0 + LUTOffset]) >> 0) & 0xFF;
		DMA.CH0.SRCADDR1	= ( (uint16_t) (&sineLUT[0 + LUTOffset]) >> 8) & 0xFF;
		DMA.CH0.CTRLA		|= DMA_CH_ENABLE_bm;		
	}	
}

void vDMAIntHandler()
{
		//Wenn wir nicht am Ararbeiten von Symbolen sind! Dann gehen wir in den Task
	if(qamBlockTransfer == 0)
	{
		
		BaseType_t xHigherPriorityTaskWoken, xResult;

		/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
		xHigherPriorityTaskWoken = pdFALSE;

		/* Set bit 0 and bit 4 in xEventGroup. */
		xResult = xEventGroupSetBitsFromISR(
									xDMAProcessEventGroup,   /* The event group being updated. */
									DMA_EVT_GRP_QAM_FINISHED, /* The bits being set. */
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
	
}

void vInitDMATimer()
{
	// set TCC1 to 11024Hz overflow, actually 11019.2838Hz (-0.052% error)
	TCC1.CTRLA = 0; // stop if running
	TCC1.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC1.CTRLD = TC_EVACT_RESTART_gc;
	TCC1.CNT = 0;
	TCC1.PER = 0x00FF; //DMA Trigger Speed. SINE WAVE f = 1/(((fCPU/TCC1_DIV) / TCC1.PER) / SYMBOL_BUFFER_SIZE)

	EVSYS.CH0MUX = EVSYS_CHMUX_TCC1_OVF_gc; // trigger on timer overflow
}

void vInitDMA()
{
	// reset DMA controller
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);
	
	DMA.CTRL = DMA_CH_ENABLE_bm; //| DMA_DBUFMODE_CH01_gc; // double buffered with channels 0 and 1
	
	//Bei Double Buffering wird automatisch aus Channel 0 und 1 ein "Pair" gebildet. 
	//Siehe dazu AVR1304.P8
	
	// channel 0
	// **** TODO: reset dma channels
	DMA.CH0.REPCNT		= 1;
	DMA.CH0.CTRLA		= DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm;// | DMA_CH_REPEAT_bm; // ADC result is 2 byte 12 bit word
	DMA.CH0.CTRLB		= 0x1;
	DMA.CH0.ADDRCTRL	= DMA_CH_SRCDIR_INC_gc /*increment*/ | DMA_CH_DESTDIR_FIXED_gc |  DMA_CH_SRCRELOAD_TRANSACTION_gc; // reload dest after every transaction
	DMA.CH0.TRIGSRC		= DMA_CH_TRIGSRC_EVSYS_CH0_gc;
	DMA.CH0.TRFCNT		= SYMBOL_BUFFER_SIZE; // always the number of bytes, even if burst length > 1
	DMA.CH0.DESTADDR0	= ((uint16_t)(&DACB.CH0DATAH)>>0) & 0xFF;
	DMA.CH0.DESTADDR1	= ((uint16_t)(&DACB.CH0DATAH)>>8) & 0xFF;
	DMA.CH0.DESTADDR2	= 0;
	DMA.CH0.SRCADDR0	= ( (uint16_t) (&sineLUT[0 + LUTOffset]) >> 0) & 0xFF;
	DMA.CH0.SRCADDR1	= ( (uint16_t) (&sineLUT[0 + LUTOffset]) >> 8) & 0xFF;
	DMA.CH0.SRCADDR2	= 0;

	//DMA.CH0.CTRLA		|= DMA_CH_ENABLE_bm;
	
	TCC1.CTRLA			= TC_CLKSEL_DIV1024_gc; // start timer, and in turn ADC
}

ISR(DMA_CH0_vect)
{	
	//Interrupt quittieren
	DMA.CH0.CTRLB |= 0x10;
	//Erst nach einem weiteren Timer Tick wird der DMA weitergeführt.
	//Wir haben also Zeit, hier den DMA umzukonfigurieren.
	vSetDMA_LUT_Offset();
	vDMAIntHandler();
}
