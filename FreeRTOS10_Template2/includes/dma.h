/*
 * dma.h
 *
 * Created: 05.04.2019 09:24:42
 *  Author: Claudio Hediger
 */ 


#ifndef DMA_H_
#define DMA_H_

void vInitDMA();

#define QAM_4_SYM_00	0
#define QAM_4_SYM_01	8
#define QAM_4_SYM_10	16
#define QAM_4_SYM_11	24

uint8_t qamSymbols[8];
volatile uint8_t qamSymbolCount;


volatile uint8_t buffer_a[2048];
volatile uint8_t buffer_b[2048];

uint8_t nextLUTOffset;


#endif /* DMA_H_ */