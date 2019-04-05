/*
 * dma.h
 *
 * Created: 05.04.2019 09:24:42
 *  Author: Claudio Hediger
 */ 


#ifndef DMA_H_
#define DMA_H_

void vInitDMA();


volatile uint8_t buffer_a[2048];
volatile uint8_t buffer_b[2048];


#endif /* DMA_H_ */