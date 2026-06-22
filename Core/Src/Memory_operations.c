/*
 * Memory_operations.c
 *
 *  This file contains the high-level operations that can be used for managing the SPI NAND
 *  You will need the SPI_NAND.c library.
 *
 */

#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include <stdint.h>
#include "Memory_operations.h"

NAND_info data;

void find_bad_blocks(uint16_t *bad_blocks){

	read_address_t blocco;
	blocco.block=0;
	blocco.page=0;
	blocco.dummy=0;
	bool is_bad_mark=true;
	int j = 0;
	for(int i = 0; i<2048; i++){
		blocco.block=i;
		spi_nand_block_is_bad(blocco, &is_bad_mark);
		/*
		if(is_bad_mark){
		  bad_blocks[i]=1;
		}*/
		if(!is_bad_mark) {
		  bad_blocks[j]=i;
		  j++;
		}

	}
}


void erase_good_blocks(uint8_t *bad_blocks){
	read_address_t blocco;
	blocco.block=0;
	blocco.page=0;
	blocco.dummy=0;
	bool is_bad_mark=true;
	for(int i = 0; i<2048; i++){
		blocco.block=i;
		spi_nand_block_is_bad(blocco, &is_bad_mark);
		if(is_bad_mark){
		  bad_blocks[i]=1;
		}
		if(!is_bad_mark) {
		  bad_blocks[i]=0;
		  spi_nand_block_erase(blocco);
		}
	}
}

extern uint16_t sample;
extern uint8_t NAND_packet[4096];
void write_memory(void);

void write_packet(uint8_t data_type, Time_Struct timestamp, uint8_t *payload, uint8_t payload_size){
	uint16_t packet_size = 1 + 5 + payload_size;
	
	if (sample + packet_size > 4096) {
		for(uint16_t i = sample; i < 4096; i++) NAND_packet[i] = 0xFF;
		sample = 4096;
		write_memory();
	}
	
	NAND_packet[sample++] = data_type;
	NAND_packet[sample++] = timestamp.hh;
	NAND_packet[sample++] = timestamp.mm;
	NAND_packet[sample++] = timestamp.ss;
	NAND_packet[sample++] = timestamp.sss & 0xFF;
	NAND_packet[sample++] = (timestamp.sss >> 8) & 0xFF;
	
	for (uint8_t i = 0; i < payload_size; i++) {
		NAND_packet[sample++] = payload[i];
	}
}
