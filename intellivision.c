/* IntelliVUSB: Intellvision controller to USB adapter
 * Copyright (C) 2008 Raphaël Assénat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The author may be contacted at raph@raphnet.net
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "gamepad.h"
#include "intellivision.h"

#define REPORT_SIZE		4


/*********** prototypes *************/
static void intellivisionInit(void);
static void intellivisionUpdate(void);
static char intellivisionChanged(void);
static void intellivisionBuildReport(unsigned char *reportBuffer);



// report matching the most recent bytes from the controller
static unsigned char last_read_controller_bytes[REPORT_SIZE];

// the most recently reported bytes
static unsigned char last_reported_controller_bytes[REPORT_SIZE];

static void readXY(char *x, char *y)
{
	char cur_x, cur_y;
	unsigned char raw_dir;

	// start by reading the rotary encoder
	//
	// Pin 1 (PC5) is common ground
	// Pin 2-6 (PC4-PC0) are inputs will pull-up
	//
	DDRB &= ~0x3E; // port B input with pull-up
	PORTB |= 0x3E; 

	PORTC=0; // set to low before changing direction
	DDRC=0x20; // bit 5 is common ground for direction. Set as output.
	PORTC = 0x1F; // enable pull-up everywhere except ground

	_delay_ms(1);
	
	raw_dir = PINC;
	raw_dir ^= 0xff;
	raw_dir &= 0x1f;

	switch(raw_dir)
	{
		case 0x04: cur_x =  0; cur_y = -2; break;
		case 0x06: cur_x = -1; cur_y = -2; break;
		case 0x07: cur_x = -2; cur_y = -2; break;
		case 0x03: cur_x = -2; cur_y = -1; break;
		case 0x02: cur_x = -2; cur_y =  0; break;
		case 0x12: cur_x = -2; cur_y =  1; break;
		case 0x13: cur_x = -2; cur_y =  2; break;
		case 0x11: cur_x = -1; cur_y =  2; break;
		case 0x10: cur_x =  0; cur_y =  2; break;
		case 0x18: cur_x =  1; cur_y =  2; break;
		case 0x19: cur_x =  2; cur_y =  2; break;
		case 0x09: cur_x =  2; cur_y =  1; break;
		case 0x08: cur_x =  2; cur_y =  0; break;
		case 0x0C: cur_x =  2; cur_y = -1; break;
		case 0x0D: cur_x =  2; cur_y = -2; break;
		case 0x05: cur_x =  1; cur_y = -2; break;

		case 0x00:  
			cur_x = 0; cur_y = 0;
			break;

		// maybe keep last valid direction?
		default:
			cur_x = 0; cur_y = 0;
	}

	*x = cur_x;
	*y = cur_y;

}

static void readController(char *x, char *y, unsigned short *btns)
{
	unsigned char cols;
	unsigned char rows;
	unsigned short b=0;

	char cur_x, cur_y;
	static char last_x=0, last_y=0;


	readXY(&cur_x, &cur_y);


	// enable pull ups on columns
	DDRC &= ~0x1E; // reverse to input...
	PORTC |= 0x1E; // and enable pull ups.

	PORTB &= ~0x38; // clear row bits.
	DDRB |= 0x38; // and set as output (low)

	_delay_ms(1); // stabilization delay

	cols = (PINC) >> 1;

	// enable ground for rows
	PORTC &= ~0x1E; // clear bits
	DDRC |= 0x1E; // set as output

	PORTB &= ~0x38; // disable pull-ups
	DDRB &= ~0x38; // reverse to input
	PORTB |= 0x38; // bit 5 4 3 with pullup to read columns

	_delay_ms(1); // stabilization delay
	
	rows = (PINB) >> 3;

	// cols bits:
	// 0: ENT 0 CLR
	// 1: 9 8 7
	// 2: 6 5 4
	// 3: 3 2 1
	//
	// rows bits:
	// 0: 1 4 7 CLR
	// 1: 2 5 8 0
	// 2: 3 6 9 ENT
	
	rows ^= 0xff;
	rows &= 0x07;
	cols ^= 0xff;
	cols &= 0x0f;
	
	switch (cols | (rows<<4))
	{
		case 0x11: b |= 1<<12; // CLR
					break;
		case 0x12: b |= 1<<9; // 7
					break;
		case 0x14: b |= 1<<6; // 4
					break;
		case 0x18: b |= 1<<3; // 1
					break;

		case 0x21: b |= 1<<13; // 0
					break;
		case 0x22: b |= 1<<10; // 8
					break;
		case 0x24: b |= 1<<7; // 5
					break;
		case 0x28: b |= 1<<4; // 2
					break;

		case 0x41: b |= 1<<14; // ENT
					break;
		case 0x42: b |= 1<<11; // 9
					break;
		case 0x44: b |= 1<<8; // 6
					break;
		case 0x48: b |= 1<<5; // 3
					break;
	}

	
	PORTC=0;
	DDRC=0;
	PORTC=0xff;

	// take care of the side buttons
	PORTB =0;
	DDRB = 0x08; // PB3 low to pull  R1+L1 or R2
	PORTB |= 0x30; // PB4 and PB5 with pullup
	
	_delay_ms(1); // stabilization delay

	cols = PINB & 0x30;

	if (! (cols & 0x10) )
		b |= 1<<2;
	if (! (cols & 0x20) )
		b |= 1<<0;
	
	// one last button
	PORTB = 0;
	DDRB = 0x10; // PB4 low to pull L2
	PORTB |= 0x20; // L2 goes to PB5
	
	_delay_ms(1); // stabilization delay

	cols = PINB & 0x20;
	if (! (cols & 0x20) )
		b |= 1<<1;

	*btns = b;

	if (b & 0xfff8)  {
		// Interfering buttons

		*x = last_x;
		*y = last_y;
	}
	else {
		// perform a second direction read. To debounce...	
	
		readXY(x, y);
		/* If the two values we just read disagree, use the last known stable value */
		if (*x != cur_x) {
			*x = last_x;
		} else {
			last_x = cur_x;
		}
		if (*y != cur_y) {
			*y = last_y;
		} else {
			last_y = cur_y;
		}

	}

}

static void intellivisionInit(void)
{
	unsigned char sreg;
	sreg = SREG;
	cli();


	/* 
	 * PC5: pin 1 (disc common)
	 * PC4: pin 2
	 * PC3: pin 3
	 * PC2: pin 4
	 *
	 * PC1: pin 5
	 * PC0: pin 6
	 * PB5: pin 7
	 * PB4: pin 8
	 * PB3: pin 9
	 */

	DDRC=0;
	DDRB=0;
	PORTC=0xff;
	PORTB=0xff;

	intellivisionUpdate();

	SREG = sreg;
}

int convAxe(char n)
{
	switch(n)
	{
		case -2: return 0;
		case -1: return 64;
		case 0: return 128;
		case 1: return 192;
		case 2: return 255;
	}
	return 128;
}

static void intellivisionUpdate(void)
{
	char x,y;
	unsigned short btns;

	readController(&x, &y, &btns);

	last_read_controller_bytes[0]=(x*63) + 128;
	last_read_controller_bytes[1]=(y*63) + 128;
 	last_read_controller_bytes[2]=btns & 0xff;
 	last_read_controller_bytes[3]=(btns & 0xff00) >> 8;
/*
	if (data[0] & 0x02) // btn 0
		last_read_controller_bytes[2] |= 0x01;
	if (data[0] & 0x01) // btn 1
		last_read_controller_bytes[2] |= 0x02;
	if (data[1] & 0x20) // btn 2
		last_read_controller_bytes[2] |= 0x04;
	if (data[1] & 0x10) // btn 3
		last_read_controller_bytes[2] |= 0x08;
	if (data[1] & 0x08) // btn 4
		last_read_controller_bytes[2] |= 0x10;
	if (data[1] & 0x04) // btn 5
		last_read_controller_bytes[2] |= 0x20;
	if (data[1] & 0x02) // btn 6
		last_read_controller_bytes[2] |= 0x40;
	if (data[1] & 0x01) // btn 7
		last_read_controller_bytes[2] |= 0x80;
*/

}	

static char intellivisionChanged(void)
{
	static int first = 1;
	if (first) { first = 0;  return 1; }
	
	return memcmp(last_read_controller_bytes, 
					last_reported_controller_bytes, REPORT_SIZE);
}

static void intellivisionBuildReport(unsigned char *reportBuffer)
{
	if (reportBuffer != NULL)
	{
		memcpy(reportBuffer, last_read_controller_bytes, REPORT_SIZE);
	}
	memcpy(last_reported_controller_bytes, 
			last_read_controller_bytes, 
			REPORT_SIZE);	
}

#include "report_desc_2axe_16btn.c"

Gamepad intellivisionGamepad = {
	report_size: 		REPORT_SIZE,
	reportDescriptorSize:	sizeof(usbHidReportDescriptor_2axe_16btn),
	init: 			intellivisionInit,
	update: 		intellivisionUpdate,
	changed:		intellivisionChanged,
	buildReport:		intellivisionBuildReport
};

Gamepad *intellivisionGetGamepad(void)
{
	intellivisionGamepad.reportDescriptor = (void*)usbHidReportDescriptor_2axe_16btn;

	return &intellivisionGamepad;
}

