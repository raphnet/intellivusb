/* IntelliVUSB: Intellvision controller to USB adapter
 * Copyright (C) 2008-2016 Raphaël Assénat
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
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

#include "usbdrv.h"
#include "oddebug.h"
#include "gamepad.h"

#include "intellivision.h"

#include "devdesc.h"

static uchar *rt_usbHidReportDescriptor=NULL;
static uchar rt_usbHidReportDescriptorSize=0;
static uchar *rt_usbDeviceDescriptor=NULL;
static uchar rt_usbDeviceDescriptorSize=0;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168A__) || \
    defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || \
	    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88__) || \
		    defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88P__) || \
			    defined(__AVR_ATmega88PA__)
#define AT168_COMPATIBLE
#endif

const PROGMEM int usbDescriptorStringSerialNumber[]  = {
 	USB_STRING_DESCRIPTOR_HEADER(4),
	'1','0','0','8'
};

char usbDescriptorConfiguration[] = { 0 }; // dummy

uchar my_usbDescriptorConfiguration[] = {    /* USB configuration descriptor */
    9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
    USBDESCR_CONFIG,    /* descriptor type */
    18 + 7 * USB_CFG_HAVE_INTRIN_ENDPOINT + 9, 0,
                /* total length of data returned (including inlined descriptors) */
    1,          /* number of interfaces in this configuration */
    1,          /* index of this configuration */
    0,          /* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
    USBATTR_SELFPOWER,  /* attributes */
#else
    USBATTR_BUSPOWER,   /* attributes */
#endif
    USB_CFG_MAX_BUS_POWER/2,            /* max USB current in 2mA units */
/* interface descriptor follows inline: */
    9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
    USBDESCR_INTERFACE, /* descriptor type */
    0,          /* index of this interface */
    0,          /* alternate setting for this interface */
    USB_CFG_HAVE_INTRIN_ENDPOINT,   /* endpoints excl 0: number of endpoint descriptors to follow */
    USB_CFG_INTERFACE_CLASS,
    USB_CFG_INTERFACE_SUBCLASS,
    USB_CFG_INTERFACE_PROTOCOL,
    0,          /* string index for interface */
//#if (USB_CFG_DESCR_PROPS_HID & 0xff)    /* HID descriptor */
    9,          /* sizeof(usbDescrHID): length of descriptor in bytes */
    USBDESCR_HID,   /* descriptor type: HID */
    0x01, 0x01, /* BCD representation of HID version */
    0x00,       /* target country code */
    0x01,       /* number of HID Report (or other HID class) Descriptor infos to follow */
    0x22,       /* descriptor type: report */
    USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0,  /* total length of report descriptor */
//#endif
#if USB_CFG_HAVE_INTRIN_ENDPOINT    /* endpoint descriptor for endpoint 1 */
    7,          /* sizeof(usbDescrEndpoint) */
    USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
    0x81,       /* IN endpoint number 1 */
    0x03,       /* attrib: Interrupt endpoint */
    8, 0,       /* maximum packet size */
    USB_CFG_INTR_POLL_INTERVAL, /* in ms */
#endif
};

static Gamepad *curGamepad;


/* ----------------------- hardware I/O abstraction ------------------------ */

static void hardwareInit(void)
{
    /* PORTB
     *
     * Bit     Description       Direction    Level/pu
     * 0       Jumpers common    Out          0
     * 1       JP1               In           1
     * 2       JP2               In           1
     * 3       MOSI              In           1
     * 4       MISO              In           1
     * 5       SCK               In           1
     * 6       -
     * 7       -
     */
    DDRB = 0x01;
    PORTB = 0xFE;

    // init port C as input with pullup
    DDRC = 0x00;
    PORTC = 0xff;

    /*
     * For port D, activate pull-ups on all lines except the D+, D- and bit 1.
     *
     * For historical reasons (a mistake on an old PCB), bit 1
     * is now always connected with bit 0. So bit 1 configured
     * as an input without pullup.
     *
     * Usb pin are init as output, low. (device reset). They are released
     * later when usbReset() is called.
     */
    PORTD = 0xf8;
    DDRD = 0x01 | 0x04;

    /* Configure timers */
#if defined(AT168_COMPATIBLE)
    TCCR2A= (1<<WGM21);
    TCCR2B=(1<<CS22)|(1<<CS21)|(1<<CS20);
    OCR2A=196;  // for 60 hz
#else
    TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);
    OCR2 = 196; // for 60 hz
#endif
}

static void usbReset(void)
{
    /* [...] a single ended zero or SE0 can be used to signify a device
       reset if held for more than 10mS. A SE0 is generated by holding
       both th D- and D+ low (< 0.3V).
    */

    PORTD &= ~(0x01 | 0x04); // Set D+ and D- to 0
    DDRD |= 0x01 | 0x04;
    _delay_ms(15);
    DDRD &= ~(0x01 | 0x04);
}

#if defined(AT168_COMPATIBLE)

#define mustPollControllers()   (TIFR2 & (1<<OCF2A))
#define clrPollControllers()    do { TIFR2 = 1<<OCF2A; } while(0)

#else

#define mustPollControllers()   (TIFR & (1<<OCF2))
#define clrPollControllers()    do { TIFR = 1<<OCF2; } while(0)

#endif

static uchar    reportBuffer[6];    /* buffer for HID reports */



/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionDescriptor(struct usbRequest *rq)
{
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_STANDARD)
		return 0;

	if (rq->bRequest == USBRQ_GET_DESCRIPTOR)
	{
		// USB spec 9.4.3, high byte is descriptor type
		switch (rq->wValue.bytes[1])
		{
			case USBDESCR_DEVICE:
				usbMsgPtr = rt_usbDeviceDescriptor;
				return rt_usbDeviceDescriptorSize;
			case USBDESCR_HID_REPORT:
				usbMsgPtr = rt_usbHidReportDescriptor;
				return rt_usbHidReportDescriptorSize;
			case USBDESCR_CONFIG:
				usbMsgPtr = my_usbDescriptorConfiguration;
				return sizeof(my_usbDescriptorConfiguration);
		}
	}

	return 0;
}

uchar	usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;

	usbMsgPtr = reportBuffer;
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
		if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* we only have one report type, so don't look at wValue */
			curGamepad->buildReport(reportBuffer);
			return curGamepad->report_size;
		}
	}else{
	/* no vendor specific requests implemented */
	}
	return 0;
}

/* ------------------------------------------------------------------------- */


int main(void)
{
	char must_report = 0, first_run = 1;

	hardwareInit();

	/* Dip switch common: DB0, outputs: DB1 and DB2 */
	DDRB |= 0x01;
	DDRB &= ~0x06;

	PORTB |= 0x06; /* enable pull up on DB1 and DB2 */
	PORTB &= ~0x01; /* Set DB0 to low */


	curGamepad = intellivisionGetGamepad();


	// configure report descriptor according to
	// the current gamepad
	rt_usbHidReportDescriptor = curGamepad->reportDescriptor;
	rt_usbHidReportDescriptorSize = curGamepad->reportDescriptorSize;

	if (curGamepad->deviceDescriptor != 0)
	{
		rt_usbDeviceDescriptor = (void*)curGamepad->deviceDescriptor;
		rt_usbDeviceDescriptorSize = curGamepad->deviceDescriptorSize;
	}
	else
	{
		// use descriptor from devdesc.c
		//
		rt_usbDeviceDescriptor = (void*)usbDescrDevice;
		rt_usbDeviceDescriptorSize = getUsbDescrDevice_size();
	}

	// patch the config descriptor with the HID report descriptor size
	my_usbDescriptorConfiguration[25] = rt_usbHidReportDescriptorSize;

	usbReset();

	//wdt_enable(WDTO_2S);
	curGamepad->init();
	odDebugInit();
	usbInit();
	sei();
	DBG1(0x00, 0, 0);

	for(;;){	/* main event loop */
		wdt_reset();

		// this must be called at each 50 ms or less
		usbPoll();

		if (first_run) {
			curGamepad->update();
			first_run = 0;
		}

		if (mustPollControllers())
		{
			clrPollControllers();
			if (!must_report)
			{
				curGamepad->update();
				if (curGamepad->changed()) {
					must_report = 1;
				}
			}

		}

		if(must_report)
		{
			if (usbInterruptIsReady())
			{
				must_report = 0;

				curGamepad->buildReport(reportBuffer);
				usbSetInterrupt(reportBuffer, curGamepad->report_size);
			}
		}
		/*
		if(must_report && usbInterruptIsReady()){
			must_report = 0;

			curGamepad->buildReport(reportBuffer);
			usbSetInterrupt(reportBuffer, curGamepad->report_size);
		}
*/
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
