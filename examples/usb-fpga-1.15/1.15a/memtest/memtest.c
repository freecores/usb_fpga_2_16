/*!
   memtest -- DDR2 SDRAM FIFO for testing memory on ZTEX USB-FPGA Module 1.15a
   Copyright (C) 2009-2014 ZTEX GmbH.
   http://www.ztex.de

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 3 as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see http://www.gnu.org/licenses/.
!*/

#include[ztex-conf.h]	// Loads the configuration macros, see ztex-conf.h for the available macros
#include[ztex-utils.h]	// include basic functions

// configure endpoint 2, in, quad buffered, 512 bytes, interface 0
EP_CONFIG(2,0,BULK,IN,512,4);

// configure endpoint 6, out, doublebuffered, 512 bytes, interface 0
EP_CONFIG(6,0,BULK,OUT,512,2);

// select ZTEX USB FPGA Module 1.11 as target  (required for FPGA configuration)
IDENTITY_UFM_1_15(10.13.0.0,0);	 

// enables high speed FPGA configuration via EP6
ENABLE_HS_FPGA_CONF(6);

// this product string is also used for identification by the host software
#define[PRODUCT_STRING]["memtest example for UFM 1.15"]

// 0 : counter mode; 1: shift pattern mode 
__xdata BYTE mode = 0;

// this is called automatically after FPGA configuration
#define[POST_FPGA_CONFIG][POST_FPGA_CONFIG
	IOA7 = 1;				// reset on
	OEA |= bmBIT7;
	IOC0 = mode ? 1 : 0;
	OEC = bmBIT0;

	EP2CS &= ~bmBIT0;			// clear stall bit
    
	REVCTL = 0x3;
	SYNCDELAY; 

	IFCONFIG = bmBIT7 | bmBIT5 | 3;	        // internel 30MHz clock, drive IFCLK ouput, slave FIFO interface
	SYNCDELAY; 
	EP2FIFOCFG = bmBIT3 | bmBIT0;           // AOTUOIN, WORDWIDE
	SYNCDELAY;
    
	EP2AUTOINLENH = 2;                 	// 512 bytes 
	SYNCDELAY;
	EP2AUTOINLENL = 0;
	SYNCDELAY;

	FIFORESET = 0x80;			// reset FIFO
	SYNCDELAY;
	FIFORESET = 2;
	SYNCDELAY;
	FIFORESET = 0x00;
	SYNCDELAY;

	FIFOPINPOLAR = 0;
	SYNCDELAY; 
	PINFLAGSAB = 0;
	SYNCDELAY; 
	PINFLAGSCD = 0;
	SYNCDELAY; 

	IOA7 = 0;				// reset off
]

// set the test pattern
ADD_EP0_VENDOR_COMMAND((0x60,,
	mode = SETUPDAT[2];
,,
	NOP;
));;

// include the main part of the firmware kit, define the descriptors, ...
#include[ztex.h]

void main(void)	
{
    init_USB();
    
    while (1) {	
    }
}

