/*!
   lightshow -- lightshow on ZTEX USB-FPGA Module 1.15a plus Experimental Board 1.10
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

// Endpoint 2 is used to high speed FPGA configuration
EP_CONFIG(2,0,BULK,OUT,512,4);	 

// select ZTEX USB FPGA Module 1.15 + Experimental Board 1.10 as target
IDENTITY_UFM_1_15(10.13.0.0,0);	 
EXTENSION_EXP_1_10;

// enables high speed FPGA configuration, use EP 2
ENABLE_HS_FPGA_CONF(2);

// this product string is also used for identification by the host software
#define[PRODUCT_STRING]["lightshow for EXP-1.10"]

// enable Flash support
ENABLE_FLASH;
ENABLE_FLASH_BITSTREAM;

// include the main part of the firmware kit, define the descriptors, ...
#include[ztex.h]

void main(void)	
{
// init everything
    init_USB();

    while ( 1 ) {
    }
}

