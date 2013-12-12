                              1 ;--------------------------------------------------------
                              2 ; File Created by SDCC : free open source ANSI-C Compiler
                              3 ; Version 2.8.0 #5117 (May 15 2008) (UNIX)
                              4 ; This file was generated Wed Dec  4 01:40:19 2013
                              5 ;--------------------------------------------------------
                              6 	.module default_tmp
                              7 	.optsdcc -mmcs51 --model-small
                              8 	
                              9 ;--------------------------------------------------------
                             10 ; Public variables in this module
                             11 ;--------------------------------------------------------
                             12 	.globl _sendStringDescriptor_PARM_3
                             13 	.globl _sendStringDescriptor_PARM_2
                             14 	.globl _spi_write_PARM_2
                             15 	.globl _flash_read_PARM_2
                             16 	.globl _mac_eeprom_init_hexdigits_1_1
                             17 	.globl _EmptyStringDescriptor
                             18 	.globl _FullSpeedConfigDescriptor_PadByte
                             19 	.globl _FullSpeedConfigDescriptor
                             20 	.globl _HighSpeedConfigDescriptor_PadByte
                             21 	.globl _HighSpeedConfigDescriptor
                             22 	.globl _DeviceQualifierDescriptor
                             23 	.globl _DeviceDescriptor
                             24 	.globl _PadByte
                             25 	.globl _configurationString
                             26 	.globl _productString
                             27 	.globl _manufacturerString
                             28 	.globl _fpga_flash_boot_id
                             29 	.globl _main
                             30 	.globl _init_USB
                             31 	.globl _mac_eeprom_init
                             32 	.globl _EP8_ISR
                             33 	.globl _EP6_ISR
                             34 	.globl _EP4_ISR
                             35 	.globl _EP2_ISR
                             36 	.globl _EP1OUT_ISR
                             37 	.globl _EP1IN_ISR
                             38 	.globl _EP0ACK_ISR
                             39 	.globl _HSGRANT_ISR
                             40 	.globl _URES_ISR
                             41 	.globl _SUSP_ISR
                             42 	.globl _SUTOK_ISR
                             43 	.globl _SOF_ISR
                             44 	.globl _abscode_identity
                             45 	.globl _fpga_configure_from_flash_init
                             46 	.globl _fpga_first_free_sector
                             47 	.globl _fpga_configure_from_flash
                             48 	.globl _init_cpld_fpga_configuration
                             49 	.globl _fpga_send_ep0
                             50 	.globl _spi_send_ep0
                             51 	.globl _spi_read_ep0
                             52 	.globl _flash_init
                             53 	.globl _flash_write_next
                             54 	.globl _flash_write_finish
                             55 	.globl _flash_write_finish_sector
                             56 	.globl _flash_write_init
                             57 	.globl _flash_write
                             58 	.globl _flash_write_byte
                             59 	.globl _spi_pp
                             60 	.globl _flash_read_finish
                             61 	.globl _flash_read_next
                             62 	.globl _flash_read_init
                             63 	.globl _spi_wait
                             64 	.globl _spi_deselect
                             65 	.globl _spi_select
                             66 	.globl _spi_write
                             67 	.globl _spi_write_byte
                             68 	.globl _flash_read
                             69 	.globl _flash_read_byte
                             70 	.globl _spi_clocks
                             71 	.globl _mac_eeprom_read_ep0
                             72 	.globl _mac_eeprom_write
                             73 	.globl _mac_eeprom_read
                             74 	.globl _eeprom_write_ep0
                             75 	.globl _eeprom_read_ep0
                             76 	.globl _eeprom_write
                             77 	.globl _eeprom_read
                             78 	.globl _eeprom_select
                             79 	.globl _i2c_waitStop
                             80 	.globl _i2c_waitStart
                             81 	.globl _i2c_waitRead
                             82 	.globl _i2c_waitWrite
                             83 	.globl _MEM_COPY1_int
                             84 	.globl _uwait
                             85 	.globl _wait
                             86 	.globl _abscode_intvec
                             87 	.globl _EIPX6
                             88 	.globl _EIPX5
                             89 	.globl _EIPX4
                             90 	.globl _PI2C
                             91 	.globl _PUSB
                             92 	.globl _BREG7
                             93 	.globl _BREG6
                             94 	.globl _BREG5
                             95 	.globl _BREG4
                             96 	.globl _BREG3
                             97 	.globl _BREG2
                             98 	.globl _BREG1
                             99 	.globl _BREG0
                            100 	.globl _EIEX6
                            101 	.globl _EIEX5
                            102 	.globl _EIEX4
                            103 	.globl _EI2C
                            104 	.globl _EUSB
                            105 	.globl _ACC7
                            106 	.globl _ACC6
                            107 	.globl _ACC5
                            108 	.globl _ACC4
                            109 	.globl _ACC3
                            110 	.globl _ACC2
                            111 	.globl _ACC1
                            112 	.globl _ACC0
                            113 	.globl _SMOD1
                            114 	.globl _ERESI
                            115 	.globl _RESI
                            116 	.globl _INT6
                            117 	.globl _CY
                            118 	.globl _AC
                            119 	.globl _F0
                            120 	.globl _RS1
                            121 	.globl _RS0
                            122 	.globl _OV
                            123 	.globl _F1
                            124 	.globl _PF
                            125 	.globl _TF2
                            126 	.globl _EXF2
                            127 	.globl _RCLK
                            128 	.globl _TCLK
                            129 	.globl _EXEN2
                            130 	.globl _TR2
                            131 	.globl _CT2
                            132 	.globl _CPRL2
                            133 	.globl _SM0_1
                            134 	.globl _SM1_1
                            135 	.globl _SM2_1
                            136 	.globl _REN_1
                            137 	.globl _TB8_1
                            138 	.globl _RB8_1
                            139 	.globl _TI_1
                            140 	.globl _RI_1
                            141 	.globl _PS1
                            142 	.globl _PT2
                            143 	.globl _PS0
                            144 	.globl _PT1
                            145 	.globl _PX1
                            146 	.globl _PT0
                            147 	.globl _PX0
                            148 	.globl _IOD7
                            149 	.globl _IOD6
                            150 	.globl _IOD5
                            151 	.globl _IOD4
                            152 	.globl _IOD3
                            153 	.globl _IOD2
                            154 	.globl _IOD1
                            155 	.globl _IOD0
                            156 	.globl _EA
                            157 	.globl _ES1
                            158 	.globl _ET2
                            159 	.globl _ES0
                            160 	.globl _ET1
                            161 	.globl _EX1
                            162 	.globl _ET0
                            163 	.globl _EX0
                            164 	.globl _IOC7
                            165 	.globl _IOC6
                            166 	.globl _IOC5
                            167 	.globl _IOC4
                            168 	.globl _IOC3
                            169 	.globl _IOC2
                            170 	.globl _IOC1
                            171 	.globl _IOC0
                            172 	.globl _SM0_0
                            173 	.globl _SM1_0
                            174 	.globl _SM2_0
                            175 	.globl _REN_0
                            176 	.globl _TB8_0
                            177 	.globl _RB8_0
                            178 	.globl _TI_0
                            179 	.globl _RI_0
                            180 	.globl _IOB7
                            181 	.globl _IOB6
                            182 	.globl _IOB5
                            183 	.globl _IOB4
                            184 	.globl _IOB3
                            185 	.globl _IOB2
                            186 	.globl _IOB1
                            187 	.globl _IOB0
                            188 	.globl _TF1
                            189 	.globl _TR1
                            190 	.globl _TF0
                            191 	.globl _TR0
                            192 	.globl _IE1
                            193 	.globl _IT1
                            194 	.globl _IE0
                            195 	.globl _IT0
                            196 	.globl _IOA7
                            197 	.globl _IOA6
                            198 	.globl _IOA5
                            199 	.globl _IOA4
                            200 	.globl _IOA3
                            201 	.globl _IOA2
                            202 	.globl _IOA1
                            203 	.globl _IOA0
                            204 	.globl _EIP
                            205 	.globl _BREG
                            206 	.globl _EIE
                            207 	.globl _ACC
                            208 	.globl _EICON
                            209 	.globl _PSW
                            210 	.globl _TH2
                            211 	.globl _TL2
                            212 	.globl _RCAP2H
                            213 	.globl _RCAP2L
                            214 	.globl _T2CON
                            215 	.globl _SBUF1
                            216 	.globl _SCON1
                            217 	.globl _GPIFSGLDATLNOX
                            218 	.globl _GPIFSGLDATLX
                            219 	.globl _GPIFSGLDATH
                            220 	.globl _GPIFTRIG
                            221 	.globl _EP01STAT
                            222 	.globl _IP
                            223 	.globl _OEE
                            224 	.globl _OED
                            225 	.globl _OEC
                            226 	.globl _OEB
                            227 	.globl _OEA
                            228 	.globl _IOE
                            229 	.globl _IOD
                            230 	.globl _AUTOPTRSETUP
                            231 	.globl _EP68FIFOFLGS
                            232 	.globl _EP24FIFOFLGS
                            233 	.globl _EP2468STAT
                            234 	.globl _IE
                            235 	.globl _INT4CLR
                            236 	.globl _INT2CLR
                            237 	.globl _IOC
                            238 	.globl _AUTOPTRL2
                            239 	.globl _AUTOPTRH2
                            240 	.globl _AUTOPTRL1
                            241 	.globl _AUTOPTRH1
                            242 	.globl _SBUF0
                            243 	.globl _SCON0
                            244 	.globl __XPAGE
                            245 	.globl _MPAGE
                            246 	.globl _EXIF
                            247 	.globl _IOB
                            248 	.globl _CKCON
                            249 	.globl _TH1
                            250 	.globl _TH0
                            251 	.globl _TL1
                            252 	.globl _TL0
                            253 	.globl _TMOD
                            254 	.globl _TCON
                            255 	.globl _PCON
                            256 	.globl _DPS
                            257 	.globl _DPH1
                            258 	.globl _DPL1
                            259 	.globl _DPH0
                            260 	.globl _DPL0
                            261 	.globl _SP
                            262 	.globl _IOA
                            263 	.globl _ISOFRAME_COUNTER
                            264 	.globl _ep0_vendor_cmd_setup
                            265 	.globl _ep0_prev_setup_request
                            266 	.globl _GPIF_WAVE_DATA_HSFPGA_12MHZ
                            267 	.globl _GPIF_WAVE_DATA_HSFPGA_24MHZ
                            268 	.globl _ep0_payload_transfer
                            269 	.globl _ep0_payload_remaining
                            270 	.globl _SN_STRING
                            271 	.globl _MODULE_RESERVED
                            272 	.globl _INTERFACE_CAPABILITIES
                            273 	.globl _INTERFACE_VERSION
                            274 	.globl _FW_VERSION
                            275 	.globl _PRODUCT_ID
                            276 	.globl _ZTEXID
                            277 	.globl _ZTEX_DESCRIPTOR_VERSION
                            278 	.globl _ZTEX_DESCRIPTOR
                            279 	.globl _OOEC
                            280 	.globl _fpga_conf_initialized
                            281 	.globl _fpga_flash_result
                            282 	.globl _fpga_init_b
                            283 	.globl _fpga_bytes
                            284 	.globl _fpga_checksum
                            285 	.globl _ep0_write_mode
                            286 	.globl _ep0_read_mode
                            287 	.globl _spi_write_sector
                            288 	.globl _spi_need_pp
                            289 	.globl _spi_write_addr_lo
                            290 	.globl _spi_write_addr_hi
                            291 	.globl _spi_buffer
                            292 	.globl _spi_last_cmd
                            293 	.globl _spi_erase_cmd
                            294 	.globl _spi_memtype
                            295 	.globl _spi_device
                            296 	.globl _spi_vendor
                            297 	.globl _flash_ec
                            298 	.globl _flash_sectors
                            299 	.globl _flash_sector_size
                            300 	.globl _flash_enabled
                            301 	.globl _config_data_valid
                            302 	.globl _mac_eeprom_addr
                            303 	.globl _eeprom_write_checksum
                            304 	.globl _eeprom_write_bytes
                            305 	.globl _eeprom_addr
                            306 	.globl _INTVEC_GPIFWF
                            307 	.globl _INTVEC_GPIFDONE
                            308 	.globl _INTVEC_EP8FF
                            309 	.globl _INTVEC_EP6FF
                            310 	.globl _INTVEC_EP2FF
                            311 	.globl _INTVEC_EP8EF
                            312 	.globl _INTVEC_EP6EF
                            313 	.globl _INTVEC_EP4EF
                            314 	.globl _INTVEC_EP2EF
                            315 	.globl _INTVEC_EP8PF
                            316 	.globl _INTVEC_EP6PF
                            317 	.globl _INTVEC_EP4PF
                            318 	.globl _INTVEC_EP2PF
                            319 	.globl _INTVEC_EP8ISOERR
                            320 	.globl _INTVEC_EP6ISOERR
                            321 	.globl _INTVEC_EP4ISOERR
                            322 	.globl _INTVEC_EP2ISOERR
                            323 	.globl _INTVEC_ERRLIMIT
                            324 	.globl _INTVEC_EP8PING
                            325 	.globl _INTVEC_EP6PING
                            326 	.globl _INTVEC_EP4PING
                            327 	.globl _INTVEC_EP2PING
                            328 	.globl _INTVEC_EP1PING
                            329 	.globl _INTVEC_EP0PING
                            330 	.globl _INTVEC_IBN
                            331 	.globl _INTVEC_EP8
                            332 	.globl _INTVEC_EP6
                            333 	.globl _INTVEC_EP4
                            334 	.globl _INTVEC_EP2
                            335 	.globl _INTVEC_EP1OUT
                            336 	.globl _INTVEC_EP1IN
                            337 	.globl _INTVEC_EP0OUT
                            338 	.globl _INTVEC_EP0IN
                            339 	.globl _INTVEC_EP0ACK
                            340 	.globl _INTVEC_HISPEED
                            341 	.globl _INTVEC_USBRESET
                            342 	.globl _INTVEC_SUSPEND
                            343 	.globl _INTVEC_SUTOK
                            344 	.globl _INTVEC_SOF
                            345 	.globl _INTVEC_SUDAV
                            346 	.globl _INT12VEC_IE6
                            347 	.globl _INT11VEC_IE5
                            348 	.globl _INT10VEC_GPIF
                            349 	.globl _INT9VEC_I2C
                            350 	.globl _INT8VEC_USB
                            351 	.globl _INT7VEC_USART1
                            352 	.globl _INT6VEC_RESUME
                            353 	.globl _INT5VEC_T2
                            354 	.globl _INT4VEC_USART0
                            355 	.globl _INT3VEC_T1
                            356 	.globl _INT2VEC_IE1
                            357 	.globl _INT1VEC_T0
                            358 	.globl _INT0VEC_IE0
                            359 	.globl _EP8FIFOBUF
                            360 	.globl _EP6FIFOBUF
                            361 	.globl _EP4FIFOBUF
                            362 	.globl _EP2FIFOBUF
                            363 	.globl _EP1INBUF
                            364 	.globl _EP1OUTBUF
                            365 	.globl _EP0BUF
                            366 	.globl _GPIFABORT
                            367 	.globl _GPIFREADYSTAT
                            368 	.globl _GPIFREADYCFG
                            369 	.globl _XGPIFSGLDATLNOX
                            370 	.globl _XGPIFSGLDATLX
                            371 	.globl _XGPIFSGLDATH
                            372 	.globl _EP8GPIFTRIG
                            373 	.globl _EP8GPIFPFSTOP
                            374 	.globl _EP8GPIFFLGSEL
                            375 	.globl _EP6GPIFTRIG
                            376 	.globl _EP6GPIFPFSTOP
                            377 	.globl _EP6GPIFFLGSEL
                            378 	.globl _EP4GPIFTRIG
                            379 	.globl _EP4GPIFPFSTOP
                            380 	.globl _EP4GPIFFLGSEL
                            381 	.globl _EP2GPIFTRIG
                            382 	.globl _EP2GPIFPFSTOP
                            383 	.globl _EP2GPIFFLGSEL
                            384 	.globl _GPIFTCB0
                            385 	.globl _GPIFTCB1
                            386 	.globl _GPIFTCB2
                            387 	.globl _GPIFTCB3
                            388 	.globl _FLOWSTBHPERIOD
                            389 	.globl _FLOWSTBEDGE
                            390 	.globl _FLOWSTB
                            391 	.globl _FLOWHOLDOFF
                            392 	.globl _FLOWEQ1CTL
                            393 	.globl _FLOWEQ0CTL
                            394 	.globl _FLOWLOGIC
                            395 	.globl _FLOWSTATE
                            396 	.globl _GPIFADRL
                            397 	.globl _GPIFADRH
                            398 	.globl _GPIFCTLCFG
                            399 	.globl _GPIFIDLECTL
                            400 	.globl _GPIFIDLECS
                            401 	.globl _GPIFWFSELECT
                            402 	.globl _wLengthH
                            403 	.globl _wLengthL
                            404 	.globl _wIndexH
                            405 	.globl _wIndexL
                            406 	.globl _wValueH
                            407 	.globl _wValueL
                            408 	.globl _bRequest
                            409 	.globl _bmRequestType
                            410 	.globl _SETUPDAT
                            411 	.globl _SUDPTRCTL
                            412 	.globl _SUDPTRL
                            413 	.globl _SUDPTRH
                            414 	.globl _EP8FIFOBCL
                            415 	.globl _EP8FIFOBCH
                            416 	.globl _EP6FIFOBCL
                            417 	.globl _EP6FIFOBCH
                            418 	.globl _EP4FIFOBCL
                            419 	.globl _EP4FIFOBCH
                            420 	.globl _EP2FIFOBCL
                            421 	.globl _EP2FIFOBCH
                            422 	.globl _EP8FIFOFLGS
                            423 	.globl _EP6FIFOFLGS
                            424 	.globl _EP4FIFOFLGS
                            425 	.globl _EP2FIFOFLGS
                            426 	.globl _EP8CS
                            427 	.globl _EP6CS
                            428 	.globl _EP4CS
                            429 	.globl _EP2CS
                            430 	.globl _EPXCS
                            431 	.globl _EP1INCS
                            432 	.globl _EP1OUTCS
                            433 	.globl _EP0CS
                            434 	.globl _EP8BCL
                            435 	.globl _EP8BCH
                            436 	.globl _EP6BCL
                            437 	.globl _EP6BCH
                            438 	.globl _EP4BCL
                            439 	.globl _EP4BCH
                            440 	.globl _EP2BCL
                            441 	.globl _EP2BCH
                            442 	.globl _EP1INBC
                            443 	.globl _EP1OUTBC
                            444 	.globl _EP0BCL
                            445 	.globl _EP0BCH
                            446 	.globl _FNADDR
                            447 	.globl _MICROFRAME
                            448 	.globl _USBFRAMEL
                            449 	.globl _USBFRAMEH
                            450 	.globl _TOGCTL
                            451 	.globl _WAKEUPCS
                            452 	.globl _SUSPEND
                            453 	.globl _USBCS
                            454 	.globl _UDMACRCQUALIFIER
                            455 	.globl _UDMACRCL
                            456 	.globl _UDMACRCH
                            457 	.globl _EXTAUTODAT2
                            458 	.globl _XAUTODAT2
                            459 	.globl _EXTAUTODAT1
                            460 	.globl _XAUTODAT1
                            461 	.globl _I2CTL
                            462 	.globl _I2DAT
                            463 	.globl _I2CS
                            464 	.globl _PORTECFG
                            465 	.globl _PORTCCFG
                            466 	.globl _PORTACFG
                            467 	.globl _INTSETUP
                            468 	.globl _INT4IVEC
                            469 	.globl _INT2IVEC
                            470 	.globl _CLRERRCNT
                            471 	.globl _ERRCNTLIM
                            472 	.globl _USBERRIRQ
                            473 	.globl _USBERRIE
                            474 	.globl _GPIFIRQ
                            475 	.globl _GPIFIE
                            476 	.globl _EPIRQ
                            477 	.globl _EPIE
                            478 	.globl _USBIRQ
                            479 	.globl _USBIE
                            480 	.globl _NAKIRQ
                            481 	.globl _NAKIE
                            482 	.globl _IBNIRQ
                            483 	.globl _IBNIE
                            484 	.globl _EP8FIFOIRQ
                            485 	.globl _EP8FIFOIE
                            486 	.globl _EP6FIFOIRQ
                            487 	.globl _EP6FIFOIE
                            488 	.globl _EP4FIFOIRQ
                            489 	.globl _EP4FIFOIE
                            490 	.globl _EP2FIFOIRQ
                            491 	.globl _EP2FIFOIE
                            492 	.globl _OUTPKTEND
                            493 	.globl _INPKTEND
                            494 	.globl _EP8ISOINPKTS
                            495 	.globl _EP6ISOINPKTS
                            496 	.globl _EP4ISOINPKTS
                            497 	.globl _EP2ISOINPKTS
                            498 	.globl _EP8FIFOPFL
                            499 	.globl _EP8FIFOPFH
                            500 	.globl _EP6FIFOPFL
                            501 	.globl _EP6FIFOPFH
                            502 	.globl _EP4FIFOPFL
                            503 	.globl _EP4FIFOPFH
                            504 	.globl _EP2FIFOPFL
                            505 	.globl _EP2FIFOPFH
                            506 	.globl _ECC2B2
                            507 	.globl _ECC2B1
                            508 	.globl _ECC2B0
                            509 	.globl _ECC1B2
                            510 	.globl _ECC1B1
                            511 	.globl _ECC1B0
                            512 	.globl _ECCRESET
                            513 	.globl _ECCCFG
                            514 	.globl _EP8AUTOINLENL
                            515 	.globl _EP8AUTOINLENH
                            516 	.globl _EP6AUTOINLENL
                            517 	.globl _EP6AUTOINLENH
                            518 	.globl _EP4AUTOINLENL
                            519 	.globl _EP4AUTOINLENH
                            520 	.globl _EP2AUTOINLENL
                            521 	.globl _EP2AUTOINLENH
                            522 	.globl _EP8FIFOCFG
                            523 	.globl _EP6FIFOCFG
                            524 	.globl _EP4FIFOCFG
                            525 	.globl _EP2FIFOCFG
                            526 	.globl _EP8CFG
                            527 	.globl _EP6CFG
                            528 	.globl _EP4CFG
                            529 	.globl _EP2CFG
                            530 	.globl _EP1INCFG
                            531 	.globl _EP1OUTCFG
                            532 	.globl _GPIFHOLDAMOUNT
                            533 	.globl _REVCTL
                            534 	.globl _REVID
                            535 	.globl _FIFOPINPOLAR
                            536 	.globl _UART230
                            537 	.globl _BPADDRL
                            538 	.globl _BPADDRH
                            539 	.globl _BREAKPT
                            540 	.globl _FIFORESET
                            541 	.globl _PINFLAGSCD
                            542 	.globl _PINFLAGSAB
                            543 	.globl _IFCONFIG
                            544 	.globl _CPUCS
                            545 	.globl _GPCR2
                            546 	.globl _GPIF_WAVE3_DATA
                            547 	.globl _GPIF_WAVE2_DATA
                            548 	.globl _GPIF_WAVE1_DATA
                            549 	.globl _GPIF_WAVE0_DATA
                            550 	.globl _GPIF_WAVE_DATA
                            551 	.globl _flash_write_PARM_2
                            552 	.globl _mac_eeprom_write_PARM_3
                            553 	.globl _mac_eeprom_write_PARM_2
                            554 	.globl _mac_eeprom_read_PARM_3
                            555 	.globl _mac_eeprom_read_PARM_2
                            556 	.globl _eeprom_write_PARM_3
                            557 	.globl _eeprom_write_PARM_2
                            558 	.globl _eeprom_read_PARM_3
                            559 	.globl _eeprom_read_PARM_2
                            560 	.globl _eeprom_select_PARM_3
                            561 	.globl _eeprom_select_PARM_2
                            562 ;--------------------------------------------------------
                            563 ; special function registers
                            564 ;--------------------------------------------------------
                            565 	.area RSEG    (DATA)
                    0080    566 _IOA	=	0x0080
                    0081    567 _SP	=	0x0081
                    0082    568 _DPL0	=	0x0082
                    0083    569 _DPH0	=	0x0083
                    0084    570 _DPL1	=	0x0084
                    0085    571 _DPH1	=	0x0085
                    0086    572 _DPS	=	0x0086
                    0087    573 _PCON	=	0x0087
                    0088    574 _TCON	=	0x0088
                    0089    575 _TMOD	=	0x0089
                    008A    576 _TL0	=	0x008a
                    008B    577 _TL1	=	0x008b
                    008C    578 _TH0	=	0x008c
                    008D    579 _TH1	=	0x008d
                    008E    580 _CKCON	=	0x008e
                    0090    581 _IOB	=	0x0090
                    0091    582 _EXIF	=	0x0091
                    0092    583 _MPAGE	=	0x0092
                    0092    584 __XPAGE	=	0x0092
                    0098    585 _SCON0	=	0x0098
                    0099    586 _SBUF0	=	0x0099
                    009A    587 _AUTOPTRH1	=	0x009a
                    009B    588 _AUTOPTRL1	=	0x009b
                    009D    589 _AUTOPTRH2	=	0x009d
                    009E    590 _AUTOPTRL2	=	0x009e
                    00A0    591 _IOC	=	0x00a0
                    00A1    592 _INT2CLR	=	0x00a1
                    00A2    593 _INT4CLR	=	0x00a2
                    00A8    594 _IE	=	0x00a8
                    00AA    595 _EP2468STAT	=	0x00aa
                    00AB    596 _EP24FIFOFLGS	=	0x00ab
                    00AC    597 _EP68FIFOFLGS	=	0x00ac
                    00AF    598 _AUTOPTRSETUP	=	0x00af
                    00B0    599 _IOD	=	0x00b0
                    00B1    600 _IOE	=	0x00b1
                    00B2    601 _OEA	=	0x00b2
                    00B3    602 _OEB	=	0x00b3
                    00B4    603 _OEC	=	0x00b4
                    00B5    604 _OED	=	0x00b5
                    00B6    605 _OEE	=	0x00b6
                    00B8    606 _IP	=	0x00b8
                    00BA    607 _EP01STAT	=	0x00ba
                    00BB    608 _GPIFTRIG	=	0x00bb
                    00BD    609 _GPIFSGLDATH	=	0x00bd
                    00BE    610 _GPIFSGLDATLX	=	0x00be
                    00BF    611 _GPIFSGLDATLNOX	=	0x00bf
                    00C0    612 _SCON1	=	0x00c0
                    00C1    613 _SBUF1	=	0x00c1
                    00C8    614 _T2CON	=	0x00c8
                    00CA    615 _RCAP2L	=	0x00ca
                    00CB    616 _RCAP2H	=	0x00cb
                    00CC    617 _TL2	=	0x00cc
                    00CD    618 _TH2	=	0x00cd
                    00D0    619 _PSW	=	0x00d0
                    00D8    620 _EICON	=	0x00d8
                    00E0    621 _ACC	=	0x00e0
                    00E8    622 _EIE	=	0x00e8
                    00F0    623 _BREG	=	0x00f0
                    00F8    624 _EIP	=	0x00f8
                            625 ;--------------------------------------------------------
                            626 ; special function bits
                            627 ;--------------------------------------------------------
                            628 	.area RSEG    (DATA)
                    0080    629 _IOA0	=	0x0080
                    0081    630 _IOA1	=	0x0081
                    0082    631 _IOA2	=	0x0082
                    0083    632 _IOA3	=	0x0083
                    0084    633 _IOA4	=	0x0084
                    0085    634 _IOA5	=	0x0085
                    0086    635 _IOA6	=	0x0086
                    0087    636 _IOA7	=	0x0087
                    0088    637 _IT0	=	0x0088
                    0089    638 _IE0	=	0x0089
                    008A    639 _IT1	=	0x008a
                    008B    640 _IE1	=	0x008b
                    008C    641 _TR0	=	0x008c
                    008D    642 _TF0	=	0x008d
                    008E    643 _TR1	=	0x008e
                    008F    644 _TF1	=	0x008f
                    0090    645 _IOB0	=	0x0090
                    0091    646 _IOB1	=	0x0091
                    0092    647 _IOB2	=	0x0092
                    0093    648 _IOB3	=	0x0093
                    0094    649 _IOB4	=	0x0094
                    0095    650 _IOB5	=	0x0095
                    0096    651 _IOB6	=	0x0096
                    0097    652 _IOB7	=	0x0097
                    0098    653 _RI_0	=	0x0098
                    0099    654 _TI_0	=	0x0099
                    009A    655 _RB8_0	=	0x009a
                    009B    656 _TB8_0	=	0x009b
                    009C    657 _REN_0	=	0x009c
                    009D    658 _SM2_0	=	0x009d
                    009E    659 _SM1_0	=	0x009e
                    009F    660 _SM0_0	=	0x009f
                    00A0    661 _IOC0	=	0x00a0
                    00A1    662 _IOC1	=	0x00a1
                    00A2    663 _IOC2	=	0x00a2
                    00A3    664 _IOC3	=	0x00a3
                    00A4    665 _IOC4	=	0x00a4
                    00A5    666 _IOC5	=	0x00a5
                    00A6    667 _IOC6	=	0x00a6
                    00A7    668 _IOC7	=	0x00a7
                    00A8    669 _EX0	=	0x00a8
                    00A9    670 _ET0	=	0x00a9
                    00AA    671 _EX1	=	0x00aa
                    00AB    672 _ET1	=	0x00ab
                    00AC    673 _ES0	=	0x00ac
                    00AD    674 _ET2	=	0x00ad
                    00AE    675 _ES1	=	0x00ae
                    00AF    676 _EA	=	0x00af
                    00B0    677 _IOD0	=	0x00b0
                    00B1    678 _IOD1	=	0x00b1
                    00B2    679 _IOD2	=	0x00b2
                    00B3    680 _IOD3	=	0x00b3
                    00B4    681 _IOD4	=	0x00b4
                    00B5    682 _IOD5	=	0x00b5
                    00B6    683 _IOD6	=	0x00b6
                    00B7    684 _IOD7	=	0x00b7
                    00B8    685 _PX0	=	0x00b8
                    00B9    686 _PT0	=	0x00b9
                    00BA    687 _PX1	=	0x00ba
                    00BB    688 _PT1	=	0x00bb
                    00BC    689 _PS0	=	0x00bc
                    00BD    690 _PT2	=	0x00bd
                    00BE    691 _PS1	=	0x00be
                    00C0    692 _RI_1	=	0x00c0
                    00C1    693 _TI_1	=	0x00c1
                    00C2    694 _RB8_1	=	0x00c2
                    00C3    695 _TB8_1	=	0x00c3
                    00C4    696 _REN_1	=	0x00c4
                    00C5    697 _SM2_1	=	0x00c5
                    00C6    698 _SM1_1	=	0x00c6
                    00C7    699 _SM0_1	=	0x00c7
                    00C8    700 _CPRL2	=	0x00c8
                    00C9    701 _CT2	=	0x00c9
                    00CA    702 _TR2	=	0x00ca
                    00CB    703 _EXEN2	=	0x00cb
                    00CC    704 _TCLK	=	0x00cc
                    00CD    705 _RCLK	=	0x00cd
                    00CE    706 _EXF2	=	0x00ce
                    00CF    707 _TF2	=	0x00cf
                    00D0    708 _PF	=	0x00d0
                    00D1    709 _F1	=	0x00d1
                    00D2    710 _OV	=	0x00d2
                    00D3    711 _RS0	=	0x00d3
                    00D4    712 _RS1	=	0x00d4
                    00D5    713 _F0	=	0x00d5
                    00D6    714 _AC	=	0x00d6
                    00D7    715 _CY	=	0x00d7
                    00DB    716 _INT6	=	0x00db
                    00DC    717 _RESI	=	0x00dc
                    00DD    718 _ERESI	=	0x00dd
                    00DF    719 _SMOD1	=	0x00df
                    00E0    720 _ACC0	=	0x00e0
                    00E1    721 _ACC1	=	0x00e1
                    00E2    722 _ACC2	=	0x00e2
                    00E3    723 _ACC3	=	0x00e3
                    00E4    724 _ACC4	=	0x00e4
                    00E5    725 _ACC5	=	0x00e5
                    00E6    726 _ACC6	=	0x00e6
                    00E7    727 _ACC7	=	0x00e7
                    00E8    728 _EUSB	=	0x00e8
                    00E9    729 _EI2C	=	0x00e9
                    00EA    730 _EIEX4	=	0x00ea
                    00EB    731 _EIEX5	=	0x00eb
                    00EC    732 _EIEX6	=	0x00ec
                    00F0    733 _BREG0	=	0x00f0
                    00F1    734 _BREG1	=	0x00f1
                    00F2    735 _BREG2	=	0x00f2
                    00F3    736 _BREG3	=	0x00f3
                    00F4    737 _BREG4	=	0x00f4
                    00F5    738 _BREG5	=	0x00f5
                    00F6    739 _BREG6	=	0x00f6
                    00F7    740 _BREG7	=	0x00f7
                    00F8    741 _PUSB	=	0x00f8
                    00F9    742 _PI2C	=	0x00f9
                    00FA    743 _EIPX4	=	0x00fa
                    00FB    744 _EIPX5	=	0x00fb
                    00FC    745 _EIPX6	=	0x00fc
                            746 ;--------------------------------------------------------
                            747 ; overlayable register banks
                            748 ;--------------------------------------------------------
                            749 	.area REG_BANK_0	(REL,OVR,DATA)
   0000                     750 	.ds 8
                            751 ;--------------------------------------------------------
                            752 ; overlayable bit register bank
                            753 ;--------------------------------------------------------
                            754 	.area BIT_BANK	(REL,OVR,DATA)
   0020                     755 bits:
   0020                     756 	.ds 1
                    8000    757 	b0 = bits[0]
                    8100    758 	b1 = bits[1]
                    8200    759 	b2 = bits[2]
                    8300    760 	b3 = bits[3]
                    8400    761 	b4 = bits[4]
                    8500    762 	b5 = bits[5]
                    8600    763 	b6 = bits[6]
                    8700    764 	b7 = bits[7]
                            765 ;--------------------------------------------------------
                            766 ; internal ram data
                            767 ;--------------------------------------------------------
                            768 	.area DSEG    (DATA)
   0008                     769 _eeprom_select_PARM_2:
   0008                     770 	.ds 1
   0009                     771 _eeprom_select_PARM_3:
   0009                     772 	.ds 1
   000A                     773 _eeprom_read_PARM_2:
   000A                     774 	.ds 2
   000C                     775 _eeprom_read_PARM_3:
   000C                     776 	.ds 1
   000D                     777 _eeprom_write_PARM_2:
   000D                     778 	.ds 2
   000F                     779 _eeprom_write_PARM_3:
   000F                     780 	.ds 1
   0010                     781 _mac_eeprom_read_PARM_2:
   0010                     782 	.ds 1
   0011                     783 _mac_eeprom_read_PARM_3:
   0011                     784 	.ds 1
   0012                     785 _mac_eeprom_write_PARM_2:
   0012                     786 	.ds 1
   0013                     787 _mac_eeprom_write_PARM_3:
   0013                     788 	.ds 1
   0014                     789 _flash_write_PARM_2:
   0014                     790 	.ds 1
   0015                     791 _fpga_send_ep0_oOEB_1_1:
   0015                     792 	.ds 1
                            793 ;--------------------------------------------------------
                            794 ; overlayable items in internal ram 
                            795 ;--------------------------------------------------------
                            796 	.area	OSEG    (OVR,DATA)
                            797 	.area	OSEG    (OVR,DATA)
                            798 	.area	OSEG    (OVR,DATA)
                            799 	.area	OSEG    (OVR,DATA)
                            800 	.area	OSEG    (OVR,DATA)
                            801 	.area	OSEG    (OVR,DATA)
                            802 	.area	OSEG    (OVR,DATA)
                            803 	.area	OSEG    (OVR,DATA)
   0016                     804 _flash_read_PARM_2::
   0016                     805 	.ds 1
                            806 	.area	OSEG    (OVR,DATA)
                            807 	.area	OSEG    (OVR,DATA)
   0016                     808 _spi_write_PARM_2::
   0016                     809 	.ds 1
                            810 	.area	OSEG    (OVR,DATA)
   0016                     811 _sendStringDescriptor_PARM_2::
   0016                     812 	.ds 1
   0017                     813 _sendStringDescriptor_PARM_3::
   0017                     814 	.ds 1
                            815 ;--------------------------------------------------------
                            816 ; Stack segment in internal ram 
                            817 ;--------------------------------------------------------
                            818 	.area	SSEG	(DATA)
   0021                     819 __start__stack:
   0021                     820 	.ds	1
                            821 
                            822 ;--------------------------------------------------------
                            823 ; indirectly addressable internal ram data
                            824 ;--------------------------------------------------------
                            825 	.area ISEG    (DATA)
                            826 ;--------------------------------------------------------
                            827 ; absolute internal ram data
                            828 ;--------------------------------------------------------
                            829 	.area IABS    (ABS,DATA)
                            830 	.area IABS    (ABS,DATA)
                            831 ;--------------------------------------------------------
                            832 ; bit data
                            833 ;--------------------------------------------------------
                            834 	.area BSEG    (BIT)
                            835 ;--------------------------------------------------------
                            836 ; paged external ram data
                            837 ;--------------------------------------------------------
                            838 	.area PSEG    (PAG,XDATA)
                            839 ;--------------------------------------------------------
                            840 ; external ram data
                            841 ;--------------------------------------------------------
                            842 	.area XSEG    (XDATA)
                    E400    843 _GPIF_WAVE_DATA	=	0xe400
                    E400    844 _GPIF_WAVE0_DATA	=	0xe400
                    E420    845 _GPIF_WAVE1_DATA	=	0xe420
                    E440    846 _GPIF_WAVE2_DATA	=	0xe440
                    E460    847 _GPIF_WAVE3_DATA	=	0xe460
                    E50D    848 _GPCR2	=	0xe50d
                    E600    849 _CPUCS	=	0xe600
                    E601    850 _IFCONFIG	=	0xe601
                    E602    851 _PINFLAGSAB	=	0xe602
                    E603    852 _PINFLAGSCD	=	0xe603
                    E604    853 _FIFORESET	=	0xe604
                    E605    854 _BREAKPT	=	0xe605
                    E606    855 _BPADDRH	=	0xe606
                    E607    856 _BPADDRL	=	0xe607
                    E608    857 _UART230	=	0xe608
                    E609    858 _FIFOPINPOLAR	=	0xe609
                    E60A    859 _REVID	=	0xe60a
                    E60B    860 _REVCTL	=	0xe60b
                    E60C    861 _GPIFHOLDAMOUNT	=	0xe60c
                    E610    862 _EP1OUTCFG	=	0xe610
                    E611    863 _EP1INCFG	=	0xe611
                    E612    864 _EP2CFG	=	0xe612
                    E613    865 _EP4CFG	=	0xe613
                    E614    866 _EP6CFG	=	0xe614
                    E615    867 _EP8CFG	=	0xe615
                    E618    868 _EP2FIFOCFG	=	0xe618
                    E619    869 _EP4FIFOCFG	=	0xe619
                    E61A    870 _EP6FIFOCFG	=	0xe61a
                    E61B    871 _EP8FIFOCFG	=	0xe61b
                    E620    872 _EP2AUTOINLENH	=	0xe620
                    E621    873 _EP2AUTOINLENL	=	0xe621
                    E622    874 _EP4AUTOINLENH	=	0xe622
                    E623    875 _EP4AUTOINLENL	=	0xe623
                    E624    876 _EP6AUTOINLENH	=	0xe624
                    E625    877 _EP6AUTOINLENL	=	0xe625
                    E626    878 _EP8AUTOINLENH	=	0xe626
                    E627    879 _EP8AUTOINLENL	=	0xe627
                    E628    880 _ECCCFG	=	0xe628
                    E629    881 _ECCRESET	=	0xe629
                    E62A    882 _ECC1B0	=	0xe62a
                    E62B    883 _ECC1B1	=	0xe62b
                    E62C    884 _ECC1B2	=	0xe62c
                    E62D    885 _ECC2B0	=	0xe62d
                    E62E    886 _ECC2B1	=	0xe62e
                    E62F    887 _ECC2B2	=	0xe62f
                    E630    888 _EP2FIFOPFH	=	0xe630
                    E631    889 _EP2FIFOPFL	=	0xe631
                    E632    890 _EP4FIFOPFH	=	0xe632
                    E633    891 _EP4FIFOPFL	=	0xe633
                    E634    892 _EP6FIFOPFH	=	0xe634
                    E635    893 _EP6FIFOPFL	=	0xe635
                    E636    894 _EP8FIFOPFH	=	0xe636
                    E637    895 _EP8FIFOPFL	=	0xe637
                    E640    896 _EP2ISOINPKTS	=	0xe640
                    E641    897 _EP4ISOINPKTS	=	0xe641
                    E642    898 _EP6ISOINPKTS	=	0xe642
                    E643    899 _EP8ISOINPKTS	=	0xe643
                    E648    900 _INPKTEND	=	0xe648
                    E649    901 _OUTPKTEND	=	0xe649
                    E650    902 _EP2FIFOIE	=	0xe650
                    E651    903 _EP2FIFOIRQ	=	0xe651
                    E652    904 _EP4FIFOIE	=	0xe652
                    E653    905 _EP4FIFOIRQ	=	0xe653
                    E654    906 _EP6FIFOIE	=	0xe654
                    E655    907 _EP6FIFOIRQ	=	0xe655
                    E656    908 _EP8FIFOIE	=	0xe656
                    E657    909 _EP8FIFOIRQ	=	0xe657
                    E658    910 _IBNIE	=	0xe658
                    E659    911 _IBNIRQ	=	0xe659
                    E65A    912 _NAKIE	=	0xe65a
                    E65B    913 _NAKIRQ	=	0xe65b
                    E65C    914 _USBIE	=	0xe65c
                    E65D    915 _USBIRQ	=	0xe65d
                    E65E    916 _EPIE	=	0xe65e
                    E65F    917 _EPIRQ	=	0xe65f
                    E660    918 _GPIFIE	=	0xe660
                    E661    919 _GPIFIRQ	=	0xe661
                    E662    920 _USBERRIE	=	0xe662
                    E663    921 _USBERRIRQ	=	0xe663
                    E664    922 _ERRCNTLIM	=	0xe664
                    E665    923 _CLRERRCNT	=	0xe665
                    E666    924 _INT2IVEC	=	0xe666
                    E667    925 _INT4IVEC	=	0xe667
                    E668    926 _INTSETUP	=	0xe668
                    E670    927 _PORTACFG	=	0xe670
                    E671    928 _PORTCCFG	=	0xe671
                    E672    929 _PORTECFG	=	0xe672
                    E678    930 _I2CS	=	0xe678
                    E679    931 _I2DAT	=	0xe679
                    E67A    932 _I2CTL	=	0xe67a
                    E67B    933 _XAUTODAT1	=	0xe67b
                    E67B    934 _EXTAUTODAT1	=	0xe67b
                    E67C    935 _XAUTODAT2	=	0xe67c
                    E67C    936 _EXTAUTODAT2	=	0xe67c
                    E67D    937 _UDMACRCH	=	0xe67d
                    E67E    938 _UDMACRCL	=	0xe67e
                    E67F    939 _UDMACRCQUALIFIER	=	0xe67f
                    E680    940 _USBCS	=	0xe680
                    E681    941 _SUSPEND	=	0xe681
                    E682    942 _WAKEUPCS	=	0xe682
                    E683    943 _TOGCTL	=	0xe683
                    E684    944 _USBFRAMEH	=	0xe684
                    E685    945 _USBFRAMEL	=	0xe685
                    E686    946 _MICROFRAME	=	0xe686
                    E687    947 _FNADDR	=	0xe687
                    E68A    948 _EP0BCH	=	0xe68a
                    E68B    949 _EP0BCL	=	0xe68b
                    E68D    950 _EP1OUTBC	=	0xe68d
                    E68F    951 _EP1INBC	=	0xe68f
                    E690    952 _EP2BCH	=	0xe690
                    E691    953 _EP2BCL	=	0xe691
                    E694    954 _EP4BCH	=	0xe694
                    E695    955 _EP4BCL	=	0xe695
                    E698    956 _EP6BCH	=	0xe698
                    E699    957 _EP6BCL	=	0xe699
                    E69C    958 _EP8BCH	=	0xe69c
                    E69D    959 _EP8BCL	=	0xe69d
                    E6A0    960 _EP0CS	=	0xe6a0
                    E6A1    961 _EP1OUTCS	=	0xe6a1
                    E6A2    962 _EP1INCS	=	0xe6a2
                    E6A3    963 _EPXCS	=	0xe6a3
                    E6A3    964 _EP2CS	=	0xe6a3
                    E6A4    965 _EP4CS	=	0xe6a4
                    E6A5    966 _EP6CS	=	0xe6a5
                    E6A6    967 _EP8CS	=	0xe6a6
                    E6A7    968 _EP2FIFOFLGS	=	0xe6a7
                    E6A8    969 _EP4FIFOFLGS	=	0xe6a8
                    E6A9    970 _EP6FIFOFLGS	=	0xe6a9
                    E6AA    971 _EP8FIFOFLGS	=	0xe6aa
                    E6AB    972 _EP2FIFOBCH	=	0xe6ab
                    E6AC    973 _EP2FIFOBCL	=	0xe6ac
                    E6AD    974 _EP4FIFOBCH	=	0xe6ad
                    E6AE    975 _EP4FIFOBCL	=	0xe6ae
                    E6AF    976 _EP6FIFOBCH	=	0xe6af
                    E6B0    977 _EP6FIFOBCL	=	0xe6b0
                    E6B1    978 _EP8FIFOBCH	=	0xe6b1
                    E6B2    979 _EP8FIFOBCL	=	0xe6b2
                    E6B3    980 _SUDPTRH	=	0xe6b3
                    E6B4    981 _SUDPTRL	=	0xe6b4
                    E6B5    982 _SUDPTRCTL	=	0xe6b5
                    E6B8    983 _SETUPDAT	=	0xe6b8
                    E6B8    984 _bmRequestType	=	0xe6b8
                    E6B9    985 _bRequest	=	0xe6b9
                    E6BA    986 _wValueL	=	0xe6ba
                    E6BB    987 _wValueH	=	0xe6bb
                    E6BC    988 _wIndexL	=	0xe6bc
                    E6BD    989 _wIndexH	=	0xe6bd
                    E6BE    990 _wLengthL	=	0xe6be
                    E6BF    991 _wLengthH	=	0xe6bf
                    E6C0    992 _GPIFWFSELECT	=	0xe6c0
                    E6C1    993 _GPIFIDLECS	=	0xe6c1
                    E6C2    994 _GPIFIDLECTL	=	0xe6c2
                    E6C3    995 _GPIFCTLCFG	=	0xe6c3
                    E6C4    996 _GPIFADRH	=	0xe6c4
                    E6C5    997 _GPIFADRL	=	0xe6c5
                    E6C6    998 _FLOWSTATE	=	0xe6c6
                    E6C7    999 _FLOWLOGIC	=	0xe6c7
                    E6C8   1000 _FLOWEQ0CTL	=	0xe6c8
                    E6C9   1001 _FLOWEQ1CTL	=	0xe6c9
                    E6CA   1002 _FLOWHOLDOFF	=	0xe6ca
                    E6CB   1003 _FLOWSTB	=	0xe6cb
                    E6CC   1004 _FLOWSTBEDGE	=	0xe6cc
                    E6CD   1005 _FLOWSTBHPERIOD	=	0xe6cd
                    E6CE   1006 _GPIFTCB3	=	0xe6ce
                    E6CF   1007 _GPIFTCB2	=	0xe6cf
                    E6D0   1008 _GPIFTCB1	=	0xe6d0
                    E6D1   1009 _GPIFTCB0	=	0xe6d1
                    E6D2   1010 _EP2GPIFFLGSEL	=	0xe6d2
                    E6D3   1011 _EP2GPIFPFSTOP	=	0xe6d3
                    E6D4   1012 _EP2GPIFTRIG	=	0xe6d4
                    E6DA   1013 _EP4GPIFFLGSEL	=	0xe6da
                    E6DB   1014 _EP4GPIFPFSTOP	=	0xe6db
                    E6DC   1015 _EP4GPIFTRIG	=	0xe6dc
                    E6E2   1016 _EP6GPIFFLGSEL	=	0xe6e2
                    E6E3   1017 _EP6GPIFPFSTOP	=	0xe6e3
                    E6E4   1018 _EP6GPIFTRIG	=	0xe6e4
                    E6EA   1019 _EP8GPIFFLGSEL	=	0xe6ea
                    E6EB   1020 _EP8GPIFPFSTOP	=	0xe6eb
                    E6EC   1021 _EP8GPIFTRIG	=	0xe6ec
                    E6F0   1022 _XGPIFSGLDATH	=	0xe6f0
                    E6F1   1023 _XGPIFSGLDATLX	=	0xe6f1
                    E6F2   1024 _XGPIFSGLDATLNOX	=	0xe6f2
                    E6F3   1025 _GPIFREADYCFG	=	0xe6f3
                    E6F4   1026 _GPIFREADYSTAT	=	0xe6f4
                    E6F5   1027 _GPIFABORT	=	0xe6f5
                    E740   1028 _EP0BUF	=	0xe740
                    E780   1029 _EP1OUTBUF	=	0xe780
                    E7C0   1030 _EP1INBUF	=	0xe7c0
                    F000   1031 _EP2FIFOBUF	=	0xf000
                    F400   1032 _EP4FIFOBUF	=	0xf400
                    F800   1033 _EP6FIFOBUF	=	0xf800
                    FC00   1034 _EP8FIFOBUF	=	0xfc00
                    0003   1035 _INT0VEC_IE0	=	0x0003
                    000B   1036 _INT1VEC_T0	=	0x000b
                    0013   1037 _INT2VEC_IE1	=	0x0013
                    001B   1038 _INT3VEC_T1	=	0x001b
                    0023   1039 _INT4VEC_USART0	=	0x0023
                    002B   1040 _INT5VEC_T2	=	0x002b
                    0033   1041 _INT6VEC_RESUME	=	0x0033
                    003B   1042 _INT7VEC_USART1	=	0x003b
                    0043   1043 _INT8VEC_USB	=	0x0043
                    004B   1044 _INT9VEC_I2C	=	0x004b
                    0053   1045 _INT10VEC_GPIF	=	0x0053
                    005B   1046 _INT11VEC_IE5	=	0x005b
                    0063   1047 _INT12VEC_IE6	=	0x0063
                    0100   1048 _INTVEC_SUDAV	=	0x0100
                    0104   1049 _INTVEC_SOF	=	0x0104
                    0108   1050 _INTVEC_SUTOK	=	0x0108
                    010C   1051 _INTVEC_SUSPEND	=	0x010c
                    0110   1052 _INTVEC_USBRESET	=	0x0110
                    0114   1053 _INTVEC_HISPEED	=	0x0114
                    0118   1054 _INTVEC_EP0ACK	=	0x0118
                    0120   1055 _INTVEC_EP0IN	=	0x0120
                    0124   1056 _INTVEC_EP0OUT	=	0x0124
                    0128   1057 _INTVEC_EP1IN	=	0x0128
                    012C   1058 _INTVEC_EP1OUT	=	0x012c
                    0130   1059 _INTVEC_EP2	=	0x0130
                    0134   1060 _INTVEC_EP4	=	0x0134
                    0138   1061 _INTVEC_EP6	=	0x0138
                    013C   1062 _INTVEC_EP8	=	0x013c
                    0140   1063 _INTVEC_IBN	=	0x0140
                    0148   1064 _INTVEC_EP0PING	=	0x0148
                    014C   1065 _INTVEC_EP1PING	=	0x014c
                    0150   1066 _INTVEC_EP2PING	=	0x0150
                    0154   1067 _INTVEC_EP4PING	=	0x0154
                    0158   1068 _INTVEC_EP6PING	=	0x0158
                    015C   1069 _INTVEC_EP8PING	=	0x015c
                    0160   1070 _INTVEC_ERRLIMIT	=	0x0160
                    0170   1071 _INTVEC_EP2ISOERR	=	0x0170
                    0174   1072 _INTVEC_EP4ISOERR	=	0x0174
                    0178   1073 _INTVEC_EP6ISOERR	=	0x0178
                    017C   1074 _INTVEC_EP8ISOERR	=	0x017c
                    0180   1075 _INTVEC_EP2PF	=	0x0180
                    0184   1076 _INTVEC_EP4PF	=	0x0184
                    0188   1077 _INTVEC_EP6PF	=	0x0188
                    018C   1078 _INTVEC_EP8PF	=	0x018c
                    0190   1079 _INTVEC_EP2EF	=	0x0190
                    0194   1080 _INTVEC_EP4EF	=	0x0194
                    0198   1081 _INTVEC_EP6EF	=	0x0198
                    019C   1082 _INTVEC_EP8EF	=	0x019c
                    01A0   1083 _INTVEC_EP2FF	=	0x01a0
                    01A8   1084 _INTVEC_EP6FF	=	0x01a8
                    01AC   1085 _INTVEC_EP8FF	=	0x01ac
                    01B0   1086 _INTVEC_GPIFDONE	=	0x01b0
                    01B4   1087 _INTVEC_GPIFWF	=	0x01b4
   3A00                    1088 _eeprom_addr::
   3A00                    1089 	.ds 2
   3A02                    1090 _eeprom_write_bytes::
   3A02                    1091 	.ds 2
   3A04                    1092 _eeprom_write_checksum::
   3A04                    1093 	.ds 1
   3A05                    1094 _mac_eeprom_addr::
   3A05                    1095 	.ds 1
   3A06                    1096 _config_data_valid::
   3A06                    1097 	.ds 1
   3A07                    1098 _flash_enabled::
   3A07                    1099 	.ds 1
   3A08                    1100 _flash_sector_size::
   3A08                    1101 	.ds 2
   3A0A                    1102 _flash_sectors::
   3A0A                    1103 	.ds 4
   3A0E                    1104 _flash_ec::
   3A0E                    1105 	.ds 1
   3A0F                    1106 _spi_vendor::
   3A0F                    1107 	.ds 1
   3A10                    1108 _spi_device::
   3A10                    1109 	.ds 1
   3A11                    1110 _spi_memtype::
   3A11                    1111 	.ds 1
   3A12                    1112 _spi_erase_cmd::
   3A12                    1113 	.ds 1
   3A13                    1114 _spi_last_cmd::
   3A13                    1115 	.ds 1
   3A14                    1116 _spi_buffer::
   3A14                    1117 	.ds 4
   3A18                    1118 _spi_write_addr_hi::
   3A18                    1119 	.ds 2
   3A1A                    1120 _spi_write_addr_lo::
   3A1A                    1121 	.ds 1
   3A1B                    1122 _spi_need_pp::
   3A1B                    1123 	.ds 1
   3A1C                    1124 _spi_write_sector::
   3A1C                    1125 	.ds 2
   3A1E                    1126 _ep0_read_mode::
   3A1E                    1127 	.ds 1
   3A1F                    1128 _ep0_write_mode::
   3A1F                    1129 	.ds 1
   3A20                    1130 _fpga_checksum::
   3A20                    1131 	.ds 1
   3A21                    1132 _fpga_bytes::
   3A21                    1133 	.ds 4
   3A25                    1134 _fpga_init_b::
   3A25                    1135 	.ds 1
   3A26                    1136 _fpga_flash_result::
   3A26                    1137 	.ds 1
   3A27                    1138 _fpga_conf_initialized::
   3A27                    1139 	.ds 1
   3A28                    1140 _OOEC::
   3A28                    1141 	.ds 1
   3A29                    1142 _fpga_first_free_sector_buf_1_1:
   3A29                    1143 	.ds 4
   3A2D                    1144 _fpga_configure_from_flash_init_buf_1_1:
   3A2D                    1145 	.ds 4
                    006C   1146 _ZTEX_DESCRIPTOR	=	0x006c
                    006D   1147 _ZTEX_DESCRIPTOR_VERSION	=	0x006d
                    006E   1148 _ZTEXID	=	0x006e
                    0072   1149 _PRODUCT_ID	=	0x0072
                    0076   1150 _FW_VERSION	=	0x0076
                    0077   1151 _INTERFACE_VERSION	=	0x0077
                    0078   1152 _INTERFACE_CAPABILITIES	=	0x0078
                    007E   1153 _MODULE_RESERVED	=	0x007e
                    008A   1154 _SN_STRING	=	0x008a
   3A31                    1155 _mac_eeprom_init_buf_1_1:
   3A31                    1156 	.ds 5
                           1157 ;--------------------------------------------------------
                           1158 ; absolute external ram data
                           1159 ;--------------------------------------------------------
                           1160 	.area XABS    (ABS,XDATA)
                           1161 ;--------------------------------------------------------
                           1162 ; external initialized ram data
                           1163 ;--------------------------------------------------------
                           1164 	.area XISEG   (XDATA)
   3A36                    1165 _ep0_payload_remaining::
   3A36                    1166 	.ds 2
   3A38                    1167 _ep0_payload_transfer::
   3A38                    1168 	.ds 1
   3A39                    1169 _GPIF_WAVE_DATA_HSFPGA_24MHZ::
   3A39                    1170 	.ds 32
   3A59                    1171 _GPIF_WAVE_DATA_HSFPGA_12MHZ::
   3A59                    1172 	.ds 32
   3A79                    1173 _ep0_prev_setup_request::
   3A79                    1174 	.ds 1
   3A7A                    1175 _ep0_vendor_cmd_setup::
   3A7A                    1176 	.ds 1
   3A7B                    1177 _ISOFRAME_COUNTER::
   3A7B                    1178 	.ds 8
                           1179 	.area HOME    (CODE)
                           1180 	.area GSINIT0 (CODE)
                           1181 	.area GSINIT1 (CODE)
                           1182 	.area GSINIT2 (CODE)
                           1183 	.area GSINIT3 (CODE)
                           1184 	.area GSINIT4 (CODE)
                           1185 	.area GSINIT5 (CODE)
                           1186 	.area GSINIT  (CODE)
                           1187 	.area GSFINAL (CODE)
                           1188 	.area CSEG    (CODE)
                           1189 ;--------------------------------------------------------
                           1190 ; interrupt vector 
                           1191 ;--------------------------------------------------------
                           1192 	.area HOME    (CODE)
   0200                    1193 __interrupt_vect:
   0200 02 02 08           1194 	ljmp	__sdcc_gsinit_startup
                           1195 ;--------------------------------------------------------
                           1196 ; global & static initialisations
                           1197 ;--------------------------------------------------------
                           1198 	.area HOME    (CODE)
                           1199 	.area GSINIT  (CODE)
                           1200 	.area GSFINAL (CODE)
                           1201 	.area GSINIT  (CODE)
                           1202 	.globl __sdcc_gsinit_startup
                           1203 	.globl __sdcc_program_startup
                           1204 	.globl __start__stack
                           1205 	.globl __mcs51_genXINIT
                           1206 	.globl __mcs51_genXRAMCLEAR
                           1207 	.globl __mcs51_genRAMCLEAR
                           1208 	.area GSFINAL (CODE)
   0261 02 02 03           1209 	ljmp	__sdcc_program_startup
                           1210 ;--------------------------------------------------------
                           1211 ; Home
                           1212 ;--------------------------------------------------------
                           1213 	.area HOME    (CODE)
                           1214 	.area HOME    (CODE)
   0203                    1215 __sdcc_program_startup:
   0203 12 1F 24           1216 	lcall	_main
                           1217 ;	return from main will lock up
   0206 80 FE              1218 	sjmp .
                           1219 ;--------------------------------------------------------
                           1220 ; code
                           1221 ;--------------------------------------------------------
                           1222 	.area CSEG    (CODE)
                           1223 ;------------------------------------------------------------
                           1224 ;Allocation info for local variables in function 'abscode_intvec'
                           1225 ;------------------------------------------------------------
                           1226 ;------------------------------------------------------------
                           1227 ;	../../include/ezintavecs.h:92: void abscode_intvec()// _naked
                           1228 ;	-----------------------------------------
                           1229 ;	 function abscode_intvec
                           1230 ;	-----------------------------------------
   0264                    1231 _abscode_intvec:
                    0002   1232 	ar2 = 0x02
                    0003   1233 	ar3 = 0x03
                    0004   1234 	ar4 = 0x04
                    0005   1235 	ar5 = 0x05
                    0006   1236 	ar6 = 0x06
                    0007   1237 	ar7 = 0x07
                    0000   1238 	ar0 = 0x00
                    0001   1239 	ar1 = 0x01
                           1240 ;	../../include/ezintavecs.h:317: ERROR: no line number 317 in file ../../include/ezintavecs.h
                           1241 	
                           1242 	    .area ABSCODE (ABS,CODE)
   0000                    1243 	    .org 0x0000
   0000                    1244 	ENTRY:
   0000 02 02 00           1245 	 ljmp #0x0200
                           1246 ;	# 94 "../../include/ezintavecs.h"
   0003                    1247 	    .org 0x0003
                           1248 ;	# 34 "../../include/ezintavecs.h"
   0003 32                 1249 	 reti
                           1250 ;	# 94 "../../include/ezintavecs.h"
   000B                    1251 	    .org 0x000b
                           1252 ;	# 35 "../../include/ezintavecs.h"
   000B 32                 1253 	 reti
                           1254 ;	# 94 "../../include/ezintavecs.h"
   0013                    1255 	    .org 0x0013
                           1256 ;	# 36 "../../include/ezintavecs.h"
   0013 32                 1257 	 reti
                           1258 ;	# 94 "../../include/ezintavecs.h"
   001B                    1259 	    .org 0x001b
                           1260 ;	# 37 "../../include/ezintavecs.h"
   001B 32                 1261 	 reti
                           1262 ;	# 94 "../../include/ezintavecs.h"
   0023                    1263 	    .org 0x0023
                           1264 ;	# 38 "../../include/ezintavecs.h"
   0023 32                 1265 	 reti
                           1266 ;	# 94 "../../include/ezintavecs.h"
   002B                    1267 	    .org 0x002b
                           1268 ;	# 39 "../../include/ezintavecs.h"
   002B 32                 1269 	 reti
                           1270 ;	# 94 "../../include/ezintavecs.h"
   0033                    1271 	    .org 0x0033
                           1272 ;	# 40 "../../include/ezintavecs.h"
   0033 32                 1273 	 reti
                           1274 ;	# 94 "../../include/ezintavecs.h"
   003B                    1275 	    .org 0x003b
                           1276 ;	# 41 "../../include/ezintavecs.h"
   003B 32                 1277 	 reti
                           1278 ;	# 94 "../../include/ezintavecs.h"
   0043                    1279 	    .org 0x0043
                           1280 ;	# 42 "../../include/ezintavecs.h"
   0043 32                 1281 	 reti
                           1282 ;	# 94 "../../include/ezintavecs.h"
   004B                    1283 	    .org 0x004b
                           1284 ;	# 43 "../../include/ezintavecs.h"
   004B 32                 1285 	 reti
                           1286 ;	# 94 "../../include/ezintavecs.h"
   0053                    1287 	    .org 0x0053
                           1288 ;	# 44 "../../include/ezintavecs.h"
   0053 32                 1289 	 reti
                           1290 ;	# 94 "../../include/ezintavecs.h"
   005B                    1291 	    .org 0x005b
                           1292 ;	# 45 "../../include/ezintavecs.h"
   005B 32                 1293 	 reti
                           1294 ;	# 94 "../../include/ezintavecs.h"
   0063                    1295 	    .org 0x0063
                           1296 ;	# 46 "../../include/ezintavecs.h"
   0063 32                 1297 	 reti
                           1298 ;	# 94 "../../include/ezintavecs.h"
   0100                    1299 	    .org 0x0100
                           1300 ;	# 47 "../../include/ezintavecs.h"
   0100 32                 1301 	 reti
                           1302 ;	# 94 "../../include/ezintavecs.h"
   0104                    1303 	    .org 0x0104
                           1304 ;	# 48 "../../include/ezintavecs.h"
   0104 32                 1305 	 reti
                           1306 ;	# 94 "../../include/ezintavecs.h"
   0108                    1307 	    .org 0x0108
                           1308 ;	# 49 "../../include/ezintavecs.h"
   0108 32                 1309 	 reti
                           1310 ;	# 94 "../../include/ezintavecs.h"
   010C                    1311 	    .org 0x010C
                           1312 ;	# 50 "../../include/ezintavecs.h"
   010C 32                 1313 	 reti
                           1314 ;	# 94 "../../include/ezintavecs.h"
   0110                    1315 	    .org 0x0110
                           1316 ;	# 51 "../../include/ezintavecs.h"
   0110 32                 1317 	 reti
                           1318 ;	# 94 "../../include/ezintavecs.h"
   0114                    1319 	    .org 0x0114
                           1320 ;	# 52 "../../include/ezintavecs.h"
   0114 32                 1321 	 reti
                           1322 ;	# 94 "../../include/ezintavecs.h"
   0118                    1323 	    .org 0x0118
                           1324 ;	# 53 "../../include/ezintavecs.h"
   0118 32                 1325 	 reti
                           1326 ;	# 94 "../../include/ezintavecs.h"
   0120                    1327 	    .org 0x0120
                           1328 ;	# 54 "../../include/ezintavecs.h"
   0120 32                 1329 	 reti
                           1330 ;	# 94 "../../include/ezintavecs.h"
   0124                    1331 	    .org 0x0124
                           1332 ;	# 55 "../../include/ezintavecs.h"
   0124 32                 1333 	 reti
                           1334 ;	# 94 "../../include/ezintavecs.h"
   0128                    1335 	    .org 0x0128
                           1336 ;	# 56 "../../include/ezintavecs.h"
   0128 32                 1337 	 reti
                           1338 ;	# 94 "../../include/ezintavecs.h"
   012C                    1339 	    .org 0x012C
                           1340 ;	# 57 "../../include/ezintavecs.h"
   012C 32                 1341 	 reti
                           1342 ;	# 94 "../../include/ezintavecs.h"
   0130                    1343 	    .org 0x0130
                           1344 ;	# 58 "../../include/ezintavecs.h"
   0130 32                 1345 	 reti
                           1346 ;	# 94 "../../include/ezintavecs.h"
   0134                    1347 	    .org 0x0134
                           1348 ;	# 59 "../../include/ezintavecs.h"
   0134 32                 1349 	 reti
                           1350 ;	# 94 "../../include/ezintavecs.h"
   0138                    1351 	    .org 0x0138
                           1352 ;	# 60 "../../include/ezintavecs.h"
   0138 32                 1353 	 reti
                           1354 ;	# 94 "../../include/ezintavecs.h"
   013C                    1355 	    .org 0x013C
                           1356 ;	# 61 "../../include/ezintavecs.h"
   013C 32                 1357 	 reti
                           1358 ;	# 94 "../../include/ezintavecs.h"
   0140                    1359 	    .org 0x0140
                           1360 ;	# 62 "../../include/ezintavecs.h"
   0140 32                 1361 	 reti
                           1362 ;	# 94 "../../include/ezintavecs.h"
   0148                    1363 	    .org 0x0148
                           1364 ;	# 63 "../../include/ezintavecs.h"
   0148 32                 1365 	 reti
                           1366 ;	# 94 "../../include/ezintavecs.h"
   014C                    1367 	    .org 0x014C
                           1368 ;	# 64 "../../include/ezintavecs.h"
   014C 32                 1369 	 reti
                           1370 ;	# 94 "../../include/ezintavecs.h"
   0150                    1371 	    .org 0x0150
                           1372 ;	# 65 "../../include/ezintavecs.h"
   0150 32                 1373 	 reti
                           1374 ;	# 94 "../../include/ezintavecs.h"
   0154                    1375 	    .org 0x0154
                           1376 ;	# 66 "../../include/ezintavecs.h"
   0154 32                 1377 	 reti
                           1378 ;	# 94 "../../include/ezintavecs.h"
   0158                    1379 	    .org 0x0158
                           1380 ;	# 67 "../../include/ezintavecs.h"
   0158 32                 1381 	 reti
                           1382 ;	# 94 "../../include/ezintavecs.h"
   015C                    1383 	    .org 0x015C
                           1384 ;	# 68 "../../include/ezintavecs.h"
   015C 32                 1385 	 reti
                           1386 ;	# 94 "../../include/ezintavecs.h"
   0160                    1387 	    .org 0x0160
                           1388 ;	# 69 "../../include/ezintavecs.h"
   0160 32                 1389 	 reti
                           1390 ;	# 94 "../../include/ezintavecs.h"
   0170                    1391 	    .org 0x0170
                           1392 ;	# 70 "../../include/ezintavecs.h"
   0170 32                 1393 	 reti
                           1394 ;	# 94 "../../include/ezintavecs.h"
   0174                    1395 	    .org 0x0174
                           1396 ;	# 71 "../../include/ezintavecs.h"
   0174 32                 1397 	 reti
                           1398 ;	# 94 "../../include/ezintavecs.h"
   0178                    1399 	    .org 0x0178
                           1400 ;	# 72 "../../include/ezintavecs.h"
   0178 32                 1401 	 reti
                           1402 ;	# 94 "../../include/ezintavecs.h"
   017C                    1403 	    .org 0x017C
                           1404 ;	# 73 "../../include/ezintavecs.h"
   017C 32                 1405 	 reti
                           1406 ;	# 94 "../../include/ezintavecs.h"
   0180                    1407 	    .org 0x0180
                           1408 ;	# 74 "../../include/ezintavecs.h"
   0180 32                 1409 	 reti
                           1410 ;	# 94 "../../include/ezintavecs.h"
   0184                    1411 	    .org 0x0184
                           1412 ;	# 75 "../../include/ezintavecs.h"
   0184 32                 1413 	 reti
                           1414 ;	# 94 "../../include/ezintavecs.h"
   0188                    1415 	    .org 0x0188
                           1416 ;	# 76 "../../include/ezintavecs.h"
   0188 32                 1417 	 reti
                           1418 ;	# 94 "../../include/ezintavecs.h"
   018C                    1419 	    .org 0x018C
                           1420 ;	# 77 "../../include/ezintavecs.h"
   018C 32                 1421 	 reti
                           1422 ;	# 94 "../../include/ezintavecs.h"
   0190                    1423 	    .org 0x0190
                           1424 ;	# 78 "../../include/ezintavecs.h"
   0190 32                 1425 	 reti
                           1426 ;	# 94 "../../include/ezintavecs.h"
   0194                    1427 	    .org 0x0194
                           1428 ;	# 79 "../../include/ezintavecs.h"
   0194 32                 1429 	 reti
                           1430 ;	# 94 "../../include/ezintavecs.h"
   0198                    1431 	    .org 0x0198
                           1432 ;	# 80 "../../include/ezintavecs.h"
   0198 32                 1433 	 reti
                           1434 ;	# 94 "../../include/ezintavecs.h"
   019C                    1435 	    .org 0x019C
                           1436 ;	# 81 "../../include/ezintavecs.h"
   019C 32                 1437 	 reti
                           1438 ;	# 94 "../../include/ezintavecs.h"
   01A0                    1439 	    .org 0x01A0
                           1440 ;	# 82 "../../include/ezintavecs.h"
   01A0 32                 1441 	 reti
                           1442 ;	# 94 "../../include/ezintavecs.h"
   01A8                    1443 	    .org 0x01A8
                           1444 ;	# 83 "../../include/ezintavecs.h"
   01A8 32                 1445 	 reti
                           1446 ;	# 94 "../../include/ezintavecs.h"
   01AC                    1447 	    .org 0x01AC
                           1448 ;	# 84 "../../include/ezintavecs.h"
   01AC 32                 1449 	 reti
                           1450 ;	# 94 "../../include/ezintavecs.h"
   01B0                    1451 	    .org 0x01B0
                           1452 ;	# 85 "../../include/ezintavecs.h"
   01B0 32                 1453 	 reti
                           1454 ;	# 94 "../../include/ezintavecs.h"
   01B4                    1455 	    .org 0x01B4
                           1456 ;	# 101 "../../include/ezintavecs.h"
   01B4 32                 1457 	 reti
   01B8                    1458 	    .org 0x01b8
   01B8                    1459 	INTVEC_DUMMY:
   01B8 32                 1460 	        reti
                           1461 	    .area CSEG (CODE)
                           1462 	    
   0264 22                 1463 	ret
                           1464 ;------------------------------------------------------------
                           1465 ;Allocation info for local variables in function 'wait'
                           1466 ;------------------------------------------------------------
                           1467 ;ms                        Allocated to registers r2 r3 
                           1468 ;i                         Allocated to registers r6 r7 
                           1469 ;j                         Allocated to registers r4 r5 
                           1470 ;------------------------------------------------------------
                           1471 ;	../../include/ztex-utils.h:78: void wait(WORD short ms) {	  // wait in ms 
                           1472 ;	-----------------------------------------
                           1473 ;	 function wait
                           1474 ;	-----------------------------------------
   0265                    1475 _wait:
   0265 AA 82              1476 	mov	r2,dpl
   0267 AB 83              1477 	mov	r3,dph
                           1478 ;	../../include/ztex-utils.h:80: for (j=0; j<ms; j++) 
   0269 7C 00              1479 	mov	r4,#0x00
   026B 7D 00              1480 	mov	r5,#0x00
   026D                    1481 00104$:
   026D C3                 1482 	clr	c
   026E EC                 1483 	mov	a,r4
   026F 9A                 1484 	subb	a,r2
   0270 ED                 1485 	mov	a,r5
   0271 9B                 1486 	subb	a,r3
   0272 50 14              1487 	jnc	00108$
                           1488 ;	../../include/ztex-utils.h:81: for (i=0; i<1200; i++);
   0274 7E B0              1489 	mov	r6,#0xB0
   0276 7F 04              1490 	mov	r7,#0x04
   0278                    1491 00103$:
   0278 1E                 1492 	dec	r6
   0279 BE FF 01           1493 	cjne	r6,#0xff,00117$
   027C 1F                 1494 	dec	r7
   027D                    1495 00117$:
   027D EE                 1496 	mov	a,r6
   027E 4F                 1497 	orl	a,r7
   027F 70 F7              1498 	jnz	00103$
                           1499 ;	../../include/ztex-utils.h:80: for (j=0; j<ms; j++) 
   0281 0C                 1500 	inc	r4
   0282 BC 00 E8           1501 	cjne	r4,#0x00,00104$
   0285 0D                 1502 	inc	r5
   0286 80 E5              1503 	sjmp	00104$
   0288                    1504 00108$:
   0288 22                 1505 	ret
                           1506 ;------------------------------------------------------------
                           1507 ;Allocation info for local variables in function 'uwait'
                           1508 ;------------------------------------------------------------
                           1509 ;us                        Allocated to registers r2 r3 
                           1510 ;i                         Allocated to registers r6 r7 
                           1511 ;j                         Allocated to registers r4 r5 
                           1512 ;------------------------------------------------------------
                           1513 ;	../../include/ztex-utils.h:88: void uwait(WORD short us) {	  // wait in 10µs steps
                           1514 ;	-----------------------------------------
                           1515 ;	 function uwait
                           1516 ;	-----------------------------------------
   0289                    1517 _uwait:
   0289 AA 82              1518 	mov	r2,dpl
   028B AB 83              1519 	mov	r3,dph
                           1520 ;	../../include/ztex-utils.h:90: for (j=0; j<us; j++) 
   028D 7C 00              1521 	mov	r4,#0x00
   028F 7D 00              1522 	mov	r5,#0x00
   0291                    1523 00104$:
   0291 C3                 1524 	clr	c
   0292 EC                 1525 	mov	a,r4
   0293 9A                 1526 	subb	a,r2
   0294 ED                 1527 	mov	a,r5
   0295 9B                 1528 	subb	a,r3
   0296 50 14              1529 	jnc	00108$
                           1530 ;	../../include/ztex-utils.h:91: for (i=0; i<10; i++);
   0298 7E 0A              1531 	mov	r6,#0x0A
   029A 7F 00              1532 	mov	r7,#0x00
   029C                    1533 00103$:
   029C 1E                 1534 	dec	r6
   029D BE FF 01           1535 	cjne	r6,#0xff,00117$
   02A0 1F                 1536 	dec	r7
   02A1                    1537 00117$:
   02A1 EE                 1538 	mov	a,r6
   02A2 4F                 1539 	orl	a,r7
   02A3 70 F7              1540 	jnz	00103$
                           1541 ;	../../include/ztex-utils.h:90: for (j=0; j<us; j++) 
   02A5 0C                 1542 	inc	r4
   02A6 BC 00 E8           1543 	cjne	r4,#0x00,00104$
   02A9 0D                 1544 	inc	r5
   02AA 80 E5              1545 	sjmp	00104$
   02AC                    1546 00108$:
   02AC 22                 1547 	ret
                           1548 ;------------------------------------------------------------
                           1549 ;Allocation info for local variables in function 'MEM_COPY1_int'
                           1550 ;------------------------------------------------------------
                           1551 ;------------------------------------------------------------
                           1552 ;	../../include/ztex-utils.h:99: void MEM_COPY1_int() // __naked 
                           1553 ;	-----------------------------------------
                           1554 ;	 function MEM_COPY1_int
                           1555 ;	-----------------------------------------
   02AD                    1556 _MEM_COPY1_int:
                           1557 ;	../../include/ztex-utils.h:110: __endasm;
                           1558 	
   02AD                    1559 	020001$:
   02AD 75 AF 07           1560 	     mov _AUTOPTRSETUP,#0x07
   02B0 90 E6 7B           1561 	     mov dptr,#_XAUTODAT1
   02B3 E0                 1562 	     movx a,@dptr
   02B4 90 E6 7C           1563 	     mov dptr,#_XAUTODAT2
   02B7 F0                 1564 	     movx @dptr,a
   02B8 DA F3              1565 	     djnz r2, 020001$
   02BA 22                 1566 	     ret
                           1567 	 
   02BB 22                 1568 	ret
                           1569 ;------------------------------------------------------------
                           1570 ;Allocation info for local variables in function 'i2c_waitWrite'
                           1571 ;------------------------------------------------------------
                           1572 ;i2csbuf                   Allocated to registers r2 
                           1573 ;toc                       Allocated to registers r2 
                           1574 ;------------------------------------------------------------
                           1575 ;	../../include/ztex-eeprom.h:41: BYTE i2c_waitWrite()
                           1576 ;	-----------------------------------------
                           1577 ;	 function i2c_waitWrite
                           1578 ;	-----------------------------------------
   02BC                    1579 _i2c_waitWrite:
                           1580 ;	../../include/ztex-eeprom.h:44: for ( toc=0; toc<255 && !(I2CS & bmBIT0); toc++ );
   02BC 7A 00              1581 	mov	r2,#0x00
   02BE                    1582 00105$:
   02BE BA FF 00           1583 	cjne	r2,#0xFF,00116$
   02C1                    1584 00116$:
   02C1 50 0B              1585 	jnc	00108$
   02C3 90 E6 78           1586 	mov	dptr,#_I2CS
   02C6 E0                 1587 	movx	a,@dptr
   02C7 FB                 1588 	mov	r3,a
   02C8 20 E0 03           1589 	jb	acc.0,00108$
   02CB 0A                 1590 	inc	r2
   02CC 80 F0              1591 	sjmp	00105$
   02CE                    1592 00108$:
                           1593 ;	../../include/ztex-eeprom.h:45: i2csbuf = I2CS;
   02CE 90 E6 78           1594 	mov	dptr,#_I2CS
   02D1 E0                 1595 	movx	a,@dptr
                           1596 ;	../../include/ztex-eeprom.h:46: if ( (i2csbuf & bmBIT2) || (!(i2csbuf & bmBIT1)) ) {
   02D2 FA                 1597 	mov	r2,a
   02D3 20 E2 04           1598 	jb	acc.2,00101$
   02D6 EA                 1599 	mov	a,r2
   02D7 20 E1 0B           1600 	jb	acc.1,00102$
   02DA                    1601 00101$:
                           1602 ;	../../include/ztex-eeprom.h:47: I2CS |= bmBIT6;
   02DA 90 E6 78           1603 	mov	dptr,#_I2CS
   02DD E0                 1604 	movx	a,@dptr
   02DE 44 40              1605 	orl	a,#0x40
   02E0 F0                 1606 	movx	@dptr,a
                           1607 ;	../../include/ztex-eeprom.h:48: return 1;
   02E1 75 82 01           1608 	mov	dpl,#0x01
                           1609 ;	../../include/ztex-eeprom.h:50: return 0;
   02E4 22                 1610 	ret
   02E5                    1611 00102$:
   02E5 75 82 00           1612 	mov	dpl,#0x00
   02E8 22                 1613 	ret
                           1614 ;------------------------------------------------------------
                           1615 ;Allocation info for local variables in function 'i2c_waitRead'
                           1616 ;------------------------------------------------------------
                           1617 ;i2csbuf                   Allocated to registers r2 
                           1618 ;toc                       Allocated to registers r2 
                           1619 ;------------------------------------------------------------
                           1620 ;	../../include/ztex-eeprom.h:57: BYTE i2c_waitRead(void)
                           1621 ;	-----------------------------------------
                           1622 ;	 function i2c_waitRead
                           1623 ;	-----------------------------------------
   02E9                    1624 _i2c_waitRead:
                           1625 ;	../../include/ztex-eeprom.h:60: for ( toc=0; toc<255 && !(I2CS & bmBIT0); toc++ );
   02E9 7A 00              1626 	mov	r2,#0x00
   02EB                    1627 00104$:
   02EB BA FF 00           1628 	cjne	r2,#0xFF,00115$
   02EE                    1629 00115$:
   02EE 50 0B              1630 	jnc	00107$
   02F0 90 E6 78           1631 	mov	dptr,#_I2CS
   02F3 E0                 1632 	movx	a,@dptr
   02F4 FB                 1633 	mov	r3,a
   02F5 20 E0 03           1634 	jb	acc.0,00107$
   02F8 0A                 1635 	inc	r2
   02F9 80 F0              1636 	sjmp	00104$
   02FB                    1637 00107$:
                           1638 ;	../../include/ztex-eeprom.h:61: i2csbuf = I2CS;
   02FB 90 E6 78           1639 	mov	dptr,#_I2CS
   02FE E0                 1640 	movx	a,@dptr
                           1641 ;	../../include/ztex-eeprom.h:62: if (i2csbuf & bmBIT2) {
   02FF FA                 1642 	mov	r2,a
   0300 30 E2 0B           1643 	jnb	acc.2,00102$
                           1644 ;	../../include/ztex-eeprom.h:63: I2CS |= bmBIT6;
   0303 90 E6 78           1645 	mov	dptr,#_I2CS
   0306 E0                 1646 	movx	a,@dptr
   0307 44 40              1647 	orl	a,#0x40
   0309 F0                 1648 	movx	@dptr,a
                           1649 ;	../../include/ztex-eeprom.h:64: return 1;
   030A 75 82 01           1650 	mov	dpl,#0x01
                           1651 ;	../../include/ztex-eeprom.h:66: return 0;
   030D 22                 1652 	ret
   030E                    1653 00102$:
   030E 75 82 00           1654 	mov	dpl,#0x00
   0311 22                 1655 	ret
                           1656 ;------------------------------------------------------------
                           1657 ;Allocation info for local variables in function 'i2c_waitStart'
                           1658 ;------------------------------------------------------------
                           1659 ;toc                       Allocated to registers r2 
                           1660 ;------------------------------------------------------------
                           1661 ;	../../include/ztex-eeprom.h:73: BYTE i2c_waitStart()
                           1662 ;	-----------------------------------------
                           1663 ;	 function i2c_waitStart
                           1664 ;	-----------------------------------------
   0312                    1665 _i2c_waitStart:
                           1666 ;	../../include/ztex-eeprom.h:76: for ( toc=0; toc<255; toc++ ) {
   0312 7A 00              1667 	mov	r2,#0x00
   0314                    1668 00103$:
   0314 BA FF 00           1669 	cjne	r2,#0xFF,00112$
   0317                    1670 00112$:
   0317 50 0F              1671 	jnc	00106$
                           1672 ;	../../include/ztex-eeprom.h:77: if ( ! (I2CS & bmBIT2) )
   0319 90 E6 78           1673 	mov	dptr,#_I2CS
   031C E0                 1674 	movx	a,@dptr
   031D FB                 1675 	mov	r3,a
   031E 20 E2 04           1676 	jb	acc.2,00105$
                           1677 ;	../../include/ztex-eeprom.h:78: return 0;
   0321 75 82 00           1678 	mov	dpl,#0x00
   0324 22                 1679 	ret
   0325                    1680 00105$:
                           1681 ;	../../include/ztex-eeprom.h:76: for ( toc=0; toc<255; toc++ ) {
   0325 0A                 1682 	inc	r2
   0326 80 EC              1683 	sjmp	00103$
   0328                    1684 00106$:
                           1685 ;	../../include/ztex-eeprom.h:80: return 1;
   0328 75 82 01           1686 	mov	dpl,#0x01
   032B 22                 1687 	ret
                           1688 ;------------------------------------------------------------
                           1689 ;Allocation info for local variables in function 'i2c_waitStop'
                           1690 ;------------------------------------------------------------
                           1691 ;toc                       Allocated to registers r2 
                           1692 ;------------------------------------------------------------
                           1693 ;	../../include/ztex-eeprom.h:87: BYTE i2c_waitStop()
                           1694 ;	-----------------------------------------
                           1695 ;	 function i2c_waitStop
                           1696 ;	-----------------------------------------
   032C                    1697 _i2c_waitStop:
                           1698 ;	../../include/ztex-eeprom.h:90: for ( toc=0; toc<255; toc++ ) {
   032C 7A 00              1699 	mov	r2,#0x00
   032E                    1700 00103$:
   032E BA FF 00           1701 	cjne	r2,#0xFF,00112$
   0331                    1702 00112$:
   0331 50 0F              1703 	jnc	00106$
                           1704 ;	../../include/ztex-eeprom.h:91: if ( ! (I2CS & bmBIT6) )
   0333 90 E6 78           1705 	mov	dptr,#_I2CS
   0336 E0                 1706 	movx	a,@dptr
   0337 FB                 1707 	mov	r3,a
   0338 20 E6 04           1708 	jb	acc.6,00105$
                           1709 ;	../../include/ztex-eeprom.h:92: return 0;
   033B 75 82 00           1710 	mov	dpl,#0x00
   033E 22                 1711 	ret
   033F                    1712 00105$:
                           1713 ;	../../include/ztex-eeprom.h:90: for ( toc=0; toc<255; toc++ ) {
   033F 0A                 1714 	inc	r2
   0340 80 EC              1715 	sjmp	00103$
   0342                    1716 00106$:
                           1717 ;	../../include/ztex-eeprom.h:94: return 1;
   0342 75 82 01           1718 	mov	dpl,#0x01
   0345 22                 1719 	ret
                           1720 ;------------------------------------------------------------
                           1721 ;Allocation info for local variables in function 'eeprom_select'
                           1722 ;------------------------------------------------------------
                           1723 ;to                        Allocated with name '_eeprom_select_PARM_2'
                           1724 ;stop                      Allocated with name '_eeprom_select_PARM_3'
                           1725 ;addr                      Allocated to registers r2 
                           1726 ;toc                       Allocated to registers 
                           1727 ;------------------------------------------------------------
                           1728 ;	../../include/ztex-eeprom.h:103: BYTE eeprom_select (BYTE addr, BYTE to, BYTE stop ) {
                           1729 ;	-----------------------------------------
                           1730 ;	 function eeprom_select
                           1731 ;	-----------------------------------------
   0346                    1732 _eeprom_select:
   0346 AA 82              1733 	mov	r2,dpl
                           1734 ;	../../include/ztex-eeprom.h:105: eeprom_select_start:
   0348 C3                 1735 	clr	c
   0349 E4                 1736 	clr	a
   034A 95 08              1737 	subb	a,_eeprom_select_PARM_2
   034C E4                 1738 	clr	a
   034D 33                 1739 	rlc	a
   034E FB                 1740 	mov	r3,a
   034F                    1741 00101$:
                           1742 ;	../../include/ztex-eeprom.h:106: I2CS |= bmBIT7;		// start bit
   034F 90 E6 78           1743 	mov	dptr,#_I2CS
   0352 E0                 1744 	movx	a,@dptr
   0353 44 80              1745 	orl	a,#0x80
   0355 F0                 1746 	movx	@dptr,a
                           1747 ;	../../include/ztex-eeprom.h:107: i2c_waitStart();
   0356 C0 02              1748 	push	ar2
   0358 C0 03              1749 	push	ar3
   035A 12 03 12           1750 	lcall	_i2c_waitStart
   035D D0 03              1751 	pop	ar3
   035F D0 02              1752 	pop	ar2
                           1753 ;	../../include/ztex-eeprom.h:108: I2DAT = addr;		// select device for writing
   0361 90 E6 79           1754 	mov	dptr,#_I2DAT
   0364 EA                 1755 	mov	a,r2
   0365 F0                 1756 	movx	@dptr,a
                           1757 ;	../../include/ztex-eeprom.h:109: if ( ! i2c_waitWrite() ) {
   0366 C0 02              1758 	push	ar2
   0368 C0 03              1759 	push	ar3
   036A 12 02 BC           1760 	lcall	_i2c_waitWrite
   036D E5 82              1761 	mov	a,dpl
   036F D0 03              1762 	pop	ar3
   0371 D0 02              1763 	pop	ar2
   0373 70 12              1764 	jnz	00107$
                           1765 ;	../../include/ztex-eeprom.h:110: if ( stop ) {
   0375 E5 09              1766 	mov	a,_eeprom_select_PARM_3
   0377 60 0A              1767 	jz	00103$
                           1768 ;	../../include/ztex-eeprom.h:111: I2CS |= bmBIT6;
   0379 90 E6 78           1769 	mov	dptr,#_I2CS
   037C E0                 1770 	movx	a,@dptr
   037D 44 40              1771 	orl	a,#0x40
   037F F0                 1772 	movx	@dptr,a
                           1773 ;	../../include/ztex-eeprom.h:112: i2c_waitStop();
   0380 12 03 2C           1774 	lcall	_i2c_waitStop
   0383                    1775 00103$:
                           1776 ;	../../include/ztex-eeprom.h:114: return 0;
   0383 75 82 00           1777 	mov	dpl,#0x00
   0386 22                 1778 	ret
   0387                    1779 00107$:
                           1780 ;	../../include/ztex-eeprom.h:116: else if (toc<to) {
   0387 EB                 1781 	mov	a,r3
   0388 60 10              1782 	jz	00108$
                           1783 ;	../../include/ztex-eeprom.h:117: uwait(10);
   038A 90 00 0A           1784 	mov	dptr,#0x000A
   038D C0 02              1785 	push	ar2
   038F C0 03              1786 	push	ar3
   0391 12 02 89           1787 	lcall	_uwait
   0394 D0 03              1788 	pop	ar3
   0396 D0 02              1789 	pop	ar2
                           1790 ;	../../include/ztex-eeprom.h:118: goto eeprom_select_start;
   0398 80 B5              1791 	sjmp	00101$
   039A                    1792 00108$:
                           1793 ;	../../include/ztex-eeprom.h:120: if ( stop ) {
   039A E5 09              1794 	mov	a,_eeprom_select_PARM_3
   039C 60 08              1795 	jz	00110$
                           1796 ;	../../include/ztex-eeprom.h:121: I2CS |= bmBIT6;
   039E 90 E6 78           1797 	mov	dptr,#_I2CS
   03A1 E0                 1798 	movx	a,@dptr
   03A2 FA                 1799 	mov	r2,a
   03A3 44 40              1800 	orl	a,#0x40
   03A5 F0                 1801 	movx	@dptr,a
   03A6                    1802 00110$:
                           1803 ;	../../include/ztex-eeprom.h:123: return 1;
   03A6 75 82 01           1804 	mov	dpl,#0x01
   03A9 22                 1805 	ret
                           1806 ;------------------------------------------------------------
                           1807 ;Allocation info for local variables in function 'eeprom_read'
                           1808 ;------------------------------------------------------------
                           1809 ;addr                      Allocated with name '_eeprom_read_PARM_2'
                           1810 ;length                    Allocated with name '_eeprom_read_PARM_3'
                           1811 ;buf                       Allocated to registers r2 r3 
                           1812 ;bytes                     Allocated to registers r4 
                           1813 ;i                         Allocated to registers 
                           1814 ;------------------------------------------------------------
                           1815 ;	../../include/ztex-eeprom.h:131: BYTE eeprom_read ( __xdata BYTE *buf, WORD addr, BYTE length ) { 
                           1816 ;	-----------------------------------------
                           1817 ;	 function eeprom_read
                           1818 ;	-----------------------------------------
   03AA                    1819 _eeprom_read:
   03AA AA 82              1820 	mov	r2,dpl
   03AC AB 83              1821 	mov	r3,dph
                           1822 ;	../../include/ztex-eeprom.h:132: BYTE bytes = 0,i;
   03AE 7C 00              1823 	mov	r4,#0x00
                           1824 ;	../../include/ztex-eeprom.h:134: if ( length == 0 ) 
   03B0 E5 0C              1825 	mov	a,_eeprom_read_PARM_3
                           1826 ;	../../include/ztex-eeprom.h:135: return 0;
   03B2 70 03              1827 	jnz	00102$
   03B4 F5 82              1828 	mov	dpl,a
   03B6 22                 1829 	ret
   03B7                    1830 00102$:
                           1831 ;	../../include/ztex-eeprom.h:137: if ( eeprom_select(EEPROM_ADDR, 100,0) ) 
   03B7 75 08 64           1832 	mov	_eeprom_select_PARM_2,#0x64
   03BA 75 09 00           1833 	mov	_eeprom_select_PARM_3,#0x00
   03BD 75 82 A2           1834 	mov	dpl,#0xA2
   03C0 C0 02              1835 	push	ar2
   03C2 C0 03              1836 	push	ar3
   03C4 C0 04              1837 	push	ar4
   03C6 12 03 46           1838 	lcall	_eeprom_select
   03C9 E5 82              1839 	mov	a,dpl
   03CB D0 04              1840 	pop	ar4
   03CD D0 03              1841 	pop	ar3
   03CF D0 02              1842 	pop	ar2
   03D1 60 03              1843 	jz	00134$
   03D3 02 04 B0           1844 	ljmp	00117$
   03D6                    1845 00134$:
                           1846 ;	../../include/ztex-eeprom.h:140: I2DAT = HI(addr);		// write address
   03D6 90 E6 79           1847 	mov	dptr,#_I2DAT
   03D9 E5 0B              1848 	mov	a,(_eeprom_read_PARM_2 + 1)
   03DB F0                 1849 	movx	@dptr,a
                           1850 ;	../../include/ztex-eeprom.h:141: if ( i2c_waitWrite() ) goto eeprom_read_end;
   03DC C0 02              1851 	push	ar2
   03DE C0 03              1852 	push	ar3
   03E0 C0 04              1853 	push	ar4
   03E2 12 02 BC           1854 	lcall	_i2c_waitWrite
   03E5 E5 82              1855 	mov	a,dpl
   03E7 D0 04              1856 	pop	ar4
   03E9 D0 03              1857 	pop	ar3
   03EB D0 02              1858 	pop	ar2
   03ED 60 03              1859 	jz	00135$
   03EF 02 04 B0           1860 	ljmp	00117$
   03F2                    1861 00135$:
                           1862 ;	../../include/ztex-eeprom.h:142: I2DAT = LO(addr);		// write address
   03F2 90 E6 79           1863 	mov	dptr,#_I2DAT
   03F5 E5 0A              1864 	mov	a,_eeprom_read_PARM_2
   03F7 F0                 1865 	movx	@dptr,a
                           1866 ;	../../include/ztex-eeprom.h:143: if ( i2c_waitWrite() ) goto eeprom_read_end;
   03F8 C0 02              1867 	push	ar2
   03FA C0 03              1868 	push	ar3
   03FC C0 04              1869 	push	ar4
   03FE 12 02 BC           1870 	lcall	_i2c_waitWrite
   0401 E5 82              1871 	mov	a,dpl
   0403 D0 04              1872 	pop	ar4
   0405 D0 03              1873 	pop	ar3
   0407 D0 02              1874 	pop	ar2
   0409 60 03              1875 	jz	00136$
   040B 02 04 B0           1876 	ljmp	00117$
   040E                    1877 00136$:
                           1878 ;	../../include/ztex-eeprom.h:144: I2CS |= bmBIT6;
   040E 90 E6 78           1879 	mov	dptr,#_I2CS
   0411 E0                 1880 	movx	a,@dptr
   0412 44 40              1881 	orl	a,#0x40
   0414 F0                 1882 	movx	@dptr,a
                           1883 ;	../../include/ztex-eeprom.h:145: i2c_waitStop();
   0415 C0 02              1884 	push	ar2
   0417 C0 03              1885 	push	ar3
   0419 C0 04              1886 	push	ar4
   041B 12 03 2C           1887 	lcall	_i2c_waitStop
                           1888 ;	../../include/ztex-eeprom.h:147: I2CS |= bmBIT7;		// start bit
   041E 90 E6 78           1889 	mov	dptr,#_I2CS
   0421 E0                 1890 	movx	a,@dptr
   0422 44 80              1891 	orl	a,#0x80
   0424 F0                 1892 	movx	@dptr,a
                           1893 ;	../../include/ztex-eeprom.h:148: i2c_waitStart();
   0425 12 03 12           1894 	lcall	_i2c_waitStart
                           1895 ;	../../include/ztex-eeprom.h:149: I2DAT = EEPROM_ADDR | 1;	// select device for reading
   0428 90 E6 79           1896 	mov	dptr,#_I2DAT
   042B 74 A3              1897 	mov	a,#0xA3
   042D F0                 1898 	movx	@dptr,a
                           1899 ;	../../include/ztex-eeprom.h:150: if ( i2c_waitWrite() ) goto eeprom_read_end;
   042E 12 02 BC           1900 	lcall	_i2c_waitWrite
   0431 E5 82              1901 	mov	a,dpl
   0433 D0 04              1902 	pop	ar4
   0435 D0 03              1903 	pop	ar3
   0437 D0 02              1904 	pop	ar2
   0439 70 75              1905 	jnz	00117$
                           1906 ;	../../include/ztex-eeprom.h:152: *buf = I2DAT;		// dummy read
   043B 90 E6 79           1907 	mov	dptr,#_I2DAT
   043E E0                 1908 	movx	a,@dptr
   043F 8A 82              1909 	mov	dpl,r2
   0441 8B 83              1910 	mov	dph,r3
   0443 F0                 1911 	movx	@dptr,a
                           1912 ;	../../include/ztex-eeprom.h:153: if ( i2c_waitRead()) goto eeprom_read_end; 
   0444 C0 02              1913 	push	ar2
   0446 C0 03              1914 	push	ar3
   0448 C0 04              1915 	push	ar4
   044A 12 02 E9           1916 	lcall	_i2c_waitRead
   044D E5 82              1917 	mov	a,dpl
   044F D0 04              1918 	pop	ar4
   0451 D0 03              1919 	pop	ar3
   0453 D0 02              1920 	pop	ar2
   0455 70 59              1921 	jnz	00117$
   0457 FD                 1922 	mov	r5,a
   0458                    1923 00118$:
                           1924 ;	../../include/ztex-eeprom.h:154: for (; bytes<length; bytes++ ) {
   0458 C3                 1925 	clr	c
   0459 ED                 1926 	mov	a,r5
   045A 95 0C              1927 	subb	a,_eeprom_read_PARM_3
   045C 50 2A              1928 	jnc	00121$
                           1929 ;	../../include/ztex-eeprom.h:155: *buf = I2DAT;		// read data
   045E 90 E6 79           1930 	mov	dptr,#_I2DAT
   0461 E0                 1931 	movx	a,@dptr
   0462 8A 82              1932 	mov	dpl,r2
   0464 8B 83              1933 	mov	dph,r3
   0466 F0                 1934 	movx	@dptr,a
   0467 A3                 1935 	inc	dptr
   0468 AA 82              1936 	mov	r2,dpl
   046A AB 83              1937 	mov	r3,dph
                           1938 ;	../../include/ztex-eeprom.h:156: buf++;
                           1939 ;	../../include/ztex-eeprom.h:157: if ( i2c_waitRead()) goto eeprom_read_end; 
   046C C0 02              1940 	push	ar2
   046E C0 03              1941 	push	ar3
   0470 C0 04              1942 	push	ar4
   0472 C0 05              1943 	push	ar5
   0474 12 02 E9           1944 	lcall	_i2c_waitRead
   0477 E5 82              1945 	mov	a,dpl
   0479 D0 05              1946 	pop	ar5
   047B D0 04              1947 	pop	ar4
   047D D0 03              1948 	pop	ar3
   047F D0 02              1949 	pop	ar2
   0481 70 2D              1950 	jnz	00117$
                           1951 ;	../../include/ztex-eeprom.h:154: for (; bytes<length; bytes++ ) {
   0483 0D                 1952 	inc	r5
   0484 8D 04              1953 	mov	ar4,r5
   0486 80 D0              1954 	sjmp	00118$
   0488                    1955 00121$:
                           1956 ;	../../include/ztex-eeprom.h:160: I2CS |= bmBIT5;		// no ACK
   0488 90 E6 78           1957 	mov	dptr,#_I2CS
   048B E0                 1958 	movx	a,@dptr
   048C 44 20              1959 	orl	a,#0x20
   048E F0                 1960 	movx	@dptr,a
                           1961 ;	../../include/ztex-eeprom.h:161: i = I2DAT;			// dummy read
   048F 90 E6 79           1962 	mov	dptr,#_I2DAT
   0492 E0                 1963 	movx	a,@dptr
                           1964 ;	../../include/ztex-eeprom.h:162: if ( i2c_waitRead()) goto eeprom_read_end; 
   0493 C0 04              1965 	push	ar4
   0495 12 02 E9           1966 	lcall	_i2c_waitRead
   0498 E5 82              1967 	mov	a,dpl
   049A D0 04              1968 	pop	ar4
   049C 70 12              1969 	jnz	00117$
                           1970 ;	../../include/ztex-eeprom.h:164: I2CS |= bmBIT6;		// stop bit
   049E 90 E6 78           1971 	mov	dptr,#_I2CS
   04A1 E0                 1972 	movx	a,@dptr
   04A2 44 40              1973 	orl	a,#0x40
   04A4 F0                 1974 	movx	@dptr,a
                           1975 ;	../../include/ztex-eeprom.h:165: i = I2DAT;			// dummy read
   04A5 90 E6 79           1976 	mov	dptr,#_I2DAT
   04A8 E0                 1977 	movx	a,@dptr
                           1978 ;	../../include/ztex-eeprom.h:166: i2c_waitStop();
   04A9 C0 04              1979 	push	ar4
   04AB 12 03 2C           1980 	lcall	_i2c_waitStop
   04AE D0 04              1981 	pop	ar4
                           1982 ;	../../include/ztex-eeprom.h:168: eeprom_read_end:
   04B0                    1983 00117$:
                           1984 ;	../../include/ztex-eeprom.h:169: return bytes;
   04B0 8C 82              1985 	mov	dpl,r4
   04B2 22                 1986 	ret
                           1987 ;------------------------------------------------------------
                           1988 ;Allocation info for local variables in function 'eeprom_write'
                           1989 ;------------------------------------------------------------
                           1990 ;addr                      Allocated with name '_eeprom_write_PARM_2'
                           1991 ;length                    Allocated with name '_eeprom_write_PARM_3'
                           1992 ;buf                       Allocated to registers r2 r3 
                           1993 ;bytes                     Allocated to registers r4 
                           1994 ;------------------------------------------------------------
                           1995 ;	../../include/ztex-eeprom.h:178: BYTE eeprom_write ( __xdata BYTE *buf, WORD addr, BYTE length ) {
                           1996 ;	-----------------------------------------
                           1997 ;	 function eeprom_write
                           1998 ;	-----------------------------------------
   04B3                    1999 _eeprom_write:
   04B3 AA 82              2000 	mov	r2,dpl
   04B5 AB 83              2001 	mov	r3,dph
                           2002 ;	../../include/ztex-eeprom.h:179: BYTE bytes = 0;
   04B7 7C 00              2003 	mov	r4,#0x00
                           2004 ;	../../include/ztex-eeprom.h:181: if ( length == 0 ) 
   04B9 E5 0F              2005 	mov	a,_eeprom_write_PARM_3
                           2006 ;	../../include/ztex-eeprom.h:182: return 0;
   04BB 70 03              2007 	jnz	00102$
   04BD F5 82              2008 	mov	dpl,a
   04BF 22                 2009 	ret
   04C0                    2010 00102$:
                           2011 ;	../../include/ztex-eeprom.h:184: if ( eeprom_select(EEPROM_ADDR, 100,0) ) 
   04C0 75 08 64           2012 	mov	_eeprom_select_PARM_2,#0x64
   04C3 75 09 00           2013 	mov	_eeprom_select_PARM_3,#0x00
   04C6 75 82 A2           2014 	mov	dpl,#0xA2
   04C9 C0 02              2015 	push	ar2
   04CB C0 03              2016 	push	ar3
   04CD C0 04              2017 	push	ar4
   04CF 12 03 46           2018 	lcall	_eeprom_select
   04D2 E5 82              2019 	mov	a,dpl
   04D4 D0 04              2020 	pop	ar4
   04D6 D0 03              2021 	pop	ar3
   04D8 D0 02              2022 	pop	ar2
   04DA 60 03              2023 	jz	00125$
   04DC 02 05 70           2024 	ljmp	00111$
   04DF                    2025 00125$:
                           2026 ;	../../include/ztex-eeprom.h:187: I2DAT = HI(addr);          	// write address
   04DF 90 E6 79           2027 	mov	dptr,#_I2DAT
   04E2 E5 0E              2028 	mov	a,(_eeprom_write_PARM_2 + 1)
   04E4 F0                 2029 	movx	@dptr,a
                           2030 ;	../../include/ztex-eeprom.h:188: if ( i2c_waitWrite() ) goto eeprom_write_end;
   04E5 C0 02              2031 	push	ar2
   04E7 C0 03              2032 	push	ar3
   04E9 C0 04              2033 	push	ar4
   04EB 12 02 BC           2034 	lcall	_i2c_waitWrite
   04EE E5 82              2035 	mov	a,dpl
   04F0 D0 04              2036 	pop	ar4
   04F2 D0 03              2037 	pop	ar3
   04F4 D0 02              2038 	pop	ar2
   04F6 60 03              2039 	jz	00126$
   04F8 02 05 70           2040 	ljmp	00111$
   04FB                    2041 00126$:
                           2042 ;	../../include/ztex-eeprom.h:189: I2DAT = LO(addr);          	// write address
   04FB 90 E6 79           2043 	mov	dptr,#_I2DAT
   04FE E5 0D              2044 	mov	a,_eeprom_write_PARM_2
   0500 F0                 2045 	movx	@dptr,a
                           2046 ;	../../include/ztex-eeprom.h:190: if ( i2c_waitWrite() ) goto eeprom_write_end;
   0501 C0 02              2047 	push	ar2
   0503 C0 03              2048 	push	ar3
   0505 C0 04              2049 	push	ar4
   0507 12 02 BC           2050 	lcall	_i2c_waitWrite
   050A E5 82              2051 	mov	a,dpl
   050C D0 04              2052 	pop	ar4
   050E D0 03              2053 	pop	ar3
   0510 D0 02              2054 	pop	ar2
   0512 70 5C              2055 	jnz	00111$
   0514 FD                 2056 	mov	r5,a
   0515                    2057 00112$:
                           2058 ;	../../include/ztex-eeprom.h:192: for (; bytes<length; bytes++ ) {
   0515 C3                 2059 	clr	c
   0516 ED                 2060 	mov	a,r5
   0517 95 0F              2061 	subb	a,_eeprom_write_PARM_3
   0519 50 47              2062 	jnc	00115$
                           2063 ;	../../include/ztex-eeprom.h:193: I2DAT = *buf;         	// write data 
   051B 8A 82              2064 	mov	dpl,r2
   051D 8B 83              2065 	mov	dph,r3
   051F E0                 2066 	movx	a,@dptr
   0520 FE                 2067 	mov	r6,a
   0521 A3                 2068 	inc	dptr
   0522 AA 82              2069 	mov	r2,dpl
   0524 AB 83              2070 	mov	r3,dph
   0526 90 E6 79           2071 	mov	dptr,#_I2DAT
   0529 EE                 2072 	mov	a,r6
   052A F0                 2073 	movx	@dptr,a
                           2074 ;	../../include/ztex-eeprom.h:194: eeprom_write_checksum += *buf;
   052B 90 3A 04           2075 	mov	dptr,#_eeprom_write_checksum
   052E E0                 2076 	movx	a,@dptr
   052F FF                 2077 	mov	r7,a
   0530 EE                 2078 	mov	a,r6
   0531 2F                 2079 	add	a,r7
   0532 F0                 2080 	movx	@dptr,a
                           2081 ;	../../include/ztex-eeprom.h:195: buf++;
                           2082 ;	../../include/ztex-eeprom.h:196: eeprom_write_bytes+=1;
   0533 90 3A 02           2083 	mov	dptr,#_eeprom_write_bytes
   0536 E0                 2084 	movx	a,@dptr
   0537 FE                 2085 	mov	r6,a
   0538 A3                 2086 	inc	dptr
   0539 E0                 2087 	movx	a,@dptr
   053A FF                 2088 	mov	r7,a
   053B 90 3A 02           2089 	mov	dptr,#_eeprom_write_bytes
   053E 74 01              2090 	mov	a,#0x01
   0540 2E                 2091 	add	a,r6
   0541 F0                 2092 	movx	@dptr,a
   0542 E4                 2093 	clr	a
   0543 3F                 2094 	addc	a,r7
   0544 A3                 2095 	inc	dptr
   0545 F0                 2096 	movx	@dptr,a
                           2097 ;	../../include/ztex-eeprom.h:197: if ( i2c_waitWrite() ) goto eeprom_write_end;
   0546 C0 02              2098 	push	ar2
   0548 C0 03              2099 	push	ar3
   054A C0 04              2100 	push	ar4
   054C C0 05              2101 	push	ar5
   054E 12 02 BC           2102 	lcall	_i2c_waitWrite
   0551 E5 82              2103 	mov	a,dpl
   0553 D0 05              2104 	pop	ar5
   0555 D0 04              2105 	pop	ar4
   0557 D0 03              2106 	pop	ar3
   0559 D0 02              2107 	pop	ar2
   055B 70 13              2108 	jnz	00111$
                           2109 ;	../../include/ztex-eeprom.h:192: for (; bytes<length; bytes++ ) {
   055D 0D                 2110 	inc	r5
   055E 8D 04              2111 	mov	ar4,r5
   0560 80 B3              2112 	sjmp	00112$
   0562                    2113 00115$:
                           2114 ;	../../include/ztex-eeprom.h:199: I2CS |= bmBIT6;		// stop bit
   0562 90 E6 78           2115 	mov	dptr,#_I2CS
   0565 E0                 2116 	movx	a,@dptr
   0566 44 40              2117 	orl	a,#0x40
   0568 F0                 2118 	movx	@dptr,a
                           2119 ;	../../include/ztex-eeprom.h:200: i2c_waitStop();
   0569 C0 04              2120 	push	ar4
   056B 12 03 2C           2121 	lcall	_i2c_waitStop
   056E D0 04              2122 	pop	ar4
                           2123 ;	../../include/ztex-eeprom.h:202: eeprom_write_end:
   0570                    2124 00111$:
                           2125 ;	../../include/ztex-eeprom.h:203: return bytes;
   0570 8C 82              2126 	mov	dpl,r4
   0572 22                 2127 	ret
                           2128 ;------------------------------------------------------------
                           2129 ;Allocation info for local variables in function 'eeprom_read_ep0'
                           2130 ;------------------------------------------------------------
                           2131 ;i                         Allocated to registers r3 
                           2132 ;b                         Allocated to registers r2 
                           2133 ;------------------------------------------------------------
                           2134 ;	../../include/ztex-eeprom.h:209: BYTE eeprom_read_ep0 () { 
                           2135 ;	-----------------------------------------
                           2136 ;	 function eeprom_read_ep0
                           2137 ;	-----------------------------------------
   0573                    2138 _eeprom_read_ep0:
                           2139 ;	../../include/ztex-eeprom.h:211: b = ep0_payload_transfer;
   0573 90 3A 38           2140 	mov	dptr,#_ep0_payload_transfer
   0576 E0                 2141 	movx	a,@dptr
   0577 FA                 2142 	mov	r2,a
                           2143 ;	../../include/ztex-eeprom.h:212: i = eeprom_read(EP0BUF, eeprom_addr, b);
   0578 90 3A 00           2144 	mov	dptr,#_eeprom_addr
   057B E0                 2145 	movx	a,@dptr
   057C F5 0A              2146 	mov	_eeprom_read_PARM_2,a
   057E A3                 2147 	inc	dptr
   057F E0                 2148 	movx	a,@dptr
   0580 F5 0B              2149 	mov	(_eeprom_read_PARM_2 + 1),a
   0582 8A 0C              2150 	mov	_eeprom_read_PARM_3,r2
   0584 90 E7 40           2151 	mov	dptr,#_EP0BUF
   0587 C0 02              2152 	push	ar2
   0589 12 03 AA           2153 	lcall	_eeprom_read
   058C AB 82              2154 	mov	r3,dpl
   058E D0 02              2155 	pop	ar2
                           2156 ;	../../include/ztex-eeprom.h:213: eeprom_addr += b;
   0590 7C 00              2157 	mov	r4,#0x00
   0592 90 3A 00           2158 	mov	dptr,#_eeprom_addr
   0595 E0                 2159 	movx	a,@dptr
   0596 FD                 2160 	mov	r5,a
   0597 A3                 2161 	inc	dptr
   0598 E0                 2162 	movx	a,@dptr
   0599 FE                 2163 	mov	r6,a
   059A 90 3A 00           2164 	mov	dptr,#_eeprom_addr
   059D EA                 2165 	mov	a,r2
   059E 2D                 2166 	add	a,r5
   059F F0                 2167 	movx	@dptr,a
   05A0 EC                 2168 	mov	a,r4
   05A1 3E                 2169 	addc	a,r6
   05A2 A3                 2170 	inc	dptr
   05A3 F0                 2171 	movx	@dptr,a
                           2172 ;	../../include/ztex-eeprom.h:214: return i;
   05A4 8B 82              2173 	mov	dpl,r3
   05A6 22                 2174 	ret
                           2175 ;------------------------------------------------------------
                           2176 ;Allocation info for local variables in function 'eeprom_write_ep0'
                           2177 ;------------------------------------------------------------
                           2178 ;length                    Allocated to registers r2 
                           2179 ;------------------------------------------------------------
                           2180 ;	../../include/ztex-eeprom.h:230: void eeprom_write_ep0 ( BYTE length ) { 	
                           2181 ;	-----------------------------------------
                           2182 ;	 function eeprom_write_ep0
                           2183 ;	-----------------------------------------
   05A7                    2184 _eeprom_write_ep0:
   05A7 AA 82              2185 	mov	r2,dpl
                           2186 ;	../../include/ztex-eeprom.h:231: eeprom_write(EP0BUF, eeprom_addr, length);
   05A9 90 3A 00           2187 	mov	dptr,#_eeprom_addr
   05AC E0                 2188 	movx	a,@dptr
   05AD F5 0D              2189 	mov	_eeprom_write_PARM_2,a
   05AF A3                 2190 	inc	dptr
   05B0 E0                 2191 	movx	a,@dptr
   05B1 F5 0E              2192 	mov	(_eeprom_write_PARM_2 + 1),a
   05B3 8A 0F              2193 	mov	_eeprom_write_PARM_3,r2
   05B5 90 E7 40           2194 	mov	dptr,#_EP0BUF
   05B8 C0 02              2195 	push	ar2
   05BA 12 04 B3           2196 	lcall	_eeprom_write
   05BD D0 02              2197 	pop	ar2
                           2198 ;	../../include/ztex-eeprom.h:232: eeprom_addr += length;
   05BF 7B 00              2199 	mov	r3,#0x00
   05C1 90 3A 00           2200 	mov	dptr,#_eeprom_addr
   05C4 E0                 2201 	movx	a,@dptr
   05C5 FC                 2202 	mov	r4,a
   05C6 A3                 2203 	inc	dptr
   05C7 E0                 2204 	movx	a,@dptr
   05C8 FD                 2205 	mov	r5,a
   05C9 90 3A 00           2206 	mov	dptr,#_eeprom_addr
   05CC EA                 2207 	mov	a,r2
   05CD 2C                 2208 	add	a,r4
   05CE F0                 2209 	movx	@dptr,a
   05CF EB                 2210 	mov	a,r3
   05D0 3D                 2211 	addc	a,r5
   05D1 A3                 2212 	inc	dptr
   05D2 F0                 2213 	movx	@dptr,a
   05D3 22                 2214 	ret
                           2215 ;------------------------------------------------------------
                           2216 ;Allocation info for local variables in function 'mac_eeprom_read'
                           2217 ;------------------------------------------------------------
                           2218 ;addr                      Allocated with name '_mac_eeprom_read_PARM_2'
                           2219 ;length                    Allocated with name '_mac_eeprom_read_PARM_3'
                           2220 ;buf                       Allocated to registers r2 r3 
                           2221 ;bytes                     Allocated to registers r4 
                           2222 ;i                         Allocated to registers 
                           2223 ;------------------------------------------------------------
                           2224 ;	../../include/ztex-eeprom.h:272: BYTE mac_eeprom_read ( __xdata BYTE *buf, BYTE addr, BYTE length ) { 
                           2225 ;	-----------------------------------------
                           2226 ;	 function mac_eeprom_read
                           2227 ;	-----------------------------------------
   05D4                    2228 _mac_eeprom_read:
   05D4 AA 82              2229 	mov	r2,dpl
   05D6 AB 83              2230 	mov	r3,dph
                           2231 ;	../../include/ztex-eeprom.h:273: BYTE bytes = 0,i;
   05D8 7C 00              2232 	mov	r4,#0x00
                           2233 ;	../../include/ztex-eeprom.h:275: if ( length == 0 ) 
   05DA E5 11              2234 	mov	a,_mac_eeprom_read_PARM_3
                           2235 ;	../../include/ztex-eeprom.h:276: return 0;
   05DC 70 03              2236 	jnz	00102$
   05DE F5 82              2237 	mov	dpl,a
   05E0 22                 2238 	ret
   05E1                    2239 00102$:
                           2240 ;	../../include/ztex-eeprom.h:278: if ( eeprom_select(EEPROM_MAC_ADDR, 100,0) ) 
   05E1 75 08 64           2241 	mov	_eeprom_select_PARM_2,#0x64
   05E4 75 09 00           2242 	mov	_eeprom_select_PARM_3,#0x00
   05E7 75 82 A6           2243 	mov	dpl,#0xA6
   05EA C0 02              2244 	push	ar2
   05EC C0 03              2245 	push	ar3
   05EE C0 04              2246 	push	ar4
   05F0 12 03 46           2247 	lcall	_eeprom_select
   05F3 E5 82              2248 	mov	a,dpl
   05F5 D0 04              2249 	pop	ar4
   05F7 D0 03              2250 	pop	ar3
   05F9 D0 02              2251 	pop	ar2
   05FB 60 03              2252 	jz	00131$
   05FD 02 06 BE           2253 	ljmp	00115$
   0600                    2254 00131$:
                           2255 ;	../../include/ztex-eeprom.h:281: I2DAT = addr;		// write address
   0600 90 E6 79           2256 	mov	dptr,#_I2DAT
   0603 E5 10              2257 	mov	a,_mac_eeprom_read_PARM_2
   0605 F0                 2258 	movx	@dptr,a
                           2259 ;	../../include/ztex-eeprom.h:282: if ( i2c_waitWrite() ) goto mac_eeprom_read_end;
   0606 C0 02              2260 	push	ar2
   0608 C0 03              2261 	push	ar3
   060A C0 04              2262 	push	ar4
   060C 12 02 BC           2263 	lcall	_i2c_waitWrite
   060F E5 82              2264 	mov	a,dpl
   0611 D0 04              2265 	pop	ar4
   0613 D0 03              2266 	pop	ar3
   0615 D0 02              2267 	pop	ar2
   0617 60 03              2268 	jz	00132$
   0619 02 06 BE           2269 	ljmp	00115$
   061C                    2270 00132$:
                           2271 ;	../../include/ztex-eeprom.h:283: I2CS |= bmBIT6;
   061C 90 E6 78           2272 	mov	dptr,#_I2CS
   061F E0                 2273 	movx	a,@dptr
   0620 44 40              2274 	orl	a,#0x40
   0622 F0                 2275 	movx	@dptr,a
                           2276 ;	../../include/ztex-eeprom.h:284: i2c_waitStop();
   0623 C0 02              2277 	push	ar2
   0625 C0 03              2278 	push	ar3
   0627 C0 04              2279 	push	ar4
   0629 12 03 2C           2280 	lcall	_i2c_waitStop
                           2281 ;	../../include/ztex-eeprom.h:286: I2CS |= bmBIT7;		// start bit
   062C 90 E6 78           2282 	mov	dptr,#_I2CS
   062F E0                 2283 	movx	a,@dptr
   0630 44 80              2284 	orl	a,#0x80
   0632 F0                 2285 	movx	@dptr,a
                           2286 ;	../../include/ztex-eeprom.h:287: i2c_waitStart();
   0633 12 03 12           2287 	lcall	_i2c_waitStart
                           2288 ;	../../include/ztex-eeprom.h:288: I2DAT = EEPROM_MAC_ADDR | 1;  // select device for reading
   0636 90 E6 79           2289 	mov	dptr,#_I2DAT
   0639 74 A7              2290 	mov	a,#0xA7
   063B F0                 2291 	movx	@dptr,a
                           2292 ;	../../include/ztex-eeprom.h:289: if ( i2c_waitWrite() ) goto mac_eeprom_read_end;
   063C 12 02 BC           2293 	lcall	_i2c_waitWrite
   063F E5 82              2294 	mov	a,dpl
   0641 D0 04              2295 	pop	ar4
   0643 D0 03              2296 	pop	ar3
   0645 D0 02              2297 	pop	ar2
   0647 70 75              2298 	jnz	00115$
                           2299 ;	../../include/ztex-eeprom.h:291: *buf = I2DAT;		// dummy read
   0649 90 E6 79           2300 	mov	dptr,#_I2DAT
   064C E0                 2301 	movx	a,@dptr
   064D 8A 82              2302 	mov	dpl,r2
   064F 8B 83              2303 	mov	dph,r3
   0651 F0                 2304 	movx	@dptr,a
                           2305 ;	../../include/ztex-eeprom.h:292: if ( i2c_waitRead()) goto mac_eeprom_read_end; 
   0652 C0 02              2306 	push	ar2
   0654 C0 03              2307 	push	ar3
   0656 C0 04              2308 	push	ar4
   0658 12 02 E9           2309 	lcall	_i2c_waitRead
   065B E5 82              2310 	mov	a,dpl
   065D D0 04              2311 	pop	ar4
   065F D0 03              2312 	pop	ar3
   0661 D0 02              2313 	pop	ar2
   0663 70 59              2314 	jnz	00115$
   0665 FD                 2315 	mov	r5,a
   0666                    2316 00116$:
                           2317 ;	../../include/ztex-eeprom.h:293: for (; bytes<length; bytes++ ) {
   0666 C3                 2318 	clr	c
   0667 ED                 2319 	mov	a,r5
   0668 95 11              2320 	subb	a,_mac_eeprom_read_PARM_3
   066A 50 2A              2321 	jnc	00119$
                           2322 ;	../../include/ztex-eeprom.h:294: *buf = I2DAT;		// read data
   066C 90 E6 79           2323 	mov	dptr,#_I2DAT
   066F E0                 2324 	movx	a,@dptr
   0670 8A 82              2325 	mov	dpl,r2
   0672 8B 83              2326 	mov	dph,r3
   0674 F0                 2327 	movx	@dptr,a
   0675 A3                 2328 	inc	dptr
   0676 AA 82              2329 	mov	r2,dpl
   0678 AB 83              2330 	mov	r3,dph
                           2331 ;	../../include/ztex-eeprom.h:295: buf++;
                           2332 ;	../../include/ztex-eeprom.h:296: if ( i2c_waitRead()) goto mac_eeprom_read_end; 
   067A C0 02              2333 	push	ar2
   067C C0 03              2334 	push	ar3
   067E C0 04              2335 	push	ar4
   0680 C0 05              2336 	push	ar5
   0682 12 02 E9           2337 	lcall	_i2c_waitRead
   0685 E5 82              2338 	mov	a,dpl
   0687 D0 05              2339 	pop	ar5
   0689 D0 04              2340 	pop	ar4
   068B D0 03              2341 	pop	ar3
   068D D0 02              2342 	pop	ar2
   068F 70 2D              2343 	jnz	00115$
                           2344 ;	../../include/ztex-eeprom.h:293: for (; bytes<length; bytes++ ) {
   0691 0D                 2345 	inc	r5
   0692 8D 04              2346 	mov	ar4,r5
   0694 80 D0              2347 	sjmp	00116$
   0696                    2348 00119$:
                           2349 ;	../../include/ztex-eeprom.h:299: I2CS |= bmBIT5;		// no ACK
   0696 90 E6 78           2350 	mov	dptr,#_I2CS
   0699 E0                 2351 	movx	a,@dptr
   069A 44 20              2352 	orl	a,#0x20
   069C F0                 2353 	movx	@dptr,a
                           2354 ;	../../include/ztex-eeprom.h:300: i = I2DAT;			// dummy read
   069D 90 E6 79           2355 	mov	dptr,#_I2DAT
   06A0 E0                 2356 	movx	a,@dptr
                           2357 ;	../../include/ztex-eeprom.h:301: if ( i2c_waitRead()) goto mac_eeprom_read_end; 
   06A1 C0 04              2358 	push	ar4
   06A3 12 02 E9           2359 	lcall	_i2c_waitRead
   06A6 E5 82              2360 	mov	a,dpl
   06A8 D0 04              2361 	pop	ar4
   06AA 70 12              2362 	jnz	00115$
                           2363 ;	../../include/ztex-eeprom.h:303: I2CS |= bmBIT6;		// stop bit
   06AC 90 E6 78           2364 	mov	dptr,#_I2CS
   06AF E0                 2365 	movx	a,@dptr
   06B0 44 40              2366 	orl	a,#0x40
   06B2 F0                 2367 	movx	@dptr,a
                           2368 ;	../../include/ztex-eeprom.h:304: i = I2DAT;			// dummy read
   06B3 90 E6 79           2369 	mov	dptr,#_I2DAT
   06B6 E0                 2370 	movx	a,@dptr
                           2371 ;	../../include/ztex-eeprom.h:305: i2c_waitStop();
   06B7 C0 04              2372 	push	ar4
   06B9 12 03 2C           2373 	lcall	_i2c_waitStop
   06BC D0 04              2374 	pop	ar4
                           2375 ;	../../include/ztex-eeprom.h:307: mac_eeprom_read_end:
   06BE                    2376 00115$:
                           2377 ;	../../include/ztex-eeprom.h:308: return bytes;
   06BE 8C 82              2378 	mov	dpl,r4
   06C0 22                 2379 	ret
                           2380 ;------------------------------------------------------------
                           2381 ;Allocation info for local variables in function 'mac_eeprom_write'
                           2382 ;------------------------------------------------------------
                           2383 ;addr                      Allocated with name '_mac_eeprom_write_PARM_2'
                           2384 ;length                    Allocated with name '_mac_eeprom_write_PARM_3'
                           2385 ;buf                       Allocated to registers r2 r3 
                           2386 ;bytes                     Allocated to registers r4 
                           2387 ;------------------------------------------------------------
                           2388 ;	../../include/ztex-eeprom.h:317: BYTE mac_eeprom_write ( __xdata BYTE *buf, BYTE addr, BYTE length ) {
                           2389 ;	-----------------------------------------
                           2390 ;	 function mac_eeprom_write
                           2391 ;	-----------------------------------------
   06C1                    2392 _mac_eeprom_write:
   06C1 AA 82              2393 	mov	r2,dpl
   06C3 AB 83              2394 	mov	r3,dph
                           2395 ;	../../include/ztex-eeprom.h:318: BYTE bytes = 0;
   06C5 7C 00              2396 	mov	r4,#0x00
                           2397 ;	../../include/ztex-eeprom.h:320: if ( length == 0 ) 
   06C7 E5 13              2398 	mov	a,_mac_eeprom_write_PARM_3
                           2399 ;	../../include/ztex-eeprom.h:321: return 0;
   06C9 70 03              2400 	jnz	00102$
   06CB F5 82              2401 	mov	dpl,a
   06CD 22                 2402 	ret
   06CE                    2403 00102$:
                           2404 ;	../../include/ztex-eeprom.h:323: if ( eeprom_select(EEPROM_MAC_ADDR, 100,0) ) 
   06CE 75 08 64           2405 	mov	_eeprom_select_PARM_2,#0x64
   06D1 75 09 00           2406 	mov	_eeprom_select_PARM_3,#0x00
   06D4 75 82 A6           2407 	mov	dpl,#0xA6
   06D7 C0 02              2408 	push	ar2
   06D9 C0 03              2409 	push	ar3
   06DB C0 04              2410 	push	ar4
   06DD 12 03 46           2411 	lcall	_eeprom_select
   06E0 E5 82              2412 	mov	a,dpl
   06E2 D0 04              2413 	pop	ar4
   06E4 D0 03              2414 	pop	ar3
   06E6 D0 02              2415 	pop	ar2
   06E8 60 03              2416 	jz	00132$
   06EA 02 07 B0           2417 	ljmp	00119$
   06ED                    2418 00132$:
                           2419 ;	../../include/ztex-eeprom.h:326: I2DAT = addr;          	// write address
   06ED 90 E6 79           2420 	mov	dptr,#_I2DAT
   06F0 E5 12              2421 	mov	a,_mac_eeprom_write_PARM_2
   06F2 F0                 2422 	movx	@dptr,a
                           2423 ;	../../include/ztex-eeprom.h:327: if ( i2c_waitWrite() ) goto mac_eeprom_write_end;
   06F3 C0 02              2424 	push	ar2
   06F5 C0 03              2425 	push	ar3
   06F7 C0 04              2426 	push	ar4
   06F9 12 02 BC           2427 	lcall	_i2c_waitWrite
   06FC E5 82              2428 	mov	a,dpl
   06FE D0 04              2429 	pop	ar4
   0700 D0 03              2430 	pop	ar3
   0702 D0 02              2431 	pop	ar2
   0704 60 03              2432 	jz	00133$
   0706 02 07 B0           2433 	ljmp	00119$
   0709                    2434 00133$:
                           2435 ;	../../include/ztex-eeprom.h:329: while ( bytes<length ) {
   0709 AD 12              2436 	mov	r5,_mac_eeprom_write_PARM_2
   070B 7E 00              2437 	mov	r6,#0x00
   070D                    2438 00116$:
   070D C3                 2439 	clr	c
   070E EE                 2440 	mov	a,r6
   070F 95 13              2441 	subb	a,_mac_eeprom_write_PARM_3
   0711 40 03              2442 	jc	00134$
   0713 02 07 A2           2443 	ljmp	00118$
   0716                    2444 00134$:
                           2445 ;	../../include/ztex-eeprom.h:330: I2DAT = *buf;         	// write data 
   0716 8A 82              2446 	mov	dpl,r2
   0718 8B 83              2447 	mov	dph,r3
   071A E0                 2448 	movx	a,@dptr
   071B FF                 2449 	mov	r7,a
   071C A3                 2450 	inc	dptr
   071D AA 82              2451 	mov	r2,dpl
   071F AB 83              2452 	mov	r3,dph
   0721 90 E6 79           2453 	mov	dptr,#_I2DAT
   0724 EF                 2454 	mov	a,r7
   0725 F0                 2455 	movx	@dptr,a
                           2456 ;	../../include/ztex-eeprom.h:331: buf++;
                           2457 ;	../../include/ztex-eeprom.h:332: if ( i2c_waitWrite() ) goto mac_eeprom_write_end;
   0726 C0 02              2458 	push	ar2
   0728 C0 03              2459 	push	ar3
   072A C0 04              2460 	push	ar4
   072C C0 05              2461 	push	ar5
   072E C0 06              2462 	push	ar6
   0730 12 02 BC           2463 	lcall	_i2c_waitWrite
   0733 E5 82              2464 	mov	a,dpl
   0735 D0 06              2465 	pop	ar6
   0737 D0 05              2466 	pop	ar5
   0739 D0 04              2467 	pop	ar4
   073B D0 03              2468 	pop	ar3
   073D D0 02              2469 	pop	ar2
   073F 70 6F              2470 	jnz	00119$
                           2471 ;	../../include/ztex-eeprom.h:334: addr++;
   0741 0D                 2472 	inc	r5
   0742 8D 12              2473 	mov	_mac_eeprom_write_PARM_2,r5
                           2474 ;	../../include/ztex-eeprom.h:335: bytes++;
   0744 0E                 2475 	inc	r6
   0745 8E 04              2476 	mov	ar4,r6
                           2477 ;	../../include/ztex-eeprom.h:336: if ( ( (addr & 8) == 0 ) && ( bytes<length ) ) {
   0747 ED                 2478 	mov	a,r5
   0748 20 E3 C2           2479 	jb	acc.3,00116$
   074B C3                 2480 	clr	c
   074C EE                 2481 	mov	a,r6
   074D 95 13              2482 	subb	a,_mac_eeprom_write_PARM_3
   074F 50 BC              2483 	jnc	00116$
                           2484 ;	../../include/ztex-eeprom.h:337: I2CS |= bmBIT6;		// stop bit
   0751 90 E6 78           2485 	mov	dptr,#_I2CS
   0754 E0                 2486 	movx	a,@dptr
   0755 44 40              2487 	orl	a,#0x40
   0757 F0                 2488 	movx	@dptr,a
                           2489 ;	../../include/ztex-eeprom.h:338: i2c_waitStop();
   0758 C0 02              2490 	push	ar2
   075A C0 03              2491 	push	ar3
   075C C0 04              2492 	push	ar4
   075E C0 05              2493 	push	ar5
   0760 C0 06              2494 	push	ar6
   0762 12 03 2C           2495 	lcall	_i2c_waitStop
                           2496 ;	../../include/ztex-eeprom.h:340: if ( eeprom_select(EEPROM_MAC_ADDR, 100,0) ) 
   0765 75 08 64           2497 	mov	_eeprom_select_PARM_2,#0x64
   0768 75 09 00           2498 	mov	_eeprom_select_PARM_3,#0x00
   076B 75 82 A6           2499 	mov	dpl,#0xA6
   076E 12 03 46           2500 	lcall	_eeprom_select
   0771 E5 82              2501 	mov	a,dpl
   0773 D0 06              2502 	pop	ar6
   0775 D0 05              2503 	pop	ar5
   0777 D0 04              2504 	pop	ar4
   0779 D0 03              2505 	pop	ar3
   077B D0 02              2506 	pop	ar2
   077D 70 31              2507 	jnz	00119$
                           2508 ;	../../include/ztex-eeprom.h:343: I2DAT = addr;          	// write address
   077F 90 E6 79           2509 	mov	dptr,#_I2DAT
   0782 ED                 2510 	mov	a,r5
   0783 F0                 2511 	movx	@dptr,a
                           2512 ;	../../include/ztex-eeprom.h:344: if ( i2c_waitWrite() ) goto mac_eeprom_write_end;
   0784 C0 02              2513 	push	ar2
   0786 C0 03              2514 	push	ar3
   0788 C0 04              2515 	push	ar4
   078A C0 05              2516 	push	ar5
   078C C0 06              2517 	push	ar6
   078E 12 02 BC           2518 	lcall	_i2c_waitWrite
   0791 E5 82              2519 	mov	a,dpl
   0793 D0 06              2520 	pop	ar6
   0795 D0 05              2521 	pop	ar5
   0797 D0 04              2522 	pop	ar4
   0799 D0 03              2523 	pop	ar3
   079B D0 02              2524 	pop	ar2
   079D 70 11              2525 	jnz	00119$
   079F 02 07 0D           2526 	ljmp	00116$
   07A2                    2527 00118$:
                           2528 ;	../../include/ztex-eeprom.h:347: I2CS |= bmBIT6;		// stop bit
   07A2 90 E6 78           2529 	mov	dptr,#_I2CS
   07A5 E0                 2530 	movx	a,@dptr
   07A6 44 40              2531 	orl	a,#0x40
   07A8 F0                 2532 	movx	@dptr,a
                           2533 ;	../../include/ztex-eeprom.h:348: i2c_waitStop();
   07A9 C0 04              2534 	push	ar4
   07AB 12 03 2C           2535 	lcall	_i2c_waitStop
   07AE D0 04              2536 	pop	ar4
                           2537 ;	../../include/ztex-eeprom.h:350: mac_eeprom_write_end:
   07B0                    2538 00119$:
                           2539 ;	../../include/ztex-eeprom.h:351: mac_eeprom_addr = addr;
   07B0 90 3A 05           2540 	mov	dptr,#_mac_eeprom_addr
   07B3 E5 12              2541 	mov	a,_mac_eeprom_write_PARM_2
   07B5 F0                 2542 	movx	@dptr,a
                           2543 ;	../../include/ztex-eeprom.h:352: return bytes;
   07B6 8C 82              2544 	mov	dpl,r4
   07B8 22                 2545 	ret
                           2546 ;------------------------------------------------------------
                           2547 ;Allocation info for local variables in function 'mac_eeprom_read_ep0'
                           2548 ;------------------------------------------------------------
                           2549 ;i                         Allocated to registers r3 
                           2550 ;b                         Allocated to registers r2 
                           2551 ;------------------------------------------------------------
                           2552 ;	../../include/ztex-eeprom.h:358: BYTE mac_eeprom_read_ep0 () { 
                           2553 ;	-----------------------------------------
                           2554 ;	 function mac_eeprom_read_ep0
                           2555 ;	-----------------------------------------
   07B9                    2556 _mac_eeprom_read_ep0:
                           2557 ;	../../include/ztex-eeprom.h:360: b = ep0_payload_transfer;
   07B9 90 3A 38           2558 	mov	dptr,#_ep0_payload_transfer
   07BC E0                 2559 	movx	a,@dptr
   07BD FA                 2560 	mov	r2,a
                           2561 ;	../../include/ztex-eeprom.h:361: i = mac_eeprom_read(EP0BUF, mac_eeprom_addr, b);
   07BE 90 3A 05           2562 	mov	dptr,#_mac_eeprom_addr
   07C1 E0                 2563 	movx	a,@dptr
   07C2 F5 10              2564 	mov	_mac_eeprom_read_PARM_2,a
   07C4 8A 11              2565 	mov	_mac_eeprom_read_PARM_3,r2
   07C6 90 E7 40           2566 	mov	dptr,#_EP0BUF
   07C9 C0 02              2567 	push	ar2
   07CB 12 05 D4           2568 	lcall	_mac_eeprom_read
   07CE AB 82              2569 	mov	r3,dpl
   07D0 D0 02              2570 	pop	ar2
                           2571 ;	../../include/ztex-eeprom.h:362: mac_eeprom_addr += b;
   07D2 90 3A 05           2572 	mov	dptr,#_mac_eeprom_addr
   07D5 E0                 2573 	movx	a,@dptr
   07D6 FC                 2574 	mov	r4,a
   07D7 EA                 2575 	mov	a,r2
   07D8 2C                 2576 	add	a,r4
   07D9 F0                 2577 	movx	@dptr,a
                           2578 ;	../../include/ztex-eeprom.h:363: return i;
   07DA 8B 82              2579 	mov	dpl,r3
   07DC 22                 2580 	ret
                           2581 ;------------------------------------------------------------
                           2582 ;Allocation info for local variables in function 'spi_clocks'
                           2583 ;------------------------------------------------------------
                           2584 ;c                         Allocated to registers 
                           2585 ;------------------------------------------------------------
                           2586 ;	../../include/ztex-flash2.h:98: void spi_clocks (BYTE c) {
                           2587 ;	-----------------------------------------
                           2588 ;	 function spi_clocks
                           2589 ;	-----------------------------------------
   07DD                    2590 _spi_clocks:
                           2591 ;	../../include/ztex-flash2.h:110: }
                           2592 	
   07DD AA 82              2593 	 mov r2,dpl
   07DF                    2594 	010014$:
   07DF D2 A6              2595 	        setb _IOC6
   07E1 00                 2596 	        nop
   07E2 00                 2597 	        nop
   07E3 00                 2598 	        nop
   07E4 C2 A6              2599 	        clr _IOC6
   07E6 DA F7              2600 	 djnz r2,010014$
                           2601 ;	# 109 "../../include/ztex-flash2.h"
   07E8 22                 2602 	ret
                           2603 ;------------------------------------------------------------
                           2604 ;Allocation info for local variables in function 'flash_read_byte'
                           2605 ;------------------------------------------------------------
                           2606 ;------------------------------------------------------------
                           2607 ;	../../include/ztex-flash2.h:118: __asm  
                           2608 ;	-----------------------------------------
                           2609 ;	 function flash_read_byte
                           2610 ;	-----------------------------------------
   07E9                    2611 _flash_read_byte:
                           2612 ;	../../include/ztex-flash2.h:169: void flash_read(__xdata BYTE *buf, BYTE len) {
                           2613 	
                           2614 	
   07E9 A2 A4              2615 	 mov c,_IOC4
                           2616 ;	# 121 "../../include/ztex-flash2.h"
   07EB D2 A6              2617 	        setb _IOC6
   07ED 33                 2618 	        rlc a
   07EE C2 A6              2619 	        clr _IOC6
                           2620 	
   07F0 A2 A4              2621 	        mov c,_IOC4
                           2622 ;	# 126 "../../include/ztex-flash2.h"
   07F2 D2 A6              2623 	        setb _IOC6
   07F4 33                 2624 	        rlc a
   07F5 C2 A6              2625 	        clr _IOC6
                           2626 	
   07F7 A2 A4              2627 	        mov c,_IOC4
                           2628 ;	# 131 "../../include/ztex-flash2.h"
   07F9 D2 A6              2629 	        setb _IOC6
   07FB 33                 2630 	        rlc a
   07FC C2 A6              2631 	        clr _IOC6
                           2632 	
   07FE A2 A4              2633 	        mov c,_IOC4
                           2634 ;	# 136 "../../include/ztex-flash2.h"
   0800 D2 A6              2635 	        setb _IOC6
   0802 33                 2636 	        rlc a
   0803 C2 A6              2637 	        clr _IOC6
                           2638 	
   0805 A2 A4              2639 	        mov c,_IOC4
                           2640 ;	# 141 "../../include/ztex-flash2.h"
   0807 D2 A6              2641 	        setb _IOC6
   0809 33                 2642 	        rlc a
   080A C2 A6              2643 	        clr _IOC6
                           2644 	
   080C A2 A4              2645 	        mov c,_IOC4
                           2646 ;	# 146 "../../include/ztex-flash2.h"
   080E D2 A6              2647 	        setb _IOC6
   0810 33                 2648 	        rlc a
   0811 C2 A6              2649 	        clr _IOC6
                           2650 	
   0813 A2 A4              2651 	        mov c,_IOC4
                           2652 ;	# 151 "../../include/ztex-flash2.h"
   0815 D2 A6              2653 	        setb _IOC6
   0817 33                 2654 	        rlc a
   0818 C2 A6              2655 	        clr _IOC6
                           2656 	
   081A A2 A4              2657 	        mov c,_IOC4
                           2658 ;	# 156 "../../include/ztex-flash2.h"
   081C D2 A6              2659 	        setb _IOC6
   081E 33                 2660 	        rlc a
   081F C2 A6              2661 	        clr _IOC6
   0821 F5 82              2662 	        mov dpl,a
   0823 22                 2663 	        ret
                           2664 ;	../../include/ztex-flash2.h:170: *buf;					// this avoids stupid warnings
   0824 75 82 00           2665 	mov	dpl,#0x00
   0827 22                 2666 	ret
                           2667 ;------------------------------------------------------------
                           2668 ;Allocation info for local variables in function 'flash_read'
                           2669 ;------------------------------------------------------------
                           2670 ;len                       Allocated with name '_flash_read_PARM_2'
                           2671 ;buf                       Allocated to registers 
                           2672 ;------------------------------------------------------------
                           2673 ;	../../include/ztex-flash2.h:169: void flash_read(__xdata BYTE *buf, BYTE len) {
                           2674 ;	-----------------------------------------
                           2675 ;	 function flash_read
                           2676 ;	-----------------------------------------
   0828                    2677 _flash_read:
                           2678 ;	../../include/ztex-flash2.h:228: __asm
                           2679 	
                           2680 ;	# 173 "../../include/ztex-flash2.h"
   0828 AA 16              2681 	 mov r2,_flash_read_PARM_2
   082A                    2682 	010012$:
                           2683 	
   082A A2 A4              2684 	 mov c,_IOC4
                           2685 ;	# 177 "../../include/ztex-flash2.h"
   082C D2 A6              2686 	        setb _IOC6
   082E 33                 2687 	        rlc a
   082F C2 A6              2688 	        clr _IOC6
                           2689 	
   0831 A2 A4              2690 	        mov c,_IOC4
                           2691 ;	# 182 "../../include/ztex-flash2.h"
   0833 D2 A6              2692 	        setb _IOC6
   0835 33                 2693 	        rlc a
   0836 C2 A6              2694 	        clr _IOC6
                           2695 	
   0838 A2 A4              2696 	        mov c,_IOC4
                           2697 ;	# 187 "../../include/ztex-flash2.h"
   083A D2 A6              2698 	        setb _IOC6
   083C 33                 2699 	        rlc a
   083D C2 A6              2700 	        clr _IOC6
                           2701 	
   083F A2 A4              2702 	        mov c,_IOC4
                           2703 ;	# 192 "../../include/ztex-flash2.h"
   0841 D2 A6              2704 	        setb _IOC6
   0843 33                 2705 	        rlc a
   0844 C2 A6              2706 	        clr _IOC6
                           2707 	
   0846 A2 A4              2708 	        mov c,_IOC4
                           2709 ;	# 197 "../../include/ztex-flash2.h"
   0848 D2 A6              2710 	        setb _IOC6
   084A 33                 2711 	        rlc a
   084B C2 A6              2712 	        clr _IOC6
                           2713 	
   084D A2 A4              2714 	        mov c,_IOC4
                           2715 ;	# 202 "../../include/ztex-flash2.h"
   084F D2 A6              2716 	        setb _IOC6
   0851 33                 2717 	        rlc a
   0852 C2 A6              2718 	        clr _IOC6
                           2719 	
   0854 A2 A4              2720 	        mov c,_IOC4
                           2721 ;	# 207 "../../include/ztex-flash2.h"
   0856 D2 A6              2722 	        setb _IOC6
   0858 33                 2723 	        rlc a
   0859 C2 A6              2724 	        clr _IOC6
                           2725 	
   085B A2 A4              2726 	        mov c,_IOC4
                           2727 ;	# 212 "../../include/ztex-flash2.h"
   085D D2 A6              2728 	        setb _IOC6
   085F 33                 2729 	        rlc a
   0860 C2 A6              2730 	        clr _IOC6
                           2731 	
   0862 F0                 2732 	 movx @dptr,a
   0863 A3                 2733 	 inc dptr
   0864 DA C4              2734 	 djnz r2,010012$
   0866 22                 2735 	ret
                           2736 ;------------------------------------------------------------
                           2737 ;Allocation info for local variables in function 'spi_write_byte'
                           2738 ;------------------------------------------------------------
                           2739 ;b                         Allocated to registers 
                           2740 ;------------------------------------------------------------
                           2741 ;	../../include/ztex-flash2.h:235: rlc	a		// 6
                           2742 ;	-----------------------------------------
                           2743 ;	 function spi_write_byte
                           2744 ;	-----------------------------------------
   0867                    2745 _spi_write_byte:
                           2746 ;	../../include/ztex-flash2.h:280: *buf;					// this avoids stupid warnings
                           2747 	
                           2748 ;	# 230 "../../include/ztex-flash2.h"
   0867 E5 82              2749 	 mov a,dpl
   0869 33                 2750 	 rlc a
                           2751 ;	# 232 "../../include/ztex-flash2.h"
                           2752 	
   086A 92 A7              2753 	 mov _IOC7,c
   086C D2 A6              2754 	        setb _IOC6
   086E 33                 2755 	 rlc a
                           2756 ;	# 236 "../../include/ztex-flash2.h"
   086F C2 A6              2757 	        clr _IOC6
                           2758 	
   0871 92 A7              2759 	 mov _IOC7,c
   0873 D2 A6              2760 	        setb _IOC6
   0875 33                 2761 	 rlc a
                           2762 ;	# 241 "../../include/ztex-flash2.h"
   0876 C2 A6              2763 	        clr _IOC6
                           2764 	
   0878 92 A7              2765 	 mov _IOC7,c
   087A D2 A6              2766 	        setb _IOC6
   087C 33                 2767 	 rlc a
                           2768 ;	# 246 "../../include/ztex-flash2.h"
   087D C2 A6              2769 	        clr _IOC6
                           2770 	
   087F 92 A7              2771 	 mov _IOC7,c
   0881 D2 A6              2772 	        setb _IOC6
   0883 33                 2773 	 rlc a
                           2774 ;	# 251 "../../include/ztex-flash2.h"
   0884 C2 A6              2775 	        clr _IOC6
                           2776 	
   0886 92 A7              2777 	 mov _IOC7,c
   0888 D2 A6              2778 	        setb _IOC6
   088A 33                 2779 	 rlc a
                           2780 ;	# 256 "../../include/ztex-flash2.h"
   088B C2 A6              2781 	        clr _IOC6
                           2782 	
   088D 92 A7              2783 	 mov _IOC7,c
   088F D2 A6              2784 	        setb _IOC6
   0891 33                 2785 	 rlc a
                           2786 ;	# 261 "../../include/ztex-flash2.h"
   0892 C2 A6              2787 	        clr _IOC6
                           2788 	
   0894 92 A7              2789 	 mov _IOC7,c
   0896 D2 A6              2790 	        setb _IOC6
   0898 33                 2791 	 rlc a
                           2792 ;	# 266 "../../include/ztex-flash2.h"
   0899 C2 A6              2793 	        clr _IOC6
                           2794 	
   089B 92 A7              2795 	 mov _IOC7,c
   089D D2 A6              2796 	        setb _IOC6
   089F 00                 2797 	 nop
   08A0 C2 A6              2798 	        clr _IOC6
   08A2 22                 2799 	ret
                           2800 ;------------------------------------------------------------
                           2801 ;Allocation info for local variables in function 'spi_write'
                           2802 ;------------------------------------------------------------
                           2803 ;len                       Allocated with name '_spi_write_PARM_2'
                           2804 ;buf                       Allocated to registers 
                           2805 ;------------------------------------------------------------
                           2806 ;	../../include/ztex-flash2.h:279: void spi_write(__xdata BYTE *buf, BYTE len) {
                           2807 ;	-----------------------------------------
                           2808 ;	 function spi_write
                           2809 ;	-----------------------------------------
   08A3                    2810 _spi_write:
                           2811 ;	../../include/ztex-flash2.h:339: void spi_select() {
                           2812 	
                           2813 ;	# 283 "../../include/ztex-flash2.h"
   08A3 AA 16              2814 	 mov r2,_flash_read_PARM_2
   08A5                    2815 	010013$:
                           2816 ;	# 286 "../../include/ztex-flash2.h"
   08A5 E0                 2817 	 movx a,@dptr
   08A6 33                 2818 	 rlc a
                           2819 ;	# 288 "../../include/ztex-flash2.h"
                           2820 	
   08A7 92 A7              2821 	 mov _IOC7,c
   08A9 D2 A6              2822 	        setb _IOC6
   08AB 33                 2823 	 rlc a
                           2824 ;	# 292 "../../include/ztex-flash2.h"
   08AC C2 A6              2825 	        clr _IOC6
                           2826 	
   08AE 92 A7              2827 	 mov _IOC7,c
   08B0 D2 A6              2828 	        setb _IOC6
   08B2 33                 2829 	 rlc a
                           2830 ;	# 297 "../../include/ztex-flash2.h"
   08B3 C2 A6              2831 	        clr _IOC6
                           2832 	
   08B5 92 A7              2833 	 mov _IOC7,c
   08B7 D2 A6              2834 	        setb _IOC6
   08B9 33                 2835 	 rlc a
                           2836 ;	# 302 "../../include/ztex-flash2.h"
   08BA C2 A6              2837 	        clr _IOC6
                           2838 	
   08BC 92 A7              2839 	 mov _IOC7,c
   08BE D2 A6              2840 	        setb _IOC6
   08C0 33                 2841 	 rlc a
                           2842 ;	# 307 "../../include/ztex-flash2.h"
   08C1 C2 A6              2843 	        clr _IOC6
                           2844 	
   08C3 92 A7              2845 	 mov _IOC7,c
   08C5 D2 A6              2846 	        setb _IOC6
   08C7 33                 2847 	 rlc a
                           2848 ;	# 312 "../../include/ztex-flash2.h"
   08C8 C2 A6              2849 	        clr _IOC6
                           2850 	
   08CA 92 A7              2851 	 mov _IOC7,c
   08CC D2 A6              2852 	        setb _IOC6
   08CE 33                 2853 	 rlc a
                           2854 ;	# 317 "../../include/ztex-flash2.h"
   08CF C2 A6              2855 	        clr _IOC6
                           2856 	
   08D1 92 A7              2857 	 mov _IOC7,c
   08D3 D2 A6              2858 	        setb _IOC6
   08D5 33                 2859 	 rlc a
                           2860 ;	# 322 "../../include/ztex-flash2.h"
   08D6 C2 A6              2861 	        clr _IOC6
                           2862 	
   08D8 92 A7              2863 	 mov _IOC7,c
   08DA D2 A6              2864 	        setb _IOC6
   08DC A3                 2865 	 inc dptr
   08DD C2 A6              2866 	        clr _IOC6
                           2867 	
   08DF DA C4              2868 	 djnz r2,010013$
   08E1 22                 2869 	ret
                           2870 ;------------------------------------------------------------
                           2871 ;Allocation info for local variables in function 'spi_select'
                           2872 ;------------------------------------------------------------
                           2873 ;------------------------------------------------------------
                           2874 ;	../../include/ztex-flash2.h:348: // de-select the flash (CS)
                           2875 ;	-----------------------------------------
                           2876 ;	 function spi_select
                           2877 ;	-----------------------------------------
   08E2                    2878 _spi_select:
                           2879 ;	../../include/ztex-flash2.h:349: void spi_deselect() {
   08E2 D2 A5              2880 	setb	_IOC5
                           2881 ;	../../include/ztex-flash2.h:350: SPI_CS = 1;					// CS = 1;
   08E4 75 82 08           2882 	mov	dpl,#0x08
   08E7 12 07 DD           2883 	lcall	_spi_clocks
                           2884 ;	../../include/ztex-flash2.h:342: SPI_CS = 0;
   08EA C2 A5              2885 	clr	_IOC5
   08EC 22                 2886 	ret
                           2887 ;------------------------------------------------------------
                           2888 ;Allocation info for local variables in function 'spi_deselect'
                           2889 ;------------------------------------------------------------
                           2890 ;------------------------------------------------------------
                           2891 ;	../../include/ztex-flash2.h:349: void spi_deselect() {
                           2892 ;	-----------------------------------------
                           2893 ;	 function spi_deselect
                           2894 ;	-----------------------------------------
   08ED                    2895 _spi_deselect:
                           2896 ;	../../include/ztex-flash2.h:350: SPI_CS = 1;					// CS = 1;
   08ED D2 A5              2897 	setb	_IOC5
                           2898 ;	../../include/ztex-flash2.h:351: spi_clocks(8);				// 8 dummy clocks to finish a previous command
   08EF 75 82 08           2899 	mov	dpl,#0x08
   08F2 02 07 DD           2900 	ljmp	_spi_clocks
                           2901 ;------------------------------------------------------------
                           2902 ;Allocation info for local variables in function 'spi_wait'
                           2903 ;------------------------------------------------------------
                           2904 ;i                         Allocated to registers r2 r3 
                           2905 ;------------------------------------------------------------
                           2906 ;	../../include/ztex-flash2.h:371: BYTE spi_wait() {
                           2907 ;	-----------------------------------------
                           2908 ;	 function spi_wait
                           2909 ;	-----------------------------------------
   08F5                    2910 _spi_wait:
                           2911 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   08F5 90 3A 13           2912 	mov	dptr,#_spi_last_cmd
   08F8 74 05              2913 	mov	a,#0x05
   08FA F0                 2914 	movx	@dptr,a
                           2915 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   08FB 12 08 E2           2916 	lcall	_spi_select
                           2917 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   08FE 75 82 05           2918 	mov	dpl,#0x05
   0901 12 08 67           2919 	lcall	_spi_write_byte
                           2920 ;	../../include/ztex-flash2.h:375: for (i=0; (flash_read_byte() & bmBIT0) && i<65535; i++ ) { 
   0904 7A 00              2921 	mov	r2,#0x00
   0906 7B 00              2922 	mov	r3,#0x00
   0908                    2923 00102$:
   0908 C0 02              2924 	push	ar2
   090A C0 03              2925 	push	ar3
   090C 12 07 E9           2926 	lcall	_flash_read_byte
   090F E5 82              2927 	mov	a,dpl
   0911 D0 03              2928 	pop	ar3
   0913 D0 02              2929 	pop	ar2
   0915 30 E0 2E           2930 	jnb	acc.0,00105$
   0918 8A 04              2931 	mov	ar4,r2
   091A 8B 05              2932 	mov	ar5,r3
   091C 7E 00              2933 	mov	r6,#0x00
   091E 7F 00              2934 	mov	r7,#0x00
   0920 C3                 2935 	clr	c
   0921 EC                 2936 	mov	a,r4
   0922 94 FF              2937 	subb	a,#0xFF
   0924 ED                 2938 	mov	a,r5
   0925 94 FF              2939 	subb	a,#0xFF
   0927 EE                 2940 	mov	a,r6
   0928 94 00              2941 	subb	a,#0x00
   092A EF                 2942 	mov	a,r7
   092B 64 80              2943 	xrl	a,#0x80
   092D 94 80              2944 	subb	a,#0x80
   092F 50 15              2945 	jnc	00105$
                           2946 ;	../../include/ztex-flash2.h:376: spi_clocks(0);				// 256 dummy clocks
   0931 75 82 00           2947 	mov	dpl,#0x00
   0934 C0 02              2948 	push	ar2
   0936 C0 03              2949 	push	ar3
   0938 12 07 DD           2950 	lcall	_spi_clocks
   093B D0 03              2951 	pop	ar3
   093D D0 02              2952 	pop	ar2
                           2953 ;	../../include/ztex-flash2.h:375: for (i=0; (flash_read_byte() & bmBIT0) && i<65535; i++ ) { 
   093F 0A                 2954 	inc	r2
   0940 BA 00 C5           2955 	cjne	r2,#0x00,00102$
   0943 0B                 2956 	inc	r3
   0944 80 C2              2957 	sjmp	00102$
   0946                    2958 00105$:
                           2959 ;	../../include/ztex-flash2.h:379: flash_ec = flash_read_byte() & bmBIT0 ? FLASH_EC_TIMEOUT : 0;
   0946 12 07 E9           2960 	lcall	_flash_read_byte
   0949 E5 82              2961 	mov	a,dpl
   094B 30 E0 04           2962 	jnb	acc.0,00108$
   094E 7A 02              2963 	mov	r2,#0x02
   0950 80 02              2964 	sjmp	00109$
   0952                    2965 00108$:
   0952 7A 00              2966 	mov	r2,#0x00
   0954                    2967 00109$:
   0954 90 3A 0E           2968 	mov	dptr,#_flash_ec
   0957 EA                 2969 	mov	a,r2
   0958 F0                 2970 	movx	@dptr,a
                           2971 ;	../../include/ztex-flash2.h:380: spi_deselect();
   0959 12 08 ED           2972 	lcall	_spi_deselect
                           2973 ;	../../include/ztex-flash2.h:381: return flash_ec;
   095C 90 3A 0E           2974 	mov	dptr,#_flash_ec
   095F E0                 2975 	movx	a,@dptr
   0960 F5 82              2976 	mov	dpl,a
   0962 22                 2977 	ret
                           2978 ;------------------------------------------------------------
                           2979 ;Allocation info for local variables in function 'flash_read_init'
                           2980 ;------------------------------------------------------------
                           2981 ;s                         Allocated to registers r2 r3 
                           2982 ;------------------------------------------------------------
                           2983 ;	../../include/ztex-flash2.h:391: BYTE flash_read_init(WORD s) {
                           2984 ;	-----------------------------------------
                           2985 ;	 function flash_read_init
                           2986 ;	-----------------------------------------
   0963                    2987 _flash_read_init:
   0963 AA 82              2988 	mov	r2,dpl
   0965 AB 83              2989 	mov	r3,dph
                           2990 ;	../../include/ztex-flash2.h:400: s = s << ((BYTE)flash_sector_size - 8);     
   0967 20 A5 0A           2991 	jb	_IOC5,00102$
                           2992 ;	../../include/ztex-flash2.h:393: flash_ec = FLASH_EC_PENDING;
   096A 90 3A 0E           2993 	mov	dptr,#_flash_ec
   096D 74 04              2994 	mov	a,#0x04
   096F F0                 2995 	movx	@dptr,a
                           2996 ;	../../include/ztex-flash2.h:394: return FLASH_EC_PENDING;		// we interrupted a pending Flash operation
   0970 75 82 04           2997 	mov	dpl,#0x04
   0973 22                 2998 	ret
   0974                    2999 00102$:
                           3000 ;	../../include/ztex-flash2.h:396: if ( spi_wait() ) {
   0974 C0 02              3001 	push	ar2
   0976 C0 03              3002 	push	ar3
   0978 12 08 F5           3003 	lcall	_spi_wait
   097B E5 82              3004 	mov	a,dpl
   097D D0 03              3005 	pop	ar3
   097F D0 02              3006 	pop	ar2
   0981 60 07              3007 	jz	00104$
                           3008 ;	../../include/ztex-flash2.h:397: return flash_ec;
   0983 90 3A 0E           3009 	mov	dptr,#_flash_ec
   0986 E0                 3010 	movx	a,@dptr
   0987 F5 82              3011 	mov	dpl,a
   0989 22                 3012 	ret
   098A                    3013 00104$:
                           3014 ;	../../include/ztex-flash2.h:400: s = s << ((BYTE)flash_sector_size - 8);     
   098A 90 3A 08           3015 	mov	dptr,#_flash_sector_size
   098D E0                 3016 	movx	a,@dptr
   098E FC                 3017 	mov	r4,a
   098F A3                 3018 	inc	dptr
   0990 E0                 3019 	movx	a,@dptr
   0991 7D 00              3020 	mov	r5,#0x00
   0993 EC                 3021 	mov	a,r4
   0994 24 F8              3022 	add	a,#0xf8
   0996 FC                 3023 	mov	r4,a
   0997 ED                 3024 	mov	a,r5
   0998 34 FF              3025 	addc	a,#0xff
   099A FD                 3026 	mov	r5,a
   099B 8C F0              3027 	mov	b,r4
   099D 05 F0              3028 	inc	b
   099F 80 06              3029 	sjmp	00112$
   09A1                    3030 00111$:
   09A1 EA                 3031 	mov	a,r2
   09A2 2A                 3032 	add	a,r2
   09A3 FA                 3033 	mov	r2,a
   09A4 EB                 3034 	mov	a,r3
   09A5 33                 3035 	rlc	a
   09A6 FB                 3036 	mov	r3,a
   09A7                    3037 00112$:
   09A7 D5 F0 F7           3038 	djnz	b,00111$
                           3039 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   09AA 90 3A 13           3040 	mov	dptr,#_spi_last_cmd
   09AD 74 0B              3041 	mov	a,#0x0B
   09AF F0                 3042 	movx	@dptr,a
                           3043 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   09B0 C0 02              3044 	push	ar2
   09B2 C0 03              3045 	push	ar3
   09B4 12 08 E2           3046 	lcall	_spi_select
                           3047 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   09B7 75 82 0B           3048 	mov	dpl,#0x0B
   09BA 12 08 67           3049 	lcall	_spi_write_byte
   09BD D0 03              3050 	pop	ar3
                           3051 ;	../../include/ztex-flash2.h:363: 
   09BF 8B 82              3052 	mov	dpl,r3
   09C1 C0 03              3053 	push	ar3
   09C3 12 08 67           3054 	lcall	_spi_write_byte
   09C6 D0 03              3055 	pop	ar3
   09C8 D0 02              3056 	pop	ar2
                           3057 ;	../../include/ztex-flash2.h:403: spi_write_byte(s & 255);
   09CA 8A 82              3058 	mov	dpl,r2
   09CC 12 08 67           3059 	lcall	_spi_write_byte
                           3060 ;	../../include/ztex-flash2.h:404: spi_write_byte(0);
   09CF 75 82 00           3061 	mov	dpl,#0x00
   09D2 12 08 67           3062 	lcall	_spi_write_byte
                           3063 ;	../../include/ztex-flash2.h:405: spi_clocks(8);				// 8 dummy clocks
   09D5 75 82 08           3064 	mov	dpl,#0x08
   09D8 12 07 DD           3065 	lcall	_spi_clocks
                           3066 ;	../../include/ztex-flash2.h:406: return 0;
   09DB 75 82 00           3067 	mov	dpl,#0x00
   09DE 22                 3068 	ret
                           3069 ;------------------------------------------------------------
                           3070 ;Allocation info for local variables in function 'flash_read_next'
                           3071 ;------------------------------------------------------------
                           3072 ;------------------------------------------------------------
                           3073 ;	../../include/ztex-flash2.h:415: BYTE flash_read_next() {
                           3074 ;	-----------------------------------------
                           3075 ;	 function flash_read_next
                           3076 ;	-----------------------------------------
   09DF                    3077 _flash_read_next:
                           3078 ;	../../include/ztex-flash2.h:416: return 0;
   09DF 75 82 00           3079 	mov	dpl,#0x00
   09E2 22                 3080 	ret
                           3081 ;------------------------------------------------------------
                           3082 ;Allocation info for local variables in function 'flash_read_finish'
                           3083 ;------------------------------------------------------------
                           3084 ;n                         Allocated to registers 
                           3085 ;------------------------------------------------------------
                           3086 ;	../../include/ztex-flash2.h:426: void flash_read_finish(WORD n) {
                           3087 ;	-----------------------------------------
                           3088 ;	 function flash_read_finish
                           3089 ;	-----------------------------------------
   09E3                    3090 _flash_read_finish:
                           3091 ;	../../include/ztex-flash2.h:428: spi_deselect();
   09E3 02 08 ED           3092 	ljmp	_spi_deselect
                           3093 ;------------------------------------------------------------
                           3094 ;Allocation info for local variables in function 'spi_pp'
                           3095 ;------------------------------------------------------------
                           3096 ;------------------------------------------------------------
                           3097 ;	../../include/ztex-flash2.h:435: BYTE spi_pp () {	
                           3098 ;	-----------------------------------------
                           3099 ;	 function spi_pp
                           3100 ;	-----------------------------------------
   09E6                    3101 _spi_pp:
                           3102 ;	../../include/ztex-flash2.h:436: spi_deselect();				// finish previous write cmd
   09E6 12 08 ED           3103 	lcall	_spi_deselect
                           3104 ;	../../include/ztex-flash2.h:438: spi_need_pp = 0;
   09E9 90 3A 1B           3105 	mov	dptr,#_spi_need_pp
   09EC E4                 3106 	clr	a
   09ED F0                 3107 	movx	@dptr,a
                           3108 ;	../../include/ztex-flash2.h:440: if ( spi_wait() ) {
   09EE 12 08 F5           3109 	lcall	_spi_wait
   09F1 E5 82              3110 	mov	a,dpl
   09F3 60 07              3111 	jz	00102$
                           3112 ;	../../include/ztex-flash2.h:441: return flash_ec;
   09F5 90 3A 0E           3113 	mov	dptr,#_flash_ec
   09F8 E0                 3114 	movx	a,@dptr
   09F9 F5 82              3115 	mov	dpl,a
   09FB 22                 3116 	ret
   09FC                    3117 00102$:
                           3118 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   09FC 90 3A 13           3119 	mov	dptr,#_spi_last_cmd
   09FF 74 06              3120 	mov	a,#0x06
   0A01 F0                 3121 	movx	@dptr,a
                           3122 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0A02 12 08 E2           3123 	lcall	_spi_select
                           3124 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0A05 75 82 06           3125 	mov	dpl,#0x06
   0A08 12 08 67           3126 	lcall	_spi_write_byte
                           3127 ;	../../include/ztex-flash2.h:444: spi_deselect();
   0A0B 12 08 ED           3128 	lcall	_spi_deselect
                           3129 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   0A0E 90 3A 13           3130 	mov	dptr,#_spi_last_cmd
   0A11 74 02              3131 	mov	a,#0x02
   0A13 F0                 3132 	movx	@dptr,a
                           3133 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0A14 12 08 E2           3134 	lcall	_spi_select
                           3135 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0A17 75 82 02           3136 	mov	dpl,#0x02
   0A1A 12 08 67           3137 	lcall	_spi_write_byte
                           3138 ;	../../include/ztex-flash2.h:363: 
   0A1D 90 3A 18           3139 	mov	dptr,#_spi_write_addr_hi
   0A20 E0                 3140 	movx	a,@dptr
   0A21 A3                 3141 	inc	dptr
   0A22 E0                 3142 	movx	a,@dptr
   0A23 F5 82              3143 	mov	dpl,a
   0A25 12 08 67           3144 	lcall	_spi_write_byte
                           3145 ;	../../include/ztex-flash2.h:448: spi_write_byte(spi_write_addr_hi & 255);
   0A28 90 3A 18           3146 	mov	dptr,#_spi_write_addr_hi
   0A2B E0                 3147 	movx	a,@dptr
   0A2C FA                 3148 	mov	r2,a
   0A2D A3                 3149 	inc	dptr
   0A2E E0                 3150 	movx	a,@dptr
   0A2F 8A 82              3151 	mov	dpl,r2
   0A31 12 08 67           3152 	lcall	_spi_write_byte
                           3153 ;	../../include/ztex-flash2.h:449: spi_write_byte(0);
   0A34 75 82 00           3154 	mov	dpl,#0x00
   0A37 12 08 67           3155 	lcall	_spi_write_byte
                           3156 ;	../../include/ztex-flash2.h:450: return 0;
   0A3A 75 82 00           3157 	mov	dpl,#0x00
   0A3D 22                 3158 	ret
                           3159 ;------------------------------------------------------------
                           3160 ;Allocation info for local variables in function 'flash_write_byte'
                           3161 ;------------------------------------------------------------
                           3162 ;b                         Allocated to registers r2 
                           3163 ;------------------------------------------------------------
                           3164 ;	../../include/ztex-flash2.h:457: BYTE flash_write_byte (BYTE b) {
                           3165 ;	-----------------------------------------
                           3166 ;	 function flash_write_byte
                           3167 ;	-----------------------------------------
   0A3E                    3168 _flash_write_byte:
   0A3E AA 82              3169 	mov	r2,dpl
                           3170 ;	../../include/ztex-flash2.h:458: if ( spi_need_pp && spi_pp() ) return flash_ec;
   0A40 90 3A 1B           3171 	mov	dptr,#_spi_need_pp
   0A43 E0                 3172 	movx	a,@dptr
   0A44 FB                 3173 	mov	r3,a
   0A45 60 12              3174 	jz	00102$
   0A47 C0 02              3175 	push	ar2
   0A49 12 09 E6           3176 	lcall	_spi_pp
   0A4C E5 82              3177 	mov	a,dpl
   0A4E D0 02              3178 	pop	ar2
   0A50 60 07              3179 	jz	00102$
   0A52 90 3A 0E           3180 	mov	dptr,#_flash_ec
   0A55 E0                 3181 	movx	a,@dptr
   0A56 F5 82              3182 	mov	dpl,a
   0A58 22                 3183 	ret
   0A59                    3184 00102$:
                           3185 ;	../../include/ztex-flash2.h:459: spi_write_byte(b);
   0A59 8A 82              3186 	mov	dpl,r2
   0A5B 12 08 67           3187 	lcall	_spi_write_byte
                           3188 ;	../../include/ztex-flash2.h:460: spi_write_addr_lo++;
   0A5E 90 3A 1A           3189 	mov	dptr,#_spi_write_addr_lo
   0A61 E0                 3190 	movx	a,@dptr
   0A62 90 3A 1A           3191 	mov	dptr,#_spi_write_addr_lo
   0A65 04                 3192 	inc	a
   0A66 F0                 3193 	movx	@dptr,a
                           3194 ;	../../include/ztex-flash2.h:461: if ( spi_write_addr_lo == 0 ) {
   0A67 90 3A 1A           3195 	mov	dptr,#_spi_write_addr_lo
   0A6A E0                 3196 	movx	a,@dptr
   0A6B FA                 3197 	mov	r2,a
   0A6C 70 1C              3198 	jnz	00105$
                           3199 ;	../../include/ztex-flash2.h:462: spi_write_addr_hi++;
   0A6E 90 3A 18           3200 	mov	dptr,#_spi_write_addr_hi
   0A71 E0                 3201 	movx	a,@dptr
   0A72 FA                 3202 	mov	r2,a
   0A73 A3                 3203 	inc	dptr
   0A74 E0                 3204 	movx	a,@dptr
   0A75 FB                 3205 	mov	r3,a
   0A76 90 3A 18           3206 	mov	dptr,#_spi_write_addr_hi
   0A79 74 01              3207 	mov	a,#0x01
   0A7B 2A                 3208 	add	a,r2
   0A7C F0                 3209 	movx	@dptr,a
   0A7D E4                 3210 	clr	a
   0A7E 3B                 3211 	addc	a,r3
   0A7F A3                 3212 	inc	dptr
   0A80 F0                 3213 	movx	@dptr,a
                           3214 ;	../../include/ztex-flash2.h:463: spi_deselect();				// finish write cmd
   0A81 12 08 ED           3215 	lcall	_spi_deselect
                           3216 ;	../../include/ztex-flash2.h:464: spi_need_pp = 1;
   0A84 90 3A 1B           3217 	mov	dptr,#_spi_need_pp
   0A87 74 01              3218 	mov	a,#0x01
   0A89 F0                 3219 	movx	@dptr,a
   0A8A                    3220 00105$:
                           3221 ;	../../include/ztex-flash2.h:466: return 0;
   0A8A 75 82 00           3222 	mov	dpl,#0x00
   0A8D 22                 3223 	ret
                           3224 ;------------------------------------------------------------
                           3225 ;Allocation info for local variables in function 'flash_write'
                           3226 ;------------------------------------------------------------
                           3227 ;len                       Allocated with name '_flash_write_PARM_2'
                           3228 ;buf                       Allocated to registers r2 r3 
                           3229 ;b                         Allocated to registers r4 
                           3230 ;------------------------------------------------------------
                           3231 ;	../../include/ztex-flash2.h:474: BYTE flash_write(__xdata BYTE *buf, BYTE len) {
                           3232 ;	-----------------------------------------
                           3233 ;	 function flash_write
                           3234 ;	-----------------------------------------
   0A8E                    3235 _flash_write:
   0A8E AA 82              3236 	mov	r2,dpl
   0A90 AB 83              3237 	mov	r3,dph
                           3238 ;	../../include/ztex-flash2.h:476: if ( spi_need_pp && spi_pp() ) return flash_ec;
   0A92 90 3A 1B           3239 	mov	dptr,#_spi_need_pp
   0A95 E0                 3240 	movx	a,@dptr
   0A96 FC                 3241 	mov	r4,a
   0A97 60 16              3242 	jz	00102$
   0A99 C0 02              3243 	push	ar2
   0A9B C0 03              3244 	push	ar3
   0A9D 12 09 E6           3245 	lcall	_spi_pp
   0AA0 E5 82              3246 	mov	a,dpl
   0AA2 D0 03              3247 	pop	ar3
   0AA4 D0 02              3248 	pop	ar2
   0AA6 60 07              3249 	jz	00102$
   0AA8 90 3A 0E           3250 	mov	dptr,#_flash_ec
   0AAB E0                 3251 	movx	a,@dptr
   0AAC F5 82              3252 	mov	dpl,a
   0AAE 22                 3253 	ret
   0AAF                    3254 00102$:
                           3255 ;	../../include/ztex-flash2.h:478: if ( spi_write_addr_lo == 0 ) {
   0AAF 90 3A 1A           3256 	mov	dptr,#_spi_write_addr_lo
   0AB2 E0                 3257 	movx	a,@dptr
   0AB3 FC                 3258 	mov	r4,a
   0AB4 70 0C              3259 	jnz	00110$
                           3260 ;	../../include/ztex-flash2.h:479: spi_write(buf,len);
   0AB6 85 14 16           3261 	mov	_spi_write_PARM_2,_flash_write_PARM_2
   0AB9 8A 82              3262 	mov	dpl,r2
   0ABB 8B 83              3263 	mov	dph,r3
   0ABD 12 08 A3           3264 	lcall	_spi_write
   0AC0 80 67              3265 	sjmp	00111$
   0AC2                    3266 00110$:
                           3267 ;	../../include/ztex-flash2.h:482: b = (~spi_write_addr_lo) + 1;
   0AC2 EC                 3268 	mov	a,r4
   0AC3 F4                 3269 	cpl	a
   0AC4 FC                 3270 	mov	r4,a
   0AC5 0C                 3271 	inc	r4
                           3272 ;	../../include/ztex-flash2.h:483: if ( len==0 || len>b ) {
   0AC6 E5 14              3273 	mov	a,_flash_write_PARM_2
   0AC8 60 06              3274 	jz	00106$
   0ACA C3                 3275 	clr	c
   0ACB EC                 3276 	mov	a,r4
   0ACC 95 14              3277 	subb	a,_flash_write_PARM_2
   0ACE 50 4F              3278 	jnc	00107$
   0AD0                    3279 00106$:
                           3280 ;	../../include/ztex-flash2.h:484: spi_write(buf,b);
   0AD0 8C 16              3281 	mov	_spi_write_PARM_2,r4
   0AD2 8A 82              3282 	mov	dpl,r2
   0AD4 8B 83              3283 	mov	dph,r3
   0AD6 C0 02              3284 	push	ar2
   0AD8 C0 03              3285 	push	ar3
   0ADA C0 04              3286 	push	ar4
   0ADC 12 08 A3           3287 	lcall	_spi_write
   0ADF D0 04              3288 	pop	ar4
   0AE1 D0 03              3289 	pop	ar3
   0AE3 D0 02              3290 	pop	ar2
                           3291 ;	../../include/ztex-flash2.h:485: len-=b;
   0AE5 E5 14              3292 	mov	a,_flash_write_PARM_2
   0AE7 C3                 3293 	clr	c
   0AE8 9C                 3294 	subb	a,r4
   0AE9 F5 14              3295 	mov	_flash_write_PARM_2,a
                           3296 ;	../../include/ztex-flash2.h:486: spi_write_addr_hi++;
   0AEB 90 3A 18           3297 	mov	dptr,#_spi_write_addr_hi
   0AEE E0                 3298 	movx	a,@dptr
   0AEF FD                 3299 	mov	r5,a
   0AF0 A3                 3300 	inc	dptr
   0AF1 E0                 3301 	movx	a,@dptr
   0AF2 FE                 3302 	mov	r6,a
   0AF3 90 3A 18           3303 	mov	dptr,#_spi_write_addr_hi
   0AF6 74 01              3304 	mov	a,#0x01
   0AF8 2D                 3305 	add	a,r5
   0AF9 F0                 3306 	movx	@dptr,a
   0AFA E4                 3307 	clr	a
   0AFB 3E                 3308 	addc	a,r6
   0AFC A3                 3309 	inc	dptr
   0AFD F0                 3310 	movx	@dptr,a
                           3311 ;	../../include/ztex-flash2.h:487: spi_write_addr_lo=0;
   0AFE 90 3A 1A           3312 	mov	dptr,#_spi_write_addr_lo
   0B01 E4                 3313 	clr	a
   0B02 F0                 3314 	movx	@dptr,a
                           3315 ;	../../include/ztex-flash2.h:488: buf+=b;
   0B03 EC                 3316 	mov	a,r4
   0B04 2A                 3317 	add	a,r2
   0B05 FA                 3318 	mov	r2,a
   0B06 E4                 3319 	clr	a
   0B07 3B                 3320 	addc	a,r3
   0B08 FB                 3321 	mov	r3,a
                           3322 ;	../../include/ztex-flash2.h:489: if ( spi_pp() ) return flash_ec;
   0B09 C0 02              3323 	push	ar2
   0B0B C0 03              3324 	push	ar3
   0B0D 12 09 E6           3325 	lcall	_spi_pp
   0B10 E5 82              3326 	mov	a,dpl
   0B12 D0 03              3327 	pop	ar3
   0B14 D0 02              3328 	pop	ar2
   0B16 60 07              3329 	jz	00107$
   0B18 90 3A 0E           3330 	mov	dptr,#_flash_ec
   0B1B E0                 3331 	movx	a,@dptr
   0B1C F5 82              3332 	mov	dpl,a
   0B1E 22                 3333 	ret
   0B1F                    3334 00107$:
                           3335 ;	../../include/ztex-flash2.h:491: spi_write(buf,len);
   0B1F 85 14 16           3336 	mov	_spi_write_PARM_2,_flash_write_PARM_2
   0B22 8A 82              3337 	mov	dpl,r2
   0B24 8B 83              3338 	mov	dph,r3
   0B26 12 08 A3           3339 	lcall	_spi_write
   0B29                    3340 00111$:
                           3341 ;	../../include/ztex-flash2.h:494: spi_write_addr_lo+=len;
   0B29 90 3A 1A           3342 	mov	dptr,#_spi_write_addr_lo
   0B2C E0                 3343 	movx	a,@dptr
   0B2D FA                 3344 	mov	r2,a
   0B2E E5 14              3345 	mov	a,_flash_write_PARM_2
   0B30 2A                 3346 	add	a,r2
   0B31 F0                 3347 	movx	@dptr,a
                           3348 ;	../../include/ztex-flash2.h:496: if ( spi_write_addr_lo == 0 ) {
   0B32 90 3A 1A           3349 	mov	dptr,#_spi_write_addr_lo
   0B35 E0                 3350 	movx	a,@dptr
   0B36 FA                 3351 	mov	r2,a
   0B37 70 1C              3352 	jnz	00113$
                           3353 ;	../../include/ztex-flash2.h:497: spi_write_addr_hi++;
   0B39 90 3A 18           3354 	mov	dptr,#_spi_write_addr_hi
   0B3C E0                 3355 	movx	a,@dptr
   0B3D FA                 3356 	mov	r2,a
   0B3E A3                 3357 	inc	dptr
   0B3F E0                 3358 	movx	a,@dptr
   0B40 FB                 3359 	mov	r3,a
   0B41 90 3A 18           3360 	mov	dptr,#_spi_write_addr_hi
   0B44 74 01              3361 	mov	a,#0x01
   0B46 2A                 3362 	add	a,r2
   0B47 F0                 3363 	movx	@dptr,a
   0B48 E4                 3364 	clr	a
   0B49 3B                 3365 	addc	a,r3
   0B4A A3                 3366 	inc	dptr
   0B4B F0                 3367 	movx	@dptr,a
                           3368 ;	../../include/ztex-flash2.h:498: spi_deselect();				// finish write cmd
   0B4C 12 08 ED           3369 	lcall	_spi_deselect
                           3370 ;	../../include/ztex-flash2.h:499: spi_need_pp = 1;
   0B4F 90 3A 1B           3371 	mov	dptr,#_spi_need_pp
   0B52 74 01              3372 	mov	a,#0x01
   0B54 F0                 3373 	movx	@dptr,a
   0B55                    3374 00113$:
                           3375 ;	../../include/ztex-flash2.h:502: return 0;
   0B55 75 82 00           3376 	mov	dpl,#0x00
   0B58 22                 3377 	ret
                           3378 ;------------------------------------------------------------
                           3379 ;Allocation info for local variables in function 'flash_write_init'
                           3380 ;------------------------------------------------------------
                           3381 ;s                         Allocated to registers r2 r3 
                           3382 ;------------------------------------------------------------
                           3383 ;	../../include/ztex-flash2.h:514: BYTE flash_write_init(WORD s) {
                           3384 ;	-----------------------------------------
                           3385 ;	 function flash_write_init
                           3386 ;	-----------------------------------------
   0B59                    3387 _flash_write_init:
   0B59 AA 82              3388 	mov	r2,dpl
   0B5B AB 83              3389 	mov	r3,dph
                           3390 ;	../../include/ztex-flash2.h:515: if ( !SPI_CS ) {
   0B5D 20 A5 0A           3391 	jb	_IOC5,00102$
                           3392 ;	../../include/ztex-flash2.h:516: flash_ec = FLASH_EC_PENDING;
   0B60 90 3A 0E           3393 	mov	dptr,#_flash_ec
   0B63 74 04              3394 	mov	a,#0x04
   0B65 F0                 3395 	movx	@dptr,a
                           3396 ;	../../include/ztex-flash2.h:517: return FLASH_EC_PENDING;		// we interrupted a pending Flash operation
   0B66 75 82 04           3397 	mov	dpl,#0x04
   0B69 22                 3398 	ret
   0B6A                    3399 00102$:
                           3400 ;	../../include/ztex-flash2.h:519: if ( spi_wait() ) {
   0B6A C0 02              3401 	push	ar2
   0B6C C0 03              3402 	push	ar3
   0B6E 12 08 F5           3403 	lcall	_spi_wait
   0B71 E5 82              3404 	mov	a,dpl
   0B73 D0 03              3405 	pop	ar3
   0B75 D0 02              3406 	pop	ar2
   0B77 60 07              3407 	jz	00104$
                           3408 ;	../../include/ztex-flash2.h:520: return flash_ec;
   0B79 90 3A 0E           3409 	mov	dptr,#_flash_ec
   0B7C E0                 3410 	movx	a,@dptr
   0B7D F5 82              3411 	mov	dpl,a
   0B7F 22                 3412 	ret
   0B80                    3413 00104$:
                           3414 ;	../../include/ztex-flash2.h:522: spi_write_sector = s;
   0B80 90 3A 1C           3415 	mov	dptr,#_spi_write_sector
   0B83 EA                 3416 	mov	a,r2
   0B84 F0                 3417 	movx	@dptr,a
   0B85 A3                 3418 	inc	dptr
   0B86 EB                 3419 	mov	a,r3
   0B87 F0                 3420 	movx	@dptr,a
                           3421 ;	../../include/ztex-flash2.h:523: s = s << ((BYTE)flash_sector_size - 8);     
   0B88 90 3A 08           3422 	mov	dptr,#_flash_sector_size
   0B8B E0                 3423 	movx	a,@dptr
   0B8C FC                 3424 	mov	r4,a
   0B8D A3                 3425 	inc	dptr
   0B8E E0                 3426 	movx	a,@dptr
   0B8F 7D 00              3427 	mov	r5,#0x00
   0B91 EC                 3428 	mov	a,r4
   0B92 24 F8              3429 	add	a,#0xf8
   0B94 FC                 3430 	mov	r4,a
   0B95 ED                 3431 	mov	a,r5
   0B96 34 FF              3432 	addc	a,#0xff
   0B98 FD                 3433 	mov	r5,a
   0B99 8C F0              3434 	mov	b,r4
   0B9B 05 F0              3435 	inc	b
   0B9D 80 06              3436 	sjmp	00112$
   0B9F                    3437 00111$:
   0B9F EA                 3438 	mov	a,r2
   0BA0 2A                 3439 	add	a,r2
   0BA1 FA                 3440 	mov	r2,a
   0BA2 EB                 3441 	mov	a,r3
   0BA3 33                 3442 	rlc	a
   0BA4 FB                 3443 	mov	r3,a
   0BA5                    3444 00112$:
   0BA5 D5 F0 F7           3445 	djnz	b,00111$
                           3446 ;	../../include/ztex-flash2.h:524: spi_write_addr_hi = s;
   0BA8 90 3A 18           3447 	mov	dptr,#_spi_write_addr_hi
   0BAB EA                 3448 	mov	a,r2
   0BAC F0                 3449 	movx	@dptr,a
   0BAD A3                 3450 	inc	dptr
   0BAE EB                 3451 	mov	a,r3
   0BAF F0                 3452 	movx	@dptr,a
                           3453 ;	../../include/ztex-flash2.h:525: spi_write_addr_lo = 0;
   0BB0 90 3A 1A           3454 	mov	dptr,#_spi_write_addr_lo
   0BB3 E4                 3455 	clr	a
   0BB4 F0                 3456 	movx	@dptr,a
                           3457 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   0BB5 90 3A 13           3458 	mov	dptr,#_spi_last_cmd
   0BB8 74 06              3459 	mov	a,#0x06
   0BBA F0                 3460 	movx	@dptr,a
                           3461 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0BBB C0 02              3462 	push	ar2
   0BBD C0 03              3463 	push	ar3
   0BBF 12 08 E2           3464 	lcall	_spi_select
                           3465 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0BC2 75 82 06           3466 	mov	dpl,#0x06
   0BC5 12 08 67           3467 	lcall	_spi_write_byte
                           3468 ;	../../include/ztex-flash2.h:528: spi_deselect();
   0BC8 12 08 ED           3469 	lcall	_spi_deselect
                           3470 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   0BCB 90 3A 12           3471 	mov	dptr,#_spi_erase_cmd
   0BCE E0                 3472 	movx	a,@dptr
   0BCF 90 3A 13           3473 	mov	dptr,#_spi_last_cmd
   0BD2 F0                 3474 	movx	@dptr,a
                           3475 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0BD3 12 08 E2           3476 	lcall	_spi_select
                           3477 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0BD6 90 3A 12           3478 	mov	dptr,#_spi_erase_cmd
   0BD9 E0                 3479 	movx	a,@dptr
   0BDA F5 82              3480 	mov	dpl,a
   0BDC 12 08 67           3481 	lcall	_spi_write_byte
   0BDF D0 03              3482 	pop	ar3
                           3483 ;	../../include/ztex-flash2.h:363: 
   0BE1 8B 82              3484 	mov	dpl,r3
   0BE3 C0 03              3485 	push	ar3
   0BE5 12 08 67           3486 	lcall	_spi_write_byte
   0BE8 D0 03              3487 	pop	ar3
   0BEA D0 02              3488 	pop	ar2
                           3489 ;	../../include/ztex-flash2.h:532: spi_write_byte(s & 255);
   0BEC 8A 82              3490 	mov	dpl,r2
   0BEE 12 08 67           3491 	lcall	_spi_write_byte
                           3492 ;	../../include/ztex-flash2.h:533: spi_write_byte(0);
   0BF1 75 82 00           3493 	mov	dpl,#0x00
   0BF4 12 08 67           3494 	lcall	_spi_write_byte
                           3495 ;	../../include/ztex-flash2.h:534: spi_deselect();
   0BF7 12 08 ED           3496 	lcall	_spi_deselect
                           3497 ;	../../include/ztex-flash2.h:536: spi_need_pp = 1;
   0BFA 90 3A 1B           3498 	mov	dptr,#_spi_need_pp
   0BFD 74 01              3499 	mov	a,#0x01
   0BFF F0                 3500 	movx	@dptr,a
                           3501 ;	../../include/ztex-flash2.h:537: return 0;
   0C00 75 82 00           3502 	mov	dpl,#0x00
   0C03 22                 3503 	ret
                           3504 ;------------------------------------------------------------
                           3505 ;Allocation info for local variables in function 'flash_write_finish_sector'
                           3506 ;------------------------------------------------------------
                           3507 ;n                         Allocated to registers 
                           3508 ;------------------------------------------------------------
                           3509 ;	../../include/ztex-flash2.h:547: BYTE flash_write_finish_sector (WORD n) {
                           3510 ;	-----------------------------------------
                           3511 ;	 function flash_write_finish_sector
                           3512 ;	-----------------------------------------
   0C04                    3513 _flash_write_finish_sector:
                           3514 ;	../../include/ztex-flash2.h:549: spi_deselect();
   0C04 12 08 ED           3515 	lcall	_spi_deselect
                           3516 ;	../../include/ztex-flash2.h:550: return 0;
   0C07 75 82 00           3517 	mov	dpl,#0x00
   0C0A 22                 3518 	ret
                           3519 ;------------------------------------------------------------
                           3520 ;Allocation info for local variables in function 'flash_write_finish'
                           3521 ;------------------------------------------------------------
                           3522 ;------------------------------------------------------------
                           3523 ;	../../include/ztex-flash2.h:560: void flash_write_finish () {
                           3524 ;	-----------------------------------------
                           3525 ;	 function flash_write_finish
                           3526 ;	-----------------------------------------
   0C0B                    3527 _flash_write_finish:
                           3528 ;	../../include/ztex-flash2.h:561: spi_deselect();
   0C0B 02 08 ED           3529 	ljmp	_spi_deselect
                           3530 ;------------------------------------------------------------
                           3531 ;Allocation info for local variables in function 'flash_write_next'
                           3532 ;------------------------------------------------------------
                           3533 ;------------------------------------------------------------
                           3534 ;	../../include/ztex-flash2.h:571: BYTE flash_write_next () {
                           3535 ;	-----------------------------------------
                           3536 ;	 function flash_write_next
                           3537 ;	-----------------------------------------
   0C0E                    3538 _flash_write_next:
                           3539 ;	../../include/ztex-flash2.h:572: spi_deselect();
   0C0E 12 08 ED           3540 	lcall	_spi_deselect
                           3541 ;	../../include/ztex-flash2.h:573: return flash_write_init(spi_write_sector+1);
   0C11 90 3A 1C           3542 	mov	dptr,#_spi_write_sector
   0C14 E0                 3543 	movx	a,@dptr
   0C15 FA                 3544 	mov	r2,a
   0C16 A3                 3545 	inc	dptr
   0C17 E0                 3546 	movx	a,@dptr
   0C18 FB                 3547 	mov	r3,a
   0C19 8A 82              3548 	mov	dpl,r2
   0C1B 8B 83              3549 	mov	dph,r3
   0C1D A3                 3550 	inc	dptr
   0C1E 02 0B 59           3551 	ljmp	_flash_write_init
                           3552 ;------------------------------------------------------------
                           3553 ;Allocation info for local variables in function 'flash_init'
                           3554 ;------------------------------------------------------------
                           3555 ;i                         Allocated to registers r2 
                           3556 ;------------------------------------------------------------
                           3557 ;	../../include/ztex-flash2.h:581: void flash_init() {
                           3558 ;	-----------------------------------------
                           3559 ;	 function flash_init
                           3560 ;	-----------------------------------------
   0C21                    3561 _flash_init:
                           3562 ;	../../include/ztex-flash2.h:584: PORTCCFG = 0;
   0C21 90 E6 71           3563 	mov	dptr,#_PORTCCFG
   0C24 E4                 3564 	clr	a
   0C25 F0                 3565 	movx	@dptr,a
                           3566 ;	../../include/ztex-flash2.h:586: flash_enabled = 1;
   0C26 90 3A 07           3567 	mov	dptr,#_flash_enabled
   0C29 74 01              3568 	mov	a,#0x01
   0C2B F0                 3569 	movx	@dptr,a
                           3570 ;	../../include/ztex-flash2.h:587: flash_ec = 0;
   0C2C 90 3A 0E           3571 	mov	dptr,#_flash_ec
   0C2F E4                 3572 	clr	a
   0C30 F0                 3573 	movx	@dptr,a
                           3574 ;	../../include/ztex-flash2.h:588: flash_sector_size = 0x8010;  // 64 KByte
   0C31 90 3A 08           3575 	mov	dptr,#_flash_sector_size
   0C34 74 10              3576 	mov	a,#0x10
   0C36 F0                 3577 	movx	@dptr,a
   0C37 A3                 3578 	inc	dptr
   0C38 74 80              3579 	mov	a,#0x80
   0C3A F0                 3580 	movx	@dptr,a
                           3581 ;	../../include/ztex-flash2.h:589: spi_erase_cmd = 0xd8;
   0C3B 90 3A 12           3582 	mov	dptr,#_spi_erase_cmd
   0C3E 74 D8              3583 	mov	a,#0xD8
   0C40 F0                 3584 	movx	@dptr,a
                           3585 ;	../../include/ztex-flash2.h:591: OESPI_OPORT &= ~bmBITSPI_BIT_DO;
   0C41 53 B4 EF           3586 	anl	_OEC,#0xEF
                           3587 ;	../../include/ztex-flash2.h:592: OESPI_PORT |= bmBITSPI_BIT_CS | bmBITSPI_BIT_DI | bmBITSPI_BIT_CLK;
   0C44 43 B4 E0           3588 	orl	_OEC,#0xE0
                           3589 ;	../../include/ztex-flash2.h:593: SPI_CS = 1;
   0C47 D2 A5              3590 	setb	_IOC5
                           3591 ;	../../include/ztex-flash2.h:594: spi_clocks(0);				// 256 clocks
   0C49 75 82 00           3592 	mov	dpl,#0x00
   0C4C 12 07 DD           3593 	lcall	_spi_clocks
                           3594 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   0C4F 90 3A 13           3595 	mov	dptr,#_spi_last_cmd
   0C52 74 90              3596 	mov	a,#0x90
   0C54 F0                 3597 	movx	@dptr,a
                           3598 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0C55 12 08 E2           3599 	lcall	_spi_select
                           3600 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0C58 75 82 90           3601 	mov	dpl,#0x90
   0C5B 12 08 67           3602 	lcall	_spi_write_byte
                           3603 ;	../../include/ztex-flash2.h:363: 
   0C5E 75 82 18           3604 	mov	dpl,#0x18
   0C61 12 07 DD           3605 	lcall	_spi_clocks
                           3606 ;	../../include/ztex-flash2.h:598: spi_device = flash_read_byte();			
   0C64 12 07 E9           3607 	lcall	_flash_read_byte
   0C67 E5 82              3608 	mov	a,dpl
   0C69 90 3A 10           3609 	mov	dptr,#_spi_device
   0C6C F0                 3610 	movx	@dptr,a
                           3611 ;	../../include/ztex-flash2.h:599: spi_deselect();				// deselect
   0C6D 12 08 ED           3612 	lcall	_spi_deselect
                           3613 ;	../../include/ztex-flash2.h:359: spi_last_cmd = $0;
   0C70 90 3A 13           3614 	mov	dptr,#_spi_last_cmd
   0C73 74 9F              3615 	mov	a,#0x9F
   0C75 F0                 3616 	movx	@dptr,a
                           3617 ;	../../include/ztex-flash2.h:360: spi_select();				// select
   0C76 12 08 E2           3618 	lcall	_spi_select
                           3619 ;	../../include/ztex-flash2.h:361: spi_write_byte($0);				// CMD 90h
   0C79 75 82 9F           3620 	mov	dpl,#0x9F
   0C7C 12 08 67           3621 	lcall	_spi_write_byte
                           3622 ;	../../include/ztex-flash2.h:363: 
   0C7F 75 16 03           3623 	mov	_flash_read_PARM_2,#0x03
   0C82 90 3A 14           3624 	mov	dptr,#_spi_buffer
   0C85 12 08 28           3625 	lcall	_flash_read
                           3626 ;	../../include/ztex-flash2.h:364: /* *********************************************************************
   0C88 12 08 ED           3627 	lcall	_spi_deselect
                           3628 ;	../../include/ztex-flash2.h:604: if ( spi_buffer[2]<16 || spi_buffer[2]>24 ) {
   0C8B 90 3A 16           3629 	mov	dptr,#(_spi_buffer + 0x0002)
   0C8E E0                 3630 	movx	a,@dptr
   0C8F FA                 3631 	mov	r2,a
   0C90 BA 10 00           3632 	cjne	r2,#0x10,00109$
   0C93                    3633 00109$:
   0C93 40 3D              3634 	jc	00104$
   0C95 EA                 3635 	mov	a,r2
   0C96 24 E7              3636 	add	a,#0xff - 0x18
   0C98 40 38              3637 	jc	00104$
                           3638 ;	../../include/ztex-flash2.h:607: spi_vendor = spi_buffer[0];
   0C9A 90 3A 14           3639 	mov	dptr,#_spi_buffer
   0C9D E0                 3640 	movx	a,@dptr
   0C9E 90 3A 0F           3641 	mov	dptr,#_spi_vendor
   0CA1 F0                 3642 	movx	@dptr,a
                           3643 ;	../../include/ztex-flash2.h:608: spi_memtype = spi_buffer[1];
   0CA2 90 3A 15           3644 	mov	dptr,#(_spi_buffer + 0x0001)
   0CA5 E0                 3645 	movx	a,@dptr
   0CA6 90 3A 11           3646 	mov	dptr,#_spi_memtype
   0CA9 F0                 3647 	movx	@dptr,a
                           3648 ;	../../include/ztex-flash2.h:624: i=spi_buffer[2]-16;
   0CAA EA                 3649 	mov	a,r2
   0CAB 24 F0              3650 	add	a,#0xf0
   0CAD FA                 3651 	mov	r2,a
                           3652 ;	../../include/ztex-flash2.h:626: flash_sectors = 1 << i;
   0CAE 8A F0              3653 	mov	b,r2
   0CB0 05 F0              3654 	inc	b
   0CB2 7A 01              3655 	mov	r2,#0x01
   0CB4 7B 00              3656 	mov	r3,#0x00
   0CB6 80 06              3657 	sjmp	00113$
   0CB8                    3658 00112$:
   0CB8 EA                 3659 	mov	a,r2
   0CB9 2A                 3660 	add	a,r2
   0CBA FA                 3661 	mov	r2,a
   0CBB EB                 3662 	mov	a,r3
   0CBC 33                 3663 	rlc	a
   0CBD FB                 3664 	mov	r3,a
   0CBE                    3665 00113$:
   0CBE D5 F0 F7           3666 	djnz	b,00112$
   0CC1 90 3A 0A           3667 	mov	dptr,#_flash_sectors
   0CC4 EA                 3668 	mov	a,r2
   0CC5 F0                 3669 	movx	@dptr,a
   0CC6 A3                 3670 	inc	dptr
   0CC7 EB                 3671 	mov	a,r3
   0CC8 F0                 3672 	movx	@dptr,a
   0CC9 EB                 3673 	mov	a,r3
   0CCA 33                 3674 	rlc	a
   0CCB 95 E0              3675 	subb	a,acc
   0CCD A3                 3676 	inc	dptr
   0CCE F0                 3677 	movx	@dptr,a
   0CCF A3                 3678 	inc	dptr
   0CD0 F0                 3679 	movx	@dptr,a
                           3680 ;	../../include/ztex-flash2.h:628: return;
                           3681 ;	../../include/ztex-flash2.h:630: disable:
   0CD1 22                 3682 	ret
   0CD2                    3683 00104$:
                           3684 ;	../../include/ztex-flash2.h:631: flash_enabled = 0;
   0CD2 90 3A 07           3685 	mov	dptr,#_flash_enabled
   0CD5 E4                 3686 	clr	a
   0CD6 F0                 3687 	movx	@dptr,a
                           3688 ;	../../include/ztex-flash2.h:632: flash_ec = FLASH_EC_NOTSUPPORTED;
   0CD7 90 3A 0E           3689 	mov	dptr,#_flash_ec
   0CDA 74 07              3690 	mov	a,#0x07
   0CDC F0                 3691 	movx	@dptr,a
                           3692 ;	../../include/ztex-flash2.h:633: OESPI_PORT &= ~( bmBITSPI_BIT_CS | bmBITSPI_BIT_DI | bmBITSPI_BIT_CLK );
   0CDD 53 B4 1F           3693 	anl	_OEC,#0x1F
   0CE0 22                 3694 	ret
                           3695 ;------------------------------------------------------------
                           3696 ;Allocation info for local variables in function 'spi_read_ep0'
                           3697 ;------------------------------------------------------------
                           3698 ;------------------------------------------------------------
                           3699 ;	../../include/ztex-flash2.h:659: void spi_read_ep0 () { 
                           3700 ;	-----------------------------------------
                           3701 ;	 function spi_read_ep0
                           3702 ;	-----------------------------------------
   0CE1                    3703 _spi_read_ep0:
                           3704 ;	../../include/ztex-flash2.h:660: flash_read(EP0BUF, ep0_payload_transfer);
   0CE1 90 3A 38           3705 	mov	dptr,#_ep0_payload_transfer
   0CE4 E0                 3706 	movx	a,@dptr
   0CE5 F5 16              3707 	mov	_flash_read_PARM_2,a
   0CE7 90 E7 40           3708 	mov	dptr,#_EP0BUF
   0CEA 12 08 28           3709 	lcall	_flash_read
                           3710 ;	../../include/ztex-flash2.h:661: if ( ep0_read_mode==2 && ep0_payload_remaining==0 ) {
   0CED 90 3A 1E           3711 	mov	dptr,#_ep0_read_mode
   0CF0 E0                 3712 	movx	a,@dptr
   0CF1 FA                 3713 	mov	r2,a
   0CF2 BA 02 0E           3714 	cjne	r2,#0x02,00104$
   0CF5 90 3A 36           3715 	mov	dptr,#_ep0_payload_remaining
   0CF8 E0                 3716 	movx	a,@dptr
   0CF9 FA                 3717 	mov	r2,a
   0CFA A3                 3718 	inc	dptr
   0CFB E0                 3719 	movx	a,@dptr
   0CFC FB                 3720 	mov	r3,a
   0CFD 4A                 3721 	orl	a,r2
   0CFE 70 03              3722 	jnz	00104$
                           3723 ;	../../include/ztex-flash2.h:662: spi_deselect();
   0D00 02 08 ED           3724 	ljmp	_spi_deselect
   0D03                    3725 00104$:
   0D03 22                 3726 	ret
                           3727 ;------------------------------------------------------------
                           3728 ;Allocation info for local variables in function 'spi_send_ep0'
                           3729 ;------------------------------------------------------------
                           3730 ;------------------------------------------------------------
                           3731 ;	../../include/ztex-flash2.h:686: void spi_send_ep0 () { 
                           3732 ;	-----------------------------------------
                           3733 ;	 function spi_send_ep0
                           3734 ;	-----------------------------------------
   0D04                    3735 _spi_send_ep0:
                           3736 ;	../../include/ztex-flash2.h:687: flash_write(EP0BUF, ep0_payload_transfer);
   0D04 90 3A 38           3737 	mov	dptr,#_ep0_payload_transfer
   0D07 E0                 3738 	movx	a,@dptr
   0D08 F5 14              3739 	mov	_flash_write_PARM_2,a
   0D0A 90 E7 40           3740 	mov	dptr,#_EP0BUF
   0D0D 12 0A 8E           3741 	lcall	_flash_write
                           3742 ;	../../include/ztex-flash2.h:688: if ( ep0_write_mode==2 && ep0_payload_remaining==0 ) {
   0D10 90 3A 1F           3743 	mov	dptr,#_ep0_write_mode
   0D13 E0                 3744 	movx	a,@dptr
   0D14 FA                 3745 	mov	r2,a
   0D15 BA 02 0E           3746 	cjne	r2,#0x02,00104$
   0D18 90 3A 36           3747 	mov	dptr,#_ep0_payload_remaining
   0D1B E0                 3748 	movx	a,@dptr
   0D1C FA                 3749 	mov	r2,a
   0D1D A3                 3750 	inc	dptr
   0D1E E0                 3751 	movx	a,@dptr
   0D1F FB                 3752 	mov	r3,a
   0D20 4A                 3753 	orl	a,r2
   0D21 70 03              3754 	jnz	00104$
                           3755 ;	../../include/ztex-flash2.h:689: spi_deselect();
   0D23 02 08 ED           3756 	ljmp	_spi_deselect
   0D26                    3757 00104$:
   0D26 22                 3758 	ret
                           3759 ;------------------------------------------------------------
                           3760 ;Allocation info for local variables in function 'reset_fpga'
                           3761 ;------------------------------------------------------------
                           3762 ;------------------------------------------------------------
                           3763 ;	../../include/ztex-fpga6.h:39: static void reset_fpga () {
                           3764 ;	-----------------------------------------
                           3765 ;	 function reset_fpga
                           3766 ;	-----------------------------------------
   0D27                    3767 _reset_fpga:
                           3768 ;	../../include/ztex-fpga6.h:40: OEE = bmBIT7;
   0D27 75 B6 80           3769 	mov	_OEE,#0x80
                           3770 ;	../../include/ztex-fpga6.h:41: IOE = 0;
   0D2A 75 B1 00           3771 	mov	_IOE,#0x00
                           3772 ;	../../include/ztex-fpga6.h:42: wait(1);
   0D2D 90 00 01           3773 	mov	dptr,#0x0001
   0D30 12 02 65           3774 	lcall	_wait
                           3775 ;	../../include/ztex-fpga6.h:43: OEE = 0;
   0D33 75 B6 00           3776 	mov	_OEE,#0x00
                           3777 ;	../../include/ztex-fpga6.h:44: fpga_conf_initialized = 0;
   0D36 90 3A 27           3778 	mov	dptr,#_fpga_conf_initialized
   0D39 E4                 3779 	clr	a
   0D3A F0                 3780 	movx	@dptr,a
   0D3B 22                 3781 	ret
                           3782 ;------------------------------------------------------------
                           3783 ;Allocation info for local variables in function 'init_fpga'
                           3784 ;------------------------------------------------------------
                           3785 ;------------------------------------------------------------
                           3786 ;	../../include/ztex-fpga6.h:50: static void init_fpga () {
                           3787 ;	-----------------------------------------
                           3788 ;	 function init_fpga
                           3789 ;	-----------------------------------------
   0D3C                    3790 _init_fpga:
                           3791 ;	../../include/ztex-fpga6.h:51: if ( (IOE & bmBIT0) == 0 ) {
   0D3C E5 B1              3792 	mov	a,_IOE
   0D3E 20 E0 0C           3793 	jb	acc.0,00102$
                           3794 ;	../../include/ztex-fpga6.h:53: OEE = bmBIT7;
   0D41 75 B6 80           3795 	mov	_OEE,#0x80
                           3796 ;	../../include/ztex-fpga6.h:54: IOE = 0;
   0D44 75 B1 00           3797 	mov	_IOE,#0x00
                           3798 ;	../../include/ztex-fpga6.h:55: wait(1);
   0D47 90 00 01           3799 	mov	dptr,#0x0001
   0D4A 12 02 65           3800 	lcall	_wait
   0D4D                    3801 00102$:
                           3802 ;	../../include/ztex-fpga6.h:57: OEE = 0;
   0D4D 75 B6 00           3803 	mov	_OEE,#0x00
                           3804 ;	../../include/ztex-fpga6.h:58: fpga_conf_initialized = 0;
   0D50 90 3A 27           3805 	mov	dptr,#_fpga_conf_initialized
   0D53 E4                 3806 	clr	a
   0D54 F0                 3807 	movx	@dptr,a
   0D55 22                 3808 	ret
                           3809 ;------------------------------------------------------------
                           3810 ;Allocation info for local variables in function 'init_fpga_configuration'
                           3811 ;------------------------------------------------------------
                           3812 ;k                         Allocated to registers r2 r3 
                           3813 ;------------------------------------------------------------
                           3814 ;	../../include/ztex-fpga6.h:64: static void init_fpga_configuration () {
                           3815 ;	-----------------------------------------
                           3816 ;	 function init_fpga_configuration
                           3817 ;	-----------------------------------------
   0D56                    3818 _init_fpga_configuration:
                           3819 ;	../../include/ztex-fpga6.h:69: IFCONFIG = bmBIT7;
   0D56 90 E6 01           3820 	mov	dptr,#_IFCONFIG
   0D59 74 80              3821 	mov	a,#0x80
   0D5B F0                 3822 	movx	@dptr,a
                           3823 ;	../../include/ezregs.h:46: __endasm;
                           3824 	
   0D5C 00                 3825 	 nop
   0D5D 00                 3826 	 nop
   0D5E 00                 3827 	 nop
   0D5F 00                 3828 	 nop
                           3829 	    
                           3830 ;	../../include/ztex-fpga6.h:71: PORTCCFG = 0;
   0D60 90 E6 71           3831 	mov	dptr,#_PORTCCFG
                           3832 ;	../../include/ztex-fpga6.h:72: PORTECFG = 0;
   0D63 E4                 3833 	clr	a
   0D64 F0                 3834 	movx	@dptr,a
   0D65 90 E6 72           3835 	mov	dptr,#_PORTECFG
   0D68 F0                 3836 	movx	@dptr,a
                           3837 ;	../../include/ztex-fpga6.h:74: OOEC = OEC;
   0D69 90 3A 28           3838 	mov	dptr,#_OOEC
   0D6C E5 B4              3839 	mov	a,_OEC
   0D6E F0                 3840 	movx	@dptr,a
                           3841 ;	../../include/ztex-fpga6.h:75: fpga_conf_initialized = 123;
   0D6F 90 3A 27           3842 	mov	dptr,#_fpga_conf_initialized
   0D72 74 7B              3843 	mov	a,#0x7B
   0D74 F0                 3844 	movx	@dptr,a
                           3845 ;	../../include/ztex-fpga6.h:77: OEC &= ~( bmBIT7 | bmBIT4);		// in: MOSI, MISO
   0D75 53 B4 6F           3846 	anl	_OEC,#0x6F
                           3847 ;	../../include/ztex-fpga6.h:78: OEC |= bmBIT6;               	// out: CCLK
   0D78 43 B4 40           3848 	orl	_OEC,#0x40
                           3849 ;	../../include/ztex-fpga6.h:79: IOC6 = 1;
   0D7B D2 A6              3850 	setb	_IOC6
                           3851 ;	../../include/ztex-fpga6.h:83: OEE = bmBIT3 | bmBIT4 | bmBIT7 | bmBIT2 | bmBIT5;
   0D7D 75 B6 BC           3852 	mov	_OEE,#0xBC
                           3853 ;	../../include/ztex-fpga6.h:84: IOE = bmBIT3;
   0D80 75 B1 08           3854 	mov	_IOE,#0x08
                           3855 ;	../../include/ztex-fpga6.h:86: wait(2);
   0D83 90 00 02           3856 	mov	dptr,#0x0002
   0D86 12 02 65           3857 	lcall	_wait
                           3858 ;	../../include/ztex-fpga6.h:87: IOE = bmBIT3 | bmBIT7;		// ready for configuration
   0D89 75 B1 88           3859 	mov	_IOE,#0x88
                           3860 ;	../../include/ztex-fpga6.h:88: IOC6 = 0;
   0D8C C2 A6              3861 	clr	_IOC6
                           3862 ;	../../include/ztex-fpga6.h:91: while (!(IOE & bmBIT1) && k<65535)
   0D8E 7A 00              3863 	mov	r2,#0x00
   0D90 7B 00              3864 	mov	r3,#0x00
   0D92                    3865 00102$:
   0D92 E5 B1              3866 	mov	a,_IOE
   0D94 20 E1 20           3867 	jb	acc.1,00104$
   0D97 8A 04              3868 	mov	ar4,r2
   0D99 8B 05              3869 	mov	ar5,r3
   0D9B 7E 00              3870 	mov	r6,#0x00
   0D9D 7F 00              3871 	mov	r7,#0x00
   0D9F C3                 3872 	clr	c
   0DA0 EC                 3873 	mov	a,r4
   0DA1 94 FF              3874 	subb	a,#0xFF
   0DA3 ED                 3875 	mov	a,r5
   0DA4 94 FF              3876 	subb	a,#0xFF
   0DA6 EE                 3877 	mov	a,r6
   0DA7 94 00              3878 	subb	a,#0x00
   0DA9 EF                 3879 	mov	a,r7
   0DAA 64 80              3880 	xrl	a,#0x80
   0DAC 94 80              3881 	subb	a,#0x80
   0DAE 50 07              3882 	jnc	00104$
                           3883 ;	../../include/ztex-fpga6.h:92: k++;
   0DB0 0A                 3884 	inc	r2
   0DB1 BA 00 DE           3885 	cjne	r2,#0x00,00102$
   0DB4 0B                 3886 	inc	r3
   0DB5 80 DB              3887 	sjmp	00102$
   0DB7                    3888 00104$:
                           3889 ;	../../include/ztex-fpga6.h:94: fpga_init_b = (IOE & bmBIT1) ? 200 : 100;
   0DB7 E5 B1              3890 	mov	a,_IOE
   0DB9 30 E1 04           3891 	jnb	acc.1,00107$
   0DBC 7A C8              3892 	mov	r2,#0xC8
   0DBE 80 02              3893 	sjmp	00108$
   0DC0                    3894 00107$:
   0DC0 7A 64              3895 	mov	r2,#0x64
   0DC2                    3896 00108$:
   0DC2 90 3A 25           3897 	mov	dptr,#_fpga_init_b
   0DC5 EA                 3898 	mov	a,r2
   0DC6 F0                 3899 	movx	@dptr,a
                           3900 ;	../../include/ztex-fpga6.h:95: fpga_bytes = 0;
   0DC7 90 3A 21           3901 	mov	dptr,#_fpga_bytes
   0DCA E4                 3902 	clr	a
   0DCB F0                 3903 	movx	@dptr,a
   0DCC A3                 3904 	inc	dptr
   0DCD F0                 3905 	movx	@dptr,a
   0DCE A3                 3906 	inc	dptr
   0DCF F0                 3907 	movx	@dptr,a
   0DD0 A3                 3908 	inc	dptr
   0DD1 F0                 3909 	movx	@dptr,a
                           3910 ;	../../include/ztex-fpga6.h:96: fpga_checksum = 0;
   0DD2 90 3A 20           3911 	mov	dptr,#_fpga_checksum
   0DD5 E4                 3912 	clr	a
   0DD6 F0                 3913 	movx	@dptr,a
   0DD7 22                 3914 	ret
                           3915 ;------------------------------------------------------------
                           3916 ;Allocation info for local variables in function 'post_fpga_config'
                           3917 ;------------------------------------------------------------
                           3918 ;------------------------------------------------------------
                           3919 ;	../../include/ztex-fpga6.h:102: static void post_fpga_config () {
                           3920 ;	-----------------------------------------
                           3921 ;	 function post_fpga_config
                           3922 ;	-----------------------------------------
   0DD8                    3923 _post_fpga_config:
                           3924 ;	../../include/ztex-fpga6.h:104: }
   0DD8 22                 3925 	ret
                           3926 ;------------------------------------------------------------
                           3927 ;Allocation info for local variables in function 'finish_fpga_configuration'
                           3928 ;------------------------------------------------------------
                           3929 ;w                         Allocated to registers r2 
                           3930 ;------------------------------------------------------------
                           3931 ;	../../include/ztex-fpga6.h:109: static void finish_fpga_configuration () {
                           3932 ;	-----------------------------------------
                           3933 ;	 function finish_fpga_configuration
                           3934 ;	-----------------------------------------
   0DD9                    3935 _finish_fpga_configuration:
                           3936 ;	../../include/ztex-fpga6.h:111: fpga_init_b += (IOE & bmBIT1) ? 22 : 11;
   0DD9 E5 B1              3937 	mov	a,_IOE
   0DDB 30 E1 04           3938 	jnb	acc.1,00109$
   0DDE 7A 16              3939 	mov	r2,#0x16
   0DE0 80 02              3940 	sjmp	00110$
   0DE2                    3941 00109$:
   0DE2 7A 0B              3942 	mov	r2,#0x0B
   0DE4                    3943 00110$:
   0DE4 90 3A 25           3944 	mov	dptr,#_fpga_init_b
   0DE7 E0                 3945 	movx	a,@dptr
   0DE8 FB                 3946 	mov	r3,a
   0DE9 EA                 3947 	mov	a,r2
   0DEA 2B                 3948 	add	a,r3
   0DEB F0                 3949 	movx	@dptr,a
                           3950 ;	../../include/ztex-fpga6.h:113: for ( w=0; w<64; w++ ) {
   0DEC 7A 00              3951 	mov	r2,#0x00
   0DEE                    3952 00103$:
   0DEE BA 40 00           3953 	cjne	r2,#0x40,00117$
   0DF1                    3954 00117$:
   0DF1 50 07              3955 	jnc	00106$
                           3956 ;	../../include/ztex-fpga6.h:114: IOC6 = 1; IOC6 = 0; 
   0DF3 D2 A6              3957 	setb	_IOC6
   0DF5 C2 A6              3958 	clr	_IOC6
                           3959 ;	../../include/ztex-fpga6.h:113: for ( w=0; w<64; w++ ) {
   0DF7 0A                 3960 	inc	r2
   0DF8 80 F4              3961 	sjmp	00103$
   0DFA                    3962 00106$:
                           3963 ;	../../include/ztex-fpga6.h:116: IOE |= bmBIT2;		// CSI = 1
   0DFA 43 B1 04           3964 	orl	_IOE,#0x04
                           3965 ;	../../include/ztex-fpga6.h:117: IOC6 = 1; IOC6 = 0;
   0DFD D2 A6              3966 	setb	_IOC6
   0DFF C2 A6              3967 	clr	_IOC6
                           3968 ;	../../include/ztex-fpga6.h:118: IOC6 = 1; IOC6 = 0;
   0E01 D2 A6              3969 	setb	_IOC6
   0E03 C2 A6              3970 	clr	_IOC6
                           3971 ;	../../include/ztex-fpga6.h:119: IOC6 = 1; IOC6 = 0;
   0E05 D2 A6              3972 	setb	_IOC6
   0E07 C2 A6              3973 	clr	_IOC6
                           3974 ;	../../include/ztex-fpga6.h:120: IOC6 = 1; IOC6 = 0;
   0E09 D2 A6              3975 	setb	_IOC6
   0E0B C2 A6              3976 	clr	_IOC6
                           3977 ;	../../include/ztex-fpga6.h:122: OEE = 0;
   0E0D 75 B6 00           3978 	mov	_OEE,#0x00
                           3979 ;	../../include/ztex-fpga6.h:123: OEC = OOEC;
   0E10 90 3A 28           3980 	mov	dptr,#_OOEC
   0E13 E0                 3981 	movx	a,@dptr
   0E14 F5 B4              3982 	mov	_OEC,a
                           3983 ;	../../include/ztex-fpga6.h:124: if ( IOE & bmBIT0 )  {
   0E16 E5 B1              3984 	mov	a,_IOE
   0E18 30 E0 03           3985 	jnb	acc.0,00107$
                           3986 ;	../../include/ztex-fpga6.h:125: post_fpga_config();
   0E1B 02 0D D8           3987 	ljmp	_post_fpga_config
   0E1E                    3988 00107$:
   0E1E 22                 3989 	ret
                           3990 ;------------------------------------------------------------
                           3991 ;Allocation info for local variables in function 'fpga_send_ep0'
                           3992 ;------------------------------------------------------------
                           3993 ;oOEB                      Allocated with name '_fpga_send_ep0_oOEB_1_1'
                           3994 ;------------------------------------------------------------
                           3995 ;	../../include/ztex-fpga6.h:160: void fpga_send_ep0() {			// send FPGA configuration data
                           3996 ;	-----------------------------------------
                           3997 ;	 function fpga_send_ep0
                           3998 ;	-----------------------------------------
   0E1F                    3999 _fpga_send_ep0:
                           4000 ;	../../include/ztex-fpga6.h:162: oOEB = OEB;
   0E1F 85 B3 15           4001 	mov	_fpga_send_ep0_oOEB_1_1,_OEB
                           4002 ;	../../include/ztex-fpga6.h:163: OEB = 255;
   0E22 75 B3 FF           4003 	mov	_OEB,#0xFF
                           4004 ;	../../include/ztex-fpga6.h:164: fpga_bytes += ep0_payload_transfer;
   0E25 90 3A 38           4005 	mov	dptr,#_ep0_payload_transfer
   0E28 E0                 4006 	movx	a,@dptr
   0E29 FB                 4007 	mov	r3,a
   0E2A 90 3A 21           4008 	mov	dptr,#_fpga_bytes
   0E2D E0                 4009 	movx	a,@dptr
   0E2E FC                 4010 	mov	r4,a
   0E2F A3                 4011 	inc	dptr
   0E30 E0                 4012 	movx	a,@dptr
   0E31 FD                 4013 	mov	r5,a
   0E32 A3                 4014 	inc	dptr
   0E33 E0                 4015 	movx	a,@dptr
   0E34 FE                 4016 	mov	r6,a
   0E35 A3                 4017 	inc	dptr
   0E36 E0                 4018 	movx	a,@dptr
   0E37 FF                 4019 	mov	r7,a
   0E38 78 00              4020 	mov	r0,#0x00
   0E3A 79 00              4021 	mov	r1,#0x00
   0E3C 7A 00              4022 	mov	r2,#0x00
   0E3E 90 3A 21           4023 	mov	dptr,#_fpga_bytes
   0E41 EB                 4024 	mov	a,r3
   0E42 2C                 4025 	add	a,r4
   0E43 F0                 4026 	movx	@dptr,a
   0E44 E8                 4027 	mov	a,r0
   0E45 3D                 4028 	addc	a,r5
   0E46 A3                 4029 	inc	dptr
   0E47 F0                 4030 	movx	@dptr,a
   0E48 E9                 4031 	mov	a,r1
   0E49 3E                 4032 	addc	a,r6
   0E4A A3                 4033 	inc	dptr
   0E4B F0                 4034 	movx	@dptr,a
   0E4C EA                 4035 	mov	a,r2
   0E4D 3F                 4036 	addc	a,r7
   0E4E A3                 4037 	inc	dptr
   0E4F F0                 4038 	movx	@dptr,a
                           4039 ;	../../include/ztex-fpga6.h:192: OEB = oOEB;
                           4040 	
   0E50 90 E6 8B           4041 	 mov dptr,#_EP0BCL
   0E53 E0                 4042 	 movx a,@dptr
   0E54 60 22              4043 	 jz 010000$
   0E56 FA                 4044 	   mov r2,a
   0E57 75 9B 40           4045 	 mov _AUTOPTRL1,#(_EP0BUF)
   0E5A 75 9A E7           4046 	 mov _AUTOPTRH1,#(_EP0BUF >> 8)
   0E5D 75 AF 07           4047 	 mov _AUTOPTRSETUP,#0x07
   0E60 90 3A 20           4048 	 mov dptr,#_fpga_checksum
   0E63 E0                 4049 	 movx a,@dptr
   0E64 F9                 4050 	 mov r1,a
   0E65 90 E6 7B           4051 	 mov dptr,#_XAUTODAT1
   0E68                    4052 	010001$:
   0E68 E0                 4053 	 movx a,@dptr
   0E69 F5 90              4054 	 mov _IOB,a
   0E6B D2 A6              4055 	 setb _IOC6
   0E6D 29                 4056 	 add a,r1
   0E6E F9                 4057 	 mov r1,a
   0E6F C2 A6              4058 	 clr _IOC6
   0E71 DA F5              4059 	 djnz r2, 010001$
                           4060 ;	# 185 "../../include/ztex-fpga6.h"
                           4061 	
   0E73 90 3A 20           4062 	 mov dptr,#_fpga_checksum
   0E76 E9                 4063 	 mov a,r1
   0E77 F0                 4064 	 movx @dptr,a
                           4065 	
   0E78                    4066 	010000$:
                           4067 	     
                           4068 ;	../../include/ztex-fpga6.h:193: if ( EP0BCL<64 ) {
   0E78 85 15 B3           4069 	mov	_OEB,_fpga_send_ep0_oOEB_1_1
                           4070 ;	../../include/ztex-fpga6.h:194: finish_fpga_configuration();
   0E7B 90 E6 8B           4071 	mov	dptr,#_EP0BCL
   0E7E E0                 4072 	movx	a,@dptr
   0E7F FA                 4073 	mov	r2,a
   0E80 BA 40 00           4074 	cjne	r2,#0x40,00106$
   0E83                    4075 00106$:
   0E83 50 03              4076 	jnc	00103$
                           4077 ;	../../include/ztex-fpga6.h:195: } 
   0E85 02 0D D9           4078 	ljmp	_finish_fpga_configuration
   0E88                    4079 00103$:
   0E88 22                 4080 	ret
                           4081 ;------------------------------------------------------------
                           4082 ;Allocation info for local variables in function 'init_cpld_fpga_configuration'
                           4083 ;------------------------------------------------------------
                           4084 ;------------------------------------------------------------
                           4085 ;	../../include/ztex-fpga6.h:249: void init_cpld_fpga_configuration() {
                           4086 ;	-----------------------------------------
                           4087 ;	 function init_cpld_fpga_configuration
                           4088 ;	-----------------------------------------
   0E89                    4089 _init_cpld_fpga_configuration:
                           4090 ;	../../include/ztex-fpga6.h:250: IFCONFIG = bmBIT7 | bmBIT6 | bmBIT5 | 2;	// Internal source, 48MHz, GPIF
   0E89 90 E6 01           4091 	mov	dptr,#_IFCONFIG
   0E8C 74 E2              4092 	mov	a,#0xE2
   0E8E F0                 4093 	movx	@dptr,a
                           4094 ;	../../include/ztex-fpga6.h:252: GPIFREADYCFG = 0; //bmBIT7 | bmBIT6 | bmBIT5;
   0E8F 90 E6 F3           4095 	mov	dptr,#_GPIFREADYCFG
                           4096 ;	../../include/ztex-fpga6.h:253: GPIFCTLCFG = 0x0; 
                           4097 ;	../../include/ztex-fpga6.h:254: GPIFIDLECS = 0;
   0E92 E4                 4098 	clr	a
   0E93 F0                 4099 	movx	@dptr,a
   0E94 90 E6 C3           4100 	mov	dptr,#_GPIFCTLCFG
   0E97 F0                 4101 	movx	@dptr,a
   0E98 90 E6 C1           4102 	mov	dptr,#_GPIFIDLECS
   0E9B F0                 4103 	movx	@dptr,a
                           4104 ;	../../include/ztex-fpga6.h:255: GPIFIDLECTL = 4;
   0E9C 90 E6 C2           4105 	mov	dptr,#_GPIFIDLECTL
   0E9F 74 04              4106 	mov	a,#0x04
   0EA1 F0                 4107 	movx	@dptr,a
                           4108 ;	../../include/ztex-fpga6.h:256: GPIFWFSELECT = 0x4E;
   0EA2 90 E6 C0           4109 	mov	dptr,#_GPIFWFSELECT
   0EA5 74 4E              4110 	mov	a,#0x4E
   0EA7 F0                 4111 	movx	@dptr,a
                           4112 ;	../../include/ztex-fpga6.h:257: GPIFREADYSTAT = 0;
   0EA8 90 E6 F4           4113 	mov	dptr,#_GPIFREADYSTAT
   0EAB E4                 4114 	clr	a
   0EAC F0                 4115 	movx	@dptr,a
                           4116 ;	../../include/ztex-utils.h:121: AUTOPTRL1=LO(&($0));
   0EAD 75 9B 39           4117 	mov	_AUTOPTRL1,#_GPIF_WAVE_DATA_HSFPGA_24MHZ
                           4118 ;	../../include/ztex-utils.h:122: AUTOPTRH1=HI(&($0));
   0EB0 7A 39              4119 	mov	r2,#_GPIF_WAVE_DATA_HSFPGA_24MHZ
   0EB2 7B 3A              4120 	mov	r3,#(_GPIF_WAVE_DATA_HSFPGA_24MHZ >> 8)
   0EB4 8B 9A              4121 	mov	_AUTOPTRH1,r3
                           4122 ;	../../include/ztex-utils.h:123: AUTOPTRL2=LO(&($1));
   0EB6 75 9E 60           4123 	mov	_AUTOPTRL2,#0x60
                           4124 ;	../../include/ztex-utils.h:124: AUTOPTRH2=HI(&($1));
   0EB9 75 9D E4           4125 	mov	_AUTOPTRH2,#0xE4
                           4126 ;	../../include/ztex-utils.h:130: __endasm; 
                           4127 	
   0EBC C0 02              4128 	  push ar2
   0EBE 7A 20              4129 	    mov r2,#(32);
   0EC0 12 02 AD           4130 	  lcall _MEM_COPY1_int
   0EC3 D0 02              4131 	  pop ar2
                           4132 	        
                           4133 ;	../../include/ztex-fpga6.h:261: FLOWSTATE = 0;
   0EC5 90 E6 C6           4134 	mov	dptr,#_FLOWSTATE
   0EC8 E4                 4135 	clr	a
   0EC9 F0                 4136 	movx	@dptr,a
                           4137 ;	../../include/ztex-fpga6.h:262: FLOWLOGIC = 0x10;
   0ECA 90 E6 C7           4138 	mov	dptr,#_FLOWLOGIC
   0ECD 74 10              4139 	mov	a,#0x10
   0ECF F0                 4140 	movx	@dptr,a
                           4141 ;	../../include/ztex-fpga6.h:263: FLOWEQ0CTL = 0;
   0ED0 90 E6 C8           4142 	mov	dptr,#_FLOWEQ0CTL
                           4143 ;	../../include/ztex-fpga6.h:264: FLOWEQ1CTL = 0;
                           4144 ;	../../include/ztex-fpga6.h:265: FLOWHOLDOFF = 0;
                           4145 ;	../../include/ztex-fpga6.h:266: FLOWSTB = 0;
   0ED3 E4                 4146 	clr	a
   0ED4 F0                 4147 	movx	@dptr,a
   0ED5 90 E6 C9           4148 	mov	dptr,#_FLOWEQ1CTL
   0ED8 F0                 4149 	movx	@dptr,a
   0ED9 90 E6 CA           4150 	mov	dptr,#_FLOWHOLDOFF
   0EDC F0                 4151 	movx	@dptr,a
   0EDD 90 E6 CB           4152 	mov	dptr,#_FLOWSTB
   0EE0 F0                 4153 	movx	@dptr,a
                           4154 ;	../../include/ztex-fpga6.h:267: FLOWSTBEDGE = 0;
   0EE1 90 E6 CC           4155 	mov	dptr,#_FLOWSTBEDGE
                           4156 ;	../../include/ztex-fpga6.h:268: FLOWSTBHPERIOD = 0;
   0EE4 E4                 4157 	clr	a
   0EE5 F0                 4158 	movx	@dptr,a
   0EE6 90 E6 CD           4159 	mov	dptr,#_FLOWSTBHPERIOD
   0EE9 F0                 4160 	movx	@dptr,a
                           4161 ;	../../include/ztex-fpga6.h:270: REVCTL = 0x1;				// reset fifo
   0EEA 90 E6 0B           4162 	mov	dptr,#_REVCTL
   0EED 74 01              4163 	mov	a,#0x01
   0EEF F0                 4164 	movx	@dptr,a
                           4165 ;	../../include/ezregs.h:46: __endasm;
                           4166 	
   0EF0 00                 4167 	 nop
   0EF1 00                 4168 	 nop
   0EF2 00                 4169 	 nop
   0EF3 00                 4170 	 nop
                           4171 	    
                           4172 ;	../../include/ztex-fpga6.h:272: FIFORESET = 0x80;
   0EF4 90 E6 04           4173 	mov	dptr,#_FIFORESET
   0EF7 74 80              4174 	mov	a,#0x80
   0EF9 F0                 4175 	movx	@dptr,a
                           4176 ;	../../include/ezregs.h:46: __endasm;
                           4177 	
   0EFA 00                 4178 	 nop
   0EFB 00                 4179 	 nop
   0EFC 00                 4180 	 nop
   0EFD 00                 4181 	 nop
                           4182 	    
                           4183 ;	../../include/ztex-fpga6.h:274: FIFORESET = HS_FPGA_CONF_EP;
   0EFE 90 E6 04           4184 	mov	dptr,#_FIFORESET
   0F01 74 02              4185 	mov	a,#0x02
   0F03 F0                 4186 	movx	@dptr,a
                           4187 ;	../../include/ezregs.h:46: __endasm;
                           4188 	
   0F04 00                 4189 	 nop
   0F05 00                 4190 	 nop
   0F06 00                 4191 	 nop
   0F07 00                 4192 	 nop
                           4193 	    
                           4194 ;	../../include/ztex-fpga6.h:276: FIFORESET = 0x0;
   0F08 90 E6 04           4195 	mov	dptr,#_FIFORESET
   0F0B E4                 4196 	clr	a
   0F0C F0                 4197 	movx	@dptr,a
                           4198 ;	../../include/ezregs.h:46: __endasm;
                           4199 	
   0F0D 00                 4200 	 nop
   0F0E 00                 4201 	 nop
   0F0F 00                 4202 	 nop
   0F10 00                 4203 	 nop
                           4204 	    
                           4205 ;	../../include/ztex-fpga6.h:279: EPHS_FPGA_CONF_EPFIFOCFG = 0;		// config fifo
   0F11 90 E6 18           4206 	mov	dptr,#_EP2FIFOCFG
   0F14 E4                 4207 	clr	a
   0F15 F0                 4208 	movx	@dptr,a
                           4209 ;	../../include/ezregs.h:46: __endasm;
                           4210 	
   0F16 00                 4211 	 nop
   0F17 00                 4212 	 nop
   0F18 00                 4213 	 nop
   0F19 00                 4214 	 nop
                           4215 	    
                           4216 ;	../../include/ztex-fpga6.h:281: EPHS_FPGA_CONF_EPFIFOCFG = bmBIT4 | 0;
   0F1A 90 E6 18           4217 	mov	dptr,#_EP2FIFOCFG
   0F1D 74 10              4218 	mov	a,#0x10
   0F1F F0                 4219 	movx	@dptr,a
                           4220 ;	../../include/ezregs.h:46: __endasm;
                           4221 	
   0F20 00                 4222 	 nop
   0F21 00                 4223 	 nop
   0F22 00                 4224 	 nop
   0F23 00                 4225 	 nop
                           4226 	    
                           4227 ;	../../include/ztex-fpga6.h:283: EPHS_FPGA_CONF_EPGPIFFLGSEL = 1;
   0F24 90 E6 D2           4228 	mov	dptr,#_EP2GPIFFLGSEL
   0F27 74 01              4229 	mov	a,#0x01
   0F29 F0                 4230 	movx	@dptr,a
                           4231 ;	../../include/ezregs.h:46: __endasm;
                           4232 	
   0F2A 00                 4233 	 nop
   0F2B 00                 4234 	 nop
   0F2C 00                 4235 	 nop
   0F2D 00                 4236 	 nop
                           4237 	    
                           4238 ;	../../include/ztex-fpga6.h:286: GPIFTCB3 = 1;				// abort after at least 14*65536 transactions
   0F2E 90 E6 CE           4239 	mov	dptr,#_GPIFTCB3
   0F31 74 01              4240 	mov	a,#0x01
   0F33 F0                 4241 	movx	@dptr,a
                           4242 ;	../../include/ezregs.h:46: __endasm;
                           4243 	
   0F34 00                 4244 	 nop
   0F35 00                 4245 	 nop
   0F36 00                 4246 	 nop
   0F37 00                 4247 	 nop
                           4248 	    
                           4249 ;	../../include/ztex-fpga6.h:288: GPIFTCB2 = 0;
   0F38 90 E6 CF           4250 	mov	dptr,#_GPIFTCB2
   0F3B E4                 4251 	clr	a
   0F3C F0                 4252 	movx	@dptr,a
                           4253 ;	../../include/ezregs.h:46: __endasm;
                           4254 	
   0F3D 00                 4255 	 nop
   0F3E 00                 4256 	 nop
   0F3F 00                 4257 	 nop
   0F40 00                 4258 	 nop
                           4259 	    
                           4260 ;	../../include/ztex-fpga6.h:290: GPIFTCB1 = 0;
   0F41 90 E6 D0           4261 	mov	dptr,#_GPIFTCB1
   0F44 E4                 4262 	clr	a
   0F45 F0                 4263 	movx	@dptr,a
                           4264 ;	../../include/ezregs.h:46: __endasm;
                           4265 	
   0F46 00                 4266 	 nop
   0F47 00                 4267 	 nop
   0F48 00                 4268 	 nop
   0F49 00                 4269 	 nop
                           4270 	    
                           4271 ;	../../include/ztex-fpga6.h:292: GPIFTCB0 = 0;
   0F4A 90 E6 D1           4272 	mov	dptr,#_GPIFTCB0
   0F4D E4                 4273 	clr	a
   0F4E F0                 4274 	movx	@dptr,a
                           4275 ;	../../include/ezregs.h:46: __endasm;
                           4276 	
   0F4F 00                 4277 	 nop
   0F50 00                 4278 	 nop
   0F51 00                 4279 	 nop
   0F52 00                 4280 	 nop
                           4281 	    
                           4282 ;	../../include/ztex-fpga6.h:295: EPHS_FPGA_CONF_EPGPIFTRIG = 0xff;		// arm fifos
   0F53 90 E6 D4           4283 	mov	dptr,#_EP2GPIFTRIG
   0F56 74 FF              4284 	mov	a,#0xFF
   0F58 F0                 4285 	movx	@dptr,a
                           4286 ;	../../include/ezregs.h:46: __endasm;
                           4287 	
   0F59 00                 4288 	 nop
   0F5A 00                 4289 	 nop
   0F5B 00                 4290 	 nop
   0F5C 00                 4291 	 nop
                           4292 	    
                           4293 ;	../../include/ztex-fpga6.h:298: OEC &= ~bmBIT6;				// disable CCLK output
   0F5D 53 B4 BF           4294 	anl	_OEC,#0xBF
                           4295 ;	../../include/ztex-fpga6.h:299: IOE = bmBIT4 | bmBIT7;		        // HS config mode
   0F60 75 B1 90           4296 	mov	_IOE,#0x90
   0F63 22                 4297 	ret
                           4298 ;------------------------------------------------------------
                           4299 ;Allocation info for local variables in function 'fpga_configure_from_flash'
                           4300 ;------------------------------------------------------------
                           4301 ;force                     Allocated to registers r2 
                           4302 ;c                         Allocated to registers r2 
                           4303 ;i                         Allocated to registers r3 r4 
                           4304 ;------------------------------------------------------------
                           4305 ;	../../include/ztex-fpga6.h:351: BYTE fpga_configure_from_flash( BYTE force) {
                           4306 ;	-----------------------------------------
                           4307 ;	 function fpga_configure_from_flash
                           4308 ;	-----------------------------------------
   0F64                    4309 _fpga_configure_from_flash:
                           4310 ;	../../include/ztex-fpga6.h:355: if ( ( force == 0 ) && ( IOE & bmBIT0 ) ) {
   0F64 E5 82              4311 	mov	a,dpl
   0F66 FA                 4312 	mov	r2,a
   0F67 70 0F              4313 	jnz	00102$
   0F69 E5 B1              4314 	mov	a,_IOE
   0F6B 30 E0 0A           4315 	jnb	acc.0,00102$
                           4316 ;	../../include/ztex-fpga6.h:356: fpga_flash_result = 1;
   0F6E 90 3A 26           4317 	mov	dptr,#_fpga_flash_result
   0F71 74 01              4318 	mov	a,#0x01
   0F73 F0                 4319 	movx	@dptr,a
                           4320 ;	../../include/ztex-fpga6.h:357: return 1;
   0F74 75 82 01           4321 	mov	dpl,#0x01
   0F77 22                 4322 	ret
   0F78                    4323 00102$:
                           4324 ;	../../include/ztex-fpga6.h:360: fpga_flash_result = 0;
   0F78 90 3A 26           4325 	mov	dptr,#_fpga_flash_result
   0F7B E4                 4326 	clr	a
   0F7C F0                 4327 	movx	@dptr,a
                           4328 ;	../../include/ztex-fpga6.h:362: c = OESPI_PORT;
   0F7D AA B4              4329 	mov	r2,_OEC
                           4330 ;	../../include/ztex-fpga6.h:363: OESPI_PORT &= ~( bmBITSPI_BIT_CS | bmBITSPI_BIT_DI | bmBITSPI_BIT_CLK );	// disable SPI outputs
   0F7F 53 B4 1F           4331 	anl	_OEC,#0x1F
                           4332 ;	../../include/ztex-fpga6.h:371: OEE = bmBIT3 | bmBIT4 | bmBIT7;
   0F82 75 B6 98           4333 	mov	_OEE,#0x98
                           4334 ;	../../include/ztex-fpga6.h:372: IOE = 0;
   0F85 75 B1 00           4335 	mov	_IOE,#0x00
                           4336 ;	../../include/ztex-fpga6.h:373: wait(1);
   0F88 90 00 01           4337 	mov	dptr,#0x0001
   0F8B C0 02              4338 	push	ar2
   0F8D 12 02 65           4339 	lcall	_wait
                           4340 ;	../../include/ztex-fpga6.h:374: IOE = bmBIT7;
   0F90 75 B1 80           4341 	mov	_IOE,#0x80
                           4342 ;	../../include/ztex-fpga6.h:377: wait(10);
   0F93 90 00 0A           4343 	mov	dptr,#0x000A
   0F96 12 02 65           4344 	lcall	_wait
   0F99 D0 02              4345 	pop	ar2
                           4346 ;	../../include/ztex-fpga6.h:378: for (i=0; (IOE & bmBIT1) && (SPI_CS==0) && i<10000; i++ ) { 
   0F9B 7B 00              4347 	mov	r3,#0x00
   0F9D 7C 00              4348 	mov	r4,#0x00
   0F9F                    4349 00109$:
   0F9F E5 B1              4350 	mov	a,_IOE
   0FA1 30 E1 25           4351 	jnb	acc.1,00112$
   0FA4 20 A5 22           4352 	jb	_IOC5,00112$
   0FA7 C3                 4353 	clr	c
   0FA8 EB                 4354 	mov	a,r3
   0FA9 94 10              4355 	subb	a,#0x10
   0FAB EC                 4356 	mov	a,r4
   0FAC 94 27              4357 	subb	a,#0x27
   0FAE 50 19              4358 	jnc	00112$
                           4359 ;	../../include/ztex-fpga6.h:379: wait(1);
   0FB0 90 00 01           4360 	mov	dptr,#0x0001
   0FB3 C0 02              4361 	push	ar2
   0FB5 C0 03              4362 	push	ar3
   0FB7 C0 04              4363 	push	ar4
   0FB9 12 02 65           4364 	lcall	_wait
   0FBC D0 04              4365 	pop	ar4
   0FBE D0 03              4366 	pop	ar3
   0FC0 D0 02              4367 	pop	ar2
                           4368 ;	../../include/ztex-fpga6.h:378: for (i=0; (IOE & bmBIT1) && (SPI_CS==0) && i<10000; i++ ) { 
   0FC2 0B                 4369 	inc	r3
   0FC3 BB 00 D9           4370 	cjne	r3,#0x00,00109$
   0FC6 0C                 4371 	inc	r4
   0FC7 80 D6              4372 	sjmp	00109$
   0FC9                    4373 00112$:
                           4374 ;	../../include/ztex-fpga6.h:382: wait(1);
   0FC9 90 00 01           4375 	mov	dptr,#0x0001
   0FCC C0 02              4376 	push	ar2
   0FCE 12 02 65           4377 	lcall	_wait
   0FD1 D0 02              4378 	pop	ar2
                           4379 ;	../../include/ztex-fpga6.h:384: if ( IOE & bmBIT0 )  {
   0FD3 E5 B1              4380 	mov	a,_IOE
   0FD5 30 E0 09           4381 	jnb	acc.0,00105$
                           4382 ;	../../include/ztex-fpga6.h:385: post_fpga_config();
   0FD8 C0 02              4383 	push	ar2
   0FDA 12 0D D8           4384 	lcall	_post_fpga_config
   0FDD D0 02              4385 	pop	ar2
   0FDF 80 13              4386 	sjmp	00106$
   0FE1                    4387 00105$:
                           4388 ;	../../include/ztex-fpga6.h:388: IOE =  bmBIT3 | bmBIT4;	// leave master SPI config mode
   0FE1 75 B1 18           4389 	mov	_IOE,#0x18
                           4390 ;	../../include/ztex-fpga6.h:389: wait(1);
   0FE4 90 00 01           4391 	mov	dptr,#0x0001
   0FE7 C0 02              4392 	push	ar2
   0FE9 12 02 65           4393 	lcall	_wait
   0FEC D0 02              4394 	pop	ar2
                           4395 ;	../../include/ztex-fpga6.h:390: fpga_flash_result = 4;
   0FEE 90 3A 26           4396 	mov	dptr,#_fpga_flash_result
   0FF1 74 04              4397 	mov	a,#0x04
   0FF3 F0                 4398 	movx	@dptr,a
   0FF4                    4399 00106$:
                           4400 ;	../../include/ztex-fpga6.h:392: OEE = 0;
   0FF4 75 B6 00           4401 	mov	_OEE,#0x00
                           4402 ;	../../include/ztex-fpga6.h:394: OESPI_PORT = c;
   0FF7 8A B4              4403 	mov	_OEC,r2
                           4404 ;	../../include/ztex-fpga6.h:395: SPI_CS = 1;
   0FF9 D2 A5              4405 	setb	_IOC5
                           4406 ;	../../include/ztex-fpga6.h:397: return fpga_flash_result;
   0FFB 90 3A 26           4407 	mov	dptr,#_fpga_flash_result
   0FFE E0                 4408 	movx	a,@dptr
   0FFF F5 82              4409 	mov	dpl,a
   1001 22                 4410 	ret
                           4411 ;------------------------------------------------------------
                           4412 ;Allocation info for local variables in function 'fpga_first_free_sector'
                           4413 ;------------------------------------------------------------
                           4414 ;i                         Allocated to registers r2 
                           4415 ;j                         Allocated to registers r3 
                           4416 ;buf                       Allocated with name '_fpga_first_free_sector_buf_1_1'
                           4417 ;------------------------------------------------------------
                           4418 ;	../../include/ztex-fpga-flash2.h:31: WORD fpga_first_free_sector() {
                           4419 ;	-----------------------------------------
                           4420 ;	 function fpga_first_free_sector
                           4421 ;	-----------------------------------------
   1002                    4422 _fpga_first_free_sector:
                           4423 ;	../../include/ztex-fpga-flash2.h:36: if ( config_data_valid ) {
   1002 90 3A 06           4424 	mov	dptr,#_config_data_valid
   1005 E0                 4425 	movx	a,@dptr
   1006 FA                 4426 	mov	r2,a
   1007 60 56              4427 	jz	00104$
                           4428 ;	../../include/ztex-fpga-flash2.h:37: mac_eeprom_read ( (__xdata BYTE*) buf, 26, 4 );		// read actual and max bitstream size 
   1009 75 10 1A           4429 	mov	_mac_eeprom_read_PARM_2,#0x1A
   100C 75 11 04           4430 	mov	_mac_eeprom_read_PARM_3,#0x04
   100F 90 3A 29           4431 	mov	dptr,#_fpga_first_free_sector_buf_1_1
   1012 12 05 D4           4432 	lcall	_mac_eeprom_read
                           4433 ;	../../include/ztex-fpga-flash2.h:38: if ( buf[1] != 0 ) {
   1015 90 3A 2B           4434 	mov	dptr,#(_fpga_first_free_sector_buf_1_1 + 0x0002)
   1018 E0                 4435 	movx	a,@dptr
   1019 FA                 4436 	mov	r2,a
   101A A3                 4437 	inc	dptr
   101B E0                 4438 	movx	a,@dptr
   101C FB                 4439 	mov	r3,a
   101D 4A                 4440 	orl	a,r2
   101E 60 3F              4441 	jz	00104$
                           4442 ;	../../include/ztex-fpga-flash2.h:39: return ( ( ( buf[1] > buf[0] ? buf[1] : buf[0] ) - 1 ) >> ((flash_sector_size & 255) - 12) ) + 1;
   1020 90 3A 29           4443 	mov	dptr,#_fpga_first_free_sector_buf_1_1
   1023 E0                 4444 	movx	a,@dptr
   1024 FC                 4445 	mov	r4,a
   1025 A3                 4446 	inc	dptr
   1026 E0                 4447 	movx	a,@dptr
   1027 FD                 4448 	mov	r5,a
   1028 C3                 4449 	clr	c
   1029 EC                 4450 	mov	a,r4
   102A 9A                 4451 	subb	a,r2
   102B ED                 4452 	mov	a,r5
   102C 9B                 4453 	subb	a,r3
   102D 40 04              4454 	jc	00115$
   102F 8C 02              4455 	mov	ar2,r4
   1031 8D 03              4456 	mov	ar3,r5
   1033                    4457 00115$:
   1033 1A                 4458 	dec	r2
   1034 BA FF 01           4459 	cjne	r2,#0xff,00127$
   1037 1B                 4460 	dec	r3
   1038                    4461 00127$:
   1038 90 3A 08           4462 	mov	dptr,#_flash_sector_size
   103B E0                 4463 	movx	a,@dptr
   103C FC                 4464 	mov	r4,a
   103D A3                 4465 	inc	dptr
   103E E0                 4466 	movx	a,@dptr
   103F 7D 00              4467 	mov	r5,#0x00
   1041 EC                 4468 	mov	a,r4
   1042 24 F4              4469 	add	a,#0xf4
   1044 FC                 4470 	mov	r4,a
   1045 ED                 4471 	mov	a,r5
   1046 34 FF              4472 	addc	a,#0xff
   1048 FD                 4473 	mov	r5,a
   1049 8C F0              4474 	mov	b,r4
   104B 05 F0              4475 	inc	b
   104D 80 07              4476 	sjmp	00129$
   104F                    4477 00128$:
   104F C3                 4478 	clr	c
   1050 EB                 4479 	mov	a,r3
   1051 13                 4480 	rrc	a
   1052 FB                 4481 	mov	r3,a
   1053 EA                 4482 	mov	a,r2
   1054 13                 4483 	rrc	a
   1055 FA                 4484 	mov	r2,a
   1056                    4485 00129$:
   1056 D5 F0 F6           4486 	djnz	b,00128$
   1059 8A 82              4487 	mov	dpl,r2
   105B 8B 83              4488 	mov	dph,r3
   105D A3                 4489 	inc	dptr
   105E 22                 4490 	ret
   105F                    4491 00104$:
                           4492 ;	../../include/ztex-fpga-flash2.h:42: #endif    
   105F 90 00 00           4493 	mov	dptr,#0x0000
   1062 12 09 63           4494 	lcall	_flash_read_init
                           4495 ;	../../include/ztex-fpga-flash2.h:44: for ( i=0; i<8 && flash_read_byte()==fpga_flash_boot_id[i]; i++ );
   1065 7A 00              4496 	mov	r2,#0x00
   1067                    4497 00108$:
   1067 BA 08 00           4498 	cjne	r2,#0x08,00130$
   106A                    4499 00130$:
   106A 50 16              4500 	jnc	00111$
   106C C0 02              4501 	push	ar2
   106E 12 07 E9           4502 	lcall	_flash_read_byte
   1071 AB 82              4503 	mov	r3,dpl
   1073 D0 02              4504 	pop	ar2
   1075 EA                 4505 	mov	a,r2
   1076 90 1F 40           4506 	mov	dptr,#_fpga_flash_boot_id
   1079 93                 4507 	movc	a,@a+dptr
   107A FC                 4508 	mov	r4,a
   107B EB                 4509 	mov	a,r3
   107C B5 04 03           4510 	cjne	a,ar4,00111$
   107F 0A                 4511 	inc	r2
   1080 80 E5              4512 	sjmp	00108$
   1082                    4513 00111$:
                           4514 ;	../../include/ztex-fpga-flash2.h:45: if ( i != 8 ) {
   1082 BA 08 02           4515 	cjne	r2,#0x08,00134$
   1085 80 1A              4516 	sjmp	00106$
   1087                    4517 00134$:
                           4518 ;	../../include/ztex-fpga-flash2.h:46: flash_read_finish(flash_sector_size - i);	// dummy-read the rest of the sector + finish read opration
   1087 7B 00              4519 	mov	r3,#0x00
   1089 90 3A 08           4520 	mov	dptr,#_flash_sector_size
   108C E0                 4521 	movx	a,@dptr
   108D FC                 4522 	mov	r4,a
   108E A3                 4523 	inc	dptr
   108F E0                 4524 	movx	a,@dptr
   1090 FD                 4525 	mov	r5,a
   1091 EC                 4526 	mov	a,r4
   1092 C3                 4527 	clr	c
   1093 9A                 4528 	subb	a,r2
   1094 F5 82              4529 	mov	dpl,a
   1096 ED                 4530 	mov	a,r5
   1097 9B                 4531 	subb	a,r3
   1098 F5 83              4532 	mov	dph,a
   109A 12 09 E3           4533 	lcall	_flash_read_finish
                           4534 ;	../../include/ztex-fpga-flash2.h:47: return 0;
   109D 90 00 00           4535 	mov	dptr,#0x0000
   10A0 22                 4536 	ret
   10A1                    4537 00106$:
                           4538 ;	../../include/ztex-fpga-flash2.h:49: i=flash_read_byte();
   10A1 12 07 E9           4539 	lcall	_flash_read_byte
   10A4 AA 82              4540 	mov	r2,dpl
                           4541 ;	../../include/ztex-fpga-flash2.h:50: j=flash_read_byte();
   10A6 C0 02              4542 	push	ar2
   10A8 12 07 E9           4543 	lcall	_flash_read_byte
   10AB AB 82              4544 	mov	r3,dpl
                           4545 ;	../../include/ztex-fpga-flash2.h:51: flash_read_finish(flash_sector_size - 10);		// dummy-read the rest of the sector + finish read opration
   10AD 90 3A 08           4546 	mov	dptr,#_flash_sector_size
   10B0 E0                 4547 	movx	a,@dptr
   10B1 FC                 4548 	mov	r4,a
   10B2 A3                 4549 	inc	dptr
   10B3 E0                 4550 	movx	a,@dptr
   10B4 FD                 4551 	mov	r5,a
   10B5 EC                 4552 	mov	a,r4
   10B6 24 F6              4553 	add	a,#0xf6
   10B8 F5 82              4554 	mov	dpl,a
   10BA ED                 4555 	mov	a,r5
   10BB 34 FF              4556 	addc	a,#0xff
   10BD F5 83              4557 	mov	dph,a
   10BF C0 03              4558 	push	ar3
   10C1 12 09 E3           4559 	lcall	_flash_read_finish
   10C4 D0 03              4560 	pop	ar3
   10C6 D0 02              4561 	pop	ar2
                           4562 ;	../../include/ztex-fpga-flash2.h:53: return (i | (j<<8))+1;
   10C8 8B 04              4563 	mov	ar4,r3
   10CA E4                 4564 	clr	a
   10CB FB                 4565 	mov	r3,a
   10CC FD                 4566 	mov	r5,a
   10CD EA                 4567 	mov	a,r2
   10CE 42 03              4568 	orl	ar3,a
   10D0 ED                 4569 	mov	a,r5
   10D1 42 04              4570 	orl	ar4,a
   10D3 8B 82              4571 	mov	dpl,r3
   10D5 8C 83              4572 	mov	dph,r4
   10D7 A3                 4573 	inc	dptr
   10D8 22                 4574 	ret
                           4575 ;------------------------------------------------------------
                           4576 ;Allocation info for local variables in function 'fpga_configure_from_flash_init'
                           4577 ;------------------------------------------------------------
                           4578 ;i                         Allocated to registers r2 
                           4579 ;buf                       Allocated with name '_fpga_configure_from_flash_init_buf_1_1'
                           4580 ;------------------------------------------------------------
                           4581 ;	../../include/ztex-fpga-flash2.h:60: BYTE fpga_configure_from_flash_init() {
                           4582 ;	-----------------------------------------
                           4583 ;	 function fpga_configure_from_flash_init
                           4584 ;	-----------------------------------------
   10D9                    4585 _fpga_configure_from_flash_init:
                           4586 ;	../../include/ztex-fpga-flash2.h:66: if ( config_data_valid ) {
   10D9 90 3A 06           4587 	mov	dptr,#_config_data_valid
   10DC E0                 4588 	movx	a,@dptr
   10DD FA                 4589 	mov	r2,a
   10DE 60 2F              4590 	jz	00106$
                           4591 ;	../../include/ztex-fpga-flash2.h:67: mac_eeprom_read ( (__xdata BYTE*) buf, 26, 4 );		// read actual and max bitstream size 
   10E0 75 10 1A           4592 	mov	_mac_eeprom_read_PARM_2,#0x1A
   10E3 75 11 04           4593 	mov	_mac_eeprom_read_PARM_3,#0x04
   10E6 90 3A 2D           4594 	mov	dptr,#_fpga_configure_from_flash_init_buf_1_1
   10E9 12 05 D4           4595 	lcall	_mac_eeprom_read
                           4596 ;	../../include/ztex-fpga-flash2.h:68: if ( buf[1] != 0 ) {
   10EC 90 3A 2F           4597 	mov	dptr,#(_fpga_configure_from_flash_init_buf_1_1 + 0x0002)
   10EF E0                 4598 	movx	a,@dptr
   10F0 FA                 4599 	mov	r2,a
   10F1 A3                 4600 	inc	dptr
   10F2 E0                 4601 	movx	a,@dptr
   10F3 FB                 4602 	mov	r3,a
   10F4 4A                 4603 	orl	a,r2
   10F5 60 18              4604 	jz	00106$
                           4605 ;	../../include/ztex-fpga-flash2.h:69: if ( buf[0] == 0 ) {
   10F7 90 3A 2D           4606 	mov	dptr,#_fpga_configure_from_flash_init_buf_1_1
   10FA E0                 4607 	movx	a,@dptr
   10FB FA                 4608 	mov	r2,a
   10FC A3                 4609 	inc	dptr
   10FD E0                 4610 	movx	a,@dptr
   10FE FB                 4611 	mov	r3,a
   10FF 4A                 4612 	orl	a,r2
   1100 60 03              4613 	jz	00140$
   1102 02 11 9C           4614 	ljmp	00113$
   1105                    4615 00140$:
                           4616 ;	../../include/ztex-fpga-flash2.h:70: return fpga_flash_result = 3;
   1105 90 3A 26           4617 	mov	dptr,#_fpga_flash_result
   1108 74 03              4618 	mov	a,#0x03
   110A F0                 4619 	movx	@dptr,a
   110B 75 82 03           4620 	mov	dpl,#0x03
   110E 22                 4621 	ret
                           4622 ;	../../include/ztex-fpga-flash2.h:73: goto flash_config;
   110F                    4623 00106$:
                           4624 ;	../../include/ztex-fpga-flash2.h:80: if ( flash_read_init( 0 ) )		// prepare reading sector 0
   110F 90 00 00           4625 	mov	dptr,#0x0000
   1112 12 09 63           4626 	lcall	_flash_read_init
   1115 E5 82              4627 	mov	a,dpl
   1117 60 0A              4628 	jz	00132$
                           4629 ;	../../include/ztex-fpga-flash2.h:81: return fpga_flash_result = 2;
   1119 90 3A 26           4630 	mov	dptr,#_fpga_flash_result
   111C 74 02              4631 	mov	a,#0x02
   111E F0                 4632 	movx	@dptr,a
   111F 75 82 02           4633 	mov	dpl,#0x02
   1122 22                 4634 	ret
                           4635 ;	../../include/ztex-fpga-flash2.h:82: for ( i=0; i<8 && flash_read_byte()==fpga_flash_boot_id[i]; i++ );
   1123                    4636 00132$:
   1123 7A 00              4637 	mov	r2,#0x00
   1125                    4638 00120$:
   1125 BA 08 00           4639 	cjne	r2,#0x08,00142$
   1128                    4640 00142$:
   1128 50 16              4641 	jnc	00123$
   112A C0 02              4642 	push	ar2
   112C 12 07 E9           4643 	lcall	_flash_read_byte
   112F AB 82              4644 	mov	r3,dpl
   1131 D0 02              4645 	pop	ar2
   1133 EA                 4646 	mov	a,r2
   1134 90 1F 40           4647 	mov	dptr,#_fpga_flash_boot_id
   1137 93                 4648 	movc	a,@a+dptr
   1138 FC                 4649 	mov	r4,a
   1139 EB                 4650 	mov	a,r3
   113A B5 04 03           4651 	cjne	a,ar4,00123$
   113D 0A                 4652 	inc	r2
   113E 80 E5              4653 	sjmp	00120$
   1140                    4654 00123$:
                           4655 ;	../../include/ztex-fpga-flash2.h:83: if ( i != 8 ) {
   1140 BA 08 02           4656 	cjne	r2,#0x08,00146$
   1143 80 20              4657 	sjmp	00110$
   1145                    4658 00146$:
                           4659 ;	../../include/ztex-fpga-flash2.h:84: flash_read_finish(flash_sector_size - i);	// dummy-read the rest of the sector + finish read opration
   1145 7B 00              4660 	mov	r3,#0x00
   1147 90 3A 08           4661 	mov	dptr,#_flash_sector_size
   114A E0                 4662 	movx	a,@dptr
   114B FC                 4663 	mov	r4,a
   114C A3                 4664 	inc	dptr
   114D E0                 4665 	movx	a,@dptr
   114E FD                 4666 	mov	r5,a
   114F EC                 4667 	mov	a,r4
   1150 C3                 4668 	clr	c
   1151 9A                 4669 	subb	a,r2
   1152 F5 82              4670 	mov	dpl,a
   1154 ED                 4671 	mov	a,r5
   1155 9B                 4672 	subb	a,r3
   1156 F5 83              4673 	mov	dph,a
   1158 12 09 E3           4674 	lcall	_flash_read_finish
                           4675 ;	../../include/ztex-fpga-flash2.h:85: return fpga_flash_result = 3;
   115B 90 3A 26           4676 	mov	dptr,#_fpga_flash_result
   115E 74 03              4677 	mov	a,#0x03
   1160 F0                 4678 	movx	@dptr,a
   1161 75 82 03           4679 	mov	dpl,#0x03
   1164 22                 4680 	ret
   1165                    4681 00110$:
                           4682 ;	../../include/ztex-fpga-flash2.h:87: i = flash_read_byte();
   1165 12 07 E9           4683 	lcall	_flash_read_byte
   1168 AA 82              4684 	mov	r2,dpl
                           4685 ;	../../include/ztex-fpga-flash2.h:88: i |= flash_read_byte();
   116A C0 02              4686 	push	ar2
   116C 12 07 E9           4687 	lcall	_flash_read_byte
   116F AB 82              4688 	mov	r3,dpl
   1171 D0 02              4689 	pop	ar2
   1173 EB                 4690 	mov	a,r3
   1174 42 02              4691 	orl	ar2,a
                           4692 ;	../../include/ztex-fpga-flash2.h:89: flash_read_finish(flash_sector_size - 10);		// dummy-read the rest of the sector + finish read opration
   1176 90 3A 08           4693 	mov	dptr,#_flash_sector_size
   1179 E0                 4694 	movx	a,@dptr
   117A FB                 4695 	mov	r3,a
   117B A3                 4696 	inc	dptr
   117C E0                 4697 	movx	a,@dptr
   117D FC                 4698 	mov	r4,a
   117E EB                 4699 	mov	a,r3
   117F 24 F6              4700 	add	a,#0xf6
   1181 F5 82              4701 	mov	dpl,a
   1183 EC                 4702 	mov	a,r4
   1184 34 FF              4703 	addc	a,#0xff
   1186 F5 83              4704 	mov	dph,a
   1188 C0 02              4705 	push	ar2
   118A 12 09 E3           4706 	lcall	_flash_read_finish
   118D D0 02              4707 	pop	ar2
                           4708 ;	../../include/ztex-fpga-flash2.h:90: if ( i==0 )
   118F EA                 4709 	mov	a,r2
   1190 70 0A              4710 	jnz	00113$
                           4711 ;	../../include/ztex-fpga-flash2.h:91: return fpga_flash_result = 3;
   1192 90 3A 26           4712 	mov	dptr,#_fpga_flash_result
   1195 74 03              4713 	mov	a,#0x03
   1197 F0                 4714 	movx	@dptr,a
   1198 75 82 03           4715 	mov	dpl,#0x03
                           4716 ;	../../include/ztex-fpga-flash2.h:93: flash_config:
   119B 22                 4717 	ret
   119C                    4718 00113$:
                           4719 ;	../../include/ztex-fpga-flash2.h:94: fpga_flash_result = fpga_configure_from_flash(0);
   119C 75 82 00           4720 	mov	dpl,#0x00
   119F 12 0F 64           4721 	lcall	_fpga_configure_from_flash
   11A2 AA 82              4722 	mov	r2,dpl
   11A4 90 3A 26           4723 	mov	dptr,#_fpga_flash_result
   11A7 EA                 4724 	mov	a,r2
   11A8 F0                 4725 	movx	@dptr,a
                           4726 ;	../../include/ztex-fpga-flash2.h:95: if ( fpga_flash_result == 1 ) {
   11A9 BA 01 05           4727 	cjne	r2,#0x01,00117$
                           4728 ;	../../include/ztex-fpga-flash2.h:96: post_fpga_config();
   11AC 12 0D D8           4729 	lcall	_post_fpga_config
   11AF 80 0F              4730 	sjmp	00118$
   11B1                    4731 00117$:
                           4732 ;	../../include/ztex-fpga-flash2.h:98: else if ( fpga_flash_result == 4 ) {
   11B1 BA 04 0C           4733 	cjne	r2,#0x04,00118$
                           4734 ;	../../include/ztex-fpga-flash2.h:99: fpga_flash_result = fpga_configure_from_flash(0);	// up to two tries
   11B4 75 82 00           4735 	mov	dpl,#0x00
   11B7 12 0F 64           4736 	lcall	_fpga_configure_from_flash
   11BA E5 82              4737 	mov	a,dpl
   11BC 90 3A 26           4738 	mov	dptr,#_fpga_flash_result
   11BF F0                 4739 	movx	@dptr,a
   11C0                    4740 00118$:
                           4741 ;	../../include/ztex-fpga-flash2.h:101: return fpga_flash_result;
   11C0 90 3A 26           4742 	mov	dptr,#_fpga_flash_result
   11C3 E0                 4743 	movx	a,@dptr
   11C4 F5 82              4744 	mov	dpl,a
   11C6 22                 4745 	ret
                           4746 ;------------------------------------------------------------
                           4747 ;Allocation info for local variables in function 'abscode_identity'
                           4748 ;------------------------------------------------------------
                           4749 ;------------------------------------------------------------
                           4750 ;	../../include/ztex-descriptors.h:129: void abscode_identity()// _naked
                           4751 ;	-----------------------------------------
                           4752 ;	 function abscode_identity
                           4753 ;	-----------------------------------------
   11C7                    4754 _abscode_identity:
                           4755 ;	../../include/ztex-descriptors.h:183: + 64
                           4756 	
                           4757 	    .area ABSCODE (ABS,CODE)
                           4758 	
   006C                    4759 	    .org 0x06c
   006C 28                 4760 	    .db 40
                           4761 	
   006D                    4762 	    .org _ZTEX_DESCRIPTOR_VERSION
   006D 01                 4763 	    .db 1
                           4764 	
   006E                    4765 	    .org _ZTEXID
   006E 5A 54 45 58        4766 	    .ascii "ZTEX"
                           4767 	
   0072                    4768 	    .org _PRODUCT_ID
   0072 0A                 4769 	    .db 10
   0073 11                 4770 	    .db 17
   0074 00                 4771 	    .db 0
   0075 00                 4772 	    .db 0
                           4773 	
   0076                    4774 	    .org _FW_VERSION
   0076 00                 4775 	    .db 0
                           4776 	
   0077                    4777 	    .org _INTERFACE_VERSION
   0077 01                 4778 	    .db 1
                           4779 	
   0078                    4780 	    .org _INTERFACE_CAPABILITIES
                           4781 ;	# 183 "../../include/ztex-descriptors.h"
   0078 67                 4782 	    .db 0 + 1 + 2 + 4 + 32 + 64
                           4783 ;	# 189 "../../include/ztex-descriptors.h"
   0079 00                 4784 	    .db 0
   007A 00                 4785 	    .db 0
   007B 00                 4786 	    .db 0
   007C 00                 4787 	    .db 0
   007D 00                 4788 	    .db 0
                           4789 	
   007E                    4790 	    .org _MODULE_RESERVED
   007E 00                 4791 	    .db 0
   007F 00                 4792 	    .db 0
   0080 00                 4793 	    .db 0
   0081 00                 4794 	    .db 0
   0082 00                 4795 	    .db 0
   0083 00                 4796 	    .db 0
   0084 00                 4797 	    .db 0
   0085 00                 4798 	    .db 0
   0086 00                 4799 	    .db 0
   0087 00                 4800 	    .db 0
   0088 00                 4801 	    .db 0
   0089 00                 4802 	    .db 0
                           4803 	
   008A                    4804 	    .org _SN_STRING
   008A 30 30 30 30 30 30  4805 	    .ascii "0000000000"
        30 30 30 30
                           4806 	
                           4807 	    .area CSEG (CODE)
                           4808 	    
   11C7 22                 4809 	ret
                           4810 ;------------------------------------------------------------
                           4811 ;Allocation info for local variables in function 'resetToggleData'
                           4812 ;------------------------------------------------------------
                           4813 ;------------------------------------------------------------
                           4814 ;	../../include/ztex-isr.h:34: static void resetToggleData () {
                           4815 ;	-----------------------------------------
                           4816 ;	 function resetToggleData
                           4817 ;	-----------------------------------------
   11C8                    4818 _resetToggleData:
                           4819 ;	../../include/ztex-isr.h:45: TOGCTL = 0;				// EP0 out
                           4820 ;	../../include/ztex-isr.h:46: TOGCTL = 0 | bmBIT5;
                           4821 ;	../../include/ztex-isr.h:47: TOGCTL = 0x10;			// EP0 in
                           4822 ;	../../include/ztex-isr.h:48: TOGCTL = 0x10 | bmBIT5;
   11C8 90 E6 83           4823 	mov	dptr,#_TOGCTL
   11CB E4                 4824 	clr	a
   11CC F0                 4825 	movx	@dptr,a
   11CD 74 20              4826 	mov	a,#0x20
   11CF F0                 4827 	movx	@dptr,a
   11D0 74 10              4828 	mov	a,#0x10
   11D2 F0                 4829 	movx	@dptr,a
   11D3 74 30              4830 	mov	a,#0x30
   11D5 F0                 4831 	movx	@dptr,a
                           4832 ;	../../include/ztex-isr.h:49: #ifeq[EP1OUT_DIR][OUT]
                           4833 ;	../../include/ztex-isr.h:51: TOGCTL = 1 | bmBIT5;
                           4834 ;	../../include/ztex-isr.h:52: #endif    
                           4835 ;	../../include/ztex-isr.h:55: TOGCTL = 0x11 | bmBIT5;
   11D6 90 E6 83           4836 	mov	dptr,#_TOGCTL
   11D9 74 01              4837 	mov	a,#0x01
   11DB F0                 4838 	movx	@dptr,a
   11DC 74 21              4839 	mov	a,#0x21
   11DE F0                 4840 	movx	@dptr,a
   11DF 74 11              4841 	mov	a,#0x11
   11E1 F0                 4842 	movx	@dptr,a
   11E2 74 31              4843 	mov	a,#0x31
   11E4 F0                 4844 	movx	@dptr,a
                           4845 ;	../../include/ztex-isr.h:36: #ifeq[EP$0_DIR][OUT]
                           4846 ;	../../include/ztex-isr.h:38: TOGCTL = $0 | bmBIT5;
   11E5 90 E6 83           4847 	mov	dptr,#_TOGCTL
   11E8 74 02              4848 	mov	a,#0x02
   11EA F0                 4849 	movx	@dptr,a
   11EB 74 22              4850 	mov	a,#0x22
   11ED F0                 4851 	movx	@dptr,a
   11EE 22                 4852 	ret
                           4853 ;------------------------------------------------------------
                           4854 ;Allocation info for local variables in function 'sendStringDescriptor'
                           4855 ;------------------------------------------------------------
                           4856 ;hiAddr                    Allocated with name '_sendStringDescriptor_PARM_2'
                           4857 ;size                      Allocated with name '_sendStringDescriptor_PARM_3'
                           4858 ;loAddr                    Allocated to registers r2 
                           4859 ;i                         Allocated to registers r2 
                           4860 ;------------------------------------------------------------
                           4861 ;	../../include/ztex-isr.h:68: static void sendStringDescriptor (BYTE loAddr, BYTE hiAddr, BYTE size)
                           4862 ;	-----------------------------------------
                           4863 ;	 function sendStringDescriptor
                           4864 ;	-----------------------------------------
   11EF                    4865 _sendStringDescriptor:
   11EF AA 82              4866 	mov	r2,dpl
                           4867 ;	../../include/ztex-isr.h:71: if ( size > 31) size = 31;
   11F1 E5 17              4868 	mov	a,_sendStringDescriptor_PARM_3
   11F3 24 E0              4869 	add	a,#0xff - 0x1F
   11F5 50 03              4870 	jnc	00102$
   11F7 75 17 1F           4871 	mov	_sendStringDescriptor_PARM_3,#0x1F
   11FA                    4872 00102$:
                           4873 ;	../../include/ztex-isr.h:72: if (SETUPDAT[7] == 0 && SETUPDAT[6]<size ) size = SETUPDAT[6];
   11FA 90 E6 BF           4874 	mov	dptr,#(_SETUPDAT + 0x0007)
   11FD E0                 4875 	movx	a,@dptr
   11FE 70 10              4876 	jnz	00104$
   1200 90 E6 BE           4877 	mov	dptr,#(_SETUPDAT + 0x0006)
   1203 E0                 4878 	movx	a,@dptr
   1204 FB                 4879 	mov	r3,a
   1205 C3                 4880 	clr	c
   1206 95 17              4881 	subb	a,_sendStringDescriptor_PARM_3
   1208 50 06              4882 	jnc	00104$
   120A 90 E6 BE           4883 	mov	dptr,#(_SETUPDAT + 0x0006)
   120D E0                 4884 	movx	a,@dptr
   120E F5 17              4885 	mov	_sendStringDescriptor_PARM_3,a
   1210                    4886 00104$:
                           4887 ;	../../include/ztex-isr.h:73: AUTOPTRSETUP = 7;
   1210 75 AF 07           4888 	mov	_AUTOPTRSETUP,#0x07
                           4889 ;	../../include/ztex-isr.h:74: AUTOPTRL1 = loAddr;
   1213 8A 9B              4890 	mov	_AUTOPTRL1,r2
                           4891 ;	../../include/ztex-isr.h:75: AUTOPTRH1 = hiAddr;
   1215 85 16 9A           4892 	mov	_AUTOPTRH1,_sendStringDescriptor_PARM_2
                           4893 ;	../../include/ztex-isr.h:76: AUTOPTRL2 = (BYTE)(((unsigned short)(&EP0BUF))+1);
   1218 75 9E 41           4894 	mov	_AUTOPTRL2,#0x41
                           4895 ;	../../include/ztex-isr.h:77: AUTOPTRH2 = (BYTE)((((unsigned short)(&EP0BUF))+1) >> 8);
   121B 75 9D E7           4896 	mov	_AUTOPTRH2,#0xE7
                           4897 ;	../../include/ztex-isr.h:78: XAUTODAT2 = 3;
   121E 90 E6 7C           4898 	mov	dptr,#_XAUTODAT2
   1221 74 03              4899 	mov	a,#0x03
   1223 F0                 4900 	movx	@dptr,a
                           4901 ;	../../include/ztex-isr.h:79: for (i=0; i<size; i++) {
   1224 7A 00              4902 	mov	r2,#0x00
   1226                    4903 00106$:
   1226 C3                 4904 	clr	c
   1227 EA                 4905 	mov	a,r2
   1228 95 17              4906 	subb	a,_sendStringDescriptor_PARM_3
   122A 50 11              4907 	jnc	00109$
                           4908 ;	../../include/ztex-isr.h:80: XAUTODAT2 = XAUTODAT1;
   122C 90 E6 7B           4909 	mov	dptr,#_XAUTODAT1
   122F E0                 4910 	movx	a,@dptr
   1230 FB                 4911 	mov	r3,a
   1231 90 E6 7C           4912 	mov	dptr,#_XAUTODAT2
   1234 F0                 4913 	movx	@dptr,a
                           4914 ;	../../include/ztex-isr.h:81: XAUTODAT2 = 0;
   1235 90 E6 7C           4915 	mov	dptr,#_XAUTODAT2
   1238 E4                 4916 	clr	a
   1239 F0                 4917 	movx	@dptr,a
                           4918 ;	../../include/ztex-isr.h:79: for (i=0; i<size; i++) {
   123A 0A                 4919 	inc	r2
   123B 80 E9              4920 	sjmp	00106$
   123D                    4921 00109$:
                           4922 ;	../../include/ztex-isr.h:83: i = (size+1) << 1;
   123D E5 17              4923 	mov	a,_sendStringDescriptor_PARM_3
   123F 04                 4924 	inc	a
                           4925 ;	../../include/ztex-isr.h:84: EP0BUF[0] = i;
   1240 25 E0              4926 	add	a,acc
   1242 FA                 4927 	mov	r2,a
   1243 90 E7 40           4928 	mov	dptr,#_EP0BUF
   1246 F0                 4929 	movx	@dptr,a
                           4930 ;	../../include/ztex-isr.h:85: EP0BUF[1] = 3;
   1247 90 E7 41           4931 	mov	dptr,#(_EP0BUF + 0x0001)
   124A 74 03              4932 	mov	a,#0x03
   124C F0                 4933 	movx	@dptr,a
                           4934 ;	../../include/ztex-isr.h:86: EP0BCH = 0;
   124D 90 E6 8A           4935 	mov	dptr,#_EP0BCH
   1250 E4                 4936 	clr	a
   1251 F0                 4937 	movx	@dptr,a
                           4938 ;	../../include/ztex-isr.h:87: EP0BCL = i;
   1252 90 E6 8B           4939 	mov	dptr,#_EP0BCL
   1255 EA                 4940 	mov	a,r2
   1256 F0                 4941 	movx	@dptr,a
   1257 22                 4942 	ret
                           4943 ;------------------------------------------------------------
                           4944 ;Allocation info for local variables in function 'ep0_payload_update'
                           4945 ;------------------------------------------------------------
                           4946 ;------------------------------------------------------------
                           4947 ;	../../include/ztex-isr.h:93: static void ep0_payload_update() {
                           4948 ;	-----------------------------------------
                           4949 ;	 function ep0_payload_update
                           4950 ;	-----------------------------------------
   1258                    4951 _ep0_payload_update:
                           4952 ;	../../include/ztex-isr.h:94: ep0_payload_transfer = ( ep0_payload_remaining > 64 ) ? 64 : ep0_payload_remaining;
   1258 90 3A 36           4953 	mov	dptr,#_ep0_payload_remaining
   125B E0                 4954 	movx	a,@dptr
   125C FA                 4955 	mov	r2,a
   125D A3                 4956 	inc	dptr
   125E E0                 4957 	movx	a,@dptr
   125F FB                 4958 	mov	r3,a
   1260 C3                 4959 	clr	c
   1261 74 40              4960 	mov	a,#0x40
   1263 9A                 4961 	subb	a,r2
   1264 E4                 4962 	clr	a
   1265 9B                 4963 	subb	a,r3
   1266 50 06              4964 	jnc	00103$
   1268 7C 40              4965 	mov	r4,#0x40
   126A 7D 00              4966 	mov	r5,#0x00
   126C 80 04              4967 	sjmp	00104$
   126E                    4968 00103$:
   126E 8A 04              4969 	mov	ar4,r2
   1270 8B 05              4970 	mov	ar5,r3
   1272                    4971 00104$:
   1272 90 3A 38           4972 	mov	dptr,#_ep0_payload_transfer
   1275 EC                 4973 	mov	a,r4
   1276 F0                 4974 	movx	@dptr,a
                           4975 ;	../../include/ztex-isr.h:95: ep0_payload_remaining -= ep0_payload_transfer;
   1277 7D 00              4976 	mov	r5,#0x00
   1279 90 3A 36           4977 	mov	dptr,#_ep0_payload_remaining
   127C EA                 4978 	mov	a,r2
   127D C3                 4979 	clr	c
   127E 9C                 4980 	subb	a,r4
   127F F0                 4981 	movx	@dptr,a
   1280 EB                 4982 	mov	a,r3
   1281 9D                 4983 	subb	a,r5
   1282 A3                 4984 	inc	dptr
   1283 F0                 4985 	movx	@dptr,a
   1284 22                 4986 	ret
                           4987 ;------------------------------------------------------------
                           4988 ;Allocation info for local variables in function 'ep0_vendor_cmd_su'
                           4989 ;------------------------------------------------------------
                           4990 ;------------------------------------------------------------
                           4991 ;	../../include/ztex-isr.h:102: static void ep0_vendor_cmd_su() {
                           4992 ;	-----------------------------------------
                           4993 ;	 function ep0_vendor_cmd_su
                           4994 ;	-----------------------------------------
   1285                    4995 _ep0_vendor_cmd_su:
                           4996 ;	../../include/ztex-isr.h:103: switch ( ep0_prev_setup_request ) {
   1285 90 3A 79           4997 	mov	dptr,#_ep0_prev_setup_request
   1288 E0                 4998 	movx	a,@dptr
   1289 FA                 4999 	mov	r2,a
   128A BA 31 03           5000 	cjne	r2,#0x31,00127$
   128D 02 13 19           5001 	ljmp	00107$
   1290                    5002 00127$:
   1290 BA 32 03           5003 	cjne	r2,#0x32,00128$
   1293 02 13 1C           5004 	ljmp	00108$
   1296                    5005 00128$:
   1296 BA 34 03           5006 	cjne	r2,#0x34,00129$
   1299 02 13 28           5007 	ljmp	00111$
   129C                    5008 00129$:
   129C BA 35 03           5009 	cjne	r2,#0x35,00130$
   129F 02 13 3B           5010 	ljmp	00112$
   12A2                    5011 00130$:
   12A2 BA 39 02           5012 	cjne	r2,#0x39,00131$
   12A5 80 0D              5013 	sjmp	00101$
   12A7                    5014 00131$:
   12A7 BA 3C 02           5015 	cjne	r2,#0x3C,00132$
   12AA 80 2C              5016 	sjmp	00102$
   12AC                    5017 00132$:
   12AC BA 42 02           5018 	cjne	r2,#0x42,00133$
   12AF 80 30              5019 	sjmp	00103$
   12B1                    5020 00133$:
   12B1 02 13 5A           5021 	ljmp	00113$
                           5022 ;	../../include/ztex-conf.h:123: case $0:			
   12B4                    5023 00101$:
                           5024 ;	../../include/ztex-eeprom.h:236: eeprom_write_checksum = 0;
   12B4 90 3A 04           5025 	mov	dptr,#_eeprom_write_checksum
                           5026 ;	../../include/ztex-eeprom.h:237: eeprom_write_bytes = 0;
   12B7 E4                 5027 	clr	a
   12B8 F0                 5028 	movx	@dptr,a
   12B9 90 3A 02           5029 	mov	dptr,#_eeprom_write_bytes
   12BC F0                 5030 	movx	@dptr,a
   12BD A3                 5031 	inc	dptr
   12BE F0                 5032 	movx	@dptr,a
                           5033 ;	../../include/ztex-eeprom.h:238: eeprom_addr =  ( SETUPDAT[3] << 8) | SETUPDAT[2];	// Address
   12BF 90 E6 BB           5034 	mov	dptr,#(_SETUPDAT + 0x0003)
   12C2 E0                 5035 	movx	a,@dptr
   12C3 FB                 5036 	mov	r3,a
   12C4 7A 00              5037 	mov	r2,#0x00
   12C6 90 E6 BA           5038 	mov	dptr,#(_SETUPDAT + 0x0002)
   12C9 E0                 5039 	movx	a,@dptr
   12CA FC                 5040 	mov	r4,a
   12CB 7D 00              5041 	mov	r5,#0x00
   12CD 90 3A 00           5042 	mov	dptr,#_eeprom_addr
   12D0 EC                 5043 	mov	a,r4
   12D1 4A                 5044 	orl	a,r2
   12D2 F0                 5045 	movx	@dptr,a
   12D3 ED                 5046 	mov	a,r5
   12D4 4B                 5047 	orl	a,r3
   12D5 A3                 5048 	inc	dptr
   12D6 F0                 5049 	movx	@dptr,a
                           5050 ;	../../include/ztex-conf.h:125: break;
   12D7 22                 5051 	ret
                           5052 ;	../../include/ztex-conf.h:123: case $0:			
   12D8                    5053 00102$:
                           5054 ;	../../include/ztex-conf.h:125: break;
   12D8 90 E6 BA           5055 	mov	dptr,#(_SETUPDAT + 0x0002)
   12DB E0                 5056 	movx	a,@dptr
   12DC 90 3A 05           5057 	mov	dptr,#_mac_eeprom_addr
   12DF F0                 5058 	movx	@dptr,a
   12E0 22                 5059 	ret
                           5060 ;	../../include/ztex-conf.h:123: case $0:			
   12E1                    5061 00103$:
                           5062 ;	../../include/ztex-flash2.h:694: ep0_write_mode = SETUPDAT[5];
   12E1 90 E6 BD           5063 	mov	dptr,#(_SETUPDAT + 0x0005)
   12E4 E0                 5064 	movx	a,@dptr
   12E5 FA                 5065 	mov	r2,a
   12E6 90 3A 1F           5066 	mov	dptr,#_ep0_write_mode
   12E9 F0                 5067 	movx	@dptr,a
                           5068 ;	../../include/ztex-flash2.h:695: if ( (ep0_write_mode == 0) && flash_write_init((SETUPDAT[3] << 8) | SETUPDAT[2]) ) {
   12EA EA                 5069 	mov	a,r2
   12EB 60 01              5070 	jz	00134$
   12ED 22                 5071 	ret
   12EE                    5072 00134$:
   12EE 90 E6 BB           5073 	mov	dptr,#(_SETUPDAT + 0x0003)
   12F1 E0                 5074 	movx	a,@dptr
   12F2 FB                 5075 	mov	r3,a
   12F3 7A 00              5076 	mov	r2,#0x00
   12F5 90 E6 BA           5077 	mov	dptr,#(_SETUPDAT + 0x0002)
   12F8 E0                 5078 	movx	a,@dptr
   12F9 7D 00              5079 	mov	r5,#0x00
   12FB 4A                 5080 	orl	a,r2
   12FC F5 82              5081 	mov	dpl,a
   12FE ED                 5082 	mov	a,r5
   12FF 4B                 5083 	orl	a,r3
   1300 F5 83              5084 	mov	dph,a
   1302 12 0B 59           5085 	lcall	_flash_write_init
   1305 E5 82              5086 	mov	a,dpl
   1307 70 01              5087 	jnz	00135$
   1309 22                 5088 	ret
   130A                    5089 00135$:
                           5090 ;	../../include/ztex-conf.h:137: EP0CS |= 0x01;	// set stall
   130A 90 E6 A0           5091 	mov	dptr,#_EP0CS
   130D E0                 5092 	movx	a,@dptr
   130E 44 01              5093 	orl	a,#0x01
   1310 F0                 5094 	movx	@dptr,a
                           5095 ;	../../include/ztex-conf.h:138: ep0_payload_remaining = 0;
   1311 90 3A 36           5096 	mov	dptr,#_ep0_payload_remaining
   1314 E4                 5097 	clr	a
   1315 F0                 5098 	movx	@dptr,a
   1316 A3                 5099 	inc	dptr
   1317 F0                 5100 	movx	@dptr,a
                           5101 ;	../../include/ztex-conf.h:139: break;
   1318 22                 5102 	ret
                           5103 ;	../../include/ztex-conf.h:123: case $0:			
   1319                    5104 00107$:
                           5105 ;	../../include/ztex-conf.h:124: $1
                           5106 ;	../../include/ztex-conf.h:125: break;
   1319 02 0D 27           5107 	ljmp	_reset_fpga
                           5108 ;	../../include/ztex-conf.h:123: case $0:			
   131C                    5109 00108$:
                           5110 ;	../../include/ztex-fpga6.h:199: if ( fpga_conf_initialized != 123 )
   131C 90 3A 27           5111 	mov	dptr,#_fpga_conf_initialized
   131F E0                 5112 	movx	a,@dptr
   1320 FA                 5113 	mov	r2,a
   1321 BA 7B 01           5114 	cjne	r2,#0x7B,00136$
   1324 22                 5115 	ret
   1325                    5116 00136$:
                           5117 ;	../../include/ztex-fpga6.h:200: init_fpga_configuration();
                           5118 ;	../../include/ztex-conf.h:125: break;
   1325 02 0D 56           5119 	ljmp	_init_fpga_configuration
                           5120 ;	../../include/ztex-conf.h:123: case $0:			
   1328                    5121 00111$:
                           5122 ;	../../include/ztex-fpga6.h:304: init_fpga_configuration();
   1328 12 0D 56           5123 	lcall	_init_fpga_configuration
                           5124 ;	../../include/ztex-fpga6.h:306: EPHS_FPGA_CONF_EPCS &= ~bmBIT0;		// clear stall bit
   132B 90 E6 A3           5125 	mov	dptr,#_EP2CS
   132E E0                 5126 	movx	a,@dptr
   132F 54 FE              5127 	anl	a,#0xFE
   1331 F0                 5128 	movx	@dptr,a
                           5129 ;	../../include/ztex-fpga6.h:308: GPIFABORT = 0xFF;				// abort pendig 
   1332 90 E6 F5           5130 	mov	dptr,#_GPIFABORT
   1335 74 FF              5131 	mov	a,#0xFF
   1337 F0                 5132 	movx	@dptr,a
                           5133 ;	../../include/ztex-fpga6.h:310: init_cpld_fpga_configuration();
                           5134 ;	../../include/ztex-conf.h:125: break;
   1338 02 0E 89           5135 	ljmp	_init_cpld_fpga_configuration
                           5136 ;	../../include/ztex-conf.h:123: case $0:			
   133B                    5137 00112$:
                           5138 ;	../../include/ztex-fpga6.h:319: IOE = bmBIT3 | bmBIT7;		
   133B 75 B1 88           5139 	mov	_IOE,#0x88
                           5140 ;	../../include/ztex-fpga6.h:320: OEC |= bmBIT6;               	// out: CCLK
   133E 43 B4 40           5141 	orl	_OEC,#0x40
                           5142 ;	../../include/ztex-fpga6.h:322: GPIFABORT = 0xFF;
   1341 90 E6 F5           5143 	mov	dptr,#_GPIFABORT
   1344 74 FF              5144 	mov	a,#0xFF
   1346 F0                 5145 	movx	@dptr,a
                           5146 ;	../../include/ezregs.h:46: __endasm;
                           5147 	
   1347 00                 5148 	 nop
   1348 00                 5149 	 nop
   1349 00                 5150 	 nop
   134A 00                 5151 	 nop
                           5152 	    
                           5153 ;	../../include/ztex-fpga6.h:324: IFCONFIG &= 0xf0;
   134B 90 E6 01           5154 	mov	dptr,#_IFCONFIG
   134E E0                 5155 	movx	a,@dptr
   134F FA                 5156 	mov	r2,a
   1350 54 F0              5157 	anl	a,#0xF0
   1352 F0                 5158 	movx	@dptr,a
                           5159 ;	../../include/ezregs.h:46: __endasm;
                           5160 	
   1353 00                 5161 	 nop
   1354 00                 5162 	 nop
   1355 00                 5163 	 nop
   1356 00                 5164 	 nop
                           5165 	    
                           5166 ;	../../include/ztex-fpga6.h:327: finish_fpga_configuration();
                           5167 ;	../../include/ztex-conf.h:125: break;
                           5168 ;	../../include/ztex-isr.h:105: default:
   1357 02 0D D9           5169 	ljmp	_finish_fpga_configuration
   135A                    5170 00113$:
                           5171 ;	../../include/ztex-isr.h:106: EP0CS |= 0x01;			// set stall, unknown request
   135A 90 E6 A0           5172 	mov	dptr,#_EP0CS
   135D E0                 5173 	movx	a,@dptr
   135E 44 01              5174 	orl	a,#0x01
   1360 F0                 5175 	movx	@dptr,a
                           5176 ;	../../include/ztex-isr.h:107: }
   1361 22                 5177 	ret
                           5178 ;------------------------------------------------------------
                           5179 ;Allocation info for local variables in function 'SUDAV_ISR'
                           5180 ;------------------------------------------------------------
                           5181 ;a                         Allocated to registers r2 
                           5182 ;------------------------------------------------------------
                           5183 ;	../../include/ztex-isr.h:113: static void SUDAV_ISR () __interrupt
                           5184 ;	-----------------------------------------
                           5185 ;	 function SUDAV_ISR
                           5186 ;	-----------------------------------------
   1362                    5187 _SUDAV_ISR:
   1362 C0 20              5188 	push	bits
   1364 C0 E0              5189 	push	acc
   1366 C0 F0              5190 	push	b
   1368 C0 82              5191 	push	dpl
   136A C0 83              5192 	push	dph
   136C C0 02              5193 	push	(0+2)
   136E C0 03              5194 	push	(0+3)
   1370 C0 04              5195 	push	(0+4)
   1372 C0 05              5196 	push	(0+5)
   1374 C0 06              5197 	push	(0+6)
   1376 C0 07              5198 	push	(0+7)
   1378 C0 00              5199 	push	(0+0)
   137A C0 01              5200 	push	(0+1)
   137C C0 D0              5201 	push	psw
   137E 75 D0 00           5202 	mov	psw,#0x00
                           5203 ;	../../include/ztex-isr.h:116: ep0_prev_setup_request = bRequest;
   1381 90 E6 B9           5204 	mov	dptr,#_bRequest
   1384 E0                 5205 	movx	a,@dptr
   1385 FA                 5206 	mov	r2,a
   1386 90 3A 79           5207 	mov	dptr,#_ep0_prev_setup_request
   1389 F0                 5208 	movx	@dptr,a
                           5209 ;	../../include/ztex-isr.h:117: SUDPTRCTL = 1;
   138A 90 E6 B5           5210 	mov	dptr,#_SUDPTRCTL
   138D 74 01              5211 	mov	a,#0x01
   138F F0                 5212 	movx	@dptr,a
                           5213 ;	../../include/ztex-isr.h:120: switch ( bRequest ) {
   1390 90 E6 B9           5214 	mov	dptr,#_bRequest
   1393 E0                 5215 	movx	a,@dptr
   1394 FA                 5216 	mov  r2,a
   1395 24 F3              5217 	add	a,#0xff - 0x0C
   1397 50 03              5218 	jnc	00240$
   1399 02 17 18           5219 	ljmp	00160$
   139C                    5220 00240$:
   139C EA                 5221 	mov	a,r2
   139D 2A                 5222 	add	a,r2
   139E 2A                 5223 	add	a,r2
   139F 90 13 A3           5224 	mov	dptr,#00241$
   13A2 73                 5225 	jmp	@a+dptr
   13A3                    5226 00241$:
   13A3 02 13 CA           5227 	ljmp	00101$
   13A6 02 14 7E           5228 	ljmp	00112$
   13A9 02 17 18           5229 	ljmp	00160$
   13AC 02 14 FB           5230 	ljmp	00122$
   13AF 02 17 18           5231 	ljmp	00160$
   13B2 02 17 18           5232 	ljmp	00160$
   13B5 02 15 93           5233 	ljmp	00132$
   13B8 02 16 AE           5234 	ljmp	00152$
   13BB 02 16 B0           5235 	ljmp	00153$
   13BE 02 16 C1           5236 	ljmp	00154$
   13C1 02 16 C6           5237 	ljmp	00155$
   13C4 02 16 D7           5238 	ljmp	00156$
   13C7 02 16 DC           5239 	ljmp	00157$
                           5240 ;	../../include/ztex-isr.h:121: case 0x00:	// get status 
   13CA                    5241 00101$:
                           5242 ;	../../include/ztex-isr.h:122: switch(SETUPDAT[0]) {
   13CA 90 E6 B8           5243 	mov	dptr,#_SETUPDAT
   13CD E0                 5244 	movx	a,@dptr
   13CE FA                 5245 	mov	r2,a
   13CF BA 80 02           5246 	cjne	r2,#0x80,00242$
   13D2 80 0D              5247 	sjmp	00102$
   13D4                    5248 00242$:
   13D4 BA 81 02           5249 	cjne	r2,#0x81,00243$
   13D7 80 1E              5250 	sjmp	00103$
   13D9                    5251 00243$:
   13D9 BA 82 02           5252 	cjne	r2,#0x82,00244$
   13DC 80 2F              5253 	sjmp	00104$
   13DE                    5254 00244$:
   13DE 02 17 18           5255 	ljmp	00160$
                           5256 ;	../../include/ztex-isr.h:123: case 0x80:  		// self powered and remote 
   13E1                    5257 00102$:
                           5258 ;	../../include/ztex-isr.h:124: EP0BUF[0] = 0;	// not self-powered, no remote wakeup
   13E1 90 E7 40           5259 	mov	dptr,#_EP0BUF
                           5260 ;	../../include/ztex-isr.h:125: EP0BUF[1] = 0;
                           5261 ;	../../include/ztex-isr.h:126: EP0BCH = 0;
   13E4 E4                 5262 	clr	a
   13E5 F0                 5263 	movx	@dptr,a
   13E6 90 E7 41           5264 	mov	dptr,#(_EP0BUF + 0x0001)
   13E9 F0                 5265 	movx	@dptr,a
   13EA 90 E6 8A           5266 	mov	dptr,#_EP0BCH
   13ED F0                 5267 	movx	@dptr,a
                           5268 ;	../../include/ztex-isr.h:127: EP0BCL = 2;
   13EE 90 E6 8B           5269 	mov	dptr,#_EP0BCL
   13F1 74 02              5270 	mov	a,#0x02
   13F3 F0                 5271 	movx	@dptr,a
                           5272 ;	../../include/ztex-isr.h:128: break;
   13F4 02 17 18           5273 	ljmp	00160$
                           5274 ;	../../include/ztex-isr.h:129: case 0x81:		// interface (reserved)
   13F7                    5275 00103$:
                           5276 ;	../../include/ztex-isr.h:130: EP0BUF[0] = 0; 	// always return zeros
   13F7 90 E7 40           5277 	mov	dptr,#_EP0BUF
                           5278 ;	../../include/ztex-isr.h:131: EP0BUF[1] = 0;
                           5279 ;	../../include/ztex-isr.h:132: EP0BCH = 0;
   13FA E4                 5280 	clr	a
   13FB F0                 5281 	movx	@dptr,a
   13FC 90 E7 41           5282 	mov	dptr,#(_EP0BUF + 0x0001)
   13FF F0                 5283 	movx	@dptr,a
   1400 90 E6 8A           5284 	mov	dptr,#_EP0BCH
   1403 F0                 5285 	movx	@dptr,a
                           5286 ;	../../include/ztex-isr.h:133: EP0BCL = 2;
   1404 90 E6 8B           5287 	mov	dptr,#_EP0BCL
   1407 74 02              5288 	mov	a,#0x02
   1409 F0                 5289 	movx	@dptr,a
                           5290 ;	../../include/ztex-isr.h:134: break;
   140A 02 17 18           5291 	ljmp	00160$
                           5292 ;	../../include/ztex-isr.h:135: case 0x82:	
   140D                    5293 00104$:
                           5294 ;	../../include/ztex-isr.h:136: switch ( SETUPDAT[4] ) {
   140D 90 E6 BC           5295 	mov	dptr,#(_SETUPDAT + 0x0004)
   1410 E0                 5296 	movx	a,@dptr
   1411 FA                 5297 	mov	r2,a
   1412 60 0F              5298 	jz	00106$
   1414 BA 01 02           5299 	cjne	r2,#0x01,00246$
   1417 80 19              5300 	sjmp	00107$
   1419                    5301 00246$:
   1419 BA 80 02           5302 	cjne	r2,#0x80,00247$
   141C 80 05              5303 	sjmp	00106$
   141E                    5304 00247$:
                           5305 ;	../../include/ztex-isr.h:138: case 0x80 :
   141E BA 81 2F           5306 	cjne	r2,#0x81,00109$
   1421 80 1E              5307 	sjmp	00108$
   1423                    5308 00106$:
                           5309 ;	../../include/ztex-isr.h:139: EP0BUF[0] = EP0CS & bmBIT0;
   1423 90 E6 A0           5310 	mov	dptr,#_EP0CS
   1426 E0                 5311 	movx	a,@dptr
   1427 FA                 5312 	mov	r2,a
   1428 53 02 01           5313 	anl	ar2,#0x01
   142B 90 E7 40           5314 	mov	dptr,#_EP0BUF
   142E EA                 5315 	mov	a,r2
   142F F0                 5316 	movx	@dptr,a
                           5317 ;	../../include/ztex-isr.h:140: break;
                           5318 ;	../../include/ztex-isr.h:141: case 0x01 :
   1430 80 3A              5319 	sjmp	00110$
   1432                    5320 00107$:
                           5321 ;	../../include/ztex-isr.h:142: EP0BUF[0] = EP1OUTCS & bmBIT0;
   1432 90 E6 A1           5322 	mov	dptr,#_EP1OUTCS
   1435 E0                 5323 	movx	a,@dptr
   1436 FA                 5324 	mov	r2,a
   1437 53 02 01           5325 	anl	ar2,#0x01
   143A 90 E7 40           5326 	mov	dptr,#_EP0BUF
   143D EA                 5327 	mov	a,r2
   143E F0                 5328 	movx	@dptr,a
                           5329 ;	../../include/ztex-isr.h:143: break;
                           5330 ;	../../include/ztex-isr.h:144: case 0x81 :
   143F 80 2B              5331 	sjmp	00110$
   1441                    5332 00108$:
                           5333 ;	../../include/ztex-isr.h:145: EP0BUF[0] = EP1INCS & bmBIT0;
   1441 90 E6 A2           5334 	mov	dptr,#_EP1INCS
   1444 E0                 5335 	movx	a,@dptr
   1445 FA                 5336 	mov	r2,a
   1446 53 02 01           5337 	anl	ar2,#0x01
   1449 90 E7 40           5338 	mov	dptr,#_EP0BUF
   144C EA                 5339 	mov	a,r2
   144D F0                 5340 	movx	@dptr,a
                           5341 ;	../../include/ztex-isr.h:146: break;
                           5342 ;	../../include/ztex-isr.h:147: default:
   144E 80 1C              5343 	sjmp	00110$
   1450                    5344 00109$:
                           5345 ;	../../include/ztex-isr.h:148: EP0BUF[0] = EPXCS[ ((SETUPDAT[4] >> 1)-1) & 3 ] & bmBIT0;
   1450 90 E6 BC           5346 	mov	dptr,#(_SETUPDAT + 0x0004)
   1453 E0                 5347 	movx	a,@dptr
   1454 C3                 5348 	clr	c
   1455 13                 5349 	rrc	a
   1456 14                 5350 	dec	a
   1457 54 03              5351 	anl	a,#0x03
   1459 24 A3              5352 	add	a,#_EPXCS
   145B F5 82              5353 	mov	dpl,a
   145D E4                 5354 	clr	a
   145E 34 E6              5355 	addc	a,#(_EPXCS >> 8)
   1460 F5 83              5356 	mov	dph,a
   1462 E0                 5357 	movx	a,@dptr
   1463 FA                 5358 	mov	r2,a
   1464 53 02 01           5359 	anl	ar2,#0x01
   1467 90 E7 40           5360 	mov	dptr,#_EP0BUF
   146A EA                 5361 	mov	a,r2
   146B F0                 5362 	movx	@dptr,a
                           5363 ;	../../include/ztex-isr.h:150: }
   146C                    5364 00110$:
                           5365 ;	../../include/ztex-isr.h:151: EP0BUF[1] = 0;
   146C 90 E7 41           5366 	mov	dptr,#(_EP0BUF + 0x0001)
                           5367 ;	../../include/ztex-isr.h:152: EP0BCH = 0;
   146F E4                 5368 	clr	a
   1470 F0                 5369 	movx	@dptr,a
   1471 90 E6 8A           5370 	mov	dptr,#_EP0BCH
   1474 F0                 5371 	movx	@dptr,a
                           5372 ;	../../include/ztex-isr.h:153: EP0BCL = 2;
   1475 90 E6 8B           5373 	mov	dptr,#_EP0BCL
   1478 74 02              5374 	mov	a,#0x02
   147A F0                 5375 	movx	@dptr,a
                           5376 ;	../../include/ztex-isr.h:156: break;
   147B 02 17 18           5377 	ljmp	00160$
                           5378 ;	../../include/ztex-isr.h:157: case 0x01:	// disable feature, e.g. remote wake, stall bit
   147E                    5379 00112$:
                           5380 ;	../../include/ztex-isr.h:158: if ( SETUPDAT[0] == 2 && SETUPDAT[2] == 0 ) {
   147E 90 E6 B8           5381 	mov	dptr,#_SETUPDAT
   1481 E0                 5382 	movx	a,@dptr
   1482 FA                 5383 	mov	r2,a
   1483 BA 02 02           5384 	cjne	r2,#0x02,00249$
   1486 80 03              5385 	sjmp	00250$
   1488                    5386 00249$:
   1488 02 17 18           5387 	ljmp	00160$
   148B                    5388 00250$:
   148B 90 E6 BA           5389 	mov	dptr,#(_SETUPDAT + 0x0002)
   148E E0                 5390 	movx	a,@dptr
   148F 60 03              5391 	jz	00251$
   1491 02 17 18           5392 	ljmp	00160$
   1494                    5393 00251$:
                           5394 ;	../../include/ztex-isr.h:159: switch ( SETUPDAT[4] ) {
   1494 90 E6 BC           5395 	mov	dptr,#(_SETUPDAT + 0x0004)
   1497 E0                 5396 	movx	a,@dptr
   1498 FA                 5397 	mov	r2,a
   1499 60 0F              5398 	jz	00114$
   149B BA 01 02           5399 	cjne	r2,#0x01,00253$
   149E 80 15              5400 	sjmp	00115$
   14A0                    5401 00253$:
   14A0 BA 80 02           5402 	cjne	r2,#0x80,00254$
   14A3 80 05              5403 	sjmp	00114$
   14A5                    5404 00254$:
                           5405 ;	../../include/ztex-isr.h:161: case 0x80 :
   14A5 BA 81 23           5406 	cjne	r2,#0x81,00117$
   14A8 80 16              5407 	sjmp	00116$
   14AA                    5408 00114$:
                           5409 ;	../../include/ztex-isr.h:162: EP0CS &= ~bmBIT0;
   14AA 90 E6 A0           5410 	mov	dptr,#_EP0CS
   14AD E0                 5411 	movx	a,@dptr
   14AE FA                 5412 	mov	r2,a
   14AF 54 FE              5413 	anl	a,#0xFE
   14B1 F0                 5414 	movx	@dptr,a
                           5415 ;	../../include/ztex-isr.h:163: break;
   14B2 02 17 18           5416 	ljmp	00160$
                           5417 ;	../../include/ztex-isr.h:164: case 0x01 :
   14B5                    5418 00115$:
                           5419 ;	../../include/ztex-isr.h:165: EP1OUTCS &= ~bmBIT0;
   14B5 90 E6 A1           5420 	mov	dptr,#_EP1OUTCS
   14B8 E0                 5421 	movx	a,@dptr
   14B9 FA                 5422 	mov	r2,a
   14BA 54 FE              5423 	anl	a,#0xFE
   14BC F0                 5424 	movx	@dptr,a
                           5425 ;	../../include/ztex-isr.h:166: break;
   14BD 02 17 18           5426 	ljmp	00160$
                           5427 ;	../../include/ztex-isr.h:167: case 0x81 :
   14C0                    5428 00116$:
                           5429 ;	../../include/ztex-isr.h:168: EP1INCS &= ~bmBIT0;
   14C0 90 E6 A2           5430 	mov	dptr,#_EP1INCS
   14C3 E0                 5431 	movx	a,@dptr
   14C4 FA                 5432 	mov	r2,a
   14C5 54 FE              5433 	anl	a,#0xFE
   14C7 F0                 5434 	movx	@dptr,a
                           5435 ;	../../include/ztex-isr.h:169: break;
   14C8 02 17 18           5436 	ljmp	00160$
                           5437 ;	../../include/ztex-isr.h:170: default:
   14CB                    5438 00117$:
                           5439 ;	../../include/ztex-isr.h:171: EPXCS[ ((SETUPDAT[4] >> 1)-1) & 3 ] &= ~bmBIT0;
   14CB 90 E6 BC           5440 	mov	dptr,#(_SETUPDAT + 0x0004)
   14CE E0                 5441 	movx	a,@dptr
   14CF C3                 5442 	clr	c
   14D0 13                 5443 	rrc	a
   14D1 14                 5444 	dec	a
   14D2 54 03              5445 	anl	a,#0x03
   14D4 24 A3              5446 	add	a,#_EPXCS
   14D6 FA                 5447 	mov	r2,a
   14D7 E4                 5448 	clr	a
   14D8 34 E6              5449 	addc	a,#(_EPXCS >> 8)
   14DA FB                 5450 	mov	r3,a
   14DB 90 E6 BC           5451 	mov	dptr,#(_SETUPDAT + 0x0004)
   14DE E0                 5452 	movx	a,@dptr
   14DF C3                 5453 	clr	c
   14E0 13                 5454 	rrc	a
   14E1 14                 5455 	dec	a
   14E2 54 03              5456 	anl	a,#0x03
   14E4 24 A3              5457 	add	a,#_EPXCS
   14E6 F5 82              5458 	mov	dpl,a
   14E8 E4                 5459 	clr	a
   14E9 34 E6              5460 	addc	a,#(_EPXCS >> 8)
   14EB F5 83              5461 	mov	dph,a
   14ED E0                 5462 	movx	a,@dptr
   14EE FC                 5463 	mov	r4,a
   14EF 53 04 FE           5464 	anl	ar4,#0xFE
   14F2 8A 82              5465 	mov	dpl,r2
   14F4 8B 83              5466 	mov	dph,r3
   14F6 EC                 5467 	mov	a,r4
   14F7 F0                 5468 	movx	@dptr,a
                           5469 ;	../../include/ztex-isr.h:175: break;
   14F8 02 17 18           5470 	ljmp	00160$
                           5471 ;	../../include/ztex-isr.h:176: case 0x03:      // enable feature, e.g. remote wake, test mode, stall bit
   14FB                    5472 00122$:
                           5473 ;	../../include/ztex-isr.h:177: if ( SETUPDAT[0] == 2 && SETUPDAT[2] == 0 ) {
   14FB 90 E6 B8           5474 	mov	dptr,#_SETUPDAT
   14FE E0                 5475 	movx	a,@dptr
   14FF FA                 5476 	mov	r2,a
   1500 BA 02 02           5477 	cjne	r2,#0x02,00256$
   1503 80 03              5478 	sjmp	00257$
   1505                    5479 00256$:
   1505 02 17 18           5480 	ljmp	00160$
   1508                    5481 00257$:
   1508 90 E6 BA           5482 	mov	dptr,#(_SETUPDAT + 0x0002)
   150B E0                 5483 	movx	a,@dptr
   150C 60 03              5484 	jz	00258$
   150E 02 17 18           5485 	ljmp	00160$
   1511                    5486 00258$:
                           5487 ;	../../include/ztex-isr.h:178: switch ( SETUPDAT[4] ) {
   1511 90 E6 BC           5488 	mov	dptr,#(_SETUPDAT + 0x0004)
   1514 E0                 5489 	movx	a,@dptr
   1515 FA                 5490 	mov	r2,a
   1516 60 0F              5491 	jz	00124$
   1518 BA 01 02           5492 	cjne	r2,#0x01,00260$
   151B 80 14              5493 	sjmp	00125$
   151D                    5494 00260$:
   151D BA 80 02           5495 	cjne	r2,#0x80,00261$
   1520 80 05              5496 	sjmp	00124$
   1522                    5497 00261$:
                           5498 ;	../../include/ztex-isr.h:180: case 0x80 :
   1522 BA 81 20           5499 	cjne	r2,#0x81,00127$
   1525 80 14              5500 	sjmp	00126$
   1527                    5501 00124$:
                           5502 ;	../../include/ztex-isr.h:181: EP0CS |= bmBIT0;
   1527 90 E6 A0           5503 	mov	dptr,#_EP0CS
   152A E0                 5504 	movx	a,@dptr
   152B FA                 5505 	mov	r2,a
   152C 44 01              5506 	orl	a,#0x01
   152E F0                 5507 	movx	@dptr,a
                           5508 ;	../../include/ztex-isr.h:182: break;
                           5509 ;	../../include/ztex-isr.h:183: case 0x01 :
   152F 80 41              5510 	sjmp	00128$
   1531                    5511 00125$:
                           5512 ;	../../include/ztex-isr.h:184: EP1OUTCS |= bmBIT0;
   1531 90 E6 A1           5513 	mov	dptr,#_EP1OUTCS
   1534 E0                 5514 	movx	a,@dptr
   1535 FA                 5515 	mov	r2,a
   1536 44 01              5516 	orl	a,#0x01
   1538 F0                 5517 	movx	@dptr,a
                           5518 ;	../../include/ztex-isr.h:185: break;
                           5519 ;	../../include/ztex-isr.h:186: case 0x81 :
   1539 80 37              5520 	sjmp	00128$
   153B                    5521 00126$:
                           5522 ;	../../include/ztex-isr.h:187: EP1INCS |= bmBIT0;
   153B 90 E6 A2           5523 	mov	dptr,#_EP1INCS
   153E E0                 5524 	movx	a,@dptr
   153F FA                 5525 	mov	r2,a
   1540 44 01              5526 	orl	a,#0x01
   1542 F0                 5527 	movx	@dptr,a
                           5528 ;	../../include/ztex-isr.h:188: break;
                           5529 ;	../../include/ztex-isr.h:189: default:
   1543 80 2D              5530 	sjmp	00128$
   1545                    5531 00127$:
                           5532 ;	../../include/ztex-isr.h:190: EPXCS[ ((SETUPDAT[4] >> 1)-1) & 3 ] |= ~bmBIT0;
   1545 90 E6 BC           5533 	mov	dptr,#(_SETUPDAT + 0x0004)
   1548 E0                 5534 	movx	a,@dptr
   1549 C3                 5535 	clr	c
   154A 13                 5536 	rrc	a
   154B 14                 5537 	dec	a
   154C 54 03              5538 	anl	a,#0x03
   154E 24 A3              5539 	add	a,#_EPXCS
   1550 FA                 5540 	mov	r2,a
   1551 E4                 5541 	clr	a
   1552 34 E6              5542 	addc	a,#(_EPXCS >> 8)
   1554 FB                 5543 	mov	r3,a
   1555 90 E6 BC           5544 	mov	dptr,#(_SETUPDAT + 0x0004)
   1558 E0                 5545 	movx	a,@dptr
   1559 C3                 5546 	clr	c
   155A 13                 5547 	rrc	a
   155B 14                 5548 	dec	a
   155C 54 03              5549 	anl	a,#0x03
   155E 24 A3              5550 	add	a,#_EPXCS
   1560 F5 82              5551 	mov	dpl,a
   1562 E4                 5552 	clr	a
   1563 34 E6              5553 	addc	a,#(_EPXCS >> 8)
   1565 F5 83              5554 	mov	dph,a
   1567 E0                 5555 	movx	a,@dptr
   1568 FC                 5556 	mov	r4,a
   1569 43 04 FE           5557 	orl	ar4,#0xFE
   156C 8A 82              5558 	mov	dpl,r2
   156E 8B 83              5559 	mov	dph,r3
   1570 EC                 5560 	mov	a,r4
   1571 F0                 5561 	movx	@dptr,a
                           5562 ;	../../include/ztex-isr.h:192: }
   1572                    5563 00128$:
                           5564 ;	../../include/ztex-isr.h:193: a = ( (SETUPDAT[4] & 0x80) >> 3 ) | (SETUPDAT[4] & 0x0f);
   1572 90 E6 BC           5565 	mov	dptr,#(_SETUPDAT + 0x0004)
   1575 E0                 5566 	movx	a,@dptr
   1576 54 80              5567 	anl	a,#0x80
   1578 C4                 5568 	swap	a
   1579 23                 5569 	rl	a
   157A 54 1F              5570 	anl	a,#0x1f
   157C FA                 5571 	mov	r2,a
   157D 90 E6 BC           5572 	mov	dptr,#(_SETUPDAT + 0x0004)
   1580 E0                 5573 	movx	a,@dptr
   1581 FB                 5574 	mov	r3,a
   1582 74 0F              5575 	mov	a,#0x0F
   1584 5B                 5576 	anl	a,r3
   1585 42 02              5577 	orl	ar2,a
                           5578 ;	../../include/ztex-isr.h:194: TOGCTL = a;
                           5579 ;	../../include/ztex-isr.h:195: TOGCTL = a | bmBIT5;
   1587 90 E6 83           5580 	mov	dptr,#_TOGCTL
   158A EA                 5581 	mov	a,r2
   158B F0                 5582 	movx	@dptr,a
   158C 74 20              5583 	mov	a,#0x20
   158E 4A                 5584 	orl	a,r2
   158F F0                 5585 	movx	@dptr,a
                           5586 ;	../../include/ztex-isr.h:197: break;
   1590 02 17 18           5587 	ljmp	00160$
                           5588 ;	../../include/ztex-isr.h:198: case 0x06:			// get descriptor
   1593                    5589 00132$:
                           5590 ;	../../include/ztex-isr.h:199: switch(SETUPDAT[3]) {
   1593 90 E6 BB           5591 	mov	dptr,#(_SETUPDAT + 0x0003)
   1596 E0                 5592 	movx	a,@dptr
   1597 FA                 5593 	mov	r2,a
   1598 BA 01 02           5594 	cjne	r2,#0x01,00263$
   159B 80 19              5595 	sjmp	00133$
   159D                    5596 00263$:
   159D BA 02 02           5597 	cjne	r2,#0x02,00264$
   15A0 80 26              5598 	sjmp	00134$
   15A2                    5599 00264$:
   15A2 BA 03 02           5600 	cjne	r2,#0x03,00265$
   15A5 80 4D              5601 	sjmp	00138$
   15A7                    5602 00265$:
   15A7 BA 06 03           5603 	cjne	r2,#0x06,00266$
   15AA 02 16 67           5604 	ljmp	00145$
   15AD                    5605 00266$:
   15AD BA 07 03           5606 	cjne	r2,#0x07,00267$
   15B0 02 16 79           5607 	ljmp	00146$
   15B3                    5608 00267$:
   15B3 02 16 A4           5609 	ljmp	00150$
                           5610 ;	../../include/ztex-isr.h:200: case 0x01:		// device
   15B6                    5611 00133$:
                           5612 ;	../../include/ztex-isr.h:201: SUDPTRH = MSB(&DeviceDescriptor);
   15B6 7A 76              5613 	mov	r2,#_DeviceDescriptor
   15B8 7B 1F              5614 	mov	r3,#(_DeviceDescriptor >> 8)
   15BA 90 E6 B3           5615 	mov	dptr,#_SUDPTRH
   15BD EB                 5616 	mov	a,r3
   15BE F0                 5617 	movx	@dptr,a
                           5618 ;	../../include/ztex-isr.h:202: SUDPTRL = LSB(&DeviceDescriptor);
   15BF 90 E6 B4           5619 	mov	dptr,#_SUDPTRL
   15C2 74 76              5620 	mov	a,#_DeviceDescriptor
   15C4 F0                 5621 	movx	@dptr,a
                           5622 ;	../../include/ztex-isr.h:203: break;
   15C5 02 17 18           5623 	ljmp	00160$
                           5624 ;	../../include/ztex-isr.h:204: case 0x02: 		// configuration
   15C8                    5625 00134$:
                           5626 ;	../../include/ztex-isr.h:205: if (USBCS & bmBIT7) {
   15C8 90 E6 80           5627 	mov	dptr,#_USBCS
   15CB E0                 5628 	movx	a,@dptr
   15CC FA                 5629 	mov	r2,a
   15CD 30 E7 12           5630 	jnb	acc.7,00136$
                           5631 ;	../../include/ztex-isr.h:206: SUDPTRH = MSB(&HighSpeedConfigDescriptor);
   15D0 7A 92              5632 	mov	r2,#_HighSpeedConfigDescriptor
   15D2 7B 1F              5633 	mov	r3,#(_HighSpeedConfigDescriptor >> 8)
   15D4 90 E6 B3           5634 	mov	dptr,#_SUDPTRH
   15D7 EB                 5635 	mov	a,r3
   15D8 F0                 5636 	movx	@dptr,a
                           5637 ;	../../include/ztex-isr.h:207: SUDPTRL = LSB(&HighSpeedConfigDescriptor);
   15D9 90 E6 B4           5638 	mov	dptr,#_SUDPTRL
   15DC 74 92              5639 	mov	a,#_HighSpeedConfigDescriptor
   15DE F0                 5640 	movx	@dptr,a
   15DF 02 17 18           5641 	ljmp	00160$
   15E2                    5642 00136$:
                           5643 ;	../../include/ztex-isr.h:210: SUDPTRH = MSB(&FullSpeedConfigDescriptor);
   15E2 7A BA              5644 	mov	r2,#_FullSpeedConfigDescriptor
   15E4 7B 1F              5645 	mov	r3,#(_FullSpeedConfigDescriptor >> 8)
   15E6 90 E6 B3           5646 	mov	dptr,#_SUDPTRH
   15E9 EB                 5647 	mov	a,r3
   15EA F0                 5648 	movx	@dptr,a
                           5649 ;	../../include/ztex-isr.h:211: SUDPTRL = LSB(&FullSpeedConfigDescriptor);
   15EB 90 E6 B4           5650 	mov	dptr,#_SUDPTRL
   15EE 74 BA              5651 	mov	a,#_FullSpeedConfigDescriptor
   15F0 F0                 5652 	movx	@dptr,a
                           5653 ;	../../include/ztex-isr.h:213: break; 
   15F1 02 17 18           5654 	ljmp	00160$
                           5655 ;	../../include/ztex-isr.h:214: case 0x03:		// strings
   15F4                    5656 00138$:
                           5657 ;	../../include/ztex-isr.h:215: switch (SETUPDAT[2]) {
   15F4 90 E6 BA           5658 	mov	dptr,#(_SETUPDAT + 0x0002)
   15F7 E0                 5659 	movx	a,@dptr
   15F8 FA                 5660 	mov	r2,a
   15F9 BA 01 02           5661 	cjne	r2,#0x01,00269$
   15FC 80 0F              5662 	sjmp	00139$
   15FE                    5663 00269$:
   15FE BA 02 02           5664 	cjne	r2,#0x02,00270$
   1601 80 1C              5665 	sjmp	00140$
   1603                    5666 00270$:
   1603 BA 03 02           5667 	cjne	r2,#0x03,00271$
   1606 80 29              5668 	sjmp	00141$
   1608                    5669 00271$:
                           5670 ;	../../include/ztex-isr.h:216: case 1:
   1608 BA 04 4A           5671 	cjne	r2,#0x04,00143$
   160B 80 36              5672 	sjmp	00142$
   160D                    5673 00139$:
                           5674 ;	../../include/ztex-isr.h:217: SEND_STRING_DESCRIPTOR(manufacturerString);
   160D 75 82 48           5675 	mov	dpl,#_manufacturerString
   1610 7A 48              5676 	mov	r2,#_manufacturerString
   1612 7B 1F              5677 	mov	r3,#(_manufacturerString >> 8)
   1614 8B 16              5678 	mov	_sendStringDescriptor_PARM_2,r3
   1616 75 17 05           5679 	mov	_sendStringDescriptor_PARM_3,#0x05
   1619 12 11 EF           5680 	lcall	_sendStringDescriptor
                           5681 ;	../../include/ztex-isr.h:218: break;
   161C 02 17 18           5682 	ljmp	00160$
                           5683 ;	../../include/ztex-isr.h:219: case 2:
   161F                    5684 00140$:
                           5685 ;	../../include/ztex-isr.h:220: SEND_STRING_DESCRIPTOR(productString);
   161F 75 82 4D           5686 	mov	dpl,#_productString
   1622 7A 4D              5687 	mov	r2,#_productString
   1624 7B 1F              5688 	mov	r3,#(_productString >> 8)
   1626 8B 16              5689 	mov	_sendStringDescriptor_PARM_2,r3
   1628 75 17 20           5690 	mov	_sendStringDescriptor_PARM_3,#0x20
   162B 12 11 EF           5691 	lcall	_sendStringDescriptor
                           5692 ;	../../include/ztex-isr.h:221: break;
   162E 02 17 18           5693 	ljmp	00160$
                           5694 ;	../../include/ztex-isr.h:222: case 3:
   1631                    5695 00141$:
                           5696 ;	../../include/ztex-isr.h:223: SEND_STRING_DESCRIPTOR(SN_STRING);
   1631 75 82 8A           5697 	mov	dpl,#_SN_STRING
   1634 7A 8A              5698 	mov	r2,#_SN_STRING
   1636 7B 00              5699 	mov	r3,#(_SN_STRING >> 8)
   1638 8B 16              5700 	mov	_sendStringDescriptor_PARM_2,r3
   163A 75 17 0A           5701 	mov	_sendStringDescriptor_PARM_3,#0x0A
   163D 12 11 EF           5702 	lcall	_sendStringDescriptor
                           5703 ;	../../include/ztex-isr.h:224: break;
   1640 02 17 18           5704 	ljmp	00160$
                           5705 ;	../../include/ztex-isr.h:225: case 4:
   1643                    5706 00142$:
                           5707 ;	../../include/ztex-isr.h:226: SEND_STRING_DESCRIPTOR(configurationString);
   1643 75 82 6D           5708 	mov	dpl,#_configurationString
   1646 7A 6D              5709 	mov	r2,#_configurationString
   1648 7B 1F              5710 	mov	r3,#(_configurationString >> 8)
   164A 8B 16              5711 	mov	_sendStringDescriptor_PARM_2,r3
   164C 75 17 08           5712 	mov	_sendStringDescriptor_PARM_3,#0x08
   164F 12 11 EF           5713 	lcall	_sendStringDescriptor
                           5714 ;	../../include/ztex-isr.h:227: break; 
   1652 02 17 18           5715 	ljmp	00160$
                           5716 ;	../../include/ztex-isr.h:228: default:
   1655                    5717 00143$:
                           5718 ;	../../include/ztex-isr.h:229: SUDPTRH = MSB(&EmptyStringDescriptor);
   1655 7A E2              5719 	mov	r2,#_EmptyStringDescriptor
   1657 7B 1F              5720 	mov	r3,#(_EmptyStringDescriptor >> 8)
   1659 90 E6 B3           5721 	mov	dptr,#_SUDPTRH
   165C EB                 5722 	mov	a,r3
   165D F0                 5723 	movx	@dptr,a
                           5724 ;	../../include/ztex-isr.h:230: SUDPTRL = LSB(&EmptyStringDescriptor);
   165E 90 E6 B4           5725 	mov	dptr,#_SUDPTRL
   1661 74 E2              5726 	mov	a,#_EmptyStringDescriptor
   1663 F0                 5727 	movx	@dptr,a
                           5728 ;	../../include/ztex-isr.h:233: break;
   1664 02 17 18           5729 	ljmp	00160$
                           5730 ;	../../include/ztex-isr.h:234: case 0x06:		// device qualifier
   1667                    5731 00145$:
                           5732 ;	../../include/ztex-isr.h:235: SUDPTRH = MSB(&DeviceQualifierDescriptor);
   1667 7A 88              5733 	mov	r2,#_DeviceQualifierDescriptor
   1669 7B 1F              5734 	mov	r3,#(_DeviceQualifierDescriptor >> 8)
   166B 90 E6 B3           5735 	mov	dptr,#_SUDPTRH
   166E EB                 5736 	mov	a,r3
   166F F0                 5737 	movx	@dptr,a
                           5738 ;	../../include/ztex-isr.h:236: SUDPTRL = LSB(&DeviceQualifierDescriptor);
   1670 90 E6 B4           5739 	mov	dptr,#_SUDPTRL
   1673 74 88              5740 	mov	a,#_DeviceQualifierDescriptor
   1675 F0                 5741 	movx	@dptr,a
                           5742 ;	../../include/ztex-isr.h:237: break;
   1676 02 17 18           5743 	ljmp	00160$
                           5744 ;	../../include/ztex-isr.h:238: case 0x07: 		// other speed configuration
   1679                    5745 00146$:
                           5746 ;	../../include/ztex-isr.h:239: if (USBCS & bmBIT7) {
   1679 90 E6 80           5747 	mov	dptr,#_USBCS
   167C E0                 5748 	movx	a,@dptr
   167D FA                 5749 	mov	r2,a
   167E 30 E7 12           5750 	jnb	acc.7,00148$
                           5751 ;	../../include/ztex-isr.h:240: SUDPTRH = MSB(&FullSpeedConfigDescriptor);
   1681 7A BA              5752 	mov	r2,#_FullSpeedConfigDescriptor
   1683 7B 1F              5753 	mov	r3,#(_FullSpeedConfigDescriptor >> 8)
   1685 90 E6 B3           5754 	mov	dptr,#_SUDPTRH
   1688 EB                 5755 	mov	a,r3
   1689 F0                 5756 	movx	@dptr,a
                           5757 ;	../../include/ztex-isr.h:241: SUDPTRL = LSB(&FullSpeedConfigDescriptor);
   168A 90 E6 B4           5758 	mov	dptr,#_SUDPTRL
   168D 74 BA              5759 	mov	a,#_FullSpeedConfigDescriptor
   168F F0                 5760 	movx	@dptr,a
   1690 02 17 18           5761 	ljmp	00160$
   1693                    5762 00148$:
                           5763 ;	../../include/ztex-isr.h:244: SUDPTRH = MSB(&HighSpeedConfigDescriptor);
   1693 7A 92              5764 	mov	r2,#_HighSpeedConfigDescriptor
   1695 7B 1F              5765 	mov	r3,#(_HighSpeedConfigDescriptor >> 8)
   1697 90 E6 B3           5766 	mov	dptr,#_SUDPTRH
   169A EB                 5767 	mov	a,r3
   169B F0                 5768 	movx	@dptr,a
                           5769 ;	../../include/ztex-isr.h:245: SUDPTRL = LSB(&HighSpeedConfigDescriptor);
   169C 90 E6 B4           5770 	mov	dptr,#_SUDPTRL
   169F 74 92              5771 	mov	a,#_HighSpeedConfigDescriptor
   16A1 F0                 5772 	movx	@dptr,a
                           5773 ;	../../include/ztex-isr.h:247: break; 
                           5774 ;	../../include/ztex-isr.h:248: default:
   16A2 80 74              5775 	sjmp	00160$
   16A4                    5776 00150$:
                           5777 ;	../../include/ztex-isr.h:249: EP0CS |= 0x01;	// set stall, unknown descriptor
   16A4 90 E6 A0           5778 	mov	dptr,#_EP0CS
   16A7 E0                 5779 	movx	a,@dptr
   16A8 FA                 5780 	mov	r2,a
   16A9 44 01              5781 	orl	a,#0x01
   16AB F0                 5782 	movx	@dptr,a
                           5783 ;	../../include/ztex-isr.h:251: break;
                           5784 ;	../../include/ztex-isr.h:252: case 0x07:			// set descriptor
   16AC 80 6A              5785 	sjmp	00160$
   16AE                    5786 00152$:
                           5787 ;	../../include/ztex-isr.h:253: break;			
                           5788 ;	../../include/ztex-isr.h:254: case 0x08:			// get configuration
   16AE 80 68              5789 	sjmp	00160$
   16B0                    5790 00153$:
                           5791 ;	../../include/ztex-isr.h:255: EP0BUF[0] = 0;		// only one configuration
   16B0 90 E7 40           5792 	mov	dptr,#_EP0BUF
                           5793 ;	../../include/ztex-isr.h:256: EP0BCH = 0;
   16B3 E4                 5794 	clr	a
   16B4 F0                 5795 	movx	@dptr,a
   16B5 90 E6 8A           5796 	mov	dptr,#_EP0BCH
   16B8 F0                 5797 	movx	@dptr,a
                           5798 ;	../../include/ztex-isr.h:257: EP0BCL = 1;
   16B9 90 E6 8B           5799 	mov	dptr,#_EP0BCL
   16BC 74 01              5800 	mov	a,#0x01
   16BE F0                 5801 	movx	@dptr,a
                           5802 ;	../../include/ztex-isr.h:258: break;
                           5803 ;	../../include/ztex-isr.h:259: case 0x09:			// set configuration
   16BF 80 57              5804 	sjmp	00160$
   16C1                    5805 00154$:
                           5806 ;	../../include/ztex-isr.h:260: resetToggleData();
   16C1 12 11 C8           5807 	lcall	_resetToggleData
                           5808 ;	../../include/ztex-isr.h:261: break;			// do nothing since we have only one configuration
                           5809 ;	../../include/ztex-isr.h:262: case 0x0a:			// get alternate setting for an interface
   16C4 80 52              5810 	sjmp	00160$
   16C6                    5811 00155$:
                           5812 ;	../../include/ztex-isr.h:263: EP0BUF[0] = 0;		// only one alternate setting
   16C6 90 E7 40           5813 	mov	dptr,#_EP0BUF
                           5814 ;	../../include/ztex-isr.h:264: EP0BCH = 0;
   16C9 E4                 5815 	clr	a
   16CA F0                 5816 	movx	@dptr,a
   16CB 90 E6 8A           5817 	mov	dptr,#_EP0BCH
   16CE F0                 5818 	movx	@dptr,a
                           5819 ;	../../include/ztex-isr.h:265: EP0BCL = 1;
   16CF 90 E6 8B           5820 	mov	dptr,#_EP0BCL
   16D2 74 01              5821 	mov	a,#0x01
   16D4 F0                 5822 	movx	@dptr,a
                           5823 ;	../../include/ztex-isr.h:266: break;
                           5824 ;	../../include/ztex-isr.h:267: case 0x0b:			// set alternate setting for an interface
   16D5 80 41              5825 	sjmp	00160$
   16D7                    5826 00156$:
                           5827 ;	../../include/ztex-isr.h:268: resetToggleData();
   16D7 12 11 C8           5828 	lcall	_resetToggleData
                           5829 ;	../../include/ztex-isr.h:269: break;			// do nothing since we have only on alternate setting
                           5830 ;	../../include/ztex-isr.h:270: case 0x0c:			// sync frame
   16DA 80 3C              5831 	sjmp	00160$
   16DC                    5832 00157$:
                           5833 ;	../../include/ztex-isr.h:271: if ( SETUPDAT[0] == 0x82 ) {
   16DC 90 E6 B8           5834 	mov	dptr,#_SETUPDAT
   16DF E0                 5835 	movx	a,@dptr
   16E0 FA                 5836 	mov	r2,a
   16E1 BA 82 34           5837 	cjne	r2,#0x82,00160$
                           5838 ;	../../include/ztex-isr.h:272: ISOFRAME_COUNTER[ ((SETUPDAT[4] >> 1)-1) & 3 ] = 0;
   16E4 90 E6 BC           5839 	mov	dptr,#(_SETUPDAT + 0x0004)
   16E7 E0                 5840 	movx	a,@dptr
   16E8 C3                 5841 	clr	c
   16E9 13                 5842 	rrc	a
   16EA 14                 5843 	dec	a
   16EB 54 03              5844 	anl	a,#0x03
   16ED 25 E0              5845 	add	a,acc
   16EF 24 7B              5846 	add	a,#_ISOFRAME_COUNTER
   16F1 F5 82              5847 	mov	dpl,a
   16F3 E4                 5848 	clr	a
   16F4 34 3A              5849 	addc	a,#(_ISOFRAME_COUNTER >> 8)
   16F6 F5 83              5850 	mov	dph,a
   16F8 E4                 5851 	clr	a
   16F9 F0                 5852 	movx	@dptr,a
   16FA A3                 5853 	inc	dptr
   16FB F0                 5854 	movx	@dptr,a
                           5855 ;	../../include/ztex-isr.h:273: EP0BUF[0] = USBFRAMEL;	// use current frame as sync frame, i hope that works
   16FC 90 E6 85           5856 	mov	dptr,#_USBFRAMEL
   16FF E0                 5857 	movx	a,@dptr
   1700 90 E7 40           5858 	mov	dptr,#_EP0BUF
   1703 F0                 5859 	movx	@dptr,a
                           5860 ;	../../include/ztex-isr.h:274: EP0BUF[1] = USBFRAMEH;	
   1704 90 E6 84           5861 	mov	dptr,#_USBFRAMEH
   1707 E0                 5862 	movx	a,@dptr
   1708 FA                 5863 	mov	r2,a
   1709 90 E7 41           5864 	mov	dptr,#(_EP0BUF + 0x0001)
   170C F0                 5865 	movx	@dptr,a
                           5866 ;	../../include/ztex-isr.h:275: EP0BCH = 0;
   170D 90 E6 8A           5867 	mov	dptr,#_EP0BCH
   1710 E4                 5868 	clr	a
   1711 F0                 5869 	movx	@dptr,a
                           5870 ;	../../include/ztex-isr.h:276: EP0BCL = 2;
   1712 90 E6 8B           5871 	mov	dptr,#_EP0BCL
   1715 74 02              5872 	mov	a,#0x02
   1717 F0                 5873 	movx	@dptr,a
                           5874 ;	../../include/ztex-isr.h:280: }
   1718                    5875 00160$:
                           5876 ;	../../include/ztex-isr.h:283: switch ( bmRequestType ) {
   1718 90 E6 B8           5877 	mov	dptr,#_bmRequestType
   171B E0                 5878 	movx	a,@dptr
   171C FA                 5879 	mov	r2,a
   171D BA 40 03           5880 	cjne	r2,#0x40,00276$
   1720 02 19 56           5881 	ljmp	00183$
   1723                    5882 00276$:
   1723 BA C0 02           5883 	cjne	r2,#0xC0,00277$
   1726 80 03              5884 	sjmp	00278$
   1728                    5885 00277$:
   1728 02 19 80           5886 	ljmp	00187$
   172B                    5887 00278$:
                           5888 ;	../../include/ztex-isr.h:285: ep0_payload_remaining = (SETUPDAT[7] << 8) | SETUPDAT[6];
   172B 90 E6 BF           5889 	mov	dptr,#(_SETUPDAT + 0x0007)
   172E E0                 5890 	movx	a,@dptr
   172F FB                 5891 	mov	r3,a
   1730 7A 00              5892 	mov	r2,#0x00
   1732 90 E6 BE           5893 	mov	dptr,#(_SETUPDAT + 0x0006)
   1735 E0                 5894 	movx	a,@dptr
   1736 FC                 5895 	mov	r4,a
   1737 7D 00              5896 	mov	r5,#0x00
   1739 90 3A 36           5897 	mov	dptr,#_ep0_payload_remaining
   173C EC                 5898 	mov	a,r4
   173D 4A                 5899 	orl	a,r2
   173E F0                 5900 	movx	@dptr,a
   173F ED                 5901 	mov	a,r5
   1740 4B                 5902 	orl	a,r3
   1741 A3                 5903 	inc	dptr
   1742 F0                 5904 	movx	@dptr,a
                           5905 ;	../../include/ztex-isr.h:286: ep0_payload_update();
   1743 12 12 58           5906 	lcall	_ep0_payload_update
                           5907 ;	../../include/ztex-isr.h:288: switch ( bRequest ) {
   1746 90 E6 B9           5908 	mov	dptr,#_bRequest
   1749 E0                 5909 	movx	a,@dptr
   174A FA                 5910 	mov	r2,a
   174B BA 22 02           5911 	cjne	r2,#0x22,00279$
   174E 80 37              5912 	sjmp	00162$
   1750                    5913 00279$:
   1750 BA 30 03           5914 	cjne	r2,#0x30,00280$
   1753 02 18 EF           5915 	ljmp	00176$
   1756                    5916 00280$:
   1756 BA 33 03           5917 	cjne	r2,#0x33,00281$
   1759 02 19 35           5918 	ljmp	00180$
   175C                    5919 00281$:
   175C BA 38 02           5920 	cjne	r2,#0x38,00282$
   175F 80 43              5921 	sjmp	00163$
   1761                    5922 00282$:
   1761 BA 3A 02           5923 	cjne	r2,#0x3A,00283$
   1764 80 67              5924 	sjmp	00164$
   1766                    5925 00283$:
   1766 BA 3B 03           5926 	cjne	r2,#0x3B,00284$
   1769 02 18 0C           5927 	ljmp	00165$
   176C                    5928 00284$:
   176C BA 3D 03           5929 	cjne	r2,#0x3D,00285$
   176F 02 18 25           5930 	ljmp	00166$
   1772                    5931 00285$:
   1772 BA 40 03           5932 	cjne	r2,#0x40,00286$
   1775 02 18 46           5933 	ljmp	00167$
   1778                    5934 00286$:
   1778 BA 41 03           5935 	cjne	r2,#0x41,00287$
   177B 02 18 7C           5936 	ljmp	00171$
   177E                    5937 00287$:
   177E BA 43 03           5938 	cjne	r2,#0x43,00288$
   1781 02 18 C9           5939 	ljmp	00175$
   1784                    5940 00288$:
   1784 02 19 4C           5941 	ljmp	00181$
                           5942 ;	../../include/ztex-isr.h:289: case 0x22: 				// get ZTEX descriptor
   1787                    5943 00162$:
                           5944 ;	../../include/ztex-isr.h:290: SUDPTRCTL = 0;
   1787 90 E6 B5           5945 	mov	dptr,#_SUDPTRCTL
                           5946 ;	../../include/ztex-isr.h:291: EP0BCH = 0;
   178A E4                 5947 	clr	a
   178B F0                 5948 	movx	@dptr,a
   178C 90 E6 8A           5949 	mov	dptr,#_EP0BCH
   178F F0                 5950 	movx	@dptr,a
                           5951 ;	../../include/ztex-isr.h:292: EP0BCL = ZTEX_DESCRIPTOR_LEN;
   1790 90 E6 8B           5952 	mov	dptr,#_EP0BCL
   1793 74 28              5953 	mov	a,#0x28
   1795 F0                 5954 	movx	@dptr,a
                           5955 ;	../../include/ztex-isr.h:293: SUDPTRH = MSB(ZTEX_DESCRIPTOR_OFFS);
   1796 90 E6 B3           5956 	mov	dptr,#_SUDPTRH
   1799 E4                 5957 	clr	a
   179A F0                 5958 	movx	@dptr,a
                           5959 ;	../../include/ztex-isr.h:294: SUDPTRL = LSB(ZTEX_DESCRIPTOR_OFFS); 
   179B 90 E6 B4           5960 	mov	dptr,#_SUDPTRL
   179E 74 6C              5961 	mov	a,#0x6C
   17A0 F0                 5962 	movx	@dptr,a
                           5963 ;	../../include/ztex-isr.h:295: break;
   17A1 02 19 80           5964 	ljmp	00187$
                           5965 ;	../../include/ztex-conf.h:100: case $0:
   17A4                    5966 00163$:
                           5967 ;	../../include/ztex-conf.h:102: break;
   17A4 90 E6 BB           5968 	mov	dptr,#(_SETUPDAT + 0x0003)
   17A7 E0                 5969 	movx	a,@dptr
   17A8 FB                 5970 	mov	r3,a
   17A9 7A 00              5971 	mov	r2,#0x00
   17AB 90 E6 BA           5972 	mov	dptr,#(_SETUPDAT + 0x0002)
   17AE E0                 5973 	movx	a,@dptr
   17AF FC                 5974 	mov	r4,a
   17B0 7D 00              5975 	mov	r5,#0x00
   17B2 90 3A 00           5976 	mov	dptr,#_eeprom_addr
   17B5 EC                 5977 	mov	a,r4
   17B6 4A                 5978 	orl	a,r2
   17B7 F0                 5979 	movx	@dptr,a
   17B8 ED                 5980 	mov	a,r5
   17B9 4B                 5981 	orl	a,r3
   17BA A3                 5982 	inc	dptr
   17BB F0                 5983 	movx	@dptr,a
                           5984 ;	../../include/ztex-eeprom.h:219: EP0BCH = 0;
   17BC 90 E6 8A           5985 	mov	dptr,#_EP0BCH
   17BF E4                 5986 	clr	a
   17C0 F0                 5987 	movx	@dptr,a
                           5988 ;	../../include/ztex-eeprom.h:220: EP0BCL = eeprom_read_ep0(); 
   17C1 12 05 73           5989 	lcall	_eeprom_read_ep0
   17C4 E5 82              5990 	mov	a,dpl
   17C6 90 E6 8B           5991 	mov	dptr,#_EP0BCL
   17C9 F0                 5992 	movx	@dptr,a
                           5993 ;	../../include/ztex-conf.h:102: break;
   17CA 02 19 80           5994 	ljmp	00187$
                           5995 ;	../../include/ztex-conf.h:100: case $0:
   17CD                    5996 00164$:
                           5997 ;	../../include/ztex-eeprom.h:247: EP0BUF[0] = LSB(eeprom_write_bytes);
   17CD 90 3A 02           5998 	mov	dptr,#_eeprom_write_bytes
   17D0 E0                 5999 	movx	a,@dptr
   17D1 FA                 6000 	mov	r2,a
   17D2 A3                 6001 	inc	dptr
   17D3 E0                 6002 	movx	a,@dptr
   17D4 FB                 6003 	mov	r3,a
   17D5 8A 04              6004 	mov	ar4,r2
   17D7 90 E7 40           6005 	mov	dptr,#_EP0BUF
   17DA EC                 6006 	mov	a,r4
   17DB F0                 6007 	movx	@dptr,a
                           6008 ;	../../include/ztex-eeprom.h:248: EP0BUF[1] = MSB(eeprom_write_bytes);
   17DC 8B 02              6009 	mov	ar2,r3
   17DE 90 E7 41           6010 	mov	dptr,#(_EP0BUF + 0x0001)
   17E1 EA                 6011 	mov	a,r2
   17E2 F0                 6012 	movx	@dptr,a
                           6013 ;	../../include/ztex-eeprom.h:249: EP0BUF[2] = eeprom_write_checksum;
   17E3 90 3A 04           6014 	mov	dptr,#_eeprom_write_checksum
   17E6 E0                 6015 	movx	a,@dptr
   17E7 90 E7 42           6016 	mov	dptr,#(_EP0BUF + 0x0002)
   17EA F0                 6017 	movx	@dptr,a
                           6018 ;	../../include/ztex-eeprom.h:250: EP0BUF[3] = eeprom_select(EEPROM_ADDR,0,1);		// 1 means busy or error
   17EB 75 08 00           6019 	mov	_eeprom_select_PARM_2,#0x00
   17EE 75 09 01           6020 	mov	_eeprom_select_PARM_3,#0x01
   17F1 75 82 A2           6021 	mov	dpl,#0xA2
   17F4 12 03 46           6022 	lcall	_eeprom_select
   17F7 AA 82              6023 	mov	r2,dpl
   17F9 90 E7 43           6024 	mov	dptr,#(_EP0BUF + 0x0003)
   17FC EA                 6025 	mov	a,r2
   17FD F0                 6026 	movx	@dptr,a
                           6027 ;	../../include/ztex-eeprom.h:251: EP0BCH = 0;
   17FE 90 E6 8A           6028 	mov	dptr,#_EP0BCH
   1801 E4                 6029 	clr	a
   1802 F0                 6030 	movx	@dptr,a
                           6031 ;	../../include/ztex-eeprom.h:252: EP0BCL = 4;
   1803 90 E6 8B           6032 	mov	dptr,#_EP0BCL
   1806 74 04              6033 	mov	a,#0x04
   1808 F0                 6034 	movx	@dptr,a
                           6035 ;	../../include/ztex-conf.h:102: break;
   1809 02 19 80           6036 	ljmp	00187$
                           6037 ;	../../include/ztex-conf.h:100: case $0:
   180C                    6038 00165$:
                           6039 ;	../../include/ztex-conf.h:102: break;
   180C 90 E6 BA           6040 	mov	dptr,#(_SETUPDAT + 0x0002)
   180F E0                 6041 	movx	a,@dptr
   1810 90 3A 05           6042 	mov	dptr,#_mac_eeprom_addr
   1813 F0                 6043 	movx	@dptr,a
                           6044 ;	../../include/ztex-eeprom.h:368: EP0BCH = 0;
   1814 90 E6 8A           6045 	mov	dptr,#_EP0BCH
   1817 E4                 6046 	clr	a
   1818 F0                 6047 	movx	@dptr,a
                           6048 ;	../../include/ztex-eeprom.h:369: EP0BCL = mac_eeprom_read_ep0(); 
   1819 12 07 B9           6049 	lcall	_mac_eeprom_read_ep0
   181C E5 82              6050 	mov	a,dpl
   181E 90 E6 8B           6051 	mov	dptr,#_EP0BCL
   1821 F0                 6052 	movx	@dptr,a
                           6053 ;	../../include/ztex-conf.h:102: break;
   1822 02 19 80           6054 	ljmp	00187$
                           6055 ;	../../include/ztex-conf.h:100: case $0:
   1825                    6056 00166$:
                           6057 ;	../../include/ztex-conf.h:102: break;
   1825 75 08 00           6058 	mov	_eeprom_select_PARM_2,#0x00
   1828 75 09 01           6059 	mov	_eeprom_select_PARM_3,#0x01
   182B 75 82 A6           6060 	mov	dpl,#0xA6
   182E 12 03 46           6061 	lcall	_eeprom_select
   1831 AA 82              6062 	mov	r2,dpl
   1833 90 E7 40           6063 	mov	dptr,#_EP0BUF
   1836 EA                 6064 	mov	a,r2
   1837 F0                 6065 	movx	@dptr,a
                           6066 ;	../../include/ztex-eeprom.h:390: EP0BCH = 0;
   1838 90 E6 8A           6067 	mov	dptr,#_EP0BCH
   183B E4                 6068 	clr	a
   183C F0                 6069 	movx	@dptr,a
                           6070 ;	../../include/ztex-eeprom.h:391: EP0BCL = 1;
   183D 90 E6 8B           6071 	mov	dptr,#_EP0BCL
   1840 74 01              6072 	mov	a,#0x01
   1842 F0                 6073 	movx	@dptr,a
                           6074 ;	../../include/ztex-conf.h:102: break;
   1843 02 19 80           6075 	ljmp	00187$
                           6076 ;	../../include/ztex-conf.h:100: case $0:
   1846                    6077 00167$:
                           6078 ;	../../include/ztex-flash2.h:642: if ( flash_ec == 0 && SPI_CS == 0 ) {
   1846 90 3A 0E           6079 	mov	dptr,#_flash_ec
   1849 E0                 6080 	movx	a,@dptr
   184A FA                 6081 	mov	r2,a
   184B 70 09              6082 	jnz	00169$
   184D 20 A5 06           6083 	jb	_IOC5,00169$
                           6084 ;	../../include/ztex-flash2.h:643: flash_ec = FLASH_EC_PENDING;
   1850 90 3A 0E           6085 	mov	dptr,#_flash_ec
   1853 74 04              6086 	mov	a,#0x04
   1855 F0                 6087 	movx	@dptr,a
   1856                    6088 00169$:
                           6089 ;	../../include/ztex-utils.h:121: AUTOPTRL1=LO(&($0));
   1856 75 9B 07           6090 	mov	_AUTOPTRL1,#_flash_enabled
                           6091 ;	../../include/ztex-utils.h:122: AUTOPTRH1=HI(&($0));
   1859 7A 07              6092 	mov	r2,#_flash_enabled
   185B 7B 3A              6093 	mov	r3,#(_flash_enabled >> 8)
   185D 8B 9A              6094 	mov	_AUTOPTRH1,r3
                           6095 ;	../../include/ztex-utils.h:123: AUTOPTRL2=LO(&($1));
   185F 75 9E 40           6096 	mov	_AUTOPTRL2,#0x40
                           6097 ;	../../include/ztex-utils.h:124: AUTOPTRH2=HI(&($1));
   1862 75 9D E7           6098 	mov	_AUTOPTRH2,#0xE7
                           6099 ;	../../include/ztex-utils.h:130: __endasm; 
                           6100 	
   1865 C0 02              6101 	  push ar2
   1867 7A 08              6102 	    mov r2,#(8);
   1869 12 02 AD           6103 	  lcall _MEM_COPY1_int
   186C D0 02              6104 	  pop ar2
                           6105 	        
                           6106 ;	../../include/ztex-flash2.h:646: EP0BCH = 0;
   186E 90 E6 8A           6107 	mov	dptr,#_EP0BCH
   1871 E4                 6108 	clr	a
   1872 F0                 6109 	movx	@dptr,a
                           6110 ;	../../include/ztex-flash2.h:647: EP0BCL = 8;
   1873 90 E6 8B           6111 	mov	dptr,#_EP0BCL
   1876 74 08              6112 	mov	a,#0x08
   1878 F0                 6113 	movx	@dptr,a
                           6114 ;	../../include/ztex-conf.h:102: break;
   1879 02 19 80           6115 	ljmp	00187$
                           6116 ;	../../include/ztex-conf.h:100: case $0:
   187C                    6117 00171$:
                           6118 ;	../../include/ztex-flash2.h:667: ep0_read_mode = SETUPDAT[5];
   187C 90 E6 BD           6119 	mov	dptr,#(_SETUPDAT + 0x0005)
   187F E0                 6120 	movx	a,@dptr
   1880 FA                 6121 	mov	r2,a
   1881 90 3A 1E           6122 	mov	dptr,#_ep0_read_mode
   1884 F0                 6123 	movx	@dptr,a
                           6124 ;	../../include/ztex-flash2.h:668: if ( (ep0_read_mode==0) && flash_read_init((SETUPDAT[3] << 8) | SETUPDAT[2]) ) {
   1885 EA                 6125 	mov	a,r2
   1886 70 2D              6126 	jnz	00173$
   1888 90 E6 BB           6127 	mov	dptr,#(_SETUPDAT + 0x0003)
   188B E0                 6128 	movx	a,@dptr
   188C FB                 6129 	mov	r3,a
   188D 7A 00              6130 	mov	r2,#0x00
   188F 90 E6 BA           6131 	mov	dptr,#(_SETUPDAT + 0x0002)
   1892 E0                 6132 	movx	a,@dptr
   1893 7D 00              6133 	mov	r5,#0x00
   1895 4A                 6134 	orl	a,r2
   1896 F5 82              6135 	mov	dpl,a
   1898 ED                 6136 	mov	a,r5
   1899 4B                 6137 	orl	a,r3
   189A F5 83              6138 	mov	dph,a
   189C 12 09 63           6139 	lcall	_flash_read_init
   189F E5 82              6140 	mov	a,dpl
   18A1 60 12              6141 	jz	00173$
                           6142 ;	../../include/ztex-conf.h:137: EP0CS |= 0x01;	// set stall
   18A3 90 E6 A0           6143 	mov	dptr,#_EP0CS
   18A6 E0                 6144 	movx	a,@dptr
   18A7 FA                 6145 	mov	r2,a
   18A8 44 01              6146 	orl	a,#0x01
   18AA F0                 6147 	movx	@dptr,a
                           6148 ;	../../include/ztex-conf.h:138: ep0_payload_remaining = 0;
   18AB 90 3A 36           6149 	mov	dptr,#_ep0_payload_remaining
   18AE E4                 6150 	clr	a
   18AF F0                 6151 	movx	@dptr,a
   18B0 A3                 6152 	inc	dptr
   18B1 F0                 6153 	movx	@dptr,a
                           6154 ;	../../include/ztex-conf.h:139: break;
   18B2 02 19 80           6155 	ljmp	00187$
   18B5                    6156 00173$:
                           6157 ;	../../include/ztex-flash2.h:671: spi_read_ep0();  
   18B5 12 0C E1           6158 	lcall	_spi_read_ep0
                           6159 ;	../../include/ztex-flash2.h:672: EP0BCH = 0;
   18B8 90 E6 8A           6160 	mov	dptr,#_EP0BCH
   18BB E4                 6161 	clr	a
   18BC F0                 6162 	movx	@dptr,a
                           6163 ;	../../include/ztex-flash2.h:673: EP0BCL = ep0_payload_transfer; 
   18BD 90 3A 38           6164 	mov	dptr,#_ep0_payload_transfer
   18C0 E0                 6165 	movx	a,@dptr
   18C1 FA                 6166 	mov	r2,a
   18C2 90 E6 8B           6167 	mov	dptr,#_EP0BCL
   18C5 F0                 6168 	movx	@dptr,a
                           6169 ;	../../include/ztex-conf.h:102: break;
   18C6 02 19 80           6170 	ljmp	00187$
                           6171 ;	../../include/ztex-conf.h:100: case $0:
   18C9                    6172 00175$:
                           6173 ;	../../include/ztex-utils.h:121: AUTOPTRL1=LO(&($0));
   18C9 75 9B 0E           6174 	mov	_AUTOPTRL1,#_flash_ec
                           6175 ;	../../include/ztex-utils.h:122: AUTOPTRH1=HI(&($0));
   18CC 7A 0E              6176 	mov	r2,#_flash_ec
   18CE 7B 3A              6177 	mov	r3,#(_flash_ec >> 8)
   18D0 8B 9A              6178 	mov	_AUTOPTRH1,r3
                           6179 ;	../../include/ztex-utils.h:123: AUTOPTRL2=LO(&($1));
   18D2 75 9E 40           6180 	mov	_AUTOPTRL2,#0x40
                           6181 ;	../../include/ztex-utils.h:124: AUTOPTRH2=HI(&($1));
   18D5 75 9D E7           6182 	mov	_AUTOPTRH2,#0xE7
                           6183 ;	../../include/ztex-utils.h:130: __endasm; 
                           6184 	
   18D8 C0 02              6185 	  push ar2
   18DA 7A 0A              6186 	    mov r2,#(10);
   18DC 12 02 AD           6187 	  lcall _MEM_COPY1_int
   18DF D0 02              6188 	  pop ar2
                           6189 	        
                           6190 ;	../../include/ztex-flash2.h:715: EP0BCH = 0;
   18E1 90 E6 8A           6191 	mov	dptr,#_EP0BCH
   18E4 E4                 6192 	clr	a
   18E5 F0                 6193 	movx	@dptr,a
                           6194 ;	../../include/ztex-flash2.h:716: EP0BCL = 10;
   18E6 90 E6 8B           6195 	mov	dptr,#_EP0BCL
   18E9 74 0A              6196 	mov	a,#0x0A
   18EB F0                 6197 	movx	@dptr,a
                           6198 ;	../../include/ztex-conf.h:102: break;
   18EC 02 19 80           6199 	ljmp	00187$
                           6200 ;	../../include/ztex-conf.h:100: case $0:
   18EF                    6201 00176$:
                           6202 ;	../../include/ztex-utils.h:121: AUTOPTRL1=LO(&($0));
   18EF 75 9B 20           6203 	mov	_AUTOPTRL1,#_fpga_checksum
                           6204 ;	../../include/ztex-utils.h:122: AUTOPTRH1=HI(&($0));
   18F2 7A 20              6205 	mov	r2,#_fpga_checksum
   18F4 7B 3A              6206 	mov	r3,#(_fpga_checksum >> 8)
   18F6 8B 9A              6207 	mov	_AUTOPTRH1,r3
                           6208 ;	../../include/ztex-utils.h:123: AUTOPTRL2=LO(&($1));
   18F8 75 9E 41           6209 	mov	_AUTOPTRL2,#(_EP0BUF + 0x0001)
                           6210 ;	../../include/ztex-utils.h:124: AUTOPTRH2=HI(&($1));
   18FB 7A 41              6211 	mov	r2,#(_EP0BUF + 0x0001)
   18FD 7B E7              6212 	mov	r3,#((_EP0BUF + 0x0001) >> 8)
   18FF 8B 9D              6213 	mov	_AUTOPTRH2,r3
                           6214 ;	../../include/ztex-utils.h:130: __endasm; 
                           6215 	
   1901 C0 02              6216 	  push ar2
   1903 7A 07              6217 	    mov r2,#(7);
   1905 12 02 AD           6218 	  lcall _MEM_COPY1_int
   1908 D0 02              6219 	  pop ar2
                           6220 	        
                           6221 ;	../../include/ztex-fpga6.h:135: if ( IOE & bmBIT0 )  {
   190A E5 B1              6222 	mov	a,_IOE
   190C 30 E0 07           6223 	jnb	acc.0,00178$
                           6224 ;	../../include/ztex-fpga6.h:136: EP0BUF[0] = 0; 	 		// FPGA configured 
   190F 90 E7 40           6225 	mov	dptr,#_EP0BUF
   1912 E4                 6226 	clr	a
   1913 F0                 6227 	movx	@dptr,a
   1914 80 0C              6228 	sjmp	00179$
   1916                    6229 00178$:
                           6230 ;	../../include/ztex-fpga6.h:139: EP0BUF[0] = 1;			// FPGA unconfigured 
   1916 90 E7 40           6231 	mov	dptr,#_EP0BUF
   1919 74 01              6232 	mov	a,#0x01
   191B F0                 6233 	movx	@dptr,a
                           6234 ;	../../include/ztex-fpga6.h:140: OEE = 0;
   191C 75 B6 00           6235 	mov	_OEE,#0x00
                           6236 ;	../../include/ztex-fpga6.h:141: reset_fpga();			// prepare FPGA for configuration
   191F 12 0D 27           6237 	lcall	_reset_fpga
   1922                    6238 00179$:
                           6239 ;	../../include/ztex-fpga6.h:144: EP0BUF[8] = 1;			// bit order for bitstream in Flash memory: swapped
   1922 90 E7 48           6240 	mov	dptr,#(_EP0BUF + 0x0008)
   1925 74 01              6241 	mov	a,#0x01
   1927 F0                 6242 	movx	@dptr,a
                           6243 ;	../../include/ztex-fpga6.h:146: EP0BCH = 0;
   1928 90 E6 8A           6244 	mov	dptr,#_EP0BCH
   192B E4                 6245 	clr	a
   192C F0                 6246 	movx	@dptr,a
                           6247 ;	../../include/ztex-fpga6.h:147: EP0BCL = 9;
   192D 90 E6 8B           6248 	mov	dptr,#_EP0BCL
   1930 74 09              6249 	mov	a,#0x09
   1932 F0                 6250 	movx	@dptr,a
                           6251 ;	../../include/ztex-conf.h:102: break;
                           6252 ;	../../include/ztex-conf.h:100: case $0:
   1933 80 4B              6253 	sjmp	00187$
   1935                    6254 00180$:
                           6255 ;	../../include/ztex-conf.h:102: break;
   1935 90 E7 40           6256 	mov	dptr,#_EP0BUF
   1938 74 02              6257 	mov	a,#0x02
   193A F0                 6258 	movx	@dptr,a
                           6259 ;	../../include/ztex-conf.h:103: ]
   193B 90 E7 41           6260 	mov	dptr,#(_EP0BUF + 0x0001)
                           6261 ;	../../include/ztex-fpga6.h:223: EP0BCH = 0;
   193E E4                 6262 	clr	a
   193F F0                 6263 	movx	@dptr,a
   1940 90 E6 8A           6264 	mov	dptr,#_EP0BCH
   1943 F0                 6265 	movx	@dptr,a
                           6266 ;	../../include/ztex-fpga6.h:224: EP0BCL = 2;
   1944 90 E6 8B           6267 	mov	dptr,#_EP0BCL
   1947 74 02              6268 	mov	a,#0x02
   1949 F0                 6269 	movx	@dptr,a
                           6270 ;	../../include/ztex-conf.h:102: break;
                           6271 ;	../../include/ztex-isr.h:297: default:
   194A 80 34              6272 	sjmp	00187$
   194C                    6273 00181$:
                           6274 ;	../../include/ztex-isr.h:298: EP0CS |= 0x01;			// set stall, unknown request
   194C 90 E6 A0           6275 	mov	dptr,#_EP0CS
   194F E0                 6276 	movx	a,@dptr
   1950 FA                 6277 	mov	r2,a
   1951 44 01              6278 	orl	a,#0x01
   1953 F0                 6279 	movx	@dptr,a
                           6280 ;	../../include/ztex-isr.h:300: break;
                           6281 ;	../../include/ztex-isr.h:301: case 0x40: 					// vendor command
   1954 80 2A              6282 	sjmp	00187$
   1956                    6283 00183$:
                           6284 ;	../../include/ztex-isr.h:305: if ( SETUPDAT[7]!=0 || SETUPDAT[6]!=0 ) {
   1956 90 E6 BF           6285 	mov	dptr,#(_SETUPDAT + 0x0007)
   1959 E0                 6286 	movx	a,@dptr
   195A 70 06              6287 	jnz	00184$
   195C 90 E6 BE           6288 	mov	dptr,#(_SETUPDAT + 0x0006)
   195F E0                 6289 	movx	a,@dptr
   1960 60 16              6290 	jz	00185$
   1962                    6291 00184$:
                           6292 ;	../../include/ztex-isr.h:306: ep0_vendor_cmd_setup = 1;
   1962 90 3A 7A           6293 	mov	dptr,#_ep0_vendor_cmd_setup
   1965 74 01              6294 	mov	a,#0x01
   1967 F0                 6295 	movx	@dptr,a
                           6296 ;	../../include/ztex-isr.h:307: EP0BCL = 0;
   1968 90 E6 8B           6297 	mov	dptr,#_EP0BCL
   196B E4                 6298 	clr	a
   196C F0                 6299 	movx	@dptr,a
                           6300 ;	../../include/ztex-isr.h:308: EXIF &= ~bmBIT4;			// clear main USB interrupt flag
   196D 53 91 EF           6301 	anl	_EXIF,#0xEF
                           6302 ;	../../include/ztex-isr.h:309: USBIRQ = bmBIT0;			// clear SUADV IRQ
   1970 90 E6 5D           6303 	mov	dptr,#_USBIRQ
   1973 74 01              6304 	mov	a,#0x01
   1975 F0                 6305 	movx	@dptr,a
                           6306 ;	../../include/ztex-isr.h:310: return;					// don't clear HSNAK bit. This is done after the command has completed
   1976 80 19              6307 	sjmp	00188$
   1978                    6308 00185$:
                           6309 ;	../../include/ztex-isr.h:312: ep0_vendor_cmd_su();			// setup sequences of vendor command with no payload ara executed immediately
   1978 12 12 85           6310 	lcall	_ep0_vendor_cmd_su
                           6311 ;	../../include/ztex-isr.h:313: EP0BCL = 0;
   197B 90 E6 8B           6312 	mov	dptr,#_EP0BCL
   197E E4                 6313 	clr	a
   197F F0                 6314 	movx	@dptr,a
                           6315 ;	../../include/ztex-isr.h:315: }
   1980                    6316 00187$:
                           6317 ;	../../include/ztex-isr.h:317: EXIF &= ~bmBIT4;					// clear main USB interrupt flag
   1980 53 91 EF           6318 	anl	_EXIF,#0xEF
                           6319 ;	../../include/ztex-isr.h:318: USBIRQ = bmBIT0;					// clear SUADV IRQ
   1983 90 E6 5D           6320 	mov	dptr,#_USBIRQ
   1986 74 01              6321 	mov	a,#0x01
   1988 F0                 6322 	movx	@dptr,a
                           6323 ;	../../include/ztex-isr.h:319: EP0CS |= 0x80;					// clear the HSNAK bit
   1989 90 E6 A0           6324 	mov	dptr,#_EP0CS
   198C E0                 6325 	movx	a,@dptr
   198D FA                 6326 	mov	r2,a
   198E 44 80              6327 	orl	a,#0x80
   1990 F0                 6328 	movx	@dptr,a
   1991                    6329 00188$:
   1991 D0 D0              6330 	pop	psw
   1993 D0 01              6331 	pop	(0+1)
   1995 D0 00              6332 	pop	(0+0)
   1997 D0 07              6333 	pop	(0+7)
   1999 D0 06              6334 	pop	(0+6)
   199B D0 05              6335 	pop	(0+5)
   199D D0 04              6336 	pop	(0+4)
   199F D0 03              6337 	pop	(0+3)
   19A1 D0 02              6338 	pop	(0+2)
   19A3 D0 83              6339 	pop	dph
   19A5 D0 82              6340 	pop	dpl
   19A7 D0 F0              6341 	pop	b
   19A9 D0 E0              6342 	pop	acc
   19AB D0 20              6343 	pop	bits
   19AD 32                 6344 	reti
                           6345 ;------------------------------------------------------------
                           6346 ;Allocation info for local variables in function 'SOF_ISR'
                           6347 ;------------------------------------------------------------
                           6348 ;------------------------------------------------------------
                           6349 ;	../../include/ztex-isr.h:325: void SOF_ISR() __interrupt
                           6350 ;	-----------------------------------------
                           6351 ;	 function SOF_ISR
                           6352 ;	-----------------------------------------
   19AE                    6353 _SOF_ISR:
   19AE C0 E0              6354 	push	acc
   19B0 C0 82              6355 	push	dpl
   19B2 C0 83              6356 	push	dph
                           6357 ;	../../include/ztex-isr.h:327: EXIF &= ~bmBIT4;
   19B4 53 91 EF           6358 	anl	_EXIF,#0xEF
                           6359 ;	../../include/ztex-isr.h:328: USBIRQ = bmBIT1;
   19B7 90 E6 5D           6360 	mov	dptr,#_USBIRQ
   19BA 74 02              6361 	mov	a,#0x02
   19BC F0                 6362 	movx	@dptr,a
   19BD D0 83              6363 	pop	dph
   19BF D0 82              6364 	pop	dpl
   19C1 D0 E0              6365 	pop	acc
   19C3 32                 6366 	reti
                           6367 ;	eliminated unneeded push/pop psw
                           6368 ;	eliminated unneeded push/pop b
                           6369 ;------------------------------------------------------------
                           6370 ;Allocation info for local variables in function 'SUTOK_ISR'
                           6371 ;------------------------------------------------------------
                           6372 ;------------------------------------------------------------
                           6373 ;	../../include/ztex-isr.h:334: void SUTOK_ISR() __interrupt 
                           6374 ;	-----------------------------------------
                           6375 ;	 function SUTOK_ISR
                           6376 ;	-----------------------------------------
   19C4                    6377 _SUTOK_ISR:
   19C4 C0 E0              6378 	push	acc
   19C6 C0 82              6379 	push	dpl
   19C8 C0 83              6380 	push	dph
                           6381 ;	../../include/ztex-isr.h:336: EXIF &= ~bmBIT4;
   19CA 53 91 EF           6382 	anl	_EXIF,#0xEF
                           6383 ;	../../include/ztex-isr.h:337: USBIRQ = bmBIT2;
   19CD 90 E6 5D           6384 	mov	dptr,#_USBIRQ
   19D0 74 04              6385 	mov	a,#0x04
   19D2 F0                 6386 	movx	@dptr,a
   19D3 D0 83              6387 	pop	dph
   19D5 D0 82              6388 	pop	dpl
   19D7 D0 E0              6389 	pop	acc
   19D9 32                 6390 	reti
                           6391 ;	eliminated unneeded push/pop psw
                           6392 ;	eliminated unneeded push/pop b
                           6393 ;------------------------------------------------------------
                           6394 ;Allocation info for local variables in function 'SUSP_ISR'
                           6395 ;------------------------------------------------------------
                           6396 ;------------------------------------------------------------
                           6397 ;	../../include/ztex-isr.h:343: void SUSP_ISR() __interrupt
                           6398 ;	-----------------------------------------
                           6399 ;	 function SUSP_ISR
                           6400 ;	-----------------------------------------
   19DA                    6401 _SUSP_ISR:
   19DA C0 E0              6402 	push	acc
   19DC C0 82              6403 	push	dpl
   19DE C0 83              6404 	push	dph
                           6405 ;	../../include/ztex-isr.h:345: EXIF &= ~bmBIT4;
   19E0 53 91 EF           6406 	anl	_EXIF,#0xEF
                           6407 ;	../../include/ztex-isr.h:346: USBIRQ = bmBIT3;
   19E3 90 E6 5D           6408 	mov	dptr,#_USBIRQ
   19E6 74 08              6409 	mov	a,#0x08
   19E8 F0                 6410 	movx	@dptr,a
   19E9 D0 83              6411 	pop	dph
   19EB D0 82              6412 	pop	dpl
   19ED D0 E0              6413 	pop	acc
   19EF 32                 6414 	reti
                           6415 ;	eliminated unneeded push/pop psw
                           6416 ;	eliminated unneeded push/pop b
                           6417 ;------------------------------------------------------------
                           6418 ;Allocation info for local variables in function 'URES_ISR'
                           6419 ;------------------------------------------------------------
                           6420 ;------------------------------------------------------------
                           6421 ;	../../include/ztex-isr.h:352: void URES_ISR() __interrupt
                           6422 ;	-----------------------------------------
                           6423 ;	 function URES_ISR
                           6424 ;	-----------------------------------------
   19F0                    6425 _URES_ISR:
   19F0 C0 E0              6426 	push	acc
   19F2 C0 82              6427 	push	dpl
   19F4 C0 83              6428 	push	dph
                           6429 ;	../../include/ztex-isr.h:354: EXIF &= ~bmBIT4;
   19F6 53 91 EF           6430 	anl	_EXIF,#0xEF
                           6431 ;	../../include/ztex-isr.h:355: USBIRQ = bmBIT4;
   19F9 90 E6 5D           6432 	mov	dptr,#_USBIRQ
   19FC 74 10              6433 	mov	a,#0x10
   19FE F0                 6434 	movx	@dptr,a
   19FF D0 83              6435 	pop	dph
   1A01 D0 82              6436 	pop	dpl
   1A03 D0 E0              6437 	pop	acc
   1A05 32                 6438 	reti
                           6439 ;	eliminated unneeded push/pop psw
                           6440 ;	eliminated unneeded push/pop b
                           6441 ;------------------------------------------------------------
                           6442 ;Allocation info for local variables in function 'HSGRANT_ISR'
                           6443 ;------------------------------------------------------------
                           6444 ;------------------------------------------------------------
                           6445 ;	../../include/ztex-isr.h:361: void HSGRANT_ISR() __interrupt
                           6446 ;	-----------------------------------------
                           6447 ;	 function HSGRANT_ISR
                           6448 ;	-----------------------------------------
   1A06                    6449 _HSGRANT_ISR:
   1A06 C0 E0              6450 	push	acc
   1A08 C0 82              6451 	push	dpl
   1A0A C0 83              6452 	push	dph
                           6453 ;	../../include/ztex-isr.h:363: EXIF &= ~bmBIT4;
   1A0C 53 91 EF           6454 	anl	_EXIF,#0xEF
                           6455 ;	../../include/ztex-isr.h:365: USBIRQ = bmBIT5;
   1A0F 90 E6 5D           6456 	mov	dptr,#_USBIRQ
   1A12 74 20              6457 	mov	a,#0x20
   1A14 F0                 6458 	movx	@dptr,a
   1A15 D0 83              6459 	pop	dph
   1A17 D0 82              6460 	pop	dpl
   1A19 D0 E0              6461 	pop	acc
   1A1B 32                 6462 	reti
                           6463 ;	eliminated unneeded push/pop psw
                           6464 ;	eliminated unneeded push/pop b
                           6465 ;------------------------------------------------------------
                           6466 ;Allocation info for local variables in function 'EP0ACK_ISR'
                           6467 ;------------------------------------------------------------
                           6468 ;------------------------------------------------------------
                           6469 ;	../../include/ztex-isr.h:371: void EP0ACK_ISR() __interrupt
                           6470 ;	-----------------------------------------
                           6471 ;	 function EP0ACK_ISR
                           6472 ;	-----------------------------------------
   1A1C                    6473 _EP0ACK_ISR:
   1A1C C0 E0              6474 	push	acc
   1A1E C0 82              6475 	push	dpl
   1A20 C0 83              6476 	push	dph
                           6477 ;	../../include/ztex-isr.h:373: EXIF &= ~bmBIT4;	// clear USB interrupt flag
   1A22 53 91 EF           6478 	anl	_EXIF,#0xEF
                           6479 ;	../../include/ztex-isr.h:374: USBIRQ = bmBIT6;	// clear EP0ACK IRQ
   1A25 90 E6 5D           6480 	mov	dptr,#_USBIRQ
   1A28 74 40              6481 	mov	a,#0x40
   1A2A F0                 6482 	movx	@dptr,a
   1A2B D0 83              6483 	pop	dph
   1A2D D0 82              6484 	pop	dpl
   1A2F D0 E0              6485 	pop	acc
   1A31 32                 6486 	reti
                           6487 ;	eliminated unneeded push/pop psw
                           6488 ;	eliminated unneeded push/pop b
                           6489 ;------------------------------------------------------------
                           6490 ;Allocation info for local variables in function 'EP0IN_ISR'
                           6491 ;------------------------------------------------------------
                           6492 ;------------------------------------------------------------
                           6493 ;	../../include/ztex-isr.h:380: static void EP0IN_ISR () __interrupt
                           6494 ;	-----------------------------------------
                           6495 ;	 function EP0IN_ISR
                           6496 ;	-----------------------------------------
   1A32                    6497 _EP0IN_ISR:
   1A32 C0 20              6498 	push	bits
   1A34 C0 E0              6499 	push	acc
   1A36 C0 F0              6500 	push	b
   1A38 C0 82              6501 	push	dpl
   1A3A C0 83              6502 	push	dph
   1A3C C0 02              6503 	push	(0+2)
   1A3E C0 03              6504 	push	(0+3)
   1A40 C0 04              6505 	push	(0+4)
   1A42 C0 05              6506 	push	(0+5)
   1A44 C0 06              6507 	push	(0+6)
   1A46 C0 07              6508 	push	(0+7)
   1A48 C0 00              6509 	push	(0+0)
   1A4A C0 01              6510 	push	(0+1)
   1A4C C0 D0              6511 	push	psw
   1A4E 75 D0 00           6512 	mov	psw,#0x00
                           6513 ;	../../include/ztex-isr.h:382: EUSB = 0;			// block all USB interrupts
   1A51 C2 E8              6514 	clr	_EUSB
                           6515 ;	../../include/ztex-isr.h:383: ep0_payload_update();
   1A53 12 12 58           6516 	lcall	_ep0_payload_update
                           6517 ;	../../include/ztex-isr.h:384: switch ( ep0_prev_setup_request ) {
   1A56 90 3A 79           6518 	mov	dptr,#_ep0_prev_setup_request
   1A59 E0                 6519 	movx	a,@dptr
   1A5A FA                 6520 	mov	r2,a
   1A5B BA 30 03           6521 	cjne	r2,#0x30,00126$
   1A5E 02 1A D2           6522 	ljmp	00113$
   1A61                    6523 00126$:
   1A61 BA 33 03           6524 	cjne	r2,#0x33,00127$
   1A64 02 1A D2           6525 	ljmp	00113$
   1A67                    6526 00127$:
   1A67 BA 38 02           6527 	cjne	r2,#0x38,00128$
   1A6A 80 1E              6528 	sjmp	00101$
   1A6C                    6529 00128$:
   1A6C BA 3A 02           6530 	cjne	r2,#0x3A,00129$
   1A6F 80 61              6531 	sjmp	00113$
   1A71                    6532 00129$:
   1A71 BA 3B 02           6533 	cjne	r2,#0x3B,00130$
   1A74 80 24              6534 	sjmp	00103$
   1A76                    6535 00130$:
   1A76 BA 3D 02           6536 	cjne	r2,#0x3D,00131$
   1A79 80 57              6537 	sjmp	00113$
   1A7B                    6538 00131$:
   1A7B BA 40 02           6539 	cjne	r2,#0x40,00132$
   1A7E 80 52              6540 	sjmp	00113$
   1A80                    6541 00132$:
   1A80 BA 41 02           6542 	cjne	r2,#0x41,00133$
   1A83 80 25              6543 	sjmp	00106$
   1A85                    6544 00133$:
                           6545 ;	../../include/ztex-conf.h:105: case $0:
   1A85 BA 43 41           6546 	cjne	r2,#0x43,00112$
   1A88 80 48              6547 	sjmp	00113$
   1A8A                    6548 00101$:
                           6549 ;	../../include/ztex-eeprom.h:222: EP0BCH = 0;
   1A8A 90 E6 8A           6550 	mov	dptr,#_EP0BCH
   1A8D E4                 6551 	clr	a
   1A8E F0                 6552 	movx	@dptr,a
                           6553 ;	../../include/ztex-eeprom.h:223: EP0BCL = eeprom_read_ep0(); 
   1A8F 12 05 73           6554 	lcall	_eeprom_read_ep0
   1A92 E5 82              6555 	mov	a,dpl
   1A94 90 E6 8B           6556 	mov	dptr,#_EP0BCL
   1A97 F0                 6557 	movx	@dptr,a
                           6558 ;	../../include/ztex-conf.h:107: break;
                           6559 ;	../../include/ztex-conf.h:105: case $0:
   1A98 80 38              6560 	sjmp	00113$
   1A9A                    6561 00103$:
                           6562 ;	../../include/ztex-eeprom.h:371: EP0BCH = 0;
   1A9A 90 E6 8A           6563 	mov	dptr,#_EP0BCH
   1A9D E4                 6564 	clr	a
   1A9E F0                 6565 	movx	@dptr,a
                           6566 ;	../../include/ztex-eeprom.h:372: EP0BCL = mac_eeprom_read_ep0(); 
   1A9F 12 07 B9           6567 	lcall	_mac_eeprom_read_ep0
   1AA2 E5 82              6568 	mov	a,dpl
   1AA4 90 E6 8B           6569 	mov	dptr,#_EP0BCL
   1AA7 F0                 6570 	movx	@dptr,a
                           6571 ;	../../include/ztex-conf.h:107: break;
                           6572 ;	../../include/ztex-conf.h:105: case $0:
   1AA8 80 28              6573 	sjmp	00113$
   1AAA                    6574 00106$:
                           6575 ;	../../include/ztex-flash2.h:675: if ( ep0_payload_transfer != 0 ) {
   1AAA 90 3A 38           6576 	mov	dptr,#_ep0_payload_transfer
   1AAD E0                 6577 	movx	a,@dptr
   1AAE FA                 6578 	mov	r2,a
   1AAF 60 08              6579 	jz	00108$
                           6580 ;	../../include/ztex-flash2.h:676: flash_ec = 0;
   1AB1 90 3A 0E           6581 	mov	dptr,#_flash_ec
   1AB4 E4                 6582 	clr	a
   1AB5 F0                 6583 	movx	@dptr,a
                           6584 ;	../../include/ztex-flash2.h:677: spi_read_ep0(); 
   1AB6 12 0C E1           6585 	lcall	_spi_read_ep0
   1AB9                    6586 00108$:
                           6587 ;	../../include/ztex-flash2.h:679: EP0BCH = 0;
   1AB9 90 E6 8A           6588 	mov	dptr,#_EP0BCH
   1ABC E4                 6589 	clr	a
   1ABD F0                 6590 	movx	@dptr,a
                           6591 ;	../../include/ztex-flash2.h:680: EP0BCL = ep0_payload_transfer;
   1ABE 90 3A 38           6592 	mov	dptr,#_ep0_payload_transfer
   1AC1 E0                 6593 	movx	a,@dptr
   1AC2 FA                 6594 	mov	r2,a
   1AC3 90 E6 8B           6595 	mov	dptr,#_EP0BCL
   1AC6 F0                 6596 	movx	@dptr,a
                           6597 ;	../../include/ztex-conf.h:107: break;
                           6598 ;	../../include/ztex-isr.h:386: default:
   1AC7 80 09              6599 	sjmp	00113$
   1AC9                    6600 00112$:
                           6601 ;	../../include/ztex-isr.h:387: EP0BCH = 0;
   1AC9 90 E6 8A           6602 	mov	dptr,#_EP0BCH
                           6603 ;	../../include/ztex-isr.h:388: EP0BCL = 0;
   1ACC E4                 6604 	clr	a
   1ACD F0                 6605 	movx	@dptr,a
   1ACE 90 E6 8B           6606 	mov	dptr,#_EP0BCL
   1AD1 F0                 6607 	movx	@dptr,a
                           6608 ;	../../include/ztex-isr.h:389: }
   1AD2                    6609 00113$:
                           6610 ;	../../include/ztex-isr.h:390: EXIF &= ~bmBIT4;		// clear USB interrupt flag
   1AD2 53 91 EF           6611 	anl	_EXIF,#0xEF
                           6612 ;	../../include/ztex-isr.h:391: EPIRQ = bmBIT0;		// clear EP0IN IRQ
   1AD5 90 E6 5F           6613 	mov	dptr,#_EPIRQ
   1AD8 74 01              6614 	mov	a,#0x01
   1ADA F0                 6615 	movx	@dptr,a
                           6616 ;	../../include/ztex-isr.h:392: EUSB = 1;
   1ADB D2 E8              6617 	setb	_EUSB
   1ADD D0 D0              6618 	pop	psw
   1ADF D0 01              6619 	pop	(0+1)
   1AE1 D0 00              6620 	pop	(0+0)
   1AE3 D0 07              6621 	pop	(0+7)
   1AE5 D0 06              6622 	pop	(0+6)
   1AE7 D0 05              6623 	pop	(0+5)
   1AE9 D0 04              6624 	pop	(0+4)
   1AEB D0 03              6625 	pop	(0+3)
   1AED D0 02              6626 	pop	(0+2)
   1AEF D0 83              6627 	pop	dph
   1AF1 D0 82              6628 	pop	dpl
   1AF3 D0 F0              6629 	pop	b
   1AF5 D0 E0              6630 	pop	acc
   1AF7 D0 20              6631 	pop	bits
   1AF9 32                 6632 	reti
                           6633 ;------------------------------------------------------------
                           6634 ;Allocation info for local variables in function 'EP0OUT_ISR'
                           6635 ;------------------------------------------------------------
                           6636 ;------------------------------------------------------------
                           6637 ;	../../include/ztex-isr.h:398: static void EP0OUT_ISR () __interrupt
                           6638 ;	-----------------------------------------
                           6639 ;	 function EP0OUT_ISR
                           6640 ;	-----------------------------------------
   1AFA                    6641 _EP0OUT_ISR:
   1AFA C0 20              6642 	push	bits
   1AFC C0 E0              6643 	push	acc
   1AFE C0 F0              6644 	push	b
   1B00 C0 82              6645 	push	dpl
   1B02 C0 83              6646 	push	dph
   1B04 C0 02              6647 	push	(0+2)
   1B06 C0 03              6648 	push	(0+3)
   1B08 C0 04              6649 	push	(0+4)
   1B0A C0 05              6650 	push	(0+5)
   1B0C C0 06              6651 	push	(0+6)
   1B0E C0 07              6652 	push	(0+7)
   1B10 C0 00              6653 	push	(0+0)
   1B12 C0 01              6654 	push	(0+1)
   1B14 C0 D0              6655 	push	psw
   1B16 75 D0 00           6656 	mov	psw,#0x00
                           6657 ;	../../include/ztex-isr.h:400: EUSB = 0;			// block all USB interrupts
   1B19 C2 E8              6658 	clr	_EUSB
                           6659 ;	../../include/ztex-isr.h:401: if ( ep0_vendor_cmd_setup ) {
   1B1B 90 3A 7A           6660 	mov	dptr,#_ep0_vendor_cmd_setup
   1B1E E0                 6661 	movx	a,@dptr
   1B1F FA                 6662 	mov	r2,a
   1B20 60 20              6663 	jz	00102$
                           6664 ;	../../include/ztex-isr.h:402: ep0_vendor_cmd_setup = 0;
   1B22 90 3A 7A           6665 	mov	dptr,#_ep0_vendor_cmd_setup
   1B25 E4                 6666 	clr	a
   1B26 F0                 6667 	movx	@dptr,a
                           6668 ;	../../include/ztex-isr.h:403: ep0_payload_remaining = (SETUPDAT[7] << 8) | SETUPDAT[6];
   1B27 90 E6 BF           6669 	mov	dptr,#(_SETUPDAT + 0x0007)
   1B2A E0                 6670 	movx	a,@dptr
   1B2B FB                 6671 	mov	r3,a
   1B2C 7A 00              6672 	mov	r2,#0x00
   1B2E 90 E6 BE           6673 	mov	dptr,#(_SETUPDAT + 0x0006)
   1B31 E0                 6674 	movx	a,@dptr
   1B32 FC                 6675 	mov	r4,a
   1B33 7D 00              6676 	mov	r5,#0x00
   1B35 90 3A 36           6677 	mov	dptr,#_ep0_payload_remaining
   1B38 EC                 6678 	mov	a,r4
   1B39 4A                 6679 	orl	a,r2
   1B3A F0                 6680 	movx	@dptr,a
   1B3B ED                 6681 	mov	a,r5
   1B3C 4B                 6682 	orl	a,r3
   1B3D A3                 6683 	inc	dptr
   1B3E F0                 6684 	movx	@dptr,a
                           6685 ;	../../include/ztex-isr.h:404: ep0_vendor_cmd_su();
   1B3F 12 12 85           6686 	lcall	_ep0_vendor_cmd_su
   1B42                    6687 00102$:
                           6688 ;	../../include/ztex-isr.h:407: ep0_payload_update();
   1B42 12 12 58           6689 	lcall	_ep0_payload_update
                           6690 ;	../../include/ztex-isr.h:409: switch ( ep0_prev_setup_request ) {
   1B45 90 3A 79           6691 	mov	dptr,#_ep0_prev_setup_request
   1B48 E0                 6692 	movx	a,@dptr
   1B49 FA                 6693 	mov	r2,a
   1B4A BA 31 03           6694 	cjne	r2,#0x31,00131$
   1B4D 02 1B BA           6695 	ljmp	00114$
   1B50                    6696 00131$:
   1B50 BA 32 02           6697 	cjne	r2,#0x32,00132$
   1B53 80 62              6698 	sjmp	00111$
   1B55                    6699 00132$:
   1B55 BA 34 02           6700 	cjne	r2,#0x34,00133$
   1B58 80 60              6701 	sjmp	00114$
   1B5A                    6702 00133$:
   1B5A BA 35 02           6703 	cjne	r2,#0x35,00134$
   1B5D 80 5B              6704 	sjmp	00114$
   1B5F                    6705 00134$:
   1B5F BA 39 02           6706 	cjne	r2,#0x39,00135$
   1B62 80 0A              6707 	sjmp	00103$
   1B64                    6708 00135$:
   1B64 BA 3C 02           6709 	cjne	r2,#0x3C,00136$
   1B67 80 10              6710 	sjmp	00104$
   1B69                    6711 00136$:
                           6712 ;	../../include/ztex-conf.h:128: case $0:			
   1B69 BA 42 4E           6713 	cjne	r2,#0x42,00114$
   1B6C 80 1F              6714 	sjmp	00105$
   1B6E                    6715 00103$:
                           6716 ;	../../include/ztex-eeprom.h:240: eeprom_write_ep0(EP0BCL);
   1B6E 90 E6 8B           6717 	mov	dptr,#_EP0BCL
   1B71 E0                 6718 	movx	a,@dptr
   1B72 F5 82              6719 	mov	dpl,a
   1B74 12 05 A7           6720 	lcall	_eeprom_write_ep0
                           6721 ;	../../include/ztex-conf.h:130: break;
                           6722 ;	../../include/ztex-conf.h:128: case $0:			
   1B77 80 41              6723 	sjmp	00114$
   1B79                    6724 00104$:
                           6725 ;	../../include/ztex-eeprom.h:382: mac_eeprom_write(EP0BUF, mac_eeprom_addr, EP0BCL);
   1B79 90 3A 05           6726 	mov	dptr,#_mac_eeprom_addr
   1B7C E0                 6727 	movx	a,@dptr
   1B7D F5 12              6728 	mov	_mac_eeprom_write_PARM_2,a
   1B7F 90 E6 8B           6729 	mov	dptr,#_EP0BCL
   1B82 E0                 6730 	movx	a,@dptr
   1B83 F5 13              6731 	mov	_mac_eeprom_write_PARM_3,a
   1B85 90 E7 40           6732 	mov	dptr,#_EP0BUF
   1B88 12 06 C1           6733 	lcall	_mac_eeprom_write
                           6734 ;	../../include/ztex-conf.h:130: break;
                           6735 ;	../../include/ztex-conf.h:128: case $0:			
   1B8B 80 2D              6736 	sjmp	00114$
   1B8D                    6737 00105$:
                           6738 ;	../../include/ztex-flash2.h:699: if ( ep0_payload_transfer != 0 ) {
   1B8D 90 3A 38           6739 	mov	dptr,#_ep0_payload_transfer
   1B90 E0                 6740 	movx	a,@dptr
   1B91 FA                 6741 	mov	r2,a
   1B92 60 26              6742 	jz	00114$
                           6743 ;	../../include/ztex-flash2.h:700: flash_ec = 0;
   1B94 90 3A 0E           6744 	mov	dptr,#_flash_ec
   1B97 E4                 6745 	clr	a
   1B98 F0                 6746 	movx	@dptr,a
                           6747 ;	../../include/ztex-flash2.h:701: spi_send_ep0();
   1B99 12 0D 04           6748 	lcall	_spi_send_ep0
                           6749 ;	../../include/ztex-flash2.h:702: if ( flash_ec != 0 ) {
   1B9C 90 3A 0E           6750 	mov	dptr,#_flash_ec
   1B9F E0                 6751 	movx	a,@dptr
   1BA0 FA                 6752 	mov	r2,a
   1BA1 60 17              6753 	jz	00114$
                           6754 ;	../../include/ztex-flash2.h:703: spi_deselect();
   1BA3 12 08 ED           6755 	lcall	_spi_deselect
                           6756 ;	../../include/ztex-conf.h:137: EP0CS |= 0x01;	// set stall
   1BA6 90 E6 A0           6757 	mov	dptr,#_EP0CS
   1BA9 E0                 6758 	movx	a,@dptr
   1BAA FA                 6759 	mov	r2,a
   1BAB 44 01              6760 	orl	a,#0x01
   1BAD F0                 6761 	movx	@dptr,a
                           6762 ;	../../include/ztex-conf.h:138: ep0_payload_remaining = 0;
   1BAE 90 3A 36           6763 	mov	dptr,#_ep0_payload_remaining
   1BB1 E4                 6764 	clr	a
   1BB2 F0                 6765 	movx	@dptr,a
   1BB3 A3                 6766 	inc	dptr
   1BB4 F0                 6767 	movx	@dptr,a
                           6768 ;	../../include/ztex-conf.h:139: break;
                           6769 ;	../../include/ztex-conf.h:128: case $0:			
   1BB5 80 03              6770 	sjmp	00114$
   1BB7                    6771 00111$:
                           6772 ;	../../include/ztex-fpga6.h:202: fpga_send_ep0();
   1BB7 12 0E 1F           6773 	lcall	_fpga_send_ep0
                           6774 ;	../../include/ztex-isr.h:411: } 
   1BBA                    6775 00114$:
                           6776 ;	../../include/ztex-isr.h:413: EP0BCL = 0;
   1BBA 90 E6 8B           6777 	mov	dptr,#_EP0BCL
   1BBD E4                 6778 	clr	a
   1BBE F0                 6779 	movx	@dptr,a
                           6780 ;	../../include/ztex-isr.h:415: EXIF &= ~bmBIT4;		// clear main USB interrupt flag
   1BBF 53 91 EF           6781 	anl	_EXIF,#0xEF
                           6782 ;	../../include/ztex-isr.h:416: EPIRQ = bmBIT1;		// clear EP0OUT IRQ
   1BC2 90 E6 5F           6783 	mov	dptr,#_EPIRQ
   1BC5 74 02              6784 	mov	a,#0x02
   1BC7 F0                 6785 	movx	@dptr,a
                           6786 ;	../../include/ztex-isr.h:417: if ( ep0_payload_remaining == 0 ) {
   1BC8 90 3A 36           6787 	mov	dptr,#_ep0_payload_remaining
   1BCB E0                 6788 	movx	a,@dptr
   1BCC FA                 6789 	mov	r2,a
   1BCD A3                 6790 	inc	dptr
   1BCE E0                 6791 	movx	a,@dptr
   1BCF FB                 6792 	mov	r3,a
   1BD0 4A                 6793 	orl	a,r2
   1BD1 70 08              6794 	jnz	00116$
                           6795 ;	../../include/ztex-isr.h:418: EP0CS |= 0x80; 		// clear the HSNAK bit
   1BD3 90 E6 A0           6796 	mov	dptr,#_EP0CS
   1BD6 E0                 6797 	movx	a,@dptr
   1BD7 FA                 6798 	mov	r2,a
   1BD8 44 80              6799 	orl	a,#0x80
   1BDA F0                 6800 	movx	@dptr,a
   1BDB                    6801 00116$:
                           6802 ;	../../include/ztex-isr.h:420: EUSB = 1;
   1BDB D2 E8              6803 	setb	_EUSB
   1BDD D0 D0              6804 	pop	psw
   1BDF D0 01              6805 	pop	(0+1)
   1BE1 D0 00              6806 	pop	(0+0)
   1BE3 D0 07              6807 	pop	(0+7)
   1BE5 D0 06              6808 	pop	(0+6)
   1BE7 D0 05              6809 	pop	(0+5)
   1BE9 D0 04              6810 	pop	(0+4)
   1BEB D0 03              6811 	pop	(0+3)
   1BED D0 02              6812 	pop	(0+2)
   1BEF D0 83              6813 	pop	dph
   1BF1 D0 82              6814 	pop	dpl
   1BF3 D0 F0              6815 	pop	b
   1BF5 D0 E0              6816 	pop	acc
   1BF7 D0 20              6817 	pop	bits
   1BF9 32                 6818 	reti
                           6819 ;------------------------------------------------------------
                           6820 ;Allocation info for local variables in function 'EP1IN_ISR'
                           6821 ;------------------------------------------------------------
                           6822 ;------------------------------------------------------------
                           6823 ;	../../include/ztex-isr.h:427: void EP1IN_ISR() __interrupt
                           6824 ;	-----------------------------------------
                           6825 ;	 function EP1IN_ISR
                           6826 ;	-----------------------------------------
   1BFA                    6827 _EP1IN_ISR:
   1BFA C0 E0              6828 	push	acc
   1BFC C0 82              6829 	push	dpl
   1BFE C0 83              6830 	push	dph
                           6831 ;	../../include/ztex-isr.h:429: EXIF &= ~bmBIT4;
   1C00 53 91 EF           6832 	anl	_EXIF,#0xEF
                           6833 ;	../../include/ztex-isr.h:430: EPIRQ = bmBIT2;
   1C03 90 E6 5F           6834 	mov	dptr,#_EPIRQ
   1C06 74 04              6835 	mov	a,#0x04
   1C08 F0                 6836 	movx	@dptr,a
   1C09 D0 83              6837 	pop	dph
   1C0B D0 82              6838 	pop	dpl
   1C0D D0 E0              6839 	pop	acc
   1C0F 32                 6840 	reti
                           6841 ;	eliminated unneeded push/pop psw
                           6842 ;	eliminated unneeded push/pop b
                           6843 ;------------------------------------------------------------
                           6844 ;Allocation info for local variables in function 'EP1OUT_ISR'
                           6845 ;------------------------------------------------------------
                           6846 ;------------------------------------------------------------
                           6847 ;	../../include/ztex-isr.h:437: void EP1OUT_ISR() __interrupt
                           6848 ;	-----------------------------------------
                           6849 ;	 function EP1OUT_ISR
                           6850 ;	-----------------------------------------
   1C10                    6851 _EP1OUT_ISR:
   1C10 C0 E0              6852 	push	acc
   1C12 C0 82              6853 	push	dpl
   1C14 C0 83              6854 	push	dph
                           6855 ;	../../include/ztex-isr.h:439: EXIF &= ~bmBIT4;
   1C16 53 91 EF           6856 	anl	_EXIF,#0xEF
                           6857 ;	../../include/ztex-isr.h:440: EPIRQ = bmBIT3;
   1C19 90 E6 5F           6858 	mov	dptr,#_EPIRQ
   1C1C 74 08              6859 	mov	a,#0x08
   1C1E F0                 6860 	movx	@dptr,a
   1C1F D0 83              6861 	pop	dph
   1C21 D0 82              6862 	pop	dpl
   1C23 D0 E0              6863 	pop	acc
   1C25 32                 6864 	reti
                           6865 ;	eliminated unneeded push/pop psw
                           6866 ;	eliminated unneeded push/pop b
                           6867 ;------------------------------------------------------------
                           6868 ;Allocation info for local variables in function 'EP2_ISR'
                           6869 ;------------------------------------------------------------
                           6870 ;------------------------------------------------------------
                           6871 ;	../../include/ztex-isr.h:446: void EP2_ISR() __interrupt
                           6872 ;	-----------------------------------------
                           6873 ;	 function EP2_ISR
                           6874 ;	-----------------------------------------
   1C26                    6875 _EP2_ISR:
   1C26 C0 E0              6876 	push	acc
   1C28 C0 82              6877 	push	dpl
   1C2A C0 83              6878 	push	dph
                           6879 ;	../../include/ztex-isr.h:448: EXIF &= ~bmBIT4;
   1C2C 53 91 EF           6880 	anl	_EXIF,#0xEF
                           6881 ;	../../include/ztex-isr.h:449: EPIRQ = bmBIT4;
   1C2F 90 E6 5F           6882 	mov	dptr,#_EPIRQ
   1C32 74 10              6883 	mov	a,#0x10
   1C34 F0                 6884 	movx	@dptr,a
   1C35 D0 83              6885 	pop	dph
   1C37 D0 82              6886 	pop	dpl
   1C39 D0 E0              6887 	pop	acc
   1C3B 32                 6888 	reti
                           6889 ;	eliminated unneeded push/pop psw
                           6890 ;	eliminated unneeded push/pop b
                           6891 ;------------------------------------------------------------
                           6892 ;Allocation info for local variables in function 'EP4_ISR'
                           6893 ;------------------------------------------------------------
                           6894 ;------------------------------------------------------------
                           6895 ;	../../include/ztex-isr.h:455: void EP4_ISR() __interrupt
                           6896 ;	-----------------------------------------
                           6897 ;	 function EP4_ISR
                           6898 ;	-----------------------------------------
   1C3C                    6899 _EP4_ISR:
   1C3C C0 E0              6900 	push	acc
   1C3E C0 82              6901 	push	dpl
   1C40 C0 83              6902 	push	dph
                           6903 ;	../../include/ztex-isr.h:457: EXIF &= ~bmBIT4;
   1C42 53 91 EF           6904 	anl	_EXIF,#0xEF
                           6905 ;	../../include/ztex-isr.h:458: EPIRQ = bmBIT5;
   1C45 90 E6 5F           6906 	mov	dptr,#_EPIRQ
   1C48 74 20              6907 	mov	a,#0x20
   1C4A F0                 6908 	movx	@dptr,a
   1C4B D0 83              6909 	pop	dph
   1C4D D0 82              6910 	pop	dpl
   1C4F D0 E0              6911 	pop	acc
   1C51 32                 6912 	reti
                           6913 ;	eliminated unneeded push/pop psw
                           6914 ;	eliminated unneeded push/pop b
                           6915 ;------------------------------------------------------------
                           6916 ;Allocation info for local variables in function 'EP6_ISR'
                           6917 ;------------------------------------------------------------
                           6918 ;------------------------------------------------------------
                           6919 ;	../../include/ztex-isr.h:464: void EP6_ISR() __interrupt
                           6920 ;	-----------------------------------------
                           6921 ;	 function EP6_ISR
                           6922 ;	-----------------------------------------
   1C52                    6923 _EP6_ISR:
   1C52 C0 E0              6924 	push	acc
   1C54 C0 82              6925 	push	dpl
   1C56 C0 83              6926 	push	dph
                           6927 ;	../../include/ztex-isr.h:466: EXIF &= ~bmBIT4;
   1C58 53 91 EF           6928 	anl	_EXIF,#0xEF
                           6929 ;	../../include/ztex-isr.h:467: EPIRQ = bmBIT6;
   1C5B 90 E6 5F           6930 	mov	dptr,#_EPIRQ
   1C5E 74 40              6931 	mov	a,#0x40
   1C60 F0                 6932 	movx	@dptr,a
   1C61 D0 83              6933 	pop	dph
   1C63 D0 82              6934 	pop	dpl
   1C65 D0 E0              6935 	pop	acc
   1C67 32                 6936 	reti
                           6937 ;	eliminated unneeded push/pop psw
                           6938 ;	eliminated unneeded push/pop b
                           6939 ;------------------------------------------------------------
                           6940 ;Allocation info for local variables in function 'EP8_ISR'
                           6941 ;------------------------------------------------------------
                           6942 ;------------------------------------------------------------
                           6943 ;	../../include/ztex-isr.h:473: void EP8_ISR() __interrupt
                           6944 ;	-----------------------------------------
                           6945 ;	 function EP8_ISR
                           6946 ;	-----------------------------------------
   1C68                    6947 _EP8_ISR:
   1C68 C0 E0              6948 	push	acc
   1C6A C0 82              6949 	push	dpl
   1C6C C0 83              6950 	push	dph
                           6951 ;	../../include/ztex-isr.h:475: EXIF &= ~bmBIT4;
   1C6E 53 91 EF           6952 	anl	_EXIF,#0xEF
                           6953 ;	../../include/ztex-isr.h:476: EPIRQ = bmBIT7;
   1C71 90 E6 5F           6954 	mov	dptr,#_EPIRQ
   1C74 74 80              6955 	mov	a,#0x80
   1C76 F0                 6956 	movx	@dptr,a
   1C77 D0 83              6957 	pop	dph
   1C79 D0 82              6958 	pop	dpl
   1C7B D0 E0              6959 	pop	acc
   1C7D 32                 6960 	reti
                           6961 ;	eliminated unneeded push/pop psw
                           6962 ;	eliminated unneeded push/pop b
                           6963 ;------------------------------------------------------------
                           6964 ;Allocation info for local variables in function 'mac_eeprom_init'
                           6965 ;------------------------------------------------------------
                           6966 ;b                         Allocated to registers r2 
                           6967 ;c                         Allocated to registers r2 
                           6968 ;d                         Allocated to registers r4 
                           6969 ;buf                       Allocated with name '_mac_eeprom_init_buf_1_1'
                           6970 ;------------------------------------------------------------
                           6971 ;	../../include/ztex.h:241: void mac_eeprom_init ( ) { 
                           6972 ;	-----------------------------------------
                           6973 ;	 function mac_eeprom_init
                           6974 ;	-----------------------------------------
   1C7E                    6975 _mac_eeprom_init:
                           6976 ;	../../include/ztex.h:246: mac_eeprom_read ( buf, 0, 3 );	// read signature
   1C7E 75 10 00           6977 	mov	_mac_eeprom_read_PARM_2,#0x00
   1C81 75 11 03           6978 	mov	_mac_eeprom_read_PARM_3,#0x03
   1C84 90 3A 31           6979 	mov	dptr,#_mac_eeprom_init_buf_1_1
   1C87 12 05 D4           6980 	lcall	_mac_eeprom_read
                           6981 ;	../../include/ztex.h:247: if ( buf[0]==67 && buf[1]==68 && buf[2]==48 ) {
   1C8A 90 3A 31           6982 	mov	dptr,#_mac_eeprom_init_buf_1_1
   1C8D E0                 6983 	movx	a,@dptr
   1C8E FA                 6984 	mov	r2,a
   1C8F BA 43 24           6985 	cjne	r2,#0x43,00102$
   1C92 90 3A 32           6986 	mov	dptr,#(_mac_eeprom_init_buf_1_1 + 0x0001)
   1C95 E0                 6987 	movx	a,@dptr
   1C96 FA                 6988 	mov	r2,a
   1C97 BA 44 1C           6989 	cjne	r2,#0x44,00102$
   1C9A 90 3A 33           6990 	mov	dptr,#(_mac_eeprom_init_buf_1_1 + 0x0002)
   1C9D E0                 6991 	movx	a,@dptr
   1C9E FA                 6992 	mov	r2,a
   1C9F BA 30 14           6993 	cjne	r2,#0x30,00102$
                           6994 ;	../../include/ztex.h:248: config_data_valid = 1;
   1CA2 90 3A 06           6995 	mov	dptr,#_config_data_valid
   1CA5 74 01              6996 	mov	a,#0x01
   1CA7 F0                 6997 	movx	@dptr,a
                           6998 ;	../../include/ztex.h:249: mac_eeprom_read ( SN_STRING, 16, 10 );	// copy serial number
   1CA8 75 10 10           6999 	mov	_mac_eeprom_read_PARM_2,#0x10
   1CAB 75 11 0A           7000 	mov	_mac_eeprom_read_PARM_3,#0x0A
   1CAE 90 00 8A           7001 	mov	dptr,#_SN_STRING
   1CB1 12 05 D4           7002 	lcall	_mac_eeprom_read
   1CB4 80 05              7003 	sjmp	00123$
   1CB6                    7004 00102$:
                           7005 ;	../../include/ztex.h:252: config_data_valid = 0;
   1CB6 90 3A 06           7006 	mov	dptr,#_config_data_valid
   1CB9 E4                 7007 	clr	a
   1CBA F0                 7008 	movx	@dptr,a
                           7009 ;	../../include/ztex.h:255: for (b=0; b<10; b++) {	// abort if SN != "0000000000"
   1CBB                    7010 00123$:
   1CBB 7A 00              7011 	mov	r2,#0x00
   1CBD                    7012 00108$:
   1CBD BA 0A 00           7013 	cjne	r2,#0x0A,00133$
   1CC0                    7014 00133$:
   1CC0 50 12              7015 	jnc	00111$
                           7016 ;	../../include/ztex.h:256: if ( SN_STRING[b] != 48 )
   1CC2 EA                 7017 	mov	a,r2
   1CC3 24 8A              7018 	add	a,#_SN_STRING
   1CC5 F5 82              7019 	mov	dpl,a
   1CC7 E4                 7020 	clr	a
   1CC8 34 00              7021 	addc	a,#(_SN_STRING >> 8)
   1CCA F5 83              7022 	mov	dph,a
   1CCC E0                 7023 	movx	a,@dptr
   1CCD FB                 7024 	mov	r3,a
                           7025 ;	../../include/ztex.h:257: return;
   1CCE BB 30 54           7026 	cjne	r3,#0x30,00116$
                           7027 ;	../../include/ztex.h:255: for (b=0; b<10; b++) {	// abort if SN != "0000000000"
   1CD1 0A                 7028 	inc	r2
   1CD2 80 E9              7029 	sjmp	00108$
   1CD4                    7030 00111$:
                           7031 ;	../../include/ztex.h:260: mac_eeprom_read ( buf, 0xfb, 5 );	// read the last 5 MAC digits
   1CD4 75 10 FB           7032 	mov	_mac_eeprom_read_PARM_2,#0xFB
   1CD7 75 11 05           7033 	mov	_mac_eeprom_read_PARM_3,#0x05
   1CDA 90 3A 31           7034 	mov	dptr,#_mac_eeprom_init_buf_1_1
   1CDD 12 05 D4           7035 	lcall	_mac_eeprom_read
                           7036 ;	../../include/ztex.h:262: c=0;
   1CE0 7A 00              7037 	mov	r2,#0x00
                           7038 ;	../../include/ztex.h:263: for (b=0; b<5; b++) {	// convert to MAC to SN string
   1CE2 7B 00              7039 	mov	r3,#0x00
   1CE4                    7040 00112$:
   1CE4 BB 05 00           7041 	cjne	r3,#0x05,00136$
   1CE7                    7042 00136$:
   1CE7 50 3C              7043 	jnc	00116$
                           7044 ;	../../include/ztex.h:264: d = buf[b];
   1CE9 EB                 7045 	mov	a,r3
   1CEA 24 31              7046 	add	a,#_mac_eeprom_init_buf_1_1
   1CEC F5 82              7047 	mov	dpl,a
   1CEE E4                 7048 	clr	a
   1CEF 34 3A              7049 	addc	a,#(_mac_eeprom_init_buf_1_1 >> 8)
   1CF1 F5 83              7050 	mov	dph,a
   1CF3 E0                 7051 	movx	a,@dptr
   1CF4 FC                 7052 	mov	r4,a
                           7053 ;	../../include/ztex.h:265: SN_STRING[c] = hexdigits[d>>4];
   1CF5 EA                 7054 	mov	a,r2
   1CF6 24 8A              7055 	add	a,#_SN_STRING
   1CF8 FD                 7056 	mov	r5,a
   1CF9 E4                 7057 	clr	a
   1CFA 34 00              7058 	addc	a,#(_SN_STRING >> 8)
   1CFC FE                 7059 	mov	r6,a
   1CFD EC                 7060 	mov	a,r4
   1CFE C4                 7061 	swap	a
   1CFF 54 0F              7062 	anl	a,#0x0f
   1D01 90 1F E6           7063 	mov	dptr,#_mac_eeprom_init_hexdigits_1_1
   1D04 93                 7064 	movc	a,@a+dptr
   1D05 FF                 7065 	mov	r7,a
   1D06 8D 82              7066 	mov	dpl,r5
   1D08 8E 83              7067 	mov	dph,r6
   1D0A F0                 7068 	movx	@dptr,a
                           7069 ;	../../include/ztex.h:266: c++;
   1D0B 0A                 7070 	inc	r2
                           7071 ;	../../include/ztex.h:267: SN_STRING[c] = hexdigits[d & 15];
   1D0C EA                 7072 	mov	a,r2
   1D0D 24 8A              7073 	add	a,#_SN_STRING
   1D0F FD                 7074 	mov	r5,a
   1D10 E4                 7075 	clr	a
   1D11 34 00              7076 	addc	a,#(_SN_STRING >> 8)
   1D13 FE                 7077 	mov	r6,a
   1D14 74 0F              7078 	mov	a,#0x0F
   1D16 5C                 7079 	anl	a,r4
   1D17 90 1F E6           7080 	mov	dptr,#_mac_eeprom_init_hexdigits_1_1
   1D1A 93                 7081 	movc	a,@a+dptr
   1D1B FC                 7082 	mov	r4,a
   1D1C 8D 82              7083 	mov	dpl,r5
   1D1E 8E 83              7084 	mov	dph,r6
   1D20 F0                 7085 	movx	@dptr,a
                           7086 ;	../../include/ztex.h:268: c++;
   1D21 0A                 7087 	inc	r2
                           7088 ;	../../include/ztex.h:263: for (b=0; b<5; b++) {	// convert to MAC to SN string
   1D22 0B                 7089 	inc	r3
   1D23 80 BF              7090 	sjmp	00112$
   1D25                    7091 00116$:
   1D25 22                 7092 	ret
                           7093 ;------------------------------------------------------------
                           7094 ;Allocation info for local variables in function 'init_USB'
                           7095 ;------------------------------------------------------------
                           7096 ;------------------------------------------------------------
                           7097 ;	../../include/ztex.h:317: void init_USB ()
                           7098 ;	-----------------------------------------
                           7099 ;	 function init_USB
                           7100 ;	-----------------------------------------
   1D26                    7101 _init_USB:
                           7102 ;	../../include/ztex.h:319: USBCS |= bmBIT3;
   1D26 90 E6 80           7103 	mov	dptr,#_USBCS
   1D29 E0                 7104 	movx	a,@dptr
   1D2A 44 08              7105 	orl	a,#0x08
   1D2C F0                 7106 	movx	@dptr,a
                           7107 ;	../../include/ztex.h:321: CPUCS = bmBIT4 | bmBIT1;
   1D2D 90 E6 00           7108 	mov	dptr,#_CPUCS
   1D30 74 12              7109 	mov	a,#0x12
   1D32 F0                 7110 	movx	@dptr,a
                           7111 ;	../../include/ztex.h:322: wait(2);
   1D33 90 00 02           7112 	mov	dptr,#0x0002
   1D36 12 02 65           7113 	lcall	_wait
                           7114 ;	../../include/ztex.h:323: CKCON &= ~7;
   1D39 53 8E F8           7115 	anl	_CKCON,#0xF8
                           7116 ;	../../include/ztex.h:348: init_fpga();
   1D3C 12 0D 3C           7117 	lcall	_init_fpga
                           7118 ;	../../include/ztex-fpga-flash2.h:105: fpga_flash_result= 255;
   1D3F 90 3A 26           7119 	mov	dptr,#_fpga_flash_result
   1D42 74 FF              7120 	mov	a,#0xFF
   1D44 F0                 7121 	movx	@dptr,a
                           7122 ;	../../include/ztex.h:353: EA = 0;
   1D45 C2 AF              7123 	clr	_EA
                           7124 ;	../../include/ztex.h:354: EUSB = 0;
   1D47 C2 E8              7125 	clr	_EUSB
                           7126 ;	../../include/ezintavecs.h:123: INT8VEC_USB.op=0x02;
   1D49 90 00 43           7127 	mov	dptr,#_INT8VEC_USB
   1D4C 74 02              7128 	mov	a,#0x02
   1D4E F0                 7129 	movx	@dptr,a
                           7130 ;	../../include/ezintavecs.h:124: INT8VEC_USB.addrH = 0x01;
   1D4F 90 00 44           7131 	mov	dptr,#(_INT8VEC_USB + 0x0001)
   1D52 74 01              7132 	mov	a,#0x01
   1D54 F0                 7133 	movx	@dptr,a
                           7134 ;	../../include/ezintavecs.h:125: INT8VEC_USB.addrL = 0xb8;
   1D55 90 00 45           7135 	mov	dptr,#(_INT8VEC_USB + 0x0002)
   1D58 74 B8              7136 	mov	a,#0xB8
   1D5A F0                 7137 	movx	@dptr,a
                           7138 ;	../../include/ezintavecs.h:126: INTSETUP |= 8;
   1D5B 90 E6 68           7139 	mov	dptr,#_INTSETUP
   1D5E E0                 7140 	movx	a,@dptr
   1D5F 44 08              7141 	orl	a,#0x08
   1D61 F0                 7142 	movx	@dptr,a
                           7143 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1D62 90 01 00           7144 	mov	dptr,#_INTVEC_SUDAV
   1D65 74 02              7145 	mov	a,#0x02
   1D67 F0                 7146 	movx	@dptr,a
                           7147 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1D68 7A 62              7148 	mov	r2,#_SUDAV_ISR
   1D6A 7B 13              7149 	mov	r3,#(_SUDAV_ISR >> 8)
   1D6C 8B 04              7150 	mov	ar4,r3
   1D6E 90 01 01           7151 	mov	dptr,#(_INTVEC_SUDAV + 0x0001)
   1D71 EC                 7152 	mov	a,r4
   1D72 F0                 7153 	movx	@dptr,a
                           7154 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1D73 90 01 02           7155 	mov	dptr,#(_INTVEC_SUDAV + 0x0002)
   1D76 EA                 7156 	mov	a,r2
   1D77 F0                 7157 	movx	@dptr,a
                           7158 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1D78 90 01 04           7159 	mov	dptr,#_INTVEC_SOF
   1D7B 74 02              7160 	mov	a,#0x02
   1D7D F0                 7161 	movx	@dptr,a
                           7162 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1D7E 7A AE              7163 	mov	r2,#_SOF_ISR
   1D80 7B 19              7164 	mov	r3,#(_SOF_ISR >> 8)
   1D82 8B 04              7165 	mov	ar4,r3
   1D84 90 01 05           7166 	mov	dptr,#(_INTVEC_SOF + 0x0001)
   1D87 EC                 7167 	mov	a,r4
   1D88 F0                 7168 	movx	@dptr,a
                           7169 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1D89 90 01 06           7170 	mov	dptr,#(_INTVEC_SOF + 0x0002)
   1D8C EA                 7171 	mov	a,r2
   1D8D F0                 7172 	movx	@dptr,a
                           7173 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1D8E 90 01 08           7174 	mov	dptr,#_INTVEC_SUTOK
   1D91 74 02              7175 	mov	a,#0x02
   1D93 F0                 7176 	movx	@dptr,a
                           7177 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1D94 7A C4              7178 	mov	r2,#_SUTOK_ISR
   1D96 7B 19              7179 	mov	r3,#(_SUTOK_ISR >> 8)
   1D98 8B 04              7180 	mov	ar4,r3
   1D9A 90 01 09           7181 	mov	dptr,#(_INTVEC_SUTOK + 0x0001)
   1D9D EC                 7182 	mov	a,r4
   1D9E F0                 7183 	movx	@dptr,a
                           7184 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1D9F 90 01 0A           7185 	mov	dptr,#(_INTVEC_SUTOK + 0x0002)
   1DA2 EA                 7186 	mov	a,r2
   1DA3 F0                 7187 	movx	@dptr,a
                           7188 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1DA4 90 01 0C           7189 	mov	dptr,#_INTVEC_SUSPEND
   1DA7 74 02              7190 	mov	a,#0x02
   1DA9 F0                 7191 	movx	@dptr,a
                           7192 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1DAA 7A DA              7193 	mov	r2,#_SUSP_ISR
   1DAC 7B 19              7194 	mov	r3,#(_SUSP_ISR >> 8)
   1DAE 8B 04              7195 	mov	ar4,r3
   1DB0 90 01 0D           7196 	mov	dptr,#(_INTVEC_SUSPEND + 0x0001)
   1DB3 EC                 7197 	mov	a,r4
   1DB4 F0                 7198 	movx	@dptr,a
                           7199 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1DB5 90 01 0E           7200 	mov	dptr,#(_INTVEC_SUSPEND + 0x0002)
   1DB8 EA                 7201 	mov	a,r2
   1DB9 F0                 7202 	movx	@dptr,a
                           7203 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1DBA 90 01 10           7204 	mov	dptr,#_INTVEC_USBRESET
   1DBD 74 02              7205 	mov	a,#0x02
   1DBF F0                 7206 	movx	@dptr,a
                           7207 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1DC0 7A F0              7208 	mov	r2,#_URES_ISR
   1DC2 7B 19              7209 	mov	r3,#(_URES_ISR >> 8)
   1DC4 8B 04              7210 	mov	ar4,r3
   1DC6 90 01 11           7211 	mov	dptr,#(_INTVEC_USBRESET + 0x0001)
   1DC9 EC                 7212 	mov	a,r4
   1DCA F0                 7213 	movx	@dptr,a
                           7214 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1DCB 90 01 12           7215 	mov	dptr,#(_INTVEC_USBRESET + 0x0002)
   1DCE EA                 7216 	mov	a,r2
   1DCF F0                 7217 	movx	@dptr,a
                           7218 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1DD0 90 01 14           7219 	mov	dptr,#_INTVEC_HISPEED
   1DD3 74 02              7220 	mov	a,#0x02
   1DD5 F0                 7221 	movx	@dptr,a
                           7222 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1DD6 7A 06              7223 	mov	r2,#_HSGRANT_ISR
   1DD8 7B 1A              7224 	mov	r3,#(_HSGRANT_ISR >> 8)
   1DDA 8B 04              7225 	mov	ar4,r3
   1DDC 90 01 15           7226 	mov	dptr,#(_INTVEC_HISPEED + 0x0001)
   1DDF EC                 7227 	mov	a,r4
   1DE0 F0                 7228 	movx	@dptr,a
                           7229 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1DE1 90 01 16           7230 	mov	dptr,#(_INTVEC_HISPEED + 0x0002)
   1DE4 EA                 7231 	mov	a,r2
   1DE5 F0                 7232 	movx	@dptr,a
                           7233 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1DE6 90 01 18           7234 	mov	dptr,#_INTVEC_EP0ACK
   1DE9 74 02              7235 	mov	a,#0x02
   1DEB F0                 7236 	movx	@dptr,a
                           7237 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1DEC 7A 1C              7238 	mov	r2,#_EP0ACK_ISR
   1DEE 7B 1A              7239 	mov	r3,#(_EP0ACK_ISR >> 8)
   1DF0 8B 04              7240 	mov	ar4,r3
   1DF2 90 01 19           7241 	mov	dptr,#(_INTVEC_EP0ACK + 0x0001)
   1DF5 EC                 7242 	mov	a,r4
   1DF6 F0                 7243 	movx	@dptr,a
                           7244 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1DF7 90 01 1A           7245 	mov	dptr,#(_INTVEC_EP0ACK + 0x0002)
   1DFA EA                 7246 	mov	a,r2
   1DFB F0                 7247 	movx	@dptr,a
                           7248 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1DFC 90 01 20           7249 	mov	dptr,#_INTVEC_EP0IN
   1DFF 74 02              7250 	mov	a,#0x02
   1E01 F0                 7251 	movx	@dptr,a
                           7252 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E02 7A 32              7253 	mov	r2,#_EP0IN_ISR
   1E04 7B 1A              7254 	mov	r3,#(_EP0IN_ISR >> 8)
   1E06 8B 04              7255 	mov	ar4,r3
   1E08 90 01 21           7256 	mov	dptr,#(_INTVEC_EP0IN + 0x0001)
   1E0B EC                 7257 	mov	a,r4
   1E0C F0                 7258 	movx	@dptr,a
                           7259 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E0D 90 01 22           7260 	mov	dptr,#(_INTVEC_EP0IN + 0x0002)
   1E10 EA                 7261 	mov	a,r2
   1E11 F0                 7262 	movx	@dptr,a
                           7263 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E12 90 01 24           7264 	mov	dptr,#_INTVEC_EP0OUT
   1E15 74 02              7265 	mov	a,#0x02
   1E17 F0                 7266 	movx	@dptr,a
                           7267 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E18 7A FA              7268 	mov	r2,#_EP0OUT_ISR
   1E1A 7B 1A              7269 	mov	r3,#(_EP0OUT_ISR >> 8)
   1E1C 8B 04              7270 	mov	ar4,r3
   1E1E 90 01 25           7271 	mov	dptr,#(_INTVEC_EP0OUT + 0x0001)
   1E21 EC                 7272 	mov	a,r4
   1E22 F0                 7273 	movx	@dptr,a
                           7274 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E23 90 01 26           7275 	mov	dptr,#(_INTVEC_EP0OUT + 0x0002)
   1E26 EA                 7276 	mov	a,r2
   1E27 F0                 7277 	movx	@dptr,a
                           7278 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E28 90 01 28           7279 	mov	dptr,#_INTVEC_EP1IN
   1E2B 74 02              7280 	mov	a,#0x02
   1E2D F0                 7281 	movx	@dptr,a
                           7282 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E2E 7A FA              7283 	mov	r2,#_EP1IN_ISR
   1E30 7B 1B              7284 	mov	r3,#(_EP1IN_ISR >> 8)
   1E32 8B 04              7285 	mov	ar4,r3
   1E34 90 01 29           7286 	mov	dptr,#(_INTVEC_EP1IN + 0x0001)
   1E37 EC                 7287 	mov	a,r4
   1E38 F0                 7288 	movx	@dptr,a
                           7289 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E39 90 01 2A           7290 	mov	dptr,#(_INTVEC_EP1IN + 0x0002)
   1E3C EA                 7291 	mov	a,r2
   1E3D F0                 7292 	movx	@dptr,a
                           7293 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E3E 90 01 2C           7294 	mov	dptr,#_INTVEC_EP1OUT
   1E41 74 02              7295 	mov	a,#0x02
   1E43 F0                 7296 	movx	@dptr,a
                           7297 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E44 7A 10              7298 	mov	r2,#_EP1OUT_ISR
   1E46 7B 1C              7299 	mov	r3,#(_EP1OUT_ISR >> 8)
   1E48 8B 04              7300 	mov	ar4,r3
   1E4A 90 01 2D           7301 	mov	dptr,#(_INTVEC_EP1OUT + 0x0001)
   1E4D EC                 7302 	mov	a,r4
   1E4E F0                 7303 	movx	@dptr,a
                           7304 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E4F 90 01 2E           7305 	mov	dptr,#(_INTVEC_EP1OUT + 0x0002)
   1E52 EA                 7306 	mov	a,r2
   1E53 F0                 7307 	movx	@dptr,a
                           7308 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E54 90 01 30           7309 	mov	dptr,#_INTVEC_EP2
   1E57 74 02              7310 	mov	a,#0x02
   1E59 F0                 7311 	movx	@dptr,a
                           7312 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E5A 7A 26              7313 	mov	r2,#_EP2_ISR
   1E5C 7B 1C              7314 	mov	r3,#(_EP2_ISR >> 8)
   1E5E 8B 04              7315 	mov	ar4,r3
   1E60 90 01 31           7316 	mov	dptr,#(_INTVEC_EP2 + 0x0001)
   1E63 EC                 7317 	mov	a,r4
   1E64 F0                 7318 	movx	@dptr,a
                           7319 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E65 90 01 32           7320 	mov	dptr,#(_INTVEC_EP2 + 0x0002)
   1E68 EA                 7321 	mov	a,r2
   1E69 F0                 7322 	movx	@dptr,a
                           7323 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E6A 90 01 34           7324 	mov	dptr,#_INTVEC_EP4
   1E6D 74 02              7325 	mov	a,#0x02
   1E6F F0                 7326 	movx	@dptr,a
                           7327 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E70 7A 3C              7328 	mov	r2,#_EP4_ISR
   1E72 7B 1C              7329 	mov	r3,#(_EP4_ISR >> 8)
   1E74 8B 04              7330 	mov	ar4,r3
   1E76 90 01 35           7331 	mov	dptr,#(_INTVEC_EP4 + 0x0001)
   1E79 EC                 7332 	mov	a,r4
   1E7A F0                 7333 	movx	@dptr,a
                           7334 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E7B 90 01 36           7335 	mov	dptr,#(_INTVEC_EP4 + 0x0002)
   1E7E EA                 7336 	mov	a,r2
   1E7F F0                 7337 	movx	@dptr,a
                           7338 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E80 90 01 38           7339 	mov	dptr,#_INTVEC_EP6
   1E83 74 02              7340 	mov	a,#0x02
   1E85 F0                 7341 	movx	@dptr,a
                           7342 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E86 7A 52              7343 	mov	r2,#_EP6_ISR
   1E88 7B 1C              7344 	mov	r3,#(_EP6_ISR >> 8)
   1E8A 8B 04              7345 	mov	ar4,r3
   1E8C 90 01 39           7346 	mov	dptr,#(_INTVEC_EP6 + 0x0001)
   1E8F EC                 7347 	mov	a,r4
   1E90 F0                 7348 	movx	@dptr,a
                           7349 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1E91 90 01 3A           7350 	mov	dptr,#(_INTVEC_EP6 + 0x0002)
   1E94 EA                 7351 	mov	a,r2
   1E95 F0                 7352 	movx	@dptr,a
                           7353 ;	../../include/ezintavecs.h:115: $0.op=0x02;
   1E96 90 01 3C           7354 	mov	dptr,#_INTVEC_EP8
   1E99 74 02              7355 	mov	a,#0x02
   1E9B F0                 7356 	movx	@dptr,a
                           7357 ;	../../include/ezintavecs.h:116: $0.addrH=((unsigned short)(&$1)) >> 8;
   1E9C 7A 68              7358 	mov	r2,#_EP8_ISR
   1E9E 7B 1C              7359 	mov	r3,#(_EP8_ISR >> 8)
   1EA0 8B 04              7360 	mov	ar4,r3
   1EA2 90 01 3D           7361 	mov	dptr,#(_INTVEC_EP8 + 0x0001)
   1EA5 EC                 7362 	mov	a,r4
   1EA6 F0                 7363 	movx	@dptr,a
                           7364 ;	../../include/ezintavecs.h:117: $0.addrL=(unsigned short)(&$1);
   1EA7 90 01 3E           7365 	mov	dptr,#(_INTVEC_EP8 + 0x0002)
   1EAA EA                 7366 	mov	a,r2
   1EAB F0                 7367 	movx	@dptr,a
                           7368 ;	../../include/ztex.h:375: EXIF &= ~bmBIT4;
   1EAC 53 91 EF           7369 	anl	_EXIF,#0xEF
                           7370 ;	../../include/ztex.h:376: USBIRQ = 0x7f;
   1EAF 90 E6 5D           7371 	mov	dptr,#_USBIRQ
   1EB2 74 7F              7372 	mov	a,#0x7F
   1EB4 F0                 7373 	movx	@dptr,a
                           7374 ;	../../include/ztex.h:377: USBIE |= 0x7f; 
   1EB5 90 E6 5C           7375 	mov	dptr,#_USBIE
   1EB8 E0                 7376 	movx	a,@dptr
   1EB9 FA                 7377 	mov	r2,a
   1EBA 44 7F              7378 	orl	a,#0x7F
   1EBC F0                 7379 	movx	@dptr,a
                           7380 ;	../../include/ztex.h:378: EPIRQ = 0xff;
   1EBD 90 E6 5F           7381 	mov	dptr,#_EPIRQ
   1EC0 74 FF              7382 	mov	a,#0xFF
   1EC2 F0                 7383 	movx	@dptr,a
                           7384 ;	../../include/ztex.h:379: EPIE = 0xff;
   1EC3 90 E6 5E           7385 	mov	dptr,#_EPIE
   1EC6 74 FF              7386 	mov	a,#0xFF
   1EC8 F0                 7387 	movx	@dptr,a
                           7388 ;	../../include/ztex.h:381: EUSB = 1;
   1EC9 D2 E8              7389 	setb	_EUSB
                           7390 ;	../../include/ztex.h:382: EA = 1;
   1ECB D2 AF              7391 	setb	_EA
                           7392 ;	../../include/ztex.h:305: EP$0CFG = bmBIT7 | bmBIT5;
   1ECD 90 E6 11           7393 	mov	dptr,#_EP1INCFG
   1ED0 74 A0              7394 	mov	a,#0xA0
   1ED2 F0                 7395 	movx	@dptr,a
                           7396 ;	../../include/ezregs.h:46: __endasm;
                           7397 	
   1ED3 00                 7398 	 nop
   1ED4 00                 7399 	 nop
   1ED5 00                 7400 	 nop
   1ED6 00                 7401 	 nop
                           7402 	    
                           7403 ;	../../include/ztex.h:305: EP$0CFG = bmBIT7 | bmBIT5;
   1ED7 90 E6 10           7404 	mov	dptr,#_EP1OUTCFG
   1EDA 74 A0              7405 	mov	a,#0xA0
   1EDC F0                 7406 	movx	@dptr,a
                           7407 ;	../../include/ezregs.h:46: __endasm;
                           7408 	
   1EDD 00                 7409 	 nop
   1EDE 00                 7410 	 nop
   1EDF 00                 7411 	 nop
   1EE0 00                 7412 	 nop
                           7413 	    
                           7414 ;	../../include/ztex.h:300: ;
   1EE1 90 E6 12           7415 	mov	dptr,#_EP2CFG
   1EE4 74 A0              7416 	mov	a,#0xA0
   1EE6 F0                 7417 	movx	@dptr,a
                           7418 ;	../../include/ezregs.h:46: __endasm;
                           7419 	
   1EE7 00                 7420 	 nop
   1EE8 00                 7421 	 nop
   1EE9 00                 7422 	 nop
   1EEA 00                 7423 	 nop
                           7424 	    
                           7425 ;	../../include/ztex.h:300: ;
   1EEB 90 E6 13           7426 	mov	dptr,#_EP4CFG
   1EEE E4                 7427 	clr	a
   1EEF F0                 7428 	movx	@dptr,a
                           7429 ;	../../include/ezregs.h:46: __endasm;
                           7430 	
   1EF0 00                 7431 	 nop
   1EF1 00                 7432 	 nop
   1EF2 00                 7433 	 nop
   1EF3 00                 7434 	 nop
                           7435 	    
                           7436 ;	../../include/ztex.h:300: ;
   1EF4 90 E6 14           7437 	mov	dptr,#_EP6CFG
   1EF7 E4                 7438 	clr	a
   1EF8 F0                 7439 	movx	@dptr,a
                           7440 ;	../../include/ezregs.h:46: __endasm;
                           7441 	
   1EF9 00                 7442 	 nop
   1EFA 00                 7443 	 nop
   1EFB 00                 7444 	 nop
   1EFC 00                 7445 	 nop
                           7446 	    
                           7447 ;	../../include/ztex.h:300: ;
   1EFD 90 E6 15           7448 	mov	dptr,#_EP8CFG
   1F00 E4                 7449 	clr	a
   1F01 F0                 7450 	movx	@dptr,a
                           7451 ;	../../include/ezregs.h:46: __endasm;
                           7452 	
   1F02 00                 7453 	 nop
   1F03 00                 7454 	 nop
   1F04 00                 7455 	 nop
   1F05 00                 7456 	 nop
                           7457 	    
                           7458 ;	../../include/ztex.h:402: flash_init();
   1F06 12 0C 21           7459 	lcall	_flash_init
                           7460 ;	../../include/ztex.h:411: mac_eeprom_init();
   1F09 12 1C 7E           7461 	lcall	_mac_eeprom_init
                           7462 ;	../../include/ztex.h:417: fpga_configure_from_flash_init();
   1F0C 12 10 D9           7463 	lcall	_fpga_configure_from_flash_init
                           7464 ;	../../include/ztex.h:420: USBCS |= bmBIT7 | bmBIT1;
   1F0F 90 E6 80           7465 	mov	dptr,#_USBCS
   1F12 E0                 7466 	movx	a,@dptr
   1F13 44 82              7467 	orl	a,#0x82
   1F15 F0                 7468 	movx	@dptr,a
                           7469 ;	../../include/ztex.h:421: wait(10);
   1F16 90 00 0A           7470 	mov	dptr,#0x000A
   1F19 12 02 65           7471 	lcall	_wait
                           7472 ;	../../include/ztex.h:423: USBCS &= ~bmBIT3;
   1F1C 90 E6 80           7473 	mov	dptr,#_USBCS
   1F1F E0                 7474 	movx	a,@dptr
   1F20 54 F7              7475 	anl	a,#0xF7
   1F22 F0                 7476 	movx	@dptr,a
   1F23 22                 7477 	ret
                           7478 ;------------------------------------------------------------
                           7479 ;Allocation info for local variables in function 'main'
                           7480 ;------------------------------------------------------------
                           7481 ;------------------------------------------------------------
                           7482 ;	default.c:39: void main(void)	
                           7483 ;	-----------------------------------------
                           7484 ;	 function main
                           7485 ;	-----------------------------------------
   1F24                    7486 _main:
                           7487 ;	default.c:41: init_USB();
   1F24 12 1D 26           7488 	lcall	_init_USB
                           7489 ;	default.c:43: if ( config_data_valid ) {
   1F27 90 3A 06           7490 	mov	dptr,#_config_data_valid
   1F2A E0                 7491 	movx	a,@dptr
   1F2B FA                 7492 	mov	r2,a
   1F2C 60 0C              7493 	jz	00104$
                           7494 ;	default.c:44: mac_eeprom_read ( (__xdata BYTE*) (productString+20), 6, 1 );
   1F2E 90 1F 61           7495 	mov	dptr,#(_productString + 0x0014)
   1F31 75 10 06           7496 	mov	_mac_eeprom_read_PARM_2,#0x06
   1F34 75 11 01           7497 	mov	_mac_eeprom_read_PARM_3,#0x01
   1F37 12 05 D4           7498 	lcall	_mac_eeprom_read
                           7499 ;	default.c:47: while (1) {	}					//  twiddle thumbs
   1F3A                    7500 00104$:
   1F3A 80 FE              7501 	sjmp	00104$
                           7502 	.area CSEG    (CODE)
                           7503 	.area CONST   (CODE)
   1F40                    7504 _fpga_flash_boot_id:
   1F40 5A                 7505 	.db #0x5A
   1F41 54                 7506 	.db #0x54
   1F42 45                 7507 	.db #0x45
   1F43 58                 7508 	.db #0x58
   1F44 42                 7509 	.db #0x42
   1F45 53                 7510 	.db #0x53
   1F46 01                 7511 	.db #0x01
   1F47 01                 7512 	.db #0x01
   1F48                    7513 _manufacturerString:
   1F48 5A 54 45 58        7514 	.ascii "ZTEX"
   1F4C 00                 7515 	.db 0x00
   1F4D                    7516 _productString:
   1F4D 55 53 42 2D 46 50  7517 	.ascii "USB-FPGA Module 2.13  (default)"
        47 41 20 4D 6F 64
        75 6C 65 20 32 2E
        31 33 20 20 28 64
        65 66 61 75 6C 74
        29
   1F6C 00                 7518 	.db 0x00
   1F6D                    7519 _configurationString:
   1F6D 64 65 66 61 75 6C  7520 	.ascii "default"
        74
   1F74 00                 7521 	.db 0x00
   1F75                    7522 _PadByte:
   1F75 00                 7523 	.db #0x00
   1F76                    7524 _DeviceDescriptor:
   1F76 12                 7525 	.db #0x12
   1F77 01                 7526 	.db #0x01
   1F78 00                 7527 	.db #0x00
   1F79 02                 7528 	.db #0x02
   1F7A FF                 7529 	.db #0xFF
   1F7B FF                 7530 	.db #0xFF
   1F7C FF                 7531 	.db #0xFF
   1F7D 40                 7532 	.db #0x40
   1F7E 1A                 7533 	.db #0x1A
   1F7F 22                 7534 	.db #0x22
   1F80 00                 7535 	.db #0x00
   1F81 01                 7536 	.db #0x01
   1F82 00                 7537 	.db #0x00
   1F83 00                 7538 	.db #0x00
   1F84 01                 7539 	.db #0x01
   1F85 02                 7540 	.db #0x02
   1F86 03                 7541 	.db #0x03
   1F87 01                 7542 	.db #0x01
   1F88                    7543 _DeviceQualifierDescriptor:
   1F88 0A                 7544 	.db #0x0A
   1F89 06                 7545 	.db #0x06
   1F8A 00                 7546 	.db #0x00
   1F8B 02                 7547 	.db #0x02
   1F8C FF                 7548 	.db #0xFF
   1F8D FF                 7549 	.db #0xFF
   1F8E FF                 7550 	.db #0xFF
   1F8F 40                 7551 	.db #0x40
   1F90 01                 7552 	.db #0x01
   1F91 00                 7553 	.db #0x00
   1F92                    7554 _HighSpeedConfigDescriptor:
   1F92 09                 7555 	.db #0x09
   1F93 02                 7556 	.db #0x02
   1F94 27                 7557 	.db #0x27
   1F95 00                 7558 	.db #0x00
   1F96 01                 7559 	.db #0x01
   1F97 01                 7560 	.db #0x01
   1F98 04                 7561 	.db #0x04
   1F99 C0                 7562 	.db #0xC0
   1F9A 32                 7563 	.db #0x32
   1F9B 09                 7564 	.db #0x09
   1F9C 04                 7565 	.db #0x04
   1F9D 00                 7566 	.db #0x00
   1F9E 00                 7567 	.db #0x00
   1F9F 03                 7568 	.db #0x03
   1FA0 FF                 7569 	.db #0xFF
   1FA1 FF                 7570 	.db #0xFF
   1FA2 FF                 7571 	.db #0xFF
   1FA3 00                 7572 	.db #0x00
   1FA4 07                 7573 	.db #0x07
   1FA5 05                 7574 	.db #0x05
   1FA6 81                 7575 	.db #0x81
   1FA7 02                 7576 	.db #0x02
   1FA8 00                 7577 	.db #0x00
   1FA9 02                 7578 	.db #0x02
   1FAA 00                 7579 	.db #0x00
   1FAB 07                 7580 	.db #0x07
   1FAC 05                 7581 	.db #0x05
   1FAD 01                 7582 	.db #0x01
   1FAE 02                 7583 	.db #0x02
   1FAF 00                 7584 	.db #0x00
   1FB0 02                 7585 	.db #0x02
   1FB1 00                 7586 	.db #0x00
   1FB2 07                 7587 	.db #0x07
   1FB3 05                 7588 	.db #0x05
   1FB4 02                 7589 	.db #0x02
   1FB5 02                 7590 	.db #0x02
   1FB6 00                 7591 	.db #0x00
   1FB7 02                 7592 	.db #0x02
   1FB8 00                 7593 	.db #0x00
   1FB9                    7594 _HighSpeedConfigDescriptor_PadByte:
   1FB9 00                 7595 	.db #0x00
   1FBA                    7596 _FullSpeedConfigDescriptor:
   1FBA 09                 7597 	.db #0x09
   1FBB 02                 7598 	.db #0x02
   1FBC 27                 7599 	.db #0x27
   1FBD 00                 7600 	.db #0x00
   1FBE 01                 7601 	.db #0x01
   1FBF 01                 7602 	.db #0x01
   1FC0 04                 7603 	.db #0x04
   1FC1 C0                 7604 	.db #0xC0
   1FC2 32                 7605 	.db #0x32
   1FC3 09                 7606 	.db #0x09
   1FC4 04                 7607 	.db #0x04
   1FC5 00                 7608 	.db #0x00
   1FC6 00                 7609 	.db #0x00
   1FC7 03                 7610 	.db #0x03
   1FC8 FF                 7611 	.db #0xFF
   1FC9 FF                 7612 	.db #0xFF
   1FCA FF                 7613 	.db #0xFF
   1FCB 00                 7614 	.db #0x00
   1FCC 07                 7615 	.db #0x07
   1FCD 05                 7616 	.db #0x05
   1FCE 81                 7617 	.db #0x81
   1FCF 02                 7618 	.db #0x02
   1FD0 40                 7619 	.db #0x40
   1FD1 00                 7620 	.db #0x00
   1FD2 00                 7621 	.db #0x00
   1FD3 07                 7622 	.db #0x07
   1FD4 05                 7623 	.db #0x05
   1FD5 01                 7624 	.db #0x01
   1FD6 02                 7625 	.db #0x02
   1FD7 40                 7626 	.db #0x40
   1FD8 00                 7627 	.db #0x00
   1FD9 00                 7628 	.db #0x00
   1FDA 07                 7629 	.db #0x07
   1FDB 05                 7630 	.db #0x05
   1FDC 02                 7631 	.db #0x02
   1FDD 02                 7632 	.db #0x02
   1FDE 40                 7633 	.db #0x40
   1FDF 00                 7634 	.db #0x00
   1FE0 00                 7635 	.db #0x00
   1FE1                    7636 _FullSpeedConfigDescriptor_PadByte:
   1FE1 00                 7637 	.db #0x00
   1FE2                    7638 _EmptyStringDescriptor:
   1FE2 04                 7639 	.db #0x04
   1FE3 03                 7640 	.db #0x03
   1FE4 00                 7641 	.db #0x00
   1FE5 00                 7642 	.db #0x00
   1FE6                    7643 _mac_eeprom_init_hexdigits_1_1:
   1FE6 30 31 32 33 34 35  7644 	.ascii "0123456789ABCDEF"
        36 37 38 39 41 42
        43 44 45 46
   1FF6 00                 7645 	.db 0x00
                           7646 	.area XINIT   (CODE)
   1FF7                    7647 __xinit__ep0_payload_remaining:
   1FF7 00 00              7648 	.byte #0x00,#0x00
   1FF9                    7649 __xinit__ep0_payload_transfer:
   1FF9 00                 7650 	.db #0x00
   1FFA                    7651 __xinit__GPIF_WAVE_DATA_HSFPGA_24MHZ:
   1FFA 01                 7652 	.db #0x01
   1FFB 88                 7653 	.db #0x88
   1FFC 01                 7654 	.db #0x01
   1FFD 01                 7655 	.db #0x01
   1FFE 01                 7656 	.db #0x01
   1FFF 01                 7657 	.db #0x01
   2000 01                 7658 	.db #0x01
   2001 07                 7659 	.db #0x07
   2002 02                 7660 	.db #0x02
   2003 07                 7661 	.db #0x07
   2004 02                 7662 	.db #0x02
   2005 02                 7663 	.db #0x02
   2006 02                 7664 	.db #0x02
   2007 02                 7665 	.db #0x02
   2008 02                 7666 	.db #0x02
   2009 00                 7667 	.db #0x00
   200A 20                 7668 	.db #0x20
   200B 00                 7669 	.db #0x00
   200C 00                 7670 	.db #0x00
   200D 00                 7671 	.db #0x00
   200E 00                 7672 	.db #0x00
   200F 00                 7673 	.db #0x00
   2010 00                 7674 	.db #0x00
   2011 20                 7675 	.db #0x20
   2012 00                 7676 	.db #0x00
   2013 36                 7677 	.db #0x36
   2014 00                 7678 	.db #0x00
   2015 00                 7679 	.db #0x00
   2016 00                 7680 	.db #0x00
   2017 00                 7681 	.db #0x00
   2018 00                 7682 	.db #0x00
   2019 3F                 7683 	.db #0x3F
   201A                    7684 __xinit__GPIF_WAVE_DATA_HSFPGA_12MHZ:
   201A 02                 7685 	.db #0x02
   201B 01                 7686 	.db #0x01
   201C 90                 7687 	.db #0x90
   201D 01                 7688 	.db #0x01
   201E 01                 7689 	.db #0x01
   201F 01                 7690 	.db #0x01
   2020 01                 7691 	.db #0x01
   2021 07                 7692 	.db #0x07
   2022 02                 7693 	.db #0x02
   2023 02                 7694 	.db #0x02
   2024 07                 7695 	.db #0x07
   2025 02                 7696 	.db #0x02
   2026 02                 7697 	.db #0x02
   2027 02                 7698 	.db #0x02
   2028 02                 7699 	.db #0x02
   2029 00                 7700 	.db #0x00
   202A 20                 7701 	.db #0x20
   202B 00                 7702 	.db #0x00
   202C 00                 7703 	.db #0x00
   202D 00                 7704 	.db #0x00
   202E 00                 7705 	.db #0x00
   202F 00                 7706 	.db #0x00
   2030 00                 7707 	.db #0x00
   2031 20                 7708 	.db #0x20
   2032 00                 7709 	.db #0x00
   2033 00                 7710 	.db #0x00
   2034 36                 7711 	.db #0x36
   2035 00                 7712 	.db #0x00
   2036 00                 7713 	.db #0x00
   2037 00                 7714 	.db #0x00
   2038 00                 7715 	.db #0x00
   2039 3F                 7716 	.db #0x3F
   203A                    7717 __xinit__ep0_prev_setup_request:
   203A FF                 7718 	.db #0xFF
   203B                    7719 __xinit__ep0_vendor_cmd_setup:
   203B 00                 7720 	.db #0x00
   203C                    7721 __xinit__ISOFRAME_COUNTER:
   203C 00 00              7722 	.byte #0x00,#0x00
   203E 00 00              7723 	.byte #0x00,#0x00
   2040 00 00              7724 	.byte #0x00,#0x00
   2042 00 00              7725 	.byte #0x00,#0x00
                           7726 	.area CABS    (ABS,CODE)
