/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved */

/*
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

/*!
 * @file sdma_script_code.h
 * @brief This file contains functions of SDMA scripts code initialization
 *
 * The file was generated automatically. Based on sdma scripts library.
 *
 * @ingroup SDMA
 */
/*******************************************************************************

			SDMA RELEASE LABEL:	"SS15_ELVIS V1.1"

*******************************************************************************/

#ifndef __SDMA_SCRIPT_CODE_H__
#define __SDMA_SCRIPT_CODE_H__

/*!
* SDMA ROM scripts start addresses and sizes
*/

#define start_ADDR_MX51   0
#define start_SIZE_MX51   24

#define core_ADDR_MX51   80
#define core_SIZE_MX51   232

#define common_ADDR_MX51   312
#define common_SIZE_MX51   330

#define ap_2_ap_ADDR_MX51   642
#define ap_2_ap_SIZE_MX51   41

#define app_2_mcu_ADDR_MX51   683
#define app_2_mcu_SIZE_MX51   64

#define mcu_2_app_ADDR_MX51   747
#define mcu_2_app_SIZE_MX51   70

#define uart_2_mcu_ADDR_MX51   817
#define uart_2_mcu_SIZE_MX51   75

#define shp_2_mcu_ADDR_MX51   892
#define shp_2_mcu_SIZE_MX51   69

#define mcu_2_shp_ADDR_MX51   961
#define mcu_2_shp_SIZE_MX51   72

#define app_2_per_ADDR_MX51   1033
#define app_2_per_SIZE_MX51   66

#define per_2_app_ADDR_MX51   1099
#define per_2_app_SIZE_MX51   74

#define per_2_shp_ADDR_MX51   1173
#define per_2_shp_SIZE_MX51   78

#define shp_2_per_ADDR_MX51   1251
#define shp_2_per_SIZE_MX51   72

#define uartsh_2_mcu_ADDR_MX51   1323
#define uartsh_2_mcu_SIZE_MX51   69

#define mcu_2_ata_ADDR_MX51   1392
#define mcu_2_ata_SIZE_MX51   81

#define ata_2_mcu_ADDR_MX51   1473
#define ata_2_mcu_SIZE_MX51   96

#define loop_DMAs_routines_ADDR_MX51   1569
#define loop_DMAs_routines_SIZE_MX51   227

#define test_ADDR_MX51   1796
#define test_SIZE_MX51   63

#define signature_ADDR_MX51   1023
#define signature_SIZE_MX51   1

/*!
* SDMA RAM scripts start addresses and sizes
*/

#define ext_mem__ipu_ram_ADDR_MX51   6144
#define ext_mem__ipu_ram_SIZE_MX51   123

#define firi_2_mcu_ADDR_MX51   6267
#define firi_2_mcu_SIZE_MX51   97

#define mcu_2_firi_ADDR_MX51   6364
#define mcu_2_firi_SIZE_MX51   79

#define mcu_2_spdif_ADDR_MX51   6443
#define mcu_2_spdif_SIZE_MX51   59

#define mcu_2_ssiapp_ADDR_MX51   6502
#define mcu_2_ssiapp_SIZE_MX51   98

#define mcu_2_ssish_ADDR_MX51   6600
#define mcu_2_ssish_SIZE_MX51   89

#define ssiapp_2_mcu_ADDR_MX51   6689
#define ssiapp_2_mcu_SIZE_MX51   94

#define ssish_2_mcu_ADDR_MX51   6783
#define ssish_2_mcu_SIZE_MX51   84

#define uart_2_per_ADDR_MX51   6867
#define uart_2_per_SIZE_MX51   73

#define uartsh_2_per_ADDR_MX51   6940
#define uartsh_2_per_SIZE_MX51   67

/*!
* SDMA RAM image start address and size
*/

#define RAM_CODE_START_ADDR_MX51     6144
#define RAM_CODE_SIZE_MX51           863

/*!
* Buffer that holds the SDMA RAM image
*/
__attribute__ ((__aligned__(4)))
#ifndef CONFIG_XIP_KERNEL
const
#endif
static const short sdma_code_mx51[] = {
0x0e70, 0x0611, 0x5616, 0xc13c, 0x7d2a, 0x5ade, 0x008e, 0xc14e,
0x7c26, 0x5be0, 0x5ef0, 0x5ce8, 0x0688, 0x08ff, 0x0011, 0x28ff,
0x00bc, 0x53f6, 0x05df, 0x7d0b, 0x6dc5, 0x03df, 0x7d03, 0x6bd5,
0xd84f, 0x982b, 0x6b05, 0xc681, 0x7e27, 0x7f29, 0x982b, 0x6d01,
0x03df, 0x7d05, 0x6bd5, 0xc6ab, 0x7e18, 0x7f1a, 0x982b, 0x6b05,
0xc621, 0x7e07, 0x7f06, 0x52de, 0x53e6, 0xc159, 0x7dd7, 0x0200,
0x9803, 0x0007, 0x6004, 0x680c, 0x53f6, 0x028e, 0x00a3, 0xc256,
0x048b, 0x0498, 0x0454, 0x068a, 0x982b, 0x0207, 0x680c, 0x6ddf,
0x0107, 0x68ff, 0x60d0, 0x9834, 0x0207, 0x68ff, 0x6d28, 0x0107,
0x6004, 0x680c, 0x9834, 0x0007, 0x68ff, 0x60d0, 0x9834, 0x0288,
0x03a5, 0x3b03, 0x3d03, 0x4d00, 0x7d0a, 0x0804, 0x00a5, 0x00da,
0x7d1a, 0x02a0, 0x7b01, 0x65d8, 0x7eee, 0x65ff, 0x7eec, 0x0804,
0x02d0, 0x7d11, 0x4b00, 0x7c0f, 0x008a, 0x3003, 0x6dcf, 0x6bdf,
0x0015, 0x0015, 0x7b02, 0x65d8, 0x0000, 0x7edd, 0x63ff, 0x7edb,
0x3a03, 0x6dcd, 0x6bdd, 0x008a, 0x7b02, 0x65d8, 0x0000, 0x7ed3,
0x65ff, 0x7ed1, 0x0006, 0x2618, 0x1e10, 0x0b70, 0x0311, 0x5313,
0x58d3, 0x008b, 0x5efb, 0xc13c, 0x7d55, 0x5ac0, 0x5bc8, 0xc14e,
0x7c51, 0x0388, 0x6d04, 0x0dff, 0x0511, 0x1dff, 0x05bc, 0x56fb,
0x6ec3, 0x62c8, 0x7e38, 0x0264, 0x7d0b, 0x0212, 0x3aff, 0x02df,
0x7c05, 0x008f, 0x05a0, 0x0015, 0x0015, 0xd8b5, 0x0400, 0x988f,
0x56fb, 0x6ec3, 0x62c8, 0x0212, 0x3aff, 0x008a, 0x4800, 0x7d02,
0x05a0, 0xd8c0, 0x6a28, 0x7f24, 0x008b, 0x52c0, 0x53c8, 0x04a5,
0xc159, 0x7dd4, 0x0401, 0x0200, 0x9886, 0x1e08, 0x6ec3, 0x7802,
0x62c8, 0x6a0b, 0x7e10, 0x6a28, 0x7f13, 0x0000, 0x2608, 0x0006,
0x1e08, 0x6ec1, 0x7802, 0x62c8, 0x6a09, 0x7e05, 0x6a28, 0x7f08,
0x0000, 0x2608, 0x0006, 0x0007, 0x68cc, 0x6a28, 0x7f01, 0x98d9,
0x0007, 0x6a0c, 0x6a0c, 0x6204, 0x6a04, 0x6a2b, 0x6a28, 0x0007,
0x680c, 0x0454, 0x0200, 0x9883, 0x0b70, 0x0311, 0x5313, 0x58d3,
0x008b, 0x5efb, 0xc13c, 0x7d45, 0x5ac0, 0x5bc8, 0xc14e, 0x56f8,
0x7c40, 0x6ed1, 0x0388, 0x6d00, 0x0dff, 0x0511, 0x1dff, 0x05bc,
0x4d00, 0x7d2f, 0x0e70, 0x0611, 0x522e, 0x02b9, 0x4a00, 0x7c09,
0x52fe, 0x50d3, 0x02b8, 0x4a00, 0x7c04, 0x62ff, 0x7e1c, 0x0400,
0x98f2, 0x008f, 0x00d5, 0x7d0b, 0x008d, 0x05a0, 0x56fb, 0x6ed1,
0x7802, 0x6209, 0x6ac8, 0x0000, 0x7e11, 0x7f0d, 0x98f0, 0x05a0,
0x0015, 0x0015, 0x56fb, 0x6ed3, 0x7802, 0x620b, 0x6ac8, 0x0000,
0x7e05, 0x7f01, 0x98f0, 0x0007, 0x68cc, 0x9920, 0x0007, 0x6a0c,
0x0454, 0x62ff, 0x7efb, 0x008b, 0x52c0, 0x53c8, 0xc159, 0x7dbd,
0x0401, 0x0200, 0x98e2, 0xc1d9, 0xc1e3, 0x57db, 0x52f3, 0x6a01,
0x008f, 0x00d5, 0x7d01, 0x008d, 0x05a0, 0x5deb, 0x56fb, 0x0478,
0x7d28, 0x0479, 0x7c16, 0x0015, 0x0015, 0x0388, 0x620a, 0x0808,
0x7801, 0x0217, 0x5a06, 0x7f1d, 0x620a, 0x0808, 0x7801, 0x0217,
0x5a26, 0x7f17, 0x2301, 0x4b00, 0x7cf1, 0x0b70, 0x0311, 0x5313,
0x995a, 0x0015, 0x0015, 0x0015, 0x7804, 0x620b, 0x5a06, 0x620b,
0x5a26, 0x7c07, 0x0000, 0x55eb, 0x4d00, 0x7d06, 0xc1fa, 0x57db,
0x9930, 0x0007, 0x680c, 0xc213, 0xc20a, 0x992d, 0xc1e3, 0x57db,
0x5fe3, 0x57e3, 0x52f3, 0x6a01, 0x008f, 0x00d5, 0x7d01, 0x008d,
0x05a0, 0x5deb, 0x0478, 0x7d03, 0x0479, 0x7d2c, 0x7c36, 0x0479,
0x7c1f, 0x56ee, 0x0f00, 0x0660, 0x7d05, 0x6509, 0x7e43, 0x620a,
0x7e41, 0x9986, 0x620a, 0x7e3e, 0x6509, 0x7e3c, 0x0512, 0x0512,
0x02ad, 0x0760, 0x7d03, 0x55fb, 0x6dd3, 0x9991, 0x55fb, 0x1d04,
0x6dd3, 0x6ac8, 0x7f2f, 0x1f01, 0x2003, 0x4800, 0x7ce4, 0x99b9,
0x55fb, 0x6dd7, 0x0015, 0x7805, 0x6209, 0x6ac8, 0x6209, 0x6ac8,
0x6dd7, 0x99b8, 0x55fb, 0x6dd7, 0x0015, 0x0015, 0x7805, 0x620a,
0x6ac8, 0x620a, 0x6ac8, 0x6dd7, 0x99b8, 0x55fb, 0x6dd7, 0x0015,
0x0015, 0x0015, 0x7805, 0x620b, 0x6ac8, 0x620b, 0x6ac8, 0x6dd7,
0x7c09, 0x6ddf, 0x7f07, 0x0000, 0x55eb, 0x4d00, 0x7d07, 0xc1fa,
0x57e3, 0x996c, 0x0007, 0x68cc, 0x680c, 0xc213, 0xc20a, 0x9969,
0xc1d9, 0xc1e3, 0x57db, 0x5fe3, 0x57e3, 0x52f3, 0x6a21, 0x008f,
0x00d5, 0x7d01, 0x008d, 0x05a0, 0x5deb, 0x56fb, 0x0478, 0x7d03,
0x0479, 0x7d2a, 0x7c31, 0x0479, 0x7c20, 0x0b70, 0x0311, 0x53eb,
0x0f00, 0x0360, 0x7d05, 0x6509, 0x7e37, 0x620a, 0x7e35, 0x99ec,
0x620a, 0x7e32, 0x6509, 0x7e30, 0x0512, 0x0512, 0x02ad, 0x0760,
0x7c02, 0x5a06, 0x99f4, 0x5a26, 0x7f27, 0x1f01, 0x2003, 0x4800,
0x7ce8, 0x0b70, 0x0311, 0x5313, 0x9a15, 0x0015, 0x7804, 0x6209,
0x5a06, 0x6209, 0x5a26, 0x9a14, 0x0015, 0x0015, 0x7804, 0x620a,
0x5a06, 0x620a, 0x5a26, 0x9a14, 0x0015, 0x0015, 0x0015, 0x7804,
0x620b, 0x5a06, 0x620b, 0x5a26, 0x7c07, 0x0000, 0x55eb, 0x4d00,
0x7d06, 0xc1fa, 0x57e3, 0x99cf, 0x0007, 0x680c, 0xc213, 0xc20a,
0x99cc, 0xc1e3, 0x57db, 0x52fb, 0x6ac3, 0x52f3, 0x6a05, 0x008f,
0x00d5, 0x7d01, 0x008d, 0x05a0, 0x5deb, 0x0478, 0x7d03, 0x0479,
0x7d2b, 0x7c1e, 0x0479, 0x7c33, 0x56ee, 0x0f00, 0x55fb, 0x0760,
0x7d02, 0x6dc3, 0x9a3d, 0x1d04, 0x6dc3, 0x62c8, 0x7e3c, 0x0660,
0x7d02, 0x0210, 0x0212, 0x6a09, 0x7f36, 0x0212, 0x6a09, 0x7f33,
0x0212, 0x6a09, 0x7f30, 0x1f01, 0x2003, 0x4800, 0x7ce7, 0x9a71,
0x55fb, 0x6dc7, 0x0015, 0x0015, 0x0015, 0x7805, 0x62c8, 0x6a0b,
0x62c8, 0x6a0b, 0x6dc7, 0x9a70, 0x55fb, 0x6dc7, 0x0015, 0x0015,
0x7805, 0x62c8, 0x6a0a, 0x62c8, 0x6a0a, 0x6dc7, 0x9a70, 0x55fb,
0x6dc7, 0x0015, 0x7805, 0x62c8, 0x6a09, 0x62c8, 0x6a09, 0x6dc7,
0x7c0a, 0x6a28, 0x57db, 0x7f07, 0x0000, 0x55eb, 0x4d00, 0x7d05,
0xc1fa, 0x57db, 0x9a27, 0xc277, 0x0454, 0xc20a, 0x9a22, 0xc1d9,
0xc1e3, 0x57db, 0x52f3, 0x6a05, 0x008f, 0x00d5, 0x7d01, 0x008d,
0x05a0, 0x56fb, 0x0478, 0x7d03, 0x0479, 0x7d29, 0x7c1f, 0x0479,
0x7c2e, 0x5de3, 0x0d70, 0x0511, 0x55ed, 0x0f00, 0x0760, 0x7d02,
0x5206, 0x9a9b, 0x5226, 0x7e33, 0x0560, 0x7d02, 0x0210, 0x0212,
0x6a09, 0x7f2d, 0x0212, 0x6a09, 0x7f2a, 0x0212, 0x6a09, 0x7f27,
0x1f01, 0x2003, 0x4800, 0x7cea, 0x55e3, 0x9ac6, 0x0015, 0x0015,
0x0015, 0x7804, 0x5206, 0x6a0b, 0x5226, 0x6a0b, 0x9ac5, 0x0015,
0x0015, 0x7804, 0x5206, 0x6a0a, 0x5226, 0x6a0a, 0x9ac5, 0x0015,
0x7804, 0x5206, 0x6a09, 0x5226, 0x6a09, 0x7c09, 0x6a28, 0x7f07,
0x0000, 0x57db, 0x4d00, 0x7d05, 0xc1fa, 0x57db, 0x9a84, 0xc277,
0x0454, 0xc20a, 0x9a81, 0xc1e3, 0x57db, 0x52f3, 0x6ad5, 0x56fb,
0x028e, 0x1a94, 0x6ac3, 0x62c8, 0x0269, 0x7d1e, 0x1e94, 0x6ee3,
0x62d0, 0x5aeb, 0x62c8, 0x0248, 0x6ed3, 0x6ac8, 0x2694, 0x52eb,
0x6ad5, 0x6ee3, 0x62c8, 0x026e, 0x7d27, 0x6ac8, 0x7f23, 0x2501,
0x4d00, 0x7d26, 0x028e, 0x1a98, 0x6ac3, 0x62c8, 0x6ec3, 0x0260,
0x7df1, 0x62d0, 0xc27a, 0x9b18, 0x6ee3, 0x008f, 0x2001, 0x00d5,
0x7d01, 0x008d, 0x05a0, 0x62c8, 0x026e, 0x7d0e, 0x6ac8, 0x7f0a,
0x2001, 0x7cf9, 0x6add, 0x7f06, 0x0000, 0x4d00, 0x7d09, 0xc1fa,
0x57db, 0x9ad7, 0x0007, 0x6aff, 0x62d0, 0xc27a, 0x0458, 0x0454,
0x6add, 0x7ff8, 0xc20a, 0x9ad4, 0xc1d9, 0xc1e3, 0x57db, 0x52f3,
0x6ad5, 0x56fb, 0x028e, 0x1a94, 0x5202, 0x0269, 0x7d17, 0x1e94,
0x5206, 0x0248, 0x5a06, 0x2694, 0x5206, 0x026e, 0x7d26, 0x6ac8,
0x7f22, 0x2501, 0x4d00, 0x7d27, 0x028e, 0x1a98, 0x5202, 0x0260,
0x7df3, 0x6add, 0x7f18, 0x62d0, 0xc27a, 0x9b5b, 0x008f, 0x2001,
0x00d5, 0x7d01, 0x008d, 0x05a0, 0x5206, 0x026e, 0x7d0e, 0x6ac8,
0x7f0a, 0x2001, 0x7cf9, 0x6add, 0x7f06, 0x0000, 0x4d00, 0x7d0b,
0xc1fa, 0x57db, 0x9b21, 0x0007, 0x6aff, 0x6add, 0x7ffc, 0x62d0,
0xc27a, 0x0458, 0x0454, 0x6add, 0x7ff6, 0xc20a, 0x9b1e
};
#endif

