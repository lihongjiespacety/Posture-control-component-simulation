/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _CAN_H_
#define _CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <csp/csp.h>
#include <csp/interfaces/csp_if_can.h>

/* The can_frame_t and can_id_t types intentionally matches the
 * can_frame struct and can_id types in include/linux/can.h
 */

/** CAN Identifier */
typedef uint32_t can_id_t;

/** CAN Frame */
typedef struct {
	/** 32 bit CAN identifier */
	can_id_t id;
	/** Data Length Code */
	uint8_t dlc;
	/**< Frame Data - 0 to 8 bytes */
	union __attribute__((aligned(8))) {
		uint8_t data[8];
		uint16_t data16[4];
		uint32_t data32[2];
	};
} can_frame_t;

typedef enum  {
	CFP_RESET = 0,
	CFP_SINGLE = 1,
	CFP_BEGIN = 2,
	CFP_MORE = 3,
	CFP_TIMESYNC = 4
}cfp_ext_t;

typedef enum
{
	GOM_OBC_CANID = 0,
	GOM_UHF_CANID,
	GPS_CANID,
	POWER_BOARD_CANID, /* 平台电源管理板 id */
	PL_XWAVE_CANID,
	UVHF1_CANID, 
	EXT_MAGTM_CANID,  /* 0x6 */
	UVHF2_CANID,
	WHEEL_X_CANID,
	WHEEL_Y_CANID,
	WHEEL_Z_CANID, /* 0xA */
	WHEEL_A_CANID,
	STAR_SENSOR_CANID, /*0xC 天银星敏1*/
	OPTI_GYRO_CANID,  /* 光纤陀螺 id */
	CAN_CSP_CANID,
	UNUSED_F, /* 0xF */
	JKB_CANID, /* 0x10 */
	PLC_CANID,
	ANTENNA_CANID,
	GNSS_CANID, /* FINISHED TY-6 */
	STAR_SENSOR2_CANID, /* 0x14 天银星敏2*/
	STAR_SENSOR3_CANID, /* 0x15 天银星敏3*/
	GXCW_CANID, /* FINISHED TY-2 */
	UNUSED1_CANID,
	SDR_CANID, /* FINISHED TY-2 */
	TSINGHUA_XRAY_CANID,
	GRAYSCALE_CAMERA_CANID, /* 0x1A */
	AIS_CANID,
	TSINGHUA_GRID_CANID,
	ADS_B_CANID,
	SAT_UHF_CANID,
	UNUSED_CANID_2, /* FINISHED, TY-6 */
	IOT_CANID, /* 0x20, FINISHED, TY-6 */
	AMAT_CANID,
	FRENCH_THRUSTER_CANID, /* UNUSED  0x22, to use 0x0b later */
	DCS_CANID,
	UNUSED_CANID_4,
	RADIO_EDUCATION_CANID, /* 0x25 */
	LASER_CANID,
	THRUSTER_CANID,
	UNUSED_CANID_6,
	RASPBERRY_PI_CANID = 0x29,
	TP0805_OBC_CANID, /* 0X2A */
	TP0805_PWR_CANID,
	U_ANT_1_CANID,
	U_ANT_2_CANID,
	V_ANT_CANID, /* 0x2E */
	EARTH_SENSOR_CANID = 0x2F,
	/* UNUSED .... */
	AI_CANID = 0x30,
	MICRO_THRUSTER_CANID = 0x31,
	PL1_POWER_BOARD_CANID = 0x32,   /* 载荷电源管理板1 id */
	PL2_POWER_BOARD_CANID = 0x33,   /* 载荷电源管理板2 id */
	PL3_POWER_BOARD_CANID = 0x34,   /* 载荷电源管理板3 id */
	PL4_POWER_BOARD_CANID = 0x35,   /* 载荷电源管理板4 id */
	PL5_POWER_BOARD_CANID = 0x36,   /* 载荷电源管理板5 id */
	SAR_CANID = 0x37,
	/* VIRTUAL DEVICES */
	ACS_CANID = 0x7E,
	FW_UPDATER_CANID = 0x7F,
	RASPBERRY_CANID = 0x80,
} gom_obc_can_id;

#define CANDEV_NUM 8
extern uint8_t s_canid_au8[CANDEV_NUM];
typedef enum {
	CAN_ERROR = 0,
	CAN_NO_ERROR = 1,
} can_error_t;

#define PBUF_MTU 465 /**< GPS包465 */

/* EXTENDED FRAME DEFINITIONS */
#define CFP_HOST_SIZE		8
#define CFP_TYPE_SIZE		4
#define CFP_REMAIN_SIZE		1
#define CFP_ID_SIZE		    8

/* Macros for extracting header fields */
#define CFP_FIELD(id,rsiz,fsiz) ((uint32_t)((uint32_t)((id) >> (rsiz)) & (uint32_t)((1 << (fsiz)) - 1)))
#define CFP_SRC(id)		CFP_FIELD(id, CFP_HOST_SIZE + CFP_TYPE_SIZE + CFP_REMAIN_SIZE + CFP_ID_SIZE, CFP_HOST_SIZE)
#define CFP_DST(id)		CFP_FIELD(id, CFP_TYPE_SIZE + CFP_REMAIN_SIZE + CFP_ID_SIZE, CFP_HOST_SIZE)
#define CFP_TYPE(id)		CFP_FIELD(id, CFP_REMAIN_SIZE + CFP_ID_SIZE, CFP_TYPE_SIZE)
#define CFP_REMAIN(id)		CFP_FIELD(id, CFP_ID_SIZE, CFP_REMAIN_SIZE)
#define CFP_ID(id)		CFP_FIELD(id, 0, CFP_ID_SIZE)

/* Macros for building CFP headers */
#define CFP_MAKE_FIELD(id,fsiz,rsiz) ((uint32_t)(((id) & (uint32_t)((uint32_t)(1 << (fsiz)) - 1)) << (rsiz)))
#define CFP_MAKE_SRC(id)	CFP_MAKE_FIELD(id, CFP_HOST_SIZE, CFP_HOST_SIZE + CFP_TYPE_SIZE + CFP_REMAIN_SIZE + CFP_ID_SIZE)
#define CFP_MAKE_DST(id)	CFP_MAKE_FIELD(id, CFP_HOST_SIZE, CFP_TYPE_SIZE + CFP_REMAIN_SIZE + CFP_ID_SIZE)
#define CFP_MAKE_TYPE(id)	CFP_MAKE_FIELD(id, CFP_TYPE_SIZE, CFP_REMAIN_SIZE + CFP_ID_SIZE)
#define CFP_MAKE_REMAIN(id)	CFP_MAKE_FIELD(id, CFP_REMAIN_SIZE, CFP_ID_SIZE)
#define CFP_MAKE_ID(id)		CFP_MAKE_FIELD(id, CFP_ID_SIZE, 0)

/* Mask to uniquely separate connections */
#define CFP_ID_CONN_MASK	(CFP_MAKE_SRC((uint32_t)(1 << CFP_HOST_SIZE) - 1) | \
				 CFP_MAKE_DST((uint32_t)(1 << CFP_HOST_SIZE) - 1) | \
				 CFP_MAKE_REMAIN((uint32_t)(1 << CFP_REMAIN_SIZE) - 1) | \
				 CFP_MAKE_ID((uint32_t)(1 << CFP_ID_SIZE) - 1))

/* Maximum Transmission Unit for CSP over CAN */
#define CSP_CAN_MTU		256

/* Maximum number of frames in RX queue */
#define CSP_CAN_RX_QUEUE_SIZE	100
/* Number of packet buffer elements */
#define PBUF_ELEMENTS		CSP_CONN_MAX
/* Buffer element timeout in ms */
#define PBUF_TIMEOUT_MS		5000

#define CAN_ID_EXTENDED_FRAME_FORMAT_MASK 0x80000000

//int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf);
void can_pbuf_cleanup(void);
int can_process_frame(can_frame_t *f);
int process_eps_can_request(csp_packet_t* in);
int process_obc_update_request(csp_packet_t* in);
int tianyi_fw_can_tx(uint8_t *b, int len, int dst_id, int delay, int has_cfpid);
int csp_can_tx(csp_iface_t *interface, csp_packet_t *packet, uint32_t timeout);

/* 0 成功 1失败
 * 
 */
int8_t  can_tx_raw_data(uint8_t src,uint8_t dest, uint8_t *data, uint16_t len,cfp_ext_t type,uint8_t has_cfpid,uint32_t delay);

void csp_can_pbuf_cleanup(void);
int csp_can_init(uint8_t mode, struct csp_can_config *conf);

int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf);
int can_send(can_id_t id, uint8_t * data, uint8_t dlc,uint32_t delay);

int csp_can_rx_frame(can_frame_t *frame, CSP_BASE_TYPE *task_woken);

typedef __packed struct can_info
{
    uint8_t  req_cnt_u8;
    uint8_t  frm_ok_u8;
    uint8_t  frm_err_u8;
    uint8_t  last_cmd_u8;
    uint8_t  frame_num_u8;
} can_info_t;

extern can_info_t  g_can_info_t;
void* can_rx_handle(void* parameters);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CAN_H_ */
