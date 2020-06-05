/* CAN frames contains at most 8 bytes of data, so in order to transmit CSP
 * packets larger than this, a fragmentation protocol is required. The CAN
 * Fragmentation Protocol (CFP) header is designed to match the 29 bit CAN
 * identifier.
 *
 * The CAN identifier is divided in these fields:
 * src:          8 bits
 * dst:          8 bits
 * type:         4 bit
 * remain:       1 bits
 * identifier:   8 bits
 *
 * Source and Destination addresses must match the CSP packet. The type field
 * is used to distinguish the first and subsequent frames in a fragmented CSP
 * packet. Type is BEGIN (0) for the first fragment and MORE (1) for all other
 * fragments. Remain indicates number of remaining fragments, and must be
 * decremented by one for each fragment sent. The identifier field serves the
 * same purpose as in the Internet Protocol, and should be an auto incrementing
 * integer to uniquely separate sessions.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <csp/csp.h>
#include <csp/csp_interface.h>
#include <csp/csp_endian.h>
#include <csp/interfaces/csp_if_can.h>

#include <csp/arch/csp_semaphore.h>
#include <csp/arch/csp_time.h>
#include <csp/arch/csp_queue.h>
#include <csp/arch/csp_thread.h>

#include <csp/drivers/can.h>
#include "common.h"
#include "driver_can.h"
#include "can.h"
#include "csp_if_can.h"
#include "bsp_time.h"
#include "obc_protocal.h"

/* CFP identification number */
static int extended_can_id = 0;
int drop_can_tx_data = 0;
//extern uint32_t clocks_per_msec;

//struct can_rx_action can_process_maps[256];
//static void init_can_process_maps(void);

/* Packet buffers */
typedef enum {
	BUF_FREE = 0,	/* Buffer element free */
	BUF_USED = 1,	/* Buffer element used */
} can_pbuf_state_t;

typedef struct {
	uint16_t rx_count;	/* Received bytes */
	uint32_t cfpid;		/* Connection CFP identification number */
	csp_packet_t *packet;	/* Pointer to packet buffer */
	can_pbuf_state_t state;	/* Element state */
	uint32_t last_used;	/* Timestamp in ms for last use of buffer */
} can_pbuf_element_t;

static can_pbuf_element_t can_pbuf[PBUF_ELEMENTS];

can_info_t  g_can_info_t;

extern uint8_t ty_starss_flag;


uint8_t s_canid_au8[CANDEV_NUM]={GPS_CANID,EXT_MAGTM_CANID,WHEEL_X_CANID,WHEEL_Y_CANID,WHEEL_Z_CANID,WHEEL_A_CANID,STAR_SENSOR_CANID,STAR_SENSOR2_CANID,STAR_SENSOR3_CANID,OPTI_GYRO_CANID};  /*新增设备要将CANID添加至此*/

uint8_t is_canid_valid(uint8_t canid)
{
    uint8_t i;
    uint8_t res = 0;
    for(i=0;i<sizeof(s_canid_au8)/sizeof(uint8_t);i++)
    {
        if(canid == s_canid_au8[i])
        {
            res = 1;
            break;
        }
    }
    return res;
}


static int
can_pbuf_init(void)
{
	/* Initialize packet buffers */
	int i;
	can_pbuf_element_t *buf;

	for (i = 0; i < PBUF_ELEMENTS; i++) {
		buf = can_pbuf + i;
		buf->rx_count = 0;
		buf->cfpid = 0;
		buf->packet = NULL;
		buf->state = BUF_FREE;
		buf->last_used = 0;
	}

	return 0;
}

static void
can_pbuf_timestamp(can_pbuf_element_t *buf)
{
	if (buf != NULL)
		buf->last_used = csp_get_ms();
}

static int
can_pbuf_free(can_pbuf_element_t *buf)
{
	if (buf != NULL) {
		/* Free CSP packet */
		if (buf->packet != NULL)
			csp_buffer_free(buf->packet);

		/* Mark buffer element free */
		buf->packet = NULL;
		buf->state = BUF_FREE;
		buf->rx_count = 0;
		buf->cfpid = 0;
		buf->last_used = 0;
	}

	return 0;
}

static can_pbuf_element_t *
can_pbuf_new(uint32_t id)
{
	int i;
	can_pbuf_element_t *buf, *ret = NULL;

	for (i = 0; i < PBUF_ELEMENTS; i++) {
		buf = can_pbuf + i;
		if (buf->state == BUF_FREE) {
			buf->state = BUF_USED;
			buf->cfpid = id;
			can_pbuf_timestamp(buf);
			ret = buf;
			break;
		}
	}

	return ret;
}

static can_pbuf_element_t *
can_pbuf_find(uint32_t id, uint32_t mask)
{
	int i;
	can_pbuf_element_t *buf, *ret = NULL;

	for (i = 0; i < PBUF_ELEMENTS; i++) {
		buf = can_pbuf + i;

		if ((buf->state == BUF_USED) && ((buf->cfpid & mask) == (id & mask))) {
			can_pbuf_timestamp(buf);
			ret = buf;
			break;
		}
	}

	return ret;
}

void
can_pbuf_cleanup(void)
{
	int i;
	can_pbuf_element_t *buf;

	for (i = 0; i < PBUF_ELEMENTS; i++) {
		buf = can_pbuf + i;

		/* Skip if not used */
		if (buf->state != BUF_USED)
			continue;

		/* Check timeout */
		uint32_t now = csp_get_ms();
		if (now - buf->last_used > PBUF_TIMEOUT_MS) {
			if (buf->packet != NULL) {
				csp_log_warn("CPBUF TIMEOUT ERROR, CANID 0x%x, CFPID 0x%x\n", (unsigned int) CFP_SRC(buf->packet->id.ext), (unsigned int) CFP_ID(buf->packet->id.ext));
			}
			/* Recycle packet buffer */
			can_pbuf_free(buf);
		}
	}
}

uint8_t
get_dst_canid(can_id_t id)
{
	return (CFP_DST(id) & 0xff);
}

int
is_libcsp(can_id_t id)
{
	return CFP_REMAIN(id);
}

uint32_t
get_can_extid(int dst, int type, int libcsp, int cfp_id)
{
	uint32_t id;

	id = 0;
	id |= CFP_MAKE_DST(dst);
	id |= CFP_MAKE_TYPE(type);
	id |= CFP_MAKE_REMAIN(libcsp);
	id |= CFP_MAKE_ID(cfp_id);

	return id;
}

/* EXTENED CAN FRAME */
/* Identification number */
static int
extended_can_id_init(void)
{
	/* Init ID field to random number */
	srand((int)csp_get_ms());
	extended_can_id = rand() & ((1 << CFP_ID_SIZE) - 1);

	return 0;
}

int
extended_can_id_get(void)
{
	int id;

	id = extended_can_id ++;
	extended_can_id = extended_can_id & ((1 << CFP_ID_SIZE) - 1);
	return id;
}


int can_process_frame(can_frame_t *f)
{
	can_pbuf_element_t *buf = NULL;
	uint8_t offset, ext_type;
	int r = 0;
	uint32_t id;
	uint8_t dst_id = 0, src_id = 0, cfp_id = 0;

	if (f == NULL)
		return 1;

	id = f->id;

	/* Bind incoming frame to a packet buffer */
	dst_id = CFP_DST(id);
	src_id = CFP_SRC(id);
	cfp_id = CFP_ID(id);

	/* don't process can frame from myself, or not our business */
	//if (src_id == can_getid() || src_id == own_csp_can_id || (dst_id != own_csp_can_id && dst_id != EPS_CANID))
		//return 1;
    if((src_id != GOM_OBC_CANID)  && (src_id != WHEEL_X_CANID) && (src_id != WHEEL_Y_CANID) && (src_id != WHEEL_Z_CANID) && (src_id != WHEEL_A_CANID))
    {
        return 1;  /*只接收OBC的数据*/
    }
	if ((is_canid_valid(dst_id)) == 0 && (dst_id!=0))
		return 1;
	buf = can_pbuf_find(id, CFP_ID_CONN_MASK);
	/* Check returned buffer */
	ext_type = CFP_TYPE(id);
	if (buf == NULL) {
		if (ext_type == CFP_BEGIN || ext_type == CFP_RESET || ext_type == CFP_TIMESYNC || ext_type == CFP_SINGLE) {
			buf = can_pbuf_new(id);
			if (buf == NULL) {
				csp_log_error("NO FREE CAN PBUF\n");
				csp_if_can.rx_error ++;
				return 1;
			}
		} else {
			csp_log_error("CFP_MORE FRAME WITHOUT PBUF, CANID 0x%x, CFPID 0x%x\n", src_id, cfp_id);
			csp_if_can.frame ++;
			return 1;
		}
	}

	/* Reset frame data offset */
	offset = 0;
	switch (ext_type) {
	case CFP_RESET:
	case CFP_TIMESYNC:
	case CFP_SINGLE:
		if (buf->packet == NULL) {
			buf->packet = csp_buffer_get(PBUF_MTU);
			if (buf->packet == NULL) {
				csp_log_error("NO FREE CSP BUFFER, CANID 0x%x, CFPID 0x%x\n", src_id, cfp_id);
				csp_if_can.frame ++;
				can_pbuf_free(buf);
				break;
			}
		}
		buf->packet->id.ext = id;
		buf->packet->length = f->dlc;
		memcpy(buf->packet->data, f->data, f->dlc);
		buf->rx_count = f->dlc;
		r = 1;
		break;
	case CFP_BEGIN:
		/* Check for incomplete frame */
		if (buf->packet != NULL) {
			/* Reuse the buffer */
			csp_if_can.frame ++;
			csp_buffer_free(buf->packet);
		}

		/* Allocate memory for frame */
		buf->packet = csp_buffer_get(PBUF_MTU);

		if (buf->packet == NULL) {
			csp_log_error("NO FREE CSP BUFFER, CANID 0x%x, CFPID 0x%x\n", src_id, cfp_id);
			csp_if_can.frame ++;
			can_pbuf_free(buf);
			break;
		}

		if (CFP_REMAIN(id)) { /* libcsp packet */
			/* Copy CSP identifier and length*/
			memcpy(&(buf->packet->id), f->data, sizeof(csp_id_t));
			buf->packet->id.ext = csp_ntoh32(buf->packet->id.ext);
			memcpy(&(buf->packet->length), f->data + sizeof(csp_id_t), sizeof(uint16_t));
			buf->packet->length = csp_ntoh16(buf->packet->length);
			/* Set offset to prevent CSP header from being copied to CSP data */
			offset = sizeof(csp_id_t) + sizeof(uint16_t);
		} else { /* normal packet */
			buf->packet->id.ext = id;
			buf->packet->length = (f->data[0] << 8) + f->data[1]; /* network order / big endian */
			///if(((CFP_SRC(id)) == STAR_SENSOR_CANID) && (ty_starss_flag == 1))
				///buf->packet->length += 2; /* 2b length + sumofbuffer byte */
			///else
				buf->packet->length += 3; /* 2b length + sumofbuffer byte */
			offset = 0;
		}

		/* Reset RX count */
		buf->rx_count = 0;

		/* Set offset to prevent CSP header from being copied to CSP data */
		/* FALL THROUGH */
	case CFP_MORE:
		/* double check buf->packet because we may encounter out of order CFP_MORE */
		if (buf->packet == NULL) {
			csp_log_error("CFP_MORE FRAME WITHOUT PBUF, CANID 0x%x, CFPID 0x%x\n", src_id, cfp_id);
			csp_if_can.frame ++;
			can_pbuf_free(buf);
			break;
		}

		/* Check for overflow */
		if ((buf->rx_count + f->dlc - offset) > buf->packet->length) {
			csp_log_error("RX BUFFER OVERFLOW #%d > #%d, CANID 0x%x, CFPID 0x%x\n", buf->rx_count + f->dlc - offset, buf->packet->length, src_id, cfp_id);
			can_pbuf_free(buf);
			csp_if_can.frame++;
			break;
		}

		/* 检查溢出 */
		if ((buf->rx_count + f->dlc - offset) > PBUF_MTU) {
			csp_log_error("RX PBUF_MTU OVERFLOW #%d > #%d, CANID 0x%x, CFPID 0x%x\n", buf->rx_count + f->dlc - offset, buf->packet->length, src_id, cfp_id);
			can_pbuf_free(buf);
			csp_if_can.frame++;
			break;
		}
        
		/* Copy dlc bytes into buffer */
		memcpy(buf->packet->data + buf->rx_count, f->data + offset, f->dlc - offset);
		buf->rx_count += f->dlc - offset;

		/* Check if all data is received */
		if (buf->rx_count >= buf->packet->length) {
			r = 1;
		}
		break;
	default:
		csp_log_error("UNKOWN CAN TYPE 0x%x, CANID 0x%x, CFPID 0x%x\n", ext_type, src_id, cfp_id);
		can_pbuf_free(buf);
		break;
	}

	if (r) {
		/* new data */
        g_can_info_t.frame_num_u8++;
		if (CFP_REMAIN(buf->cfpid)) {
			/* libcsp packet */
			csp_new_packet(buf->packet, &csp_if_can, NULL);
		} else {
			/* normal packet */
			//tianyi_can_handler(buf->packet);  ！！！调用自定义处包处理函数
			if(buf->packet->length < 255)
			{
                //process_eps_can_request(buf->packet);
                /*调试用 等待发送完  用于判断接收包到发送响应的时间间隔*/
            }
			else
			{
				//process_obc_update_request(buf->packet);
			}
            //can_tx_raw_data(CFP_DST(buf->packet->id.ext),CFP_SRC(buf->packet->id.ext),buf->packet->data,buf->packet->length,(cfp_ext_t)CFP_TYPE(buf->packet->id.ext),1,100);
            obc_protocol_handle_data(CFP_DST(buf->packet->id.ext),CFP_SRC(buf->packet->id.ext),buf->packet->data,buf->packet->length);
            csp_buffer_free(buf->packet);
		}
		/* Drop packet buffer reference */
		buf->packet = NULL;
		/* Free packet buffer */
		can_pbuf_free(buf);
	}

	return 0;
}


static int
real_csp_can_tx(csp_iface_t *interface, csp_packet_t *packet, uint32_t timeout, int is_uv)
{
	uint16_t tx_count;
	uint8_t bytes, overhead, avail, dest;
	uint8_t frame_buf[8];
	can_id_t id = 0, id2 = 0;

	/* Get CFP identification number */
	int ident = extended_can_id_get();
	if (ident < 0) {
		return CSP_ERR_INVAL;
	}
	/* Calculate overhead */
	overhead = sizeof(csp_id_t) + sizeof(uint16_t);

	/* Insert destination node mac address into the CFP destination field */
	dest = csp_rtable_find_mac(packet->id.dst);
	if (dest == CSP_NODE_MAC)
		dest = packet->id.dst;

	/* Create CAN identifier */
	id |= CFP_MAKE_SRC(packet->id.src);
	if (is_uv == 0)
		id |= CFP_MAKE_DST(dest);
	else
		id |= CFP_MAKE_DST(UVHF1_CANID); /* SEND TO UV NODE */
	id |= CFP_MAKE_ID(ident);
	id |= CFP_MAKE_REMAIN(1); /* libcsp packet */
	id |= CAN_ID_EXTENDED_FRAME_FORMAT_MASK;

	id2 = id;

	id |= CFP_MAKE_TYPE(CFP_BEGIN);
	/* Calculate first frame data bytes */
	avail = 8 - overhead;
	bytes = (packet->length <= avail) ? packet->length : avail;

	/* Copy CSP headers and data */
	uint32_t csp_id_be = csp_hton32(packet->id.ext);
	uint16_t csp_length_be = csp_hton16(packet->length);

	memcpy(frame_buf, &csp_id_be, sizeof(csp_id_be));
	memcpy(frame_buf + sizeof(csp_id_be), &csp_length_be, sizeof(csp_length_be));
	memcpy(frame_buf + overhead, packet->data, bytes);

	/* Increment tx counter */
	tx_count = bytes;

	/* Send first frame */
	if (can_send(id, frame_buf, overhead + bytes,timeout)) {
		csp_log_error("Failed to send CAN frame to CANID 0x%x in csp_tx_can()\n", id);
		return CSP_ERR_DRIVER;
	}

	/* Send next frames if not complete */
	while (tx_count < packet->length) {
		/* Calculate frame data bytes */
		bytes = (packet->length - tx_count >= 8) ? 8 : packet->length - tx_count;

		/* Prepare identifier */
		id = id2;
		id |= CFP_MAKE_TYPE(CFP_MORE);

		/* Increment tx counter */
		tx_count += bytes;

		/* Send frame */
		if (can_send(id, packet->data + tx_count - bytes, bytes,timeout)) {
			csp_log_error("Failed to send CAN frame in Tx callback\n");
			csp_if_can.tx_error ++;
			return CSP_ERR_DRIVER;
		}
	}

	csp_buffer_free(packet);

	return CSP_ERR_NONE;
}

int
csp_can_tx(csp_iface_t *interface, csp_packet_t *packet, uint32_t timeout)
{
	return real_csp_can_tx(interface, packet, timeout, 0);
}

int
csp_uv_can_tx(csp_iface_t *interface, csp_packet_t *packet, uint32_t timeout)
{
	return real_csp_can_tx(interface, packet, timeout, 1);
}



int csp_can_init(uint8_t mode, struct csp_can_config *conf)
{
    uint32_t mask;
	//init_can_process_maps();
  
	/* Initialize packet buffer */
	if (can_pbuf_init() != 0) {
		csp_log_error("Failed to initialize CAN packet buffers\n");
		return 1;
	}

	/* Initialize CFP identifier */
	if (extended_can_id_init() != 0) {
		csp_log_error("Failed to initialize CAN identification number\n");
		return 1;
	}

	if (mode == CSP_CAN_MASKED) {
		mask = CFP_MAKE_DST((1 << CFP_HOST_SIZE) - 1);
	} else if (mode == CSP_CAN_PROMISC) {
		mask = 0;
	} else {
		csp_log_error("Unknown CAN mode");
		return CSP_ERR_INVAL;
	}

	//own_csp_can_id = own_node;

	/* Initialize CAN driver */
	if (can_init(CFP_MAKE_DST(csp_get_address()), mask, conf) != 0) {
		csp_log_error("Failed to initialize CAN driver");
		return CSP_ERR_DRIVER;
	}

	/* Regsiter interface */
	csp_iflist_add(&csp_if_can);

	return CSP_ERR_NONE;
}



/* 0 成功 1失败
 * 
 */
int8_t  can_tx_raw_data(uint8_t src,uint8_t dest, uint8_t *data, uint16_t len,cfp_ext_t type,uint8_t has_cfpid,uint32_t delay)
{
	uint16_t tx_count;
	uint8_t bytes, overhead=0, avail;
	uint8_t frame_buf[9];
	can_id_t id;
	uint8_t ident;
	if ((data == NULL) || (len > PBUF_MTU) || (len <= 0))
        {
            return 1;
        }

	/* 组CAN ID */
	id = 0;
	id |= CFP_MAKE_SRC(src);  /*源地址*/
	id |= CFP_MAKE_DST(dest);  /*目的地址*/
        if(has_cfpid)
        {
	    /* 获取包ID */
            ident = extended_can_id_get(); /*8位ID*/
	    id |= CFP_MAKE_ID(ident);  /*包序号*/
	    id |= CFP_MAKE_TYPE(type); /*类型*/
	    id |= CFP_MAKE_REMAIN(0);  /* 自定义包 */
	    id |= (uint32_t)((uint32_t)1<<31);
        }


	if ((type == CFP_RESET) || (type == CFP_TIMESYNC) || (type == CFP_SINGLE)) 
        {
            /*单帧*/
          if(len>8)
          {
              return 1;
          }
          else
          {
            
          }
	} 
        else
        {
            /*多帧*/
          if(len<=8)
          {
              return 1;
          }
          else
          {
            
          }
	} 

	/*填充首帧 */
	avail = 8 - overhead;
	bytes = (len <= avail) ? len : avail;
	memcpy(frame_buf + overhead, data,bytes);
	/*已经发送字节数 */
	tx_count = bytes;

	/* 发送第一帧 */
	frame_buf[overhead + bytes] = '\0';
    //id = id | CFP_MAKE_TYPE(CFP_BEGIN);
	if (can_send(id, frame_buf, overhead + bytes,delay)) 
        {
		csp_log_error("Failed to send CAN frame to CANID 0x%x in extended_can_tx()\n", id);
		return 1;
	}

	/* 发送后续帧 */
	while (tx_count < len) 
    {
		/* Calculate frame data bytes */
		bytes = (len - tx_count >= 8) ? 8 : len - tx_count;  /*本帧长度*/
        id &= ~CFP_MAKE_TYPE((uint32_t)(1 << CFP_TYPE_SIZE) - 1);
		id = id | CFP_MAKE_TYPE(CFP_MORE);  /*后续帧*/
		tx_count += bytes;  /*发送计数递增*/

		/* 发送帧 */
		if (can_send(id, data + tx_count - bytes, bytes,delay)) 
                {
			csp_log_error("Failed to send more CAN frames in extended_can_tx()\n");
			return 1;
		}
	}
	return 0;
}

void * can_rx_handle(void * parameters)
{
  static uint8_t s_rx_idlenum =0;
  uint8_t i;
  can_frame_t frame;
  driver_can_data_t tmp_can_recvdata;
  if((int8_t)0 == driver_can_recv(&tmp_can_recvdata, (uint32_t)1000))
  {
    frame.id = tmp_can_recvdata.id_u32;
    if(tmp_can_recvdata.len_u8 > (uint8_t)8)
    {
      frame.dlc = (uint8_t)8;
    }
    else
    {
      frame.dlc = tmp_can_recvdata.len_u8;
    }
    for(i=0;i<frame.dlc;i++)
    {
        frame.data[i] = tmp_can_recvdata.buff_au8[i];
    
    }
    /* Call RX callback */
    can_process_frame(&frame);
    s_rx_idlenum=0;
  }
  else
  {
    s_rx_idlenum++;
    if(s_rx_idlenum>=10)
    {
      s_rx_idlenum=0;
      can_pbuf_cleanup();
    }
  }
  return 0;
}

int can_send(can_id_t id, uint8_t data[], uint8_t dlc,uint32_t delay)
{
  driver_can_data_t senddata;
  uint8_t i;
  uint8_t tries = 10;
  int8_t res;
  if (dlc > 8)
  {
    return -1;
  }
  else
  {
    senddata.type_u8 = 0x02;
    senddata.len_u8 = dlc;
    senddata.id_u32 = id;
    for(i=0;i<dlc;i++)
    {
      senddata.buff_au8[i] = data[i];
    }
    do
    {
      res = driver_can_send(senddata, delay);
    }
    while((res != 0) && (tries--));
    return res;
  }
}

int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf)
{
  driver_can_init(conf->bitrate);
  //driver_can_filter(0,0,id,mask);
  //driver_can_filter(0,0,(uint32_t)can_getid()<<13,(uint32_t)0xFF<<13);
  //driver_can_filter(1,0,(uint32_t)can_getid()<<19,(uint32_t)0x1F<<19);
  driver_can_filter(0, 0, (uint32_t)can_getid() << 13, (uint32_t)0xFF << 13);
  return 0;
}




/** Interface definition */
csp_iface_t csp_if_can = {
	.name = "CAN",
	.nexthop = csp_can_tx,
	.mtu = PBUF_MTU,
};

/** Interface definition */
csp_iface_t csp_if_uv = {
	.name = "UV",
	.nexthop = csp_uv_can_tx,
	.mtu = PBUF_MTU,
};
