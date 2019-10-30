#ifndef __TCP_SERVER_DEMO_H
#define __TCP_SERVER_DEMO_H
#include "sys.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip_comm.h"
//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌĞòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßĞí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK STM32F407¿ª·¢°å
//TCP Server ²âÊÔ´úÂë	   
//ÕıµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//´´½¨ÈÕÆÚ:2014/8/15
//°æ±¾£ºV1.0
//°æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//ĞŞ¸ÄĞÅÏ¢
//ÎŞ
////////////////////////////////////////////////////////////////////////////////// 	   
 
//#define TCP_SERVER_RX_BUFSIZE	2000		//¶¨Òåtcp server×î´ó½ÓÊÕÊı¾İ³¤¶È
#define TCP_SERVER_RX_BUFSIZE	36		//¶¨Òåtcp server×î´ó½ÓÊÕÊı¾İ³¤¶
#define TCP_SERVER_PORT			8088	//¶¨Òåtcp serverµÄ¶Ë¿Ú
 



//tcp·şÎñÆ÷Á¬½Ó×´Ì¬
enum tcp_server_states
{
	ES_TCPSERVER_NONE = 0,		//Ã»ÓĞÁ¬½Ó
	ES_TCPSERVER_ACCEPTED,		//ÓĞ¿Í»§¶ËÁ¬½ÓÉÏÁË 
	ES_TCPSERVER_CLOSING,		//¼´½«¹Ø±ÕÁ¬½Ó
}; 
//LWIP»Øµ÷º¯ÊıÊ¹ÓÃµÄ½á¹¹Ìå
struct tcp_server_struct
{
	u8 state;               //µ±Ç°Á¬½Ó×´
	struct tcp_pcb *pcb;    //Ö¸Ïòµ±Ç°µÄpcb
	struct pbuf *p;         //Ö¸Ïò½ÓÊÕ/»ò´«ÊäµÄpbuf
}; 

void tcpserverInit(void);
bool tcpslkGetDataWithTimout(u8 *c);
void set_tcpclient_connected_status (bool flag);
u8 get_tcpclient_connected_status(void);
void tcpServerTask(void *param);
void tcp_server_test(void);//TCP Server²âÊÔº¯Êı
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void tcp_server_error(void *arg,err_t err);
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
void TcpServerSendDataBlocking(u32 dataSize, u8 *sendBuffer);
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_server_senddata(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
void tcp_server_remove_timewait(void);
#endif 
