#include "tcp_server.h" 
#include "delay.h"
#include "usart.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"  


/*FreeRTOSÏà¹ØÍ·ÎÄ¼ş*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

#define TCPSLK_DATA_TIMEOUT_MS 	1000
#define TCPSLK_DATA_TIMEOUT_TICKS (TCPSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)

//TCP Server½ÓÊÕÊı¾İ»º³åÇø
u8 tcp_server_recvbuf[TCP_SERVER_RX_BUFSIZE];	
//TCP·şÎñÆ÷·¢ËÍÊı¾İÄÚÈİ
//const u8 *tcp_server_sendbuf="Explorer STM32F407 TCP Server send data\r\n";

//TCP Server ²âÊÔÈ«¾Ö×´Ì¬±ê¼Ç±äÁ¿
//bit7:0,Ã»ÓĞÊı¾İÒª·¢ËÍ;1,ÓĞÊı¾İÒª·¢ËÍ
//bit6:0,Ã»ÓĞÊÕµ½Êı¾İ;1,ÊÕµ½Êı¾İÁË.
//bit5:0,Ã»ÓĞ¿Í»§¶ËÁ¬½ÓÉÏ;1,ÓĞ¿Í»§¶ËÁ¬½ÓÉÏÁË.
//bit4~0:±£Áô
u8 tcp_server_flag;	 
struct tcp_server_struct *tcp_echoserver_es;
static struct tcp_pcb *tcp_echoserver_pcb;

//static xSemaphoreHandle tcpwaitUntilSendDone;
static xSemaphoreHandle tcpBusy;
static xQueueHandle tcpslkDataDelivery;
static bool isInit = false;
static bool tcp_client_status=0;			//÷

void tcpserverInit(void)
{
	if (isInit) return;
	
//	tcpwaitUntilSendDone = xSemaphoreCreateBinary(); 	/*µÈ´ı·¢ËÍÍê³É ¶şÖµĞÅºÅÁ¿*/
	tcpBusy = xSemaphoreCreateBinary();			/*´®¿ÚÃ¦ ¶şÖµĞÅºÅÁ¿*/
	xSemaphoreGive(tcpBusy); 
	
	tcpslkDataDelivery = xQueueCreate(128, sizeof(u8));	/*¶ÓÁĞ 1024¸öÏûÏ¢*/
	ASSERT(tcpslkDataDelivery);
	
	isInit = true;
}

bool tcpslkGetDataWithTimout(u8 *c)
{
	/*?¨®¨º?uartslkDataDelivery(1024??¨¨Y¨¢?)???¡é*/
	if (xQueueReceive(tcpslkDataDelivery, c, TCPSLK_DATA_TIMEOUT_TICKS) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}


void set_tcpclient_connected_status (bool flag)
{
	tcp_client_status=flag;
}


u8 get_tcpclient_connected_status(void)
{
	return tcp_client_status;
}


void tcpServerTask(void *param)
{
	if(get_lwip_init_flag() == 0) {	
		printf("lwip init fail! \n");

		while(lwip_comm_init()) //lwip³õÊ¼»¯
		{
			printf("LWIP Init Retrying... \n");  	
			vTaskDelay(3000);			
		}	
	}

	set_lwip_init_flag(1);		 	
#if LWIP_DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//µÈ´ıDHCP»ñÈ¡³É¹¦/³¬Ê±Òç³ö
	{
		lwip_periodic_handle();
	}
#endif 	

		
	while(1){
	
		if((tcp_server_flag & 1<<6)) 
			printf("TCPÁ¬½ÓÒÑ¾­½¨Á¢,²»ÄÜÖØ¸´Á¬½Ó\r\n");	//Èç¹ûÁ¬½Ó³É¹¦,²»×öÈÎºÎ´¦Àí
		else 
			tcp_server_test();		//µ±¶Ï¿ªÁ¬½Óºó,µ÷ÓÃtcp_server_test()º¯Êı
		vTaskDelay(10);
	}
}
 
//TCP Server ²âÊÔ
void tcp_server_test(void)
{
	err_t err;  
	struct tcp_pcb *tcppcbnew;  	//¶¨ÒåÒ»¸öTCP·şÎñÆ÷¿ØÖÆ¿é
	struct tcp_pcb *tcppcbconn;  	//¶¨ÒåÒ»¸öTCP·şÎñÆ÷¿ØÖÆ¿é
	
	u8 *tbuf;
	u8 res=0;		
	u8 t=0; 
	u8 connflag=0;		//Á¬½Ó±ê¼Ç
	
	tbuf=mymalloc(SRAMIN,200);	//ÉêÇëÄÚ´æ
	if(tbuf==NULL)return ;		//ÄÚ´æÉêÇëÊ§°ÜÁË,Ö±½ÓÍË³ö	
	printf("Server IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//·şÎñÆ÷IP
	printf("Server Port:%d \n",TCP_SERVER_PORT);//·şÎñÆ÷¶Ë¿ÚºÅ

	tcppcbnew=tcp_new();	//´´½¨Ò»¸öĞÂµÄpcb
	if(tcppcbnew)			//´´½¨³É¹¦
	{ 
		err=tcp_bind(tcppcbnew,IP_ADDR_ANY,TCP_SERVER_PORT);	//½«±¾µØIPÓëÖ¸¶¨µÄ¶Ë¿ÚºÅ°ó¶¨ÔÚÒ»Æğ,IP_ADDR_ANYÎª°ó¶¨±¾µØËùÓĞµÄIPµØÖ·
		if(err==ERR_OK)	//°ó¶¨Íê³É
		{
			tcppcbconn=tcp_listen(tcppcbnew); 			//ÉèÖÃtcppcb½øÈë¼àÌı×´Ì¬
			tcp_echoserver_pcb = tcppcbconn;
			tcp_accept(tcppcbconn,tcp_server_accept); 	//³õÊ¼»¯LWIPµÄtcp_acceptµÄ»Øµ÷º¯Êı
		}else res=1;  
	}else res=1;

	while(res==0)
	{	
		if(tcp_server_flag&1<<6)//ÊÇ·ñÊÕµ½Êı¾İ?
		{
//			printf("%s",tcp_server_recvbuf);//ÏÔÊ¾½ÓÊÕµ½µÄÊı¾
			tcp_server_flag&=~(1<<6);//±ê¼ÇÊı¾İÒÑ¾­±»´¦ÀíÁË.
			tcp_server_flag|=1<<7;//±ê¼ÇÒª·¢ËÍÊı¾İ
		}
		if(tcp_server_flag&1<<5)//ÊÇ·ñÁ¬½ÓÉÏ?
		{
			if(connflag==0)
			{ 
				printf("Client IP:%d.%d.%d.%d \n",lwipdev.remoteip[0],lwipdev.remoteip[1],lwipdev.remoteip[2],lwipdev.remoteip[3]);//¿Í»§¶ËIP
				printf("Receive Data: \n");//ÌáÊ¾ÏûÏ
				set_tcpclient_connected_status (1);
				connflag=1;//±ê¼ÇÁ¬½ÓÁË
			} 
		}else if(connflag)
		{
			connflag=0;	//±ê¼ÇÁ¬½Ó¶Ï¿ªÁË
			printf("Disconnect...\n");
		}
		lwip_periodic_handle();
		delay_ms(2);
		t++;
		if(t==200)
		{
			t=0;
		} 
	}   
	tcp_server_connection_close(tcppcbnew,0);//¹Ø±ÕTCP ServerÁ¬½Ó
	tcp_server_connection_close(tcppcbconn,0);//¹Ø±ÕTCP ServerÁ¬½+Ó 
	tcp_server_remove_timewait(); 
	memset(tcppcbnew,0,sizeof(struct tcp_pcb));
	memset(tcppcbconn,0,sizeof(struct tcp_pcb)); 
	myfree(SRAMIN,tbuf);
	set_tcpclient_connected_status (0);
	printf("Connect break! \n");  
} 
//lwIP tcp_accept()µÄ»Øµ÷º¯Êı
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err)
{
	err_t ret_err;
	struct tcp_server_struct *es; 
 	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);
	tcp_setprio(newpcb,TCP_PRIO_MIN);//ÉèÖÃĞÂ´´½¨µÄpcbÓÅÏÈ¼¶
	es=(struct tcp_server_struct*)mem_malloc(sizeof(struct tcp_server_struct)); //·ÖÅäÄÚ´æ

	tcp_echoserver_es=es;
	tcp_echoserver_pcb = newpcb;
	
 	if(es!=NULL) //ÄÚ´æ·ÖÅä³É¹¦
	{
		es->state=ES_TCPSERVER_ACCEPTED;  	//½ÓÊÕÁ¬½Ó
		es->pcb=newpcb;
		es->p=NULL;
		
		tcp_arg(newpcb,es);
		tcp_recv(newpcb,tcp_server_recv);	//³õÊ¼»¯tcp_recv()µÄ»Øµ÷º¯Êı
		tcp_err(newpcb,tcp_server_error); 	//³õÊ¼»¯tcp_err()»Øµ÷º¯Êı
		tcp_poll(newpcb,tcp_server_poll,1);	//³õÊ¼»¯tcp_poll»Øµ÷º¯Êı
		tcp_sent(newpcb,tcp_server_sent);  	//³õÊ¼»¯·¢ËÍ»Øµ÷º¯Êı
		printf("tcp_server_accept \n ");//ÌáÊ¾ÏûÏ
		  
		tcp_server_flag|=1<<5;				//±ê¼ÇÓĞ¿Í»§¶ËÁ¬ÉÏÁË
		lwipdev.remoteip[0]=newpcb->remote_ip.addr&0xff; 		//IADDR4
		lwipdev.remoteip[1]=(newpcb->remote_ip.addr>>8)&0xff;  	//IADDR3
		lwipdev.remoteip[2]=(newpcb->remote_ip.addr>>16)&0xff; 	//IADDR2
		lwipdev.remoteip[3]=(newpcb->remote_ip.addr>>24)&0xff; 	//IADDR1 
		ret_err=ERR_OK;
	}else ret_err=ERR_MEM;
	return ret_err;
}
//lwIP tcp_recv()º¯ÊıµÄ»Øµ÷º¯Êı
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	err_t ret_err;
//	u32 data_len = 0;
	struct pbuf *q;
  	struct tcp_server_struct *es;
	LWIP_ASSERT("arg != NULL",arg != NULL);
	es=(struct tcp_server_struct *)arg;
	if(p==NULL) //´Ó¿Í»§¶Ë½ÓÊÕµ½¿ÕÊı¾İ
	{
		es->state=ES_TCPSERVER_CLOSING;//ĞèÒª¹Ø±ÕTCP Á¬½ÓÁË
		es->p=p; 
		ret_err=ERR_OK;
	}else if(err!=ERR_OK)	//´Ó¿Í»§¶Ë½ÓÊÕµ½Ò»¸ö·Ç¿ÕÊı¾İ,µ«ÊÇÓÉÓÚÄ³ÖÖÔ­Òòerr!=ERR_OK
	{
		if(p)pbuf_free(p);	//ÊÍ·Å½ÓÊÕpbuf
		ret_err=err;
	}else if(es->state==ES_TCPSERVER_ACCEPTED) 	//´¦ÓÚÁ¬½Ó×´Ì¬
	{
		if(p!=NULL)  //µ±´¦ÓÚÁ¬½Ó×´Ì¬²¢ÇÒ½ÓÊÕµ½µÄÊı¾İ²»Îª¿ÕÊ±½«Æä´òÓ¡³öÀ´
		{
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
			u8 *data;
		
			for(q=p;q!=NULL;q=q->next)  //±éÀúÍêÕû¸öpbufÁ´±í
			{
				data = q->payload;
				for(int i=0; i<q->len; i++)
				{
					u8 c = *(data + i);	
//					printf("%0x, ",c); 	
					xQueueSendFromISR(tcpslkDataDelivery, &c, &xHigherPriorityTaskWoken);
				}
//				printf("\n"); 	
			}
			tcp_server_flag|=1<<6;	//±ê¼Ç½ÓÊÕµ½Êı¾İÁË
			lwipdev.remoteip[0]=tpcb->remote_ip.addr&0xff; 		//IADDR4
			lwipdev.remoteip[1]=(tpcb->remote_ip.addr>>8)&0xff; //IADDR3
			lwipdev.remoteip[2]=(tpcb->remote_ip.addr>>16)&0xff;//IADDR2
			lwipdev.remoteip[3]=(tpcb->remote_ip.addr>>24)&0xff;//IADDR1 
 			tcp_recved(tpcb,p->tot_len);//ÓÃÓÚ»ñÈ¡½ÓÊÕÊı¾İ,Í¨ÖªLWIP¿ÉÒÔ»ñÈ¡¸ü¶àÊı¾İ
			pbuf_free(p);  	//ÊÍ·ÅÄÚ´æ
			ret_err=ERR_OK;
		}
	}else//·şÎñÆ÷¹Ø±ÕÁË
	{
		tcp_recved(tpcb,p->tot_len);//ÓÃÓÚ»ñÈ¡½ÓÊÕÊı¾İ,Í¨ÖªLWIP¿ÉÒÔ»ñÈ¡¸ü¶àÊı¾İ
		es->p=NULL;
		pbuf_free(p); //ÊÍ·ÅÄÚ´æ
		ret_err=ERR_OK;
	}
	return ret_err;
}
//lwIP tcp_errº¯ÊıµÄ»Øµ÷º¯Êı
void tcp_server_error(void *arg,err_t err)
{  
	LWIP_UNUSED_ARG(err);  
	printf("tcp error:%x\r\n",(u32)arg);
	if(arg!=NULL)mem_free(arg);//ÊÍ·ÅÄÚ´æ
} 
//lwIP tcp_pollµÄ»Øµ÷º¯Êı
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
	err_t ret_err;
	struct tcp_server_struct *es; 
	es=(struct tcp_server_struct *)arg; 
	if(es!=NULL)
	{
		if(tcp_server_flag&(1<<7))	//ÅĞ¶ÏÊÇ·ñÓĞÊı¾İÒª·¢ËÍ
		{
			tcp_server_flag&=~(1<<7);  			//Çå³ıÊı¾İ·¢ËÍ±êÖ¾Î»
			tcp_echoserver_es = es;
			tcp_echoserver_pcb = tpcb;

		}else if(es->state==ES_TCPSERVER_CLOSING)//ĞèÒª¹Ø±ÕÁ¬½Ó?Ö´ĞĞ¹Ø±Õ²Ù×÷
		{
			tcp_server_connection_close(tpcb,es);//¹Ø±ÕÁ¬½Ó
			set_tcpclient_connected_status (0);
			printf("tcpserver closing!... \n");  
		}
		ret_err=ERR_OK;
	}else
	{
		tcp_abort(tpcb);//ÖÕÖ¹Á¬½Ó,É¾³ıpcb¿ØÖÆ¿é
		ret_err=ERR_ABRT; 
	}
	return ret_err;
} 

void TcpServerSendDataBlocking(u32 dataSize,u8 *sendBuffer)
{
//	if(tcp_server_flag&(1<<4))
	{
		struct tcp_server_struct *es = tcp_echoserver_es; 
		struct tcp_pcb *tpcb = tcp_echoserver_pcb;
		
		es->p=pbuf_alloc(PBUF_TRANSPORT,dataSize,PBUF_POOL);//ÉêÇëÄÚ´æ
		pbuf_take(es->p,(char*)sendBuffer,dataSize);
		tcp_server_senddata(tpcb,es); 		//ÂÖÑ¯µÄÊ±ºò·¢ËÍÒª·¢ËÍµÄÊı¾İ
//		tcp_server_flag&=~(1<<4);  			//Çå³ıÊı¾İ·¢ËÍ±êÖ¾Î»
		if(es->p!=NULL)pbuf_free(es->p); 	//ÊÍ·ÅÄÚ´æ	
//		printf("TcpServerSendDataBlocking......! \r\n");		
	}

}


//lwIP tcp_sentµÄ»Øµ÷º¯Êı(µ±´ÓÔ¶¶ËÖ÷»ú½ÓÊÕµ½ACKĞÅºÅºó·¢ËÍÊı¾İ)
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	struct tcp_server_struct *es;
	LWIP_UNUSED_ARG(len); 
	es = (struct tcp_server_struct *) arg;
	if(es->p)tcp_server_senddata(tpcb,es);//·¢ËÍÊı¾İ
	return ERR_OK;
} 
//´Ëº¯ÊıÓÃÀ´·¢ËÍÊı¾İ
void tcp_server_senddata(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
	struct pbuf *ptr;
	u16 plen;
	err_t wr_err=ERR_OK;
	 while((wr_err==ERR_OK)&&es->p&&(es->p->len<=tcp_sndbuf(tpcb)))
	 {
		ptr=es->p;
		wr_err=tcp_write(tpcb,ptr->payload,ptr->len,1);
		if(wr_err==ERR_OK)
		{ 
			plen=ptr->len;
			es->p=ptr->next;			//Ö¸ÏòÏÂÒ»¸öpbuf
			if(es->p)pbuf_ref(es->p);	//pbufµÄref¼ÓÒ»
			pbuf_free(ptr);
			tcp_recved(tpcb,plen); 		//¸üĞÂtcp´°¿Ú´óĞ¡
		}else if(wr_err==ERR_MEM)es->p=ptr;
	 }
} 
//¹Ø±ÕtcpÁ¬½Ó
void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
	tcp_close(tpcb);
	tcp_arg(tpcb,NULL);
	tcp_sent(tpcb,NULL);
	tcp_recv(tpcb,NULL);
	tcp_err(tpcb,NULL);
	tcp_poll(tpcb,NULL,0);
	if(es)mem_free(es); 
	tcp_server_flag&=~(1<<5);//±ê¼ÇÁ¬½Ó¶Ï¿ªÁË
}
extern void tcp_pcb_purge(struct tcp_pcb *pcb);	//ÔÚ tcp.cÀïÃæ 
extern struct tcp_pcb *tcp_active_pcbs;			//ÔÚ tcp.cÀïÃæ 
extern struct tcp_pcb *tcp_tw_pcbs;				//ÔÚ tcp.cÀïÃæ  
//Ç¿ÖÆÉ¾³ıTCP ServerÖ÷¶¯¶Ï¿ªÊ±µÄtime wait
void tcp_server_remove_timewait(void)
{
	struct tcp_pcb *pcb,*pcb2; 
	u8 t=0;
	while(tcp_active_pcbs!=NULL&&t<200)
	{
		lwip_periodic_handle();	//¼ÌĞøÂÖÑ¯
		t++;
		delay_ms(10);			//µÈ´ıtcp_active_pcbsÎª¿Õ  
	}
	pcb=tcp_tw_pcbs;
	while(pcb!=NULL)//Èç¹ûÓĞµÈ´ı×´Ì¬µÄpcbs
	{
		tcp_pcb_purge(pcb); 
		tcp_tw_pcbs=pcb->next;
		pcb2=pcb;
		pcb=pcb->next;
		memp_free(MEMP_TCP_PCB,pcb2);	
	}
}




































