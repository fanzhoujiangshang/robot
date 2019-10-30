#include "lwip_comm.h" 
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "ethernetif.h" 
#include "lwip/timers.h"
#include "lwip/tcp_impl.h"
#include "lwip/ip_frag.h"
#include "lwip/tcpip.h" 
#include "malloc.h"
#include "delay.h"
#include "usart.h"  
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌÐòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßÐí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK STM32F407¿ª·¢°å
//lwipÍ¨ÓÃÇý¶¯ ´úÂë	   
//ÕýµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//´´½¨ÈÕÆÚ:2014/8/15
//°æ±¾£ºV1.0
//°æÈ¨ËùÓÐ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//ÐÞ¸ÄÐÅÏ¢
//ÎÞ
////////////////////////////////////////////////////////////////////////////////// 	   
   
__lwip_dev lwipdev;						//lwip¿ØÖÆ½á¹¹Ìå 
struct netif lwip_netif;				//¶¨ÒåÒ»¸öÈ«¾ÖµÄÍøÂç½Ó¿Ú

extern u32 memp_get_memorysize(void);	//ÔÚmemp.cÀïÃæ¶¨Òå
extern u8_t *memp_memory;				//ÔÚmemp.cÀïÃæ¶¨Òå.
extern u8_t *ram_heap;					//ÔÚmem.cÀïÃæ¶¨Òå.

u32 TCPTimer=0;			//TCP²éÑ¯¼ÆÊ±Æ÷
u32 ARPTimer=0;			//ARP²éÑ¯¼ÆÊ±Æ÷
u32 lwip_localtime;		//lwip±¾µØÊ±¼ä¼ÆÊýÆ÷,µ¥Î»:ms

#if LWIP_DHCP
u32 DHCPfineTimer=0;	//DHCP¾«Ï¸´¦Àí¼ÆÊ±Æ÷
u32 DHCPcoarseTimer=0;	//DHCP´Ö²Ú´¦Àí¼ÆÊ±Æ÷
#endif

bool lwip_init_flag=0;			//÷

void set_lwip_init_flag (bool flag)
{
	lwip_init_flag=flag;
}


u8 get_lwip_init_flag(void)
{
	return lwip_init_flag;
}


//lwipÖÐmemºÍmempµÄÄÚ´æÉêÇë
//·µ»ØÖµ:0,³É¹¦;
//    ÆäËû,Ê§°Ü
u8 lwip_comm_mem_malloc(void)
{
	u32 mempsize;
	u32 ramheapsize; 
	mempsize=memp_get_memorysize();			//µÃµ½memp_memoryÊý×é´óÐ¡
	memp_memory=mymalloc(SRAMIN,mempsize);	//Îªmemp_memoryÉêÇëÄÚ´æ
	ramheapsize=LWIP_MEM_ALIGN_SIZE(MEM_SIZE)+2*LWIP_MEM_ALIGN_SIZE(4*3)+MEM_ALIGNMENT;//µÃµ½ram heap´óÐ¡
	ram_heap=mymalloc(SRAMIN,ramheapsize);	//Îªram_heapÉêÇëÄÚ´æ 
	if(!memp_memory||!ram_heap)//ÓÐÉêÇëÊ§°ÜµÄ
	{
		lwip_comm_mem_free();
		return 1;
	}
	return 0;	
}
//lwipÖÐmemºÍmempÄÚ´æÊÍ·Å
void lwip_comm_mem_free(void)
{ 	
	myfree(SRAMIN,memp_memory);
	myfree(SRAMIN,ram_heap);
}
//lwip Ä¬ÈÏIPÉèÖÃ
//lwipx:lwip¿ØÖÆ½á¹¹ÌåÖ¸Õë
void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
	u32 sn0;
	sn0=*(vu32*)(0x1FFF7A10);//»ñÈ¡STM32µÄÎ¨Ò»IDµÄÇ°24Î»×÷ÎªMACµØÖ·ºóÈý×Ö½Ú
	//Ä¬ÈÏÔ¶¶ËIPÎª:192.168.1.100
	lwipx->remoteip[0]=192;	
	lwipx->remoteip[1]=168;
	lwipx->remoteip[2]=1;
//	lwipx->remoteip[3]=104;
	lwipx->remoteip[3]=1;
	//MACµØÖ·ÉèÖÃ(¸ßÈý×Ö½Ú¹Ì¶¨Îª:2.0.0,µÍÈý×Ö½ÚÓÃSTM32Î¨Ò»ID)
	lwipx->mac[0]=2;//¸ßÈý×Ö½Ú(IEEE³ÆÖ®Îª×éÖ¯Î¨Ò»ID,OUI)µØÖ·¹Ì¶¨Îª:2.0.0
	lwipx->mac[1]=0;
	lwipx->mac[2]=0;
	lwipx->mac[3]=(sn0>>16)&0XFF;//µÍÈý×Ö½ÚÓÃSTM32µÄÎ¨Ò»ID
	lwipx->mac[4]=(sn0>>8)&0XFFF;;
	lwipx->mac[5]=sn0&0XFF; 
	//Ä¬ÈÏ±¾µØIPÎª:192.168.1.30
	lwipx->ip[0]=192;	
	lwipx->ip[1]=168;
	lwipx->ip[2]=1;
	lwipx->ip[3]=30;
	//Ä¬ÈÏ×ÓÍøÑÚÂë:255.255.255.0
	lwipx->netmask[0]=255;	
	lwipx->netmask[1]=255;
	lwipx->netmask[2]=255;
	lwipx->netmask[3]=0;
	//Ä¬ÈÏÍø¹Ø:192.168.1.1
	lwipx->gateway[0]=192;	
	lwipx->gateway[1]=168;
	lwipx->gateway[2]=1;
	lwipx->gateway[3]=1;	
	lwipx->dhcpstatus=0;//Ã»ÓÐDHCP	
} 

//LWIP³õÊ¼»¯(LWIPÆô¶¯µÄÊ±ºòÊ¹ÓÃ)
//·µ»ØÖµ:0,³É¹¦
//      1,ÄÚ´æ´íÎó
//      2,LAN8720³õÊ¼»¯Ê§°Ü
//      3,Íø¿¨Ìí¼ÓÊ§°Ü.
u8 lwip_comm_init(void)
{
	struct netif *Netif_Init_Flag;		//µ÷ÓÃnetif_add()º¯ÊýÊ±µÄ·µ»ØÖµ,ÓÃÓÚÅÐ¶ÏÍøÂç³õÊ¼»¯ÊÇ·ñ³É¹¦
	struct ip_addr ipaddr;  			//ipµØÖ·
	struct ip_addr netmask; 			//×ÓÍøÑÚÂë
	struct ip_addr gw;      			//Ä¬ÈÏÍø¹Ø 
	if(ETH_Mem_Malloc())return 1;		//ÄÚ´æÉêÇëÊ§°Ü
	if(lwip_comm_mem_malloc())return 1;	//ÄÚ´æÉêÇëÊ§°Ü
	if(LAN8720_Init())return 2;			//³õÊ¼»¯LAN8720Ê§°Ü 
	lwip_init();						//³õÊ¼»¯LWIPÄÚºË
	lwip_comm_default_ip_set(&lwipdev);	//ÉèÖÃÄ¬ÈÏIPµÈÐÅÏ¢

#if LWIP_DHCP		//Ê¹ÓÃ¶¯Ì¬IP
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;
#else				//Ê¹ÓÃ¾²Ì¬IP
	IP4_ADDR(&ipaddr,lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
	IP4_ADDR(&netmask,lwipdev.netmask[0],lwipdev.netmask[1] ,lwipdev.netmask[2],lwipdev.netmask[3]);
	IP4_ADDR(&gw,lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
	printf("Íø¿¨enµÄMACµØÖ·Îª:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
	printf("¾²Ì¬IPµØÖ·........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
	printf("×ÓÍøÑÚÂë..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
	printf("Ä¬ÈÏÍø¹Ø..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
#endif
	Netif_Init_Flag=netif_add(&lwip_netif,&ipaddr,&netmask,&gw,NULL,&ethernetif_init,&ethernet_input);//ÏòÍø¿¨ÁÐ±íÖÐÌí¼ÓÒ»¸öÍø¿Ú
	
#if LWIP_DHCP			//Èç¹ûÊ¹ÓÃDHCPµÄ»°
	lwipdev.dhcpstatus=0;	//DHCP±ê¼ÇÎª0
	dhcp_start(&lwip_netif);	//¿ªÆôDHCP·þÎñ
#endif
	
	if(Netif_Init_Flag==NULL)return 3;//Íø¿¨Ìí¼ÓÊ§°Ü 
	else//Íø¿ÚÌí¼Ó³É¹¦ºó,ÉèÖÃnetifÎªÄ¬ÈÏÖµ,²¢ÇÒ´ò¿ªnetifÍø¿Ú
	{
		netif_set_default(&lwip_netif); //ÉèÖÃnetifÎªÄ¬ÈÏÍø¿Ú
		netif_set_up(&lwip_netif);		//´ò¿ªnetifÍø¿Ú
	}
	return 0;//²Ù×÷OK.
}   

//µ±½ÓÊÕµ½Êý¾Ýºóµ÷ÓÃ 
void lwip_pkt_handle(void)
{
  //´ÓÍøÂç»º³åÇøÖÐ¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý°ü²¢½«Æä·¢ËÍ¸øLWIP´¦Àí 
 ethernetif_input(&lwip_netif);
}

//LWIPÂÖÑ¯ÈÎÎñ
void lwip_periodic_handle()
{
#if LWIP_TCP
	//Ã¿250msµ÷ÓÃÒ»´Îtcp_tmr()º¯Êý
  if (lwip_localtime - TCPTimer >= TCP_TMR_INTERVAL)
  {
    TCPTimer =  lwip_localtime;
    tcp_tmr();
  }
#endif
  //ARPÃ¿5sÖÜÆÚÐÔµ÷ÓÃÒ»´Î
  if ((lwip_localtime - ARPTimer) >= ARP_TMR_INTERVAL)
  {
    ARPTimer =  lwip_localtime;
    etharp_tmr();
  }

#if LWIP_DHCP //Èç¹ûÊ¹ÓÃDHCPµÄ»°
  //Ã¿500msµ÷ÓÃÒ»´Îdhcp_fine_tmr()
  if (lwip_localtime - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
  {
    DHCPfineTimer =  lwip_localtime;
    dhcp_fine_tmr();
    if ((lwipdev.dhcpstatus != 2)&&(lwipdev.dhcpstatus != 0XFF))
    { 
      lwip_dhcp_process_handle();  //DHCP´¦Àí
    }
  }

  //Ã¿60sÖ´ÐÐÒ»´ÎDHCP´Ö²Ú´¦Àí
  if (lwip_localtime - DHCPcoarseTimer >= DHCP_COARSE_TIMER_MSECS)
  {
    DHCPcoarseTimer =  lwip_localtime;
    dhcp_coarse_tmr();
  }  
#endif
}


//Èç¹ûÊ¹ÄÜÁËDHCP
#if LWIP_DHCP

//DHCP´¦ÀíÈÎÎñ
void lwip_dhcp_process_handle(void)
{
	u32 ip=0,netmask=0,gw=0;
	switch(lwipdev.dhcpstatus)
	{
		case 0: 	//¿ªÆôDHCP
			dhcp_start(&lwip_netif);
			lwipdev.dhcpstatus = 1;		//µÈ´ýÍ¨¹ýDHCP»ñÈ¡µ½µÄµØÖ·
			printf("ÕýÔÚ²éÕÒDHCP·þÎñÆ÷,ÇëÉÔµÈ...........\r\n");  
			break;
		case 1:		//µÈ´ý»ñÈ¡µ½IPµØÖ·
		{
			ip=lwip_netif.ip_addr.addr;		//¶ÁÈ¡ÐÂIPµØÖ·
			netmask=lwip_netif.netmask.addr;//¶ÁÈ¡×ÓÍøÑÚÂë
			gw=lwip_netif.gw.addr;			//¶ÁÈ¡Ä¬ÈÏÍø¹Ø 
			
			if(ip!=0)			//ÕýÈ·»ñÈ¡µ½IPµØÖ·µÄÊ±ºò
			{
				lwipdev.dhcpstatus=2;	//DHCP³É¹¦
				printf("Íø¿¨enµÄMACµØÖ·Îª:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
				//½âÎö³öÍ¨¹ýDHCP»ñÈ¡µ½µÄIPµØÖ·
				lwipdev.ip[3]=(uint8_t)(ip>>24); 
				lwipdev.ip[2]=(uint8_t)(ip>>16);
				lwipdev.ip[1]=(uint8_t)(ip>>8);
				lwipdev.ip[0]=(uint8_t)(ip);
				printf("Í¨¹ýDHCP»ñÈ¡µ½IPµØÖ·..............%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
				//½âÎöÍ¨¹ýDHCP»ñÈ¡µ½µÄ×ÓÍøÑÚÂëµØÖ·
				lwipdev.netmask[3]=(uint8_t)(netmask>>24);
				lwipdev.netmask[2]=(uint8_t)(netmask>>16);
				lwipdev.netmask[1]=(uint8_t)(netmask>>8);
				lwipdev.netmask[0]=(uint8_t)(netmask);
				printf("Í¨¹ýDHCP»ñÈ¡µ½×ÓÍøÑÚÂë............%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
				//½âÎö³öÍ¨¹ýDHCP»ñÈ¡µ½µÄÄ¬ÈÏÍø¹Ø
				lwipdev.gateway[3]=(uint8_t)(gw>>24);
				lwipdev.gateway[2]=(uint8_t)(gw>>16);
				lwipdev.gateway[1]=(uint8_t)(gw>>8);
				lwipdev.gateway[0]=(uint8_t)(gw);
				printf("Í¨¹ýDHCP»ñÈ¡µ½µÄÄ¬ÈÏÍø¹Ø..........%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			}else if(lwip_netif.dhcp->tries>LWIP_MAX_DHCP_TRIES) //Í¨¹ýDHCP·þÎñ»ñÈ¡IPµØÖ·Ê§°Ü,ÇÒ³¬¹ý×î´ó³¢ÊÔ´ÎÊý
			{
				lwipdev.dhcpstatus=0XFF;//DHCP³¬Ê±Ê§°Ü.
				//Ê¹ÓÃ¾²Ì¬IPµØÖ·
				IP4_ADDR(&(lwip_netif.ip_addr),lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
				IP4_ADDR(&(lwip_netif.netmask),lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
				IP4_ADDR(&(lwip_netif.gw),lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
				printf("DHCP·þÎñ³¬Ê±,Ê¹ÓÃ¾²Ì¬IPµØÖ·!\r\n");
				printf("Íø¿¨enµÄMACµØÖ·Îª:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
				printf("¾²Ì¬IPµØÖ·........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
				printf("×ÓÍøÑÚÂë..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
				printf("Ä¬ÈÏÍø¹Ø..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			}
		}
		break;
		default : break;
	}
}
#endif 



























