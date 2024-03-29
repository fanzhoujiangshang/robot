#include "system.h"
#include "motor_control.h"
#include "stmflash.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
static bool systemTest(void);

int mag_calibration_data[9];

bool lwip_test(void)
{
	u8 speed;

	if(lwipdev.dhcpstatus==2)printf("DHCP IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印动态IP地址
	else printf("Static IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印静态IP地址

	speed=LAN8720_Get_Speed();//得到网速
	if(speed&1<<1)printf("Ethernet Speed:100M \n");
	else printf("Ethernet Speed:10M \n");
	printf("PORT: 8088 \n");
	
	return true;
}

void read_paraments(void)
{
	STMFLASH_Read(0X080E0000,(u32*)mag_calibration_data,6);
	
	mag_calibration_data[6] = (mag_calibration_data[0] + mag_calibration_data[1])/2;
	mag_calibration_data[7] = (mag_calibration_data[2] + mag_calibration_data[3])/2;
	mag_calibration_data[8] = (mag_calibration_data[4] + mag_calibration_data[5])/2;
}

/*底层硬件初始化*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*中断配置初始化*/
	extiInit();			/*外部中断初始化*/	
	delay_init(96);		/*delay初始化*/

	commInit();			/*通信初始化  STM32 & NRF51822 */
	atkpInit();			/*传输协议初始化*/
	consoleInit();		/*打印初始化*/
	printf("<--------------------->\n");
	printf("<---FIRMWARE START--->\n");
	printf("<--------------------->\n");	
	
	configParamInit();	/*初始化配置参数*/
	read_paraments();
	pmInit();			/*电源管理初始化*/
	stabilizerInit();	/*电机 传感器 PID初始化*/
	
	mymem_init(SRAMIN);		//初始化内部内存池
	mymem_init(SRAMEX);		//初始化外部内存池
	mymem_init(SRAMCCM);	//初始化CCM内存池
	TIM3_Int_Init(999,839); //100khz的频率,计数1000为10ms
#if 1	
	set_lwip_init_flag(1);		
	while(lwip_comm_init()) //lwip初始化
	{
		if(cnt++ > 4)
		{
			cnt=0;
			set_lwip_init_flag(0);
			printf("LWIP Init Falied! \n");			
			break;
		}
		printf("LWIP Init Retrying... \n");  	
		delay_ms(1000);				
	}	
	if(get_lwip_init_flag()){
		printf("LWIP Init Success! \n");	
	 	printf("DHCP IP configing... \n");  //等待DHCP获�
	}
	
#if LWIP_DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//等待DHCP获取成功/超时溢出
	{
		lwip_periodic_handle();
	}
#endif 	
#endif
#if 1
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*初始化通过 左上绿灯快闪5次*/
		{
//			ledFlashOne(LED_GREEN_L, 50, 50);
		}	
		printf("systemTest success!\n");	
	}else
	{		
		while(1)		/*初始化错误 右上红灯间隔1s快闪5次*/
		{
			if(cnt++ > 4)
			{
				cnt=0;
				delay_xms(1000);
			}
			printf("systemTest err!\n");	
//			ledFlashOne(LED_RED_L, 50, 50);		
		}
	}
	watchdogInit(WATCHDOG_RESET_MS);	/*看门狗初始化*/
#endif	
}




static bool systemTest(void)
{
	bool pass = true;
	
//	pass &= ledseqTest();
//	pass &= pmTest();
	
	pass &= configParamTest();
	pass &= commTest();
	pass &= stabilizerTest();	
	pass &= watchdogTest();
	pass &= lwip_test();
	
	return pass;
}

