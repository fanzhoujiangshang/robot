#include "system.h"
#include "motor_control.h"
#include "stmflash.h"

/********************************************************************************	 
 * ¥¥Ω®»’∆⁄:2018/6/22
 * ∞Ê±æ£∫V1.2
 * ∞Ê»®À˘”–£¨µ¡∞Ê±ÿæø°£
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
static bool systemTest(void);

int mag_calibration_data[9];

bool lwip_test(void)
{
	u8 speed;

	if(lwipdev.dhcpstatus==2)printf("DHCP IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//¥Ú”°∂ØÃ¨IPµÿ÷∑
	else printf("Static IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//¥Ú”°æ≤Ã¨IPµÿ÷∑

	speed=LAN8720_Get_Speed();//µ√µΩÕ¯ÀŸ
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

/*µ◊≤„”≤º˛≥ı ºªØ*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*÷–∂œ≈‰÷√≥ı ºªØ*/
	extiInit();			/*Õ‚≤ø÷–∂œ≥ı ºªØ*/	
	delay_init(96);		/*delay≥ı ºªØ*/

	commInit();			/*Õ®–≈≥ı ºªØ  STM32 & NRF51822 */
	atkpInit();			/*¥´ ‰–≠“È≥ı ºªØ*/
	consoleInit();		/*¥Ú”°≥ı ºªØ*/
	printf("<--------------------->\n");
	printf("<---FIRMWARE START--->\n");
	printf("<--------------------->\n");	
	
	configParamInit();	/*≥ı ºªØ≈‰÷√≤Œ ˝*/
	read_paraments();
	pmInit();			/*µÁ‘¥π‹¿Ì≥ı ºªØ*/
	stabilizerInit();	/*µÁª˙ ¥´∏–∆˜ PID≥ı ºªØ*/
	
	mymem_init(SRAMIN);		//≥ı ºªØƒ⁄≤øƒ⁄¥Ê≥ÿ
	mymem_init(SRAMEX);		//≥ı ºªØÕ‚≤øƒ⁄¥Ê≥ÿ
	mymem_init(SRAMCCM);	//≥ı ºªØCCMƒ⁄¥Ê≥ÿ
	TIM3_Int_Init(999,839); //100khzµƒ∆µ¬ ,º∆ ˝1000Œ™10ms
#if 1	
	set_lwip_init_flag(1);		
	while(lwip_comm_init()) //lwip≥ı ºªØ
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
	 	printf("DHCP IP configing... \n");  //µ»¥˝DHCPªÒ»
	}
	
#if LWIP_DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//µ»¥˝DHCPªÒ»°≥…π¶/≥¨ ±“Á≥ˆ
	{
		lwip_periodic_handle();
	}
#endif 	
#endif
#if 1
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*≥ı ºªØÕ®π˝ ◊Û…œ¬Ãµ∆øÏ…¡5¥Œ*/
		{
//			ledFlashOne(LED_GREEN_L, 50, 50);
		}	
		printf("systemTest success!\n");	
	}else
	{		
		while(1)		/*≥ı ºªØ¥ÌŒÛ ”“…œ∫Ïµ∆º‰∏Ù1søÏ…¡5¥Œ*/
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
	watchdogInit(WATCHDOG_RESET_MS);	/*ø¥√≈π∑≥ı ºªØ*/
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

