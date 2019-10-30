#include "system.h"
#include "motor_control.h"
#include "stmflash.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
static bool systemTest(void);

int mag_calibration_data[9];

bool lwip_test(void)
{
	u8 speed;

	if(lwipdev.dhcpstatus==2)printf("DHCP IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ
	else printf("Static IP:%d.%d.%d.%d \n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ

	speed=LAN8720_Get_Speed();//�õ�����
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

/*�ײ�Ӳ����ʼ��*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*�ж����ó�ʼ��*/
	extiInit();			/*�ⲿ�жϳ�ʼ��*/	
	delay_init(96);		/*delay��ʼ��*/

	commInit();			/*ͨ�ų�ʼ��  STM32 & NRF51822 */
	atkpInit();			/*����Э���ʼ��*/
	consoleInit();		/*��ӡ��ʼ��*/
	printf("<--------------------->\n");
	printf("<---FIRMWARE START--->\n");
	printf("<--------------------->\n");	
	
	configParamInit();	/*��ʼ�����ò���*/
	read_paraments();
	pmInit();			/*��Դ�����ʼ��*/
	stabilizerInit();	/*��� ������ PID��ʼ��*/
	
	mymem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	mymem_init(SRAMEX);		//��ʼ���ⲿ�ڴ��
	mymem_init(SRAMCCM);	//��ʼ��CCM�ڴ��
	TIM3_Int_Init(999,839); //100khz��Ƶ��,����1000Ϊ10ms
#if 1	
	set_lwip_init_flag(1);		
	while(lwip_comm_init()) //lwip��ʼ��
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
	 	printf("DHCP IP configing... \n");  //�ȴ�DHCP���
	}
	
#if LWIP_DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//�ȴ�DHCP��ȡ�ɹ�/��ʱ���
	{
		lwip_periodic_handle();
	}
#endif 	
#endif
#if 1
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*��ʼ��ͨ�� �����̵ƿ���5��*/
		{
//			ledFlashOne(LED_GREEN_L, 50, 50);
		}	
		printf("systemTest success!\n");	
	}else
	{		
		while(1)		/*��ʼ������ ���Ϻ�Ƽ��1s����5��*/
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
	watchdogInit(WATCHDOG_RESET_MS);	/*���Ź���ʼ��*/
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

