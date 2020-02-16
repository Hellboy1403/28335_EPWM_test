/*
 * dc_motor.c
 *
 *  Created on: 2018-1-23
 *      Author: Administrator
 */


#include "dc_motor.h"
#include "key.h"

void DC_Motor_Init(void)
{
	EALLOW;

	//DC_MOTOR�˿�����--1·
	GpioCtrlRegs.GPAMUX1.bit.GPIO2=0;
	GpioCtrlRegs.GPADIR.bit.GPIO2=1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO3=0;
	GpioCtrlRegs.GPADIR.bit.GPIO3=1;


	//DC_MOTOR�˿�����--2·
	GpioCtrlRegs.GPAMUX1.bit.GPIO4=0;
	GpioCtrlRegs.GPADIR.bit.GPIO4=1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO5=0;
	GpioCtrlRegs.GPADIR.bit.GPIO5=1;

	EDIS;

	GpioDataRegs.GPACLEAR.bit.GPIO2=1;
	GpioDataRegs.GPACLEAR.bit.GPIO3=1;
	GpioDataRegs.GPACLEAR.bit.GPIO4=1;
	GpioDataRegs.GPACLEAR.bit.GPIO5=1;
}







// �궨��ÿ����ʱ�����ڼĴ���������ֵ��
#define EPWM2_TIMER_TBPRD  3750  // ����ֵ
#define EPWM2_MAX_CMPA     3700
#define EPWM2_MIN_CMPA       0
#define EPWM2_MAX_CMPB     3700
#define EPWM2_MIN_CMPB       0

/***************ȫ�ֱ�������****************/
Uint16 pwm_stepValue=0;  //�ߵ�ƽʱ��
Uint16 Direction=0;//ת�ٷ���

interrupt void epwm2_isr(void);

void DCMotor_ePWM2_Init(void)
{
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Disable TBCLK within the ePWM
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
	EDIS;

	InitEPwm2Gpio();

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.EPWM2_INT = &epwm2_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	// ����ʱ���׼��ʱ���źţ�TBCLK��
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // ��������ģʽ
	EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;       // ���ö�ʱ������
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // ��ֹ��λ����
	EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // ʱ����λ�Ĵ�����ֵ��ֵ0
	EPwm2Regs.TBCTR = 0x0000;                   // ʱ������������
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // ����ʱ��ʱ������Ϊϵͳʱ��SYSCLKOUT/4=37.5MHZ;
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;//��ʱ��ʱ��Ƶ�ʺ�ʱ�����ڿ�֪PWM1Ƶ��=10KHZ��

	// ���ñȽϼĴ�������Ӱ�Ĵ�������������ʱ��������0
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


	// ���ñȽϼĴ�����ֵ
	EPwm2Regs.CMPA.half.CMPA = EPWM2_MIN_CMPA;     // ���ñȽϼĴ���A��ֵ
	EPwm2Regs.CMPB = EPWM2_MIN_CMPB;               // ���ñȽϼĴ���B��ֵ

	// ���ö����޶�������Ĭ��Ϊת������Ϊ��ת����ʱֻ��PWM1A���ռ�ձȣ�
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // ������0ʱPWM1A����ߵ�ƽ
	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1A���

	EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;            // ������0ʱPWM1B����͵�ƽ
	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1B���

	// 3��0ƥ���¼�����ʱ����һ���ж�����һ��ƥ����100us��һ��300us����һ���жϣ�
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // ѡ��0ƥ���¼��ж�
	EPwm2Regs.ETSEL.bit.INTEN = 1;                // ʹ���¼������ж�
	EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // 3���¼������ж�����

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	IER |= M_INT3;
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;

	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

interrupt void epwm2_isr(void)
{
	static unsigned char key=0;

	key=KEY_Scan(0);
	if(key==KEY1_PRESS||key==KEY2_PRESS||key==KEY3_PRESS)
	{
		if(key==KEY3_PRESS)
		{
			//��֤����EPWMA��EPWMB�໥�л�ͬʱ���0��ƽ��
			EPwm2Regs.CMPA.half.CMPA = 0;//�ı�����
			EPwm2Regs.CMPB = 0;//�ı�����

			if(Direction==0)
			{
				// ���ö����޶�������Ĭ��Ϊת������Ϊ��ת����ʱֻ��PWM1B���ռ�ձȣ�
				EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // ������0ʱPWM1A����͵�ƽ
				EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1A���

				EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;            // ������0ʱPWM1B����ߵ�ƽ
				EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1B���
				Direction=1;
			}
			else
			{
				// ���ö����޶�������Ĭ��Ϊת������Ϊ��ת����ʱֻ��PWM1A���ռ�ձȣ�
				EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // ������0ʱPWM1A����ߵ�ƽ
				EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1A���

				EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;            // ������0ʱPWM1B����͵�ƽ
				EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // ��������ʱ�������ȽϼĴ���Aƥ��ʱ���PWM1B���
				Direction=0;
			}
			pwm_stepValue=0;
		}
		else
		{
			if(key==KEY1_PRESS)
			{
				if(pwm_stepValue!=3500)
					pwm_stepValue+=500;
			}
			else if(key==KEY2_PRESS)
			{
				if(pwm_stepValue!=0)
					pwm_stepValue=pwm_stepValue-500;
			}
		}

		EPwm2Regs.CMPA.half.CMPA = pwm_stepValue;//�ı�����
		EPwm2Regs.CMPB = pwm_stepValue;//�ı�����
	}

	// ��������ʱ�����жϱ�־λ
	EPwm2Regs.ETCLR.bit.INT = 1;
	// ���PIEӦ��Ĵ����ĵ���λ������Ӧ��3�ڵ������ж�����
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

