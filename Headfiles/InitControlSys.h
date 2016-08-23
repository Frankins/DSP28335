/*
 * InitControlSys.c
 *
 *  Created on: 2016-8-4
 *      Author: Frank
 */
#include "InitControlSys.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

extern axis_t AXIS;

uint16_t EPwm1TimerCount = 0;
uint16_t EPwm2TimerCount = 0;
uint16_t EPwm3TimerCount = 0;
uint16_t EPwm4TimerCount = 0;
uint16_t EPwm5TimerCount = 0;
uint16_t EPwm6TimerCount = 0;

uint16_t EPWMPulseNum1 = 0;
uint16_t EPWMPulseNum2 = 0;
uint16_t EPWMPulseNum3 = 0;
uint16_t EPWMPulseNum4 = 0;
uint16_t EPWMPulseNum5 = 0;
uint16_t EPWMPulseNum6 = 0;


void Init_Muti_EPwm1(void)
{
	// Setup TBCLK
	   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm1Regs.TBPRD = 0;       // Set timer period
	   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   //EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   //EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm1Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   //EPwm1Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   //EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Muti_EPwm2(void)
{
	// Setup TBCLK
	   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm2Regs.TBPRD = 0;       // Set timer period
	   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm2Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   EPwm2Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Muti_EPwm3(void)
{
	// Setup TBCLK
	   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm3Regs.TBPRD = 0;       // Set timer period
	   EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm3Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   EPwm3Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Muti_EPwm4(void)
{
	// Setup TBCLK
	   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm4Regs.TBPRD = 0;       // Set timer period
	   EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm4Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   EPwm4Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Muti_EPwm5(void)
{
	// Setup TBCLK
	   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm5Regs.TBPRD = 0;       // Set timer period
	   EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm5Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm5Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   EPwm5Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Muti_EPwm6(void)
{
	// Setup TBCLK
	   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm6Regs.TBPRD = 0;       // Set timer period
	   EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm6Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
	   EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
	   EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV4;

	   // Setup shadow register load on ZERO
	   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set Compare values
	   EPwm6Regs.CMPA.half.CMPA = CMPA_TBPRD;    // Set compare A value
	   EPwm6Regs.CMPB = CMPB_TBPRD;              // Set Compare B value

	   // Set actions
	   EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
	   EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

	   EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
	   EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

	   // Interrupt where we will change the Compare Values
	   EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // Select INT on PRD event
	   EPwm6Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

}

void Init_Direction_Pin(void)
{
	EALLOW;

	//GPIO18 for Axis 1
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;

	//GPIO19 for Axis 2
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;

	//GPIO20 for Axis 3
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;

	//GPIO21 for Axis 4
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;

	//GPIO22 for Axis 5
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;

	//GPIO23 for Axis 6
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;

	EDIS;
}



void Init_Brake_Pin(void)
{
	EALLOW;

	//GPIO13 for releasing Axis 2
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
	GpioDataRegs.GPADAT.bit.GPIO13=1;

	//GPIO14 for releasing Axis 3
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
	GpioDataRegs.GPADAT.bit.GPIO13=1;

	//GPIO15 for releasing Axis 5
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
	GpioDataRegs.GPADAT.bit.GPIO13=1;

	EDIS;
}

void DirControl(uint16_t axis,uint16_t dir)
{
	EALLOW;
	switch (axis)
	{
		case AXIS1:GpioDataRegs.GPADAT.bit.GPIO18=dir;break;
		case AXIS2:GpioDataRegs.GPADAT.bit.GPIO19=dir;break;
		case AXIS3:GpioDataRegs.GPADAT.bit.GPIO20=dir;break;
		case AXIS4:GpioDataRegs.GPADAT.bit.GPIO24=dir;break;
		case AXIS5:GpioDataRegs.GPADAT.bit.GPIO22=dir;break;
		case AXIS6:GpioDataRegs.GPADAT.bit.GPIO23=dir;break;
	}
	EDIS;
}

void BraControl(uint16_t axis)
{
	switch(axis)
	{
	case AXIS2:GpioDataRegs.GPADAT.bit.GPIO13=0;break;
	case AXIS3:GpioDataRegs.GPADAT.bit.GPIO14=0;break;
	case AXIS5:GpioDataRegs.GPADAT.bit.GPIO15=0;break;
	}
}

void ControlDriver(uint16_t axis ,uint16_t postion ,uint16_t direction)
{
	switch(axis)
	{
	case AXIS1:EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;
					   EPWMPulseNum1 = postion*100;
					   DirControl(axis,direction);
					   break;
	case AXIS2:EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;
					   EPWMPulseNum2 = postion*100;
					   DirControl(axis,direction);
					   BraControl(axis);
					   break;
	case AXIS3:EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;
	 	 	 	 	   EPWMPulseNum3 = postion*100;
					   DirControl(axis,direction);
					   BraControl(axis);
					   break;
	case AXIS4:EPwm4Regs.TBPRD = EPWM_TIMER_TBPRD;
	 	 	 	 	   EPWMPulseNum4 = postion*100;
					   DirControl(axis,direction);
					   break;
	case AXIS5:EPwm5Regs.TBPRD = EPWM_TIMER_TBPRD;
					   EPWMPulseNum5 = postion*100;
					   DirControl(axis,direction);
					   //BraControl(axis);
					   break;
	case AXIS6:EPwm6Regs.TBPRD = EPWM_TIMER_TBPRD;
	 	 	 	 	   EPWMPulseNum6 = postion*100;
					   DirControl(axis,direction);
					   break;
	}
}


