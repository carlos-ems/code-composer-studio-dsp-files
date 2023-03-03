/*
 * main.c
 *
 *  Federal University of Santa Catarina (UFSC)
 *  Institutional Scientific Initiation Scholarship Program (PIBIC/CNPq)
 *  Real-time simulation Boost Converter in Closed Loop
 *  Scholarship holder: Carlos E. Miranda da Silva
 *  Advisor: Daniel Pagano
 *
 *  Created on: Feb. 16, 2023
 *      Author: Carlos Silva
 */

// ---------- Includes ----------
#include "F28x_Project.h"
#include "F2837xD_device.h"
#include <math.h>

// ---------- Defines ----------
#define fs 50e3
#define Ts 50e-6
#define dead_time_rise 25
#define dead_time_fall 25
#define timer_pass 0.00003333333    // 1/(60*100)
#define timer_u_pass 33.333333  // [1/(60*100)]/1000000 (microseconds)

// ---------- Variables ----------
int enable = 0, n = 0, cal = 0, period_PWM, period_ADC;
double d = 0.5, t = 0;
double ADC1, ADC2, zero1, zero2;
double vout, vref, iout, iref, reference;
double ek, ek1, uk, uk1, upk, upk1, uik, uik1, umax, kp = 0.1, ki = 0.01;

// ---------- Peripheral settings ----------
void ConfigurePWM(void);
void SetupTimer(void);
void ConfigureADC(void);

// ---------- Interrupt Routines ----------
interrupt void Timer(void);
interrupt void ADC(void); // In progress

// ---------- Peripherals and Interrupts subroutines ----------

// ---------- PWM settings ----------
void ConfigurePWM(void){
    period_PWM = 90e6/fs;

    // ---------- EPwm Module 1 settings ----------
    EPwm1Regs.TBPRD = period_PWM;   // after this, test "period_PWM*0.5"
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm1Regs.CMPA.bit.CMPA = period_PWM*d;  // adjust duty cycle for output EPwm1A
    EPwm1Regs.CMPB.bit.CMPB = 0;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBRED.bit.DBRED = dead_time_rise;
    EPwm1Regs.DBFED.bit.DBFED = dead_time_fall;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;

    InitEPwm1Gpio();
}

// ---------- Timer settings ----------
void SetupTimer(void){
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 90, timer_u_pass);
    CpuTimer1Regs.TCR.all = 0X4000;
}

// ---------- Timer Interrupt subroutine ----------
__interrupt void Timer(void){
    if (enable == 1){
        t += timer_pass;
        n++;
    }
    else{
        t = 0;
        n = 0;
    }
}

// ---------- ADC settings ----------
void ConfigureADC(){
    period_ADC = 90e6/fs;

    EALLOW;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 7;

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 6;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 8;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 6;
    EDIS;

    /* This part of the code must be check */
    EPwm4Regs.TBPRD = period_PWM;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm4Regs.CMPA.bit.CMPA = period_ADC*d;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm4Regs.ETSEL.bit.SOCAEN   = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL  = 1;
    EPwm4Regs.ETPS.bit.SOCAPRD   = 1;

}

// ---------- ADC Interrupt Subroutine ----------
__interrupt void ADC(void){

    GpioDataRegs.GPADAT.bit.GPIO31 = 1;

    if (cal < 50){
        zero1 = (float)AdcaResultRegs.ADCRESULT1;
        zero2 = (float)AdcaResultRegs.ADCRESULT2;
        cal++;
    }

    // Read Vout
    ADC1 = ((float)AdcaResultRegs.ADCRESULT1 - zero1) * (3.3/4095);

    vout = ADC1*(10/1.5)*100;

    ek = vref - vout;

    upk = kp*ek;

    uik = ki*(1/Ts)*(ek);

    uk = upk + uik;

    uk1 = uk;

    ek1 = ek;

    reference = vout;

    /* This part of the code must be finish */

    GpioDataRegs.GPADAT.bit.GPIO31 = 0;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;     // Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

void main(void){

    // Initialize system settings
    InitSysCtrl();
    InitGpio();
    DINT;

    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    // Access registers
    EALLOW;
    PieVectTable.TIMER1_INT = &Timer;
    PieVectTable.ADCA1_INT = &ADC;
    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    EDIS;

    ConfigurePWM();
    SetupTimer();
    ConfigureADC();

    IER |= M_INT1;
    IER |= M_INT13;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;

    EINT;
    ERTM;
}
