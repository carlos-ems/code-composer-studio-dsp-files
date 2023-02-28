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
#define ts 50e-6
#define dead_time_rise 25
#define dead_time_fall 25
#define timer_pass 0.00003333333    // 1/(60*100)
#define timer_u_pass 33.333333  // [1/(60*100)]/1000000 (microseconds)

// ---------- Variables ----------
int enable = 0, n = 0, period_PWM, period_ADC;
double d = 0.5, t = 0;
double ADC1, ADC2, zero1, zero2;
double vout, vref, iout, iref, reference;
double ek, ek1, uk, uk1, upk, upk1, uik, uik1, umax, kp, ki;

// ---------- Peripheral settings ----------
void ConfigurePWM(void); // Done
void SetupTimer(void); // Done
void ConfigureADC(void);

// ---------- Interrupt Routines ----------
interrupt void Time(void);
interrupt void ADC(void);

// ---------- Peripheral subroutines ----------

// ---------- PWM settings ----------
void ConfigurePWM(void){
    period_PWM = 90e6/fs;

    // ---------- EPwm Module 1 settings ----------
    EPwm1Regs.TBPRD = period_PWM;   // test period_PWM*0.5
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

    InitEPwm(); /* VERIFY */
    InitEPwm1Gpio();
}

// ---------- Timer settings ----------
void SetupTimer(void){
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 90, timer_u_pass);
    CpuTimer1Regs.TCR.all = 0X4000;
}

// ---------- ADC settings ----------
void ConfigureADC(){
    T_ADC = 90e6/fs;

    InitADC(); /* VERIFY */

    EALLOW;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 7;

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
    /*
     * FINALIZAR ESTE TRECHO DO CÓDIGO
     */

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
    PieVectTable.TIMER1_INT = &Time;
    PieVectTable.ADCA1_INT = &ADC;
    PieVectTable.XINT1_INT = &Calibration;
    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    EDIS;
}
