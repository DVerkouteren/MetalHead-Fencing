//File Name: madgwickControlMain.c
//ECE230 Spring 2024-2025
//Junhwa Kim
//Date: May 18, 2025
/********************************************************
 *  ______________________________________________________________________________
 *  |       |         |               |       |         |                 |      |
 *  | Start | Addr  W | <1 Byte Send> | Start | Addr  R |  <6 Byte Read>  | Stop |
 *  |_______|_________|_______________|_______|_________|_________________|______|
 *
 *                                  ___  ___
 *                                   |    |
 *               MSP432P411x        10k  10k     MPU6050
 *             ------------------    |    |    -----------
 *         /|\|     P1.6/UCB0SDA |<--|----|-->| SDA
 *          | |                  |   |        |
 *          --|RST               |   |        |
 *            |     P1.7/UCB0SCL |<--|------->| SCL
 *    0V,GND--|AD0               |            |
 *    Bluetooth 3.2(TDX) and 3.3(RDX)
 *    Switch 6.4 & GND
 *
********************************************************/
#include "msp.h"
#include <stdio.h>  //printf(); sprintf();
#include <stdint.h>
#include <string.h>
#include <math.h>   //pow(base, exponent)

//switch port and masks
#define SwitchPort P6   //Port 6
#define Switch1   0b00010000    //P6.4

#define I2Cport P1
#define SCLPIN  BIT7
#define SDAPIN  BIT6

enum {g2, g4, g8, g16} gFullScale=g2;  //2g, 4g, 8g and 16g mapped t0 0, 1,2,3
#define fullscale1g  pow(2,14)/pow(2,gFullScale)    //2^14=16384

#define NUM_OF_REC_BYTES        6       // number of bytes to receive from sensor read
/* TODO update peripheral address - 0b110100X */
#define GY521_ADDRESS      0x68    // AD0=GND, I2C address of GY-521 sensor
/* TODO update register addresses   */
#define ACCEL_BASE_ADDR  0x3B    // base address of accelerometer data registers
#define PWR_MGMT_ADDR           0x6B    // address of power management register

uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer;

volatile float q0=1, q1=0, q2=0, q3=0;
float gx_off, gy_off, gz_off;
// Filter & calibration
#define SAMPLE_FREQ       100.0f
#define DELTA_T           (1.0f/SAMPLE_FREQ)
#define BETA              0.1f
#define GYRO_CAL_SAMPLES  500

void UART_init(void);
void UART_sendChar(char c);
void UART_sendString(const char *str);
int fputc(int ch, FILE *f) {
    UART_sendChar((char)ch);
    return ch;
}


void main(void)
{
    //Switch configuration
    SwitchPort->DIR &= ~Switch1;
    SwitchPort->SEL0 &= ~Switch1;
    SwitchPort->SEL1 &= ~Switch1;
    SwitchPort->OUT |= Switch1;
    SwitchPort->REN |= Switch1;


    volatile uint32_t i;
    volatile int16_t accel_x, accel_y, accel_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float accel_x1 = 0;
    float accel_x2 = 0;
    float accel_y1 = 0;
    float accel_y2 = 0;
    float accel_z1 = 0;
    float accel_z2 = 0;
    char buf[64];
    int Pressed = 0;

    float pitch, roll, yaw;
    float sumx=0, sumy=0, sumz=0;
    float ax, ay, az, gx, gy, gz;

    uint16_t cal_i;
    volatile uint32_t d;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
    /*––– Initialize UART for HC-05 on P1.2 (RX) / P1.3 (TX) –––*/
    UART_init();
    /* Configure UART pins */
    I2Cport->SEL0 |= SCLPIN | SDAPIN;                // set I2C pins as secondary function
    I2Cport->SEL1 &= ~(SCLPIN | SDAPIN);
    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;
    /* Configure eUSCI_B0 for I2C mode
     *  I2C master mode, synchronous, 7-bit address, SMCLK clock source,
     *  transmit mode, with automatic STOP condition generation
     */
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Software reset enabled
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset mode
            EUSCI_B_CTLW0_MODE_3 |          // I2C mode
            EUSCI_B_CTLW0_MST |             // Master mode
            EUSCI_B_CTLW0_SYNC |            // Sync mode
            EUSCI_B_CTLW0_TR |              // Transmitter mode
            EUSCI_B_CTLW0_SSEL__SMCLK;      // SMCLK
    /* I2C clock calculation
     * Refer to Section 26.3.6 of Technical Reference manual
     * BRCLK = 3MHz, I2C bit clock rate = 100kbps
    */
    // TODO configure eUSCI_B0 bit rate control for 100 kbps
    EUSCI_B0->BRW = 0x1E;
    /* Configure I2C to communicate with GY-521 */
    EUSCI_B0->I2CSA = GY521_ADDRESS;            // I2C peripheral address
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Release eUSCI from reset

    /* Initialize GY-521 by writing to Power Management Register
     *
     *  format for Write operations
     *  _________________________________________________________________
     *  |       |          |                 |                  |       |
     *  | Start |  Addr  W | <Register Addr> | <Value to write> | Stop  |
     *  |_______|__________|_________________|__________________|_______|
     */
    // Ensure stop condition not pending

    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    do {
        // Send I2C start condition and address frame with W
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
//***************************************************************************************
        uint32_t t = 10000;
        while ((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT) && --t) { }
        if (!t || (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)) {
            // error: clear NACK, reset bus, bail
            EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            return;
        }
//***********************************************************************************************
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 1st data byte into TX buffer
        EUSCI_B0->TXBUF = PWR_MGMT_ADDR;            // send register address
        // wait for ACK/NACK after address frame
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
    } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
    // wait for TX buffer to be ready
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // load 2nd data byte into TX buffer
    EUSCI_B0->TXBUF = 0;                // write value to register
    // wait for 2nd data byte to begin to transmit
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // Send I2C stop condition
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    // ensure flags are cleared before enabling interrupts
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
            EUSCI_A_IE_TXIE |               // Enable transmit interrupt
            EUSCI_B_IE_NACKIE;              // Enable NACK interrupt
    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);
    // Enable global interrupt
    __enable_irq();

//Calibration**************************************************************************
    for ( cal_i = 0; cal_i < GYRO_CAL_SAMPLES; cal_i++) {
        ax=(int16_t)(RXData[0]<<8|RXData[1]);
        ay=(int16_t)(RXData[2]<<8|RXData[3]);
        az=(int16_t)(RXData[4]<<8|RXData[5]);
        // after accel, trigger next read for gyro (same registers)
        RXDataPointer = 0;
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;
        while (RXDataPointer < NUM_OF_REC_BYTES);
        gx=(int16_t)(RXData[0]<<8|RXData[1]);
        gy=(int16_t)(RXData[2]<<8|RXData[3]);
        gz=(int16_t)(RXData[4]<<8|RXData[5]);
        sumx += gx; sumy += gy; sumz += gz;
        for ( d = 2000; d > 0; d--);  // lazy delay
    }
    gx_off = sumx / GYRO_CAL_SAMPLES;
    gy_off = sumy / GYRO_CAL_SAMPLES;
    gz_off = sumz / GYRO_CAL_SAMPLES;
//*************************************************************************************

    while (1) {
        // Arbitrary delay before transmitting the next byte
        for (i = 2000; i > 0; i--);        // lazy delay
        //Switch button first:
        if ((SwitchPort->IN & Switch1) == 0) {
          for (i = 50000; i; --i);
          Pressed = 1;
        } else {
          Pressed = 0;
        }
        // Ensure stop condition got sent
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

        /* Read register values from sensor by sending register address and restart
         *  format for Write-Restart-Read operation
         *  _______________________________________________________________________
         *  |       | Periph | <Register |       | Periph |               |       |
         *  | Start |  Addr  |  Address> | Start |  Addr  | <6 Byte Read> | Stop  |
         *  |_______|____W___|___________|_______|____R___|_______________|_______|
         *  Initiated with start condition - completion handled in ISR
         */
        // change to transmitter mode (Write)
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
        // send I2C start condition with address frame and W bit
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        // wait for sensor data to be received
        while (RXDataPointer < NUM_OF_REC_BYTES) ;
        /* TODO combine bytes to form 16-bit acceleration values for accel_x, accel_y, accel_z */
        //Madgwick caomputation
        //*********************************************************************************************

            ax=(int16_t)(RXData[0]<<8|RXData[1]);
            ay=(int16_t)(RXData[2]<<8|RXData[3]);
            az=(int16_t)(RXData[4]<<8|RXData[5]);
            // after accel, trigger next read for gyro (same registers)
            EUSCI_B0->CTLW0|=EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;   // pointer to ACCEL_XOUT_H
            while(RXDataPointer<NUM_OF_REC_BYTES);
            gx=(int16_t)(RXData[0]<<8|RXData[1]);
            gy=(int16_t)(RXData[2]<<8|RXData[3]);
            gz=(int16_t)(RXData[4]<<8|RXData[5]);

            ax/=16384.0f; ay/=16384.0f; az/=16384.0f;
            gx=(gx-gx_off)/131.0f*(M_PI/180);
            gy=(gy-gy_off)/131.0f*(M_PI/180);
            gz=(gz-gz_off)/131.0f*(M_PI/180);

            float recipNorm, s0,s1,s2,s3;
            float qDot1=0.5f*(-q1*gx -q2*gy -q3*gz);
            float qDot2=0.5f*( q0*gx +q2*gz -q3*gy);
            float qDot3=0.5f*( q0*gy -q1*gz +q3*gx);
            float qDot4=0.5f*( q0*gz +q1*gy -q2*gx);
            recipNorm=1.0f/sqrtf(ax*ax+ay*ay+az*az);ax*=recipNorm;ay*=recipNorm;az*=recipNorm;
            float _2q0=2*q0,_2q1=2*q1,_2q2=2*q2,_2q3=2*q3;
            float _4q0=4*q0,_4q1=4*q1,_4q2=4*q2,_8q1=8*q1,_8q2=8*q2;
            float q0q0=q0*q0,q1q1=q1*q1,q2q2=q2*q2,q3q3=q3*q3;
            s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
            s1=_4q1*q3q3-_2q3*ax+4*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
            s2=4*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
            s3=4*q1q1*q3-_2q1*ax+4*q2q2*q3-_2q2*ay;
            recipNorm=1.0f/sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);s0*=recipNorm;s1*=recipNorm;s2*=recipNorm;s3*=recipNorm;
            qDot1-=BETA*s0; qDot2-=BETA*s1; qDot3-=BETA*s2; qDot4-=BETA*s3;
            q0+=qDot1*DELTA_T; q1+=qDot2*DELTA_T; q2+=qDot3*DELTA_T; q3+=qDot4*DELTA_T;
            recipNorm=1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3); q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

            // To Euler
            pitch = atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*57.2958f;
            roll  = asinf(2*(q0*q2-q3*q1))*57.2958f;
            yaw   = atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*57.2958f;
        //*********************************************************************************************
        accel_x =(RXData[0] << 8) + (RXData[1]);
        accel_y=(RXData[2] << 8) + (RXData[3]);
        accel_z=(RXData[4] << 8) + (RXData[5]);
        if(accel_x1-accel_x >5000){
            accel_x2 = 1;
        }else if(accel_x1-accel_x < -5000){
            accel_x2 = -1;
        }else{
            accel_x2 = 0;
        }
        accel_x1 = accel_x;// this is why

        if(accel_y1-accel_y >5000){
                    accel_y2 = 1;
                }else if(accel_y1-accel_y < -5000){
                    accel_y2 = -1;
                }else{
                    accel_y2 = 0;
                }
                accel_y1 = accel_y;

        if(accel_z1-accel_z >5000){
             accel_z2 = 1;
        }else if(accel_z1-accel_z < -5000){
              accel_z2 = -1;
        }else{
             accel_z2 = 0;
        }
         accel_z1 = accel_z;

         accel_x_g = (float) accel_x/fullscale1g;
         accel_y_g = (float) accel_y/fullscale1g;
         accel_z_g = (float) accel_z/fullscale1g;
         // send exactly “<X>,<Y>,<Z>\r\n”
         sprintf(buf,"P:%.2f,R:%.2f,Y:%.2f,%d\r\n",pitch,roll,yaw,Pressed);

         printf("P: %.2f\tR: %.2f\tY: %.2f\tBtn: %d\r\n",pitch, roll, yaw, Pressed);
//         sprintf(buf, "%d,%d,%d,%d\r\n",
//                 accel_x,
//                 accel_y,
//                 accel_z,
//                 Pressed);
         UART_sendString(buf);


//         printf("\n\r");
//         printf("\r\n   accel_x = %d", accel_x);
//         printf("   accel_y = %d", accel_y);
//         printf("   accel_z = %d", accel_z2);
//         printf("   Pressed = %d", Pressed);
         if(accel_x2 !=0){
             accel_x1 = 0;
             for (i = 200000; i > 0; i--);        // lazy delay
         }else if(accel_y2 !=0){
                      accel_y1 = 0;
                      for (i = 200000; i > 0; i--);        // lazy delay
          }else if(accel_z2 !=0){
                      accel_z1 = 0;
                      for (i = 200000; i > 0; i--);        // lazy delay
                  }
//         printf("\r\n   accel_x = %f", accel_x_g);
//         printf("   accel_y = %f", accel_y_g);
//         printf("   accel_z = %f", accel_z_g);
         RXDataPointer = 0;
         for (i = 2000; i > 0; i--);        // lazy delay

        RXDataPointer = 0;
    }
}



/*––– UART on eUSCI_A0 at 9 600 baud –––*/
void UART_init(void)
{
    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses
    // Route P1.2→UCA0RX, P1.3→UCA0TX
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);

    // Hold in reset
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;
    // SMCLK, 8N1, LSB first, UART mode
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST
                    | EUSCI_A_CTLW0_SSEL__SMCLK;

    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 9600
     * N = fBRCLK / Baud rate = 12000000/9600 = 1250
     * from Technical Reference manual Table 24-5:
     *
     * TODO lookup values for UCOS16, UCBRx, UCBRFx, and UCBRSx in Table 24-5
     */
    const uint16_t UCOS16  = 1;    // oversampling enable bit
    const uint16_t UCBRx   = 78;   // integer divisor
    const uint16_t UCBRFx  = 2;    // first-stage modulation (0–15)
    const uint16_t UCBRSx  = 0x0;  // second-stage modulation (Table 24-5)

    EUSCI_A2->BRW = UCBRx;
    EUSCI_A2->MCTLW = (UCBRSx<<8)|(UCBRFx<<4)|UCOS16;
    // Release from reset
    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
//    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
//    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A2 RX interrupt
}

void UART_sendChar(char c)
{
    // wait until TX buffer is free
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = c;
}

void UART_sendString(const char *str)
{
    while (*str) {
        UART_sendChar(*str++);
    }
}
// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    // Handle if ACK not received for address frame
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // resend I2C start condition and address frame
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        TXDataPointer = 0;
        RXDataPointer = 0;
    }
    // When TX buffer is ready, load next byte or Restart for Read
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        if (TXDataPointer == 0) {
            // load 1st data byte into TX buffer (writing to buffer clears the flag)
            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
            TXDataPointer = 1;
        } else {
            // change to receiver mode (Read)
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            // send Restart and address frame with R bit
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            TXDataPointer = 0;
            RXDataPointer = 0;
            // need to clear flag since not writing to buffer
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        }
    }
    // When new byte is received, read value from RX buffer
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
        // Get RX data
        if (RXDataPointer < NUM_OF_REC_BYTES) {
            // reading the buffer clears the flag
            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
        }
        else {  // in case of glitch, avoid array out-of-bounds error
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
        }
        // check if last byte being received - if so, initiate STOP (and NACK)
        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
            // Send I2C stop condition
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }
} //end of void EUSCIB0_IRQHandler(void)
