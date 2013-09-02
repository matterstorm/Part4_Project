/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *----------------------------------------------------------------------------
 */

/**********************************************************************************************
  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "cc430f6137.h"
#include "app_remap_led.h"

#ifndef APP_AUTO_ACK
#error ERROR: Must define the macro APP_AUTO_ACK for this application.
#endif

void toggleLED(uint8_t);

/**************************** COMMENTS ON ASYNC LISTEN APPLICATION ***********************
Summary:
  This AP build includes implementation of an unknown number of end device peers in
  addition to AP functionality. In this scenario all End Devices establish a link to
  the AP and only to the AP. The AP acts as a data hub. All End Device peers are on
  the AP and not on other distinct ED platforms.

  There is still a limit to the number of peers supported on the AP that is defined
  by the macro NUM_CONNECTIONS. The AP will support NUM_CONNECTIONS or fewer peers
  but the exact number does not need to be known at build time.

  In this special but common scenario SimpliciTI restricts each End Device object to a
  single connection to the AP. If multiple logical connections are required these must
  be accommodated by supporting contexts in the application payload itself.

Solution overview:
  When a new peer connection is required the AP main loop must be notified. In essence
  the main loop polls a semaphore to know whether to begin listening for a peer Link
  request from a new End Device. There are two solutions: automatic notification and
  external notification. The only difference between the automatic notification
  solution and the external notification solution is how the listen semaphore is
  set. In the external notification solution the sempahore is set by the user when the
  AP is stimulated for example by a button press or a commend over a serial link. In the
  automatic scheme the notification is accomplished as a side effect of a new End Device
  joining.

  The Rx callback must be implemented. When the callback is invoked with a non-zero
  Link ID the handler could set a semaphore that alerts the main work loop that a
  SMPL_Receive() can be executed successfully on that Link ID.

  If the callback conveys an argument (LinkID) of 0 then a new device has joined the
  network. A SMPL_LinkListen() should be executed.

  Whether the joining device supports ED objects is indirectly inferred on the joining
  device from the setting of the NUM_CONNECTIONS macro. The value of this macro should
  be non-zero only if ED objects exist on the device. This macro is always non-zero
  for ED-only devices. But Range Extenders may or may not support ED objects. The macro
  should be be set to 0 for REs that do not also support ED objects. This prevents the
  Access Point from reserving resources for a joinng device that does not support any
  End Device Objects and it prevents the AP from executing a SMPL_LinkListen(). The
  Access Point will not ever see a Link frame if the joining device does not support
  any connections.

  Each joining device must execute a SMPL_Link() after receiving the join reply from the
  Access Point. The Access Point will be listening.

*************************** END COMMENTS ON ASYNC LISTEN APPLICATION ********************/

/************  THIS SOURCE FILE REPRESENTS THE AUTOMATIC NOTIFICATION SOLUTION ************/

unsigned char *PRxData;                     // Pointer to RX data
unsigned char RXByteCtr;
volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM
unsigned int Radio_Recieved = 0;
void I2C_init(void);


unsigned char reg = 0x00;
unsigned int i= 0;
#define I2C_MESSAGE_SIZE 2
#define RADIO_MESSAGE_SIZE 4

const unsigned char VoltageData[] =              // Table of data to transmit
{
		0x11,
		0x22,
		0x33,
		0x44
};
const unsigned char CurrentData[] =              // Table of data to transmit
{
		0x12,
		0x23,
		0x34,
		0x45
};


/* reserve space for the maximum possible peer Link IDs */
static linkID_t sLID[NUM_CONNECTIONS] = {0};
static uint8_t  sNumCurrentPeers = 0;

/* callback handler */
static uint8_t sCB(linkID_t);

/* received message handler */
static void processMessage(linkID_t, uint8_t *, uint8_t);

/* Frequency Agility helper functions */
static void    checkChangeChannel(void);
static void    changeChannel(void);

/* work loop semaphores */
static volatile uint8_t sPeerFrameSem = 0;
static volatile uint8_t sJoinSem = 0;

#ifdef FREQUENCY_AGILITY
/*     ************** BEGIN interference detection support */

#define INTERFERNCE_THRESHOLD_DBM (-70)
#define SSIZE    25
#define IN_A_ROW  3
static int8_t  sSample[SSIZE];
static uint8_t sChannel = 0;
#endif  /* FREQUENCY_AGILITY */

#define Data_Width 4
#define MISSES_IN_A_ROW  2

/* blink LEDs when channel changes... */
static volatile uint8_t sBlinky = 0;

/*     ************** END interference detection support                       */

#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

bspIState_t intState;

void main (void)
{

  //memset(sSample, 0x0, sizeof(sSample));
  
  BSP_Init();



  /* If an on-the-fly device address is generated it must be done before the
   * call to SMPL_Init(). If the address is set here the ROM value will not
   * be used. If SMPL_Init() runs before this IOCTL is used the IOCTL call
   * will not take effect. One shot only. The IOCTL call below is conformal.
   */
#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
  {
    addr_t lAddr;

    createRandomAddress(&lAddr);
    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
  }
#endif /* I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE */

  SMPL_Init(sCB);

  /* green and red LEDs on solid to indicate waiting for a Join. */
  if (!BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }


  I2C_init();

	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
	UCB0I2COA = 0x48;                         // Own Address is 048h
	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
	UCB0IE |= UCSTPIE + UCSTTIE + UCRXIE + UCTXIE;     // Enable STT, STP & RX interrupt

	SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SETPWR, (void*)IOCTL_LEVEL_2 );
	freqEntry_t init_freq;
	init_freq.logicalChan = 0;
	SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &init_freq);
  /* main work loop */
  while (1)
  {
    /* Wait for the Join semaphore to be set by the receipt of a Join frame from a
     * device that supports an End Device.
     *
     * An external method could be used as well. A button press could be connected
     * to an ISR and the ISR could set a semaphore that is checked by a function
     * call here, or a command shell running in support of a serial connection
     * could set a semaphore that is checked by a function call.
     */

	__bis_SR_register(GIE);     // Enter, enable interrupt


    if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
    {
      /* listen for a new connection */
      while (1)
      {
        if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
        {
          break;
        }
        /* Implement fail-to-link policy here. otherwise, listen again. */
      }

      sNumCurrentPeers++;

      BSP_ENTER_CRITICAL_SECTION(intState);
      sJoinSem--;
      BSP_EXIT_CRITICAL_SECTION(intState);
    }

    /* Have we received a frame on one of the ED connections?
     * No critical section -- it doesn't really matter much if we miss a poll
     */
    /*if (sPeerFrameSem)
    {
      uint8_t     msg[MAX_APP_PAYLOAD], len, i;

      /* process all frames waiting */
     /* for (i=0; i<sNumCurrentPeers; ++i)
      {
        if (SMPL_SUCCESS == SMPL_Receive(sLID[i], msg, &len))
        {
          processMessage(sLID[i], msg, len);

          BSP_ENTER_CRITICAL_SECTION(intState);
          sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);
        }
      }
    }*/
    if (BSP_BUTTON1())
    {
      SPIN_ABOUT_A_QUARTER_SECOND;  /* debounce */
      changeChannel();
    }
    else
    {
      checkChangeChannel();
    }
/*    BSP_ENTER_CRITICAL_SECTION(intState);
    if (sBlinky)
    {
      if (++sBlinky >= 0xF)
      {
        sBlinky = 1;
        toggleLED(1);
        toggleLED(2);
      }
    }
    BSP_EXIT_CRITICAL_SECTION(intState);*/
    __no_operation();
  }

}

void toggleLED(uint8_t which)
{
  if (1 == which)
  {
    BSP_TOGGLE_LED1();
  }
  else if (2 == which)
  {
    BSP_TOGGLE_LED2();
  }

  return;
}

/* Runs in ISR context. Reading the frame should be done in the */
/* application thread not in the ISR thread. */
static uint8_t sCB(linkID_t lid)
{
  if (lid)
  {
	Radio_Recieved=1;
    sPeerFrameSem++;
    sBlinky = 0;
  }
  else
  {
    sJoinSem++;
  }

  /* leave frame to be read by application. */
  return 0;
}

static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len)
{
  /* do something useful */
  if (len)
  {
    toggleLED(*msg);
  }
  return;
}

static void changeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  freqEntry_t freq;

  if (++sChannel >= NWK_FREQ_TBL_SIZE)
  {
    sChannel = 0;
  }
  freq.logicalChan = sChannel;
  SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
  //BSP_TURN_OFF_LED1();
  //BSP_TURN_OFF_LED2();
  sBlinky = 1;
#endif
  return;
}

/* implement auto-channel-change policy here... */
static void  checkChangeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  int8_t dbm, inARow = 0;

  uint8_t i;

  memset(sSample, 0x0, SSIZE);
  for (i=0; i<SSIZE; ++i)
  {
    /* quit if we need to service an app frame */
    if (sPeerFrameSem || sJoinSem)
    {
      return;
    }
    NWK_DELAY(1);
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, (void *)&dbm);
    sSample[i] = dbm;

    if (dbm > INTERFERNCE_THRESHOLD_DBM)
    {
      if (++inARow == IN_A_ROW)
      {
        changeChannel();
        break;
      }
    }
    else
    {
      inARow = 0;
    }
  }
#endif
  return;
}

//------------------------------------------------------------------------------
// The USCI_B0 data ISR RX vector is used to move received data from the I2C
// master to the MSP430 memory.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// The USCI_B0 state ISR TX vector is used to wake up the CPU from LPM0 in order
// to process the received data in the main program. LPM0 is only exit in case
// of a (re-)start or stop condition when actual data was received.
//------------------------------------------------------------------------------
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	uint8_t done,misses;
	uint8_t      noAck;
	smplStatus_t rc;
	uint8_t  I2C_msg[I2C_MESSAGE_SIZE];
	uint8_t  Radio_msg[RADIO_MESSAGE_SIZE];
	uint8_t  len;
	switch(__even_in_range(UCB0IV,12))
	{
		case  0: break;                           // Vector  0: No interrupts

		case  2: break;                           // Vector  2: ALIFG

		case  4: break;                           // Vector  4: NACKIFG

		case  6:                                  // Vector  6: STTIFG
		UCB0IFG &= ~UCSTTIFG;
		break;


		case  8:                                  // Vector  8: STPIFG
			UCB0IFG &= ~UCSTPIFG;
			/*if (RXByteCtr)                          // Check RX byte counter
				__bic_SR_register_on_exit(LPM0_bits);
			break;*/

		case 10:                                  // Vector 10: RXIFG
			reg = (uint8_t) UCB0RXBUF;		//Set reg to the value of register wanted to be read from
			I2C_msg[1]=reg;
			I2C_msg[0]=0;
			done=0;
			while (!done)
			{
				noAck = 0;

				/* Try sending message MISSES_IN_A_ROW times looking for ack */
				for (misses=0; misses < MISSES_IN_A_ROW; ++misses)
				{
				  if (SMPL_SUCCESS == (rc=SMPL_SendOpt(sLID[sNumCurrentPeers-1], I2C_msg, sizeof(I2C_msg), SMPL_TXOPTION_ACKREQ)))
				  {
					/* Message acked. We're done. Toggle LED 1 to indicate ack received. */
					toggleLED(1);
					break;
				  }
				  if (SMPL_NO_ACK == rc)
				  {
					/* Count ack failures. Could also fail becuase of CCA and
					 * we don't want to scan in this case.
					 */
					noAck++;
				  }
				}
				if (MISSES_IN_A_ROW == noAck)
				{
				  /* Message not acked. Toggle LED 2. */
				  toggleLED(2);
				#ifdef FREQUENCY_AGILITY
				  /* Assume we're on the wrong channel so look for channel by
				   * using the Ping to initiate a scan when it gets no reply. With
				   * a successful ping try sending the message again. Otherwise,
				   * for any error we get we will wait until the next button
				   * press to try again.
				   */
				  if (SMPL_SUCCESS != SMPL_Ping(sLID[sNumCurrentPeers-1]))
				  {
					done = 1;
				  }
				#else
				  done = 1;
				#endif  /* FREQUENCY_AGILITY */
				}
				else
				{
				  /* Got the ack or we don't care. We're done. */
				  done = 1;
				}
			}

			break;
		case 12:
			/*switch(reg)
			{
				case 0x01:
					UCB0TXBUF=VoltageData[i/2];
					break;
				case 0x02:
					UCB0TXBUF=CurrentData[i/2];
					break;
				default: break;
			}
			i++;
			if(i>=Data_Width*2){
				i=0;
			}*/

			while(!Radio_Recieved);
			if (SMPL_SUCCESS == SMPL_Receive(sLID[sLID[sNumCurrentPeers-1]], Radio_msg, &len))
				{
				  //processMessage(sLID[sLID[sNumCurrentPeers-1]], msg, len);

				  BSP_ENTER_CRITICAL_SECTION(intState);
				  sPeerFrameSem--;
				  BSP_EXIT_CRITICAL_SECTION(intState);
				}
			switch(reg)
			{
				case 0x01:
					UCB0TXBUF=VoltageData[i/2];
					break;
				case 0x02:
					UCB0TXBUF=CurrentData[i/2];
					break;
				default: break;
			}
			i++;
			if(i>=Data_Width*2){
				i=0;
			}
			break;                           // Vector 12: TXIFG

		default: break;
		}
}

void I2C_init(void) {
	__disable_interrupt();						// Disable interrupts - needed as any interrupts occuring during pin remapping will cause remapping to fail

	PMAPKEYID = 0x02D52;                      // Get write-access to port mapping regs
	//P3OUT &= ~BIT6;
	//P3DIR |= BIT6;
	P1MAP3 = PM_UCB0SDA;                      // Map UCB0SDA output to P1.3
	P1MAP2 = PM_UCB0SCL;                      // Map UCB0SCL output to P1.2
	PMAPKEYID = 0;                            // Lock port mapping registers
	__enable_interrupt();						// Enable interrupts

	P1SEL |= BIT2 + BIT3;                     // Select P1.2 & P1.3 to I2C function
}
