/*
 * Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"


#if CPU_INIT_CONFIG
  #include "Init_Config.h"
#endif


volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */

#include <stdint.h>
#include <stdbool.h>

/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/



#define GPIO_PORTA                     PTA
#define GPIO_PORTB           		   PTB
#define GPIO_PORTC                     PTC
#define GPIO_PORTD           		   PTD
#define GPIO_PORTE                     PTE



#define Right_Indicator             1U//PTD
#define Left_Indicator				0U//PTD
#define GLYPH1						14//PTC
#define GLYPH2						15//PTC
//#define	GLYPH3				    16//PTC
//#define GLYPH4					17//PTC
#define Cluster_Screen_Trig          7//PTE
#define Battery_Unlock	             1//PTE
#define Boot_Unlock   				11//PTE
#define Discharge_Contactor  		16//PTD
#define Charge_Contactor			 9//PTE
#define Motor_Ignition				11//PTA
#define Motor_Park					 2//PTE
#define Light_Sound_Relay			 0//PTC
#define Throttle_Relay				 3//PTB
#define Display_Ignition_Relay		 2//PTB
#define SL_Motor_Op1				 5//PTB
#define SL_Motor_Op2				 8//PTE




//CAN Tx Frame ID and MAILBOX Setup for CAN_PAL1
#define TX1VOLTAGE_MAILBOX_1       (1UL)
#define TX1VOLTAGE_ID1             (0x100)
#define TX1SOC_MAILBOX_2           (2UL)
#define TX1SOC_ID2                 (0x101)
#define TX1STATUS_MAILBOX_3	       (3UL)
#define TX1STATUS_ID3	           (0x102)
#define TX1TEMP1_MAILBOX_4	       (4UL)
#define TX1TEMP1_ID4			   (0x105)
#define TX1TEMP2_MAILBOX_5		   (5UL)
#define TX1TEMP2_ID5			   (0x106)




//CAN Rx Frame ID and MAILBOX Setup for CAN_PAL1
#define RX1VOLTAGE_MAILBOX_1       (6UL)
#define RX1VOLTAGE_ID1             (0x100)
#define RX1SOC_MAILBOX_2           (7UL)
#define RX1SOC_ID2                 (0x101)
#define RX1STATUS_MAILBOX_3	       (8UL)
#define RX1STATUS_ID3	           (0x102)
#define RX1TEMP1_MAILBOX_4	       (9UL)
#define RX1TEMP1_ID4			   (0x105)
#define RX1TEMP2_MAILBOX_5		   (10UL)
#define RX1TEMP2_ID5			   (0x106)


//CAN Tx Frame ID and MAILBOX Setup for CAN_PAL2
#define TX2VOLTAGE_MAILBOX_1       (11UL)
#define TX2VOLTAGE_ID1             (0x100)
#define TX2SOC_MAILBOX_2           (12UL)
#define TX2SOC_ID2                 (0x101)
#define TX2STATUS_MAILBOX_3	       (13UL)
#define TX2STATUS_ID3	           (0x102)
#define TX2TEMP1_MAILBOX_4	       (14UL)
#define TX2TEMP1_ID4			   (0x105)
#define TX2TEMP2_MAILBOX_5		   (15UL)
#define TX2TEMP2_ID5			   (0x106)
#define TX2stefen_SOC_MAILBOX_6    (10UL) /* mailbox index must be < maxBuffNum (16). 16 is out-of-range */
#define TX2stefen_SOC_ID6		   (0x310)



// RFID command frames
uint8_t Inventory_cmd[5] = {0x03,0x2F,0x01,0x2D};
uint8_t Blankarray[11];
uint32_t Timeout = 100; /* Timeout in ms for blocking operations */
uint8_t arrayindex = 0 ;
uint8_t arrayindex1 = 0;
uint8_t arrayindex3 = 0;
uint8_t Motor_CAN_Receive =0;



// SYSTEM STATES
uint8_t SYSTEM_PARK,SYSTEM_ON,SYSTEM_READY,SYSTEM_MOVING,BATTERY_LOCKED,BOOT_LOCKED,CHARGING = 0;

uint16_t stefen_SOC;
uint16_t micronix_voltage,stefen_voltage;






void delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}


void BoardInit(void)
{

/* Initialize and configure clocks
*  -   Setup system clocks, dividers
*  -   Configure FlexCAN clock, GPIO, LPSPI
*  -   see clock manager component for more details
*/
CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
			g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

/* Initialize pins
*  -   Init FlexCAN, LPSPI and GPIO pins
*  -   See PinSettings component for more info
*/
PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

void Screen_Trigger_ON (void)
{

	//Battery Pack Unlock
	if((PTD -> PDIR & (1<<2)) == 0 )        //Soft switch one pressed for Battery Unlock
	{
		PINS_DRV_WritePin(GPIO_PORTE,Battery_Unlock,1);
		delay (100000);//adjust the delay based on the actuator duty cycle
		PINS_DRV_WritePin(GPIO_PORTE,Battery_Unlock,0);
	}

	//Boot Unlock Operation
if((PTD -> PDIR & (1<<3)) == 0 )      //Soft switch two pressed for Boot Unlock
	{
		PINS_DRV_WritePin(GPIO_PORTE,Boot_Unlock,1);
		delay (12500000);//adjust the delay based on the actuator duty cycle
		PINS_DRV_WritePin(GPIO_PORTE,Boot_Unlock,0);
	}

	//Charging_Contactor Operation
	if((PTD -> PDIR & (1<<4)) == 0 )  	 //Soft switch two state
	{
		PINS_DRV_WritePin(GPIO_PORTE,Charge_Contactor,1);//Charging Contactor Turned ON
		CHARGING = 1;//Flag raised for Charging
	}
	else if ((PTD -> PDIR & (1<<4)) != 0 )
	{
		PINS_DRV_WritePin(GPIO_PORTE,Charge_Contactor,0);//Charging Contactor Turned OFF
		CHARGING = 0;//Flag raised for Charging Disconnect
	}
}

void Discharge_CONTACTOR_ON (void)
{
	if ((PTE -> PDIR & (1<<0)) == 0)//Battery Position Switch
	{
		PINS_DRV_WritePin(GPIO_PORTD,Discharge_Contactor,1); //Discharge Contactor ON
		BATTERY_LOCKED = 1;
	}
	else
	{
		BATTERY_LOCKED = 0;
	}
}

void Steering_Unlock(void)

{
	//if((PTE->PDIR & (1<<4)) == 0 && (PTE->PDIR & (1<<3)) != 1 ) //  this condition checks the LOCK state piston position (it should satisify the lock state piston position)
	//{

		PINS_DRV_WritePin(GPIO_PORTB, SL_Motor_Op1,0);
		PINS_DRV_WritePin(GPIO_PORTE, SL_Motor_Op2,1);
		delay (750000); //map this delay to around the same time till it unlock
		PINS_DRV_WritePin(GPIO_PORTB, SL_Motor_Op1,0);
		PINS_DRV_WritePin(GPIO_PORTE, SL_Motor_Op2,0);

	//}

}


void Steering_lock(void)

{
	//if((PTE->PDIR & (1<<4)) == 0 && (PTE->PDIR & (1<<3)) != 1 ) //  this condition checks the LOCK state piston position (it should satisify the lock state piston position)
	//{

		PINS_DRV_WritePin(GPIO_PORTB, SL_Motor_Op1,1);
		PINS_DRV_WritePin(GPIO_PORTE, SL_Motor_Op2,0);
		delay (750000); //map this delay to around the same time till it locks
		PINS_DRV_WritePin(GPIO_PORTB, SL_Motor_Op1,0);
		PINS_DRV_WritePin(GPIO_PORTE, SL_Motor_Op2,0);

	//}

}


void Indicator_Light_Start_Stop (void)
{


// flag required to run this function once when called upon
//	static bool functionExecuted = false;
//	if(!functionExecuted)
//

	PINS_DRV_WritePin(GPIO_PORTD,Right_Indicator,1);
	PINS_DRV_WritePin(GPIO_PORTD, Left_Indicator,1);

	delay (1000000);

	PINS_DRV_WritePin(GPIO_PORTD,Right_Indicator,0);
	PINS_DRV_WritePin(GPIO_PORTD,Left_Indicator,0);

	delay (1000000);

	PINS_DRV_WritePin(GPIO_PORTD,Right_Indicator,1);
	PINS_DRV_WritePin(GPIO_PORTD, Left_Indicator,1);

	delay (1000000);

	PINS_DRV_WritePin(GPIO_PORTD,Right_Indicator,0);
	PINS_DRV_WritePin(GPIO_PORTD,Left_Indicator,0);

//
//	functionExecuted = true;
//	}


}





void Vehicle_Switching_Operation(void)
{
	if (Blankarray[6]==0x9D && Blankarray[7]==0x30 && Blankarray[8]==0x49 && Blankarray[9]==0x6B)
	{
		PINS_DRV_WritePin(GPIO_PORTB,Display_Ignition_Relay,1);
		delay(10000000);
		PINS_DRV_WritePin(GPIO_PORTE,Cluster_Screen_Trig,1);//Sleep/wake Signal Cluster
		Steering_Unlock();
		Indicator_Light_Start_Stop ();
		SYSTEM_PARK = 1;
		BATTERY_LOCKED = 1; //since VE4 is a fixed battery pack

		 // Reset array values to zero
		    Blankarray[6] = 0;
		    Blankarray[7] = 0;
		    Blankarray[8] = 0;
		    Blankarray[9] = 0;
	}

	Screen_Trigger_ON ();// this function allows to run the soft switches


	if (CHARGING == 0 && SYSTEM_PARK == 1 && (PTC->PDIR & (1<<8)) == 0)// RED KEY SWITCH ON
	{
		PINS_DRV_WritePin(GPIO_PORTE,Cluster_Screen_Trig,0);
		PINS_DRV_WritePin(GPIO_PORTD,Discharge_Contactor,1);//since the battery is not removed it can directly be turned On
		delay(100000);//delay required, so, power doesn't flow immediately after the contactor turned on
		if (BATTERY_LOCKED ==1)
		{
		PINS_DRV_WritePin(GPIO_PORTC,Light_Sound_Relay,1);
		SYSTEM_ON = 1;
		SYSTEM_PARK = 0;
		}

	}

    if (SYSTEM_ON==1 && (PTC->PDIR & (1<<9)) == 0) // Brake Pressed
    {
		 if (SYSTEM_PARK == 0 && (PTA -> PDIR & (1<<7)) == 0)//Push Button 1 Pressed
		 {
			PINS_DRV_WritePin(GPIO_PORTE,Motor_Park,1);//SONACOM M.CON START SEQ
			SYSTEM_READY = 1;
		 }
    }

    if (  SYSTEM_ON==1 && SYSTEM_READY==1) // Throttle Enable
    {
    	PINS_DRV_WritePin(GPIO_PORTB,Throttle_Relay,1);
    	SYSTEM_MOVING = 1;
    }

    if (SYSTEM_ON == 1 && (PTC->PDIR & (1<<8)) !=0 ) // RED KEY SWITCH OFF
    {
    	PINS_DRV_WritePin(GPIO_PORTC,Light_Sound_Relay,0);
    	PINS_DRV_WritePin(GPIO_PORTE,Motor_Park,0);
    	PINS_DRV_WritePin(GPIO_PORTB,Throttle_Relay,0);
    	PINS_DRV_WritePin(GPIO_PORTD,Discharge_Contactor,0);
    	PINS_DRV_WritePin(GPIO_PORTE,Cluster_Screen_Trig,1);
    	SYSTEM_ON = 0;
    	SYSTEM_READY = 0;
    	SYSTEM_MOVING = 0;
    	SYSTEM_PARK = 1;

    }

    if (SYSTEM_PARK == 1 &&  (PTA -> PDIR & (1<<7)) == 0) //Push Button 1 Pressed
    {
    	PINS_DRV_WritePin(GPIO_PORTB,Display_Ignition_Relay,0);
    	Indicator_Light_Start_Stop ();
    	Steering_lock();
		SYSTEM_PARK = 0;
    }
}

int main(void)
{

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                 /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/


    BoardInit();

  //RFID LPUART Initialization
  	LPUART_DRV_Init(INST_LPUART_RFID, &lpuart_rfid_State, &lpuart_rfid_InitConfig0);
  	delay (10000);


 // CAN Initialization+
  CAN_Init(INST_CAN_PAL1,&can_pal1_Config0);
  CAN_Init(INST_CAN_PAL2,&can_pal2_Config0);

//CAN Buffer Configuration for CAN PAL1 and CAN_PAL2
  can_buff_config_t stdbuff = {
							.enableFD = true,
							.enableBRS = true,
							.idType = CAN_MSG_ID_STD
							};

  //Configure Tx Buffer with Tx_Mailbox for CAN_PAL1

  CAN_ConfigTxBuff(INST_CAN_PAL1, TX1VOLTAGE_MAILBOX_1, &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL1, TX1SOC_MAILBOX_2,     &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL1, TX1STATUS_MAILBOX_3,  &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL1, TX1TEMP1_MAILBOX_4,   &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL1, TX1TEMP2_MAILBOX_5,   &stdbuff);



  //Configure Rx Buffer with Rx_mailbox for CAN_PAL1

  CAN_ConfigRxBuff(INST_CAN_PAL1, RX1VOLTAGE_MAILBOX_1, &stdbuff, RX1VOLTAGE_ID1);
  CAN_ConfigRxBuff(INST_CAN_PAL1, RX1SOC_MAILBOX_2,     &stdbuff, RX1SOC_ID2);
  CAN_ConfigRxBuff(INST_CAN_PAL1, RX1STATUS_MAILBOX_3,  &stdbuff, RX1STATUS_ID3);
  CAN_ConfigRxBuff(INST_CAN_PAL1, RX1TEMP1_MAILBOX_4,   &stdbuff, RX1TEMP1_ID4);
  CAN_ConfigRxBuff(INST_CAN_PAL1, RX1TEMP2_MAILBOX_5,   &stdbuff, RX1TEMP2_ID5);


  //Configure Tx Buffer with Tx_Mailbox for CAN_PAL2

  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2VOLTAGE_MAILBOX_1, &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2SOC_MAILBOX_2,     &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2STATUS_MAILBOX_3,  &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2TEMP1_MAILBOX_4,   &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2TEMP2_MAILBOX_5,   &stdbuff);
  CAN_ConfigTxBuff(INST_CAN_PAL2, TX2stefen_SOC_MAILBOX_6,   &stdbuff);


  // Define messages in TX1 send buffer

  can_message_t TX1VOLTAGE = {
              	  .cs = 0U,
              	  .id = TX1VOLTAGE_ID1,
.data[0] = 0x5A,.data[1] = 00,.data[2] = 00,.data[3] = 00,.data[4] = 00,.data[5] = 00,.data[6] = 00,.data[7] = 00,
                    .length = 8U
              	          };


  can_message_t TX1SOC = {
              	  .cs = 0U,
              	  .id = TX1SOC_ID2,
.data[0] = 0x5A,.data[1] = 00,.data[2] = 00,.data[3] = 00,.data[4] = 00,.data[5] = 00,.data[6] = 00,.data[7] = 00,
                    .length = 8U
              	          };


  can_message_t TX1STATUS = {
              	  .cs = 0U,
              	  .id = TX1STATUS_ID3,
.data[0] = 0x5A,.data[1] = 00,.data[2] = 00,.data[3] = 00,.data[4] = 00,.data[5] = 00,.data[6] = 00,.data[7] = 00,
                    .length = 8U
              	          };


  can_message_t TX1TEMP1 = {
              	  .cs = 0U,
              	  .id = TX1TEMP1_ID4,
.data[0] = 0x5A,.data[1] = 00,.data[2] = 00,.data[3] = 00,.data[4] = 00,.data[5] = 00,.data[6] = 00,.data[7] = 00,
                    .length = 8U
              	          };


  can_message_t TX1TEMP2 = {
              	  .cs = 0U,
              	  .id = TX1TEMP2_ID5,
.data[0] = 0x5A,.data[1] = 00,.data[2] = 00,.data[3] = 00,.data[4] = 00,.data[5] = 00,.data[6] = 00,.data[7] = 00,
                    .length = 8U
              	          };



  //Setup and Initialization for Receiving the  CAN messages
  can_message_t RX1VOLTAGE;
  can_message_t RX1SOC;
  can_message_t RX1STATUS;
  can_message_t RX1TEMP1;
  can_message_t RX1TEMP2;


  while (1)
{

//  LPUART Send and Receive loop setup
	LPUART_DRV_SendDataBlocking(INST_LPUART_RFID,Inventory_cmd,5, Timeout);

	LPUART_DRV_ReceiveDataBlocking(INST_LPUART_RFID,Blankarray,20, Timeout);


//	Vehicle Switching Operation flow.
	Vehicle_Switching_Operation();


//Transmitt the TX1 frames

	  delay(5000);
	  if (CAN_Send(INST_CAN_PAL1, TX1VOLTAGE_MAILBOX_1,     &TX1VOLTAGE) == STATUS_SUCCESS)

	  	{
		  arrayindex++;
	  	 }

	  delay (5000);
	  if (CAN_Send(INST_CAN_PAL1, TX1SOC_MAILBOX_2,     &TX1SOC) == STATUS_SUCCESS)
	  	{
		  arrayindex++;

	  	 }

	  delay(5000);
	  if (CAN_Send(INST_CAN_PAL1, TX1STATUS_MAILBOX_3,  &TX1STATUS) == STATUS_SUCCESS)
	  	{
		  arrayindex++;

	  	 }

	  delay(5000);
	  if (CAN_Send(INST_CAN_PAL1, TX1TEMP1_MAILBOX_4,   &TX1TEMP1) == STATUS_SUCCESS)
	  	{
		  arrayindex++;

	  	 }

	  delay(5000);
	  if (CAN_Send(INST_CAN_PAL1, TX1TEMP2_MAILBOX_5,   &TX1TEMP2) == STATUS_SUCCESS)
	  	{
		  arrayindex++;

	  	 }


//Receiving the RX1 frames

	  if (CAN_Receive(INST_CAN_PAL1, RX1VOLTAGE_MAILBOX_1, &RX1VOLTAGE) == STATUS_SUCCESS)
	 	  	{
		  	  micronix_voltage = ((uint16_t)RX1VOLTAGE.data[0]<<8) | (uint16_t)RX1VOLTAGE.data[1];//Big Endian mapping
		  	  //micronix_voltage = ((uint16_t)RX1VOLTAGE.data[1]<<8) | (uint16_t)RX1VOLTAGE.data[0];//Little Endian mapping
		  	  stefen_voltage = micronix_voltage/10;
	 	  	 }

	  if (CAN_Receive(INST_CAN_PAL1, RX1SOC_MAILBOX_2,     &RX1SOC) == STATUS_SUCCESS)
		{
		 stefen_SOC = RX1SOC.data[5] *10;

		 }


	  if (CAN_Receive(INST_CAN_PAL1, RX1STATUS_MAILBOX_3,  &RX1STATUS) == STATUS_SUCCESS)
		{

		}

	  if (CAN_Receive(INST_CAN_PAL1, RX1TEMP1_MAILBOX_4,   &RX1TEMP1) == STATUS_SUCCESS)
		{

		 }

	  if (CAN_Receive(INST_CAN_PAL1, RX1TEMP2_MAILBOX_5,   &RX1TEMP2) == STATUS_SUCCESS)
		{

		 }


// Set up for the  TX2 Message with the shifted Payloads from RX1 Message


can_message_t TX2VOLTAGE = {
					  .cs = 0U,
					  .id = TX2VOLTAGE_ID1,
				 .data[0] = RX1VOLTAGE.data[0],
				 .data[1] = RX1VOLTAGE.data[1],
				 .data[2] = RX1VOLTAGE.data[2],
				 .data[3] = RX1VOLTAGE.data[3],
				 .data[4] = RX1VOLTAGE.data[4],
				 .data[5] = RX1VOLTAGE.data[5],
				 .data[6] = RX1VOLTAGE.data[6],
				 .data[7] = RX1VOLTAGE.data[7],
				  .length = 8U
							};


can_message_t TX2SOC = {
					  .cs = 0U,
					  .id = TX2SOC_ID2,
				 .data[0] = RX1SOC.data[0],
				 .data[1] = RX1SOC.data[1],
				 .data[2] = RX1SOC.data[2],
				 .data[3] = RX1SOC.data[3],
				 .data[4] = RX1SOC.data[4],
				 .data[5] = RX1SOC.data[5],
				 .data[6] = RX1SOC.data[6],
				 .data[7] = RX1SOC.data[7],
				  .length = 8U
							};


can_message_t TX2STATUS = {
					  .cs = 0U,
					  .id = TX2STATUS_ID3,
				 .data[0] = RX1STATUS.data[0],
				 .data[1] = RX1STATUS.data[1],
				 .data[2] = RX1STATUS.data[2],
				 .data[3] = RX1STATUS.data[3],
				 .data[4] = RX1STATUS.data[4],
				 .data[5] = RX1STATUS.data[5],
				 .data[6] = RX1STATUS.data[6],
				 .data[7] = RX1STATUS.data[7],
				  .length = 8U
					        };

can_message_t TX2TEMP1 = {
            	  .cs = 0U,
            	  .id = TX2TEMP1_ID4,
             .data[0] = RX1TEMP1.data[0],
		     .data[1] = RX1TEMP1.data[1],
			 .data[2] = RX1TEMP1.data[2],
			 .data[3] = RX1TEMP1.data[3],
			 .data[4] = RX1TEMP1.data[4],
			 .data[5] = RX1TEMP1.data[5],
			 .data[6] = RX1TEMP1.data[6],
			 .data[7] = RX1TEMP1.data[7],
              .length = 8U
            	        };

can_message_t TX2TEMP2 = {
            	  .cs = 0U,
            	  .id = TX2TEMP2_ID5,
             .data[0] = RX1TEMP2.data[0],
		     .data[1] = RX1TEMP2.data[1],
			 .data[2] = RX1TEMP2.data[2],
			 .data[3] = RX1TEMP2.data[3],
			 .data[4] = RX1TEMP2.data[4],
			 .data[5] = RX1TEMP2.data[5],
			 .data[6] = RX1TEMP2.data[6],
			 .data[7] = RX1TEMP2.data[7],
              .length = 8U
            	        };

can_message_t TX2stefen_SOC = {
             	  .cs = 0U,
             	  .id = TX2stefen_SOC_ID6,
             	  /* Start with a known, zeroed payload. The original code used
             	     TX2stefen_SOC.data[...] in its own initializer which is
             	     undefined behavior (reading uninitialized memory). That can
             	     prevent the correct CAN payload or cause the compiler to
             	     optimize unexpectedly. We initialize the whole data array to
             	     zeros here and explicitly set bytes later based on
             	     'stefen_SOC' and 'stefen_voltage'. */
             	  .data = {0,0,0,0,0,0,0,0},
             	  .length = 8U
             	        };


// mapping the Micronix SOC data to stefen SOC
TX2stefen_SOC.data[6] = (uint8_t)(stefen_SOC&0xFF); // TX2stefen_SOC.data[6] stores the least significant byte (LSB) of stefen_SOC.
                                                    //This is done using (stefen_SOC & 0xFF) to isolate the LSB.
TX2stefen_SOC.data[7] = (uint8_t)((stefen_SOC>>8)&0xFF);

// mapping the micronix voltage data to stefen voltage
TX2stefen_SOC.data[0] = (uint8_t)(stefen_voltage&0xFF);
TX2stefen_SOC.data[1] = (uint8_t)((stefen_voltage>>8)&0xFF);


// Switch Case Conditions for Tx the Std BMS Data over CAN PAL 2
switch (RX1VOLTAGE.id)
		{
		case 0x100: //CAN

			CAN_Send (INST_CAN_PAL2,TX2VOLTAGE_MAILBOX_1, &TX2VOLTAGE);
			arrayindex++;

			//if (CAN_Send (INST_CAN_PAL2,TX2VOLTAGE_MAILBOX_1, &TX2VOLTAGE) == STATUS_SUCCESS)
			//{
			//LPUART1_transmit_string("\r\n  0x310 Transmitted \r\n");
			//}

			break;

		default:
			break;

		}


switch (RX1SOC.id)
		{
		case 0x101: //CAN

			CAN_Send (INST_CAN_PAL2,TX2SOC_MAILBOX_2, &TX2SOC);
			arrayindex1++;
			{
				/* Capture the return code so we can debug why this send may fail.
				   If it fails, send a short debug packet over the RFID LPUART so
				   you can monitor the failure code in your terminal/analyzer. */
				int can_ret = CAN_Send (INST_CAN_PAL2,TX2stefen_SOC_MAILBOX_6, &TX2stefen_SOC);
				if (can_ret == STATUS_SUCCESS)
				{
					arrayindex3++;
				}
				else
				{
					/* Prepare a tiny debug payload: 'C','6', status_low, status_high, '\r','\n' */
					uint8_t dbg[6];
					dbg[0] = 'C';
					dbg[1] = '6';
					dbg[2] = (uint8_t)(can_ret & 0xFF);
					dbg[3] = (uint8_t)((can_ret >> 8) & 0xFF);
					dbg[4] = '\r';
					dbg[5] = '\n';
					/* Use the same LPUART used for RFID (may interleave with RFID traffic) */
					LPUART_DRV_SendDataBlocking(INST_LPUART_RFID, dbg, sizeof(dbg), Timeout);
				}
			}


			break;

		default:
			break;

		}

switch (RX1STATUS.id)
		{
		case 0x102: //CAN

			CAN_Send (INST_CAN_PAL2,TX2STATUS_MAILBOX_3, &TX2STATUS);
			arrayindex1++;

			break;

		default:
			break;

		}

switch (RX1TEMP1.id)
		{
		case 0x105: //CAN

			CAN_Send (INST_CAN_PAL2,TX2TEMP1_MAILBOX_4, &TX2TEMP1);
			arrayindex1++;

			break;

		default:
			break;

		}

switch (RX1TEMP2.id)
		{
		case 0x106: //CAN

			CAN_Send (INST_CAN_PAL2,TX2TEMP2_MAILBOX_5, &TX2TEMP2);
			arrayindex1++;

			break;

		default:
			break;

		}


}

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
