#include "stm32f10x.h"

#define CAN_MODE    		    (0x03U) 		// silent loop-back
#define CAN_PRESCALER		    (0U)            // Baud-rate prescaler
#define CAN_SJW                 (0x01U)         // Resynchronization jump width
#define CAN_BS1                 (0x04U)         // Time segment 1
#define CAN_BS2                 (0x02U)         // Time segment 2

#define CAN_WAIT_TIME_OUT       (65500U)

static uint8_t My_CAN_Init(void)
{
    uint8_t status = 0U;
    
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    
    CAN1->MCR &= ~((uint32_t)CAN_MCR_RESET << 1U); 
    
    // Set CANTX in ouput push-pull 
    GPIOA->CRH |= GPIO_CRH_MODE12_0;
    // Set CANRX in input mode
    GPIOA->CRH &= ~GPIO_CRH_MODE11;
    // Enable altrenative function Rx in floating input
    GPIOA->CRH |= (GPIO_CRH_CNF11_0);
    // Enable alt func TX in alternative push-pull
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
    GPIOA->CRH &= ~(GPIO_CRH_CNF12_0);
    
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    
    /* Exit from sleep mode */
    CAN1->MCR &= (~(uint32_t)CAN_MCR_SLEEP);

    /* Request initialisation */
    CAN1->MCR |= CAN_MCR_INRQ ;

    uint16_t wait_ack = 0U;

    /* Wait the acknowledge */
    while (((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != CAN_WAIT_TIME_OUT))
    {
        wait_ack++;
    }
	
	/* Check acknowledge */
    if ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) 
    {
        /* Set the time triggered communication mode */
            // Disable. Diasble of time trig
        CAN1->MCR &= ~(uint32_t)CAN_MCR_TTCM;

        /* Set the automatic bus-off management */
            // Disable. After 255 TEC set in BUS-off. And leave of software.
        CAN1->MCR &= ~(uint32_t)CAN_MCR_ABOM;

        /* Set the automatic wake-up mode */
            // Disable. Exit from SLEEP mode set in the software
        CAN1->MCR &= ~(uint32_t)CAN_MCR_AWUM;

        /* Set the no automatic retransmission */
            // Disable. Autimatin retransimition is ON
        CAN1->MCR &= ~(uint32_t)CAN_MCR_NART;

        /* Set the receive FIFO locked mode */
        // Disable. Next mes have overwrite latest mes
        CAN1->MCR &= ~(uint32_t)CAN_MCR_RFLM;

        /* Set the transmit FIFO priority */
        // Priority on the ID message
        CAN1->MCR &= ~(uint32_t)CAN_MCR_TXFP;
            
        // : CAN working during debug
        CAN1->MCR &= ~((uint32_t)0x10000);

        /* Set the bit timing register */
        CAN1->BTR = CAN_MODE << 30 | CAN_PRESCALER | CAN_BS1 << 16 | CAN_BS2 << 20 | CAN_SJW << 24; 

        /* Request leave initialisation */
        CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;

        /* Wait the acknowledge */
        wait_ack = 0U;

        while (((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != CAN_WAIT_TIME_OUT))
        {
            wait_ack++;
        }

        /* ...and check acknowledged */
        if ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
        {
            status = 0U;
        }
        else
        {
            status = 1U;
        }
    }

    return status;
}



static void My_CAN_FilterInit(void)
{
    /* Initialisation mode for the filter */
    CAN1->FMR |= CAN_FMR_FINIT;
    
    // All filter Scale set in 32 bit
    CAN1->FS1R = 0xFFFFFFFF;
    
    // Set all filters for CAN1
    CAN1->FMR |= 0x3F << 8U;
    
    /* 32-bit identifier or First 32-bit identifier */
    CAN1->sFilterRegister[0].FR1 = 
    ((0x00000000) << 16) | (0x00000000);
    /* 32-bit mask or Second 32-bit identifier */
    CAN1->sFilterRegister[0].FR2 = 
    ((0x00000000) << 16) | (0x00000000);
    
    // Set in first filter in FIFO0 assigned
    CAN1->FFA1R &= ~((uint32_t)CAN_FFA1R_FFA0);
    
    // Activation fist filter
    CAN1->FA1R |= CAN_FA1R_FACT0;
    
    CAN1->FMR &= (uint32_t) ~CAN_FMR_FINIT;   
}



static void My_CAN_Transmit(uint8_t* mes)
{
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    {
       
        // Mailbox reset transmit request 
        CAN1->sTxMailBox[0U].TIR &= (uint32_t)~0x01;
      
        // set id ant rtr bit
        uint16_t StdId = 0U;
        CAN1->sTxMailBox[0U].TIR = (uint32_t)((StdId << 21) | 0x00);

        /* Set up the DLC */
        uint8_t dlc = (uint8_t)0x00000001;
        CAN1->sTxMailBox[0U].TDTR &= (uint32_t)0xFFFFFFF0;
        CAN1->sTxMailBox[0U].TDTR |= dlc;

        /* Set up the data field */
        CAN1->sTxMailBox[0U].TDLR = ((uint32_t) mes[0]);

        // Mailbox Set transmit request
        CAN1->sTxMailBox[0U].TIR |= (uint32_t)0x01;
    }
}



static void blink(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH |= GPIO_CRH_MODE13;
	GPIOC->CRH &= ~GPIO_CRH_CNF13;
	long counter = 0;
	while (1)
	{
		GPIOC->BSRR ^= GPIO_BSRR_BS13;
		while (counter <= 2000000)
		{
			counter++;
		}
		counter = 0;
		GPIOC->BSRR ^= GPIO_BSRR_BR13;
		while (counter <= 2000000)
		{
			counter++;
		}
		counter = 0;
	}
}

int main()
{
	uint8_t status = 0U; 
    status = My_CAN_Init();
    if (status)
    {
        My_CAN_FilterInit();
    }
       
    // Test message for transsmit
    uint8_t message[1U] = {25};
    
    // Transmitt message
    My_CAN_Transmit(message);
    
    // Check transmit
    while(1)
    {
        if(CAN1->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
        {
            break;
        }
    }
    
    // Receive
    uint8_t Data[1] = {0};
    
    /* Get the Id */
    //uint8_t ide = (uint8_t)0x04 & CAN1->sFIFOMailBox[0U].RIR;
    uint32_t StdId = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0U].RIR >> 21);
    
    uint8_t RTR = (uint8_t)0x02 & CAN1->sFIFOMailBox[0U].RIR;
    
    /* Get the DLC */
    uint8_t DLC = (uint8_t)0x0F & CAN1->sFIFOMailBox[0U].RDTR;
    /* Get the FMI */
    uint8_t FMI = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0U].RDTR >> 8);
    /* Get the data field */
    Data[0U] = (uint8_t)0xFF & CAN1->sFIFOMailBox[0U].RDLR;
    /* Release the FIFO */
    /* Release FIFO0 */
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    if (Data[0] == 25)
    {
        blink();
    }
}
