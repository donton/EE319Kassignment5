// ***** 0. Documentation Section *****
// TrafficLight.c for Lab 5
// Runs on LM4F120/TM4C123
// Implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// Feb 24, 2015
// designed by Don Ton and Domino Weir
// east/west red light connected to PE5
// east/west yellow light connected to PE4
// east/west green light connected to PE3
// north/south facing red light connected to PE2
// north/south facing yellow light connected to PE1
// north/south facing green light connected to PEO
// pedestrian detector connected to PA4 (1=pedestrian present)
// north/south car detector connected to PA3 (1=car present)
// east/west car detector connected to PA2 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "SysTick.h"

#define GOS 0
#define WTS 1
#define GOW 2
#define WTW 3
#define PRQS 4
#define PRQW 5
#define GOP 6
#define FLP0 7
#define FLP1 8
#define FLP2 9
#define FLP3 10
#define FLP4 11


// ***** 2. Global Declarations Section *****
struct state{
	uint8_t outB;
	uint8_t outMC;
	uint16_t time;
	uint8_t next[8];
};//end struct
typedef const struct state STyp;
uint8_t inp;
uint8_t CS;
volatile uint32_t delay;
STyp FSM[12] = 	{
	{0x21,0x02,200,{GOS,WTS,GOS,WTS,PRQS,WTS,PRQS,WTS}},
	{0x22,0x02,200,{GOW,GOW,GOW,GOW,GOP,GOP,GOP,GOW}},
	{0x0C,0x02,200,{GOW,GOW,WTW,WTW,PRQW,PRQW,WTW,PRQW}},
	{0x14,0x02,200,{GOS,GOS,GOS,GOS,GOP,GOP,GOP,GOP}},
	{0x22,0x02,200,{GOP,GOP,GOP,GOP,GOP,GOP,GOP,GOP}},
	{0x14,0x02,200,{GOP,GOP,GOP,GOP,GOP,GOP,GOP,GOP}},
	{0x24,0x08,200,{FLP0,FLP0,FLP0,FLP0,FLP0,FLP0,FLP0,FLP0}},
	{0x24,0x02,100,{FLP1,FLP1,FLP1,FLP1,FLP1,FLP1,FLP1,FLP1}},
	{0x24,0x00,100,{FLP2,FLP2,FLP2,FLP2,FLP2,FLP2,FLP2,FLP2}},
	{0x24,0x02,100,{FLP3,FLP3,FLP3,FLP3,FLP3,FLP3,FLP3,FLP3}},
	{0x24,0x00,100,{FLP4,FLP4,FLP4,FLP4,FLP4,FLP4,FLP4,FLP4}},
	{0x24,0x02,100,{GOS,GOW,GOS,GOW,GOS,GOW,GOS,GOS	}}
};

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
void PortF_Init (void){
		SYSCTL_RCGC2_R |= 0x20;  								// 1) enable clock to F
		delay = SYSCTL_RCGC2_R;  								// 2) no need to unlock
		//SYSCTL_RCGCGPIO_R |= 0x20;
		//while ((SYSCTL_RCGCGPIO_R&0x20) == 0){};
		GPIO_PORTF_LOCK_R |= 0x4C4F434B;
		GPIO_PORTF_CR_R 	|= 0x0A;							// allow changes to PF1 & PF3
		GPIO_PORTF_DIR_R	|= 0x0A;							// PF1 & PF3 are outputs
		GPIO_PORTF_DEN_R	|= 0x0A;							// digital enable
}
		
void PortA_Init (void){
		SYSCTL_RCGC2_R |= 0x01;  								// 1) enable clock to A
		delay = SYSCTL_RCGC2_R;  								// 2) no need to unlock
		//SYSCTL_RCGCGPIO_R |= 0x01;
		//while ((SYSCTL_RCGCGPIO_R&0x01) == 0){};
		GPIO_PORTA_LOCK_R |= 0x4C4F434B;
		GPIO_PORTA_CR_R 	|= 0x1C;							// allow changes to PF1 & PF3
		GPIO_PORTA_DIR_R	|= 0x00;							// PF1 & PF3 are outputs
		GPIO_PORTA_DEN_R	|= 0x1C;							// digital enable
}
			
void PortE_Init (void){
		SYSCTL_RCGC2_R |= 0x10;  								// 1) enable clock to E
		delay = SYSCTL_RCGC2_R;  								// 2) no need to unlock
		//SYSCTL_RCGCGPIO_R |= 0x10;
		//while ((SYSCTL_RCGCGPIO_R&0x10) == 0){};
		GPIO_PORTE_LOCK_R |= 0x4C4F434B;
		GPIO_PORTE_CR_R 	|= 0x3F;							// allow changes to PE0-PE5
		GPIO_PORTE_DIR_R	|= 0x3F;							// PE0-PE5 are outputs
		GPIO_PORTE_DEN_R	|= 0x3F;							// digital enable
}

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); 	// activate grader and set system clock to 80 MHz
	PortA_Init(); 																//init stuff
	PortE_Init();
	PortF_Init();
	SysTick_Init();			
  CS = 0;																				//set current state to initial state
  EnableInterrupts(); 													//enable interrupts now that the critical section is finished
  while(1){
		GPIO_PORTE_DATA_R = FSM[CS].outB;						//give PortE board output- traffic lights
		GPIO_PORTF_DATA_R = FSM[CS].outMC;					//give PortF mc output- walk light
		SysTick_Wait10ms(FSM[CS].time);
		inp = GPIO_PORTA_DATA_R & 0x1E;							//read input
		inp = inp >> 2;															//adjust for the fact that imput is PA4-PA2
		CS = FSM[CS].next[inp];											
  }//endwhile
}//endmain
