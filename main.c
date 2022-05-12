#include "MKL46Z4.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd.h"

TaskHandle_t tskDis = NULL;
TaskHandle_t tskSum = NULL;
TaskHandle_t tskRec = NULL;
TaskHandle_t tskCnt = NULL;
TaskHandle_t tskMul = NULL;
int status = 0;
int rep = 0;
int cont = 0;
int status_btn = 0;
int status_btn2 = 0;
double points = 0.0;
double mult = 1.0;
QueueHandle_t myQ;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

// Switches
void bt1_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 //SW1_POS y SW2 son los puertos de los botones.
 PORTC->PCR[3] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[3] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[3] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void bt2_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 PORTC->PCR[12] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[12] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[12] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void led_green_init()
{
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[5] = PORT_PCR_MUX(1);
	GPIOD->PDDR |= (1 << 5);
	GPIOD->PSOR = (1 << 5);
}

void led_green_toggle()
{
	GPIOD->PTOR = (1 << 5);
}

void led_green_on(void)
{
 GPIOD->PCOR |= (1 << 5);
}

void led_green_off(void) {
 GPIOD->PSOR |= (1 << 5);
}

void led_red_init()
{
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[29] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << 29);
	GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
	GPIOE->PTOR = (1 << 29);
}

void led_red_on(void)
{
 GPIOD->PCOR |= (1 << 29);
}

void led_red_off(void) {
 GPIOD->PSOR |= (1 << 29);
}

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
	if(pressed_switch == (0x8)) {
		status_btn2 = 1;
	} else {
		status_btn2 = 0;
	}
	if(pressed_switch == (0x1000)) {
		status_btn = 1;
	} else {
		status_btn = 0;
	}
}

void taskSumSwitch(void *pvParameters)
{
    int sum = 0;
    sum = 1;
    for (;;) {
	if (status_btn == 1) {	
		sum = 5*mult;
		if (myQ != 0)
    			xQueueSend(myQ, (void*)&sum, (TickType_t) 0);
		status_btn = 0;
        }
    }
}

void taskReceiveP(void *pvParameters)
{
    int sum = 0;
    for (;;) {
	if (myQ != NULL) {
		if (xQueueReceive(myQ, &sum, (TickType_t) 5))
			points = points+(int)sum;
	}
    }
}

void taskCountSec(void *pvParameters)
{
    TickType_t lastUnblock;
    int sum = 1;
    lastUnblock = xTaskGetTickCount();
    for (;;) {
	if (myQ != NULL) {
		xQueueSend(myQ, (void*)&sum, (TickType_t) 0);
		vTaskDelayUntil(&lastUnblock, 1000*configTICK_RATE_HZ/1000);
	}
    }
}

void taskMultiplier(void *pvParameters)
{
    for (;;) {
	if (points >= 50*mult)
		led_green_on();
	else led_green_off();
	if (status_btn2 == 1) {	
		if (points >= 50*mult) {
			points = points-(50*mult);
			mult = mult+0.2;
		}
		status_btn2 = 0;
        }
    }
}

void taskDisplay(void *pvParameters)
{
    for (;;) {
	lcd_display_dec(points);
    }
}

int main(void)
{
	irclk_ini();
	led_green_init();
	led_red_init();
	lcd_ini();
	bt1_init();
	bt2_init();

	myQ = xQueueCreate(1, sizeof(int));

	xTaskCreate(taskDisplay, "TaskDisplay", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskDis);
	xTaskCreate(taskReceiveP, "TaskReceiveP", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskRec);
	xTaskCreate(taskCountSec, "TaskCountSec", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskCnt);
	xTaskCreate(taskSumSwitch, "TaskSumSwitch", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskSum);
	xTaskCreate(taskMultiplier, "TaskMultiplier", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskMul);
	/* start the scheduler */
	vTaskStartScheduler();

	/* should never reach here! */
	for (;;);

	return 0;
}

