#include "MKL46Z4.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd.h"

TaskHandle_t tskDis = NULL;
TaskHandle_t tskSum = NULL;
TaskHandle_t tskRec = NULL;
int status = 0;
int rep = 0;
int cont = 0;
int status_btn = 0;
double points = 0.0;
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
	if(pressed_switch == (0x1000)) {
		status_btn = 1;
		led_red_on();
	} else {
		status_btn = 0;
		led_red_off();
	}
}

void taskSumSwitch(void *pvParameters)
{
    int sum = 0;
    myQ = xQueueCreate(5, sizeof(sum));
    sum = 1;
    xQueueSend(myQ, (void*)sum, (TickType_t) 0);
    vTaskDelay(1000/portTICK_RATE_MS);
    led_green_on();
    for (;;) {
	/*if (status_btn == 1) {	
		sum = 5;
		xQueueSend(myQ, (void*)sum, (TickType_t) 0);
		vTaskDelay(100/portTICK_RATE_MS);
        } else {
		sum = 1;
		xQueueSend(myQ, (void*)sum, (TickType_t) 0);
		vTaskDelay(1000/portTICK_RATE_MS);
	}*/
    }
}

void taskReceiveP(void *pvParameters)
{
    int sum = 0;
    for (;;) {
	if (myQ != 0) {
		if (xQueueReceive(myQ, (void*)sum, (TickType_t) 100))
			points = points+(int)sum;
		
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

	xTaskCreate(taskDisplay, "TaskDisplay", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskDis);
	xTaskCreate(taskReceiveP, "TaskReceiveP", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskRec);
	xTaskCreate(taskSumSwitch, "TaskSumSwitch", 
		configMINIMAL_STACK_SIZE, (void *)NULL, 1, &tskSum);
	/* start the scheduler */
	vTaskStartScheduler();

	/* should never reach here! */
	for (;;);

	return 0;
}

