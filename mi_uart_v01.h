#ifndef _MI_UART_V01_H_
    #define _MI_UART_V01_H_


// version 0.1, con interrupts y bufers
// hay una inicializacion para cada uart
// TODO: hacer una version para uart0 y otra para uart1, con diferentes buffers y coso y cuestion
// TODO: deberia terminar los buffers en \0? O no...?
// TODO: hacer prueba de fuego con transmision full duplez en ambos cores
// TODO: Pines reubicables
// TODO: Optimizacion general

/*
     usuario                     Hardware       usuario                       hardware
                   buff tx                                      buff rx
                 |_________|                                  |_________|
puntero_putc  -> |         | -> puntero_pop   puntero_getc <- |         | <- puntero_push
                 |---------|                                  |---------|
*/


#include "hardware/uart.h"
#include "hardware/resets.h"


#define UART_TX_PIN     12  //pin tx alternativo del uart0
#define UART_RX_PIN     13  //pin rx alternativo del uart0
#define UART_ID         uart0
#define BAUD_RATE       9600    //115200
#define DATA_BITS       8
#define STOP_BITS       1
#define PARITY          UART_PARITY_NONE

#ifndef TAMANIO_BUFFER0
    #define TAMANIO_BUFFER0  64
#endif
#ifndef TAMANIO_BUFFER1
    #define TAMANIO_BUFFER1  64
#endif

volatile uint8_t recibido;

volatile uint8_t buffer0_tx[TAMANIO_BUFFER0]; //buffer de transmision de la uart0
volatile uint8_t buffer0_rx[TAMANIO_BUFFER0]; //buffer de recepcion de la uart0
volatile uint8_t puntero0_putc;      //puntero que va a meter los chars para ser transmitidos
volatile uint8_t puntero0_pop;       //puntero que apunta al proximo caracter a ser enviado a la uart
volatile uint8_t puntero0_getc;      //puntero que va a sacar los chars recibidos
volatile uint8_t puntero0_push;      //puntero que va a poner los chars recien llegados de la uart

volatile uint8_t buffer1_tx[TAMANIO_BUFFER1]; //buffer de transmision de la uart0
volatile uint8_t buffer1_rx[TAMANIO_BUFFER1]; //buffer de recepcion de la uart0
volatile uint8_t puntero1_putc;      //puntero que va a meter los chars para ser transmitidos
volatile uint8_t puntero1_pop;       //puntero que apunta al proximo caracter a ser enviado a la uart
volatile uint8_t puntero1_getc;      //puntero que va a sacar los chars recibidos
volatile uint8_t puntero1_push;      //puntero que va a poner los chars recien llegados de la uart



 static inline void uart_reset(uart_inst_t *uart) 
{
invalid_params_if(UART, uart != uart0 && uart != uart1);
reset_block(uart_get_index(uart) ? RESETS_RESET_UART1_BITS : RESETS_RESET_UART0_BITS);
}

static inline void uart_unreset(uart_inst_t *uart) 
{
invalid_params_if(UART, uart != uart0 && uart != uart1);
unreset_block_wait(uart_get_index(uart) ? RESETS_RESET_UART1_BITS :RESETS_RESET_UART0_BITS);
}

//--------- Interrupt handler de la uart0 -----------
void uart0_interrupt() 
{
uart_inst_t *uart=uart0;
if(((uart_get_hw(uart)->ris) & UART_UARTRIS_RXRIS_BITS) !=0)  //si es int de RX
    {
    if(((uart_get_hw(uart)->fr) & UART_UARTFR_RXFF_BITS)!=0)    //si el buffer rx esta full
        {
        uart_get_hw(uart)->icr |= (UART_UARTICR_RXIC_BITS); //UARTICR & RXIC //clear RX interrupt
        buffer0_rx[puntero0_push] = (uart_get_hw(uart)->dr);
        puntero0_push++;
        if(puntero0_push > (TAMANIO_BUFFER0-1)) puntero0_push=0;
        buffer0_rx[puntero0_push]='\0'; //marco el final del buffer
        }
    }
if(((uart_get_hw(uart)->ris) & UART_UARTRIS_TXRIS_BITS) !=0)      //si es int de TX
    {
    uart_get_hw(uart)->icr |= (UART_UARTICR_TXIC_BITS); //UARTICR & TXIC //clear TX interrupt
    if(puntero0_pop==puntero0_putc) uart_get_hw(uart)->imsc &= ~(UART_UARTIMSC_TXIM_BITS);//disable TXE ints
    else if(((uart_get_hw(uart)->fr) & UART_UARTFR_TXFE_BITS)!=0) //si es una int de buffer empty
        {
        uart_get_hw(uart)->dr = buffer0_tx[puntero0_pop];
        puntero0_pop++;
        if((puntero0_pop) > (TAMANIO_BUFFER0-1)) puntero0_pop=0;
        }
    }
}

//--------- Interrupt handler de la uart1 ----------------
void uart1_interrupt() 
{
uart_inst_t *uart=uart1;
if(((uart_get_hw(uart)->ris) & UART_UARTRIS_RXRIS_BITS) !=0)  //si es int de RX
    {
    if(((uart_get_hw(uart)->fr) & UART_UARTFR_RXFF_BITS)!=0)    //si el buffer rx esta full
        {
        uart_get_hw(uart)->icr |= (UART_UARTICR_RXIC_BITS); //UARTICR & RXIC //clear RX interrupt
        buffer1_rx[puntero1_push] = (uart_get_hw(uart)->dr);
        puntero1_push++;
        if(puntero1_push > (TAMANIO_BUFFER1-1)) puntero1_push=0;
        buffer1_rx[puntero1_push]='\0'; //marco el final del buffer
        }
    }
if(((uart_get_hw(uart)->ris) & UART_UARTRIS_TXRIS_BITS) !=0)      //si es int de TX
    {
    uart_get_hw(uart)->icr |= (UART_UARTICR_TXIC_BITS); //UARTICR & TXIC //clear TX interrupt
    if(puntero1_pop==puntero1_putc) uart_get_hw(uart)->imsc &= ~(UART_UARTIMSC_TXIM_BITS);//disable TXE ints
    else if(((uart_get_hw(uart)->fr) & UART_UARTFR_TXFE_BITS)!=0) //si es una int de buffer empty
        {
        uart_get_hw(uart)->dr = buffer1_tx[puntero1_pop];
        puntero1_pop++;
        if((puntero1_pop) > (TAMANIO_BUFFER1-1)) puntero1_pop=0;
        }
    }
}

//----- Get char de acuerdo a lo que haiga en buffers -------
char get_char(uart_inst_t *uart)
{
char caracter_temporal;

if(uart==uart0)
    {
    if (puntero0_getc!=puntero0_push)
        {
        caracter_temporal=buffer0_rx[puntero0_getc];
        puntero0_getc++;
        if(puntero0_getc > (TAMANIO_BUFFER0-1)) puntero0_getc=0;    //hago rollover, puntero circular
        return(caracter_temporal);
        }
    else return(-1);
    }
else if(uart==uart1)
    {
    if (puntero1_getc!=puntero1_push)
        {
        caracter_temporal=buffer1_rx[puntero1_getc];
        if(++puntero1_getc > (TAMANIO_BUFFER1-1)) puntero1_getc=0;    //hago rollover, puntero circular
        return(caracter_temporal);
        }
    else return(-1);
    }

}


// Put char en el registro de transmision o en el buffer, si ya esta transmitiendo
// TODO: chequear si esta habilitada UART?
void put_char(uart_inst_t *uart, char caracter)
{
if(uart == uart0)
    {
                            //cual uso? UART_UARTFR_TXFE_BITS o UART_UARTFR_BUSY_BITS
    if(((uart_get_hw(uart)->fr) & UART_UARTFR_TXFE_BITS) !=0) uart_get_hw(uart)->dr = caracter; //si no esta busy le mando al registro de una
    else
        {
        buffer0_tx[puntero0_putc]=caracter;
        puntero0_putc++;
        if(puntero0_putc > (TAMANIO_BUFFER0-1)) puntero0_putc=0;
        uart_get_hw(uart)->icr &= ~(UART_UARTICR_TXIC_BITS);// borro flag ints
        uart_get_hw(uart)->imsc |= (UART_UARTIMSC_TXIM_BITS);//enable TXE ints
        }
    }
else if (uart==uart1)
    {
                            //cual uso? UART_UARTFR_TXFE_BITS o UART_UARTFR_BUSY_BITS
    if(((uart_get_hw(uart)->fr) & UART_UARTFR_TXFE_BITS) !=0) uart_get_hw(uart)->dr = caracter; //si no esta busy le mando al registro de una
    else
        {
        buffer1_tx[puntero1_putc]=caracter;
        puntero1_putc++;
        if(puntero1_putc > (TAMANIO_BUFFER1-1)) puntero1_putc=0;
        uart_get_hw(uart)->icr &= ~(UART_UARTICR_TXIC_BITS);// borro flag ints
        uart_get_hw(uart)->imsc |= (UART_UARTIMSC_TXIM_BITS);//enable TXE ints
        }
    }
}


//----------------- Inicializa uarts -----------------
//TODO: que se inicialicen tambien los pines
void inicializa_uart(uart_inst_t *uart, uint baudios)
{
if(uart==uart0)
    {
    puntero0_putc =0;   //pongo en cero los punteros
    puntero0_pop  =0; 
    puntero0_getc =0;
    puntero0_push =0;
    uart_reset(uart);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); // configuro los  GPIO para funcion de UART. Pumba.
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_unreset(uart);
        
    int __unused actual = uart_set_baudrate(uart0, baudios);    //seteo baudrate

    uart_set_hw_flow(uart0, false, false);    //deshabilito RTS y CTS

    uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY); //formato

    uart_set_fifo_enabled(uart0, false); //deshabilito FIFO

    uart_get_hw(uart)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    
    irq_set_exclusive_handler(UART0_IRQ, uart0_interrupt); //interrupt handler
    irq_set_enabled(UART0_IRQ, true);                    //habilito las ints

    uart_set_irq_enables(uart0, true, false); //habilito ints de rx y tx
    }
else if(uart==uart1)
    {
    puntero1_putc =0;   //pongo en cero los punteros
    puntero1_pop  =0; 
    puntero1_getc =0;
    puntero1_push =0;
    uart_reset(uart);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); // configuro los  GPIO para funcion de UART. Pumba.
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_unreset(uart);
        
    int __unused actual = uart_set_baudrate(uart, baudios);    //seteo baudrate

    uart_set_hw_flow(uart, false, false);    //deshabilito RTS y CTS

    uart_set_format(uart, DATA_BITS, STOP_BITS, PARITY); //formato

    uart_set_fifo_enabled(uart, false); //deshabilito FIFO

    uart_get_hw(uart)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    
    irq_set_exclusive_handler(UART1_IRQ, uart1_interrupt); //interrupt handler
    irq_set_enabled(UART1_IRQ, true);                    //habilito las ints

    uart_set_irq_enables(uart, true, false); //habilito ints de rx y tx
    }    
}


//----------- envio una string por el puerto serial --------------
void put_string(uart_inst_t *uart, const char *str)
{
while (*str) put_char(uart, *str++);    
}














#endif
