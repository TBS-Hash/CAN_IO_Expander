# CAN_IO_Expander

This software is written on a STM32F072C8Tx. It contains 3 branches. 

The Main Branch is for the standard program. It is has no EXTI interrupts and is definitely working. Use this if you don't want the EXTI options. There is no need to compile.

## Compilation

To compile the program use [Keil](https://www.keil.com/download/product/), or alternatively a Keil extension on any preferred IDE. 

To upload and debug the program use [Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger/). The board must be on to be able to do this.

## Usage

There are multiple commands that can be used to configure the desired outcome. These are the first 

### Main

Set Pin direction -- AA  
Set Pin output -- AB  
Read Pin -- AC  
Set alternate pin functions -- AD  
Set timer frequency -- BA  
Set duty cycle -- BB   
Set timer on time -- BC  
Read UART buffer -- CA  
Change CAN ID -- DA  
Disable USB -- E0  
Enable USB -- E1  

### EXTI

EXTI has all the previous settings, plus a few specialized ones

EXTI Enable -- F0  
EXTI Timer -- FA  
EXTI Channel -- FB   
EXTI Duty Cycle -- FC  
EXTI Sensor Function -- FD
EXTI Sensor Pins -- FE
EXTI All Settings -- FF



## Contributors

Andy Tang

Jakob Menzinger
