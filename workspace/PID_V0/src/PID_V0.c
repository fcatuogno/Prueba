/*
===============================================================================
 Name        : PID_V0.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
//#define NO_BOARD_LIB

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
int Kp, Kd, Ki = 0;
int set;
int sampletime;
int muestra;

const int OUTPUT_MAX;
const int OUTPUT_MIN;

void vTask_PID(){
	static int input,
			last_input=0,
			output, error;

	static int i_term = 0, d_term;

	input = muestra;
	error = input - set;

	//Calculo termino integral - se verifica saturacion (Anti Windup)
	i_term += Ki*error;
	if(i_term>OUTPUT_MAX)
		i_term=OUTPUT_MAX;
	else if(i_term<OUTPUT_MIN)
		i_term=OUTPUT_MIN;

	//Termino derivativo se calcula solo con la entrada para evitar kicks al cambiar setpoint (En nuestro caso es trivial)
	d_term= input-last_input;

	//Calculo salida - se verifica saturacion (Anti Windup)
	output=Kp*error + i_term - Kd*d_term; //derivada neg.
	if(output>OUTPUT_MAX)
		output=OUTPUT_MAX;
	else if(output<OUTPUT_MIN)
		output=OUTPUT_MIN;

	last_input = input;

}
int main(void) {


#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#endif

    while(1) {
    	sampletime = 1; //Debe configurarse el ADC sample time

    	Kd/=sampletime;
    	Ki*=sampletime;

    	vTask_PID();
    }
    return 0 ;
}


