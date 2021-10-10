///////////////////////////////////////////////////////////////////////////////
/// \file Terminal.c
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////
//#include "Nodate.h"
#include "universal.h"
#include "mcu/led.h"
#include "MCU/usart2.h"
#include "MCU/tick.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines our terminal buffer size which in turn set the longest command
///////////////////////////////////////////////////////////////////////////////
#define TERMINAL_BUFFER_SIZE 25

///////////////////////////////////////////////////////////////////////////////
/// \brief UART device used for communications
///////////////////////////////////////////////////////////////////////////////
//static USART_devices my_usart;

///////////////////////////////////////////////////////////////////////////////
/// \brief Keep track of the number of bytes we received from the computer
///////////////////////////////////////////////////////////////////////////////
static uint32_t NumberOfByteReceived;

///////////////////////////////////////////////////////////////////////////////
/// \brief The terminal data buffer
///////////////////////////////////////////////////////////////////////////////
static uint8_t Buffer[TERMINAL_BUFFER_SIZE];

///////////////////////////////////////////////////////////////////////////////
/// \brief define the number of characters we can hold on the string
///////////////////////////////////////////////////////////////////////////////
#define PARAMETER_STRING_SIZE 20

///////////////////////////////////////////////////////////////////////////////
/// \brief define the max number of parameters
///////////////////////////////////////////////////////////////////////////////
#define MAX_NUMBER_OF_PARAMETERS 10


///////////////////////////////////////////////////////////////////////////////
/// \brief this message has the system information data that we want to display
/// on the terminal
///////////////////////////////////////////////////////////////////////////////
static const char SystemMessageString[] = "-----------------------------------\r\n"
											"Firm : " FIRMWARE_VERSION "\r\n"
											"Hard : " HARDWARE_VERSION "\r\n"
											COMPILED_DATA_TIME "\r\n"
											"-----------------------------------\r\n";

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines the parameter data type
///////////////////////////////////////////////////////////////////////////////
typedef union {
	struct{
		uint8_t Type;			///< Parameter data type. S = command, U = integer
		DataConverter Value;	///< parameter data
	};
	uint8_t StringArray[PARAMETER_STRING_SIZE];	///< Data string format. Note the first character is the command
} ParamStructureType;

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines the list of parameter data structure
///////////////////////////////////////////////////////////////////////////////
typedef struct {

	uint32_t NumberOfParameter;		///< used to keep track of the number of parameter we haver
	ParamStructureType List[MAX_NUMBER_OF_PARAMETERS];	///< hold the parameter
} ListOfParameterStructureType;

///////////////////////////////////////////////////////////////////////////////
/// \brief holds our current list of parameter extracted by ProcessData
///
///	\sa ProcessData
///////////////////////////////////////////////////////////////////////////////
static ListOfParameterStructureType ParameterList;


///////////////////////////////////////////////////////////////////////////////
/// \brief Send the system information to the computer. This first clear the
///	computer terminal screen.
///////////////////////////////////////////////////////////////////////////////
static void DisplaySystemInformation(void)
{
	//SerialPort2.SendByte(0x0C); // clear terminal
	char ch = 0x0C;
	//USART::sendUart(my_usart, ch);  // clear termina
    SerialPort2.SendString(&SystemMessageString[0]);
	// Send new line feed and prompt
	SerialPort2.SendString("\n> ");
	//printf("%s\n ", SystemMessageString);
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Init the terminal program
///////////////////////////////////////////////////////////////////////////////
void Terminal_Init(void)
{
    Led_Init();
    Tick_init();
    SerialPort2.Open(115200);

    NumberOfByteReceived = 0;
    DisplaySystemInformation();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief process the buffer data and extract the commands
///
///	\return TRUE success
///
///	\todo update the document return state value. need to implement
///////////////////////////////////////////////////////////////////////////////
static int_fast8_t ProcessData(uint8_t *source, uint32_t length, ListOfParameterStructureType * destination)
{
	uint32_t ParameterCharacterLength = 0;
	uint32_t ParameterCounter = 0;
	uint8_t *CurrentParamStrPointer;

	// checking the pointer aren't null
	if ( !destination )
	{
		return ERROR;
	}
	if ( !source )
	{
		// don't do this unless the destination pointer has already been validated
		destination->NumberOfParameter = 0; // reset the number of parameters
		return ERROR;
	}

	destination->NumberOfParameter = 0; // reset
	CurrentParamStrPointer = &destination->List[destination->NumberOfParameter].StringArray[0];

	for( ; length ; length--)
	{

		if ( *source  == ' ')
		{
			destination->NumberOfParameter++;

			if (destination->NumberOfParameter < MAX_NUMBER_OF_PARAMETERS )
			{
				CurrentParamStrPointer = &destination->List[destination->NumberOfParameter].StringArray[0];
			}
			else
			{
				destination->NumberOfParameter = 0; // reset the number of parameters
				return FALSE;
			}

			ParameterCharacterLength = 0;

		}
		else
		{

			if ( ParameterCharacterLength < (PARAMETER_STRING_SIZE -1) )
			{

				*CurrentParamStrPointer = *source;
				CurrentParamStrPointer++;
				*CurrentParamStrPointer = 0;
				ParameterCharacterLength++;
			}
		}

		source++;
	}


	if (ParameterCharacterLength >= 2)
	{
		destination->NumberOfParameter++;
	}
	else if ( !destination->NumberOfParameter )
	{
		destination->NumberOfParameter = 0;
		return FALSE;
	} else if ( 's' == !destination->List[0].Type && 'S' == !destination->List[0].Type)
	{
		destination->NumberOfParameter = 0;
		return ERROR;
	}



	// We have a valid number of parameters
	for(ParameterCounter = 0; ParameterCounter < destination->NumberOfParameter ; ParameterCounter++)
	{

		switch (destination->List[ParameterCounter].Type)
		{
		case 'S':
			/// \fallthrough
		case 'U':
			/// \fallthrough
			destination->List[ParameterCounter].Type += 32; // convert to lowercase
		case 's':
			/// \fallthrough
		case 'u':
			destination->List[ParameterCounter].Value.i32_t[0] = atoi((char *) &destination->List[ParameterCounter].StringArray[1]);
			break;

		case 'F':
			/// \fallthrough
		case 'f':
			destination->List[ParameterCounter].Value.f32_t[0] = atof((char *) &destination->List[ParameterCounter].StringArray[1]);
			break;


		case 'T':
			/// \fallthrough
			destination->List[ParameterCounter].Type += 32; // convert to lowercase
		case 't':
			// don't need to do anything its already converted
			break;

		default:
			destination->NumberOfParameter = 0;
			return ERROR;
			break;

		}
	}





	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief run the terminal command
///
///	\return TRUE success
///
///	\todo update the document return state value. need to implement
///////////////////////////////////////////////////////////////////////////////
static int_fast8_t RunCommand(ListOfParameterStructureType *source)
{

	switch ( source->List[0].Value.i32_t[0] )
	{
		case 1: // control the LED

			if ( source->NumberOfParameter > 1 && source->List[1].Type == 'u')
			{
				if ( source->List[1].Value.i32_t[0] )
				{
					Led_On();
				}
				else
				{
					Led_Off();
				}

			}
			else
			{
				Led_Toggle();
			}
			break;

		default:
			// undefined command
			return FALSE;
			break;

	}

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief process the buffer data and extract the commands
///
///	\return TRUE success. FALSE no error
///
///	\todo update the document return state value. need to implement
///////////////////////////////////////////////////////////////////////////////
int_fast8_t Terminal_Process(void)
{
	uint8_t SerialTempData = 0; // hold the new byte from the serial fifo
	int_fast8_t Result = FALSE;

	Result = SerialPort2.GetByte(&SerialTempData);

	if ( TRUE != Result )
	{
		return Result;
	}

	// echo the user command
	SerialPort2.SendByte(SerialTempData);

	if ('\r' == SerialTempData)
	{
		// Send new line feed and prompt
		SerialPort2.SendString("\n> ");

		if (NumberOfByteReceived)
		{
			/// \todo call the process data
			if ( TRUE == ProcessData(&Buffer[0], NumberOfByteReceived, &ParameterList) )
			{
				if(  TRUE == RunCommand(&ParameterList) )
				{
					Result =  TRUE;
				}

			}
		}
		else
		{
			Result =  FALSE;
		}
	}
	else if ( (SerialTempData >= '0' && SerialTempData <= '9') ||
			(SerialTempData >= 'A' && SerialTempData <= 'Z') ||
			(SerialTempData >= 'a' && SerialTempData <= 'z') || ' ' == SerialTempData || '.' == SerialTempData)

	{
		if ( NumberOfByteReceived < TERMINAL_BUFFER_SIZE )
		{
			Buffer[NumberOfByteReceived] = SerialTempData;
			NumberOfByteReceived++;
			return FALSE;
		}

	}

	// reset buffer by reseting the counter
	NumberOfByteReceived = 0;
	return Result;


}









