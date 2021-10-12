///////////////////////////////////////////////////////////////////////////////
/// \file Terminal.c
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////
//#include "Nodate.h"
#include "universal.h"
#include "mcu/led.h"
#include "MCU/usart3.h"
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
    SerialPort3.SendString(&SystemMessageString[0]);
	// Send new line feed and prompt
	SerialPort3.SendString("\n> ");
	//printf("%s\n ", SystemMessageString);
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Init the terminal program
///////////////////////////////////////////////////////////////////////////////
void Terminal_Init(void)
{
    Led_Init();
    Tick_init();
    SerialPort3.Open(115200);

    NumberOfByteReceived = 0;
    DisplaySystemInformation();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief process the buffer data and extract the commands
///
/// 'source' is a pointer to a list of uint8_t, ostensibly a list of bytes representing ascii characters.
/// 'length' is the lenght of said source character list
/// 'destination' is a pointer to a list of parameters parsed from the source list
/// A parameter is of type ParamStructureType, containing either a string, or a structure that includes both a type field and a
/// data converter field.  The type field is just a single character field, 'S' for command and 'U' for integer.
/// The data converter field is an 8-byte union of data types, ranging from int8_t to long double.
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

	// TODO: should NumberOfParameter start at 1?  Is it an index or a count?  How would newline be handled?
	// looks like it is first used as an index and subseqently incremented, possibly to a count?
	
	destination->NumberOfParameter = 0; // reset
	CurrentParamStrPointer = &destination->List[destination->NumberOfParameter].StringArray[0];

	/* iterate through the source list. A blank space with advance to the next parameter.  Else
	 the source character is copied into the string array of the current location in the destination array.
	 In summary, the destination array is filled with white-space separated character strings from the terminal input line*/
	for( ; length ; length--)
	{

		if ( *source  == ' ')
		{
			// we found a "space" in the input source string, so we must have another value coming up
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
			// TODO: should number
			if ( ParameterCharacterLength < (PARAMETER_STRING_SIZE -1) )
			{

				*CurrentParamStrPointer = *source;
				CurrentParamStrPointer++;
				*CurrentParamStrPointer = 0;  // make sure the string is zero-terminated (will be overwritten if another character is added)
				ParameterCharacterLength++;
			}
		}

		source++;
	}
	
	// TODO: figure out what this is for!!!!!
	if (ParameterCharacterLength >= 2)
	{
		destination->NumberOfParameter++;
	}
	else if ( !destination->NumberOfParameter )
	{
		destination->NumberOfParameter = 0;
		return FALSE;
	}
	// much weirdness here.  We have never set List[0].Type.  However, List[0].Type occupies the same memory location as
	// List[0].StringArray[0].  However, should it be "!=" instead of "== !"?  May be pointer weirdness
	// anyway, appears to be verifying that the first parameter is a command paramenter
	else if ( 's' == !destination->List[0].Type && 'S' == !destination->List[0].Type)
	{
		destination->NumberOfParameter = 0;
		return ERROR;
	}



	// We have a valid number of parameters
	
	/* Here we appear to be making sure the parameter types are lower-case.  Note that the accepted parameter types are now
	 defined as S, U, F, and T (a change from the comment in the ParamStructureType definition */
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
/// Commands are as follows.
/// The switch statement looks at the value of the first parameter.  An integer value of 1 will control the LED.
/// If there is a second parameter of type 'u' (integer), the integer value of the second parameter is checked.
/// A non-zero value will result in Led_On.  A zero value will result in Led_Off.
///
/// If there is no second parameter or if the second parameter is not of type 'u' (integer), the LED is toggled.
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

	Result = SerialPort3.GetByte(&SerialTempData);

	if ( TRUE != Result )
	{
		return Result;
	}

	// echo the user command
	SerialPort3.SendByte(SerialTempData);

	if ('\r' == SerialTempData)
	{
		// Send new line feed and prompt
		SerialPort3.SendString("\n> ");

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









