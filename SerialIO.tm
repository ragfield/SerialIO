/*
 * SerialIO.tm
 *
 * Copyright 2004-2009 Rob Raguet-Schofield. All rights reserved.
 *
 */

/*
 * Implementation
 */

#include "SerialPort.h"
#include "SerialPort.h"
#include <stdio.h>

#if UNIX_MATHLINK
#include <errno.h>
#include <string.h>
#define SNPRINTF snprintf
#elif WINDOWS_MATHLINK
#define SNPRINTF _snprintf
#endif

static void message(const char* theSymbol, const char* theTag, const char* theParam1 = NULL, const char* theParam2 = NULL)
{
	MLPutFunction(stdlink, "EvaluatePacket", 1);
	int theParamCount = 0;
	if(theParam1 != NULL)
	{
		theParamCount++;
		if(theParam2 != NULL)
		{
			theParamCount++;
		}
	}
	
	MLPutFunction(stdlink, "Message", 1 + theParamCount);
		MLPutFunction(stdlink, "MessageName", 2);
			MLPutSymbol(stdlink, theSymbol);
			MLPutString(stdlink, theTag);
		
	if(theParam1 != NULL)
	{	
		MLPutString(stdlink, theParam1);
		if(theParam2 != NULL)
		{
			MLPutString(stdlink, theParam2);
		}
	}
	MLNextPacket(stdlink);
}

class Node
{
public:
	Node(SerialPort* thePort)
		: port(thePort), next(NULL), prev(NULL)
	{
	}
	
	SerialPort* port;
	Node* next;
	Node* prev;
};

static Node* FirstNode = NULL;
static Node* LastNode = NULL;

static Node* getPortNode(int thePort, bool messageOnFailure)
{
	Node* theNode = FirstNode;
	while(theNode != NULL)
	{
		if(thePort == theNode->port->getIdentifier())
			break;
		theNode = theNode->next;
	}
	
	if(theNode == NULL && messageOnFailure)
	{
		char buffer[256];
		SNPRINTF(buffer, 256, "%d", thePort);
		message("SerialPort", "inval", buffer);
	}
	
	return theNode;
}

static SerialPort* getPortObject(int thePort, bool messageOnFailure)
{
	Node* theNode = getPortNode(thePort, messageOnFailure);
	return (theNode != NULL ? theNode->port : NULL);
}

static int getPortNumber(SerialPort* thePort)
{
	return thePort->getIdentifier();
}

void serialOpen(kcharp_ct thePathName)
{
	SerialPort* thePort = new SerialPort(thePathName);
	if(!thePort->openPort())
	{
		delete thePort;
		message("SerialOpen", "failed", strerror(errno));
		MLPutSymbol(stdlink, "$Failed");
		return;
	}
	
	Node* theNode = new Node(thePort);
	if(LastNode != NULL)
	{
		LastNode->next = theNode;
		theNode->prev = LastNode;
		LastNode = theNode;
	}
	else
		FirstNode = LastNode = theNode;
	
	int thePortNumber = getPortNumber(thePort);
	MLPutInteger(stdlink, thePortNumber);
}

char* serialClose(int thePort)
{
	Node* theNode = getPortNode(thePort, true);
	if(theNode != NULL)
	{
		if(theNode->prev != NULL)
			theNode->prev->next = theNode->next;
		else
			FirstNode = theNode->next;
		
		if(theNode->next != NULL)
			theNode->next->prev = theNode->prev;
		else
			LastNode = theNode->prev;
		
		SerialPort* port = theNode->port;
		delete theNode;
		delete port;
		return "Null";
	}
	else
		return "$Failed";
}

void serialSetOptions()
{
	long argc;
	if(MLCheckFunction(stdlink, "List", &argc))
	{
		int thePort;
		if((argc == 2) && MLGetInteger(stdlink, &thePort))
		{
			SerialPort* port = getPortObject(thePort, true);
			if(port != NULL)
			{
				if(MLCheckFunction(stdlink, "List", &argc))
				{
					for(long i = 0; i < argc; i++)
					{
						long argc2;
						if(MLCheckFunction(stdlink, "Rule", &argc2) && (argc2 == 2))
						{
							const char* lhs;
							if(MLGetString(stdlink, &lhs))
							{
								int rhs;
								if(MLGetInteger(stdlink, &rhs))
								{
									if(strcmp(lhs, "BaudRate") == 0)
										port->setSpeed(rhs);
									else if(strcmp(lhs, "DataBits") == 0)
										port->setCharSize((SerialPort::CharSize) rhs);
									else if(strcmp(lhs, "StopBits") == 0)
										port->setDoubleStopBits(rhs == 2);
									else if(strcmp(lhs, "Parity") == 0)
										port->setParity((SerialPort::Parity) rhs);
								}
								MLDisownString(stdlink, lhs);
							}
						}
					}
					
					MLPutFunction(stdlink, "List", 4);
						MLPutFunction(stdlink, "Rule", 2);
							MLPutString(stdlink, "BaudRate");
							MLPutInteger(stdlink, port->getSpeed());
						MLPutFunction(stdlink, "Rule", 2);
							MLPutString(stdlink, "DataBits");
							MLPutInteger(stdlink, port->getCharSize());
						MLPutFunction(stdlink, "Rule", 2);
							MLPutString(stdlink, "StopBits");
							MLPutInteger(stdlink, port->getDoubleStopBits() ? 2 : 1);
						MLPutFunction(stdlink, "Rule", 2);
							MLPutString(stdlink, "Parity");
							MLPutInteger(stdlink, port->getParity());
					return;
				}
			}
		}
	}
	MLPutSymbol(stdlink, "$Failed");
}

char* serialReadyQ(int thePort, double theTimeout)
{
	SerialPort* port = getPortObject(thePort, true);
	if(port != NULL)
	{
		if(port->isDataAvailable(theTimeout))
			return "True";
		else
			return "False";
	}
	else
		return "$Failed";
}

#define kBufferSize 1024

void serialRead(int thePort, double theTimeout)
{
	SerialPort* port = getPortObject(thePort, true);
	if(port != NULL)
	{
		char buffer[kBufferSize];
		size_t res = port->readData(buffer, kBufferSize, theTimeout);
		MLPutByteString(stdlink, (kucharp_ct) buffer, res);
	}
	else
		MLPutSymbol(stdlink, "$Failed");
}

void serialWrite(int thePort, kucharp_ct theData, int len)
{
	SerialPort* port = getPortObject(thePort, true);
	if(port != NULL)
	{
		len = port->writeData((char*) theData, len);
		MLPutByteString(stdlink, theData, len);
	}
	else
		MLPutSymbol(stdlink, "$Failed");
}

/*
 * MathLink template
 */

:Evaluate: Begin["SerialIO`Private`"]

:Begin:
:Function: serialOpen
:Pattern: serialOpen[pathName_String]
:Arguments: { pathName }
:ArgumentTypes: { String }
:ReturnType: Manual
:End:

:Begin:
:Function: serialClose
:Pattern: serialClose[port_Integer]
:Arguments: { port }
:ArgumentTypes: { Integer }
:ReturnType: Symbol
:End:

:Begin:
:Function: serialSetOptions
:Pattern: serialSetOptions[port_Integer, opts_List]
:Arguments: { port, opts }
:ArgumentTypes: Manual
:ReturnType: Manual
:End:

:Begin:
:Function: serialReadyQ
:Pattern: serialReadyQ[port_Integer, timeout_Real]
:Arguments: { port, timeout }
:ArgumentTypes: { Integer, Real }
:ReturnType: Symbol
:End:

:Begin:
:Function: serialRead
:Pattern: serialRead[port_Integer, timeout_Real]
:Arguments: { port, timeout }
:ArgumentTypes: { Integer, Real }
:ReturnType: Manual
:End:

:Begin:
:Function: serialWrite
:Pattern: serialWrite[port_Integer, data_String]
:Arguments: { port, data }
:ArgumentTypes: { Integer, ByteString }
:ReturnType: Manual
:End:

:Evaluate: End[]

/*
 * main()
 */

#if WINDOWS_MATHLINK
int __stdcall WinMain(HINSTANCE, HINSTANCE, LPSTR lpszCmdLine, int)
{
	char  buff[512];
	char FAR * buff_start = buff;
	char FAR * argv[32];
	char FAR * FAR * argv_end = argv + 32;
	
	MLScanString( argv, &argv_end, &lpszCmdLine, &buff_start);
	return MLMain( argv_end - argv, argv);
}
#else
int main(int argc, charpp_ct argv)
{
	return MLMain(argc, argv);
}
#endif // WINDOWS_MATHLINK
