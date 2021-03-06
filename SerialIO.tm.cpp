/*
 * This file automatically produced by /Applications/Mathematica.app/SystemFiles/Links/MathLink/DeveloperKit/CompilerAdditions//mprep from:
 *	/Files/schofield/ragfield-packages/SerialIO/SerialIO.tm
 * mprep Revision 14 Copyright (c) Wolfram Research, Inc. 1990-2008
 */

#define MPREP_REVISION 14

#if CARBON_MPREP
#include <Carbon/Carbon.h>
#include <mathlink/mathlink.h>
#else
#include "mathlink.h"
#endif

int MLAbort = 0;
int MLDone  = 0;
long MLSpecialCharacter = '\0';

MLINK stdlink = 0;
MLEnvironment stdenv = 0;
#if MLINTERFACE >= 3
MLYieldFunctionObject stdyielder = (MLYieldFunctionObject)0;
MLMessageHandlerObject stdhandler = (MLMessageHandlerObject)0;
#else
MLYieldFunctionObject stdyielder = 0;
MLMessageHandlerObject stdhandler = 0;
#endif /* MLINTERFACE >= 3 */

#if DARWIN_MATHLINK && CARBON_MPREP
#define rMenuBar	1128
#define rAboutBox	1128
#define rBadSIZE	1127
#define mApple		1128
#define iAbout		1
#define mFile		1129
#define mEdit		1130

AEEventHandlerUPP handle_core_ae_upp;
ModalFilterUPP about_filter_upp;
UserItemUPP outline_hook_upp;

extern pascal OSErr handle_core_ae( const AppleEvent* event, AppleEvent* reply, long refcon);
extern int init_macintosh( void);
extern void do_about_box( void);
extern int _handle_user_event( unsigned long ticks);

pascal OSErr handle_core_ae( const AppleEvent* event, AppleEvent* reply, long refcon)
{
	DescType eventid, gottype;
	Size got;
	reply = (AppleEvent*)0; /* suppress unused warning */
	refcon = 0; /* suppress unused warning */
	if( noErr == AEGetAttributePtr(event, keyEventIDAttr, typeType, &gottype, (Ptr)&eventid, sizeof(eventid), &got)
	&& errAEDescNotFound == AEGetAttributePtr(event, keyMissedKeywordAttr, typeWildCard, &gottype, nil, 0, &got)
	){
		switch(eventid){
		case kAEQuitApplication:
			MLDone = MLAbort = 1;
		case kAEOpenApplication:
			return noErr;
		}
	}
	return errAEEventNotHandled;
}


static void set_about_item(void){
	Str255 aboutitem;
	StringHandle abouthandle;

	GetMenuItemText( GetMenuHandle(mApple), iAbout, aboutitem);
	abouthandle = NewString( aboutitem);
	if( abouthandle){
		StringPtr curApName = LMGetCurApName();
		long len = Munger( (Handle)abouthandle, 1, "MathLink\252", 9, curApName + 1, *curApName); 
		if( len > 0){
			**abouthandle = (unsigned char)len; 
			HLock( (Handle)abouthandle);
			SetMenuItemText( GetMenuHandle(mApple), iAbout, *abouthandle);
		}
		DisposeHandle( (Handle)abouthandle);
	}
}


static pascal Boolean about_filter(DialogPtr dptr, EventRecord *theEvent, short *theItem){
	if( theEvent->what == keyDown || theEvent->what == autoKey){
		unsigned char theKey = (unsigned char)(theEvent->message & charCodeMask);
		if( theKey == 0x0D || (theKey == 0x03 && !(theEvent->modifiers & controlKey))){
			short itemType;
			ControlHandle okHdl;
			Rect itemRect;
#if UNIVERSAL_INTERFACES_VERSION >= 0x0301
			unsigned long Ticks;
#else
			long Ticks;
#endif
			GetDialogItem( dptr, ok, &itemType, (Handle*) &okHdl, &itemRect);
			HiliteControl( okHdl, kControlButtonPart);
#ifdef __cplusplus
			Delay( 3, &Ticks);
#else
			Delay( 3, (void *)&Ticks);
#endif
			HiliteControl( okHdl, 0);
			*theItem = ok;
			return true;
		}
	}
	return false;
}

static pascal void outline_hook(DialogRef dptr, short theItem){
	short  itemType;
	Handle itemHdl;
	Rect itemRect;
	PenState oldpen;
	GetDialogItem( dptr, theItem, &itemType, &itemHdl, &itemRect);
	GetPenState( &oldpen);
	PenSize( 3, 3);
	FrameRoundRect( &itemRect, 16, 16);
	SetPenState( &oldpen);
}



/* edit here and in mathlink.r */
static unsigned short missing_DITL[] = { 5,
	0, 0, 76, 120, 96, 200, 0x5C, 0x30, 0x30, 0x34, 0x5C, 0x30, 0x30, 0x32, 0x4F, 0x4B, /* '\004\002', 'OK', */
	0, 0, 12, 13, 30, 28, 0x5C, 0x32, 0x31, 0x30, 0x5C, 0x30, 0x30, 0x31, 0x41, 0x5C, 0x30, /*'\210\001', 'A\0', */
	0, 0, 12, 27, 30, 96, 0x5C, 0x32, 0x31, 0x30, 0x5C, 0x30, 0x31, 0x30, 0x20, 0x4D, 0x61, 0x74, 0x68, 0x4C, 0x69, 0x6E, 0x6B, /*'\210\010', 'Ma','th','Li','nk', */
	0, 0, 12, 95, 30, 308, 0x5C, 0x32, 0x31, 0x30, 0x5C, 0x30, 0x33, 0x34, 0x5C, 0x32, 0x35, 0x32, 0x20, 0x70, 0x72, 0x6F, 0x67, 0x72, 0x61, 0x6D, 0x20, 0x67, 0x65, 0x6E, 0x65, 0x72, 0x61, 0x74, 0x65, 0x64, 0x20, 0x62, 0x79, 0x20, 0x6D, 0x70, 0x72, 0x65, 0x70, /*'\210\034', '\252','pr','og','ra','m ','ge','ne','ra','te','d ','by',' m','pr','ep', */
	0, 0, 42, 49, 56, 271, 0x5C, 0x32, 0x31, 0x30, 0x5C, 0x30, 0x35, 0x30, 0x6D, 0x70, 0x72, 0x65, 0x70, 0x5C, 0x32, 0x35, 0x31, 0x20, 0x57, 0x6F, 0x6C, 0x66, 0x72, 0x61, 0x6D, 0x20, 0x52, 0x65, 0x73, 0x65, 0x61, 0x72, 0x63, 0x68, 0x2C, 0x20, 0x49, 0x6E, 0x63, 0x2E, 0x20, 0x31, 0x39, 0x39, 0x30, 0x2D, 0x32, 0x30, 0x30, 0x32, /*'\210\050', 'mp','re','p ','\251 ','Wo','lf','ra','m','Re','se','ar','ch',', ','In','c.',' 1','99','0-','20','02', */ /* 1990-2002 */
	0, 0, 170, 10, 190, 30, 0x5C, 0x32, 0x30, 0x30, 0x5C, 0x30, 0x30, 0x30 /*'\200\000' */
};


int init_macintosh( void)
{
	static int initdone = 0;
	Handle menuBar;
	long attributes;

	/* semaphore required for preemptive threads */
	if( initdone) return initdone == 1;
	initdone = -1;

	/* should I check for MLNK resource too as launch-filtering is done based on it? */
	/* too late--since I'm running there likely wasn't a problem (this time anyway). */
	
	menuBar = GetNewMBar(rMenuBar);
	if( menuBar){
		SetMenuBar(menuBar);
		DisposeHandle(menuBar);
	}else{
		MenuHandle am, fm, em;
		am = NewMenu( mApple, (unsigned char*)"\001\024");
		fm = NewMenu( mFile, (unsigned char*)"\004File");
		em = NewMenu( mEdit, (unsigned char*)"\004Edit");
		if( !am || !fm || !em) return 0;
		AppendMenu( am, (unsigned char*)"\022About MathLink\252\311;-");
                DisableMenuItem(am, 0);
		InsertMenu( am, 0);
		AppendMenu( fm, (unsigned char*)"\006Quit/Q");
		InsertMenu( fm, 0);
		AppendMenu( em, (unsigned char*)"\043Undo/Z;-;Cut/X;Copy/C;Paste/V;Clear");
                DisableMenuItem(em, 0);
		InsertMenu( em, 0);
	}

	AppendResMenu( GetMenuHandle(mApple), 'DRVR');
	set_about_item();
	DrawMenuBar();
	about_filter_upp =  NewModalFilterUPP( about_filter);
	outline_hook_upp = NewUserItemUPP( outline_hook);
	if( Gestalt( gestaltAppleEventsAttr, &attributes) == noErr
	&& ((1 << gestaltAppleEventsPresent) & attributes)){
		handle_core_ae_upp = NewAEEventHandlerUPP( handle_core_ae);
		(void) AEInstallEventHandler( kCoreEventClass, typeWildCard, handle_core_ae_upp, 0, false);
	}else{
		return 0; /* this may be too strong since I am, after all, running. */
	}

	initdone = 1;
	return initdone == 1;
}

void do_about_box( void)
{
	GrafPtr oldPort;
	DialogPtr dptr;
	short item, itemType;
	Handle itemHdl;
	Rect itemRect;

	dptr = GetNewDialog( rAboutBox, nil, (WindowPtr)-1L);
	
	if( dptr == (DialogPtr)0){
		Handle items = NewHandle( sizeof(missing_DITL));
		static Rect bounds = {40, 20, 150, 340};

		if( ! items) return;
		BlockMove( missing_DITL, *items, sizeof(missing_DITL));

		dptr = NewColorDialog( nil, &bounds, (unsigned char*)"\005About",
					false, dBoxProc, (WindowPtr)-1L, false, 0, items);
                }
	
	if( dptr == (DialogPtr)0) return;
	GetPort (&oldPort);
	SetPort (GetDialogPort(dptr));
	GetDialogItem( dptr, ok, &itemType, &itemHdl, &itemRect);
	InsetRect( &itemRect, -4, -4);
	SetDialogItem( dptr, 6, userItem + itemDisable, (Handle)outline_hook_upp, &itemRect);

	FlushEvents( everyEvent, 0);
        ShowWindow( GetDialogWindow(dptr));

	do {
		ModalDialog( about_filter_upp, &item);
	} while( item != ok);

	DisposeDialog( dptr);
	SetPort( oldPort);
}

int _handle_user_event( unsigned long ticks)
{
	EventRecord event;

	if( WaitNextEvent(everyEvent, &event, ticks, nil)){
		long      menuResult = 0;
		short     menuID, menuItem;
		WindowPtr window;
		Str255    daName;

		switch ( event.what ) {
		case mouseDown:
			if( FindWindow(event.where, &window) == inMenuBar)
				menuResult = MenuSelect(event.where);
			break;
		case keyDown:
			if( event.modifiers & cmdKey )
				menuResult = MenuKey((short)event.message & charCodeMask);
			break;
		case kHighLevelEvent:
			AEProcessAppleEvent(&event);
			break;
		}

		menuID = HiWord(menuResult);
		menuItem = LoWord(menuResult);
		switch ( menuID ) {
		case mFile:
			MLDone = MLAbort = 1;
			break;
		case mApple:
			switch ( menuItem ) {
			case iAbout:
				do_about_box();
				break;
			default:
				GetMenuItemText(GetMenuHandle(mApple), menuItem, daName);
				break;
			}
			HiliteMenu(0);
		}
	}
	return MLDone;
}

#if MLINTERFACE >= 3
MLYDEFN( int, MLDefaultYielder, ( MLINK mlp, MLYieldParameters yp))
#else
MLYDEFN( devyield_result, MLDefaultYielder, ( MLINK mlp, MLYieldParameters yp))
#endif /* MLINTERFACE >= 3 */
{
	mlp = mlp; /* suppress unused warning */
#if MLINTERFACE >= 3
	return (int)_handle_user_event( MLSleepYP(yp));
#else
	return _handle_user_event( MLSleepYP(yp));
#endif /* MLINTERFACE >= 3 */
}

#endif /* (DARWIN_MATHLINK && CARBON_MPREP */

/********************************* end header *********************************/
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


void serialOpen P(( const char * _tp1));

#if MLPROTOTYPES
static int _tr0( MLINK mlp)
#else
static int _tr0(mlp) MLINK mlp;
#endif
{
	int	res = 0;
	const char * _tp1;
	if ( ! MLGetString( mlp, &_tp1) ) goto L0;
	if ( ! MLNewPacket(mlp) ) goto L1;

	serialOpen(_tp1);

	res = 1;
L1:	MLDisownString(mlp, _tp1);

L0:	return res;
} /* _tr0 */


char* serialClose P(( int _tp1));

#if MLPROTOTYPES
static int _tr1( MLINK mlp)
#else
static int _tr1(mlp) MLINK mlp;
#endif
{
	int	res = 0;
	const char * _tp0;
	int _tp1;
	if ( ! MLGetInteger( mlp, &_tp1) ) goto L0;
	if ( ! MLNewPacket(mlp) ) goto L1;

	_tp0 = serialClose(_tp1);

	res = MLAbort ?
		MLPutFunction( mlp, "Abort", 0) : MLPutSymbol( mlp, _tp0 ? _tp0 : "$Failed");
L1: 
L0:	return res;
} /* _tr1 */


void serialSetOptions P(( void));

#if MLPROTOTYPES
static int _tr2( MLINK mlp)
#else
static int _tr2(mlp) MLINK mlp;
#endif
{
	int	res = 0;
		if( !mlp) return res; /* avoid unused parameter warning */

	serialSetOptions();

	res = 1;

	return res;
} /* _tr2 */


char* serialReadyQ P(( int _tp1, double _tp2));

#if MLPROTOTYPES
static int _tr3( MLINK mlp)
#else
static int _tr3(mlp) MLINK mlp;
#endif
{
	int	res = 0;
	const char * _tp0;
	int _tp1;
	double _tp2;
	if ( ! MLGetInteger( mlp, &_tp1) ) goto L0;
	if ( ! MLGetReal( mlp, &_tp2) ) goto L1;
	if ( ! MLNewPacket(mlp) ) goto L2;

	_tp0 = serialReadyQ(_tp1, _tp2);

	res = MLAbort ?
		MLPutFunction( mlp, "Abort", 0) : MLPutSymbol( mlp, _tp0 ? _tp0 : "$Failed");
L2: L1: 
L0:	return res;
} /* _tr3 */


void serialRead P(( int _tp1, double _tp2));

#if MLPROTOTYPES
static int _tr4( MLINK mlp)
#else
static int _tr4(mlp) MLINK mlp;
#endif
{
	int	res = 0;
	int _tp1;
	double _tp2;
	if ( ! MLGetInteger( mlp, &_tp1) ) goto L0;
	if ( ! MLGetReal( mlp, &_tp2) ) goto L1;
	if ( ! MLNewPacket(mlp) ) goto L2;

	serialRead(_tp1, _tp2);

	res = 1;
L2: L1: 
L0:	return res;
} /* _tr4 */


void serialWrite P(( int _tp1, const unsigned char * _tp2, int _tpl2));

#if MLPROTOTYPES
static int _tr5( MLINK mlp)
#else
static int _tr5(mlp) MLINK mlp;
#endif
{
	int	res = 0;
	int _tp1;
	const unsigned char * _tp2;
	int _tpl2;
	if ( ! MLGetInteger( mlp, &_tp1) ) goto L0;
	if ( ! MLGetByteString( mlp, &_tp2, &_tpl2, MLSpecialCharacter) ) goto L1;
	if ( ! MLNewPacket(mlp) ) goto L2;

	serialWrite(_tp1, _tp2, _tpl2);

	res = 1;
L2:	MLDisownByteString( mlp, _tp2, _tpl2);
L1: 
L0:	return res;
} /* _tr5 */


static struct func {
	int   f_nargs;
	int   manual;
	int   (*f_func)P((MLINK));
	char  *f_name;
	} _tramps[6] = {
		{ 1, 0, _tr0, "serialOpen" },
		{ 1, 0, _tr1, "serialClose" },
		{-1, 1, _tr2, "serialSetOptions" },
		{ 2, 0, _tr3, "serialReadyQ" },
		{ 2, 0, _tr4, "serialRead" },
		{ 2, 0, _tr5, "serialWrite" }
		};

static char* evalstrs[] = {
	"Begin[\"SerialIO`Private`\"]",
	(char*)0,
	"End[]",
	(char*)0,
	(char*)0
};
#define CARDOF_EVALSTRS 2

static int _definepattern P(( MLINK, char*, char*, int));

static int _doevalstr P(( MLINK, int));

int  _MLDoCallPacket P(( MLINK, struct func[], int));


#if MLPROTOTYPES
int MLInstall( MLINK mlp)
#else
int MLInstall(mlp) MLINK mlp;
#endif
{
	int _res;
	_res = MLConnect(mlp);
	if (_res) _res = _doevalstr( mlp, 0);
	if (_res) _res = _definepattern(mlp, "serialOpen[pathName_String]", "{ pathName }", 0);
	if (_res) _res = _definepattern(mlp, "serialClose[port_Integer]", "{ port }", 1);
	if (_res) _res = _definepattern(mlp, "serialSetOptions[port_Integer, opts_List]", "{ port, opts }", 2);
	if (_res) _res = _definepattern(mlp, "serialReadyQ[port_Integer, timeout_Real]", "{ port, timeout }", 3);
	if (_res) _res = _definepattern(mlp, "serialRead[port_Integer, timeout_Real]", "{ port, timeout }", 4);
	if (_res) _res = _definepattern(mlp, "serialWrite[port_Integer, data_String]", "{ port, data }", 5);
	if (_res) _res = _doevalstr( mlp, 1);
	if (_res) _res = MLPutSymbol( mlp, "End");
	if (_res) _res = MLFlush( mlp);
	return _res;
} /* MLInstall */


#if MLPROTOTYPES
int MLDoCallPacket( MLINK mlp)
#else
int MLDoCallPacket( mlp) MLINK mlp;
#endif
{
	return _MLDoCallPacket( mlp, _tramps, 6);
} /* MLDoCallPacket */

/******************************* begin trailer ********************************/

#ifndef EVALSTRS_AS_BYTESTRINGS
#	define EVALSTRS_AS_BYTESTRINGS 1
#endif

#if CARDOF_EVALSTRS
static int  _doevalstr( MLINK mlp, int n)
{
	long bytesleft, charsleft, bytesnow;
#if !EVALSTRS_AS_BYTESTRINGS
	long charsnow;
#endif
	char **s, **p;
	char *t;

	s = evalstrs;
	while( n-- > 0){
		if( *s == 0) break;
		while( *s++ != 0){}
	}
	if( *s == 0) return 0;
	bytesleft = 0;
	charsleft = 0;
	p = s;
	while( *p){
		t = *p; while( *t) ++t;
		bytesnow = t - *p;
		bytesleft += bytesnow;
		charsleft += bytesnow;
#if !EVALSTRS_AS_BYTESTRINGS
		t = *p;
		charsleft -= MLCharacterOffset( &t, t + bytesnow, bytesnow);
		/* assert( t == *p + bytesnow); */
#endif
		++p;
	}


	MLPutNext( mlp, MLTKSTR);
#if EVALSTRS_AS_BYTESTRINGS
	p = s;
	while( *p){
		t = *p; while( *t) ++t;
		bytesnow = t - *p;
		bytesleft -= bytesnow;
		MLPut8BitCharacters( mlp, bytesleft, (unsigned char*)*p, bytesnow);
		++p;
	}
#else
	MLPut7BitCount( mlp, charsleft, bytesleft);
	p = s;
	while( *p){
		t = *p; while( *t) ++t;
		bytesnow = t - *p;
		bytesleft -= bytesnow;
		t = *p;
		charsnow = bytesnow - MLCharacterOffset( &t, t + bytesnow, bytesnow);
		/* assert( t == *p + bytesnow); */
		charsleft -= charsnow;
		MLPut7BitCharacters(  mlp, charsleft, *p, bytesnow, charsnow);
		++p;
	}
#endif
	return MLError( mlp) == MLEOK;
}
#endif /* CARDOF_EVALSTRS */


static int  _definepattern( MLINK mlp, char* patt, char* args, int func_n)
{
	MLPutFunction( mlp, "DefineExternal", (long)3);
	  MLPutString( mlp, patt);
	  MLPutString( mlp, args);
	  MLPutInteger( mlp, func_n);
	return !MLError(mlp);
} /* _definepattern */


int _MLDoCallPacket( MLINK mlp, struct func functable[], int nfuncs)
{
	long len;
	int n, res = 0;
	struct func* funcp;

	if( ! MLGetInteger( mlp, &n) ||  n < 0 ||  n >= nfuncs) goto L0;
	funcp = &functable[n];

	if( funcp->f_nargs >= 0
	&& ( ! MLCheckFunction(mlp, "List", &len)
	     || ( !funcp->manual && (len != funcp->f_nargs))
	     || (  funcp->manual && (len <  funcp->f_nargs))
	   )
	) goto L0;

	stdlink = mlp;
	res = (*funcp->f_func)( mlp);

L0:	if( res == 0)
		res = MLClearError( mlp) && MLPutSymbol( mlp, "$Failed");
	return res && MLEndPacket( mlp) && MLNewPacket( mlp);
} /* _MLDoCallPacket */


mlapi_packet MLAnswer( MLINK mlp)
{
	mlapi_packet pkt = 0;

	while( !MLDone && !MLError(mlp) && (pkt = MLNextPacket(mlp), pkt) && pkt == CALLPKT){
		MLAbort = 0;
		if( !MLDoCallPacket(mlp)) pkt = 0;
	}
	MLAbort = 0;
	return pkt;
} /* MLAnswer */



/*
	Module[ { me = $ParentLink},
		$ParentLink = contents of RESUMEPKT;
		Message[ MessageName[$ParentLink, "notfe"], me];
		me]
*/

static int refuse_to_be_a_frontend( MLINK mlp)
{
	int pkt;

	MLPutFunction( mlp, "EvaluatePacket", 1);
	  MLPutFunction( mlp, "Module", 2);
	    MLPutFunction( mlp, "List", 1);
		  MLPutFunction( mlp, "Set", 2);
		    MLPutSymbol( mlp, "me");
	        MLPutSymbol( mlp, "$ParentLink");
	  MLPutFunction( mlp, "CompoundExpression", 3);
	    MLPutFunction( mlp, "Set", 2);
	      MLPutSymbol( mlp, "$ParentLink");
	      MLTransferExpression( mlp, mlp);
	    MLPutFunction( mlp, "Message", 2);
	      MLPutFunction( mlp, "MessageName", 2);
	        MLPutSymbol( mlp, "$ParentLink");
	        MLPutString( mlp, "notfe");
	      MLPutSymbol( mlp, "me");
	    MLPutSymbol( mlp, "me");
	MLEndPacket( mlp);

	while( (pkt = MLNextPacket( mlp), pkt) && pkt != SUSPENDPKT)
		MLNewPacket( mlp);
	MLNewPacket( mlp);
	return MLError( mlp) == MLEOK;
}


int MLEvaluate( MLINK mlp, char* s)
{
	if( MLAbort) return 0;
	return MLPutFunction( mlp, "EvaluatePacket", 1L)
		&& MLPutFunction( mlp, "ToExpression", 1L)
		&& MLPutString( mlp, s)
		&& MLEndPacket( mlp);
} /* MLEvaluate */


int MLEvaluateString( MLINK mlp, char* s)
{
	int pkt;
	if( MLAbort) return 0;
	if( MLEvaluate( mlp, s)){
		while( (pkt = MLAnswer( mlp), pkt) && pkt != RETURNPKT)
			MLNewPacket( mlp);
		MLNewPacket( mlp);
	}
	return MLError( mlp) == MLEOK;
} /* MLEvaluateString */


#if MLINTERFACE >= 3
MLMDEFN( void, MLDefaultHandler, ( MLINK mlp, int message, int n))
#else
MLMDEFN( void, MLDefaultHandler, ( MLINK mlp, unsigned long message, unsigned long n))
#endif /* MLINTERFACE >= 3 */
{
	mlp = (MLINK)0; /* suppress unused warning */
	n = 0; /* suppress unused warning */

	switch (message){
	case MLTerminateMessage:
		MLDone = 1;
	case MLInterruptMessage:
	case MLAbortMessage:
		MLAbort = 1;
	default:
		return;
	}
}

#if MLINTERFACE >= 3
static int _MLMain( char **argv, char **argv_end, char *commandline)
#else
static int _MLMain( charpp_ct argv, charpp_ct argv_end, charp_ct commandline)
#endif /* MLINTERFACE >= 3 */
{
	MLINK mlp;
#if MLINTERFACE >= 3
	int err;
#else
	long err;
#endif /* MLINTERFACE >= 3 */

#if (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))
	if( !init_macintosh()) goto R0;
#endif /* (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__)) */

	if( !stdenv)
		stdenv = MLInitialize( (MLParametersPointer)0);
	if( stdenv == (MLEnvironment)0) goto R0;

#if (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))
#if MLINTERFACE >= 3
	if( !stdyielder)
		stdyielder = (MLYieldFunctionObject)MLDefaultYielder;
#else
	if( !stdyielder)
		stdyielder = MLCreateYieldFunction( stdenv, NewMLYielderProc( MLDefaultYielder), 0);
#endif /* MLINTERFACE >= 3 */
#endif /* (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))*/

#if MLINTERFACE >= 3
	if( !stdhandler)
		stdhandler = (MLMessageHandlerObject)MLDefaultHandler;
#else
	if( !stdhandler)
		stdhandler = MLCreateMessageHandler( stdenv, NewMLHandlerProc( MLDefaultHandler), 0);
#endif /* MLINTERFACE >= 3 */

#if (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))
        MLSetDialogFunction(stdenv, MLRequestToInteractFunction, NewMLRequestToInteractProc(MLDontPermit_darwin));

	mlp = commandline
		? MLOpenString( stdenv, commandline, &err)
#if MLINTERFACE >= 3
			: MLOpenArgcArgv( stdenv, (int)(argv_end - argv), argv, &err);
#else
			: MLOpenArgv( stdenv, argv, argv_end, &err);
#endif

        MLSetDialogFunction(stdenv, MLRequestToInteractFunction, NewMLRequestToInteractProc(MLPermit_darwin));

	if( mlp == (MLINK)0){
                        mlp = commandline
                                ? MLOpenString( stdenv, commandline, &err)
#if MLINTERFACE < 3
                                : MLOpenArgv( stdenv, argv, argv_end, &err);
#else
                                : MLOpenArgcArgv( stdenv, (int)(argv_end - argv), argv, &err);
#endif
        }
#else /* !(DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))*/
	mlp = commandline
		? MLOpenString( stdenv, commandline, &err)
#if MLINTERFACE < 3
		: MLOpenArgv( stdenv, argv, argv_end, &err);
#else
		: MLOpenArgcArgv( stdenv, (int)(argv_end - argv), argv, &err);
#endif
#endif /* (DARWIN_MATHLINK && defined(__CONDITIONALMACROS__))*/

	if( mlp == (MLINK)0){
		MLAlert( stdenv, MLErrorString( stdenv, err));
		goto R1;
	}

	if( stdyielder) MLSetYieldFunction( mlp, stdyielder);
	if( stdhandler) MLSetMessageHandler( mlp, stdhandler);

	if( MLInstall( mlp))
		while( MLAnswer( mlp) == RESUMEPKT){
			if( ! refuse_to_be_a_frontend( mlp)) break;
		}

	MLClose( mlp);
R1:	MLDeinitialize( stdenv);
	stdenv = (MLEnvironment)0;
R0:	return !MLDone;
} /* _MLMain */


#if MLINTERFACE >= 3
int MLMainString( char *commandline)
#else
int MLMainString( charp_ct commandline)
#endif /* MLINTERFACE >= 3 */
{
#if MLINTERFACE >= 3
	return _MLMain( (char **)0, (char **)0, commandline);
#else
	return _MLMain( (charpp_ct)0, (charpp_ct)0, commandline);
#endif /* MLINTERFACE >= 3 */
}

int MLMainArgv( char** argv, char** argv_end) /* note not FAR pointers */
{   
	static char FAR * far_argv[128];
	int count = 0;
	
	while(argv < argv_end)
		far_argv[count++] = *argv++;
		 
	return _MLMain( far_argv, far_argv + count, (charp_ct)0);

}

#if MLINTERFACE >= 3
int MLMain( int argc, char ** argv)
#else
int MLMain( int argc, charpp_ct argv)
#endif /* MLINTERFACE >= 3 */
{
#if MLINTERFACE >= 3
 	return _MLMain( argv, argv + argc, (char *)0);
#else
 	return _MLMain( argv, argv + argc, (charp_ct)0);
#endif /* MLINTERFACE >= 3 */
}
