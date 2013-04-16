///
#ifndef __SERIALPORT__
#define __SERIALPORT__

#include <sys/types.h>		// for size_t
#include <stdlib.h>			// for NULL
#include <string>

#if WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

///
class SerialPort
{
public:
#if WIN32
	///
	typedef DWORD Speed;
	
	///
	typedef DCB Attributes;
	
	///
	typedef HANDLE Port;
#else
	///
	typedef speed_t Speed;
	
	///
	typedef struct termios Attributes;
	
	///
	typedef int Port;
#endif // WIN32
	
	///
	enum CharSize
	{
		e0 = 0, // uknown or error
		e5 = 5,
		e6 = 6,
		e7 = 7,
		e8 = 8
	};
	
	///
	enum Parity
	{
		eNone = 0,
		eOdd = 1,
		eEven = 2
	};
	
	/// constructor
	SerialPort(const std::string& thePathName);
	
	/// destructor
	~SerialPort();
	
	///
	int getIdentifier();
	
	///
	bool openPort();
	
	///
	void closePort();
	
	///
	bool isOpen();
	
	///
	size_t isDataAvailable(double theTimeout = 0.0);
	
	///
	size_t readData(void* theBuffer, size_t theBufferSize, double theTimeout = 0.0);
	
	///
	size_t writeData(const void* theBuffer, size_t theBufferSize);
	
	///
	Speed getSpeed();
	
	///
	bool setSpeed(Speed theSpeed);
	
	///
	CharSize getCharSize();
	
	///
	bool setCharSize(CharSize theCharSize);
	
	///
	Parity getParity();
	
	///
	bool setParity(Parity theParity);
	
	///
	bool getDoubleStopBits();
	
	///
	bool setDoubleStopBits(bool theDoubleStopBits);
	
	///
	bool getSoftwareFlowControl();
	
	///
	bool setSoftwareFlowControl(bool theSoftwareFlowControl);
	
	///
	bool getHardwareFlowControl();
	
	///
	bool setHardwareFlowControl(bool theHardwareFlowControl);
	
protected:
	///
	void printData_(const char* theMessage, const void* theBuffer, size_t theBufferSize);
	
	///
	bool beginAttrChange_(Attributes& theAttrs);
	
	///
	bool endAttrChange_(Attributes& theAttrs);

	
	///
	std::string pathName_;
	
	///
	Port port_;
	
	///
	Attributes oldAttrs_;
	
	///
	Attributes newAttrs_;
};

#endif // __SERIALPORT__
