///
#include "SerialPort.h"

#if WIN32
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <strings.h>
#endif // WIN32

#include <stdio.h>
#include <string.h>

#if WIN32
static const SerialPort::Port kInvalidPort = INVALID_HANDLE_VALUE;
#else
static const SerialPort::Port kInvalidPort = -1;
#endif // WIN32

SerialPort::SerialPort(const std::string& thePathName)
	: pathName_(thePathName), port_(kInvalidPort)
{
	memset(&oldAttrs_, 0, sizeof(oldAttrs_));
	memset(&newAttrs_, 0, sizeof(newAttrs_));
#if WIN32
	oldAttrs_.DCBlength = sizeof(oldAttrs_);
	newAttrs_.DCBlength = sizeof(newAttrs_);
#endif // WIN32
}

SerialPort::~SerialPort()
{
	if(isOpen())
		closePort();
}

int SerialPort::getIdentifier()
{
#if WIN32
	return (int) port_;
#else
	return port_;
#endif // WIN32
}

bool SerialPort::openPort()
{
#if WIN32
	std::string thePathName = "\\\\.\\" + pathName_;
	port_ = CreateFile(thePathName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if(port_ == INVALID_HANDLE_VALUE)
		return false; // GetLastError()
	
	if(!GetCommState(port_, &oldAttrs_))
	{
		CloseHandle(port_);
		port_ = kInvalidPort;
		return false;
	}
	
	newAttrs_ = oldAttrs_;
	
	return true;
#else
	port_ = open(pathName_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(port_ < 0)	
		return false;
	
	// make sure no other processes open this file while we have it open
	if(ioctl(port_, TIOCEXCL) < 0)
	{
		close(port_);
		port_ = kInvalidPort;
		return false;
	}
	
	// re-allow blocking io
	if(fcntl(port_, F_SETFL, 0) < 0)
	{
		close(port_);
		port_ = kInvalidPort;
		return false;
	}
	
	// save the original terminal control attributes
	if(tcgetattr(port_, &oldAttrs_) < 0)
	{
		close(port_);
		port_ = kInvalidPort;
		return false;
	}
	
	newAttrs_ = oldAttrs_;
	newAttrs_.c_cflag |= CLOCAL | CREAD;
	if(tcsetattr(port_, TCSANOW, &newAttrs_) < 0)
	{
		close(port_);
		port_ = kInvalidPort;
		return false;
	}
	
	return true;
#endif // WIN32
}

void SerialPort::closePort()
{
	if(!isOpen())
		return;
	
#if WIN32
	if(!SetCommState(port_, &oldAttrs_))
		return;
	
	if(!CloseHandle(port_))
		return; // GetLastError()
#else
	if(tcdrain(port_) < 0)
		return;
	
	if(tcsetattr(port_, TCSANOW, &oldAttrs_) < 0)
		return;
	
	close(port_);
#endif // WIN32
	
	port_ = kInvalidPort;
}

bool SerialPort::isOpen()
{
#if WIN32
	return (port_ != INVALID_HANDLE_VALUE);
#else
	return (port_ >= 0);
#endif // WIN32
}

size_t SerialPort::isDataAvailable(double theTimeout)
{
	if(theTimeout < 0)
		theTimeout = 0.0;
		
#if WIN32
	return 0;
#else
	fd_set theFiles;
	FD_ZERO(&theFiles);
	FD_SET(port_, &theFiles);
	
	struct timeval theTime;
	theTime.tv_sec = (int) theTimeout;
	theTime.tv_usec = (int) ((theTimeout - theTime.tv_sec) * 1000000);
	
	int res = select(port_ + 1, &theFiles, NULL, NULL, &theTime);
	if((res == 1) && FD_ISSET(port_, &theFiles))
	{
		ioctl(port_, FIONREAD, &res);
		return res;
	}
	return 0;
#endif // WIN32
}

size_t SerialPort::readData(void* theBuffer, size_t theBufferSize, double theTimeout)
{
	if(!isOpen() && !openPort())
		return 0;
	
	if(theTimeout < 0)
		theTimeout = 0.0;
	
#if WIN32
	COMMTIMEOUTS theTimeouts = { 0 };
	
	theTimeouts.ReadTotalTimeoutConstant = (DWORD)(theTimeout * 1000);
	if(!SetCommTimeouts(port_, &theTimeouts))
		return 0; // FIXME error
	
	DWORD res;
	if(!ReadFile(port_, theBuffer, 1, &res, NULL) || (res == 0))
	{
		DWORD err;
		ClearCommError(port_, &err, NULL);
		return 0;
	}
	
	theTimeouts.ReadTotalTimeoutConstant = 1;
	if(!SetCommTimeouts(port_, &theTimeouts))
	{
		DWORD err;
		ClearCommError(port_, &err, NULL);
		return res;
	}
	
	if(!ReadFile(port_, ((char*)theBuffer + 1), theBufferSize - 1, &res, NULL))
	{
		DWORD err;
		ClearCommError(port_, &err, NULL);
		return res;
	}
	res++;
#else
	fd_set theFiles;
	FD_ZERO(&theFiles);
	FD_SET(port_, &theFiles);
	
	struct timeval theTime;
	theTime.tv_sec = (int) theTimeout;
	theTime.tv_usec = (int) ((theTimeout - theTime.tv_sec) * 1000000);
	
	int res = select(port_ + 1, &theFiles, NULL, NULL, &theTime);
	if((res == 1) && FD_ISSET(port_, &theFiles))
		res = read(port_, theBuffer, theBufferSize);
	else
		res = 0;
#endif // WIN32
	
	if(res >= 0)
		printData_("Reading", theBuffer, res);
	return res;
}
	
size_t SerialPort::writeData(const void* theBuffer, size_t theBufferSize)
{
	if(!isOpen() && !openPort())
		return 0;
	
	printData_("Writing", theBuffer, theBufferSize);
	
#if WIN32
	DWORD theSize = 0;
	if(WriteFile(port_, theBuffer, theBufferSize, &theSize, NULL) == 0)
	{
		DWORD err;
		ClearCommError(port_, &err, NULL);
		return 0;
	}
	return theSize;
#else
	return write(port_, theBuffer, theBufferSize);
#endif // WIN32
}

SerialPort::Speed SerialPort::getSpeed()
{
	if(!isOpen() && !openPort())
		return 0;

#if WIN32
	return newAttrs_.BaudRate;
#else
	return cfgetospeed(&newAttrs_);
#endif // WIN32
}

bool SerialPort::setSpeed(SerialPort::Speed theSpeed)
{
	if(!isOpen() && !openPort())
		return false;
	
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		theAttrs.BaudRate = theSpeed;
#else
		cfsetispeed(&theAttrs, theSpeed);
		cfsetospeed(&theAttrs, theSpeed);
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

SerialPort::CharSize SerialPort::getCharSize()
{
	if(!isOpen() && !openPort())
		return e0;
	
#if WIN32
	switch(newAttrs_.ByteSize)
	{
		case 5: return e5;
		case 6: return e6;
		case 7: return e7;
		case 8: return e8;
		case 4:
		default:
			// assert()
			return e0;
	}
	return e0;
#else
	switch(newAttrs_.c_cflag & CSIZE)
	{
		case CS5: return e5;
		case CS6: return e6;
		case CS7: return e7;
		case CS8: return e8;
		default:
			//assert()
			return e0;
	}
#endif // WIN32
}

bool SerialPort::setCharSize(SerialPort::CharSize theCharSize)
{
	if(!isOpen() && !openPort())
		return false;
	
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		switch(theCharSize)
		{
			case e5: theAttrs.ByteSize = 5; break;
			case e6: theAttrs.ByteSize = 6; break;
			case e7: theAttrs.ByteSize = 7; break;
			case e8: theAttrs.ByteSize = 8; break;
			case e0:
			default:
				// assert()
				return false;
		}
#else
		theAttrs.c_cflag &= ~CSIZE;
		switch(theCharSize)
		{
			case e5: theAttrs.c_cflag |= CS5; break;
			case e6: theAttrs.c_cflag |= CS6; break;
			case e7: theAttrs.c_cflag |= CS7; break;
			case e8: theAttrs.c_cflag |= CS8; break;
			case e0:
				// assert
				return false;
		}
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

SerialPort::Parity SerialPort::getParity()
{
	if(!isOpen() && !openPort())
		return eNone;
		
#if WIN32
	switch(newAttrs_.Parity)
	{
		case 1: return eOdd;
		case 2: return eEven;
		case 0:
		default:
			return eNone;
	}
#else
	if((newAttrs_.c_cflag & PARENB) != 0)
	{
		if((newAttrs_.c_cflag & PARODD) != 0)
			return eOdd;
		else
			return eEven;
	}
	else
		return eNone;
#endif // WIN32
}

bool SerialPort::setParity(SerialPort::Parity theParity)
{
	if(!isOpen() && !openPort())
		return false;
	
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		switch(theParity)
		{
			case eNone: theAttrs.Parity = 0; break;
			case eOdd:  theAttrs.Parity = 1; break;
			case eEven: theAttrs.Parity = 2; break;
		}
#else
		switch(theParity)
		{
			case eNone:
				theAttrs.c_cflag &= ~PARENB;
				theAttrs.c_iflag &= ~INPCK;
				break;
			
			case eOdd:
				theAttrs.c_cflag |= (PARENB | PARODD);
				theAttrs.c_iflag |= INPCK;
				break;
			
			case eEven:
				theAttrs.c_cflag |= PARENB;
				theAttrs.c_cflag &= ~PARODD;
				theAttrs.c_iflag |= INPCK;
				break;
		}
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

bool SerialPort::getDoubleStopBits()
{
	if(!isOpen() && !openPort())
		return false;
	
#if WIN32
	return (newAttrs_.StopBits == 2);
#else
	return ((newAttrs_.c_cflag & CSTOPB) != 0);
#endif // WIN32
}

bool SerialPort::setDoubleStopBits(bool theDoubleStopBits)
{
	if(!isOpen() && !openPort())
		return false;
	
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		theAttrs.StopBits = (theDoubleStopBits ? 2 : 1);
#else
		if(theDoubleStopBits)
			theAttrs.c_cflag |= CSTOPB;
		else
			theAttrs.c_cflag &= ~CSTOPB;
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

bool SerialPort::getSoftwareFlowControl()
{
	if(!isOpen() && !openPort())
		return false;
	
#if WIN32
	return (newAttrs_.fInX || newAttrs_.fOutX);
#else
	return ((newAttrs_.c_cflag & (IXON | IXOFF)) != 0);
#endif // WIN32
}

bool SerialPort::setSoftwareFlowControl(bool theSoftwareFlowControl)
{
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		theAttrs.fInX = theSoftwareFlowControl;
		theAttrs.fOutX = theSoftwareFlowControl;
#else
		if(theSoftwareFlowControl)
			theAttrs.c_cflag |= (IXON | IXOFF);
		else
			theAttrs.c_cflag &= ~(IXON | IXOFF);
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

bool SerialPort::getHardwareFlowControl()
{
	if(!isOpen() && !openPort())
		return false;
	
#if WIN32
	return (newAttrs_.fOutxCtsFlow  || newAttrs_.fOutxDsrFlow);
#else
	return ((newAttrs_.c_cflag & CRTSCTS) != 0);
#endif // WIN32
}

bool SerialPort::setHardwareFlowControl(bool theHardwareFlowControl)
{
	Attributes theAttrs;
	if(beginAttrChange_(theAttrs))
	{
#if WIN32
		theAttrs.fOutxCtsFlow = theHardwareFlowControl;
		theAttrs.fOutxDsrFlow = theHardwareFlowControl;
#else
		if(theHardwareFlowControl)
			theAttrs.c_cflag |= CRTSCTS;
		else
			theAttrs.c_cflag &= ~CRTSCTS;
#endif // WIN32
		return endAttrChange_(theAttrs);
	}
	return false;
}

void SerialPort::printData_(const char* theMessage, const void* theBuffer, size_t theBufferSize)
{
	printf("%s: ", theMessage);
	for(size_t i = 0; i < theBufferSize; i++)
	{
		unsigned char c = ((char*)theBuffer)[i];
		if(c >= 32 && c < 127)
			printf("%c", c);
		else
			switch(c)
			{
				case '\t': printf("\\t"); break;
				case '\r': printf("\\r"); break;
				case '\n': printf("\\n"); break;
				default: printf("0x%.2x", c); break;
			}
	}
	printf("\n");
}

bool SerialPort::beginAttrChange_(Attributes& theAttrs)
{
	if(!isOpen() && !openPort())
		return false;
	
	theAttrs = newAttrs_;
	return true;
}

bool SerialPort::endAttrChange_(Attributes& theAttrs)
{
#if WIN32
	if(SetCommState(port_, &theAttrs))
#else
	if(tcsetattr(port_, TCSANOW, &theAttrs) == 0)
#endif // WIN32
	{
		newAttrs_ = theAttrs;
		return true;
	}
	return false;
}
