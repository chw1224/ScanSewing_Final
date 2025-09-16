/*********************************************************************
 * Serial Port Communication C++ Library
 * 2020 February - Sohee J. Yoon
 * 
 * This library provides a simpler interface to do serial communication
 * through virtual com ports in Windows. I have referenced various
 * serial port libraries online to construct this one.
 * Functions here are timed blocking calls (i.e. they do NOT use overlapped).
 * 
 * References:
 * 	- https://docs.microsoft.com/en-us/previous-versions/ff802693(v=msdn.10)?redirectedfrom=MSDN
 * 	- https://github.com/manashmandal/SerialPort
 * 	- https://www.codeguru.com/cpp/i-n/network/serialcommunications/article.php/c2503/CSerial--A-C-Class-for-Serial-Communications.htm
 *********************************************************************/
#pragma once
#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#include <windows.h>
#include <cstdint>
#include <iostream>
#include <string>

class SerialPort {
	private:
		HANDLE handler;
		bool connected;
		COMSTAT status;
		DWORD errors;

	public:
		SerialPort(unsigned int portNum, unsigned int baudrate, float timeout=0.1, DWORD rxBufferLen=1e4, DWORD txBufferLen=1e4);
		~SerialPort();

		bool isConnected();

		int read(uint8_t *buffer, unsigned int nbytes);
		int readUntil(uint8_t *buffer, unsigned int limit, char eos);
		int readline(uint8_t *buffer, unsigned int limit);
		int readAvailable(uint8_t *buffer, unsigned int limit);

		int write(uint8_t *buffer, unsigned int nbytes);

		inline void resetRxBuffer() { PurgeComm(this->handler, PURGE_RXCLEAR); }	// Clears received data stored in the rxBuffer.
		inline void resetTxBuffer() { PurgeComm(this->handler, PURGE_TXCLEAR); }	// Clears pending data stored in the txBuffer.
};

#endif // __SERIAL_PORT_H__