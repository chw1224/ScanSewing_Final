/*********************************************************************
 * Serial Port Communication C++ Library
 * 2020 February - 윤소희 Sohee J. Yoon
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
#include "SerialPort.h"

/* Constructor that creates a handle to the desired port and opens it.
 * @params
 * 		portNum			Desired COM port number (5 for "COM5")
 * 		baudrate		Communication baudrate (9600, 115200...)
 * 		timeout			Timeout duration in seconds. Zero returns immediately with data in the buffer (if any).
 * 		rxBufferLen		Buffer size for data in.
 * 		txBufferLen		Buffer size for data out.
 */
SerialPort::SerialPort(unsigned int portNum, unsigned int baudrate, float timeout, DWORD rxBufferLen, DWORD txBufferLen){
	// Configure arguments for convenience
	std::string portName = "\\\\.\\COM" + std::to_string(portNum);	// concat the port number to windows style port name
	int timeout_ms = (timeout < 0.001f) ? 0 : (int)(timeout*1000+0.5f);	// positive timeout with ms rounding

	// Create a serial port handle
	this->connected = false;
	this->handler = CreateFileA(
			static_cast<LPCSTR>(portName.c_str()),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL
		);

	// Check if handle is valid
	if(this->handler == INVALID_HANDLE_VALUE){
		// Invalid handle. Show error.
		switch(GetLastError()){
			case ERROR_FILE_NOT_FOUND:
				std::cout<<"SerialPort Error: Handle was not attached. COM"<<portNum<<" not found."<<std::endl;
				break;
			case ERROR_ACCESS_DENIED:
				std::cout<<"SerialPort Error: Handle was not attached. COM"<<portNum<<" access denied."<<std::endl;
				break;
			default:
				std::cout<<"SerialPort Error: Handle was not attached. Reason unknown."<<std::endl;
				break;
		}
		return;
	}

	// Valid handle. Configure timeouts.
	COMMTIMEOUTS CommTimeOuts;
	CommTimeOuts.ReadIntervalTimeout = (timeout_ms==0) ? MAXDWORD : timeout_ms;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = timeout_ms;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
	CommTimeOuts.WriteTotalTimeoutConstant = timeout_ms;
	if(!SetCommTimeouts(this->handler, &CommTimeOuts)){
		std::cout<<"SerialPort Error: Invalid handle. Reason unknown."<<std::endl;
		return;
	}

	// Serial port settings
	DCB dcbSerialParameters;
	if(!GetCommState(this->handler, &dcbSerialParameters)){
		std::cout<<"SerialPort Error: Failed to get serial parameters."<<std::endl;
		return;
	}
	dcbSerialParameters.BaudRate = baudrate;
	dcbSerialParameters.ByteSize = 8;
	dcbSerialParameters.StopBits = ONESTOPBIT;
	dcbSerialParameters.Parity = NOPARITY;
	dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

	// Configure and connect to serial port. Handler is now allocated.
	if(!SetCommState(handler, &dcbSerialParameters)
	|| !SetupComm(this->handler, rxBufferLen, txBufferLen)){
		std::cout<<"SerialPort Error: Failed to set serial parameters."<<std::endl;
		CloseHandle(this->handler);	// need to close the handler in case the serial has connected somehow.
		return;
	}

	// If code reaches here, serial is properly configured and connected.
	this->connected = true;
	PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);	// flush buffers

	// Sleep(1000);	// delay in ms for serial initialization
}

/* Destructor that closes the serial port handle.
 * Note that the handle does not need to be closed if it is disconnected.
 */
SerialPort::~SerialPort(){
	if(this->connected){
		this->connected = false;
		CloseHandle(this->handler);
	}
}

/* Returns connection state of serial port. */
bool SerialPort::isConnected(){
	if (!ClearCommError(this->handler, &this->errors, &this->status))
		this->connected = false;
		
	return this->connected;
}

/* Read a fixed number of bytes from serial port and store into user buffer.
 * Note that the user buffer should be large enough to store requested data.
 * @params
 * 		buffer		Buffer to store read data
 * 		nbytes		Number of bytes to read
 * @retval
 * 		Number of bytes successfully read
 */
int SerialPort::read(uint8_t *buffer, unsigned int nbytes){
	if(!this->connected || this->handler==NULL) return 0;

	DWORD dwBytesRead;
	
	if(ReadFile(this->handler, buffer, nbytes, &dwBytesRead, NULL))
		return (int)dwBytesRead;

	// If code reaches here, read has failed.
	return 0;
}

/* Read bytes until the specified end of sequence (eos) character is found.
 * Note that the function returns if the designated buffer size limit is
 * reached before the eos character is found.
 * @params
 * 		buffer	Buffer to store read data
 * 		limit	Size limit of buffer
 * 		eos		End of sequence character
 * @retval
 * 		Number of bytes successfully read
 */
int SerialPort::readUntil(uint8_t *buffer, unsigned int limit, char eos){
	uint8_t ch=0;
	int bytesRead=0;

	for(int i=0; i<limit; i++){
		// read one character at a time
		bytesRead += this->read(&ch, 1);
		buffer[i] = ch;

		// exit loop if end of sequence character is found
		if((char)ch == eos) break;
	}

	return bytesRead;
}

/* Read bytes until a newline character ('\n') is found or until
 * the buffer provided reaches its size limit.
 */
int SerialPort::readline(uint8_t *buffer, unsigned int limit){
	return this->readUntil(buffer, limit, '\n');
}

/* Attempt to read all available data in rxBuffer of serial port.
 * If the user buffer is too small, not all data in rxBuffer will be read.
 * This function returns immediately if rxBuffer is empty.
 * @params
 * 		buffer		Buffer to store read data
 * 		limit		Buffer length
 * @retval
 * 		Number of bytes successfully read
 */
int SerialPort::readAvailable(uint8_t *buffer, unsigned int limit){
	ClearCommError(this->handler, &this->errors, &this->status);	// Get serial status
	if(this->status.cbInQue==0) return 0;	// no data in rxBuffer so return immediately

	// If buffer is too small, read until buffer is filled.
	if(limit < this->status.cbInQue) limit=this->status.cbInQue;

	// Read from serial
	return this->read(buffer, limit);
}

/* Write a fixed number of bytes from the user buffer to the serial port.
 * Note that the user buffer should contain all the necessary bytes.
 * @params
 * 		buffer		Buffer containing data to send
 * 		nbytes		Number of bytes to write
 * @retval
 * 		Number of bytes successfully sent
 */
int SerialPort::write(uint8_t *buffer, unsigned int nbytes){
	if(!this->connected || this->handler==NULL) return 0;

	DWORD dwBytesSent;

	if (WriteFile(this->handler, (void*) buffer, nbytes, &dwBytesSent, NULL)){
		return (int)dwBytesSent;
	}

	// If code reaches here, write has failed.
	ClearCommError(this->handler, &this->errors, &this->status);
	return 0;
}