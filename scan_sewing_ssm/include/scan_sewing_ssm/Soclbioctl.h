//----------------------------------------------------------------------------
//  Project SOCLB.SYS
//  Delta Tau Data Systems, Inc.
//  Copyright © 2000. All Rights Reserved.
//
//  SUBSYSTEM:    PMAC USB WDM Driver
//  FILE:         Soclbioctl.h
//  AUTHOR:       Henry Bausley
//
//  OVERVIEW
//  ~~~~~~~~
//  Contains device i/o control codes compatable with Anchor chips control 
//  panel program and codes for getting and setting data to the SOCLB
//----------------------------------------------------------------------------
#include <windef.h>

#ifndef SOCLBIOCTL_H
#define SOCLBIOCTL_H

//////////////////////////////////////////////////////////////////////////////
// These are the PMAC.SYS IOCTL Control calls
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef CTL_CODE
  #define CTL_CODE( DeviceType, Function, Method, Access ) (                 \
      ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) \
       )
#define FILE_ANY_ACCESS  0
#define FILE_DEVICE_SOUND               0x0000001d
#define FILE_DEVICE_UNKNOWN             0x00000022
#define METHOD_BUFFERED                 0
#define METHOD_IN_DIRECT                1
#define METHOD_OUT_DIRECT               2
#define METHOD_NEITHER                  3
#define SOCLB_IN_DPROFFSET              0x3F40  // 0x0FD0
#define SOCLB_OUT_DPROFFSET             0x3F60  // 0x0FD8

#endif

// Generic Board I/O count
#define OUTPUTCOUNT           8
#define INPUTCOUNT            8
#define MACROIOCOUNT          3
#define AD7822CHANNELS        4  // Number of channels on A/D


typedef struct tagSOCLBIn
{
  BYTE LocalInput[INPUTCOUNT];
  BYTE MacroInput[MACROIOCOUNT];
  BYTE pad0;                          // Allows the data to be aligned on 32 bit boundaries
  BYTE EncoderCounter[MACROIOCOUNT];  
  BYTE pad1;                          // Allows the data to be aligned on 32 bit boundaries
  BYTE AD7822Data[AD7822CHANNELS]; 
} SOCLBIN,*PSOCLBIN;

typedef struct tagSOCLBInDevice
{
  BYTE LocalInput[INPUTCOUNT];
  BYTE MacroInput[MACROIOCOUNT];
  BYTE EncoderCounter[MACROIOCOUNT];  
  BYTE AD7822Data[AD7822CHANNELS]; 
} DEVSOCLBIN,*PDEVSOCLBIN;


///////////////////////////////////////////////////////
//
//              IOCTL Definitions
//
// User mode applications wishing to send IOCTLs to a kernel mode driver
// must use this file to set up the correct type of IOCTL code permissions.
//
// Note: this file depends on the file DEVIOCTL.H which contains the macro
// definition for "CTL_CODE" below.  Include that file before  you include
// this one in your source code.
//
///////////////////////////////////////////////////////

//
// Set the base of the IOCTL control codes.  This is somewhat of an
// arbitrary base number, so you can change this if you want unique
// IOCTL codes.  You should consult the Windows NT DDK for valid ranges
// of IOCTL index codes before you choose a base index number.
//

#define Ezusb_IOCTL_INDEX  0x0800


#define IOCTL_Ezusb_GET_PIPE_INFO     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+0,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_GET_DEVICE_DESCRIPTOR CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+1,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_GET_CONFIGURATION_DESCRIPTOR CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+2,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_BULK_OR_INTERRUPT_WRITE     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+3,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_BULK_OR_INTERRUPT_READ      CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+4,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_VENDOR_REQUEST              CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+5,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_GET_CURRENT_CONFIG          CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+6,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_ANCHOR_DOWNLOAD             CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+7,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_RESET  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+12,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Soclb_ResetPipe  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+13,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Soclb_AbortPipe  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+15,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_SETINTERFACE  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+16,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_Ezusb_GET_STRING_DESCRIPTOR CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+17,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)


//
// Perform an IN transfer over the specified bulk or interrupt pipe.
//
// lpInBuffer: BULK_TRANSFER_CONTROL stucture specifying the pipe number to read from
// nInBufferSize: sizeof(BULK_TRANSFER_CONTROL)
// lpOutBuffer: Buffer to hold data read from the device.  
// nOutputBufferSize: size of lpOutBuffer.  This parameter determines
//    the size of the USB transfer.
// lpBytesReturned: actual number of bytes read
// 
#define IOCTL_EZUSB_BULK_READ             CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+19,\
                                                   METHOD_OUT_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// Perform an OUT transfer over the specified bulk or interrupt pipe.
//
// lpInBuffer: BULK_TRANSFER_CONTROL stucture specifying the pipe number to write to
// nInBufferSize: sizeof(BULK_TRANSFER_CONTROL)
// lpOutBuffer: Buffer of data to write to the device
// nOutputBufferSize: size of lpOutBuffer.  This parameter determines
//    the size of the USB transfer.
// lpBytesReturned: actual number of bytes written
// 
#define IOCTL_EZUSB_BULK_WRITE            CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+20,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// The following IOCTL's are defined as using METHOD_DIRECT_IN buffering.
// This means that the output buffer is directly mapped into system
// space and probed for read access by the driver.  This means that it is
// brought into memory if it happens to be paged out to disk.  Even though
// the buffer is only probed for read access, it is safe (probably) to
// write to it as well.  This read/write capability is used for the loopback
// IOCTL's
// 

// TODO Insert Loopback IOCTL's

//
// Retrieve the current USB frame number from the Host Controller
//
// lpInBuffer: NULL
// nInBufferSize: 0
// lpOutBuffer: PULONG to hold current frame number
// nOutputBufferSize: sizeof(PULONG)
// 
#define IOCTL_EZUSB_GET_CURRENT_FRAME_NUMBER  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+21,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)


//
// Performs a vendor or class specific control transfer to EP0.  The contents of
// the input parameter determine the type of request.  See the USB spec
// for more information on class and vendor control transfers.
//
// lpInBuffer: PVENDOR_OR_CLASS_REQUEST_CONTROL
// nInBufferSize: sizeof(VENDOR_OR_CLASS_REQUEST_CONTROL)
// lpOutBuffer: pointer to a buffer if the request involves a data transfer
// nOutputBufferSize: size of the transfer buffer (corresponds to the wLength
//    field of the USB setup packet)
// 
#define IOCTL_EZUSB_VENDOR_OR_CLASS_REQUEST   CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+22,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// Retrieves the actual USBD_STATUS code for the most recently failed
// URB.
//
// lpInBuffer: NULL
// nInBufferSize: 0
// lpOutBuffer: PULONG to hold the URB status
// nOutputBufferSize: sizeof(ULONG)
// 

#define IOCTL_EZUSB_GET_LAST_ERROR   CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+23,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

//
// Reads from the specified ISO endpoint. (USB IN Transfer)
//
// lpInBuffer: ISO_TRANSFER_CONTROL
// nInBufferSize: sizeof(ISO_TRANSFER_CONTROL)
// lpOutBuffer: buffer to hold data read from the device
// nOutputBufferSize: size of the read buffer.
//
// 
// 

#define IOCTL_EZUSB_ISO_READ          CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+25,\
                                                   METHOD_OUT_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// Writes to the specified ISO endpoint. (USB OUT Transfer)
//
// lpInBuffer: ISO_TRANSFER_CONTROL
// nInBufferSize: sizeof(ISO_TRANSFER_CONTROL)
// lpOutBuffer: buffer to hold data to write to the device
// nOutputBufferSize: size of the write buffer.
//
// 
// 

#define IOCTL_EZUSB_ISO_WRITE          CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+26,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// Performs and Anchor Download.
//
// lpInBuffer: PANCHOR_DOWNLOAD_CONTROL
// nInBufferSize: sizeof(ANCHOR_DOWNLOAD_CONTROL)
// lpOutBuffer: pointer to a buffer of data to download to the device
// nOutputBufferSize: size of the transfer buffer
// 
#define IOCTL_EZUSB_ANCHOR_DOWNLOAD   CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+27,\
                                                   METHOD_IN_DIRECT,  \
                                                   FILE_ANY_ACCESS)

//
// Returns driver version information
//
// lpInBuffer: NULL
// nInBufferSize: 0
// lpOutBuffer: PEZUSB_DRIVER_VERSION
// nOutputBufferSize: sizeof(EZUSB_DRIVER_VERSION)
// 
#define IOCTL_EZUSB_GET_DRIVER_VERSION   CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+29,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_EZUSB_START_ISO_STREAM     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+30,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_EZUSB_STOP_ISO_STREAM     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+31,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_EZUSB_READ_ISO_BUFFER     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+32,\
                                                   METHOD_OUT_DIRECT,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_GET_DATA            CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+50,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_SET_DATA            CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+51,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_RESET_ENCODER      CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+52,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_READWRITE_DATA     CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                                                   Ezusb_IOCTL_INDEX+53,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_GET_DATA_BULK       CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+54,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_SET_DATA_BULK       CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+55,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#define IOCTL_SOCLB_READWRITE_DATA_BULK CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                                                   Ezusb_IOCTL_INDEX+56,\
                                                   METHOD_BUFFERED,  \
                                                   FILE_ANY_ACCESS)

#endif