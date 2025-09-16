 /*
 * common.h
 *
 * 32-bit Motion Control Device Driver
 * Hardware-specific data structures for the Pmac.
 *
 *
 * This header describes the structures that are shared between
 * the hardware-specific user-mode code and the hardware-specific
 * kernel mode code.
 *

HISTORY

*/


#ifndef _COMMON_H_
  #define _COMMON_H_


#ifdef _OLD_DLL


#else
// PT_PMAC is the transition marker between PMAC and TURBO do not remove

typedef enum { 
  PT_PMAC1 = 1, PT_PMAC2, PT_PMACUL, PT_PMAC,  
  PT_PMAC1T, PT_PMAC2T, PT_PMACUT, PT_UMAC, PT_QMAC 
} PMACDEVICETYPE;
typedef enum {
  PID = 1, ESA, LHB
} PMACFIRMWARETYPE;
// Enumerates where the communications are based from for a PMAC device
typedef enum {LT_UNKNOWN, LT_ISA, LT_SER, LT_ETH, LT_PCI, LT_USB, LT_PCIE, LT_LAST} LOCATIONTYPE;
// Enumerates the different ways a PMAC can communicate
typedef enum { BUS, DPR } ASCIIMODE;

#endif

///////////////////////////////////////////////////////////////////////////
// Error defines for system
  #define SYS_FATAL_FERR    0X0001
  #define SYS_WARN_FERR     0X0002
  #define SYS_SERVO_ERR     0X0004
  #define SYS_REENTRY_ERR   0X0008
  #define SYS_MEM_CHKSUM    0X0010
  #define SYS_PROM_CHKSUM   0X0020
  #define SYS_RUNTIME_ERR   0X0040
  #define SYS_CIR_RAD_ERR   0X0080
  #define SYS_AMP_FAULT     0X0100
  #define SYS_RING_ERR      0X0200
  #define SYS_RING_IO_ERR   0X0400

// Error Defines for communications
  #define ERR_BAUDSEARCH    -27
  #define ERR_VBG_DPRNOTON    -28
// Variable background buffer defines
  #define ERR_VBG_MAXUSERS    -100 // Maximum number of users achieved in Variable Background Data Buffer
  #define ERR_VBG_MAXENTRIES  -101 // Maximum number of entries (>128) in Variable Background Data Buffer

// Polled Checksum communication defines
  #define CHECKSUMACKOK   -20   // good acknowlegde checksum
  #define CHECKSUMDATAOK    -21 // good data checksum
  #define CHECKSUMTIMEOUT   -22 // it times out
  #define CHECKSUMDATABAD   -23 // bad data checksum
  #define CHECKSUMACKBAD    -24 // bad command acknowledge checksum

/*
 * default settings for port, interrupt and dpram
 */

  #define DEF_PCBUSTYPE          0 // ISA, VME etc.
  #define DEF_TYPE               1 // PMAC 1
  #define DEF_PORT             528 // 0x210
  #define DEF_INTERRUPT          0 // No interrupt, make them select!
  #define DEF_FIFO             0x0
  #define DEF_LOCATION           0 // Computer BUS
  #define DEF_TIMEOUT          200
  #define DEF_FLUSHTIMEOUT      20
  #define DEF_DPRROT1ADR     0x400
  #define DEF_DPRROT2ADR     0x400
  #define DEF_DPRROT3ADR       0x0
  #define DEF_DPRROT4ADR       0x0
  #define DEF_DPRROT5ADR       0x0
  #define DEF_DPRROT6ADR       0x0
  #define DEF_DPRROT7ADR       0x0
  #define DEF_DPRROT8ADR       0x0
  #define DEF_DPRROT9ADR       0x0
  #define DEF_DPRROT10ADR       0x0
  #define DEF_DPRROT11ADR       0x0
  #define DEF_DPRROT12ADR       0x0
  #define DEF_DPRROT13ADR       0x0
  #define DEF_DPRROT14ADR       0x0
  #define DEF_DPRROT15ADR       0x0
  #define DEF_DPRROT16ADR       0x0
  #define DEF_DPRVARADR      0xD60
  #define DEF_DPRUSERBUF     0x580
  #define DEF_FBSTART       0x9f00
  #define DEF_FBTIMER       0x9f80
  #define DEF_BAUDRATE       38400
  #define DEF_COMPORT            0 // COM1
  #define DEF_PARITY             0 // No Parity
  #define DEF_RTS_CONTROL        1 // Enabled
  #define DEF_CTS_OUTPUT         1 // Checked
  #define DEF_VMEBASE     0x7FA000 // PMAC's Default VME base address
  #define DEF_VMEDPRBASE  0x700000 // PMAC's Default VME DPR Base Address
  #define DEF_VMEINTERRUPT       7 // PMAC's Default VME Interrupt Level
  #define DEF_VMEHOSTID          0 // VMIC 7686
  #define DEF_VMEAM           0x39 // Address Modifier
  #define DEF_VMEAMDC         0x04 // Address Modifier Dont care
  #define DEF_VMEIRQVECT      0xA1 // VME IRQ Vector
  #define DEF_BUSCHARTIMEOUT    1000
  #define DEF_BUSFLUSHTIMEOUT   10
  #define DEF_VMECHARTIMEOUT    30
  #define DEF_VMEFLUSHTIMEOUT    3
  #define DEF_DPBKTIMEOUT       10
  #define DEF_DPRTTIMEOUT        2
  #define DEF_DPCHARTIMEOUT   1000
  #define DEF_DPFLUSHTIMEOUT    20
  #define DEF_SERCHARTIMEOUT  2000
  #define DEF_SERFLUSHTIMEOUT   30
  #define DEF_SERWRITECONSTANTTIMEOUT 1000
  #define DEF_SERWRITEMULTIPLYERTIMEOUT   0
  #define DEF_SERREADCONSTANTTIMEOUT   30
  #define DEF_SERREADINTERVALTIMEOUT   30 // A VALUE OF 0 indicates interval timeouts are not used
  #define DEF_SERREADCONSTANTTIMEOUTSLOW   30
  #define DEF_SERREADINTERVALTIMEOUTSLOW   30 // A VALUE OF 0 indicates interval timeouts are not used
  #define DEF_SERREADMULTIPLYERTIMEOUT  0
  #define DEF_USBMODE                   0
  #define DEF_IPADDRESS          0xc0065e05

  #define DEF_DISABLEIRQ         1
  #define DEF_DISABLEDPRAM       1

  #define DEF_USENEWCOMMFEATURES 1 // Use New Firmware features by default
/*
* MEMORY Allocation Related macro,defines
*
*/

  #define MAX_DPRBUFFER       (160)
  #define MAX_DPRREADBUFFER   (256)   // this is the maximum string Turbo can return
  #define DPR_READ_COPY_BUFFER_SIZE (MAX_DPRREADBUFFER+4) // +4 for stack protection
/*
* Common defines
*
*/

  #define MAXMOTORS           8
  #define MAXMOTORSTURBO      32
  #define MAXLINKS            68               // max number of PLCC links
  #define MINLINKS            41
  #define MAXLDS              27               // max number of PLCC LADDER DIAG links
  #define MINLDS              27               // MIN number of PLCC LADDER DIAG links
  #define MAXSTRLINK          ((8*MAXLINKS)+1) // max string length of links
  #define VERSION_BUFFER_SIZE 100
  #define DATE_BUFFER_SIZE    100
  #define ISR_IPOS            1                /*         IR0 for in position          */
  #define ISR_BREQ            2                /*         IR1 for buffer request       */
  #define ISR_FERROR          4                /*         IR2 for general error        */
  #define ISR_ERROR           8                /*         IR3 for following error      */
  #define ISR_HREQ            16               /*         IR4 for communication        */
  #define ISR_IR5             32               /*         IR5                          */
  #define ISR_IR6             64               /*         IR6                          */
  #define ISR_IR7             128              /*         IR7                          */
  // Motor registers
  #define MTRSTATUS_X_OFS            0x3C // offset to each motor status X word
  #define MTRSTATUS_Y_OFS            0xC0 // offset to each motor status Y word
  #define MTRSTATUS_X_BASE         0x003D // address of mtr 1 first status word
  #define MTRSTATUS_X_BASE_ESA     0x004F // address of mtr 1 first status word
  #define MTRSTATUS_Y_BASE         0x0814 // address of mtr 1 second status word

  #define TURBO_MTRSTATUS_OFS        0x80 // offset to each motor status word
  #define TURBO_MTRSTATUS_X_BASE     0xB0 // address of mtr 1 first status word
  #define TURBO_MTRSTATUS_Y_BASE     0xC0 // address of mtr 1 second status word

/*
 * Parameter Names:
 *
 * These are the names of Values in the Parameters key (or driver section
 * of the profile) used for communicating configuration information and errors
 * between the kernel and user-mode drivers.
 */

  #define PARAM_TYPE        TEXT("PmacType")     // model of PMAC
  #define PARAM_PCBUSTYPE   TEXT("PCBusType")    // bus type isa, pci
  #define PARAM_PORT        TEXT("Port")         // port i/o address
  #define PARAM_INTERRUPT   TEXT("Interrupt")    // interrupt number
  #define PARAM_BUSCHARTIMEOUT  TEXT("BUSCharTimeout")
  #define PARAM_BUSFLUSHTIMEOUT TEXT("BUSFlushTimeout")
  #define PARAM_FIFO        TEXT("DualPortRam")  // DP Ram physical addr
  #define PARAM_ERROR       TEXT("InstallError") // config error/success code (below)
  #define PARAM_DPRROT1     TEXT("DualPortRamRot1")
  #define PARAM_DPRROT2     TEXT("DualPortRamRot2")
  #define PARAM_DPRROT3     TEXT("DualPortRamRot3")
  #define PARAM_DPRROT4     TEXT("DualPortRamRot4")
  #define PARAM_DPRROT5     TEXT("DualPortRamRot5")
  #define PARAM_DPRROT6     TEXT("DualPortRamRot6")
  #define PARAM_DPRROT7     TEXT("DualPortRamRot7")
  #define PARAM_DPRROT8     TEXT("DualPortRamRot8")
  #define PARAM_DPRROT9     TEXT("DualPortRamRot9")
  #define PARAM_DPRROT10     TEXT("DualPortRamRot10")
  #define PARAM_DPRROT11     TEXT("DualPortRamRot11")
  #define PARAM_DPRROT12     TEXT("DualPortRamRot12")
  #define PARAM_DPRROT13     TEXT("DualPortRamRot13")
  #define PARAM_DPRROT14     TEXT("DualPortRamRot14")
  #define PARAM_DPRROT15     TEXT("DualPortRamRot15")
  #define PARAM_DPRROT16     TEXT("DualPortRamRot16")
  #define PARAM_DPRVARADR   TEXT("DualPortRamVar")
  #define PARAM_DPRUSERSIZE TEXT("DualPortRamUserSize")
  #define PARAM_FBSTART     TEXT("FunctionBlockStart")
  #define PARAM_FBTIMER     TEXT("FunctionBlockTimer")
  #define PARAM_PORTNUMBER  TEXT("SerialPortNumber")
  #define PARAM_BAUDRATE    TEXT("SerialBaudrate")
  #define PARAM_PARITY      TEXT("SerialParity")
  #define PARAM_CTS         TEXT("SerialCTSOutput")
  #define PARAM_RTS         TEXT("SerialRTSControl")
  #define PARAM_LOCATION    TEXT("Location")
  #define PARAM_ENUMERATION TEXT("Enumeration")
  #define PARAM_DISABLEDPR  TEXT("DisableDPRAM")
  #define PARAM_DISABLEIRQ  TEXT("DisableIRQ")
  #define PARAM_PCI_SLOT    TEXT("PCISlot")
  #define PARAM_PCI_BUS     TEXT("PCIBus")
  #define PARAM_IPADDRESS   TEXT("IPAddress")
  #define PARAM_USENEWCOMMFEATURES  TEXT("UseNewCommFeatures")
  #define PARAM_USBMODE     TEXT("USBMode")
  #define PARAM_DPRAUTOACTIVE TEXT("DPRAutoActive")
  #define PARAM_DPRRTACTIVE TEXT("DPRRealTimeActive")
  #define PARAM_DPRBGACTIVE TEXT("DPRBackgroundActive")
  #define PARAM_DPRBGPERIOD TEXT("DPRBackgroundPeriod")
  #define PARAM_DPRRTPERIOD TEXT("DPRRealTimePeriod")
  #define PARAM_DPRRTMTRMASK TEXT("DPRRealTimeMtrMask")
  #define PARAM_DPRBGCOORDSYS TEXT("DPRBackgroundCrdSys")
  #define PARAM_DPRNUMMOTORS TEXT("DPRNumMotors")
  #define PARAM_DPRMONITORPERIOD TEXT("DPRMonitorPeriod")
  #define PARAM_ENABLEINTR  TEXT("EnableInterrupts")

  #define PARAM_DPBKTIMEOUT    TEXT("DPRBackTimeout")
  #define PARAM_DPRTTIMEOUT    TEXT("DPRRTTimeout")
  #define PARAM_DPCHARTIMEOUT  TEXT("DPRCharTimeout")
  #define PARAM_DPFLUSHTIMEOUT TEXT("DPRFlushTimeout")

  #define PARAM_VMEBASE         TEXT("VMEBase")
  #define PARAM_VMEDPRBASE      TEXT("VMEDPRBase")
  #define PARAM_VMEINTERRUPT    TEXT("VMEInterrupt")
  #define PARAM_VMEHOSTID       TEXT("VMEHostID")
  #define PARAM_VMEAM           TEXT("VMEAM")
  #define PARAM_VMEAMDC         TEXT("VMEAMDontCare")
  #define PARAM_VMEIRQVECT      TEXT("VMEIRQVector")
  #define PARAM_VMECHARTIMEOUT  TEXT("VMECharTimeout")
  #define PARAM_VMEFLUSHTIMEOUT TEXT("VMEFlushTimeout")
  #define PARAM_SERWRITEMULTIPLYERTIMEOUT  TEXT("SerWriteMultiplyerTimeout")
  #define PARAM_SERWRITECONSTANTTIMEOUT    TEXT("SerWriteConstantTimeout")
  #define PARAM_SERREADMULTIPLYERTIMEOUT   TEXT("SerReadMultiplyerTimeout")
  #define PARAM_SERREADCONSTANTTIMEOUT     TEXT("SerReadConstantTimeout")
  #define PARAM_SERREADINTERVALTIMEOUT     TEXT("SerReadIntervalTimeout")
  #define PARAM_SERCHARTIMEOUT  TEXT("SerCharTimeout")
  #define PARAM_SERFLUSHTIMEOUT TEXT("SerFlushTimeout")

  #ifdef _NC
    #define PARAM_NCTITLE         TEXT("Title")
    #define PARAM_SOURCEPROFILE   TEXT("SourceProfile")
    #define PARAM_TOOLPROFILE     TEXT("ToolProfile")
    #define PARAM_COORDPROFILE    TEXT("CoordProfile")
    #define PARAM_MACHINETYPE     TEXT("MachineType")
    #define PARAM_NOOFTOOLS       TEXT("NoOfTools")
    #define PARAM_NOOFBLOCKS      TEXT("NoOfBlocks")
    #define PARAM_NOOFCOORDSYS    TEXT("NoOfCoordSys")
    #define PARAM_METRICDISPLAY   TEXT("MetricDisplay")
    #define PARAM_LEASTHANDLEINC  TEXT("LeastHandleInc")
    #define PARAM_MAXHANDLEINC    TEXT("MaxHandleInc")
    #define PARAM_LEASTJOGINC     TEXT("LeastJogInc")
    #define PARAM_AXISMOTORMAP    TEXT("AxisMotorMap")
    #define PARAM_AXISMOTORSEL    TEXT("AxisMotorSel")
    #define PARAM_AXISDISPMAP     TEXT("AxisDispMap")
    #define PARAM_MAXRAPIDOVRD    TEXT("MaxRapidOvrd")
    #define PARAM_MAXFEEDOVRD     TEXT("MaxFeedOvrd")
// Axis stuff
    #define PARAM_ISSPINDLE       TEXT("IsSpindle")
    #define PARAM_HASSLAVE        TEXT("HasSlave")
    #define PARAM_ISPHANTOM       TEXT("IsPhantom")
    #define PARAM_DISPLAY         TEXT("Display")
    #define PARAM_DISPLAYSLAVE    TEXT("DisplaySlave")
    #define PARAM_HOMEMODE        TEXT("HomeMode")
    #define PARAM_HOMEPRGNUMBER   TEXT("HomePrgNumber")
    #define PARAM_PRECISION       TEXT("Precision")
    #define PARAM_METRICDISPLAY   TEXT("MetricDisplay")
    #define PARAM_METRICUNITS     TEXT("MetricUnits")
    #define PARAM_PROBEPRGNUMBER  TEXT("ProbePrgNumber")
    #define PARAM_PULSEPERUNIT    TEXT("PulsePerInit")
    #define PARAM_INPOSITIONBAND  TEXT("InpositionBand")
    #define PARAM_MAXRAPID        TEXT("MaxRapid")
    #define PARAM_MAXFEED         TEXT("MaxFeed")
    #define PARAM_FATALFERROR     TEXT("FatalFError")
    #define PARAM_WARNFERROR      TEXT("WarnFError")
    #define PARAM_JOGSPEEDLOW     TEXT("JogSpeedLow")
    #define PARAM_JOGSPEEDMedLow  TEXT("JogSpeedMedLow")
    #define PARAM_JOGSPEEDMed     TEXT("JogSpeedMed")
    #define PARAM_JOGSPEEDMedHigh TEXT("JogSpeedMedHigh")
    #define PARAM_JOGSPEEDHigh    TEXT("JogSpeedHigh")
    #define PARAM_FORMATINCH      TEXT("FormatInch")
    #define PARAM_FORMATMM        TEXT("FormatMM")
  #endif

/*
 * Error handling
 *
 * during startup of the kernel-driver, the PARAM_ERROR value is written with
 * one of the values below. These are the IDs of strings in
 * pmac\dll\pmac.rc that are produced in a dialog box by the user-mode
 * driver during configuration if not MC_ERR_OK
 */

  #define ERR_OK           0    // no configuration error
  #define ERR_CREATEDEVICE 1001 // failed to create device object
  #define ERR_CONFLICT     1002 // resource conflict
  #define ERR_DETECTFAILED 1003 // could not find hardware
  #define ERR_INTERRUPT    1004 // interrupt did not install
  #define ERR_INTRDETECT   1005 // interrupt did not occur
  #define ERR_BUSTIMEOUT   1006 // timeout during bus read
  #define ERR_DPRTIMEOUT   1007 // timeout during dpram ascii read
  #define ERR_SERTIMEOUT   1008 // timeout during serial ascii rea
  #define ERR_INBOOTSTRAP  1009 // PMAC in bootstrap mode can't load
  #define ERR_CONFIGDRIVER 1010 // Unable to configure driver (in registry perhaps)
  #define ERR_EMPTYBUS     1011 // Continuous characters from bus, No Card!!
  #define ERR_EMPTYDPRAM   1012 // Continuous characters from bus, No Card!!


#endif //_COMMON_H_

