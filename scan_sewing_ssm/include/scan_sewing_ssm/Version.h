//-----------------------------------------------------------------------------
// Delta Tau Product Version Information
//
// VERSION.H
// ---------
//
// Modify this file when making new releases.  xxxx_major_ver,xxxx_minor_ver
// xxxx_release,xxxx_build.  If nothing in a project has changed DO NOT up the
// version number.  For instance if Ncui32 is rebuilt and PMAC.SYS and PMAC.VXD
// have not changed DO NOT increase their version numbers.
//
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// General Version information applicable to all Delta Tau Products
//-----------------------------------------------------------------------------
#define szCopyRight    "Copyright © 1994-99. Delta Tau Data Systems, Inc.\000"
#define szCompanyName "Delta Tau Data Systems, Inc.\000"
// This define (version_special) represents a SPECIAL BUILD.
// If it is any value other than zero (0) then the products will be marked
// as such.
// This marking will be availble in about boxes or resource properties.
#ifdef _DEBUG
  #define version_special 1
  #define szSpecialName "Development\000"
#else
  #define version_special 0
  #define szSpecialName "\000"
#endif


//-----------------------------------------------------------------------------
// NC 32 bit Product Version Info
//
// NCUI32.EXE
// ----------
//
//-----------------------------------------------------------------------------
#define szNcui32Description     "PMAC-NC 32-bit for Windows\000"
#define ncui32_major_ver        4    // use for major changes
#define ncui32_minor_ver        0    // use for feature changes
#define ncui32_release          0    // use for feature changes
#define ncui32_build            3    // use for feature changes
#define ncui32prod_major_ver    4    // use for major changes
#define ncui32prod_minor_ver    0   // use for feature changes
#define ncui32prod_release      0    // use for feature changes
#define ncui32prod_build        3    // use for feature changes
#define szNcui32FileVerName     "4.0.0.3\000"
#define szNcui32InternalName    "NCUI32\000"
#define szNcui32LegalTradeMarks "PMAC Trademark\000"
#define szNcui32OriginalName    "NCUI32.EXE\000"
#define szNcui32ProdName        "PMAC-NC 32\000"
#define szNcui32ProdVerName     "4.0.0.3\000"
#ifdef _DEBUG
#define szNcui32Comment         "Internal Development version NOT FOR GENERAL RELEASE\000"
#else
#define szNcui32Comment         "Official Release\000"
#endif


//-----------------------------------------------------------------------------
// PMAC.DLL 32 bit Product Version Info
//
// PCOMM32.DLL
// --------
//
//-----------------------------------------------------------------------------
#define szPmac32Description     "PMAC Win32 Communications Library000\000"
#define pmac32_major_ver        10   // use for major changes
#define pmac32_minor_ver        41   // use for feature changes
#define pmac32_release          0    // use for feature changes
#define pmac32_build            0    // use for feature changes
#define pmac32prod_major_ver    10   // use for major changes
#define pmac32prod_minor_ver    41   // use for feature changes
#define pmac32prod_release      0    // use for feature changes
#define pmac32prod_build        0    // use for feature changes
#define szPmac32FileVerName     "10.41.0.0\000"
#define szPmac32InternalName    "PCOMM32PRO\000"
#define szPmac32LegalTradeMarks "\000"
#define szPmac32OriginalName    "PComm32.dll\000"
#define szPmac32ProdName        "PCOMM32 Communications Library\000"
#define szPmac32ProdVerName     "10.41.0.0\000"
#ifdef _DEBUG
#define szPmac32Comment         "Internal Development version NOT FOR GENERAL RELEASE\000"
#else
#define szPmac32Comment         "Official Release\000"
#endif


//-----------------------------------------------------------------------------
// PEWIN 32 bit Product Version Info
//
// PEWIN.EXE
// ---------
//
//-----------------------------------------------------------------------------
#define szPewin32Description     "PMAC 32 Bit Executive for Windows\000"
#define pewin32_major_ver        2    // use for major changes
#define pewin32_minor_ver        39   // use for feature changes
#define pewin32_release          0    // use for feature changes
#define pewin32_build            0    // use for feature changes
#define pewin32prod_major_ver    2    // use for major changes
#define pewin32prod_minor_ver    39   // use for feature changes
#define pewin32prod_release      0    // use for feature changes
#define pewin32prod_build        0    // use for feature changes
#define szPewin32FileVerName     "2.39.0.0\000"
#define szPewin32InternalName    "PEWIN\000"
#define szPewin32LegalTradeMarks "PMAC Trademark\000"
#define szPewin32OriginalName    "PEWIN32.EXE\000"
#define szPewin32ProdName        "PMAC-NC 32\000"
#define szPewin32ProdVerName     "2.39.0.0\000"
#ifdef _DEBUG
#define szPewin32Comment         "Internal Development version NOT FOR GENERAL RELEASE\000"
#else
#define szPewin32Comment         "Official Release\000"
#endif



//-----------------------------------------------------------------------------
// PMAC Windows-NT Driver
//
// PMAC.SYS
// --------
//
//-----------------------------------------------------------------------------
#define szpmacSysDescription     "PMAC 32 Bit Kernel Mode Driver for Windows-NT\000"
#define pmacSys_major_ver        2    // use for major changes
#define pmacSys_minor_ver        0    // use for feature changes
#define pmacSys_release          0    // use for feature changes
#define pmacSys_build            2    // use for feature changes
#define pmacSysprod_major_ver    2    // use for major changes
#define pmacSysprod_minor_ver    0    // use for feature changes
#define pmacSysprod_release      0    // use for feature changes
#define pmacSysprod_build        2    // use for feature changes
#define szpmacSysFileVerName     "2.0.0.2\000"
#define szpmacSysInternalName    "PMAC\000"
#define szpmacSysLegalTradeMarks "PMAC Trademark\000"
#define szpmacSysOriginalName    "PMAC.SYS\000"
#define szpmacSysProdName        "PMAC-NT-Driver\000"
#define szpmacSysProdVerName     "2.0.0.2\000"
#ifdef _DEBUG
#define szpmacSysComment         "Internal Development version NOT FOR GENERAL RELEASE\000"
#else
#define szpmacSysComment         "Special Interupt Event  NOT FOR GENERAL RELEASE\000"
#endif


//-----------------------------------------------------------------------------
// PMAC Windows-95/98 Driver
//
// PMAC.VXD
// --------
//
//-----------------------------------------------------------------------------
#define szpmacVxdDescription     "PMAC 32 Bit Virtual Driver for Windows95/98\000"
#define pmacVxd_major_ver        2    // use for major changes
#define pmacVxd_minor_ver        0    // use for feature changes
#define pmacVxd_release          0    // use for feature changes
#define pmacVxd_build            1    // use for feature changes
#define pmacVxdprod_major_ver    2    // use for major changes
#define pmacVxdprod_minor_ver    0    // use for feature changes
#define pmacVxdprod_release      0    // use for feature changes
#define pmacVxdprod_build        1    // use for feature changes
#define szpmacVxdFileVerName     "2.0.0.1\000"
#define szpmacVxdInternalName    "PMAC\000"
#define szpmacVxdLegalTradeMarks "PMAC Trademark\000"
#define szpmacVxdOriginalName    "PMAC.VXD\000"
#define szpmacVxdProdName        "PMAC-Virtual Device Driver\000"
#define szpmacVxdProdVerName     "2.0.0.1\000"
#ifdef _DEBUG
#define szpmacVxdComment         "Internal Development version NOT FOR GENERAL RELEASE\000"
#else
#define szpmacVxdComment         "Official Release\000"
#endif
