/* address.h

         Header file for common defines used by both PMAC
    and PC programs. This file is used by the DLL and LIB
    librarys when compiled.

    Warning: This file may change at any new library compilation.

        !!!!!! Change this file at your own risk. !!!!!!

///////////////////////// Naming Conventions //////////////////////////////

1. At beginning of macro name:

  A. VARIABLE

    CS  = DPRAM Command/Status related value matching that used by DLL.
    ES  = DPRAM Error/Status related value matching that used by DLL.
    PC  = PC DPRAM related value used by DLL.
    PM  = PMAC DPRAM related value used by pmac programs.
    VS  = DPRAM Value/Status related value matching that used by DLL.

DATE    BY  CHANGES
------- --- ----------------------------------------------------------
07Oct99 JET Added PMAC_ROT_BUF_FIL_STOP16, we still want single cycle access
            on pmac side, with 24 bit def. But the pc only fills lower 16 bits
            and afterwards the upper 8 have garbage!! so when reading on PMAC
            side we need to mask out the upper 8. 
20Jul99 JET Added USER_BAR definitions.
18May99 JET Added DPR_POINTER etc, to Turbo.
23Mar99 JET Added VS_L_NLABEL_M and VS_L_NLABEL2_M for long N codes.
16Feb99 JET It is OK to use 0,24 on ROT_BUF_FIL_STOP, because PMAC never
            uses it, but only writes to it. So we take advantage of the
            one ASM op rule of 0,24. 
15Feb99 JET changes of 21Jan99 back to 0,16 because pmac can only read
            16 bits of DPR and the upper 8 bits are random!!!
            so we have no choice, but to use read-modify-write method.
21Jan99 JET use 0,24 instead 0,16 in PMAC_RESET_COMP_ADR,PC_RESET_COMP_ADR,
            ROT_BUF_FIL_STOP_ADR, and UNUSED_13_ADR so that PMAC operates
            on values in on ASM operation.
19Jan99 JET added PMAC_ROT_BUF_FIL_STOP, solves bit twiddle conflict in DWORD6.
03Dec98 JET added PMAC_RESET_COMPLETE, PLC_RESET_BITS and PC_RESET_COMPLETE
21Aug98 JET reserved G65_LEVEL2, RT_BLOCK addresses in DPR.
14Oct97 JET change position of CS_ROT_BUF_FIL_STOP due to CS_BUFFER_HOLD
27AUG97 JET Added Q127 as VS_SPINDLE_RPM_Q
06JUN97 JET probing definitions
14May97 jet added CS_ROT_BUF_FIL_STOP for G31 and offset updates
04/26/4 ED LAY  Added comments for user information
**************************************************************************/

#ifndef ADDRESS_H
  #define ADDRESS_H

  #define TRUE    1
  #define FALSE   0
  #define ON      TRUE
  #define OFF     FALSE

///////////////////////////////////////////////////////////////////////////
// Command / Control / Status BIT Mapping
// Command/Status Area Reserves M0..M15 for 32 bit words
// The selectors reserve M60..M99
///////////////////////////////////////////////////////////////////////////
  #define PC_STATUS_OFS       0x3780
  #define PC_COMMAND_OFS      0x37C0

// Command/Status (CS) #1 32 bits /////////////////////////////////////////
// This 32 bit word makes up the 8 selector switch inputs for Delta Tau's
// Control panel.  Normally these are put into the Dual Port Ram by the
// MAIN.PLC into the CS_COMMAND1_M word.  The DLL operates on these inputs
// and then writes its operating values to the CS_STATUS1_M word.  In certain
// cases where the DLL is not allowing the requested COMMAND the STATUS will
// not match the COMMAND.  In the case where a PLC is not making a request
// but needing to inform the DLL of a status change, the PLC should do this
// via the STATUS word.
// The handwheel panel input is generally inputed and filtered in the
// MAIN.PLC and the motor master handwheel address ( Ix05 ) is then
// pointed to this address.  Since the handwheel input is directly operated
// on by the PMAC, this input is not passed thruough DPRAM to the DLL.
//

  #define PC_STATUS1_M        0
  #define CS_STATUS1_M        M0
  #define PC_COMMAND1_M       8
  #define CS_COMMAND1_M       M8


// The following are the individual switch selectors for the Delta Tau
// Control panel.  They make up the above 32 bit inputs to the CNC DLL
//  ( 8 switches x 4 bits = 32 bits ).

  #define S_JOG_M             M60
  #define C_JOG_M             M80
  #define SEL_JOG_MASK        15 // $0000000F
  #define SEL_JOG_HANDLE      1  // $00000001
  #define SEL_JOG_INC         2  // $00000002
  #define SEL_JOG_CONT        3  // $00000003
  #define SEL_JOG_HOME        4  // $00000004

  #define S_MODE_M            M61
  #define C_MODE_M            M81
  #define SEL_MODE_MASK       240 // $000000F0
  #define SEL_MODE_AUTO       1   // $00000010
  #define SEL_MODE_MANUAL     2   // $00000020
  #define SEL_MODE_MDI        3   // $00000030

  #define S_AXIS_M            M62
  #define C_AXIS_M            M82
  #define SEL_AXIS_MASK       3840 // $00000F00
  #define SEL_AXIS_X          1    // $00000100
  #define SEL_AXIS_Y          2    // $00000200
  #define SEL_AXIS_Z          3    // $00000300
  #define SEL_AXIS_A          4    // $00000400
  #define SEL_AXIS_B          5    // $00000500
  #define SEL_AXIS_C          6    // $00000600
  #define SEL_AXIS_U          7    // $00000700
  #define SEL_AXIS_V          8    // $00000800
  #define SEL_AXIS_W          9    // $00000900

  #define S_FEED_M            M63
  #define C_FEED_M            M83
  #define SEL_FEED_MASK       61440 // $0000F000
  #define SEL_FEED_1          1     // $00001000
  #define SEL_FEED_10         2     // $00002000
  #define SEL_FEED_100        3     // $00003000
  #define SEL_FEED_1000       4     // $00004000
  #define SEL_FEED_10000      5     // $00005000

  #define S_ROVRD_M           M64
  #define C_ROVRD_M           M84
  #define SEL_ROVRD_MASK      983040 // $000F0000
  #define SEL_ROVRD_FEED      1      // $00010000
  #define SEL_ROVRD_25        2      // $00020000
  #define SEL_ROVRD_50        3      // $00030000
  #define SEL_ROVRD_100       4      // $00040000

  #define PC_SPND_M           65
  #define S_SPND_M            M65
  #define C_SPND_M            M85
  #define SEL_SPND_MASK       15728640 // $00F00000
  #define SEL_SPND_CCW        1        // $00100000
  #define SEL_SPND_OFF        2        // $00200000
  #define SEL_SPND_CW         3        // $00300000
  #define SEL_SPND_ORIENT1    4        // $00400000
  #define SEL_SPND_ORIENT2    5        // $00500000
  #define SEL_SPND_LOCK       6        // $00600000

  #define S_FOVRD_M           M66
  #define C_FOVRD_M           M86
  #define SEL_FOVRD_MASK      51658240 // $0F000000
  #define SEL_FOVRD_0         1        // $01000000
  #define SEL_FOVRD_12        2        // $02000000
  #define SEL_FOVRD_25        3        // $03000000
  #define SEL_FOVRD_37        4        // $04000000
  #define SEL_FOVRD_50        5        // $05000000
  #define SEL_FOVRD_62        6        // $06000000
  #define SEL_FOVRD_75        7        // $07000000
  #define SEL_FOVRD_87        8        // $08000000
  #define SEL_FOVRD_100       9        // $09000000
  #define SEL_FOVRD_112       10       // $0A000000
  #define SEL_FOVRD_125       11       // $0B000000
  #define SEL_FOVRD_137       12       // $0C000000
  #define SEL_FOVRD_150       13       // $0D000000
  #define SEL_FOVRD_187       14       // $0E000000
  #define SEL_FOVRD_200       15       // $0F000000

// More Selectors
  #define S_SOVRD_M           M67
  #define C_SOVRD_M           M87
  #define SEL_SOVRD_MASK      4026531840 // $F0000000
  #define SEL_SOVRD_50        1          // $10000000
  #define SEL_SOVRD_55        2          // $20000000
  #define SEL_SOVRD_60        3          // $30000000
  #define SEL_SOVRD_65        4          // $40000000
  #define SEL_SOVRD_70        5          // $50000000
  #define SEL_SOVRD_75        6          // $60000000
  #define SEL_SOVRD_80        7          // $70000000
  #define SEL_SOVRD_85        8          // $80000000
  #define SEL_SOVRD_90        9          // $90000000
  #define SEL_SOVRD_95        10         // $A0000000
  #define SEL_SOVRD_100       11         // $B0000000
  #define SEL_SOVRD_105       12         // $C0000000
  #define SEL_SOVRD_110       13         // $D0000000
  #define SEL_SOVRD_115       14         // $E0000000
  #define SEL_SOVRD_120       15         // $F0000000

// Command/Status (CS) #2 32 bits ////////////////////////////////////////////
// This 32 bit word make up the 2nd Command/Status interface between PMAC
// PLC(s) and the CNC DLL.  Their use is defined in each selector
// In some instances the request ( COMMAND ) could come from a different
// source than the PMAC Input.  For example it may it may come from the PC
// key board inputs or the CNC panel emulation input.  The CNC DLL would
// then be the initiator of the request.
// The STATUS value would then be set by the CNC DLL and should be used by
// the PLC that uses this information.

  #define PC_STATUS2_M        1
  #define CS_STATUS2_M        M1
  #define PC_COMMAND2_M       9
  #define CS_COMMAND2_M       M9

  #define S_COORD_M           M68
  #define C_COORD_M           M88
  #define SEL_COORD_MASK      15 // $0000000F

  #define S_GEAR_M            M69
  #define C_GEAR_M            M89
  #define SEL_GEAR_MASK       240 // $000000F0

// The HOME.PLC uses the status output of the DLL S_HOME_M for selecting
// which motor is to be homed.  The format is a follows 1 = Mtr #1,
// 2 = Mtr #2, 4 = Mtr #3,  .. 128 = Mtr #8.

  #define PC_HOME_M           70
  #define S_HOME_M            M70
  #define C_HOME_M            M90
  #define SEL_HOME_MTR_MASK   65280 // $0000FF00

  #define S_ERROR_M           M71
  #define C_ERROR_M           M91
  #define SEL_ERROR_MASK      983040 // $000F0000

  #define PC_CLNT_M           72
  #define S_CLNT_M            M72
  #define C_CLNT_M            M92 
  #define SEL_CLNT_MASK       15728640 // $00F00000
  #define SEL_CLNT_OFF        1        // $00100000
  #define SEL_CLNT_AUTO       2        // $00200000
  #define SEL_CLNT_FLOOD      3        // $00300000
  #define SEL_CLNT_MIST       4        // $00400000
  #define SEL_CLNT_FM         5        // $00500000

  #define S_DIST_M            M73
  #define C_DIST_M            M93
  #define SEL_DIST_MASK       51658240 // $0F000000
  #define SEL_DIST_1          1        // $01000000
  #define SEL_DIST_10         2        // $02000000
  #define SEL_DIST_100        3        // $03000000
  #define SEL_DIST_1000       4        // $04000000
  #define SEL_DIST_10000      5        // $05000000

  #define S_MACHINE_M         M74
  #define C_MACHINE_M         M94
  #define SEL_MACHINE_MASK    4026531840 // $F0000000
  #define SEL_MACHINE_MILL    1          // $10000000
  #define SEL_MACHINE_LATHE   2          // $20000000
  #define SEL_MACHINE_GRINDER 3          // $30000000
  #define SEL_MACHINE_LASER   4          // $40000000
  #define SEL_MACHINE_EDM     5          // $50000000

// Command/Status (CS) #3 32 bits ////////////////////////////////////////////
  #define PC_STATUS3_M        2
  #define CS_STATUS3_M       M2
  #define PC_COMMAND3_M      10
  #define CS_COMMAND3_M      M10

// Singles
  #define CS_MACHINE_LOCK      1          // $00000001
  #define CS_RESET            2           // $00000002
  #define CS_SINGLE_BLOCK     4           // $00000004
  #define CS_OPT_STOP         8           // $00000008
  #define CS_BLOCK_DELETE     16          // $00000010
  #define CS_CLNT_FLOOD       32          // $00000020
  #define CS_CLNT_MIST        64          // $00000040
  #define CS_JOG_PLUS         128         // $00000080
  #define CS_JOG_MINUS        256         // $00000100
  #define CS_JOG_STOP          512        // $00000200
  #define CS_JOG_RETURN       1024        // $00000400
  #define CS_HOME             2048        // $00000800
  #define CS_CYCLE_START      4096        // $00001000
  #define CS_CYCLE_RESTART    8192        // $00002000
  #define CS_FEED_HOLD        16384       // $00004000
  #define CS_DRY_RUN          32768       // $00008000
  #define CS_SPND_CW          65536       // $00010000
  #define CS_SPND_CCW         131072      // $00020000
  #define CS_SPND_BRAKE       262144      // $00040000
  #define CS_SPND_NEUTRAL     524288      // $00080000
  #define CS_PRG_REWIND       1048576     // $00100000
  #define CS_CHUCK_OPEN       2097152     // $00200000
  #define CS_CHUCK_CLOSE      4194304     // $00300000
  #define CS_CHUCK_OD_GRIP    8388608     // $00800000
  #define CS_CHUCK_ID_GRIP    16777216    // $01000000
  #define CS_TOOL_RELEASE     33554432    // $02000000
  #define CS_TOOL_ENGAGE      67108864    // $04000000
  #define CS_SPND_DETECT     134217728    // $08000000
  #define CS_SPND_CSS          268435456  // $10000000
  #define CS_SPND_AT_SPEED     536870912  // $20000000
  #define CS_SPND_AT_ZERO      1073741824 // $40000000
  #define CS_SPND_FEED         2147483648 // $80000000

// Command/Status (CS) #4 32 bits ////////////////////////////////////////////
  #define PC_STATUS4_M        3
  #define CS_STATUS4_M        M3
  #define PC_COMMAND4_M       11
  #define CS_COMMAND4_M       M11

  #define CS_TURRET_HOME      1          // $00000001
  #define CS_TURRET_INC_CW    2          // $00000002
  #define CS_TURRET_INC_CCW   4          // $00000004
  #define CS_TURRET_JOG_CW    8          // $00000008
  #define CS_TURRET_JOG_CCW   16         // $00000010
  #define CS_TURRET_SELECT    32         // $00000020
  #define CS_TURRET_ALIGN     64         // $00000040
  #define CS_TURRET_PARK      128        // $00000080
  #define CS_TSTOCKB_ADV      256        // $00000100
  #define CS_TSTOCKB_RET      512        // $00000200
  #define CS_TSTOCKQ_ADV      1024       // $00000400
  #define CS_TSTOCKQ_RET      2048       // $00000800
  #define CS_SREST1_OPEN      4096       // $00001000
  #define CS_SREST1_CLOSE     8192       // $00002000
  #define CS_SREST2_OPEN      16384      // $00004000
  #define CS_SREST2_CLOSE     32768      // $00008000
  #define CS_PRG_SEARCH       65536      // $00010000
  #define CS_PCATCHR_PARK     131072     // $00020000
  #define CS_HOME_INIT        262144     // $00040000
  #define CS_HOME_INPROGRESS  524288     // $00080000
  #define CS_HOME_REFERENCE   1048576    // $00100000
  #define CS_TOOL_CHANGE      2097152    // $00200000
  #define CS_SPND_ORIENT      4194304    // $00400000
  #define CS_CHIP_CONVEYOR    8388608    // $00800000
  #define CS_USER_1           16777216   // $01000000
  #define CS_USER_2           33554432   // $02000000
  #define CS_USER_3           67108864   // $04000000
  #define CS_USER_4           134217728  // $08000000
  #define CS_USER_5           268435456  // $10000000
  #define CS_USER_6           536870912  // $20000000
  #define CS_USER_7           1073741824 // $40000000
  #define CS_USER_8           2147483648 // $80000000

// Command/Status (CS) #5 32 bits ////////////////////////////////////////////
  #define PC_STATUS5_M            4
  #define CS_STATUS5_M            M4
  #define PC_COMMAND5_M           12
  #define CS_COMMAND5_M           M12

  #define S_MOTION_M              M75
  #define C_MOTION_M              M95
  #define SEL_MOTION_MASK         240 // $0000000F
  #define SEL_MOTION_LINEAR       0
  #define SEL_MOTION_RAPID        1
  #define SEL_MOTION_CIR1         2
  #define SEL_MOTION_CIR2         3
  #define SEL_MOTION_SPLINE1      4
  #define SEL_MOTION_PVT          5

  #define S_TIMEBASE_M            M76
  #define C_TIMEBASE_M            M96
  #define SEL_TIMEBASE_CUTTING    0
  #define SEL_TIMEBASE_RAPID      1
  #define SEL_TIMEBASE_THREADING  2
  #define SEL_TIMEBASE_EXACT      3

  #define S_PROG_STATUS_M         M77
  #define C_PROG_STATUS_M         M97
  #define SEL_MOTION_STOPPED      0
  #define SEL_MOTION_RUNNING      1
  #define SEL_MOTION_STEPPING     2
  #define SEL_MOTION_HOLD         3
  #define SEL_MOTION_JOGHOLD      4
  #define SEL_MOTION_STOPHOLD     5

  #define CS_SINGLE_BLOCK_ON      65536      // $00010000
  #define CS_SINGLE_BLOCK_OFF     131072     // $00020000
  #define CS_OPT_STOP_ON          262144     // $00040000
  #define CS_OPT_STOP_OFF         524288     // $00080000
  #define CS_BLOCK_DELETE_ON      1048576    // $00100000
  #define CS_BLOCK_DELETE_OFF     2097152    // $00200000
  #define CS_COOLANT_FLOOD_ON     4194304    // $00400000
  #define CS_COOLANT_FLOOD_OFF    8388608    // $00800000
  #define CS_COOLANT_MIST_ON      16777216   // $01000000
  #define CS_COOLANT_MIST_OFF     33554432   // $02000000
  #define CS_DRY_RUN_ON           67108864   // $04000000
  #define CS_DRY_RUN_OFF          134217728  // $08000000

// Command/Status (CS) #6 32 bits ////////////////////////////////////////////
  #define PC_STATUS6_M            5
  #define CS_STATUS6_M            M5
  #define PC_COMMAND6_M           13
  #define CS_COMMAND6_M           M13

  #define CS_JOG_X_PLUS           1       // $00000001
  #define CS_JOG_X_MINUS          2       // $00000002
  #define CS_JOG_Y_PLUS           4       // $00000004
  #define CS_JOG_Y_MINUS          8       // $00000008
  #define CS_JOG_Z_PLUS           16      // $00000010
  #define CS_JOG_Z_MINUS          32      // $00000020
  #define CS_JOG_U_PLUS           64      // $00000040
  #define CS_JOG_U_MINUS          128     // $00000080
  #define CS_JOG_V_PLUS           256     // $00000100
  #define CS_JOG_V_MINUS          512     // $00000200
  #define CS_JOG_W_PLUS           1024    // $00000400
  #define CS_JOG_W_MINUS          2048    // $00000800
  #define CS_JOG_A_PLUS           4096    // $00001000
  #define CS_JOG_A_MINUS          8192    // $00002000
  #define CS_JOG_B_PLUS           16384   // $00004000
  #define CS_JOG_B_MINUS          32768   // $00008000
  #define CS_JOG_C_PLUS           65536   // $00010000
  #define CS_JOG_C_MINUS          131072  // $00020000
  #define CS_HOME_ALL             262144  // $00040000
  #define CS_METRIC_MODE          524288  // $00080000
  #define CS_PARTS_COUNT          1048576 // $00100000
  #define CS_POWER_OFF            2097152 // $00200000
  #define CS_ROTATION_ON          4194304 // $00400000
  #define CS_MIRRORED_ON          8388608 // $00800000
  #define CS_ROTATION2_ON        16777216 // $01000000
  #define CS_MIRRORED2_ON        33554432 // $02000000
  #define CS_DIAMETER_MODE       67108864 // $04000000
  #define CS_COORDSYS_CHECK     134217728 // $08000000
  #define CS_PARTS_RESET        268435456 // $10000000
  #define CS_G31_SKIP_OUT       536870912 // $20000000
  #define CS_G31_SKIP_IN       1073741824 // $40000000
  #define CS_BUFFER_HOLD       2147483648 // $80000000

// Command/Status (CS) #7 32 bits ////////////////////////////////////////////
  #define PC_STATUS7_M            6
  #define CS_STATUS7_M            M6
  #define PC_COMMAND7_M           14
  #define CS_COMMAND7_M           M14


// Command/Status (CS) #8 32 bits ////////////////////////////////////////////
  #define PC_STATUS8_M            7
  #define CS_STATUS8_M            M7
  #define PC_COMMAND8_M           15
  #define CS_COMMAND8_M           M15

//////////////////////////////////////////////////////////////////////////////
// Probing  Command and Data Structure
//////////////////////////////////////////////////////////////////////////////
  #define PR_BITS_M               M160
  #define PR_COMMAND_M            M161
  #define PR_DATA_M               M162

  #define G65_LEVEL               M163
  #define G65_LEVEL2              M164

  #define PC_RT_BLOCK             165
  #define PMAC_RT_BLOCK           M165
  #define PC_RT_BLOCK2            166 
  #define PMAC_RT_BLOCK2          M166

  #define PC_RT_NM_BLOCK          167
  #define PMAC_RT_NM_BLOCK        M167
  #define PC_RT_NM_BLOCK2         168 
  #define PMAC_RT_NM_BLOCK2       M168

  #define PC_PMAC_RESET_COMPLETE  279
  #define PMAC_RESET_COMPLETE     M279
  #define PC_PC_RESET_COMPLETE    359
  #define PC_RESET_COMPLETE       M359

  #define PC_PMAC_ROT_BUF_FIL_STOP 259
  #define PMAC_ROT_BUF_FIL_STOP   M259
  #define PMAC_ROT_BUF_FIL_STOP16 (PMAC_ROT_BUF_FIL_STOP & $ffff)

// extended block numbers available in pmac.dll 10.20
  #define PC_L_NLABEL_M             78
  #define VS_L_NLABEL_M            M78
  #define PC_L_NLABEL2_M            79
  #define VS_L_NLABEL2_M           M79


//////////////////////////////////////////////////////////////////////////////
// Value/Status structure
// This section of value status reserves M20..M59
//////////////////////////////////////////////////////////////////////////////
  #define PC_VALUE_OFS                    0x3B80

// Value/Status (VS) #1 32 bits ////////////////////////////////////////////
  #define PC_CURRENT_PROG_M               20
  #define VS_CURRENT_PROG_M               M20

// Value/Status (VS) #2 32 bits ////////////////////////////////////////////
  #define PC_CURRENT_PROG2_M              21
  #define VS_CURRENT_PROG2_M              M21

// Value/Status (VS) #3 32 bits ////////////////////////////////////////////
  #define PC_CURRENT_BLOCK_M              22
  #define VS_CURRENT_BLOCK_M              M22

// Value/Status (VS) #4 32 bits ////////////////////////////////////////////
  #define PC_CURRENT_BLOCK2_M             23
  #define VS_CURRENT_BLOCK2_M             M23

// Value/Status (VS) #5 32 bits ////////////////////////////////////////////
  #define PC_LAST_BLOCK_M                 24
  #define VS_LAST_BLOCK_M                 M24

// Value/Status (VS) #6 32 bits ////////////////////////////////////////////
  #define PC_LAST_BLOCK2_M                25
  #define VS_LAST_BLOCK2_M                M25

// Value/Status (VS) #7 32 bits ////////////////////////////////////////////
  #define PC_SPINDLE_RPM_M                26
  #define VS_SPINDLE_RPM_M                M26
  #define VS_SPINDLE_RPM_Q           Q127

// PLC reset flags - use as bitmapped booleans, bits 0..31 for plc 0..31
  #define PLC_RESET_BITS                  Q128

// Value/Status (VS) #8 32 bits ////////////////////////////////////////////
  #define PC_SPINDLE_MAX_RPM_M            27
  #define VS_SPINDLE_MAX_RPM_M            M27

// Value/Status (VS) #8 32 bits ////////////////////////////////////////////
  #define PC_SPINDLE_MAX_LIM_M            28
  #define VS_SPINDLE_MAX_LIM_M            M28

// Value/Status (VS) #9 32 bits ////////////////////////////////////////////
  #define VS_SPINDLE_CMD_RPM_M            M29

// Value/Status (VS) #10 32 bits ////////////////////////////////////////////
  #define VS_SPINDLE_ACT_RPM_M            M30

// Value/Status (VS) #11 32 bits ////////////////////////////////////////////
  #define VS_SPINDLE_COUNTS_REV_M         M31

// Value/Status (VS) #12 32 bits ////////////////////////////////////////////
  #define PC_SPINDLE_CSS_M                32
  #define VS_SPINDLE_CSS_M                M32

// Value/Status (VS) #12 32 bits ////////////////////////////////////////////
  #define VS_SPINDLE_CSS_UNITS_M          M33

// Value/Status (VS) #13 16 bits ////////////////////////////////////////////
  #define VS_GEAR_RATIO_M                 M34

// Value/Status (VS) #14 16 bits ////////////////////////////////////////////
  #define VS_SPINDLE_OVERRIDE_M           M35

// Value/Status (VS) #14 16 bits ////////////////////////////////////////////
  #define VS_BLOCKS_SEC_M                 M36

// Value/Status (VS) #15 16 bits ////////////////////////////////////////////
  #define VS_TURRET2_TOOL_M               M37
  #define VS_TURRET2_TOOL_P               P37

// Value/Status (VS) #15 16 bits ////////////////////////////////////////////
  #define VS_TURRET_TOOL_M                M38
  #define VS_TURRET_TOOL_P                P38

// Value/Status (VS) #16 16 bits ////////////////////////////////////////////
  #define PC_TURRET_POT_M                 39
  #define VS_TURRET_POT_M                 M39
  #define VS_TURRET_POT_P                 P39

// Value/Status (VS) #16 16 bits ////////////////////////////////////////////
  #define PC_TURRET2_POT_M                40
  #define VS_TURRET2_POT_M                M40
  #define VS_TURRET2_POT_P                P40

// Value/Status (VS) #17 16 bits ////////////////////////////////////////////
  #define PC_TOOL_NUMBER_M                41
  #define VS_TOOL_NUMBER_M                M41
  #define VS_TOOL_NUMBER_P                P41

// Value/Status (VS) #17 16 bits ////////////////////////////////////////////
  #define PC_TOOL_NUMBER2_M               42
  #define VS_TOOL_NUMBER2_M               M42
  #define VS_TOOL_NUMBER2_P               P42

// Value/Status (VS) #18 16 bits ////////////////////////////////////////////
  #define PC_TOOL_NEXT_M                  43
  #define VS_TOOL_NEXT_M                  M43
  #define VS_TOOL_NEXT_P                  P43

// Value/Status (VS) #18 16 bits ////////////////////////////////////////////
  #define PC_TOOL_NEXT2_M                 44
  #define VS_TOOL_NEXT2_M                 M44
  #define VS_TOOL_NEXT2_P                 P44

// Value/Status (VS) #19 32 bits ////////////////////////////////////////////
  #define VS_HANDLE_LINC_M                M45

// Value/Status (VS) #20 32 bits ////////////////////////////////////////////
  #define VS_JOG_STEP_M                   M46

// Value/Status (VS) #21 32 bits ////////////////////////////////////////////
  #define VS_HAND_STEP_M                  M47

// Value/Status (VS) #24 16 bits ////////////////////////////////////////////
  #define PC_SEMAPHORE_M                  48
  #define VS_SEMAPHORE_M                  M48

// Value/Status (VS) #25 16 bits ////////////////////////////////////////////
  #define PC_SEMAPHORE2_M                 49
  #define VS_SEMAPHORE2_M                 M49

// Value/Status (VS) #26 32 bits ////////////////////////////////////////////
  #define VS_FEED_OVERRIDE_M              M50

// Value/Status (VS) #27 32 bits ////////////////////////////////////////////
  #define VS_FEED_OVERRIDE2_M             M51

// Value/Status (VS) #28 32 bits ////////////////////////////////////////////
  #define VS_RAPID_OVERRIDE_M             M52

// Value/Status (VS) #29 32 bits ////////////////////////////////////////////
  #define VS_RAPID_OVERRIDE2_M            M53

// Value/Status (VS) #26 32 bits ////////////////////////////////////////////
  #define VS_THREAD_LEAD_M                M54

// Value/Status (VS) #26 32 bits ////////////////////////////////////////////
  #define VS_THREAD_LEAD2_M               M55

/////////////////////////////////////////////////////////////////////////////
// Translation / relative[coorsys][ABCXYZUVW]
// Value/Status (VS) #30 32 bits ////////////////////////////////////////////
// This section of the value status reserves M100 .. M169
// for accurate displays
/////////////////////////////////////////////////////////////////////////////
  #define VS_A_REL_M                    M100
  #define VS_A_WRK_Q                    Q200
  #define PC_A_WRK_Q                    200
  #define VS_A_LOC_Q                    Q201
  #define PC_A_LOC_Q                    201
  #define VS_A_COM_Q                    Q202
  #define PC_A_COM_Q                    202

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_B_REL_M                    M101
  #define VS_B_WRK_Q                    Q203
  #define PC_B_WRK_Q                    203
  #define VS_B_LOC_Q                    Q204
  #define PC_B_LOC_Q                    204
  #define VS_B_COM_Q                    Q205
  #define PC_B_COM_Q                    205

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_C_REL_M                    M102
  #define VS_C_WRK_Q                    Q206
  #define PC_C_WRK_Q                    206
  #define VS_C_LOC_Q                    Q207
  #define PC_C_LOC_Q                    207
  #define VS_C_COM_Q                    Q208
  #define PC_C_COM_Q                    208

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_X_REL_M                    M103
  #define VS_X_WRK_Q                    Q209
  #define PC_X_WRK_Q                    209
  #define VS_X_LOC_Q                    Q210
  #define PC_X_LOC_Q                    210
  #define VS_X_COM_Q                    Q211
  #define PC_X_COM_Q                    211

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_Y_REL_M                    M104
  #define VS_Y_WRK_Q                    Q212
  #define PC_Y_WRK_Q                    212
  #define VS_Y_LOC_Q                    Q213
  #define PC_Y_LOC_Q                    213
  #define VS_Y_COM_Q                    Q214
  #define PC_Y_COM_Q                    214

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_Z_REL_M                    M105
  #define VS_Z_WRK_Q                    Q215
  #define PC_Z_WRK_Q                    215
  #define VS_Z_LOC_Q                    Q216
  #define PC_Z_LOC_Q                    216
  #define VS_Z_COM_Q                    Q217
  #define PC_Z_COM_Q                    217

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_U_REL_M                    M106
  #define VS_U_WRK_Q                    Q218
  #define PC_U_WRK_Q                    218
  #define VS_U_LOC_Q                    Q219
  #define PC_U_LOC_Q                    219
  #define VS_U_COM_Q                    Q220
  #define PC_U_COM_Q                    220

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_V_REL_M                    M107
  #define VS_V_WRK_Q                    Q221
  #define PC_V_WRK_Q                    221
  #define VS_V_LOC_Q                    Q222
  #define PC_V_LOC_Q                    222
  #define VS_V_COM_Q                    Q223
  #define PC_V_COM_Q                    223

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_W_REL_M                    M108
  #define VS_W_WRK_Q                    Q224
  #define PC_W_WRK_Q                    224
  #define VS_W_LOC_Q                    Q225
  #define PC_W_LOC_Q                    225
  #define VS_W_COM_Q                    Q226
  #define PC_W_COM_Q                    226

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_A_REL2_M                   M109
  #define VS_A_WRK2_Q                   Q227
  #define PC_A_WRK2_Q                   227
  #define VS_A_LOC2_Q                   Q228
  #define PC_A_LOC2_Q                   228
  #define VS_A_COM2_Q                   Q229
  #define PC_A_COM2_Q                   229

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_B_REL2_M                   M110
  #define VS_B_WRK2_Q                   Q230
  #define PC_B_WRK2_Q                   230
  #define VS_B_LOC2_Q                   Q231
  #define PC_B_LOC2_Q                   231
  #define VS_B_COM2_Q                   Q232
  #define PC_B_COM2_Q                   232

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_C_REL2_M                   M111
  #define VS_C_WRK2_Q                   Q233
  #define PC_C_WRK2_Q                   233
  #define VS_C_LOC2_Q                   Q234
  #define PC_C_LOC2_Q                   234
  #define VS_C_COM2_Q                   Q235
  #define PC_C_COM2_Q                   235

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_X_REL2_M                   M112
  #define VS_X_WRK2_Q                   Q236
  #define PC_X_WRK2_Q                    236
  #define VS_X_LOC2_Q                   Q237
  #define PC_X_LOC2_Q                   237
  #define VS_X_COM2_Q                   Q238
  #define PC_X_COM2_Q                   238

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_Y_REL2_M                   M113
  #define VS_Y_WRK2_Q                   Q239
  #define PC_Y_WRK2_Q                   239
  #define VS_Y_LOC2_Q                   Q240
  #define PC_Y_LOC2_Q                   240
  #define VS_Y_COM2_Q                   Q241
  #define PC_Y_COM2_Q                   241

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_Z_REL2_M                   M114
  #define VS_Z_WRK2_Q                   Q242
  #define PC_Z_WRK2_Q                    242
  #define VS_Z_LOC2_Q                   Q243
  #define PC_Z_LOC2_Q                   243
  #define VS_Z_COM2_Q                   Q244
  #define PC_Z_COM2_Q                   244

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_U_REL2_M                   M115
  #define VS_U_WRK2_Q                   Q245
  #define PC_U_WRK2_Q                   245
  #define VS_U_LOC2_Q                   Q246
  #define PC_U_LOC2_Q                   246
  #define VS_U_COM2_Q                   Q247
  #define PC_U_COM2_Q                   247

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_V_REL2_M                   M116
  #define VS_V_WRK2_Q                   Q248
  #define PC_V_WRK2_Q                   248
  #define VS_V_LOC2_Q                   Q249
  #define PC_V_LOC2_Q                   249
  #define VS_V_COM2_Q                   Q250
  #define PC_V_COM2_Q                   250

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_W_REL2_M                   M117
  #define VS_W_WRK2_Q                   Q251
  #define PC_W_WRK2_Q                   251
  #define VS_W_LOC2_Q                   Q252
  #define PC_W_LOC2_Q                   252
  #define VS_W_COM2_Q                   Q253
  #define PC_W_COM2_Q                   253

/***************************************************************************/
// Translation / absolute[coorsys][ABCXYZUVW]
// Value/Status (VS) #30 32 bits ////////////////////////////////////////////
  #define VS_A_ABS_M                    M118
  #define VS_A_TOOL_Q                   Q254
  #define PC_A_TOOL_Q                   254

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_B_ABS_M                    M119
  #define VS_B_TOOL_Q                   Q255
  #define PC_B_TOOL_Q                   255

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_C_ABS_M                    M120
  #define VS_C_TOOL_Q                   Q256
  #define PC_C_TOOL_Q                   256

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_X_ABS_M                    M121
  #define VS_X_TOOL_Q                   Q257
  #define PC_X_TOOL_Q                   257

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_Y_ABS_M                    M122
  #define VS_Y_TOOL_Q                   Q258
  #define PC_Y_TOOL_Q                   258

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_Z_ABS_M                    M123
  #define VS_Z_TOOL_Q                   Q259
  #define PC_Z_TOOL_Q                   259

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_U_ABS_M                    M124
  #define VS_U_TOOL_Q                   Q260
  #define PC_U_TOOL_Q                   260

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_V_ABS_M                    M125
  #define VS_V_TOOL_Q                   Q261
  #define PC_V_TOOL_Q                   261

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_W_ABS_M                    M126
  #define VS_W_TOOL_Q                   Q262
  #define PC_W_TOOL_Q                   262

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_A_ABS2_M                   M127
  #define VS_A_TOOL2_Q                  Q263
  #define PC_A_TOOL2_Q                  263

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_B_ABS2_M                   M128
  #define VS_B_TOOL2_Q                  Q264
  #define PC_B_TOOL2_Q                  264

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_C_ABS2_M                   M129
  #define VS_C_TOOL2_Q                  Q265
  #define PC_C_TOOL2_Q                  265

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_X_ABS2_M                   M130
  #define VS_X_TOOL2_Q                  Q266
  #define PC_X_TOOL2_Q                  266

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_Y_ABS2_M                   M131
  #define VS_Y_TOOL2_Q                  Q267
  #define PC_Y_TOOL2_Q                  267

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_Z_ABS2_M                   M132
  #define VS_Z_TOOL2_Q                  Q268
  #define PC_Z_TOOL2_Q                  268

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_U_ABS2_M                   M133
  #define VS_U_TOOL2_Q                  Q269
  #define PC_U_TOOL2_Q                  269

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_V_ABS2_M                   M134
  #define VS_V_TOOL2_Q                  Q270
  #define PC_V_TOOL2_Q                  270

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_W_ABS2_M                   M135
  #define VS_W_TOOL2_Q                  Q271
  #define PC_W_TOOL2_Q                  271

/***************************************************************************/
// Translation / mul_coeff[coorsys][ABCXYZUVW]
// Value/Status (VS) #30 32 bits ////////////////////////////////////////////
  #define VS_A_MUL_M                    M136
  #define VS_A_ROT_Q                    Q272
  #define VS_A_MIR_Q                    Q273
  #define VS_A_SCL_Q                    Q274

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_B_MUL_M                    M137
  #define VS_B_ROT_Q                    Q275
  #define VS_B_MIR_Q                    Q276
  #define VS_B_SCL_Q                    Q277

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_C_MUL_M                    M138
  #define VS_C_ROT_Q                    Q278
  #define VS_C_MIR_Q                    Q279
  #define VS_C_SCL_Q                    Q280

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_X_MUL_M                    M139
  #define VS_X_ROT_Q                    Q281
  #define VS_X_MIR_Q                    Q282
  #define VS_X_SCL_Q                    Q283

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_Y_MUL_M                    M140
  #define VS_Y_ROT_Q                    Q284
  #define VS_Y_MIR_Q                    Q285
  #define VS_Y_SCL_Q                    Q286

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_Z_MUL_M                    M141
  #define VS_Z_ROT_Q                    Q287
  #define VS_Z_MIR_Q                    Q288
  #define VS_Z_SCL_Q                    Q289

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_U_MUL_M                    M142
  #define VS_U_ROT_Q                    Q290
  #define VS_U_MIR_Q                    Q291
  #define VS_U_SCL_Q                    Q292

// Value/Status (VS) #31 32 bits ////////////////////////////////////////////
  #define VS_V_MUL_M                    M143
  #define VS_V_ROT_Q                    Q293
  #define VS_V_MIR_Q                    Q294
  #define VS_V_SCL_Q                    Q295

// Value/Status (VS) #32 32 bits ////////////////////////////////////////////
  #define VS_W_MUL_M                    M144
  #define VS_W_ROT_Q                    Q296
  #define VS_W_MIR_Q                    Q297
  #define VS_W_SCL_Q                    Q298

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_A_MUL2_M                   M145
  #define VS_A_ROT2_Q                   Q299
  #define VS_A_MIR2_Q                   Q300
  #define VS_A_SCL2_Q                   Q301

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_B_MUL2_M                   M146
  #define VS_B_ROT2_Q                   Q302
  #define VS_B_MIR2_Q                   Q303
  #define VS_B_SCL2_Q                   Q304

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_C_MUL2_M                   M147
  #define VS_C_ROT2_Q                   Q305
  #define VS_C_MIR2_Q                   Q306
  #define VS_C_SCL2_Q                   Q307

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_X_MUL2_M                   M148
  #define VS_X_ROT2_Q                   Q308
  #define VS_X_MIR2_Q                   Q309
  #define VS_X_SCL2_Q                   Q310

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_Y_MUL2_M                   M149
  #define VS_Y_ROT2_Q                   Q311
  #define VS_Y_MIR2_Q                   Q312
  #define VS_Y_SCL2_Q                   Q313

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_Z_MUL2_M                   M150
  #define VS_Z_ROT2_Q                   Q314
  #define VS_Z_MIR2_Q                   Q315
  #define VS_Z_SCL2_Q                   Q316

// Value/Status (VS) #33 32 bits ////////////////////////////////////////////
  #define VS_U_MUL2_M                   M151
  #define VS_U_ROT2_Q                   Q317
  #define VS_U_MIR2_Q                   Q318
  #define VS_U_SCL2_Q                   Q319

// Value/Status (VS) #34 32 bits ////////////////////////////////////////////
  #define VS_V_MUL2_M                   M152
  #define VS_V_ROT2_Q                   Q320
  #define VS_V_MIR2_Q                   Q321
  #define VS_V_SCL2_Q                   Q322

// Value/Status (VS) #35 32 bits ////////////////////////////////////////////
  #define VS_W_MUL2_M                   M153
  #define VS_W_ROT2_Q                   Q323
  #define VS_W_MIR2_Q                   Q324
  #define VS_W_SCL2_Q                   Q325

  #define VS_CUTTER_COMP_Q              Q326
  #define PC_CUTTER_COMP_Q              326

/**************************************************************************/
// Value/Status (VS) #54 32 bits ////////////////////////////////////////////
  #define VS_NLABEL_M                   M154
  #define PC_NLABEL_M                   154

// Value/Status (VS) #55 32 bits ////////////////////////////////////////////
  #define VS_NLABEL2_M                  M155
  #define PC_NLABEL2_M                  155

// Value/Status (VS) #55 32 bits ////////////////////////////////////////////
  #define VS_TOOL_OFFSET_M                M156

  #define VS_TOOL_OFFSET2_M               M157

  #define VS_COMP_OFFSET_M                M158
  #define PC_COMP_OFFSET_M                158

  #define VS_COMP_OFFSET2_M               M159
  #define PC_COMP_OFFSET2_M                159



////////////////////////////////////////////////////////////////////////////
// G Code Groups Coordinate System 1
// Reserves M170..M195  
////////////////////////////////////////////////////////////////////////////
  #define PC_GGROUP_1_OFS                   0x3DC8

  #define VS_GGROUP0_1_M                    M170

  #define VS_GGROUP1_1_M                    M171

  #define VS_GGROUP2_1_M                    M172

  #define VS_GGROUP3_1_M                    M173

  #define VS_GGROUP4_1_M                    M174

  #define VS_GGROUP5_1_M                    M175

  #define VS_GGROUP6_1_M                    M176

  #define VS_GGROUP7_1_M                    M177

  #define VS_GGROUP8_1_M                    M178

  #define VS_GGROUP9_1_M                    M179

  #define VS_GGROUP10_1_M                   M180

  #define VS_GGROUP11_1_M                   M181

  #define VS_GGROUP12_1_M                   M182

  #define VS_GGROUP13_1_M                   M183

  #define VS_GGROUP14_1_M                   M184

  #define VS_GGROUP15_1_M                   M185

  #define VS_GGROUP16_1_M                   M186

  #define VS_GGROUP17_1_M                   M187

  #define VS_GGROUP18_1_M                   M188

  #define VS_GGROUP19_1_M                   M189

  #define VS_GGROUP20_1_M                   M190

  #define VS_GGROUP21_1_M                   M191

  #define VS_GGROUP22_1_M                   M192

  #define VS_GGROUP23_1_M                   M193

  #define VS_GGROUP24_1_M                   M194

  #define VS_GGROUP25_1_M                   M195


////////////////////////////////////////////////////////////////////////////
// G Code Groups Coordinate System 2  !!! F#CK@D ---> Using  M's of crd sys 1
////////////////////////////////////////////////////////////////////////////
  #define PC_GGROUP_2_OFS                 0x3DOC

  #define VS_GGROUP0_2_M                    M195

  #define VS_GGROUP1_2_M                    M196

  #define VS_GGROUP2_2_M                    M197

  #define VS_GGROUP3_2_M                    M198

  #define VS_GGROUP4_2_M                    M199

  #define VS_GGROUP5_2_M                    M200

  #define VS_GGROUP6_2_M                    M201

  #define VS_GGROUP7_2_M                    M202

  #define VS_GGROUP8_2_M                    M203

  #define VS_GGROUP9_2_M                    M204

  #define VS_GGROUP10_2_M                   M205

  #define VS_GGROUP11_2_M                   M206

  #define VS_GGROUP12_2_M                   M207

  #define VS_GGROUP13_2_M                   M208

  #define VS_GGROUP14_2_M                   M209

  #define VS_GGROUP15_2_M                   M210

  #define VS_GGROUP16_2_M                   M211

  #define VS_GGROUP17_2_M                   M212

  #define VS_GGROUP18_2_M                   M213

  #define VS_GGROUP19_2_M                   M214

  #define VS_GGROUP20_2_M                   M215

  #define VS_GGROUP21_2_M                   M216

  #define VS_GGROUP22_2_M                   M217

  #define VS_GGROUP23_2_M                   M218

  #define VS_GGROUP24_2_M                   M219

  #define VS_GGROUP25_2_M                   M219

////////////////////////////////////////////////////////////////////////////
// Error Status Structure
////////////////////////////////////////////////////////////////////////////
  #define PC_ERROR_OFS                   0x3D80

// Error/Status (ES) #1 32 bits ////////////////////////////////////////////
  #define PC_CLOCK_OFS                   0x3D80
  #define ES_PC_CLOCK_M                  M220

// Error/Status (ES) #2 64 bits ////////////////////////////////////////////
// Drives OFF, PMAC off
  #define PC_ERR_FATAL1_OFS              0x3D84
  #define ES_ERR_FATAL_M                 M221

  #define PC_ERR_FATAL2_OFS              0x3D88
  #define ES_ERR_FATAL2_M                M222

// Error/Status (ES) #3 32 bits ////////////////////////////////////////////
// Feed stop and Spindle stop
  #define PC_ERR_STOP1_OFS               0x3D8C
  #define ES_ERR_STOP_M                  M223

  #define PC_ERR_STOP2_OFS               0x3D90
  #define ES_ERR_STOP2_M                 M224

// Error/Status (ES) #4 64 bits ////////////////////////////////////////////
// Cycle start inhibit
  #define PC_ERR_WARN1_OFS               0x3D94
  #define ES_ERR_WARN_M                  M225

  #define PC_ERR_WARN2_OFS               0x3D98
  #define ES_ERR_WARN2_M                 M226

// Error/Status (ES) #6 64 bits ////////////////////////////////////////////
// Message only
  #define PC_ERR_MSG1_OFS                0x3D9C
  #define ES_ERR_MSG_M                   M227

  #define PC_ERR_MSG2_OFS                0x3DA0
  #define ES_ERR_MSG2_M                  M228

// Error/Status (ES)  64 bits ////////////////////////////////////////////
  #define PC_ERR_PLCMSGBOX_OFS           0x3DA4
  #define ES_PLCMSGBOX_M                 M238

// Message from PMAC: TIM TIC( 10 msec units ) & PLCC SCAN Time( 1 msec units )
  #define PC_SCANAVE_OFS                 0x3DA8
  #define ES_SCANAVE_M                   M229

  #define PC_SCANMAX_OFS                 0x3DAA
  #define ES_SCANMAX_M                   M269
///////////////////////////////////////////////////////////////////////////
// IO RAW DATA
// M230..M249 (Input Images)
// M250..M268 (Output Images)
///////////////////////////////////////////////////////////////////////////
  #define PC_IO_OFS           0x3F80
  #define PC_IO_CHNG_OFS      0x3FC0

  #define IN_1_M              M230
  #define IN_1_CHNG_M         M240

  #define PP_IN_1032          0x3F84
  #define IN_2_M              M231
  #define IN_2_CHNG_M         M241

  #define PP_IN_1033          0x3F88
  #define IN_3_M              M232
  #define IN_3_CHNG_M         M242

  #define PP_IN_1034          0x3F8C
  #define IN_4_M              M233
  #define IN_4_CHNG_M         M243

  #define PP_IN_1035          0x3F90
  #define IN_5_M              M234
  #define IN_5_CHNG_M         M244

  #define IN_6_M              M235
  #define IN_6_CHNG_M         M245

  #define IN_7_M              M236
  #define IN_7_CHNG_M         M246

  #define IN_8_M              M237
  #define IN_8_CHNG_M         M247

  #define AT_DPR_POINTER      M248
  #define DPR_POINTER         M249

  #define OUT_1_M             M250
  #define OUT_1_CHNG_M        M260

  #define PP_OUT_1132         0x3FA4
  #define OUT_2_M             M251
  #define OUT_2_CHNG_M        M261

  #define PP_OUT_1133         0x3FA8
  #define OUT_3_M             M252
  #define OUT_3_CHNG_M        M262

  #define PP_OUT_1134         0x3FAC
  #define OUT_4_M             M253
  #define OUT_4_CHNG_M        M263

  #define PP_OUT_1135         0x3FB0
  #define OUT_5_M             M254
  #define OUT_5_CHNG_M        M264

  #define OUT_6_M             M255
  #define OUT_6_CHNG_M        M265

  #define OUT_7_M             M256
  #define OUT_7_CHNG_M        M266

  #define OUT_8_M             M257
  #define OUT_8_CHNG_M        M267

// M269 USED BY ERROR STRUCTURE

// DUAL PORTED RAM OFFSETS
  #define OUT_1_OFS           4072 // FE8
  #define OUT_2_OFS           4073 // FE9
  #define OUT_3_OFS           4074 // FEA
  #define OUT_4_OFS           4075 // FEB
  #define OUT_5_OFS           4076 // FEC
  #define OUT_6_OFS           4077 // FED
  #define OUT_7_OFS           4078 // FEE
  #define OUT_8_OFS           4079 // FEF

#ifdef _TURBO_
  #ifdef _UMAC_
    #define DPR_BASE            $6C000
    #define CS_COMMAND1_ADR     DP:$6CDF0
    #define CS_STATUS1_ADR      DP:$6CDE0
    #define S_JOG_ADR           Y:$6CDE0,0,4,U
    #define C_JOG_ADR           Y:$6CDF0,0,4,U
    #define S_MODE_ADR          Y:$6CDE0,4,4,U
    #define C_MODE_ADR          Y:$6CDF0,4,4,U
    #define S_AXIS_ADR          Y:$6CDE0,8,4,U
    #define C_AXIS_ADR          Y:$6CDF0,8,4,U
    #define S_FEED_ADR          Y:$6CDE0,12,4,U
    #define C_FEED_ADR          Y:$6CDF0,12,4,U
    #define S_ROVRD_ADR         X:$6CDE0,0,4,U
    #define C_ROVRD_ADR         X:$6CDF0,0,4,U
    #define S_SPND_ADR          X:$6CDE0,4,4,U
    #define C_SPND_ADR          X:$6CDF0,4,4,U
    #define S_FOVRD_ADR         X:$6CDE0,8,4,U
    #define C_FOVRD_ADR         X:$6CDF0,8,4,U
    #define S_SOVRD_ADR         X:$6CDE0,12,4,U
    #define C_SOVRD_ADR         X:$6CDF0,12,4,U
    #define CS_STATUS2_ADR      DP:$6CDE1
    #define CS_COMMAND2_ADR     DP:$6CDF1
    #define S_COORD_ADR         Y:$6CDE1,0,4,U
    #define C_COORD_ADR         Y:$6CDF1,0,4,U
    #define S_GEAR_ADR          Y:$6CDE1,4,4,U
    #define C_GEAR_ADR          Y:$6CDF1,4,4,U
    #define S_HOME_ADR          Y:$6CDE1,8,8,U
    #define C_HOME_ADR          Y:$6CDF1,8,8,U
    #define S_ERROR_ADR         X:$6CDE1,0,4,U
    #define C_ERROR_ADR         X:$6CDF1,0,4,U
    #define S_CLNT_ADR          X:$6CDE1,4,4,U
    #define C_CLNT_ADR          X:$6CDF1,4,4,U
    #define S_DIST_ADR          X:$6CDE1,8,4,U
    #define C_DIST_ADR          X:$6CDF1,8,4,U
    #define S_MACHINE_ADR       X:$6CDE1,12,4,U
    #define C_MACHINE_ADR       X:$6CDF1,12,4,U
    #define CS_STATUS3_ADR      DP:$6CDE2
    #define CS_COMMAND3_ADR     DP:$6CDF2
    #define CS_STATUS4_ADR      DP:$6CDE3
    #define CS_COMMAND4_ADR     DP:$6CDF3
    #define CS_STATUS5_ADR      DP:$6CDE4
    #define CS_COMMAND5_ADR     DP:$6CDF4
    #define S_MOTION_ADR            Y:$6CDE4,0,4,U
    #define C_MOTION_ADR            Y:$6CDF4,0,4,U
    #define S_TIMEBASE_ADR          Y:$6CDE4,4,4,U
    #define C_TIMEBASE_ADR          Y:$6CDF4,4,4,U
    #define S_PROG_STATUS_ADR       Y:$6CDE4,8,4,U
    #define C_PROG_STATUS_ADR       Y:$6CDF4,8,4,U
    #define CS_STATUS6_ADR          DP:$6CDE5
    #define CS_COMMAND6_ADR         DP:$6CDF5
    #define CS_STATUS7_ADR          DP:$6CDE6
    #define CS_COMMAND7_ADR         DP:$6CDF6
    #define CS_STATUS8_ADR          DP:$6CDE7
    #define CS_COMMAND8_ADR         DP:$6CDF7
    #define PR_BITS_ADR             DP:$6CDFD
    #define PR_COMMAND_ADR          DP:$6CDFE
    #define PR_DATA_ADR             F:$6CDFF

    #define AT_DPR_POINTER_ADR      DP:$6CFFF      // AS DEFAULT, POINT TO UNLIKELY TO BE USED
    #define DPR_POINTER_ADR         Y:$0040F8,0,16 // POINTS TO M248 ADDRESSING PART

    #define G65_LEVEL_ADR           DP:$6CF52
    #define G65_LEVEL2_ADR          DP:$6CF53

    #define PMAC_RT_BLOCK_ADR       DP:$6CF54
    #define PMAC_RT_BLOCK2_ADR      DP:$6CF55
    #define PMAC_RT_NM_BLOCK_ADR    DP:$6CF56
    #define PMAC_RT_NM_BLOCK2_ADR   DP:$6CF57

    #define PMAC_RESET_COMP_ADR     Y:$6CF58,0,16,U
    #define PC_RESET_COMP_ADR       X:$6CF58,0,16,U

    #define ROT_BUF_FIL_STOP_ADR    Y:$6CF59,0,16,U
    #define UNUSED_13_ADR           X:$6CF59,0,16,U

    #define VS_L_NLABEL_ADR         DP:$6CF5A
    #define VS_L_NLABEL2_ADR        DP:$6CF5B

    #define VS_CURRENT_PROG_ADR             DP:$6CEE0
    #define VS_CURRENT_PROG2_ADR            DP:$6CEE1
    #define VS_CURRENT_BLOCK_ADR            DP:$6CEE2
    #define VS_CURRENT_BLOCK2_ADR           DP:$6CEE3
    #define VS_LAST_BLOCK_ADR               DP:$6CEE4
    #define VS_LAST_BLOCK2_ADR              DP:$6CEE5
    #define VS_SPINDLE_RPM_ADR              DP:$6CEE6
    #define VS_SPINDLE_MAX_RPM_ADR          DP:$6CEE7
    #define VS_SPINDLE_MAX_LIM_ADR          DP:$6CEE8
    #define VS_SPINDLE_CMD_RPM_ADR          DP:$6CEE9
    #define VS_SPINDLE_ACT_RPM_ADR          DP:$6CEEA
    #define VS_SPINDLE_COUNTS_REV_ADR       F:$6CEEB
    #define VS_SPINDLE_CSS_ADR              F:$6CEEC
    #define VS_SPINDLE_CSS_UNITS_ADR        F:$6CEED
    #define VS_GEAR_RATIO_ADR               F:$6CEEE
    #define VS_SPINDLE_OVERRIDE_ADR         Y:$6CEEF,0,16,U
    #define VS_BLOCKS_SEC_ADR               X:$6CEEF,0,16,U
    #define VS_TURRET2_TOOL_ADR             Y:$6CEF0,0,16,U
    #define VS_TURRET_TOOL_ADR              X:$6CEF0,0,16,U
    #define VS_TURRET_POT_ADR               Y:$6CEF1,0,16,U
    #define VS_TURRET2_POT_ADR              X:$6CEF1,0,16,U
    #define VS_TOOL_NUMBER_ADR              Y:$6CEF2,0,16,U
    #define VS_TOOL_NUMBER2_ADR             X:$6CEF2,0,16,U
    #define VS_TOOL_NEXT_ADR                Y:$6CEF3,0,16,U
    #define VS_TOOL_NEXT2_ADR               X:$6CEF3,0,16,U
    #define VS_HANDLE_LINC_ADR              F:$6CEF4
    #define VS_JOG_STEP_ADR                 F:$6CEF5
    #define VS_HAND_STEP_ADR                F:$6CEF6
    #define VS_SEMAPHORE_ADR                Y:$6CEF7,0,16,U
    #define VS_SEMAPHORE2_ADR               X:$6CEF7,0,16,U
    #define VS_FEED_OVERRIDE_ADR            F:$6CEF8
    #define VS_FEED_OVERRIDE2_ADR           F:$6CEF9
    #define VS_RAPID_OVERRIDE_ADR           F:$6CEFA
    #define VS_RAPID_OVERRIDE2_ADR          F:$6CEFB
    #define VS_THREAD_LEAD_ADR              F:$6CEFC
    #define VS_THREAD_LEAD2_ADR             F:$6CEFD
    #define VS_A_REL_ADR                  F:$6CEFE
    #define VS_B_REL_ADR                  F:$6CEFF
    #define VS_C_REL_ADR                  F:$6CF00
    #define VS_X_REL_ADR                  F:$6CF01
    #define VS_Y_REL_ADR                  F:$6CF02
    #define VS_Z_REL_ADR                  F:$6CF03
    #define VS_U_REL_ADR                  F:$6CF04
    #define VS_V_REL_ADR                  F:$6CF05
    #define VS_W_REL_ADR                  F:$6CF06
    #define VS_A_REL2_ADR                 F:$6CF07
    #define VS_B_REL2_ADR                 F:$6CF08
    #define VS_C_REL2_ADR                 F:$6CF09
    #define VS_X_REL2_ADR                 F:$6CF0A
    #define VS_Y_REL2_ADR                 F:$6CF0B
    #define VS_Z_REL2_ADR                 F:$6CF0C
    #define VS_U_REL2_ADR                 F:$6CF0D
    #define VS_V_REL2_ADR                 F:$6CF0E
    #define VS_W_REL2_ADR                 F:$6CF0F
    #define VS_A_ABS_ADR                  F:$6CF10
    #define VS_B_ABS_ADR                  F:$6CF11
    #define VS_C_ABS_ADR                  F:$6CF12
    #define VS_X_ABS_ADR                  F:$6CF13
    #define VS_Y_ABS_ADR                  F:$6CF14
    #define VS_Z_ABS_ADR                  F:$6CF15
    #define VS_U_ABS_ADR                  F:$6CF16
    #define VS_V_ABS_ADR                  F:$6CF17
    #define VS_W_ABS_ADR                  F:$6CF18
    #define VS_A_ABS2_ADR                 F:$6CF19
    #define VS_B_ABS2_ADR                 F:$6CF1A
    #define VS_C_ABS2_ADR                 F:$6CF1B
    #define VS_X_ABS2_ADR                 F:$6CF1C
    #define VS_Y_ABS2_ADR                 F:$6CF1D
    #define VS_Z_ABS2_ADR                 F:$6CF1E
    #define VS_U_ABS2_ADR                 F:$6CF1F
    #define VS_V_ABS2_ADR                 F:$6CF20
    #define VS_W_ABS2_ADR                 F:$6CF21
    #define VS_A_MUL_ADR                  F:$6CF22
    #define VS_B_MUL_ADR                  F:$6CF23
    #define VS_C_MUL_ADR                  F:$6CF24
    #define VS_X_MUL_ADR                  F:$6CF25
    #define VS_Y_MUL_ADR                  F:$6CF26
    #define VS_Z_MUL_ADR                  F:$6CF27
    #define VS_U_MUL_ADR                  F:$6CF28
    #define VS_V_MUL_ADR                  F:$6CF29
    #define VS_W_MUL_ADR                  F:$6CF2A
    #define VS_A_MUL2_ADR                 F:$6CF2B
    #define VS_B_MUL2_ADR                 F:$6CF2C
    #define VS_C_MUL2_ADR                 F:$6CF2D
    #define VS_X_MUL2_ADR                 F:$6CF2E
    #define VS_Y_MUL2_ADR                 F:$6CF2F
    #define VS_Z_MUL2_ADR                 F:$6CF30
    #define VS_U_MUL2_ADR                 F:$6CF31
    #define VS_V_MUL2_ADR                 F:$6CF32
    #define VS_W_MUL2_ADR                 F:$6CF33
    #define VS_NLABEL_ADR                 Y:$6CF34,0,16,U
    #define VS_NLABEL2_ADR                X:$6CF34,0,16,U
    #define VS_TOOL_OFFSET_ADR              Y:$6CF35,0,16,U
    #define VS_TOOL_OFFSET2_ADR             X:$6CF35,0,16,U
    #define VS_COMP_OFFSET_ADR              Y:$6CF51,0,16,U
    #define VS_COMP_OFFSET2_ADR             X:$6CF51,0,16,U
    #define VS_GGROUP0_1_ADR                  Y:$6CF36,0,16,U
    #define VS_GGROUP1_1_ADR                  X:$6CF36,0,16,U
    #define VS_GGROUP2_1_ADR                  Y:$6CF37,0,16,U
    #define VS_GGROUP3_1_ADR                  X:$6CF37,0,16,U
    #define VS_GGROUP4_1_ADR                  Y:$6CF38,0,16,U
    #define VS_GGROUP5_1_ADR                  X:$6CF38,0,16,U
    #define VS_GGROUP6_1_ADR                  Y:$6CF39,0,16,U
    #define VS_GGROUP7_1_ADR                  X:$6CF39,0,16,U
    #define VS_GGROUP8_1_ADR                  Y:$6CF3A,0,16,U
    #define VS_GGROUP9_1_ADR                  X:$6CF3A,0,16,U
    #define VS_GGROUP10_1_ADR                 Y:$6CF3B,0,16,U
    #define VS_GGROUP11_1_ADR                 X:$6CF3B,0,16,U
    #define VS_GGROUP12_1_ADR                 Y:$6CF3C,0,16,U
    #define VS_GGROUP13_1_ADR                 X:$6CF3C,0,16,U
    #define VS_GGROUP14_1_ADR                 Y:$6CF3D,0,16,U
    #define VS_GGROUP15_1_ADR                 X:$6CF3D,0,16,U
    #define VS_GGROUP16_1_ADR                 Y:$6CF3E,0,16,U
    #define VS_GGROUP17_1_ADR                 X:$6CF3E,0,16,U
    #define VS_GGROUP18_1_ADR                 Y:$6CF3F,0,16,U
    #define VS_GGROUP19_1_ADR                 X:$6CF3F,0,16,U
    #define VS_GGROUP20_1_ADR                 Y:$6CF40,0,16,U
    #define VS_GGROUP21_1_ADR                 X:$6CF40,0,16,U
    #define VS_GGROUP22_1_ADR                 Y:$6CF41,0,16,U
    #define VS_GGROUP23_1_ADR                 X:$6CF41,0,16,U
    #define VS_GGROUP24_1_ADR                 Y:$6CF42,0,16,U
    #define VS_GGROUP25_1_ADR                 Y:$6CF42,0,16,U
    #define VS_GGROUP0_2_ADR                  Y:$6CF43,0,16,U
    #define VS_GGROUP1_2_ADR                  X:$6CF43,0,16,U
    #define VS_GGROUP2_2_ADR                  Y:$6CF44,0,16,U
    #define VS_GGROUP3_2_ADR                  X:$6CF44,0,16,U
    #define VS_GGROUP4_2_ADR                  Y:$6CF45,0,16,U
    #define VS_GGROUP5_2_ADR                  X:$6CF45,0,16,U
    #define VS_GGROUP6_2_ADR                  Y:$6CF46,0,16,U
    #define VS_GGROUP7_2_ADR                  X:$6CF46,0,16,U
    #define VS_GGROUP8_2_ADR                  Y:$6CF47,0,16,U
    #define VS_GGROUP9_2_ADR                  X:$6CF47,0,16,U
    #define VS_GGROUP10_2_ADR                 Y:$6CF48,0,16,U
    #define VS_GGROUP11_2_ADR                 X:$6CF48,0,16,U
    #define VS_GGROUP12_2_ADR                 Y:$6CF49,0,16,U
    #define VS_GGROUP13_2_ADR                 X:$6CF49,0,16,U
    #define VS_GGROUP14_2_ADR                 Y:$6CF4A,0,16,U
    #define VS_GGROUP15_2_ADR                 X:$6CF4A,0,16,U
    #define VS_GGROUP16_2_ADR                 Y:$6CF4B,0,16,U
    #define VS_GGROUP17_2_ADR                 X:$6CF4B,0,16,U
    #define VS_GGROUP18_2_ADR                 Y:$6CF4C,0,16,U
    #define VS_GGROUP19_2_ADR                 X:$6CF4C,0,16,U
    #define VS_GGROUP20_2_ADR                 Y:$6CF4D,0,16,U
    #define VS_GGROUP21_2_ADR                 X:$6CF4D,0,16,U
    #define VS_GGROUP22_2_ADR                 Y:$6CF4E,0,16,U
    #define VS_GGROUP23_2_ADR                 X:$6CF4E,0,16,U
    #define VS_GGROUP24_2_ADR                 Y:$6CF4F,0,16,U
    #define VS_GGROUP25_2_ADR                 Y:$6CF4F,0,16,U
    #define ES_PC_CLOCK_ADR                DP:$6CF60
    #define ES_ERR_FATAL_ADR               DP:$6CF61
    #define ES_ERR_FATAL2_ADR              DP:$6CF62
    #define ES_ERR_STOP_ADR                DP:$6CF63
    #define ES_ERR_STOP2_ADR               DP:$6CF64
    #define ES_ERR_WARN_ADR                DP:$6CF65
    #define ES_ERR_WARN2_ADR               DP:$6CF66
    #define ES_ERR_MSG_ADR                 DP:$6CF67
    #define ES_ERR_MSG2_ADR                DP:$6CF68
    #define ES_PLCMSGBOX_ADR               DP:$6CF69
    #define ES_SCANAVE_ADR                 X:$6CF70,0,16,U
    #define ES_SCANMAX_ADR                 Y:$6CF70,0,16,U

    // USER BAR LOCATIONS
    #define PC_UB_CMD_BOX1                 0x3DA8
    #define UB_CMD_BOX1_ADR                DP:$6CF6A
    #define UB_CMD_BOX1_STATUS             Y:$6CF6A,0,4,U
    #define UB_CMD_BOX1_TYPE               Y:$6CF6A,4,4,U
    #define UB_CMD_BOX1_COLR_TX            Y:$6CF6A,8,4,U
    #define UB_CMD_BOX1_COLR_BG            Y:$6CF6A,12,4,U
    #Define UB_CMD_BOX1_DEF_ID             X:$6CF6A,0,8,U
                                        // X:$6CF6A,8,8 not in use
    #define PC_UB_DATA_BOX1                0x3DAC
    #define UB_DATA_BOX1_DW                DP:$6CF6B
    #define UB_DATA_BOX1_F                 F:$6CF6B

    #define PC_UB_CMD_BOX2                 0x3DB0
    #define UB_CMD_BOX2_ADR                DP:$6CF6C
    #define UB_CMD_BOX2_STATUS             Y:$6CF6C,0,4,U
    #define UB_CMD_BOX2_TYPE               Y:$6CF6C,4,4,U
    #define UB_CMD_BOX2_COLR_TX            Y:$6CF6C,8,4,U
    #define UB_CMD_BOX2_COLR_BG            Y:$6CF6C,12,4,U
    #Define UB_CMD_BOX2_DEF_ID             X:$6CF6C,0,8,U
                                        // X:$6CF6C,8,8 not in use
    #define PC_UB_DATA_BOX2                0x3DB4
    #define UB_DATA_BOX2_DW                DP:$6CF6D
    #define UB_DATA_BOX2_F                 F:$6CF6D

    #define PC_UB_CMD_BOX3                 0x3DB8
    #define UB_CMD_BOX3_ADR                DP:$6CF6E
    #define UB_CMD_BOX3_STATUS             Y:$6CF6E,0,4,U
    #define UB_CMD_BOX3_TYPE               Y:$6CF6E,4,4,U
    #define UB_CMD_BOX3_COLR_TX            Y:$6CF6E,8,4,U
    #define UB_CMD_BOX3_COLR_BG            Y:$6CF6E,12,4,U
    #Define UB_CMD_BOX3_DEF_ID             X:$6CF6E,0,8,U
                                        // X:$6CF6E,8,8 not in use
    #define PC_UB_DATA_BOX3                0x3DBC
    #define UB_DATA_BOX3_DW                DP:$6CF6F
    #define UB_DATA_BOX3_F                 F:$6CF6F

    #define PC_UB_CMD_BOX4                 0x3DC0
    #define UB_CMD_BOX4_ADR                DP:$6CF70
    #define UB_CMD_BOX4_STATUS             Y:$6CF70,0,4,U
    #define UB_CMD_BOX4_TYPE               Y:$6CF70,4,4,U
    #define UB_CMD_BOX4_COLR_TX            Y:$6CF70,8,4,U
    #define UB_CMD_BOX4_COLR_BG            Y:$6CF70,12,4,U
    #Define UB_CMD_BOX4_DEF_ID             X:$6CF70,0,8,U
                                        // X:$6CF70,8,8 not in use
    #define PC_UB_DATA_BOX4                0x3DC4
    #define UB_DATA_BOX4_DW                DP:$6CF71
    #define UB_DATA_BOX4_F                 F:$6CF71

    #define PC_UB_CMD_BOX5                 0x3DC8
    #define UB_CMD_BOX5_ADR                DP:$6CF72
    #define UB_CMD_BOX5_STATUS             Y:$6CF72,0,4,U
    #define UB_CMD_BOX5_TYPE               Y:$6CF72,4,4,U
    #define UB_CMD_BOX5_COLR_TX            Y:$6CF72,8,4,U
    #define UB_CMD_BOX5_COLR_BG            Y:$6CF72,12,4,U
    #Define UB_CMD_BOX5_DEF_ID             X:$6CF72,0,8,U
                                        // X:$6CF72,8,8 not in use
    #define PC_UB_DATA_BOX5                0x3DCC
    #define UB_DATA_BOX5_DW                DP:$6CF73
    #define UB_DATA_BOX5_F                 F:$6CF73

    #define PC_UB_CMD_BOX6                 0x3DD0
    #define UB_CMD_BOX6_ADR                DP:$6CF74
    #define UB_CMD_BOX6_STATUS             Y:$6CF74,0,4,U
    #define UB_CMD_BOX6_TYPE               Y:$6CF74,4,4,U
    #define UB_CMD_BOX6_COLR_TX            Y:$6CF74,8,4,U
    #define UB_CMD_BOX6_COLR_BG            Y:$6CF74,12,4,U
    #Define UB_CMD_BOX6_DEF_ID             X:$6CF74,0,8,U
                                        // X:$6CF74,8,8 not in use
    #define PC_UB_DATA_BOX6                0x3DD4
    #define UB_DATA_BOX6_DW                DP:$6CF75
    #define UB_DATA_BOX6_F                 F:$6CF75

    #define PC_UB_CMD_BOX7                 0x3DD8
    #define UB_CMD_BOX7_ADR                DP:$6CF76
    #define UB_CMD_BOX7_STATUS             Y:$6CF76,0,4,U
    #define UB_CMD_BOX7_TYPE               Y:$6CF76,4,4,U
    #define UB_CMD_BOX7_COLR_TX            Y:$6CF76,8,4,U
    #define UB_CMD_BOX7_COLR_BG            Y:$6CF76,12,4,U
    #Define UB_CMD_BOX7_DEF_ID             X:$6CF76,0,8,U
                                        // X:$6CF76,8,8 not in use
    #define PC_UB_DATA_BOX7                0x3DDC
    #define UB_DATA_BOX7_DW                DP:$6CF77
    #define UB_DATA_BOX7_F                 F:$6CF77
    // END OF USER BAR LOCATIONS

    #define IN_1_ADR           DP:$6CFE0
    #define IN_1_CHNG_ADR       DP:$6CFF0
    #define IN_2_ADR           DP:$6CFE1
    #define IN_2_CHNG_ADR       DP:$6CFF1
    #define IN_3_ADR           DP:$6CFE2
    #define IN_3_CHNG_ADR       DP:$6CFF2
    #define IN_4_ADR           DP:$6CFE3
    #define IN_4_CHNG_ADR       DP:$6CFF3
    #define IN_5_ADR           DP:$6CFE4
    #define IN_5_CHNG_ADR       DP:$6CFF4
    #define IN_6_ADR           DP:$6CFE5
    #define IN_6_CHNG_ADR       DP:$6CFF5
    #define IN_7_ADR           DP:$6CFE6
    #define IN_7_CHNG_ADR       DP:$6CFF6
    #define IN_8_ADR           DP:$6CFE7
    #define IN_8_CHNG_ADR       DP:$6CFF7
    #define OUT_1_ADR          DP:$6CFE8
    #define OUT_1_CHNG_ADR      DP:$6CFF8
    #define OUT_2_ADR          DP:$6CFE9
    #define OUT_2_CHNG_ADR      DP:$6CFF9
    #define OUT_3_ADR          DP:$6CFEA
    #define OUT_3_CHNG_ADR      DP:$6CFFA
    #define OUT_4_ADR          DP:$6CFEB
    #define OUT_4_CHNG_ADR      DP:$6CFFB
    #define OUT_5_ADR          DP:$6CFEC
    #define OUT_5_CHNG_ADR      DP:$6CFFC
    #define OUT_6_ADR          DP:$6CFED
    #define OUT_6_CHNG_ADR     DP:$6CFFD
    #define OUT_7_ADR          DP:$6CFEE
    #define OUT_7_CHNG_ADR     DP:$6CFFE
    #define OUT_8_ADR          DP:$6CFEF
    #define OUT_8_CHNG_ADR     DP:$6CFFF
  #else 
    #ifdef _QMAC_
//-----------------------------------------------------
      #define DPR_BASE            $60000
      #define CS_COMMAND1_ADR     DP:$60DF0
      #define CS_STATUS1_ADR      DP:$60DE0
      #define S_JOG_ADR           Y:$60DE0,0,4,U
      #define C_JOG_ADR           Y:$60DF0,0,4,U
      #define S_MODE_ADR          Y:$60DE0,4,4,U
      #define C_MODE_ADR          Y:$60DF0,4,4,U
      #define S_AXIS_ADR          Y:$60DE0,8,4,U
      #define C_AXIS_ADR          Y:$60DF0,8,4,U
      #define S_FEED_ADR          Y:$60DE0,12,4,U
      #define C_FEED_ADR          Y:$60DF0,12,4,U
      #define S_ROVRD_ADR         X:$60DE0,0,4,U
      #define C_ROVRD_ADR         X:$60DF0,0,4,U
      #define S_SPND_ADR          X:$60DE0,4,4,U
      #define C_SPND_ADR          X:$60DF0,4,4,U
      #define S_FOVRD_ADR         X:$60DE0,8,4,U
      #define C_FOVRD_ADR         X:$60DF0,8,4,U
      #define S_SOVRD_ADR         X:$60DE0,12,4,U
      #define C_SOVRD_ADR         X:$60DF0,12,4,U
      #define CS_STATUS2_ADR      DP:$60DE1
      #define CS_COMMAND2_ADR     DP:$60DF1
      #define S_COORD_ADR         Y:$60DE1,0,4,U
      #define C_COORD_ADR         Y:$60DF1,0,4,U
      #define S_GEAR_ADR          Y:$60DE1,4,4,U
      #define C_GEAR_ADR          Y:$60DF1,4,4,U
      #define S_HOME_ADR          Y:$60DE1,8,8,U
      #define C_HOME_ADR          Y:$60DF1,8,8,U
      #define S_ERROR_ADR         X:$60DE1,0,4,U
      #define C_ERROR_ADR         X:$60DF1,0,4,U
      #define S_CLNT_ADR          X:$60DE1,4,4,U
      #define C_CLNT_ADR          X:$60DF1,4,4,U
      #define S_DIST_ADR          X:$60DE1,8,4,U
      #define C_DIST_ADR          X:$60DF1,8,4,U
      #define S_MACHINE_ADR       X:$60DE1,12,4,U
      #define C_MACHINE_ADR       X:$60DF1,12,4,U
      #define CS_STATUS3_ADR      DP:$60DE2
      #define CS_COMMAND3_ADR     DP:$60DF2
      #define CS_STATUS4_ADR      DP:$60DE3
      #define CS_COMMAND4_ADR     DP:$60DF3
      #define CS_STATUS5_ADR      DP:$60DE4
      #define CS_COMMAND5_ADR     DP:$60DF4
      #define S_MOTION_ADR            Y:$60DE4,0,4,U
      #define C_MOTION_ADR            Y:$60DF4,0,4,U
      #define S_TIMEBASE_ADR          Y:$60DE4,4,4,U
      #define C_TIMEBASE_ADR          Y:$60DF4,4,4,U
      #define S_PROG_STATUS_ADR       Y:$60DE4,8,4,U
      #define C_PROG_STATUS_ADR       Y:$60DF4,8,4,U
      #define CS_STATUS6_ADR          DP:$60DE5
      #define CS_COMMAND6_ADR         DP:$60DF5
      #define CS_STATUS7_ADR          DP:$60DE6
      #define CS_COMMAND7_ADR         DP:$60DF6
      #define CS_STATUS8_ADR          DP:$60DE7
      #define CS_COMMAND8_ADR         DP:$60DF7
      #define PR_BITS_ADR             DP:$60DFD
      #define PR_COMMAND_ADR          DP:$60DFE
      #define PR_DATA_ADR             F:$60DFF

      #define AT_DPR_POINTER_ADR      DP:$60FFF      // AS DEFAULT, POINT TO UNLIKELY TO BE USED
      #define DPR_POINTER_ADR         Y:$0040F8,0,16 // POINTS TO M248 ADDRESSING PART

      #define G65_LEVEL_ADR           DP:$60F52
      #define G65_LEVEL2_ADR          DP:$60F53

      #define PMAC_RT_BLOCK_ADR       DP:$60F54
      #define PMAC_RT_BLOCK2_ADR      DP:$60F55
      #define PMAC_RT_NM_BLOCK_ADR    DP:$60F56
      #define PMAC_RT_NM_BLOCK2_ADR   DP:$60F57

      #define PMAC_RESET_COMP_ADR     Y:$60F58,0,16,U
      #define PC_RESET_COMP_ADR       X:$60F58,0,16,U

      #define ROT_BUF_FIL_STOP_ADR    Y:$60F59,0,16,U
      #define UNUSED_13_ADR           X:$60F59,0,16,U

      #define VS_L_NLABEL_ADR         DP:$60F5A
      #define VS_L_NLABEL2_ADR        DP:$60F5B

      #define VS_CURRENT_PROG_ADR             DP:$60EE0
      #define VS_CURRENT_PROG2_ADR            DP:$60EE1
      #define VS_CURRENT_BLOCK_ADR            DP:$60EE2
      #define VS_CURRENT_BLOCK2_ADR           DP:$60EE3
      #define VS_LAST_BLOCK_ADR               DP:$60EE4
      #define VS_LAST_BLOCK2_ADR              DP:$60EE5
      #define VS_SPINDLE_RPM_ADR              DP:$60EE6
      #define VS_SPINDLE_MAX_RPM_ADR          DP:$60EE7
      #define VS_SPINDLE_MAX_LIM_ADR          DP:$60EE8
      #define VS_SPINDLE_CMD_RPM_ADR          DP:$60EE9
      #define VS_SPINDLE_ACT_RPM_ADR          DP:$60EEA
      #define VS_SPINDLE_COUNTS_REV_ADR       F:$60EEB
      #define VS_SPINDLE_CSS_ADR              F:$60EEC
      #define VS_SPINDLE_CSS_UNITS_ADR        F:$60EED
      #define VS_GEAR_RATIO_ADR               F:$60EEE
      #define VS_SPINDLE_OVERRIDE_ADR         Y:$60EEF,0,16,U
      #define VS_BLOCKS_SEC_ADR               X:$60EEF,0,16,U
      #define VS_TURRET2_TOOL_ADR             Y:$60EF0,0,16,U
      #define VS_TURRET_TOOL_ADR              X:$60EF0,0,16,U
      #define VS_TURRET_POT_ADR               Y:$60EF1,0,16,U
      #define VS_TURRET2_POT_ADR              X:$60EF1,0,16,U
      #define VS_TOOL_NUMBER_ADR              Y:$60EF2,0,16,U
      #define VS_TOOL_NUMBER2_ADR             X:$60EF2,0,16,U
      #define VS_TOOL_NEXT_ADR                Y:$60EF3,0,16,U
      #define VS_TOOL_NEXT2_ADR               X:$60EF3,0,16,U
      #define VS_HANDLE_LINC_ADR              F:$60EF4
      #define VS_JOG_STEP_ADR                 F:$60EF5
      #define VS_HAND_STEP_ADR                F:$60EF6
      #define VS_SEMAPHORE_ADR                Y:$60EF7,0,16,U
      #define VS_SEMAPHORE2_ADR               X:$60EF7,0,16,U
      #define VS_FEED_OVERRIDE_ADR            F:$60EF8
      #define VS_FEED_OVERRIDE2_ADR           F:$60EF9
      #define VS_RAPID_OVERRIDE_ADR           F:$60EFA
      #define VS_RAPID_OVERRIDE2_ADR          F:$60EFB
      #define VS_THREAD_LEAD_ADR              F:$60EFC
      #define VS_THREAD_LEAD2_ADR             F:$60EFD
      #define VS_A_REL_ADR                  F:$60EFE
      #define VS_B_REL_ADR                  F:$60EFF
      #define VS_C_REL_ADR                  F:$60F00
      #define VS_X_REL_ADR                  F:$60F01
      #define VS_Y_REL_ADR                  F:$60F02
      #define VS_Z_REL_ADR                  F:$60F03
      #define VS_U_REL_ADR                  F:$60F04
      #define VS_V_REL_ADR                  F:$60F05
      #define VS_W_REL_ADR                  F:$60F06
      #define VS_A_REL2_ADR                 F:$60F07
      #define VS_B_REL2_ADR                 F:$60F08
      #define VS_C_REL2_ADR                 F:$60F09
      #define VS_X_REL2_ADR                 F:$60F0A
      #define VS_Y_REL2_ADR                 F:$60F0B
      #define VS_Z_REL2_ADR                 F:$60F0C
      #define VS_U_REL2_ADR                 F:$60F0D
      #define VS_V_REL2_ADR                 F:$60F0E
      #define VS_W_REL2_ADR                 F:$60F0F
      #define VS_A_ABS_ADR                  F:$60F10
      #define VS_B_ABS_ADR                  F:$60F11
      #define VS_C_ABS_ADR                  F:$60F12
      #define VS_X_ABS_ADR                  F:$60F13
      #define VS_Y_ABS_ADR                  F:$60F14
      #define VS_Z_ABS_ADR                  F:$60F15
      #define VS_U_ABS_ADR                  F:$60F16
      #define VS_V_ABS_ADR                  F:$60F17
      #define VS_W_ABS_ADR                  F:$60F18
      #define VS_A_ABS2_ADR                 F:$60F19
      #define VS_B_ABS2_ADR                 F:$60F1A
      #define VS_C_ABS2_ADR                 F:$60F1B
      #define VS_X_ABS2_ADR                 F:$60F1C
      #define VS_Y_ABS2_ADR                 F:$60F1D
      #define VS_Z_ABS2_ADR                 F:$60F1E
      #define VS_U_ABS2_ADR                 F:$60F1F
      #define VS_V_ABS2_ADR                 F:$60F20
      #define VS_W_ABS2_ADR                 F:$60F21
      #define VS_A_MUL_ADR                  F:$60F22
      #define VS_B_MUL_ADR                  F:$60F23
      #define VS_C_MUL_ADR                  F:$60F24
      #define VS_X_MUL_ADR                  F:$60F25
      #define VS_Y_MUL_ADR                  F:$60F26
      #define VS_Z_MUL_ADR                  F:$60F27
      #define VS_U_MUL_ADR                  F:$60F28
      #define VS_V_MUL_ADR                  F:$60F29
      #define VS_W_MUL_ADR                  F:$60F2A
      #define VS_A_MUL2_ADR                 F:$60F2B
      #define VS_B_MUL2_ADR                 F:$60F2C
      #define VS_C_MUL2_ADR                 F:$60F2D
      #define VS_X_MUL2_ADR                 F:$60F2E
      #define VS_Y_MUL2_ADR                 F:$60F2F
      #define VS_Z_MUL2_ADR                 F:$60F30
      #define VS_U_MUL2_ADR                 F:$60F31
      #define VS_V_MUL2_ADR                 F:$60F32
      #define VS_W_MUL2_ADR                 F:$60F33
      #define VS_NLABEL_ADR                 Y:$60F34,0,16,U
      #define VS_NLABEL2_ADR                X:$60F34,0,16,U
      #define VS_TOOL_OFFSET_ADR              Y:$60F35,0,16,U
      #define VS_TOOL_OFFSET2_ADR             X:$60F35,0,16,U
      #define VS_COMP_OFFSET_ADR              Y:$60F51,0,16,U
      #define VS_COMP_OFFSET2_ADR             X:$60F51,0,16,U
      #define VS_GGROUP0_1_ADR                  Y:$60F36,0,16,U
      #define VS_GGROUP1_1_ADR                  X:$60F36,0,16,U
      #define VS_GGROUP2_1_ADR                  Y:$60F37,0,16,U
      #define VS_GGROUP3_1_ADR                  X:$60F37,0,16,U
      #define VS_GGROUP4_1_ADR                  Y:$60F38,0,16,U
      #define VS_GGROUP5_1_ADR                  X:$60F38,0,16,U
      #define VS_GGROUP6_1_ADR                  Y:$60F39,0,16,U
      #define VS_GGROUP7_1_ADR                  X:$60F39,0,16,U
      #define VS_GGROUP8_1_ADR                  Y:$60F3A,0,16,U
      #define VS_GGROUP9_1_ADR                  X:$60F3A,0,16,U
      #define VS_GGROUP10_1_ADR                 Y:$60F3B,0,16,U
      #define VS_GGROUP11_1_ADR                 X:$60F3B,0,16,U
      #define VS_GGROUP12_1_ADR                 Y:$60F3C,0,16,U
      #define VS_GGROUP13_1_ADR                 X:$60F3C,0,16,U
      #define VS_GGROUP14_1_ADR                 Y:$60F3D,0,16,U
      #define VS_GGROUP15_1_ADR                 X:$60F3D,0,16,U
      #define VS_GGROUP16_1_ADR                 Y:$60F3E,0,16,U
      #define VS_GGROUP17_1_ADR                 X:$60F3E,0,16,U
      #define VS_GGROUP18_1_ADR                 Y:$60F3F,0,16,U
      #define VS_GGROUP19_1_ADR                 X:$60F3F,0,16,U
      #define VS_GGROUP20_1_ADR                 Y:$60F40,0,16,U
      #define VS_GGROUP21_1_ADR                 X:$60F40,0,16,U
      #define VS_GGROUP22_1_ADR                 Y:$60F41,0,16,U
      #define VS_GGROUP23_1_ADR                 X:$60F41,0,16,U
      #define VS_GGROUP24_1_ADR                 Y:$60F42,0,16,U
      #define VS_GGROUP25_1_ADR                 Y:$60F42,0,16,U
      #define VS_GGROUP0_2_ADR                  Y:$60F43,0,16,U
      #define VS_GGROUP1_2_ADR                  X:$60F43,0,16,U
      #define VS_GGROUP2_2_ADR                  Y:$60F44,0,16,U
      #define VS_GGROUP3_2_ADR                  X:$60F44,0,16,U
      #define VS_GGROUP4_2_ADR                  Y:$60F45,0,16,U
      #define VS_GGROUP5_2_ADR                  X:$60F45,0,16,U
      #define VS_GGROUP6_2_ADR                  Y:$60F46,0,16,U
      #define VS_GGROUP7_2_ADR                  X:$60F46,0,16,U
      #define VS_GGROUP8_2_ADR                  Y:$60F47,0,16,U
      #define VS_GGROUP9_2_ADR                  X:$60F47,0,16,U
      #define VS_GGROUP10_2_ADR                 Y:$60F48,0,16,U
      #define VS_GGROUP11_2_ADR                 X:$60F48,0,16,U
      #define VS_GGROUP12_2_ADR                 Y:$60F49,0,16,U
      #define VS_GGROUP13_2_ADR                 X:$60F49,0,16,U
      #define VS_GGROUP14_2_ADR                 Y:$60F4A,0,16,U
      #define VS_GGROUP15_2_ADR                 X:$60F4A,0,16,U
      #define VS_GGROUP16_2_ADR                 Y:$60F4B,0,16,U
      #define VS_GGROUP17_2_ADR                 X:$60F4B,0,16,U
      #define VS_GGROUP18_2_ADR                 Y:$60F4C,0,16,U
      #define VS_GGROUP19_2_ADR                 X:$60F4C,0,16,U
      #define VS_GGROUP20_2_ADR                 Y:$60F4D,0,16,U
      #define VS_GGROUP21_2_ADR                 X:$60F4D,0,16,U
      #define VS_GGROUP22_2_ADR                 Y:$60F4E,0,16,U
      #define VS_GGROUP23_2_ADR                 X:$60F4E,0,16,U
      #define VS_GGROUP24_2_ADR                 Y:$60F4F,0,16,U
      #define VS_GGROUP25_2_ADR                 Y:$60F4F,0,16,U
      #define ES_PC_CLOCK_ADR                DP:$60F60
      #define ES_ERR_FATAL_ADR               DP:$60F61
      #define ES_ERR_FATAL2_ADR              DP:$60F62
      #define ES_ERR_STOP_ADR                DP:$60F63
      #define ES_ERR_STOP2_ADR               DP:$60F64
      #define ES_ERR_WARN_ADR                DP:$60F65
      #define ES_ERR_WARN2_ADR               DP:$60F66
      #define ES_ERR_MSG_ADR                 DP:$60F67
      #define ES_ERR_MSG2_ADR                DP:$60F68
      #define ES_PLCMSGBOX_ADR               DP:$60F69
      #define ES_SCANAVE_ADR                 X:$60F70,0,16,U
      #define ES_SCANMAX_ADR                 Y:$60F70,0,16,U

      // USER BAR LOCATIONS
      #define PC_UB_CMD_BOX1                 0x3DA8
      #define UB_CMD_BOX1_ADR                DP:$60F6A
      #define UB_CMD_BOX1_STATUS             Y:$60F6A,0,4,U
      #define UB_CMD_BOX1_TYPE               Y:$60F6A,4,4,U
      #define UB_CMD_BOX1_COLR_TX            Y:$60F6A,8,4,U
      #define UB_CMD_BOX1_COLR_BG            Y:$60F6A,12,4,U
      #Define UB_CMD_BOX1_DEF_ID             X:$60F6A,0,8,U
                                          // X:$60F6A,8,8 not in use
      #define PC_UB_DATA_BOX1                0x3DAC
      #define UB_DATA_BOX1_DW                DP:$60F6B
      #define UB_DATA_BOX1_F                 F:$60F6B

      #define PC_UB_CMD_BOX2                 0x3DB0
      #define UB_CMD_BOX2_ADR                DP:$60F6C
      #define UB_CMD_BOX2_STATUS             Y:$60F6C,0,4,U
      #define UB_CMD_BOX2_TYPE               Y:$60F6C,4,4,U
      #define UB_CMD_BOX2_COLR_TX            Y:$60F6C,8,4,U
      #define UB_CMD_BOX2_COLR_BG            Y:$60F6C,12,4,U
      #Define UB_CMD_BOX2_DEF_ID             X:$60F6C,0,8,U
                                          // X:$60F6C,8,8 not in use
      #define PC_UB_DATA_BOX2                0x3DB4
      #define UB_DATA_BOX2_DW                DP:$60F6D
      #define UB_DATA_BOX2_F                 F:$60F6D

      #define PC_UB_CMD_BOX3                 0x3DB8
      #define UB_CMD_BOX3_ADR                DP:$60F6E
      #define UB_CMD_BOX3_STATUS             Y:$60F6E,0,4,U
      #define UB_CMD_BOX3_TYPE               Y:$60F6E,4,4,U
      #define UB_CMD_BOX3_COLR_TX            Y:$60F6E,8,4,U
      #define UB_CMD_BOX3_COLR_BG            Y:$60F6E,12,4,U
      #Define UB_CMD_BOX3_DEF_ID             X:$60F6E,0,8,U
                                          // X:$60F6E,8,8 not in use
      #define PC_UB_DATA_BOX3                0x3DBC
      #define UB_DATA_BOX3_DW                DP:$60F6F
      #define UB_DATA_BOX3_F                 F:$60F6F

      #define PC_UB_CMD_BOX4                 0x3DC0
      #define UB_CMD_BOX4_ADR                DP:$60F70
      #define UB_CMD_BOX4_STATUS             Y:$60F70,0,4,U
      #define UB_CMD_BOX4_TYPE               Y:$60F70,4,4,U
      #define UB_CMD_BOX4_COLR_TX            Y:$60F70,8,4,U
      #define UB_CMD_BOX4_COLR_BG            Y:$60F70,12,4,U
      #Define UB_CMD_BOX4_DEF_ID             X:$60F70,0,8,U
                                          // X:$60F70,8,8 not in use
      #define PC_UB_DATA_BOX4                0x3DC4
      #define UB_DATA_BOX4_DW                DP:$60F71
      #define UB_DATA_BOX4_F                 F:$60F71

      #define PC_UB_CMD_BOX5                 0x3DC8
      #define UB_CMD_BOX5_ADR                DP:$60F72
      #define UB_CMD_BOX5_STATUS             Y:$60F72,0,4,U
      #define UB_CMD_BOX5_TYPE               Y:$60F72,4,4,U
      #define UB_CMD_BOX5_COLR_TX            Y:$60F72,8,4,U
      #define UB_CMD_BOX5_COLR_BG            Y:$60F72,12,4,U
      #Define UB_CMD_BOX5_DEF_ID             X:$60F72,0,8,U
                                          // X:$60F72,8,8 not in use
      #define PC_UB_DATA_BOX5                0x3DCC
      #define UB_DATA_BOX5_DW                DP:$60F73
      #define UB_DATA_BOX5_F                 F:$60F73

      #define PC_UB_CMD_BOX6                 0x3DD0
      #define UB_CMD_BOX6_ADR                DP:$60F74
      #define UB_CMD_BOX6_STATUS             Y:$60F74,0,4,U
      #define UB_CMD_BOX6_TYPE               Y:$60F74,4,4,U
      #define UB_CMD_BOX6_COLR_TX            Y:$60F74,8,4,U
      #define UB_CMD_BOX6_COLR_BG            Y:$60F74,12,4,U
      #Define UB_CMD_BOX6_DEF_ID             X:$60F74,0,8,U
                                          // X:$60F74,8,8 not in use
      #define PC_UB_DATA_BOX6                0x3DD4
      #define UB_DATA_BOX6_DW                DP:$60F75
      #define UB_DATA_BOX6_F                 F:$60F75

      #define PC_UB_CMD_BOX7                 0x3DD8
      #define UB_CMD_BOX7_ADR                DP:$60F76
      #define UB_CMD_BOX7_STATUS             Y:$60F76,0,4,U
      #define UB_CMD_BOX7_TYPE               Y:$60F76,4,4,U
      #define UB_CMD_BOX7_COLR_TX            Y:$60F76,8,4,U
      #define UB_CMD_BOX7_COLR_BG            Y:$60F76,12,4,U
      #Define UB_CMD_BOX7_DEF_ID             X:$60F76,0,8,U
                                          // X:$60F76,8,8 not in use
      #define PC_UB_DATA_BOX7                0x3DDC
      #define UB_DATA_BOX7_DW                DP:$60F77
      #define UB_DATA_BOX7_F                 F:$60F77
      // END OF USER BAR LOCATIONS

      #define IN_1_ADR           DP:$60FE0
      #define IN_1_CHNG_ADR       DP:$60FF0
      #define IN_2_ADR           DP:$60FE1
      #define IN_2_CHNG_ADR       DP:$60FF1
      #define IN_3_ADR           DP:$60FE2
      #define IN_3_CHNG_ADR       DP:$60FF2
      #define IN_4_ADR           DP:$60FE3
      #define IN_4_CHNG_ADR       DP:$60FF3
      #define IN_5_ADR           DP:$60FE4
      #define IN_5_CHNG_ADR       DP:$60FF4
      #define IN_6_ADR           DP:$60FE5
      #define IN_6_CHNG_ADR       DP:$60FF5
      #define IN_7_ADR           DP:$60FE6
      #define IN_7_CHNG_ADR       DP:$60FF6
      #define IN_8_ADR           DP:$60FE7
      #define IN_8_CHNG_ADR       DP:$60FF7
      #define OUT_1_ADR          DP:$60FE8
      #define OUT_1_CHNG_ADR      DP:$60FF8
      #define OUT_2_ADR          DP:$60FE9
      #define OUT_2_CHNG_ADR      DP:$60FF9
      #define OUT_3_ADR          DP:$60FEA
      #define OUT_3_CHNG_ADR      DP:$60FFA
      #define OUT_4_ADR          DP:$60FEB
      #define OUT_4_CHNG_ADR      DP:$60FFB
      #define OUT_5_ADR          DP:$60FEC
      #define OUT_5_CHNG_ADR      DP:$60FFC
      #define OUT_6_ADR          DP:$60FED
      #define OUT_6_CHNG_ADR     DP:$60FFD
      #define OUT_7_ADR          DP:$60FEE
      #define OUT_7_CHNG_ADR     DP:$60FFE
      #define OUT_8_ADR          DP:$60FEF
      #define OUT_8_CHNG_ADR     DP:$60FFF

//-----------------------------------------------------
    #else
      // NON UMAC
      #define DPR_BASE            $60000
      #define CS_COMMAND1_ADR     DP:$60DF0
      #define CS_STATUS1_ADR      DP:$60DE0
      #define S_JOG_ADR           Y:$60DE0,0,4,U
      #define C_JOG_ADR           Y:$60DF0,0,4,U
      #define S_MODE_ADR          Y:$60DE0,4,4,U
      #define C_MODE_ADR          Y:$60DF0,4,4,U
      #define S_AXIS_ADR          Y:$60DE0,8,4,U
      #define C_AXIS_ADR          Y:$60DF0,8,4,U
      #define S_FEED_ADR          Y:$60DE0,12,4,U
      #define C_FEED_ADR          Y:$60DF0,12,4,U
      #define S_ROVRD_ADR         X:$60DE0,0,4,U
      #define C_ROVRD_ADR         X:$60DF0,0,4,U
      #define S_SPND_ADR          X:$60DE0,4,4,U
      #define C_SPND_ADR          X:$60DF0,4,4,U
      #define S_FOVRD_ADR         X:$60DE0,8,4,U
      #define C_FOVRD_ADR         X:$60DF0,8,4,U
      #define S_SOVRD_ADR         X:$60DE0,12,4,U
      #define C_SOVRD_ADR         X:$60DF0,12,4,U
      #define CS_STATUS2_ADR      DP:$60DE1
      #define CS_COMMAND2_ADR     DP:$60DF1
      #define S_COORD_ADR         Y:$60DE1,0,4,U
      #define C_COORD_ADR         Y:$60DF1,0,4,U
      #define S_GEAR_ADR          Y:$60DE1,4,4,U
      #define C_GEAR_ADR          Y:$60DF1,4,4,U
      #define S_HOME_ADR          Y:$60DE1,8,8,U
      #define C_HOME_ADR          Y:$60DF1,8,8,U
      #define S_ERROR_ADR         X:$60DE1,0,4,U
      #define C_ERROR_ADR         X:$60DF1,0,4,U
      #define S_CLNT_ADR          X:$60DE1,4,4,U
      #define C_CLNT_ADR          X:$60DF1,4,4,U
      #define S_DIST_ADR          X:$60DE1,8,4,U
      #define C_DIST_ADR          X:$60DF1,8,4,U
      #define S_MACHINE_ADR       X:$60DE1,12,4,U
      #define C_MACHINE_ADR       X:$60DF1,12,4,U
      #define CS_STATUS3_ADR      DP:$60DE2
      #define CS_COMMAND3_ADR     DP:$60DF2
      #define CS_STATUS4_ADR      DP:$60DE3
      #define CS_COMMAND4_ADR     DP:$60DF3
      #define CS_STATUS5_ADR      DP:$60DE4
      #define CS_COMMAND5_ADR     DP:$60DF4
      #define S_MOTION_ADR            Y:$60DE4,0,4,U
      #define C_MOTION_ADR            Y:$60DF4,0,4,U
      #define S_TIMEBASE_ADR          Y:$60DE4,4,4,U
      #define C_TIMEBASE_ADR          Y:$60DF4,4,4,U
      #define S_PROG_STATUS_ADR       Y:$60DE4,8,4,U
      #define C_PROG_STATUS_ADR       Y:$60DF4,8,4,U
      #define CS_STATUS6_ADR          DP:$60DE5
      #define CS_COMMAND6_ADR         DP:$60DF5
      #define CS_STATUS7_ADR          DP:$60DE6
      #define CS_COMMAND7_ADR         DP:$60DF6
      #define CS_STATUS8_ADR          DP:$60DE7
      #define CS_COMMAND8_ADR         DP:$60DF7
      #define PR_BITS_ADR             DP:$60DFD
      #define PR_COMMAND_ADR          DP:$60DFE
      #define PR_DATA_ADR             F:$60DFF

      #define AT_DPR_POINTER_ADR      DP:$60FFF      // AS DEFAULT, POINT TO UNLIKELY TO BE USED
      #define DPR_POINTER_ADR         Y:$0040F8,0,16 // POINTS TO M248 ADDRESSING PART

      #define G65_LEVEL_ADR           DP:$60F52
      #define G65_LEVEL2_ADR          DP:$60F53

      #define PMAC_RT_BLOCK_ADR       DP:$60F54
      #define PMAC_RT_BLOCK2_ADR      DP:$60F55
      #define PMAC_RT_NM_BLOCK_ADR    DP:$60F56
      #define PMAC_RT_NM_BLOCK2_ADR   DP:$60F57

      #define PMAC_RESET_COMP_ADR     Y:$60F58,0,16,U
      #define PC_RESET_COMP_ADR       X:$60F58,0,16,U

      #define ROT_BUF_FIL_STOP_ADR    Y:$60F59,0,16,U
      #define UNUSED_13_ADR           X:$60F59,0,16,U

      #define VS_L_NLABEL_ADR         DP:$60F5A
      #define VS_L_NLABEL2_ADR        DP:$60F5B

      #define VS_CURRENT_PROG_ADR             DP:$60EE0
      #define VS_CURRENT_PROG2_ADR            DP:$60EE1
      #define VS_CURRENT_BLOCK_ADR            DP:$60EE2
      #define VS_CURRENT_BLOCK2_ADR           DP:$60EE3
      #define VS_LAST_BLOCK_ADR               DP:$60EE4
      #define VS_LAST_BLOCK2_ADR              DP:$60EE5
      #define VS_SPINDLE_RPM_ADR              DP:$60EE6
      #define VS_SPINDLE_MAX_RPM_ADR          DP:$60EE7
      #define VS_SPINDLE_MAX_LIM_ADR          DP:$60EE8
      #define VS_SPINDLE_CMD_RPM_ADR          DP:$60EE9
      #define VS_SPINDLE_ACT_RPM_ADR          DP:$60EEA
      #define VS_SPINDLE_COUNTS_REV_ADR       F:$60EEB
      #define VS_SPINDLE_CSS_ADR              F:$60EEC
      #define VS_SPINDLE_CSS_UNITS_ADR        F:$60EED
      #define VS_GEAR_RATIO_ADR               F:$60EEE
      #define VS_SPINDLE_OVERRIDE_ADR         Y:$60EEF,0,16,U
      #define VS_BLOCKS_SEC_ADR               X:$60EEF,0,16,U
      #define VS_TURRET2_TOOL_ADR             Y:$60EF0,0,16,U
      #define VS_TURRET_TOOL_ADR              X:$60EF0,0,16,U
      #define VS_TURRET_POT_ADR               Y:$60EF1,0,16,U
      #define VS_TURRET2_POT_ADR              X:$60EF1,0,16,U
      #define VS_TOOL_NUMBER_ADR              Y:$60EF2,0,16,U
      #define VS_TOOL_NUMBER2_ADR             X:$60EF2,0,16,U
      #define VS_TOOL_NEXT_ADR                Y:$60EF3,0,16,U
      #define VS_TOOL_NEXT2_ADR               X:$60EF3,0,16,U
      #define VS_HANDLE_LINC_ADR              F:$60EF4
      #define VS_JOG_STEP_ADR                 F:$60EF5
      #define VS_HAND_STEP_ADR                F:$60EF6
      #define VS_SEMAPHORE_ADR                Y:$60EF7,0,16,U
      #define VS_SEMAPHORE2_ADR               X:$60EF7,0,16,U
      #define VS_FEED_OVERRIDE_ADR            F:$60EF8
      #define VS_FEED_OVERRIDE2_ADR           F:$60EF9
      #define VS_RAPID_OVERRIDE_ADR           F:$60EFA
      #define VS_RAPID_OVERRIDE2_ADR          F:$60EFB
      #define VS_THREAD_LEAD_ADR              F:$60EFC
      #define VS_THREAD_LEAD2_ADR             F:$60EFD
      #define VS_A_REL_ADR                  F:$60EFE
      #define VS_B_REL_ADR                  F:$60EFF
      #define VS_C_REL_ADR                  F:$60F00
      #define VS_X_REL_ADR                  F:$60F01
      #define VS_Y_REL_ADR                  F:$60F02
      #define VS_Z_REL_ADR                  F:$60F03
      #define VS_U_REL_ADR                  F:$60F04
      #define VS_V_REL_ADR                  F:$60F05
      #define VS_W_REL_ADR                  F:$60F06
      #define VS_A_REL2_ADR                 F:$60F07
      #define VS_B_REL2_ADR                 F:$60F08
      #define VS_C_REL2_ADR                 F:$60F09
      #define VS_X_REL2_ADR                 F:$60F0A
      #define VS_Y_REL2_ADR                 F:$60F0B
      #define VS_Z_REL2_ADR                 F:$60F0C
      #define VS_U_REL2_ADR                 F:$60F0D
      #define VS_V_REL2_ADR                 F:$60F0E
      #define VS_W_REL2_ADR                 F:$60F0F
      #define VS_A_ABS_ADR                  F:$60F10
      #define VS_B_ABS_ADR                  F:$60F11
      #define VS_C_ABS_ADR                  F:$60F12
      #define VS_X_ABS_ADR                  F:$60F13
      #define VS_Y_ABS_ADR                  F:$60F14
      #define VS_Z_ABS_ADR                  F:$60F15
      #define VS_U_ABS_ADR                  F:$60F16
      #define VS_V_ABS_ADR                  F:$60F17
      #define VS_W_ABS_ADR                  F:$60F18
      #define VS_A_ABS2_ADR                 F:$60F19
      #define VS_B_ABS2_ADR                 F:$60F1A
      #define VS_C_ABS2_ADR                 F:$60F1B
      #define VS_X_ABS2_ADR                 F:$60F1C
      #define VS_Y_ABS2_ADR                 F:$60F1D
      #define VS_Z_ABS2_ADR                 F:$60F1E
      #define VS_U_ABS2_ADR                 F:$60F1F
      #define VS_V_ABS2_ADR                 F:$60F20
      #define VS_W_ABS2_ADR                 F:$60F21
      #define VS_A_MUL_ADR                  F:$60F22
      #define VS_B_MUL_ADR                  F:$60F23
      #define VS_C_MUL_ADR                  F:$60F24
      #define VS_X_MUL_ADR                  F:$60F25
      #define VS_Y_MUL_ADR                  F:$60F26
      #define VS_Z_MUL_ADR                  F:$60F27
      #define VS_U_MUL_ADR                  F:$60F28
      #define VS_V_MUL_ADR                  F:$60F29
      #define VS_W_MUL_ADR                  F:$60F2A
      #define VS_A_MUL2_ADR                 F:$60F2B
      #define VS_B_MUL2_ADR                 F:$60F2C
      #define VS_C_MUL2_ADR                 F:$60F2D
      #define VS_X_MUL2_ADR                 F:$60F2E
      #define VS_Y_MUL2_ADR                 F:$60F2F
      #define VS_Z_MUL2_ADR                 F:$60F30
      #define VS_U_MUL2_ADR                 F:$60F31
      #define VS_V_MUL2_ADR                 F:$60F32
      #define VS_W_MUL2_ADR                 F:$60F33
      #define VS_NLABEL_ADR                 Y:$60F34,0,16,U
      #define VS_NLABEL2_ADR                X:$60F34,0,16,U
      #define VS_TOOL_OFFSET_ADR              Y:$60F35,0,16,U
      #define VS_TOOL_OFFSET2_ADR             X:$60F35,0,16,U
      #define VS_COMP_OFFSET_ADR              Y:$60F51,0,16,U
      #define VS_COMP_OFFSET2_ADR             X:$60F51,0,16,U
      #define VS_GGROUP0_1_ADR                  Y:$60F36,0,16,U
      #define VS_GGROUP1_1_ADR                  X:$60F36,0,16,U
      #define VS_GGROUP2_1_ADR                  Y:$60F37,0,16,U
      #define VS_GGROUP3_1_ADR                  X:$60F37,0,16,U
      #define VS_GGROUP4_1_ADR                  Y:$60F38,0,16,U
      #define VS_GGROUP5_1_ADR                  X:$60F38,0,16,U
      #define VS_GGROUP6_1_ADR                  Y:$60F39,0,16,U
      #define VS_GGROUP7_1_ADR                  X:$60F39,0,16,U
      #define VS_GGROUP8_1_ADR                  Y:$60F3A,0,16,U
      #define VS_GGROUP9_1_ADR                  X:$60F3A,0,16,U
      #define VS_GGROUP10_1_ADR                 Y:$60F3B,0,16,U
      #define VS_GGROUP11_1_ADR                 X:$60F3B,0,16,U
      #define VS_GGROUP12_1_ADR                 Y:$60F3C,0,16,U
      #define VS_GGROUP13_1_ADR                 X:$60F3C,0,16,U
      #define VS_GGROUP14_1_ADR                 Y:$60F3D,0,16,U
      #define VS_GGROUP15_1_ADR                 X:$60F3D,0,16,U
      #define VS_GGROUP16_1_ADR                 Y:$60F3E,0,16,U
      #define VS_GGROUP17_1_ADR                 X:$60F3E,0,16,U
      #define VS_GGROUP18_1_ADR                 Y:$60F3F,0,16,U
      #define VS_GGROUP19_1_ADR                 X:$60F3F,0,16,U
      #define VS_GGROUP20_1_ADR                 Y:$60F40,0,16,U
      #define VS_GGROUP21_1_ADR                 X:$60F40,0,16,U
      #define VS_GGROUP22_1_ADR                 Y:$60F41,0,16,U
      #define VS_GGROUP23_1_ADR                 X:$60F41,0,16,U
      #define VS_GGROUP24_1_ADR                 Y:$60F42,0,16,U
      #define VS_GGROUP25_1_ADR                 Y:$60F42,0,16,U
      #define VS_GGROUP0_2_ADR                  Y:$60F43,0,16,U
      #define VS_GGROUP1_2_ADR                  X:$60F43,0,16,U
      #define VS_GGROUP2_2_ADR                  Y:$60F44,0,16,U
      #define VS_GGROUP3_2_ADR                  X:$60F44,0,16,U
      #define VS_GGROUP4_2_ADR                  Y:$60F45,0,16,U
      #define VS_GGROUP5_2_ADR                  X:$60F45,0,16,U
      #define VS_GGROUP6_2_ADR                  Y:$60F46,0,16,U
      #define VS_GGROUP7_2_ADR                  X:$60F46,0,16,U
      #define VS_GGROUP8_2_ADR                  Y:$60F47,0,16,U
      #define VS_GGROUP9_2_ADR                  X:$60F47,0,16,U
      #define VS_GGROUP10_2_ADR                 Y:$60F48,0,16,U
      #define VS_GGROUP11_2_ADR                 X:$60F48,0,16,U
      #define VS_GGROUP12_2_ADR                 Y:$60F49,0,16,U
      #define VS_GGROUP13_2_ADR                 X:$60F49,0,16,U
      #define VS_GGROUP14_2_ADR                 Y:$60F4A,0,16,U
      #define VS_GGROUP15_2_ADR                 X:$60F4A,0,16,U
      #define VS_GGROUP16_2_ADR                 Y:$60F4B,0,16,U
      #define VS_GGROUP17_2_ADR                 X:$60F4B,0,16,U
      #define VS_GGROUP18_2_ADR                 Y:$60F4C,0,16,U
      #define VS_GGROUP19_2_ADR                 X:$60F4C,0,16,U
      #define VS_GGROUP20_2_ADR                 Y:$60F4D,0,16,U
      #define VS_GGROUP21_2_ADR                 X:$60F4D,0,16,U
      #define VS_GGROUP22_2_ADR                 Y:$60F4E,0,16,U
      #define VS_GGROUP23_2_ADR                 X:$60F4E,0,16,U
      #define VS_GGROUP24_2_ADR                 Y:$60F4F,0,16,U
      #define VS_GGROUP25_2_ADR                 Y:$60F4F,0,16,U
      #define ES_PC_CLOCK_ADR                DP:$60F60
      #define ES_ERR_FATAL_ADR               DP:$60F61
      #define ES_ERR_FATAL2_ADR              DP:$60F62
      #define ES_ERR_STOP_ADR                DP:$60F63
      #define ES_ERR_STOP2_ADR               DP:$60F64
      #define ES_ERR_WARN_ADR                DP:$60F65
      #define ES_ERR_WARN2_ADR               DP:$60F66
      #define ES_ERR_MSG_ADR                 DP:$60F67
      #define ES_ERR_MSG2_ADR                DP:$60F68
      #define ES_PLCMSGBOX_ADR               DP:$60F69
      #define ES_SCANAVE_ADR                 X:$60F70,0,16,U
      #define ES_SCANMAX_ADR                 Y:$60F70,0,16,U

      // USER BAR LOCATIONS
      #define PC_UB_CMD_BOX1                 0x3DA8
      #define UB_CMD_BOX1_ADR                DP:$60F6A
      #define UB_CMD_BOX1_STATUS             Y:$60F6A,0,4,U
      #define UB_CMD_BOX1_TYPE               Y:$60F6A,4,4,U
      #define UB_CMD_BOX1_COLR_TX            Y:$60F6A,8,4,U
      #define UB_CMD_BOX1_COLR_BG            Y:$60F6A,12,4,U
      #Define UB_CMD_BOX1_DEF_ID             X:$60F6A,0,8,U
                                          // X:$60F6A,8,8 not in use
      #define PC_UB_DATA_BOX1                0x3DAC
      #define UB_DATA_BOX1_DW                DP:$60F6B
      #define UB_DATA_BOX1_F                 F:$60F6B

      #define PC_UB_CMD_BOX2                 0x3DB0
      #define UB_CMD_BOX2_ADR                DP:$60F6C
      #define UB_CMD_BOX2_STATUS             Y:$60F6C,0,4,U
      #define UB_CMD_BOX2_TYPE               Y:$60F6C,4,4,U
      #define UB_CMD_BOX2_COLR_TX            Y:$60F6C,8,4,U
      #define UB_CMD_BOX2_COLR_BG            Y:$60F6C,12,4,U
      #Define UB_CMD_BOX2_DEF_ID             X:$60F6C,0,8,U
                                          // X:$60F6C,8,8 not in use
      #define PC_UB_DATA_BOX2                0x3DB4
      #define UB_DATA_BOX2_DW                DP:$60F6D
      #define UB_DATA_BOX2_F                 F:$60F6D

      #define PC_UB_CMD_BOX3                 0x3DB8
      #define UB_CMD_BOX3_ADR                DP:$60F6E
      #define UB_CMD_BOX3_STATUS             Y:$60F6E,0,4,U
      #define UB_CMD_BOX3_TYPE               Y:$60F6E,4,4,U
      #define UB_CMD_BOX3_COLR_TX            Y:$60F6E,8,4,U
      #define UB_CMD_BOX3_COLR_BG            Y:$60F6E,12,4,U
      #Define UB_CMD_BOX3_DEF_ID             X:$60F6E,0,8,U
                                          // X:$60F6E,8,8 not in use
      #define PC_UB_DATA_BOX3                0x3DBC
      #define UB_DATA_BOX3_DW                DP:$60F6F
      #define UB_DATA_BOX3_F                 F:$60F6F

      #define PC_UB_CMD_BOX4                 0x3DC0
      #define UB_CMD_BOX4_ADR                DP:$60F70
      #define UB_CMD_BOX4_STATUS             Y:$60F70,0,4,U
      #define UB_CMD_BOX4_TYPE               Y:$60F70,4,4,U
      #define UB_CMD_BOX4_COLR_TX            Y:$60F70,8,4,U
      #define UB_CMD_BOX4_COLR_BG            Y:$60F70,12,4,U
      #Define UB_CMD_BOX4_DEF_ID             X:$60F70,0,8,U
                                          // X:$60F70,8,8 not in use
      #define PC_UB_DATA_BOX4                0x3DC4
      #define UB_DATA_BOX4_DW                DP:$60F71
      #define UB_DATA_BOX4_F                 F:$60F71

      #define PC_UB_CMD_BOX5                 0x3DC8
      #define UB_CMD_BOX5_ADR                DP:$60F72
      #define UB_CMD_BOX5_STATUS             Y:$60F72,0,4,U
      #define UB_CMD_BOX5_TYPE               Y:$60F72,4,4,U
      #define UB_CMD_BOX5_COLR_TX            Y:$60F72,8,4,U
      #define UB_CMD_BOX5_COLR_BG            Y:$60F72,12,4,U
      #Define UB_CMD_BOX5_DEF_ID             X:$60F72,0,8,U
                                          // X:$60F72,8,8 not in use
      #define PC_UB_DATA_BOX5                0x3DCC
      #define UB_DATA_BOX5_DW                DP:$60F73
      #define UB_DATA_BOX5_F                 F:$60F73

      #define PC_UB_CMD_BOX6                 0x3DD0
      #define UB_CMD_BOX6_ADR                DP:$60F74
      #define UB_CMD_BOX6_STATUS             Y:$60F74,0,4,U
      #define UB_CMD_BOX6_TYPE               Y:$60F74,4,4,U
      #define UB_CMD_BOX6_COLR_TX            Y:$60F74,8,4,U
      #define UB_CMD_BOX6_COLR_BG            Y:$60F74,12,4,U
      #Define UB_CMD_BOX6_DEF_ID             X:$60F74,0,8,U
                                          // X:$60F74,8,8 not in use
      #define PC_UB_DATA_BOX6                0x3DD4
      #define UB_DATA_BOX6_DW                DP:$60F75
      #define UB_DATA_BOX6_F                 F:$60F75

      #define PC_UB_CMD_BOX7                 0x3DD8
      #define UB_CMD_BOX7_ADR                DP:$60F76
      #define UB_CMD_BOX7_STATUS             Y:$60F76,0,4,U
      #define UB_CMD_BOX7_TYPE               Y:$60F76,4,4,U
      #define UB_CMD_BOX7_COLR_TX            Y:$60F76,8,4,U
      #define UB_CMD_BOX7_COLR_BG            Y:$60F76,12,4,U
      #Define UB_CMD_BOX7_DEF_ID             X:$60F76,0,8,U
                                          // X:$60F76,8,8 not in use
      #define PC_UB_DATA_BOX7                0x3DDC
      #define UB_DATA_BOX7_DW                DP:$60F77
      #define UB_DATA_BOX7_F                 F:$60F77
      // END OF USER BAR LOCATIONS

      #define IN_1_ADR           DP:$60FE0
      #define IN_1_CHNG_ADR       DP:$60FF0
      #define IN_2_ADR           DP:$60FE1
      #define IN_2_CHNG_ADR       DP:$60FF1
      #define IN_3_ADR           DP:$60FE2
      #define IN_3_CHNG_ADR       DP:$60FF2
      #define IN_4_ADR           DP:$60FE3
      #define IN_4_CHNG_ADR       DP:$60FF3
      #define IN_5_ADR           DP:$60FE4
      #define IN_5_CHNG_ADR       DP:$60FF4
      #define IN_6_ADR           DP:$60FE5
      #define IN_6_CHNG_ADR       DP:$60FF5
      #define IN_7_ADR           DP:$60FE6
      #define IN_7_CHNG_ADR       DP:$60FF6
      #define IN_8_ADR           DP:$60FE7
      #define IN_8_CHNG_ADR       DP:$60FF7
      #define OUT_1_ADR          DP:$60FE8
      #define OUT_1_CHNG_ADR      DP:$60FF8
      #define OUT_2_ADR          DP:$60FE9
      #define OUT_2_CHNG_ADR      DP:$60FF9
      #define OUT_3_ADR          DP:$60FEA
      #define OUT_3_CHNG_ADR      DP:$60FFA
      #define OUT_4_ADR          DP:$60FEB
      #define OUT_4_CHNG_ADR      DP:$60FFB
      #define OUT_5_ADR          DP:$60FEC
      #define OUT_5_CHNG_ADR      DP:$60FFC
      #define OUT_6_ADR          DP:$60FED
      #define OUT_6_CHNG_ADR     DP:$60FFD
      #define OUT_7_ADR          DP:$60FEE
      #define OUT_7_CHNG_ADR     DP:$60FFE
      #define OUT_8_ADR          DP:$60FEF
      #define OUT_8_CHNG_ADR     DP:$60FFF
    #endif // QMAC  
  #endif // UMAC
#else
  // NON TURBO
  #define DPR_BASE            $D000
  #define CS_COMMAND1_ADR     DP:$DDF0
  #define CS_STATUS1_ADR      DP:$DDE0
  #define S_JOG_ADR           Y:$DDE0,0,4,U
  #define C_JOG_ADR           Y:$DDF0,0,4,U
  #define S_MODE_ADR          Y:$DDE0,4,4,U
  #define C_MODE_ADR          Y:$DDF0,4,4,U
  #define S_AXIS_ADR          Y:$DDE0,8,4,U
  #define C_AXIS_ADR          Y:$DDF0,8,4,U
  #define S_FEED_ADR          Y:$DDE0,12,4,U
  #define C_FEED_ADR          Y:$DDF0,12,4,U
  #define S_ROVRD_ADR         X:$DDE0,0,4,U
  #define C_ROVRD_ADR         X:$DDF0,0,4,U
  #define S_SPND_ADR          X:$DDE0,4,4,U
  #define C_SPND_ADR          X:$DDF0,4,4,U
  #define S_FOVRD_ADR         X:$DDE0,8,4,U
  #define C_FOVRD_ADR         X:$DDF0,8,4,U
  #define S_SOVRD_ADR         X:$DDE0,12,4,U
  #define C_SOVRD_ADR         X:$DDF0,12,4,U
  #define CS_STATUS2_ADR      DP:$DDE1
  #define CS_COMMAND2_ADR     DP:$DDF1
  #define S_COORD_ADR         Y:$DDE1,0,4,U
  #define C_COORD_ADR         Y:$DDF1,0,4,U
  #define S_GEAR_ADR          Y:$DDE1,4,4,U
  #define C_GEAR_ADR          Y:$DDF1,4,4,U
  #define S_HOME_ADR          Y:$DDE1,8,8,U
  #define C_HOME_ADR          Y:$DDF1,8,8,U
  #define S_ERROR_ADR         X:$DDE1,0,4,U
  #define C_ERROR_ADR         X:$DDF1,0,4,U
  #define S_CLNT_ADR          X:$DDE1,4,4,U
  #define C_CLNT_ADR          X:$DDF1,4,4,U
  #define S_DIST_ADR          X:$DDE1,8,4,U
  #define C_DIST_ADR          X:$DDF1,8,4,U
  #define S_MACHINE_ADR       X:$DDE1,12,4,U
  #define C_MACHINE_ADR       X:$DDF1,12,4,U
  #define CS_STATUS3_ADR      DP:$DDE2
  #define CS_COMMAND3_ADR     DP:$DDF2
  #define CS_STATUS4_ADR      DP:$DDE3
  #define CS_COMMAND4_ADR     DP:$DDF3
  #define CS_STATUS5_ADR      DP:$DDE4
  #define CS_COMMAND5_ADR     DP:$DDF4
  #define S_MOTION_ADR            Y:$DDE4,0,4,U
  #define C_MOTION_ADR            Y:$DDF4,0,4,U
  #define S_TIMEBASE_ADR          Y:$DDE4,4,4,U
  #define C_TIMEBASE_ADR          Y:$DDF4,4,4,U
  #define S_PROG_STATUS_ADR       Y:$DDE4,8,4,U
  #define C_PROG_STATUS_ADR       Y:$DDF4,8,4,U
  #define CS_STATUS6_ADR          DP:$DDE5
  #define CS_COMMAND6_ADR         DP:$DDF5
  #define CS_STATUS7_ADR          DP:$DDE6
  #define CS_COMMAND7_ADR         DP:$DDF6
  #define CS_STATUS8_ADR          DP:$DDE7
  #define CS_COMMAND8_ADR         DP:$DDF7
  #define PR_BITS_ADR             DP:$DDFD
  #define PR_COMMAND_ADR          DP:$DDFE
  #define PR_DATA_ADR             F:$DDFF

  #define AT_DPR_POINTER_ADR      DP:$DFFF     // AS DEFAULT, POINT TO UNLIKELY TO BE USED
  #define DPR_POINTER_ADR         Y:$BCF8,0,16 // POINTS TO M248 ADDRESSING PART

  #define G65_LEVEL_ADR           DP:$DF52
  #define G65_LEVEL2_ADR          DP:$DF53

  #define PMAC_RT_BLOCK_ADR       DP:$DF54
  #define PMAC_RT_BLOCK2_ADR      DP:$DF55
  #define PMAC_RT_NM_BLOCK_ADR    DP:$DF56
  #define PMAC_RT_NM_BLOCK2_ADR   DP:$DF57

  #define PMAC_RESET_COMP_ADR     Y:$DF58,0,16,U
  #define PC_RESET_COMP_ADR       X:$DF58,0,16,U

  #define ROT_BUF_FIL_STOP_ADR    Y:$DF59,0,16,U
  #define UNUSED_13_ADR           X:$DF59,0,16,U

  #define VS_L_NLABEL_ADR         DP:$DF5A
  #define VS_L_NLABEL2_ADR        DP:$DF5B

  #define VS_CURRENT_PROG_ADR             DP:$DEE0
  #define VS_CURRENT_PROG2_ADR            DP:$DEE1
  #define VS_CURRENT_BLOCK_ADR            DP:$DEE2
  #define VS_CURRENT_BLOCK2_ADR           DP:$DEE3
  #define VS_LAST_BLOCK_ADR               DP:$DEE4
  #define VS_LAST_BLOCK2_ADR              DP:$DEE5
  #define VS_SPINDLE_RPM_ADR              DP:$DEE6
  #define VS_SPINDLE_MAX_RPM_ADR          DP:$DEE7
  #define VS_SPINDLE_MAX_LIM_ADR          DP:$DEE8
  #define VS_SPINDLE_CMD_RPM_ADR          DP:$DEE9
  #define VS_SPINDLE_ACT_RPM_ADR          DP:$DEEA
  #define VS_SPINDLE_COUNTS_REV_ADR       F:$DEEB
  #define VS_SPINDLE_CSS_ADR              F:$DEEC
  #define VS_SPINDLE_CSS_UNITS_ADR        F:$DEED
  #define VS_GEAR_RATIO_ADR               F:$DEEE
  #define VS_SPINDLE_OVERRIDE_ADR         Y:$DEEF,0,16,U
  #define VS_BLOCKS_SEC_ADR               X:$DEEF,0,16,U
  #define VS_TURRET2_TOOL_ADR             Y:$DEF0,0,16,U
  #define VS_TURRET_TOOL_ADR              X:$DEF0,0,16,U
  #define VS_TURRET_POT_ADR               Y:$DEF1,0,16,U
  #define VS_TURRET2_POT_ADR              X:$DEF1,0,16,U
  #define VS_TOOL_NUMBER_ADR              Y:$DEF2,0,16,U
  #define VS_TOOL_NUMBER2_ADR             X:$DEF2,0,16,U
  #define VS_TOOL_NEXT_ADR                Y:$DEF3,0,16,U
  #define VS_TOOL_NEXT2_ADR               X:$DEF3,0,16,U
  #define VS_HANDLE_LINC_ADR              F:$DEF4
  #define VS_JOG_STEP_ADR                 F:$DEF5
  #define VS_HAND_STEP_ADR                F:$DEF6
  #define VS_SEMAPHORE_ADR                Y:$DEF7,0,16,U
  #define VS_SEMAPHORE2_ADR               X:$DEF7,0,16,U
  #define VS_FEED_OVERRIDE_ADR            F:$DEF8
  #define VS_FEED_OVERRIDE2_ADR           F:$DEF9
  #define VS_RAPID_OVERRIDE_ADR           F:$DEFA
  #define VS_RAPID_OVERRIDE2_ADR          F:$DEFB
  #define VS_THREAD_LEAD_ADR              F:$DEFC
  #define VS_THREAD_LEAD2_ADR             F:$DEFD
  #define VS_A_REL_ADR                  F:$DEFE
  #define VS_B_REL_ADR                  F:$DEFF
  #define VS_C_REL_ADR                  F:$DF00
  #define VS_X_REL_ADR                  F:$DF01
  #define VS_Y_REL_ADR                  F:$DF02
  #define VS_Z_REL_ADR                  F:$DF03
  #define VS_U_REL_ADR                  F:$DF04
  #define VS_V_REL_ADR                  F:$DF05
  #define VS_W_REL_ADR                  F:$DF06
  #define VS_A_REL2_ADR                 F:$DF07
  #define VS_B_REL2_ADR                 F:$DF08
  #define VS_C_REL2_ADR                 F:$DF09
  #define VS_X_REL2_ADR                 F:$DF0A
  #define VS_Y_REL2_ADR                 F:$DF0B
  #define VS_Z_REL2_ADR                 F:$DF0C
  #define VS_U_REL2_ADR                 F:$DF0D
  #define VS_V_REL2_ADR                 F:$DF0E
  #define VS_W_REL2_ADR                 F:$DF0F
  #define VS_A_ABS_ADR                  F:$DF10
  #define VS_B_ABS_ADR                  F:$DF11
  #define VS_C_ABS_ADR                  F:$DF12
  #define VS_X_ABS_ADR                  F:$DF13
  #define VS_Y_ABS_ADR                  F:$DF14
  #define VS_Z_ABS_ADR                  F:$DF15
  #define VS_U_ABS_ADR                  F:$DF16
  #define VS_V_ABS_ADR                  F:$DF17
  #define VS_W_ABS_ADR                  F:$DF18
  #define VS_A_ABS2_ADR                 F:$DF19
  #define VS_B_ABS2_ADR                 F:$DF1A
  #define VS_C_ABS2_ADR                 F:$DF1B
  #define VS_X_ABS2_ADR                 F:$DF1C
  #define VS_Y_ABS2_ADR                 F:$DF1D
  #define VS_Z_ABS2_ADR                 F:$DF1E
  #define VS_U_ABS2_ADR                 F:$DF1F
  #define VS_V_ABS2_ADR                 F:$DF20
  #define VS_W_ABS2_ADR                 F:$DF21
  #define VS_A_MUL_ADR                  F:$DF22
  #define VS_B_MUL_ADR                  F:$DF23
  #define VS_C_MUL_ADR                  F:$DF24
  #define VS_X_MUL_ADR                  F:$DF25
  #define VS_Y_MUL_ADR                  F:$DF26
  #define VS_Z_MUL_ADR                  F:$DF27
  #define VS_U_MUL_ADR                  F:$DF28
  #define VS_V_MUL_ADR                  F:$DF29
  #define VS_W_MUL_ADR                  F:$DF2A
  #define VS_A_MUL2_ADR                 F:$DF2B
  #define VS_B_MUL2_ADR                 F:$DF2C
  #define VS_C_MUL2_ADR                 F:$DF2D
  #define VS_X_MUL2_ADR                 F:$DF2E
  #define VS_Y_MUL2_ADR                 F:$DF2F
  #define VS_Z_MUL2_ADR                 F:$DF30
  #define VS_U_MUL2_ADR                 F:$DF31
  #define VS_V_MUL2_ADR                 F:$DF32
  #define VS_W_MUL2_ADR                 F:$DF33
  #define VS_NLABEL_ADR                 Y:$DF34,0,16,U
  #define VS_NLABEL2_ADR                X:$DF34,0,16,U
  #define VS_TOOL_OFFSET_ADR              Y:$DF35,0,16,U
  #define VS_TOOL_OFFSET2_ADR             X:$DF35,0,16,U
  #define VS_COMP_OFFSET_ADR              Y:$DF51,0,16,U
  #define VS_COMP_OFFSET2_ADR             X:$DF51,0,16,U
  #define VS_GGROUP0_1_ADR                  Y:$DF36,0,16,U
  #define VS_GGROUP1_1_ADR                  X:$DF36,0,16,U
  #define VS_GGROUP2_1_ADR                  Y:$DF37,0,16,U
  #define VS_GGROUP3_1_ADR                  X:$DF37,0,16,U
  #define VS_GGROUP4_1_ADR                  Y:$DF38,0,16,U
  #define VS_GGROUP5_1_ADR                  X:$DF38,0,16,U
  #define VS_GGROUP6_1_ADR                  Y:$DF39,0,16,U
  #define VS_GGROUP7_1_ADR                  X:$DF39,0,16,U
  #define VS_GGROUP8_1_ADR                  Y:$DF3A,0,16,U
  #define VS_GGROUP9_1_ADR                  X:$DF3A,0,16,U
  #define VS_GGROUP10_1_ADR                 Y:$DF3B,0,16,U
  #define VS_GGROUP11_1_ADR                 X:$DF3B,0,16,U
  #define VS_GGROUP12_1_ADR                 Y:$DF3C,0,16,U
  #define VS_GGROUP13_1_ADR                 X:$DF3C,0,16,U
  #define VS_GGROUP14_1_ADR                 Y:$DF3D,0,16,U
  #define VS_GGROUP15_1_ADR                 X:$DF3D,0,16,U
  #define VS_GGROUP16_1_ADR                 Y:$DF3E,0,16,U
  #define VS_GGROUP17_1_ADR                 X:$DF3E,0,16,U
  #define VS_GGROUP18_1_ADR                 Y:$DF3F,0,16,U
  #define VS_GGROUP19_1_ADR                 X:$DF3F,0,16,U
  #define VS_GGROUP20_1_ADR                 Y:$DF40,0,16,U
  #define VS_GGROUP21_1_ADR                 X:$DF40,0,16,U
  #define VS_GGROUP22_1_ADR                 Y:$DF41,0,16,U
  #define VS_GGROUP23_1_ADR                 X:$DF41,0,16,U
  #define VS_GGROUP24_1_ADR                 Y:$DF42,0,16,U
  #define VS_GGROUP25_1_ADR                 Y:$DF42,0,16,U
  #define VS_GGROUP0_2_ADR                  Y:$DF43,0,16,U
  #define VS_GGROUP1_2_ADR                  X:$DF43,0,16,U
  #define VS_GGROUP2_2_ADR                  Y:$DF44,0,16,U
  #define VS_GGROUP3_2_ADR                  X:$DF44,0,16,U
  #define VS_GGROUP4_2_ADR                  Y:$DF45,0,16,U
  #define VS_GGROUP5_2_ADR                  X:$DF45,0,16,U
  #define VS_GGROUP6_2_ADR                  Y:$DF46,0,16,U
  #define VS_GGROUP7_2_ADR                  X:$DF46,0,16,U
  #define VS_GGROUP8_2_ADR                  Y:$DF47,0,16,U
  #define VS_GGROUP9_2_ADR                  X:$DF47,0,16,U
  #define VS_GGROUP10_2_ADR                 Y:$DF48,0,16,U
  #define VS_GGROUP11_2_ADR                 X:$DF48,0,16,U
  #define VS_GGROUP12_2_ADR                 Y:$DF49,0,16,U
  #define VS_GGROUP13_2_ADR                 X:$DF49,0,16,U
  #define VS_GGROUP14_2_ADR                 Y:$DF4A,0,16,U
  #define VS_GGROUP15_2_ADR                 X:$DF4A,0,16,U
  #define VS_GGROUP16_2_ADR                 Y:$DF4B,0,16,U
  #define VS_GGROUP17_2_ADR                 X:$DF4B,0,16,U
  #define VS_GGROUP18_2_ADR                 Y:$DF4C,0,16,U
  #define VS_GGROUP19_2_ADR                 X:$DF4C,0,16,U
  #define VS_GGROUP20_2_ADR                 Y:$DF4D,0,16,U
  #define VS_GGROUP21_2_ADR                 X:$DF4D,0,16,U
  #define VS_GGROUP22_2_ADR                 Y:$DF4E,0,16,U
  #define VS_GGROUP23_2_ADR                 X:$DF4E,0,16,U
  #define VS_GGROUP24_2_ADR                 Y:$DF4F,0,16,U
  #define VS_GGROUP25_2_ADR                 Y:$DF4F,0,16,U
  #define ES_PC_CLOCK_ADR                DP:$DF60
  #define ES_ERR_FATAL_ADR               DP:$DF61
  #define ES_ERR_FATAL2_ADR              DP:$DF62
  #define ES_ERR_STOP_ADR                DP:$DF63
  #define ES_ERR_STOP2_ADR               DP:$DF64
  #define ES_ERR_WARN_ADR                DP:$DF65
  #define ES_ERR_WARN2_ADR               DP:$DF66
  #define ES_ERR_MSG_ADR                 DP:$DF67
  #define ES_ERR_MSG2_ADR                DP:$DF68
  #define ES_PLCMSGBOX_ADR               DP:$DF69
  #define ES_SCANAVE_ADR                 X:$DF70,0,16,U
  #define ES_SCANMAX_ADR                 Y:$DF70,0,16,U

  // USER BAR LOCATIONS
  #define PC_UB_CMD_BOX1                 0x3DA8
  #define UB_CMD_BOX1_ADR                DP:$DF6A
  #define UB_CMD_BOX1_STATUS             Y:$DF6A,0,4,U
  #define UB_CMD_BOX1_TYPE               Y:$DF6A,4,4,U
  #define UB_CMD_BOX1_COLR_TX            Y:$DF6A,8,4,U
  #define UB_CMD_BOX1_COLR_BG            Y:$DF6A,12,4,U
  #define UB_CMD_BOX1_DEF_ID             X:$DF6A,0,8,U
                                      // X:$DF6A,8,8 not in use
  #define PC_UB_DATA_BOX1                0x3DAC
  #define UB_DATA_BOX1_DW                DP:$DF6B
  #define UB_DATA_BOX1_F                 F:$DF6B

  #define PC_UB_CMD_BOX2                 0x3DB0
  #define UB_CMD_BOX2_ADR                DP:$DF6C
  #define UB_CMD_BOX2_STATUS             Y:$DF6C,0,4,U
  #define UB_CMD_BOX2_TYPE               Y:$DF6C,4,4,U
  #define UB_CMD_BOX2_COLR_TX            Y:$DF6C,8,4,U
  #define UB_CMD_BOX2_COLR_BG            Y:$DF6C,12,4,U
  #define UB_CMD_BOX2_DEF_ID             X:$DF6C,0,8,U
                                      // X:$DF6C,8,8 not in use
  #define PC_UB_DATA_BOX2                0x3DB4
  #define UB_DATA_BOX2_DW                DP:$DF6D
  #define UB_DATA_BOX2_F                 F:$DF6D

  #define PC_UB_CMD_BOX3                 0x3DB8
  #define UB_CMD_BOX3_ADR                DP:$DF6E
  #define UB_CMD_BOX3_STATUS             Y:$DF6E,0,4,U
  #define UB_CMD_BOX3_TYPE               Y:$DF6E,4,4,U
  #define UB_CMD_BOX3_COLR_TX            Y:$DF6E,8,4,U
  #define UB_CMD_BOX3_COLR_BG            Y:$DF6E,12,4,U
  #define UB_CMD_BOX3_DEF_ID             X:$DF6E,0,8,U
                                      // X:$DF6E,8,8 not in use
  #define PC_UB_DATA_BOX3                0x3DBC
  #define UB_DATA_BOX3_DW                DP:$DF6F
  #define UB_DATA_BOX3_F                 F:$DF6F

  #define PC_UB_CMD_BOX4                 0x3DC0
  #define UB_CMD_BOX4_ADR                DP:$DF70
  #define UB_CMD_BOX4_STATUS             Y:$DF70,0,4,U
  #define UB_CMD_BOX4_TYPE               Y:$DF70,4,4,U
  #define UB_CMD_BOX4_COLR_TX            Y:$DF70,8,4,U
  #define UB_CMD_BOX4_COLR_BG            Y:$DF70,12,4,U
  #define UB_CMD_BOX4_DEF_ID             X:$DF70,0,8,U
                                      // X:$DF70,8,8 not in use
  #define PC_UB_DATA_BOX4                0x3DC4
  #define UB_DATA_BOX4_DW                DP:$DF71
  #define UB_DATA_BOX4_F                 F:$DF71

  #define PC_UB_CMD_BOX5                 0x3DC8
  #define UB_CMD_BOX5_ADR                DP:$DF72
  #define UB_CMD_BOX5_STATUS             Y:$DF72,0,4,U
  #define UB_CMD_BOX5_TYPE               Y:$DF72,4,4,U
  #define UB_CMD_BOX5_COLR_TX            Y:$DF72,8,4,U
  #define UB_CMD_BOX5_COLR_BG            Y:$DF72,12,4,U
  #define UB_CMD_BOX5_DEF_ID             X:$DF72,0,8,U
                                      // X:$DF72,8,8 not in use
  #define PC_UB_DATA_BOX5                0x3DCC
  #define UB_DATA_BOX5_DW                DP:$DF73
  #define UB_DATA_BOX5_F                 F:$DF73

  #define PC_UB_CMD_BOX6                 0x3DD0
  #define UB_CMD_BOX6_ADR                DP:$DF74
  #define UB_CMD_BOX6_STATUS             Y:$DF74,0,4,U
  #define UB_CMD_BOX6_TYPE               Y:$DF74,4,4,U
  #define UB_CMD_BOX6_COLR_TX            Y:$DF74,8,4,U
  #define UB_CMD_BOX6_COLR_BG            Y:$DF74,12,4,U
  #define UB_CMD_BOX6_DEF_ID             X:$DF74,0,8,U
                                      // X:$DF74,8,8 not in use
  #define PC_UB_DATA_BOX6                0x3DD4
  #define UB_DATA_BOX6_DW                DP:$DF75
  #define UB_DATA_BOX6_F                 F:$DF75

  #define PC_UB_CMD_BOX7                 0x3DD8
  #define UB_CMD_BOX7_ADR                DP:$DF76
  #define UB_CMD_BOX7_STATUS             Y:$DF76,0,4,U
  #define UB_CMD_BOX7_TYPE               Y:$DF76,4,4,U
  #define UB_CMD_BOX7_COLR_TX            Y:$DF76,8,4,U
  #define UB_CMD_BOX7_COLR_BG            Y:$DF76,12,4,U
  #define UB_CMD_BOX7_DEF_ID             X:$DF76,0,8,U
                                      // X:$DF76,8,8 not in use
  #define PC_UB_DATA_BOX7                0x3DDC
  #define UB_DATA_BOX7_DW                DP:$DF77
  #define UB_DATA_BOX7_F                 F:$DF77
  // END OF USER BAR LOCATIONS

  #define IN_1_ADR           DP:$DFE0
  #define IN_1_CHNG_ADR       DP:$DFF0
  #define IN_2_ADR           DP:$DFE1
  #define IN_2_CHNG_ADR       DP:$DFF1
  #define IN_3_ADR           DP:$DFE2
  #define IN_3_CHNG_ADR       DP:$DFF2
  #define IN_4_ADR           DP:$DFE3
  #define IN_4_CHNG_ADR       DP:$DFF3
  #define IN_5_ADR           DP:$DFE4
  #define IN_5_CHNG_ADR       DP:$DFF4
  #define IN_6_ADR           DP:$DFE5
  #define IN_6_CHNG_ADR       DP:$DFF5
  #define IN_7_ADR           DP:$DFE6
  #define IN_7_CHNG_ADR       DP:$DFF6
  #define IN_8_ADR           DP:$DFE7
  #define IN_8_CHNG_ADR       DP:$DFF7
  #define OUT_1_ADR          DP:$DFE8
  #define OUT_1_CHNG_ADR      DP:$DFF8
  #define OUT_2_ADR          DP:$DFE9
  #define OUT_2_CHNG_ADR      DP:$DFF9
  #define OUT_3_ADR          DP:$DFEA
  #define OUT_3_CHNG_ADR      DP:$DFFA
  #define OUT_4_ADR          DP:$DFEB
  #define OUT_4_CHNG_ADR      DP:$DFFB
  #define OUT_5_ADR          DP:$DFEC
  #define OUT_5_CHNG_ADR      DP:$DFFC
  #define OUT_6_ADR          DP:$DFED
  #define OUT_6_CHNG_ADR     DP:$DFFD
  #define OUT_7_ADR          DP:$DFEE
  #define OUT_7_CHNG_ADR     DP:$DFFE
  #define OUT_8_ADR          DP:$DFEF
  #define OUT_8_CHNG_ADR     DP:$DFFF
#endif

#ifndef __DATE__ 
// Assignments   /////////////////////////////////////////////////////////////

// Current status values variables
VS_CURRENT_BLOCK_M->VS_CURRENT_BLOCK_ADR
VS_CURRENT_BLOCK2_M->VS_CURRENT_BLOCK2_ADR
VS_LAST_BLOCK_M->VS_LAST_BLOCK_ADR
VS_LAST_BLOCK2_M->VS_LAST_BLOCK2_ADR
VS_CURRENT_PROG_M->VS_CURRENT_PROG_ADR
VS_CURRENT_PROG2_M->VS_CURRENT_PROG2_ADR
VS_SPINDLE_RPM_M->VS_SPINDLE_RPM_ADR
VS_SPINDLE_MAX_RPM_M->VS_SPINDLE_MAX_RPM_ADR
VS_SPINDLE_CMD_RPM_M->VS_SPINDLE_CMD_RPM_ADR
VS_SPINDLE_ACT_RPM_M->VS_SPINDLE_ACT_RPM_ADR
VS_SPINDLE_CSS_UNITS_M->VS_SPINDLE_CSS_UNITS_ADR
VS_SPINDLE_CSS_M->VS_SPINDLE_CSS_ADR
VS_SPINDLE_MAX_LIM_M->VS_SPINDLE_MAX_LIM_ADR
VS_SPINDLE_COUNTS_REV_M->VS_SPINDLE_COUNTS_REV_ADR
VS_GEAR_RATIO_M->VS_GEAR_RATIO_ADR
VS_SPINDLE_OVERRIDE_M->VS_SPINDLE_OVERRIDE_ADR
VS_BLOCKS_SEC_M->VS_BLOCKS_SEC_ADR
VS_TURRET_TOOL_M->VS_TURRET_TOOL_ADR
VS_TURRET2_TOOL_M->VS_TURRET2_TOOL_ADR
VS_TURRET_POT_M->VS_TURRET_POT_ADR
VS_TURRET2_POT_M->VS_TURRET2_POT_ADR
VS_TOOL_NUMBER_M->VS_TOOL_NUMBER_ADR
VS_TOOL_NUMBER2_M->VS_TOOL_NUMBER2_ADR
VS_TOOL_NEXT_M->VS_TOOL_NEXT_ADR
VS_TOOL_NEXT2_M->VS_TOOL_NEXT2_ADR
VS_HANDLE_LINC_M->VS_HANDLE_LINC_ADR
VS_JOG_STEP_M->VS_JOG_STEP_ADR
VS_HAND_STEP_M->VS_HAND_STEP_ADR
VS_SEMAPHORE_M->VS_SEMAPHORE_ADR
VS_SEMAPHORE2_M->VS_SEMAPHORE2_ADR
VS_FEED_OVERRIDE_M->VS_FEED_OVERRIDE_ADR
VS_FEED_OVERRIDE2_M->VS_FEED_OVERRIDE2_ADR
VS_RAPID_OVERRIDE_M->VS_RAPID_OVERRIDE_ADR
VS_RAPID_OVERRIDE2_M->VS_RAPID_OVERRIDE2_ADR
VS_THREAD_LEAD_M->VS_THREAD_LEAD_ADR
VS_THREAD_LEAD2_M->VS_THREAD_LEAD2_ADR
VS_TOOL_OFFSET_M->VS_TOOL_OFFSET_ADR
VS_COMP_OFFSET_M->VS_COMP_OFFSET_ADR

VS_GGROUP0_1_M->VS_GGROUP0_1_ADR
VS_GGROUP1_1_M->VS_GGROUP1_1_ADR
VS_GGROUP2_1_M->VS_GGROUP2_1_ADR
VS_GGROUP3_1_M->VS_GGROUP3_1_ADR
VS_GGROUP4_1_M->VS_GGROUP4_1_ADR
VS_GGROUP5_1_M->VS_GGROUP5_1_ADR
VS_GGROUP6_1_M->VS_GGROUP6_1_ADR
VS_GGROUP7_1_M->VS_GGROUP7_1_ADR
VS_GGROUP8_1_M->VS_GGROUP8_1_ADR
VS_GGROUP9_1_M->VS_GGROUP9_1_ADR
VS_GGROUP10_1_M->VS_GGROUP10_1_ADR
VS_GGROUP11_1_M->VS_GGROUP11_1_ADR
VS_GGROUP12_1_M->VS_GGROUP12_1_ADR
VS_GGROUP13_1_M->VS_GGROUP13_1_ADR
VS_GGROUP14_1_M->VS_GGROUP14_1_ADR
VS_GGROUP15_1_M->VS_GGROUP15_1_ADR
VS_GGROUP16_1_M->VS_GGROUP16_1_ADR
VS_GGROUP17_1_M->VS_GGROUP17_1_ADR
VS_GGROUP18_1_M->VS_GGROUP18_1_ADR
VS_GGROUP19_1_M->VS_GGROUP19_1_ADR
VS_GGROUP20_1_M->VS_GGROUP20_1_ADR
VS_GGROUP21_1_M->VS_GGROUP21_1_ADR
VS_GGROUP22_1_M->VS_GGROUP22_1_ADR
VS_GGROUP23_1_M->VS_GGROUP23_1_ADR
VS_GGROUP24_1_M->VS_GGROUP24_1_ADR
VS_GGROUP0_2_M->VS_GGROUP0_2_ADR
VS_GGROUP1_2_M->VS_GGROUP1_2_ADR
VS_GGROUP2_2_M->VS_GGROUP2_2_ADR
VS_GGROUP3_2_M->VS_GGROUP3_2_ADR
VS_GGROUP4_2_M->VS_GGROUP4_2_ADR
VS_GGROUP5_2_M->VS_GGROUP5_2_ADR
VS_GGROUP6_2_M->VS_GGROUP6_2_ADR
VS_GGROUP7_2_M->VS_GGROUP7_2_ADR
VS_GGROUP8_2_M->VS_GGROUP8_2_ADR
VS_GGROUP9_2_M->VS_GGROUP9_2_ADR
VS_GGROUP10_2_M->VS_GGROUP10_2_ADR
VS_GGROUP11_2_M->VS_GGROUP11_2_ADR
VS_GGROUP12_2_M->VS_GGROUP12_2_ADR
VS_GGROUP13_2_M->VS_GGROUP13_2_ADR
VS_GGROUP14_2_M->VS_GGROUP14_2_ADR
VS_GGROUP15_2_M->VS_GGROUP15_2_ADR
VS_GGROUP16_2_M->VS_GGROUP16_2_ADR
VS_GGROUP17_2_M->VS_GGROUP17_2_ADR
VS_GGROUP18_2_M->VS_GGROUP18_2_ADR
VS_GGROUP19_2_M->VS_GGROUP19_2_ADR
VS_GGROUP20_2_M->VS_GGROUP20_2_ADR
VS_GGROUP21_2_M->VS_GGROUP21_2_ADR
VS_GGROUP22_2_M->VS_GGROUP22_2_ADR
VS_GGROUP23_2_M->VS_GGROUP23_2_ADR
VS_GGROUP24_2_M->VS_GGROUP24_2_ADR

VS_A_REL_M->VS_A_REL_ADR
VS_B_REL_M->VS_B_REL_ADR
VS_C_REL_M->VS_C_REL_ADR
VS_X_REL_M->VS_X_REL_ADR
VS_Y_REL_M->VS_Y_REL_ADR
VS_Z_REL_M->VS_Z_REL_ADR
VS_U_REL_M->VS_U_REL_ADR
VS_V_REL_M->VS_V_REL_ADR
VS_W_REL_M->VS_W_REL_ADR
VS_A_REL2_M->VS_A_REL2_ADR
VS_B_REL2_M->VS_B_REL2_ADR
VS_C_REL2_M->VS_C_REL2_ADR
VS_X_REL2_M->VS_X_REL2_ADR
VS_Y_REL2_M->VS_Y_REL2_ADR
VS_Z_REL2_M->VS_Z_REL2_ADR
VS_U_REL2_M->VS_U_REL2_ADR
VS_V_REL2_M->VS_V_REL2_ADR
VS_W_REL2_M->VS_W_REL2_ADR
VS_A_ABS_M->VS_A_ABS_ADR
VS_B_ABS_M->VS_B_ABS_ADR
VS_C_ABS_M->VS_C_ABS_ADR
VS_X_ABS_M->VS_X_ABS_ADR
VS_Y_ABS_M->VS_Y_ABS_ADR
VS_Z_ABS_M->VS_Z_ABS_ADR
VS_U_ABS_M->VS_U_ABS_ADR
VS_V_ABS_M->VS_V_ABS_ADR
VS_W_ABS_M->VS_W_ABS_ADR
VS_A_ABS2_M->VS_A_ABS2_ADR
VS_B_ABS2_M->VS_B_ABS2_ADR
VS_C_ABS2_M->VS_C_ABS2_ADR
VS_X_ABS2_M->VS_X_ABS2_ADR
VS_Y_ABS2_M->VS_Y_ABS2_ADR
VS_Z_ABS2_M->VS_Z_ABS2_ADR
VS_U_ABS2_M->VS_U_ABS2_ADR
VS_V_ABS2_M->VS_V_ABS2_ADR
VS_W_ABS2_M->VS_W_ABS2_ADR
VS_A_MUL_M->VS_A_MUL_ADR
VS_B_MUL_M->VS_B_MUL_ADR
VS_C_MUL_M->VS_C_MUL_ADR
VS_X_MUL_M->VS_X_MUL_ADR
VS_Y_MUL_M->VS_Y_MUL_ADR
VS_Z_MUL_M->VS_Z_MUL_ADR
VS_U_MUL_M->VS_U_MUL_ADR
VS_V_MUL_M->VS_V_MUL_ADR
VS_W_MUL_M->VS_W_MUL_ADR
VS_A_MUL2_M->VS_A_MUL2_ADR
VS_B_MUL2_M->VS_B_MUL2_ADR
VS_C_MUL2_M->VS_C_MUL2_ADR
VS_X_MUL2_M->VS_X_MUL2_ADR
VS_Y_MUL2_M->VS_Y_MUL2_ADR
VS_Z_MUL2_M->VS_Z_MUL2_ADR
VS_U_MUL2_M->VS_U_MUL2_ADR
VS_V_MUL2_M->VS_V_MUL2_ADR
VS_W_MUL2_M->VS_W_MUL2_ADR
VS_NLABEL_M->VS_NLABEL_ADR
VS_NLABEL2_M->VS_NLABEL2_ADR

// Assign Error pointers
ES_PC_CLOCK_M->ES_PC_CLOCK_ADR
ES_ERR_FATAL_M->ES_ERR_FATAL_ADR
ES_ERR_FATAL2_M->ES_ERR_FATAL2_ADR
ES_ERR_STOP_M->ES_ERR_STOP_ADR
ES_ERR_STOP2_M->ES_ERR_STOP2_ADR
ES_ERR_WARN_M->ES_ERR_WARN_ADR
ES_ERR_WARN2_M->ES_ERR_WARN2_ADR
ES_ERR_MSG_M->ES_ERR_MSG_ADR
ES_ERR_MSG2_M->ES_ERR_MSG2_ADR
ES_PLCMSGBOX_M->ES_PLCMSGBOX_ADR

// Command / Status variables
CS_STATUS1_M->CS_STATUS1_ADR
CS_STATUS2_M->CS_STATUS2_ADR
CS_STATUS3_M->CS_STATUS3_ADR
CS_STATUS4_M->CS_STATUS4_ADR
CS_STATUS5_M->CS_STATUS5_ADR
CS_STATUS6_M->CS_STATUS6_ADR
CS_STATUS7_M->CS_STATUS7_ADR
CS_STATUS8_M->CS_STATUS8_ADR
CS_COMMAND1_M->CS_COMMAND1_ADR
CS_COMMAND2_M->CS_COMMAND2_ADR
CS_COMMAND3_M->CS_COMMAND3_ADR
CS_COMMAND4_M->CS_COMMAND4_ADR
CS_COMMAND5_M->CS_COMMAND5_ADR
CS_COMMAND6_M->CS_COMMAND6_ADR
CS_COMMAND7_M->CS_COMMAND7_ADR
CS_COMMAND8_M->CS_COMMAND8_ADR

IN_1_M->IN_1_ADR
IN_2_M->IN_2_ADR
IN_3_M->IN_3_ADR
IN_4_M->IN_4_ADR
IN_5_M->IN_5_ADR
IN_6_M->IN_6_ADR
IN_7_M->IN_7_ADR
IN_8_M->IN_8_ADR
OUT_1_M->OUT_1_ADR
OUT_2_M->OUT_2_ADR
OUT_3_M->OUT_3_ADR
OUT_4_M->OUT_4_ADR
OUT_5_M->OUT_5_ADR
OUT_6_M->OUT_6_ADR
OUT_7_M->OUT_7_ADR
OUT_8_M->OUT_8_ADR

IN_1_CHNG_M->IN_1_CHNG_ADR
IN_2_CHNG_M->IN_2_CHNG_ADR
IN_3_CHNG_M->IN_3_CHNG_ADR
IN_4_CHNG_M->IN_4_CHNG_ADR
IN_5_CHNG_M->IN_5_CHNG_ADR
IN_6_CHNG_M->IN_6_CHNG_ADR
IN_7_CHNG_M->IN_7_CHNG_ADR
IN_8_CHNG_M->IN_8_CHNG_ADR
OUT_1_CHNG_M->OUT_1_CHNG_ADR
OUT_2_CHNG_M->OUT_2_CHNG_ADR
OUT_3_CHNG_M->OUT_3_CHNG_ADR
OUT_4_CHNG_M->OUT_4_CHNG_ADR
OUT_5_CHNG_M->OUT_5_CHNG_ADR
OUT_6_CHNG_M->OUT_6_CHNG_ADR
OUT_7_CHNG_M->OUT_7_CHNG_ADR
OUT_8_CHNG_M->OUT_8_CHNG_ADR


//  Assign DLL selectors
//  Status
S_MODE_M->S_MODE_ADR
S_AXIS_M->S_AXIS_ADR
S_JOG_M->S_JOG_ADR
S_FEED_M->S_FEED_ADR
S_DIST_M->S_DIST_ADR
S_SOVRD_M->S_SOVRD_ADR
S_SPND_M->S_SPND_ADR
S_FOVRD_M->S_FOVRD_ADR
S_ROVRD_M->S_ROVRD_ADR
S_COORD_M->S_COORD_ADR
S_HOME_M->S_HOME_ADR
S_GEAR_M->S_GEAR_ADR
S_ERROR_M->S_ERROR_ADR
S_CLNT_M->S_CLNT_ADR
S_MACHINE_M->S_MACHINE_ADR
S_MOTION_M->S_MOTION_ADR
S_TIMEBASE_M->S_TIMEBASE_ADR
S_PROG_STATUS_M->S_PROG_STATUS_ADR

//  Command
C_MODE_M->C_MODE_ADR
C_AXIS_M->C_AXIS_ADR
C_JOG_M->C_JOG_ADR
C_FEED_M->C_FEED_ADR
C_DIST_M->C_DIST_ADR
C_SOVRD_M->C_SOVRD_ADR
C_SPND_M->C_SPND_ADR
C_FOVRD_M->C_FOVRD_ADR
C_ROVRD_M->C_ROVRD_ADR
C_COORD_M->C_COORD_ADR
C_HOME_M->C_HOME_ADR
C_GEAR_M->C_GEAR_ADR
C_ERROR_M->C_ERROR_ADR
C_CLNT_M->C_CLNT_ADR
C_MACHINE_M->C_MACHINE_ADR
C_MOTION_M->C_MOTION_ADR
C_TIMEBASE_M->C_TIMEBASE_ADR
C_PROG_STATUS_M->C_PROG_STATUS_ADR

// Probe Stuff
PR_BITS_M->PR_BITS_ADR
PR_COMMAND_M->PR_COMMAND_ADR
PR_DATA_M->PR_DATA_ADR
G65_LEVEL->G65_LEVEL_ADR
G65_LEVEL2->G65_LEVEL2_ADR
PMAC_RT_BLOCK->PMAC_RT_BLOCK_ADR
PMAC_RT_BLOCK2->PMAC_RT_BLOCK2_ADR
PMAC_RT_NM_BLOCK->PMAC_RT_NM_BLOCK_ADR
PMAC_RT_NM_BLOCK2->PMAC_RT_NM_BLOCK2_ADR

// RESET DPR HANDSHAKE
PMAC_RESET_COMPLETE->PMAC_RESET_COMP_ADR
PC_RESET_COMPLETE->PC_RESET_COMP_ADR
PMAC_ROT_BUF_FIL_STOP->ROT_BUF_FIL_STOP_ADR

VS_L_NLABEL_M->VS_L_NLABEL_ADR
VS_L_NLABEL2_M->VS_L_NLABEL2_ADR

AT_DPR_POINTER->AT_DPR_POINTER_ADR  // AS DEFAULT, POINT TO UNLIKELY TO BE USED
DPR_POINTER->DPR_POINTER_ADR        // POINTS TO M248 ADDRESSING PART

#endif

#endif



