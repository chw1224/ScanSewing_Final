/***************************************************************************

        Title:          bintoken.h

        Version:        2.00

        Date:           10/19/2000

        Author(s):      Ed Lay

----------------------------------------------------------------------------

        Change log:

       Date       Rev     Who                    Description
  ---------       -----   -----   --------------------------------------------
  04/24/1998               EBL         Creation
  09/15/1999               EBL      Adding final Expression tokens
  10/19/2000               EBL      Add TURBO LOCK, UNLOCK, SETPHASE
***************************************************************************/

//****************************************************************************
//      Second Pass Token/Data Structure & Token Codes
//
//****************************************************************************

//****************************************************************************
//
//      Defines for different types:
//      1. Operatee Types ( In, Mn, Pn, Qn, Functions )
//      2. Operator Types ( +, -, *, /, %, |, &, ^, OR, AND )
//      3. Command Types
//         a. Single( Types: 0A, 0B, 1A, 1B, 1C, 1D, 1E, 1F )
//         b. Expression( IF, WHILE
//                        I,M,P,Qnn = ( EXPRESSION )
//                        I,M,P,Q(Exp) = ( EXPRESSION )
//                        Mnn ==, |=, &= ^= ( EXPRESSION )
//                        M(Exp) = ==, |=, &= ^= ( EXPRESSION )
//                        All Type 0 A( Exp. ) .. Z( Exp. ) .. CALL( Exp. )
//                        TA( Exp. ) .. CCR( Exp. )
//****************************************************************************

#define OpI             0
#define OpM             2
#define OpP             4
#define OpQ             6
#define Constant        8
#define OpL             0xA
#define OpF             0xC
#define OpMTRNum        0xE
#define OpMI            0x10
#define OpMM            0x12
#define OpMP            0x14

#define OpIMPQMask      0x1E

//****************************************************************************
//      Token Types and Control Codes
//****************************************************************************

#define OpMaskType      0x80  // Operator/Operatee Type Mask
#define OperatorType    0x80  // Operator Type
#define OperateeType    0x00  // Operatee Type
#define CmdType         0x100 // Command Type
#define OpCmdMask       0x180 // Operator/Operatee & Cmd Type Mask
#define Exp             0x200 // Expression Token
#define CmdExp          CmdType+Exp
#define CmdIfWhmtch     0x400 // If or While Cmd Match ( only PLCC )
#define CmdElsemtch     0x800 // Else matched on this If( only PLCC )

#define OpNegate        0x01 // Operator/Operatee Negate
#define Ope             0x20 // Operatee Not Function Type
#define OpeFuncType     0x40 // Operatee Function Type

#define OprHIType       0x2                                  // HI priority Operator Type
#define OprCondType     0x20                                 // Operator Conditional Type
#define OprCompType     0x10                                 // Operator Comparison Type Operator
#define OprCondNOTType  0x8                                  // Operator Conditional NOT Type
#define OprCondCompType OprCompType+OprCondType+OperatorType // Conditional Comparison Operator
#define OprCmpdCondType OprCondType+OperatorType             // Compound Conditional

#define CmdSingle       0x40

#define CmdNULL         0 // NULL Command for init

//****************************************************************************
//      Operatee Types Allowed within an Expression
//      OperatorType    0x80                    // Operator Type
//      OperateeType    0x00                    // Operatee Type
//      CmdType         0x100                   // Command Type
//      OpCmdMask       0x180                   // Operator/Operatee & Cmd Type
//      Ope             0x20                    // Operatee Not Function Type
//      OpeFuncType     0x40                    // Operatee Function Type
//      if(token & OpCmdMask == OperateeType)
//****************************************************************************

//****************************************************************************
//      Operatee Types Allowed within an Expression & Not Functions
//      if((token & OpCmdMask == OperateeType) && ( token & OpeFuncType == 0 ))
//      Range 0 - 1E total of 16 ( bit 0 = "-" )
//****************************************************************************

#define OpeInn          Ope +OpI      // Operatee Inn              0000100000
#define OpeMnn          Ope +OpM      // Operatee Mnn              0000100010
#define OpePnn          Ope +OpP      // Operatee Pnn              0000100100
#define OpeQnn          Ope +OpQ      // Operatee Qnn              0000100110
#define OpeConst        Ope +Constant // Operatee Constant Values  0000101000
#define OpeLnn          Ope +OpL      // Operatee Lnn              0000101010
#define OpeFnn          Ope +OpF      // Operatee Fnn              0000101100
#define OpeMTRNum       Ope +OpMTRNum // Operatee MTRNum           0000101110
#define OpeMInn         Ope +OpMI     // Operatee Inn              0000110000
#define OpeMMnn         Ope +OpMM     // Operatee Mnn              0000110010
#define OpeMPnn         Ope +OpMP     // Operatee Pnn              0000110100
                                      // Spare
                                      // Remaining                 0000111110
                                      //****************************************************************************
                                      //      Operatee Types Allowed within an Expression & Functions
                                      //      if((token & OpCmdMask == OperateeType) && ( token & OpeFuncType == OpeFuncType ))
                                      //      Range 0 - 3E total of 32 ( bit 0 = "-" )
                                      //****************************************************************************

#define OpFuncAddrMsk   0x1F             // Operatee Function Address Mask
#define OpeFuncI        OpI+OpeFuncType  // Operatee Function I    000100000x
#define OpeFuncM        OpM+OpeFuncType  // Operatee Function M    000100001x
#define OpeFuncP        OpP+OpeFuncType  // Operatee Function P    000100010x
#define OpeFuncQ        OpQ+OpeFuncType  // Operatee Function Q    000100011x
#define OpeFuncSIN      0x08+OpeFuncType // Operatee Function SIN  000100100x
#define OpeFuncCOS      0x0A+OpeFuncType // Operatee Function COS  000100101x
#define OpeFuncTAN      0x0C+OpeFuncType // Operatee Function TAN  000100110x
#define OpeFuncABS      0x0E+OpeFuncType // Operatee Function ABS  000100111x
#define OpeFuncASIN     0x10+OpeFuncType // Operatee Function ASIN 000101000x
#define OpeFuncACOS     0x12+OpeFuncType // Operatee Function ACOS 000101001x
#define OpeFuncATAN     0x14+OpeFuncType // Operatee Function ATAN 000101010x
#define OpeFuncSQRT     0x16+OpeFuncType // Operatee Function SQRT 000101011x
#define OpeFuncEXP      0x18+OpeFuncType // Operatee Function EXP  000101100x
#define OpeFuncLN       0x1A+OpeFuncType // Operatee Function LN   000101101x
#define OpeFuncINT      0x1C+OpeFuncType // Operatee Function INT  000101110x
#define OpeFuncATAN2    0x1E+OpeFuncType // Operatee Funct ATAN2   000101111x
#define OpeFuncLp       0x20+OpeFuncType // Operatee Function '('  000110000x
#define OpeFuncRp       0x22+OpeFuncType // Operatee Function ')'  000110001x
#define OpeFuncLpEq     0x24+OpeFuncType // Operatee Function      000110010x
                                         // BinRot Array Assignment
#define OpeFuncLnn      0x26+OpeFuncType // Operatee Func. Lnn[    000110011x
#define OpeFuncFnn      0x28+OpeFuncType // Operatee Func. Fnn[    000110100x
#define OpeFuncLb       0x2A+OpeFuncType // Operatee Function '['  000110101x
#define OpeFuncRb       0x2C+OpeFuncType // Operatee Function '['  000110110x
#define OpeFuncFTOI     0x2E+OpeFuncType // Operatee Func. FTOI(   000110111x
#define OpeFuncITOF     0x30+OpeFuncType // Operatee Func. ITOF(   000111000x
#define OpeFuncFLIMIT   0x32+OpeFuncType // Operatee Func. FLIMIT( 000111001x
#define OpeFuncMI       0x34+OpeFuncType // Operatee Function MI   000111010x
#define OpeFuncMM       0x36+OpeFuncType // Operatee Function MM   000111011x
#define OpeFuncMP       0x38+OpeFuncType // Operatee Function MP   000111100x

                                         // Spare
                                         // Remaining              000011111x
                                         //****************************************************************************
                                         //      Operator Types Allowed within an Expression
                                         //
                                         //      OprCondType     0x20                // Conditional Type     00100000
                                         //      OprCompType     0x10                // Comparison  Operator 00010000
                                         //      OprCondNOTType  0x8                 // Conditional NOT Type 00000100
                                         //      OprCondCompType OprCompType+OprCondType+OperatorType        1011xxxx
                                         //      OprCmpdCondType OprCondType+OperatorType                    1010xxxx
                                         //      OperatorType    0x80                    // Operator Type
                                         //      CmdType         0x100                   // Command Type
                                         //      OpCmdMask       0x180                   // Operator/Operatee & Cmd Type
                                         //      if(token & OpCmdMask == OperatorType)
                                         //****************************************************************************

#define OprADD          0x00+OperatorType // Operator '+'          0010000000
#define OprSUB          0x01+OperatorType // Operator '-'          0010000001
#define OprMPY          0x02+OperatorType // Operator '*'          0010000010
#define OprDIV          0x03+OperatorType // Operator '/'          0010000011
#define OprOR           0x04+OperatorType // Operator '|'          0010000100
#define OprXOR          0x05+OperatorType // Operator '^'          0010000101
#define OprAND          0x06+OperatorType // Operator '&'          0010000110
#define OprMOD          0x07+OperatorType // Operator '%'          0010000111
#define OprADDRMSK      0x07              // Mask for addr table
#define OprComma        0x08+OperatorType // Operator ','          0010001000

                                          // Remaining  to here    0010001111
// Following for "M(Exp) &=" and "IMPQ(Exp)="
#define OprExpOREQ      Exp+0x08+OperatorType // Opr   M(Exp) |=-  101000100x
#define OprExpXOREQ     Exp+0x0A+OperatorType // Opr   M(Exp) ^=-  101000101x
#define OprExpANDEQ     Exp+0x0C+OperatorType // Opr   M(Exp) &=-  101000110x
// *** Above Bits 1-3 must match bits 0-2 of OprOR, XOR, and AND ***
#define OprExpEQ        Exp+0x0E+OperatorType // Opr IMPQ(Exp.)=-  101000111x

#define OprLOR          0x08+OprCmpdCondType // Operator OR, OR-   001010100x
#define OprLAND         0x0A+OprCmpdCondType // Operator AND, AND- 001010101x
#define OprlogMSK       0x0F                 // Logical BIN MASK
                                             // Spare - could start at   000x
                                             // Remaining          001010111x

#define AEQ             6

#define OprEQ           0x00+OprCondCompType // Operator '=',"!="  001011x00x
#define OprGT           0x02+OprCondCompType // Operator '>',"!>"  001011x01x
#define OprLT           0x04+OprCondCompType // Operator '<',"!<"  001011x10x
#define OprAEQ          AEQ+OprCondCompType  // Operator '~',"!~"  001011x11x
                                             // B0 'x' is '-'
                                             // B3 'x' is '!'
#define OprCondMSK      0x1F                 // Mask for Binary Expression

//****************************************************************************
//      Command Types:
//         a. Single( Types: 0A, 0B, 1A, 1B, 1C, 1D, 1E, 1F )
//            These are the ORIGINAL non expressions commands.
//         b. Expression( IF, WHILE
//                        I,M,P,Qnn = ( EXPRESSION )
//                        Mnn ==, |=, &= ^= ( EXPRESSION )
//                        Type 0 A( Exp. ) .. Z( Exp. ) .. CALL( Exp. ) etc.
//
//      CmdType         0x100                   // Command Type
//      OpCmdMask       0x180                   // Operator/Operatee & Cmd Type Mask
//      Exp             0x200                   // Expression Token
//      CmdExp          0x300                   // IMPQ(Exp)= Types & MSync(Exp)== Types
//      CmdIfWhmtch     0x400                   // If or While Cmd Match ( only PLCC )
//      CmdElsemtch     0x800                   // Else matched on this If( only PLCC )
//      OprCondType     0x20                    // Conditional Type     100000
//      CmdSingle       0x40                    // Single - nothing attached
//****************************************************************************

//****************************************************************************
//      Single 48 Bit Word Command Types - Used only in Bin Rotary
//      AND not Conditional
//****************************************************************************

#define CmdSingle0A     0x00+CmdType+CmdSingle // Command Type 0A  0101000000
#define CmdSingle0B     0x01+CmdType+CmdSingle // Command Type 0B  0101000001
#define CmdSingle1A     0x02+CmdType+CmdSingle // Command Type 1A  0101000010
#define CmdSingle1B     0x03+CmdType+CmdSingle // Command Type 1B  0101000011
#define CmdSingle1C     0x04+CmdType+CmdSingle // Command Type 1C  0101000100
#define CmdSingle1D     0x05+CmdType+CmdSingle // Command Type 1D  0101000101
#define CmdSingle1E     0x06+CmdType+CmdSingle // Command Type 1E  0101000110
#define CmdSingle1F     0x07+CmdType+CmdSingle // Command Type 1F  0101000111
#define CmdSingle2      0x08+CmdType+CmdSingle // Command Type 1F  0101001000
                                               // Spare
                                               // Remaining        0101011111


#define CmdInn          OpI+CmdType  // Command Inn = -            010000000x
#define CmdMnn          OpM+CmdType  // Command Mnn = -            010000001x
#define CmdPnn          OpP+CmdType  // Command Pnn = -            010000010x
#define CmdQnn          OpQ+CmdType  // Command Qnn = -            010000011x
                                     //****************************************************************************
                                     //      Command Types - Used only in PLCC
                                     //****************************************************************************
#define CmdLnn          OpL+CmdType  // Command Lnn =              010000101x
#define CmdFnn          OpF+CmdType  // Command Fnn =              010000110x
#define CmdSyncORMnn    0x0E+CmdType // Command Mnn |= -           010000111x

#define CmdMInn         OpMI+CmdType // Command MInn = -           010001000x
#define CmdMMnn         OpMM+CmdType // Command MMnn = -           010001001x
#define CmdMPnn         OpMP+CmdType // Command MPnn = -           010001010x

#define CmdSyncXORMnn   0x16+CmdType // Command Mnn ^= -           010001011x
#define CmdSyncANDMnn   0x18+CmdType // Command Mnn &= -           010001100x
#define CmdSyncORLnn    0x1A+CmdType // Command Lnn |= -           010001101x
#define CmdSyncXORLnn   0x1C+CmdType // Command Lnn ^= -           010001110x
#define CmdSyncANDLnn   0x1E+CmdType // Command Lnn &= -           010001111x

//***************************************************888888888*************************
// Note could convert the CmdSyncXXLnn and CmdSyncXXMnn's to CmdSyncMnn and CmdSyncLnn 
// with the OR, XOR or AND in the DATA type.
// Currently filled to the end.
//************************************************************8888888888****************

//****************************************************************************
//      Command Types - Used only in Bin Rotary
//****************************************************************************
#define CmdA_Call       0x0E+CmdType // Cmd A(- to CALL(-          010000111x
#define CmdTA_CCR       0x10+CmdType // Cmd TA(- to CCR(-          010001000x
#define CmdSyncMnn      0x12+CmdType // Cmd Mnn |=- to ==-         010001001x
                                     // Spare
                                     // Remaining        010001111x

//****************************************************************************
//     Used both in PLCC & Binary Rotary Cmd Tokens
// #define CmdIMPQExpEq    0x2+CmdExp   // Cmd IMPQ(Exp.) = Exp.   110000001x
//****************************************************************************
#define CmdIMPQExpEq    0x2+CmdExp // Cmd IMPQ(+-Exp.) = Type      110000001x
#define CmdSyncMExpEq   0x4+CmdExp // Cmd SyncM(+-Exp.) == Type    110000010x
#define CmdRETExp       0x6+CmdExp // Cmd RET+-Exp.)Type           110000011x

#define CmdIf           0x04+CmdType+OprCondType // Command If     010010010x
#define CmdWhile        0x08+CmdType+OprCondType // Cmmd While     010010100x

// DPR IF/WHILE (- is 1 token
// PLCC IF/WHILE (- is 2 tokens

//****************************************************************************
//     Used only in PLCC Cmd Tokens
//****************************************************************************
#define CmdElse         0x05+CmdType+OprCondType+CmdSingle //      0101100101
#define CmdEndi         0x06+CmdType+OprCondType+CmdSingle //      0101100110
#define CmdEndw         0x09+CmdType+OprCondType+CmdSingle //      0101101001
                                               // Spare
                                               // Remaining        0101111111
#define CmdAND          0x0A+CmdType+OprCondType           //      0100101010
#define CmdOR           0x0B+CmdType+OprCondType           //      0100101011
                                               // Spare
                                               // Remaining        0101011111

#define CmdEnaPLC       0x00+CmdType+CmdSingle // Cmd Enable PLC   0101000000
#define CmdDisPLC       0x01+CmdType+CmdSingle // Cmd Disable PLC  0101000001
#define CmdEnaPLCC      0x02+CmdType+CmdSingle // Cmd Enable PLCC  0101000010
#define CmdDisPLCC      0x03+CmdType+CmdSingle // Cmd Disable PLCC 0101000011
#define CmdCmdSend      0x04+CmdType+CmdSingle // Cmd SEND,SEND^,  0101000100
                                               // Cmd CMD
#define CmdCmdC         0x05+CmdType+CmdSingle // Cmd CMD^CHAR A-Z 0101000101
#define CmdAddress      0x06+CmdType+CmdSingle // Cmd ADDR         0101000110
#define CmdDisplayS     0x07+CmdType+CmdSingle // Cmd DISP String  0101000111
#define CmdDisplayV     0x08+CmdType+CmdSingle // Cmd DISP I,M,P,Q 0101001000
#define CmdOpenPLCC     0x09+CmdType+CmdSingle // Cmd Open PLCC    0101001001
#define CmdClose        0x0A+CmdType+CmdSingle // Cmd CLS Program  0101001010
#define CmdLvarDef      0x0B+CmdType+CmdSingle // Cmd L-var DEF    0101001011
#define CmdFvarDef      0x0C+CmdType+CmdSingle // Cmd F-var DEF    0101001100
#define CmdReturn       0x0D+CmdType+CmdSingle // Cmd Return       0101001101
#define CmdASMout       0x0E+CmdType+CmdSingle // Cmd ASM OUT      0101001110
#define CmdLMOvrly      0x0F+CmdType+CmdSingle // Cmd L&M OVERLY   0101001111
#define CmdLOCK         0x10+CmdType+CmdSingle // Cmd LOCK         0101010000
#define CmdUNLOCK       0x11+CmdType+CmdSingle // Cmd UNLOCK       0101010001
#define CmdSETPHASE     0x12+CmdType+CmdSingle // Cmd SETPHASE     0101010010
#define CmdOpenSERVO    0x13+CmdType+CmdSingle // Cmd Open SERVO   0101010011
#define CmdCOPRSERVO    0x14+CmdType+CmdSingle // Cmd CopyRegSERVO 0101010100
#define CmdOPTImize     0x15+CmdType+CmdSingle // Cmd OPTImize     0101010101
#define CmdOpenMACPLCC  0x16+CmdType+CmdSingle // Cmd Open MACPLCC 0101010110
                                               // Spare
                                               // Remaining        0101011111

//****************************************************************************
//     Bin Rotary Codes
//****************************************************************************

#define CodeA           1
#define CodeB           2
#define CodeC           3
#define CodeD           4
#define CodeE           5
#define CodeF           6
#define CodeG           7
#define CodeH           8
#define CodeI           9
#define CodeJ           10
#define CodeK           11
#define CodeL           12
#define CodeM           13
#define CodeP           16
#define CodeQ           17
#define CodeR           18
#define CodeS           19
#define CodeT           20
#define CodeU           21
#define CodeV           22
#define CodeW           23
#define CodeX           24
#define CodeY           25
#define CodeZ           26
#define CodeCall        27
#define Code0B          28

#define Code1Type0      28
#define Code1Type1      30
#define Code1InnEq      14
#define Code1MnnSync    15

#define Code1A          CodeA
#define Code1B          CodeB
#define Code1C          CodeC
#define Code1D          CodeD
#define Code1E          CodeE
#define Code1F          CodeF
#define Code1G          CodeG
#define Code1H          CodeH
#define Code1I          CodeI
#define Code1J          CodeJ
#define Code1K          CodeK
#define Code1L          CodeL
#define Code1M          CodeM
#define Code1P          CodeP
#define Code1Q          CodeQ
#define Code1R          CodeR
#define Code1S          CodeS
#define Code1T          CodeT
#define Code1U          CodeU
#define Code1V          CodeV
#define Code1W          CodeW
#define Code1X          CodeX
#define Code1Y          CodeY
#define Code1Z          CodeZ
#define Code1Call       CodeCall
#define Code10B         Code0B

#define Code2TA         0
#define Code2TS         1
#define Code2PVT        2
#define Code2TM         67
#define Code2DWELL      68
#define Code2DELAY      69
#define Code2Colen      70
#define Code2Cntrl      71
#define Code2CCR        72
#define Code2ENAPLC  0
#define Code2DISPLC  1
#define Code2ENAPLCC 2
#define Code2DISPLCC 3
#define Code2ABS  0
#define Code2INC  1
#define Code2FRAX 2
#define Code2HOME 3
#define Code2HOMEZ 6
#define Code2CIRn 0
#define Code2LINEAR 1
#define Code2NORM   2
#define Code2PSET   3
#define Code2SPLINEn 4
#define Code2STOP   5
#define Code2BSTART 6
#define Code2BSTOP  7
#define Code2WAIT   8
#define Code2RAPID  9
#define Code2CCn   10
#define Code2TSELn 11
#define Code2ADISn 12
#define Code2AROTn 13
#define Code2IDISn 14
#define Code2IROTn 15
#define Code2TINIT 16
#define Code2PRELUDE 17
#define Code2LSTART 18
#define Code2LSTOP  19
//****************************************************************************
//     Bin Rotary Expression Codes
//****************************************************************************

#define BinExpEOE  0x20
#define BinExpDOD  0x1F // Direct Data code1
