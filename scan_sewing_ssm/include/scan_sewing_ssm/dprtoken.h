/***************************************************************************

        Title:          dprtoken.h

        Version:        1.00

        Date:           05/22/97

        Author(s):      Ed Lay

----------------------------------------------------------------------------

        Change log:

       Date       Rev     Who                    Description
  ---------       -----   -----   --------------------------------------------
  05/22/1997                  EBL         Creation

***************************************************************************/

//****************************************************************************
//      Second Pass Token/Date Structure & Token Codes
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
//                        Mnn ==, |=, &= ^= ( EXPRESSION )
//                        All Type 0 A( Exp. ) .. Z( Exp. ) .. CALL( Exp. ) etc.
//****************************************************************************

#define OpI             0
#define OpM             2
#define OpP             4
#define OpQ             6
#define Constant        8
#define OpIMPQMask      0xE
#define Ope             0x20 // so Ope not zero ???

#define OpMaskType      0x80  // Operator/Operatee Type Mask
#define OperatorType    0x80  // Operator Type
#define OperateeType    0x00  // Operatee Type
#define CmdType         0x100 // Command Type
#define OpCmdMask       0x180 // Operator/Operatee & Cmd Type Mask

#define OpNegate        0x01                                 // Operator/Operatee Negate
#define OpeFuncType     0x40                                 // Operatee Function Type
#define OprCondType     0x20                                 // Conditional Type
#define OprCompType     0x10                                 // Comparison Type Operator
#define OprCondNOTType  0x8                                  // Conditional NOT Type
#define OprCondCompType OprCompType+OprCondType+OperatorType // Conditional Comparison Operator
#define OprCmpdCondType OprCondType+OperatorType             // Compound Conditional


#define OprHIType       0x2 // HI priority Operator Type


//****************************************************************************
//      Operatee Types Allowed within an Expression
//****************************************************************************
#define OpFuncAddrMsk   0x1E             // Operator Function Address Mask
#define OpeFuncI        OpI+OpeFuncType  // Operatee Function I
#define OpeFuncM        OpM+OpeFuncType  // Operatee Function M
#define OpeFuncP        OpP+OpeFuncType  // Operatee Function P
#define OpeFuncQ        OpQ+OpeFuncType  // Operatee Function Q
#define OpeFuncSIN      0x08+OpeFuncType // Operatee Function SIN
#define OpeFuncCOS      0x0A+OpeFuncType // Operatee Function COS
#define OpeFuncTAN      0x0C+OpeFuncType // Operatee Function TAN
#define OpeFuncABS      0x0E+OpeFuncType // Operatee Function ABS
#define OpeFuncASIN     0x10+OpeFuncType // Operatee Function ASIN
#define OpeFuncACOS     0x12+OpeFuncType // Operatee Function ACOS
#define OpeFuncATAN     0x14+OpeFuncType // Operatee Function ATAN
#define OpeFuncSQRT     0x16+OpeFuncType // Operatee Function SQRT
#define OpeFuncEXP      0x18+OpeFuncType // Operatee Function EXP
#define OpeFuncLN       0x1A+OpeFuncType // Operatee Function LN
#define OpeFuncINT      0x1C+OpeFuncType // Operatee Function INT
#define OpeFuncATAN2    0x1E+OpeFuncType // Operatee Function ATAN2
#define OpeFuncLp       0x20+OpeFuncType // Operatee Function '('
#define OpeFuncRp       0x22+OpeFuncType // Operatee Function ')'

#define OpeInn          Ope +OpI      // Operatee Inn
#define OpeMnn          Ope +OpM      // Operatee Mnn
#define OpePnn          Ope +OpP      // Operatee Pnn
#define OpeQnn          Ope +OpQ      // Operatee Qnn
#define OpeConst        Ope +Constant // Operatee Constant Values

//****************************************************************************
//      Operator Types Allowed within an Expression
//
//      OprCondType     0x20                    // Conditional Type     100000
//      OprCompType     0x10                    // Comparison  Operator 010000
//      OprCondNOTType  0x8                     // Conditional NOT Type 000100
//      OprCondCompType OprCompType+OprCondType+OperatorType            11xxxx
//      OprCmpdCondType OprCondType+OperatorType                        10xxxx
//****************************************************************************

#define OprADD          0x00+OperatorType // Operator '+'         000000
#define OprSUB          0x01+OperatorType // Operator '-'         000001
#define OprMPY          0x02+OperatorType // Operator '*'         000010
#define OprDIV          0x03+OperatorType // Operator '/'         000011
#define OprOR           0x04+OperatorType // Operator '|'         000100
#define OprXOR          0x05+OperatorType // Operator '^'         000101
#define OprAND          0x06+OperatorType // Operator '&'         000110
#define OprMOD          0x07+OperatorType // Operator '%'         000111
#define OprADDRMSK      0x07              // Mask for addr table

#define OprLOR          0x08+OprCmpdCondType // Operator OR, OR-     10100x
#define OprLAND         0x0A+OprCmpdCondType // Operator AND, AND-   10101x


#define OprEQ           0x00+OprCondCompType // Operator '=',"!="    11X00x
#define OprGT           0x02+OprCondCompType // Operator '>',"!>"    11X01x
#define OprLT           0x04+OprCondCompType // Operator '<',"!<"    11X10x
#define OprAEQ          0x06+OprCondCompType // Operator '~',"!~"    11X11x
                                             // 1ST 'x' is '-'
                                             // 2nd 'x' is '!'


//****************************************************************************
//      Command Types:
//         a. Single( Types: 0A, 0B, 1A, 1B, 1C, 1D, 1E, 1F )
//            These are the ORIGINAL non expressions commands.
//         b. Expression( IF, WHILE
//                        I,M,P,Qnn = ( EXPRESSION )
//                        Mnn ==, |=, &= ^= ( EXPRESSION )
//                        Type 0 A( Exp. ) .. Z( Exp. ) .. CALL( Exp. ) etc.
//****************************************************************************
#define CmdInn          OpI+CmdType // Command Inn = -     000000x
#define CmdMnn          OpM+CmdType // Command Mnn = -     000001x
#define CmdPnn          OpP+CmdType // Command Pnn = -     000010x
#define CmdQnn          OpQ+CmdType // Command Qnn = -     000011x
#define CmdMnn          OpM+CmdType // Command Mnn |= -    000100x ???
#define CmdMnn          OpM+CmdType // Command Mnn ^= -    000101x ???
#define CmdMnn          OpM+CmdType // Command Mnn &= -    000110x ???
#define CmdMnn          OpM+CmdType // Command Mnn == -    000111x ???


#define CodeA           0x1
#define CodeB           0x2
#define CodeC           0x3
#define CodeD           0x4
#define CodeE           0x5
#define CodeF           0x6
#define CodeG           0x7
#define CodeH           0x8
#define CodeI           0x9
#define CodeJ           0x10
#define CodeK           0x11
#define CodeL           0x12
#define CodeM           0x13
#define CodeP           0x16
#define CodeQ           0x17
#define CodeR           0x18
#define CodeS           0x19
#define CodeT           0x20
#define CodeU           0x21
#define CodeV           0x22
#define CodeW           0x23
#define CodeX           0x24
#define CodeY           0x25
#define CodeZ           0x26
#define CodeCall        0x27
#define Code0B          0x28


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

#define Code1Type1  0x1E


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
#define Code2ABS  0
#define Code2INC  1
#define Code2FRAX 2
#define Code2HOME 3
#define Code2HOMEZ 4
#define Code2ADIS 12
#define Code2AROT 13

















#define ExpA            CodeA << 1
#define ExpB            CodeB << 1
#define ExpC            CodeC << 1
#define ExpD            CodeD << 1
#define ExpE            CodeE << 1
#define ExpF            CodeF << 1
#define ExpG            CodeG << 1
#define ExpH            CodeH << 1
#define ExpI            CodeI << 1
#define ExpJ            CodeJ << 1
#define ExpK            CodeK << 1
#define ExpL            CodeL << 1
#define ExpM            CodeM << 1
#define ExpP            CodeP << 1
#define ExpQ            CodeQ << 1
#define ExpR            CodeR << 1
#define ExpS            CodeS << 1
#define ExpT            CodeT << 1
#define ExpU            CodeU << 1
#define ExpV            CodeV << 1
#define ExpW            CodeW << 1
#define ExpX            CodeX << 1
#define ExpY            CodeY << 1
#define ExpZ            CodeZ << 1
#define ExpCall         CodeCall << 1
#define Exp0B           Code0B << 1

#define CmdExp          CmdType // ??? what here

#define CmdExpA         ExpA+CmdExp    // Command A(-
#define CmdExpB         ExpB+CmdExp    // Command B(-
#define CmdExpC         ExpC+CmdExp    // Command C(-
#define CmdExpD         ExpD+CmdExp    // Command D(-
#define CmdExpE         ExpE+CmdExp    // Command E(-
#define CmdExpF         ExpF+CmdExp    // Command F(-
#define CmdExpG         ExpG+CmdExp    // Command G(-
#define CmdExpH         ExpH+CmdExp    // Command H(-
#define CmdExpI         ExpI+CmdExp    // Command I(-
#define CmdExpJ         ExpJ+CmdExp    // Command J(-
#define CmdExpK         ExpK+CmdExp    // Command K(-
#define CmdExpL         ExpL+CmdExp    // Command L(-
#define CmdExpM         ExpM+CmdExp    // Command M(-
#define CmdExpP         ExpP+CmdExp    // Command P(-
#define CmdExpQ         ExpQ+CmdExp    // Command Q(-
#define CmdExpR         ExpR+CmdExp    // Command R(-
#define CmdExpS         ExpS+CmdExp    // Command S(-
#define CmdExpT         ExpT+CmdExp    // Command T(-
#define CmdExpU         ExpU+CmdExp    // Command U(-
#define CmdExpV         ExpV+CmdExp    // Command V(-
#define CmdExpW         ExpW+CmdExp    // Command W(-
#define CmdExpX         ExpX+CmdExp    // Command X(-
#define CmdExpY         ExpY+CmdExp    // Command Y(-
#define CmdExpZ         ExpZ+CmdExp    // Command Z(-
#define CmdExpCall      ExpCall+CmdExp // Command Call(-
#define CmdExp0B        Exp0B+CmdExp   // Command TA .. CCR(-


#define CmdIf           0x04+CmdType+OprCondType // Command If       0100100
#define CmdWhile        0x08+CmdType+OprCondType // Command While    0101000

//****************************************************************************
//      Single 48 Bit Word Command Types
//****************************************************************************
#define CmdSingle       0x40 //                  1xxxxxx

#define CmdSingle0A     0x00+CmdType+CmdSingle // Command Type 0A  1000000
#define CmdSingle0B     0x01+CmdType+CmdSingle // Command Type 0B  1000001
#define CmdSingle1A     0x02+CmdType+CmdSingle // Command Type 1A  1000010
#define CmdSingle1B     0x03+CmdType+CmdSingle // Command Type 1B  1000011
#define CmdSingle1C     0x04+CmdType+CmdSingle // Command Type 1C  1000100
#define CmdSingle1D     0x05+CmdType+CmdSingle // Command Type 1D  1000101
#define CmdSingle1E     0x06+CmdType+CmdSingle // Command Type 1E  1000110
#define CmdSingle1F     0x07+CmdType+CmdSingle // Command Type 1F  1000111
#define CmdSingle2      0x08+CmdType+CmdSingle // Command Type 1F  1001000

//#define EOLType         0x40
//#define CmdEOL          EOLType+CmdType                 // End of Line Code
#define CmdNULL         0 // NULL Command for init


