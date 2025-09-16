/*
        File: linklist.h

        PMAC Address Link List structure

        Author(s):      Ed Lay

        12/06/93      EBL     Created

        Change log:

          Date       Rev     Who         Description
        ---------    -----   -----   -----------------------------------------
        01/11/4              EBL     Changes for windows DLL
        02/05/4              EBL     Add links for L-Variable MOD & DIV
        03/03/95             EBL     ADD fb_addr[8] & fb_read_addr, fb_write_addr
        10/20/00             EBL     Add New JMP Vector ADDr 1.937 TURBO
        11/21/01             AS      REMOVE fb_addr[8] & fb_read_addr, fb_write_addr
*/

struct {
//
//      Function Address
//
unsigned long func_addr[16 ]; //  = { I_SUB ,
                              // M_SUB ,
                              // P_SUB ,
                              // Q_SUB ,
                              // FSIN  ,
                              // FCOS  ,
                              // FTAN  ,
                              // FABS  ,
                              // FASIN ,
                              // FACOS ,
                              // FATAN ,
                              // FSQRT ,
                              // FEXP  ,
                              // FLN   ,
                              // FINT  ,
                              // FATAN2 };

unsigned long opr_push_addr[8 ]; // = {              // push stack Operator Addresses
                                 // FADD_XA,
                                 // FSUB_XA,
                                 // FMPY_XA,
                                 // FDIV_XA,
                                 // FOR_XA ,
                                 // FEOR_XA,
                                 // FAND_XA,
                                 // FMOD_XA  };

//
//      Operator Address Order - the POP stack direction
//
//      +
//      -       // FSUB_AX
//      *
//      /       // FDIV_AX
//      |
//      ^
//      &
//      %       // FMOD_AX
//
unsigned long opr_pop_addr[8 ]; // = {              // pop stack Operator Addresses
                                //       FADD_XA,
                                //       FSUB_AX,
                                //       FMPY_XA,
                                //       FDIV_AX,
                                //       FOR_XA ,
                                //       FEOR_XA,
                                //       FAND_XA,
                                //       FMOD_AX  };
unsigned long imrd_addr[2 ];    //  = { I_READ,
                                //  M_READ };

unsigned long imwrt_addr[2 ]; //  = { I_WRITE,
                              //  M_WRITE };

unsigned long fneg_addr;      // = FNEG;
unsigned long fcmp_xa_addr;   // = FCMP_XA;
unsigned long pack_addr;      // = PACK;
unsigned long unpack_addr;    // = UNPACK;
unsigned long plc_ena_addr;   // = PLCENADIS;
unsigned long plcq_base_addr; // PLCQ_BASE;
unsigned long movp_disp_addr; // MOVPDISP
unsigned long ftoaf_addr;     // FTOAF
unsigned long modx1a_addr;
unsigned long modax1_addr;
unsigned long divx1a_addr;
unsigned long divax1_addr;
unsigned long fb_read_addr;   // LDD PLCC FB_READ
unsigned long fb_write_addr;  //   LDD PLCC FB_WRITe
unsigned long plcn_str_addr;  // Added for TURBO PLCC 1..31 string move
unsigned long plc0_str_addr;  // Added for TURBO PLCC 0 string move
unsigned long plc_str_addr;   // Added for TURBO X:ADDR for PLC0
                              // Added for TURBO X:ADDR+1 for PLCN
unsigned long ftol_addr;      // Added for TURBO X:ADDR for FTOL function
unsigned long ltof_addr;      // Added for TURBO X:ADDR for LTOF 1.937
unsigned long l_read_addr;    // Added for TURBO X:ADDR for 1.937
unsigned long l_write_addr;   // Added for TURBO X:ADDR for 1.937
unsigned long l_read0_addr;   // Added for TURBO X:ADDR for 1.937
unsigned long l_write0_addr;  // Added for TURBO X:ADDR for 1.937
unsigned long lock_addr;      // Added for TURBO X:ADDR for 1.937
unsigned long unlock_addr;    // Added for TURBO X:ADDR for 1.937
unsigned long set_phase_addr; // Added for TURBO X:ADDR for 1.937
unsigned long plc_adr_pn_addr; // Added for TURBO X:ADDR for 1.937
unsigned long qread_a_addr;   // Added for TURBO X:ADDR for 1.937
unsigned long qread_x_addr;   // Added for TURBO X:ADDR for 1.937
unsigned long qwrite_a_addr;  // Added for TURBO X:ADDR for 1.937
unsigned long mi_read_addr;  // Added for MACRO16 PLCC
unsigned long mm_read_addr;  // Added for MACRO16 PLCC
unsigned long mi_write_addr;  // Added for MACRO16 PLCC
unsigned long mm_write_addr;  // Added for MACRO16 PLCC
unsigned long chn_write_addr; // Added for MACRO16 PLCC
} pmaclnk;
