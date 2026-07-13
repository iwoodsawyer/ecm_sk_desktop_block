// ============================================================================
// EC01M_SFunction.h
// Shared definitions for the EC-01M EtherCAT S-Function
// Platform : Windows (Simulink Desktop Real-Time, MSVC x64)
// Toolchain : MATLAB mex / Simulink Real-Time kernel
// ============================================================================
#pragma once

// Select 16-byte variant (DEF_MA_MAX=32) for velocity feedback.
// Default: 12-byte variant (DEF_MA_MAX=42), velocity output is zeroed.
// #define USE_16B

#include "NEXTWUSBLib.h"

#define S_FUNCTION_NAME   EC01M_SFunction
#define S_FUNCTION_LEVEL  2

#define ECM_MAX_SLAVES    (DEF_MA_MAX - 2)   // slots 0=ctrl, last=SDO

// ---------------------------------------------------------------------------
// S-Function parameters
// ---------------------------------------------------------------------------
#define NUM_SPARAMS          7

#define PARAM_NUM_SLAVES     0   // scalar  – number of configured slaves
#define PARAM_SLAVE_TYPES    1   // 1×N uint8 – SLAVE_DRIVE/IO/HSP/STEP/NONE
#define PARAM_DRIVE_MODE     2   // scalar  – CSP_MODE(8)/CSV_MODE(9)/CST_MODE(10)
#define PARAM_SYNC_MODE      3   // scalar  – FREERUN(0)/DCSYNC(1)
#define PARAM_DC_CYCLE_US    4   // scalar  – DC cycle time µs
#define PARAM_SAMPLE_TIME    5   // scalar  – Simulink sample time (seconds)
#define PARAM_IO_REFRESH     6   // scalar  – refresh IO inputs every N ticks

// ---------------------------------------------------------------------------
// Input ports  [V2 FIX-1: merged three target ports into one]
// ---------------------------------------------------------------------------
#define INPORT_ENABLE        0   // double scalar  – 0=off, 1=servo-on
#define INPORT_TARGET_CMD    1   // int32[N]  – pos (CSP) / vel (CSV) / torq (CST)
#define INPORT_IO_OUT        2   // uint32    – IO output bitmask
#define NUM_INPORTS          3

// ---------------------------------------------------------------------------
// Output ports
// ---------------------------------------------------------------------------
#define OUTPORT_ACTUAL_POS   0   // int32[N]  – rspBuf[i].Data1  (0x6064)
#define OUTPORT_STATUS_WORD  1   // uint32[N] – rspBuf[i].Data2  (0x6041) 
#define OUTPORT_ACTUAL_VEL   2   // int32[N]  – rspBuf[i].Data3  (0x606C, 16B only)
#define OUTPORT_ECM_STATE    3   // double    – rspBuf[0].Parm & 0xFF
#define OUTPORT_FIFO_COUNT   4   // double    – rspBuf[0].Data2 & 0xFFFF 
#define OUTPORT_IO_IN        5   // uint32    – rspBuf[0].Data1 after LIO_RD
#define NUM_OUTPORTS         6

// ---------------------------------------------------------------------------
// IWork
// ---------------------------------------------------------------------------
#define IWORK_STATE_IDX      0   // current EtherCAT state
#define IWORK_INIT_DONE      1   // 1 after mdlStart succeeds
#define IWORK_SERVO_ON       2   // 1 while servos energised
#define IWORK_TICK           3   // step counter for IO refresh
#define NUM_IWORKS           4

// ---------------------------------------------------------------------------
// PWork
// ---------------------------------------------------------------------------
#define PWORK_CTX_IDX        0
#define NUM_PWORKS           1

// ---------------------------------------------------------------------------
// Internal context
// ---------------------------------------------------------------------------
typedef struct _ECM_SFUNC_DATA {
    transData  cmdBuf[DEF_MA_MAX];
    transData  rspBuf[DEF_MA_MAX];
    int        numSlaves;
    BYTE       slaveType[DEF_MA_MAX];
    int        driveMode;            // CSP_MODE / CSV_MODE / CST_MODE
    int        syncMode;             // FREERUN / DCSYNC
    int        dcCycleUs;
    int        ioRefreshTicks;       // IO frame every N motion ticks
    uint32_T   ioInputCache;         // last known IO input value
} ECM_SFUNC_DATA;