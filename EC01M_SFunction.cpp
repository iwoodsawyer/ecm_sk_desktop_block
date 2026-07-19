// ============================================================================
// EC01M_SFunction.cpp
// Simulink C-MEX Level-2 S-Function for NEXTW EC-01M EtherCAT Master
// Supports Simulink Desktop Real-Time on Windows (WinUSB / SetupAPI)
//
// Build:
//   mex -R2018a EC01M_SFunction.cpp NEXTWUSBLib.lib ^
//       -I<path-to-headers> -L<path-to-lib> COMPFLAGS="$COMPFLAGS /EHsc /W3"
//
// S-Function Parameters (ssGetSFcnParam):
//   0 – numSlaves    : scalar  – number of configured EtherCAT slaves
//   1 – slaveTypes   : 1×N     – slave type per slot (DRIVE/IO/HSP/STEP/NONE)
//   2 – driveMode    : scalar  – CSP_MODE(8) / CSV_MODE(9) / CST_MODE(10)
//   3 – syncMode     : scalar  – FREERUN(0) / DCSYNC(1)
//   4 – dcCycleUs    : scalar  – Distributed Clock cycle time in µs
//   5 – sampleTime   : scalar  – Simulink sample time in seconds
//   6 – ioRefreshTicks: scalar – refresh IO inputs every N real-time ticks
//
// Input Ports:
//   u0 – Enable       [double scalar]   – 0 = servo off, 1 = servo on
//   u1 – TargetCmd[N] [int32 × N]       – target command per slave;
//                                          CSP = position  (encoder counts)
//                                          CSV = velocity  (counts/s)
//                                          CST = torque    (drive units)
//                                          (mode is fixed by driveMode at init)
//   u2 – IOOutputs    [uint32 scalar]   – bitmask written to LIO_WR / IO_WR
//
// Output Ports:
//   y0 – ActualPos[N]  [int32  × N]    – actual position per slave (0x6064)
//   y1 – StatusWord[N] [uint32 × N]    – CiA 402 status word per slave (0x6041)
//   y2 – ActualVel[N]  [int32  × N]    – actual velocity per slave (0x606C);
//                                          valid only when USE_16B is defined,
//                                          zero otherwise
//   y3 – ECMState      [double scalar] – current EtherCAT state
//                                          (rspBuf[0].Parm & 0xFF)
//   y4 – FIFOCount     [double scalar] – USB FIFO remaining count
//                                          (rspBuf[0].Data2 & 0xFFFF)
//   y5 – IOInputs      [uint32 scalar] – ECM local digital inputs
//                                          (rspBuf[0].Data1 after LIO_RD frame)
// ============================================================================

// ============================================================================
//  S-Function identity
// ============================================================================
#define S_FUNCTION_NAME   EC01M_SFunction
#define S_FUNCTION_LEVEL  2

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstring>
#include <cstdlib>

// Include the correct header for your hardware variant.
// Define USE_16B before the include to switch to the 16-byte packet variant
// (DEF_MA_MAX=32, Data3 field available for velocity feedback).
// Default: 12-byte variant (DEF_MA_MAX=42); velocity output is zeroed.
// #define USE_16B
#include "NEXTWUSBLib.h"
#include "simstruc.h"

// Usable motion slave slots: slot 0 = control/status, last slot = SDO
#define ECM_MAX_SLAVES    (DEF_MA_MAX - 2)

// ============================================================================
//  S-Function parameter indices
// ============================================================================
#define NUM_SPARAMS           7
#define PARAM_NUM_SLAVES      0
#define PARAM_SLAVE_TYPES     1
#define PARAM_DRIVE_MODE      2
#define PARAM_SYNC_MODE       3
#define PARAM_DC_CYCLE_US     4
#define PARAM_SAMPLE_TIME     5
#define PARAM_IO_REFRESH      6   // refresh IO inputs every N ticks

// ============================================================================
//  Input port indices
// ============================================================================
#define INPORT_ENABLE         0   // double scalar  – servo enable
#define INPORT_TARGET_CMD     1   // int32[N]       – target command per slave
#define INPORT_IO_OUT         2   // uint32 scalar  – IO output bitmask
#define NUM_INPORTS           3

// ============================================================================
//  Output port indices
// ============================================================================
#define OUTPORT_ACTUAL_POS    0   // int32[N]  – actual position  (0x6064)
#define OUTPORT_STATUS_WORD   1   // uint32[N] – CiA 402 status word (0x6041)
#define OUTPORT_ACTUAL_VEL    2   // int32[N]  – actual velocity (0x606C, USE_16B only)
#define OUTPORT_ECM_STATE     3   // double    – current EtherCAT state
#define OUTPORT_FIFO_COUNT    4   // double    – USB FIFO remaining count
#define OUTPORT_IO_IN         5   // uint32    – ECM local digital inputs
#define NUM_OUTPORTS          6

// ============================================================================
//  IWork indices
// ============================================================================
#define IWORK_STATE_IDX       0   // current EtherCAT state value
#define IWORK_INIT_DONE       1   // 1 after mdlStart completes successfully
#define IWORK_SERVO_ON        2   // 1 while servos are energised
#define IWORK_TICK            3   // step counter used for IO refresh scheduling
// Two new IWork slots for non-blocking servo-ready wait
#define IWORK_SVON_WAIT       4   // 1 while waiting for drives to become ready
#define IWORK_SVON_TICK       5   // tick value at which SV_ON frame was sent
#define NUM_IWORKS            6   // was 4

// ============================================================================
//  PWork indices
// ============================================================================
#define PWORK_CTX_IDX         0
#define NUM_PWORKS            1

// ============================================================================
//  Servo-ready settling time in seconds.
//  Drives are given this long after SV_ON before motion commands are sent.
//  1.0 s matches the original Sleep(1000) duration.
// ============================================================================
#define ECM_SVON_SETTLE_S     1.0

// ============================================================================
//  Per-instance context block (allocated in mdlStart, freed in mdlTerminate)
// ============================================================================
typedef struct _ECM_SFUNC_DATA {
    transData  cmdBuf[DEF_MA_MAX];  // outgoing command frame
    transData  rspBuf[DEF_MA_MAX];  // incoming response frame
    int        numSlaves;           // number of configured EtherCAT slaves
    BYTE       slaveType[DEF_MA_MAX]; // slave type per slot
    int        driveMode;           // CSP_MODE / CSV_MODE / CST_MODE
    int        syncMode;            // FREERUN / DCSYNC
    int        dcCycleUs;           // Distributed Clock cycle time in µs
    int        ioRefreshTicks;      // issue IO frame every N motion ticks
    uint32_T   ioInputCache;        // last value read by the IO refresh frame
    real_T     sampleTime;          // Simulink sample time (seconds)
} ECM_SFUNC_DATA;

// ============================================================================
//  Utility helpers
// ============================================================================

/// Zero-initialise the entire command buffer.
static inline void ClearCmdData(ECM_SFUNC_DATA *ctx)
{
    memset(ctx->cmdBuf, 0, sizeof(ctx->cmdBuf));
}

static bool WaitForResponse(ECM_SFUNC_DATA *ctx,
                             int slaveIdx, int column,
                             uint32_T expect, unsigned maxRetries,
                             uint32_T *lastSeen = nullptr)
{
    uint32_T realValue = 0;

    for (unsigned retried = 0; retried <= maxRetries; retried++)
    {
        memset(ctx->rspBuf, 0, sizeof(ctx->rspBuf));
        
        // Log the USB read call
        bool readOk = ECMUSBRead(reinterpret_cast<unsigned char*>(ctx->rspBuf),
                                 sizeof(ctx->rspBuf));
        
        if (!readOk && retried == 0)
        {
            // Only log the first read failure to avoid spam
            mexPrintf("[ECM WaitForResponse] FIRST READ FAILED at retry 0\n"
                      "[ECM WaitForResponse]   ECMUSBRead returned FALSE\n"
                      "[ECM WaitForResponse]   Check NEXTWUSBLib.cpp ECM_Log output\n");
        }

        switch (column)
        {
        case 0:
            realValue = (slaveIdx == DEF_MA_MAX - 1)
                      ?  (uint32_T)ctx->rspBuf[slaveIdx].CMD
                      : ((uint32_T)ctx->rspBuf[slaveIdx].CMD & 0xFFFFu);
            break;
        case 1:
            realValue = (slaveIdx == 0)
                      ? ((uint32_T)ctx->rspBuf[slaveIdx].Parm & 0xFFu)
                      :  (uint32_T)ctx->rspBuf[slaveIdx].Parm;
            break;
        case 2:
            realValue = ctx->rspBuf[slaveIdx].Data1;
            break;
        default:
            realValue = ctx->rspBuf[slaveIdx].Data2;
            break;
        }

        if (retried < 3 || retried % 50 == 0)  // log first 3 retries, then every 50th
        {
            mexPrintf("[ECM WaitForResponse] retry=%u  slot=%d  got=0x%02X  want=0x%02X  readOk=%s\n",
                      retried, slaveIdx, (unsigned)realValue, (unsigned)expect,
                      readOk ? "YES" : "NO");
        }

        if (realValue == expect)
        {
            mexPrintf("[ECM WaitForResponse] SUCCESS at retry=%u  state=0x%02X\n",
                      retried, (unsigned)realValue);
            if (lastSeen) *lastSeen = realValue;
            return true;
        }

        if (retried < maxRetries)
        {
            if (retried > 0)
                ECMUSBAbortOut();

            transData clearData[DEF_MA_MAX];
            memset(clearData, 0, sizeof(clearData));
            bool writeOk = ECMUSBWrite(reinterpret_cast<unsigned char*>(clearData),
                                       sizeof(clearData));
            
            if (retried == 0 && !writeOk)
            {
                mexPrintf("[ECM WaitForResponse] FIRST WRITE FAILED at retry 0\n");
            }

            Sleep(10);
        }
    }

    if (lastSeen) *lastSeen = realValue;
    return false;
}

/// Read a scalar integer S-Function parameter.
static inline int GetIntParam(SimStruct *S, int idx)
{
    return static_cast<int>(mxGetScalar(ssGetSFcnParam(S, idx)));
}

/// Returns true if this slave type accepts motion commands (CSP/CSV/CST/SV_ON/SV_OFF).
static inline bool IsMotionSlave(BYTE st)
{
    return (st == SLAVE_DRIVE || st == SLAVE_HSP || st == SLAVE_STEP);
}

/// Returns true if this slave type is a digital I/O slave.
static inline bool IsIOSlave(BYTE st)
{
    return (st == SLAVE_IO);
}

// ============================================================================
//  mdlInitializeSizes
// ============================================================================
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_SPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    for (int i = 0; i < NUM_SPARAMS; i++)
        ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);

    int numSlaves = GetIntParam(S, PARAM_NUM_SLAVES);
    if (numSlaves < 1)             numSlaves = 1;
    if (numSlaves > ECM_MAX_SLAVES) numSlaves = ECM_MAX_SLAVES;

    // -------------------------------------------------------------------------
    // Input ports
    // -------------------------------------------------------------------------
    if (!ssSetNumInputPorts(S, NUM_INPORTS)) return;

    // Port 0 – Enable (double scalar)
    ssSetInputPortWidth             (S, INPORT_ENABLE,     1);
    ssSetInputPortDataType          (S, INPORT_ENABLE,     SS_DOUBLE);
    ssSetInputPortDirectFeedThrough (S, INPORT_ENABLE,     1);
    ssSetInputPortRequiredContiguous(S, INPORT_ENABLE,     1);

    // Port 1 – TargetCmd[N] (int32 × numSlaves)
    // Drive mode is fixed at initialisation; this single port carries
    // position, velocity, or torque depending on the driveMode parameter.
    ssSetInputPortWidth             (S, INPORT_TARGET_CMD, numSlaves);
    ssSetInputPortDataType          (S, INPORT_TARGET_CMD, SS_INT32);
    ssSetInputPortDirectFeedThrough (S, INPORT_TARGET_CMD, 1);
    ssSetInputPortRequiredContiguous(S, INPORT_TARGET_CMD, 1);

    // Port 2 – IOOutputs (uint32 scalar)
    ssSetInputPortWidth             (S, INPORT_IO_OUT,     1);
    ssSetInputPortDataType          (S, INPORT_IO_OUT,     SS_UINT32);
    ssSetInputPortDirectFeedThrough (S, INPORT_IO_OUT,     1);
    ssSetInputPortRequiredContiguous(S, INPORT_IO_OUT,     1);

    // -------------------------------------------------------------------------
    // Output ports
    // -------------------------------------------------------------------------
    if (!ssSetNumOutputPorts(S, NUM_OUTPORTS)) return;

    // y0 – ActualPos[N] (int32 × numSlaves)
    // rspBuf[i].Data1 — actual position feedback (CiA 402 object 0x6064)
    ssSetOutputPortWidth   (S, OUTPORT_ACTUAL_POS,  numSlaves);
    ssSetOutputPortDataType(S, OUTPORT_ACTUAL_POS,  SS_INT32);

    // y1 – StatusWord[N] (uint32 × numSlaves)
    // rspBuf[i].Data2 — CiA 402 status word (object 0x6041)
    ssSetOutputPortWidth   (S, OUTPORT_STATUS_WORD, numSlaves);
    ssSetOutputPortDataType(S, OUTPORT_STATUS_WORD, SS_UINT32);

    // y2 – ActualVel[N] (int32 × numSlaves)
    // rspBuf[i].Data3 — actual velocity (CiA 402 object 0x606C)
    // Only populated when USE_16B is defined (16-byte packet variant).
    // Always zero in 12-byte mode; the field does not exist in that variant.
    ssSetOutputPortWidth   (S, OUTPORT_ACTUAL_VEL,  numSlaves);
    ssSetOutputPortDataType(S, OUTPORT_ACTUAL_VEL,  SS_INT32);

    // y3 – ECMState (double scalar)
    // rspBuf[0].Parm & 0xFF — current EtherCAT state reported by the master
    ssSetOutputPortWidth   (S, OUTPORT_ECM_STATE,   1);
    ssSetOutputPortDataType(S, OUTPORT_ECM_STATE,   SS_DOUBLE);

    // y4 – FIFOCount (double scalar)
    // rspBuf[0].Data2 & 0xFFFF — USB FIFO remaining count
    // Use this in Simulink logic to detect USB back-pressure; throttle
    // writes if the count falls below a safe threshold (e.g. 80).
    ssSetOutputPortWidth   (S, OUTPORT_FIFO_COUNT,  1);
    ssSetOutputPortDataType(S, OUTPORT_FIFO_COUNT,  SS_DOUBLE);

    // y5 – IOInputs (uint32 scalar)
    // rspBuf[0].Data1 read from a dedicated LIO_RD / IO_RD frame.
    // Refreshed every ioRefreshTicks steps; holds the last known value
    // between refresh frames.
    ssSetOutputPortWidth   (S, OUTPORT_IO_IN,       1);
    ssSetOutputPortDataType(S, OUTPORT_IO_IN,       SS_UINT32);

    // -------------------------------------------------------------------------
    // Sample time and work vectors
    // -------------------------------------------------------------------------
    ssSetNumSampleTimes(S, 1);
    ssSetNumIWork      (S, NUM_IWORKS);   // now 6
    ssSetNumPWork      (S, NUM_PWORKS);

    ssSetOptions(S,
        SS_OPTION_EXCEPTION_FREE_CODE |
        SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

// ============================================================================
//  mdlInitializeSampleTimes
// ============================================================================
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T st = mxGetScalar(ssGetSFcnParam(S, PARAM_SAMPLE_TIME));
    ssSetSampleTime(S, 0, st);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


// ============================================================================
// Diagnostic wrapper for ECMUSBWrite — logs details on failure
// ============================================================================
static bool DiagnosticECMUSBWrite(SimStruct *S, unsigned char *data, 
                                   unsigned long dwLength)
{
    mexPrintf("[ECM DiagnosticECMUSBWrite] Attempting write of %lu bytes...\n", dwLength);
    
    // Log the expected vs actual size
    mexPrintf("[ECM DiagnosticECMUSBWrite] Expected frame size: %lu bytes "
              "(sizeof(transData)*DEF_MA_MAX = %zu*%d)\n",
              (unsigned long)(sizeof(transData) * DEF_MA_MAX),
              sizeof(transData), DEF_MA_MAX);
    
    // Log the first 32 bytes of what we're sending (for debugging)
    mexPrintf("[ECM DiagnosticECMUSBWrite] First 32 bytes of buffer: ");
    for (int i = 0; i < 32 && i < (int)dwLength; i++) {
        mexPrintf("%02X ", data[i]);
    }
    mexPrintf("\n");

    bool result = ECMUSBWrite(data, dwLength);
    
    if (!result) {
        mexPrintf("[ECM DiagnosticECMUSBWrite] FAILED!\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite] Check NEXTWUSBLib.cpp logs for details\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite] Possible causes:\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite]   1. Device disconnected after OpenECMUSB()\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite]   2. OVERLAPPED handle is invalid\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite]   3. DLL size mismatch (recompile both?)\n");
        mexPrintf("[ECM DiagnosticECMUSBWrite]   4. Buffer passed is not 504 bytes exactly\n");
    } else {
        mexPrintf("[ECM DiagnosticECMUSBWrite] Success\n");
    }
    
    return result;
}

// ============================================================================
// Query function - try a simple GET_STATUS read to test the device
// ============================================================================
static bool DiagnosticTestRead(SimStruct *S)
{
    mexPrintf("[ECM DiagnosticTestRead] Attempting a test READ to verify device...\n");
    
    transData testBuf[DEF_MA_MAX];
    memset(testBuf, 0, sizeof(testBuf));
    
    bool readOk = ECMUSBRead(reinterpret_cast<unsigned char*>(testBuf), sizeof(testBuf));
    
    mexPrintf("[ECM DiagnosticTestRead] ECMUSBRead returned: %s\n", 
              readOk ? "TRUE" : "FALSE");
    
    if (readOk) {
        mexPrintf("[ECM DiagnosticTestRead] Read succeeded! Device is responding.\n");
        mexPrintf("[ECM DiagnosticTestRead] rspBuf[0].Parm (state) = 0x%02X\n",
                  testBuf[0].Parm & 0xFF);
        mexPrintf("[ECM DiagnosticTestRead] rspBuf[0].Data2 (FIFO) = 0x%04X\n",
                  testBuf[0].Data2 & 0xFFFF);
    } else {
        mexPrintf("[ECM DiagnosticTestRead] Read FAILED!\n");
        mexPrintf("[ECM DiagnosticTestRead] This suggests a fundamental USB/HID issue\n");
    }
    
    return readOk;
}

// ============================================================================
//  mdlStart  — hardware initialisation sequence
//
//  Mirrors the vendor sample application step numbering:
//    Step 2  – OpenECMUSB
//    Step 3  – Set EtherCAT state → PRE-OP
//    Step 4  – Configure slave topology (SET_AXIS)
//    Step 5  – Configure Distributed Clock (SET_DC)
//    Step 6  – Set drive mode (DRIVE_MODE)
//    Step 7  – Set state → SAFE-OP
//    Step 8  – Set state → OPERATIONAL
// ============================================================================
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // -------------------------------------------------------------------------
    // Allocate per-instance context
    // -------------------------------------------------------------------------
    ECM_SFUNC_DATA *ctx =
        static_cast<ECM_SFUNC_DATA*>(calloc(1, sizeof(ECM_SFUNC_DATA)));
    if (!ctx) {
        ssSetErrorStatus(S, "EC01M: calloc failed — out of memory.");
        return;
    }
    ssGetPWork(S)[PWORK_CTX_IDX] = ctx;

    // -------------------------------------------------------------------------
    // Read S-Function parameters into context
    // -------------------------------------------------------------------------
    ctx->numSlaves      = GetIntParam(S, PARAM_NUM_SLAVES);
    ctx->driveMode      = GetIntParam(S, PARAM_DRIVE_MODE);
    ctx->syncMode       = GetIntParam(S, PARAM_SYNC_MODE);
    ctx->dcCycleUs      = GetIntParam(S, PARAM_DC_CYCLE_US);
    ctx->ioRefreshTicks = GetIntParam(S, PARAM_IO_REFRESH);
    ctx->sampleTime     = mxGetScalar(ssGetSFcnParam(S, PARAM_SAMPLE_TIME)); 
	
    if (ctx->numSlaves < 1)              ctx->numSlaves = 1;
    if (ctx->numSlaves > ECM_MAX_SLAVES) ctx->numSlaves = ECM_MAX_SLAVES;
    if (ctx->ioRefreshTicks < 1)         ctx->ioRefreshTicks = 1;

    // Unpack slave type array from the parameter matrix
    memset(ctx->slaveType, SLAVE_NONE, sizeof(ctx->slaveType));
    const mxArray *stParam = ssGetSFcnParam(S, PARAM_SLAVE_TYPES);
    int            stLen   = static_cast<int>(mxGetNumberOfElements(stParam));
    const double  *stData  = mxGetPr(stParam);
    for (int i = 0; i < stLen && i < DEF_MA_MAX; i++)
        ctx->slaveType[i] = static_cast<BYTE>(stData[i]);

    mexPrintf("[ECM] DEF_MA_MAX = %d, sizeof(transData) = %zu, "
              "ECM_FRAME_BYTES should be %zu\n",
              DEF_MA_MAX, sizeof(transData),
              (size_t)(sizeof(transData) * DEF_MA_MAX));

    // =========================================================================
    // STEP 2 — Open the USB port
    // =========================================================================
    mexPrintf("[ECM mdlStart] === STEP 2: Opening USB ===\n");
    if (!OpenECMUSB()) {
        mexPrintf("[ECM mdlStart] CRITICAL: OpenECMUSB() returned FALSE\n");
        ssSetErrorStatus(S, "EC01M: OpenECMUSB failed — check USB connection.");
        free(ctx);
        ssGetPWork(S)[PWORK_CTX_IDX] = nullptr;
        return;
    }
    mexPrintf("[ECM mdlStart] OpenECMUSB() succeeded\n");
    
    Sleep(1000);
    mexPrintf("[ECM mdlStart] Post-open delay complete\n");

    // =========================================================================
    // DIAGNOSTIC: Try a READ first to test the device link
    // =========================================================================
    mexPrintf("[ECM mdlStart] === DIAGNOSTIC: Testing device link with READ ===\n");
    bool testReadOk = DiagnosticTestRead(S);
    
    if (!testReadOk) {
        mexPrintf("[ECM mdlStart] DIAGNOSTIC: Device READ failed!\n");
        mexPrintf("[ECM mdlStart] This is a fundamental USB/HID communication issue\n");
        mexPrintf("[ECM mdlStart] The device may be:\n");
        mexPrintf("[ECM mdlStart]   1. Not actually connected (check Device Manager NOW)\n");
        mexPrintf("[ECM mdlStart]   2. In a bad state (try power-cycling)\n");
        mexPrintf("[ECM mdlStart]   3. Using wrong HID driver\n");
        mexPrintf("[ECM mdlStart]   4. SLDRT kernel mode incompatible with HID API\n");
        ssSetErrorStatus(S, "EC01M: Diagnostic READ failed. See MATLAB console for details.");
        CloseECMUSB();
        return;
    }
    
    mexPrintf("[ECM mdlStart] DIAGNOSTIC: Device READ OK - proceeding to WRITE\n");

    // =========================================================================
    // STEP 3 — Transition EtherCAT state machine to PRE-OP
    // =========================================================================
    mexPrintf("[ECM mdlStart] === STEP 3: Sending SET_STATE -> PRE-OP ===\n");
    
    ClearCmdData(ctx);
    ctx->cmdBuf[0].CMD   = SET_STATE;
    ctx->cmdBuf[0].Data1 = STATE_PRE_OP;
    
    mexPrintf("[ECM mdlStart] cmdBuf buffer info:\n");
    mexPrintf("[ECM mdlStart]   addr = %p\n", (void*)ctx->cmdBuf);
    mexPrintf("[ECM mdlStart]   size = %zu bytes\n", sizeof(ctx->cmdBuf));
    
    bool writeOk = DiagnosticECMUSBWrite(S, 
                                         reinterpret_cast<unsigned char*>(ctx->cmdBuf), 
                                         sizeof(ctx->cmdBuf));
    
    if (!writeOk) {
        mexPrintf("[ECM mdlStart] CRITICAL: ECMUSBWrite failed!\n");
        mexPrintf("[ECM mdlStart] READ worked but WRITE failed — asymmetric error\n");
        mexPrintf("[ECM mdlStart] This suggests:\n");
        mexPrintf("[ECM mdlStart]   1. OVERLAPPED write event is invalid\n");
        mexPrintf("[ECM mdlStart]   2. Kernel mode incompatibility with WriteFile\n");
        mexPrintf("[ECM mdlStart]   3. USB write endpoint is stalled\n");
        ssSetErrorStatus(S, "EC01M: ECMUSBWrite failed. Device read works but write doesn't.");
        CloseECMUSB();
        return;
    }
    
    mexPrintf("[ECM mdlStart] SET_STATE write succeeded\n");
    
    if (!WaitForResponse(ctx, 0, 1, STATE_PRE_OP, 200)) {
        ssSetErrorStatus(S, "EC01M: Timeout waiting for PRE-OP.");
        CloseECMUSB(); 
        return;
    }
    mexPrintf("[ECM mdlStart] PRE-OP confirmed\n");

    // =========================================================================
    // STEP 4 — SET_AXIS: define slave topology
    //   slaveType[0..DEF_MA_MAX-1] maps slave positions to slave types.
    //   Topology is packed as 4-bit nibbles, 8 slaves per 32-bit word,
    //   sent 8 slaves at a time (one group per write).
    // =========================================================================
    const int SLV_PER_GRP = 8;
    int       numGroups   = DEF_MA_MAX / SLV_PER_GRP;  // 5 for 12B, 4 for 16B

    for (int grp = 0; grp < numGroups; grp++)
    {
        uint32_T topology = 0;
        for (int s = 0; s < SLV_PER_GRP; s++)
        {
            int absIdx = grp * SLV_PER_GRP + s;
            topology |= (static_cast<uint32_T>(ctx->slaveType[absIdx]) << (s * 4));
        }
        ClearCmdData(ctx);
        ctx->cmdBuf[0].CMD   = SET_AXIS;
        ctx->cmdBuf[0].Parm  = static_cast<unsigned short>(grp);
        ctx->cmdBuf[0].Data1 = topology;
        ctx->cmdBuf[0].Data2 = 0;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
        Sleep(2);
    }

    // =========================================================================
    // STEP 5 — SET_DC: configure Distributed Clock
    //   Data1 = cycle time in microseconds
    //   Data2 = 0xFFFF → automatic DC offset adjustment
    // =========================================================================
    ClearCmdData(ctx);
    ctx->cmdBuf[0].CMD   = SET_DC;
    ctx->cmdBuf[0].Parm  = 0;
    ctx->cmdBuf[0].Data1 = static_cast<unsigned int>(ctx->dcCycleUs); // cycle time in µs
    ctx->cmdBuf[0].Data2 = 0xFFFF;  // auto offset
    ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
    Sleep(2);

    // =========================================================================
    // STEP 6 — DRIVE_MODE: set drive mode + sync on every drive/HSP/STEP slave
    //   driveMode: CSP_MODE(8) / CSV_MODE(9) / CST_MODE(10)
    //   syncMode:  FREERUN(0) / DCSYNC(1)
    //   Note: STEP slaves do not have a torque PDO; CST_MODE is downgraded
    //         to CSP_MODE for STEP slaves automatically.
    // =========================================================================
    ClearCmdData(ctx);
    for (int i = 1; i < DEF_MA_MAX - 1; i++)
    {
        BYTE st = ctx->slaveType[i - 1];
        if (IsMotionSlave(st))
        {
            int dm = ctx->driveMode;
            if (st == SLAVE_STEP && dm == CST_MODE)
                dm = CSP_MODE;  // STEP slaves do not have a torque PDO
            ctx->cmdBuf[i].CMD   = DRIVE_MODE;
            ctx->cmdBuf[i].Data1 = static_cast<unsigned int>(dm);
            ctx->cmdBuf[i].Data2 = static_cast<unsigned int>(ctx->syncMode);
        }
    }
    ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
    Sleep(2);

    // =========================================================================
    // STEP 7 — Transition to SAFE-OP
    // =========================================================================
    ClearCmdData(ctx);
    ctx->cmdBuf[0].CMD   = SET_STATE;
    ctx->cmdBuf[0].Data1 = STATE_SAFE_OP;
    ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
    if (!WaitForResponse(ctx, 0, 1, STATE_SAFE_OP, 1000)) {
        ssSetErrorStatus(S, "EC01M: Timeout waiting for SAFE-OP.");
        CloseECMUSB(); return;
    }

    // =========================================================================
    // STEP 8 — Transition to OPERATIONAL
    // =========================================================================
    ClearCmdData(ctx);
    ctx->cmdBuf[0].CMD   = SET_STATE;
    ctx->cmdBuf[0].Data1 = STATE_OPERATIONAL;
    ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
    if (!WaitForResponse(ctx, 0, 1, STATE_OPERATIONAL, 1000)) {
        ssSetErrorStatus(S, "EC01M: Timeout waiting for OPERATIONAL.");
        CloseECMUSB(); return;
    }
    Sleep(3000); // Allow drives time to become ready after OP transition

    // -------------------------------------------------------------------------
    // Initialise work vectors
    // -------------------------------------------------------------------------
    ssGetIWork(S)[IWORK_STATE_IDX] = STATE_OPERATIONAL;
    ssGetIWork(S)[IWORK_INIT_DONE] = 1;
    ssGetIWork(S)[IWORK_SERVO_ON]  = 0;
    ssGetIWork(S)[IWORK_TICK]      = 0;
    ssGetIWork(S)[IWORK_SVON_WAIT] = 0;
    ssGetIWork(S)[IWORK_SVON_TICK] = 0;
    ctx->ioInputCache              = 0;
}

// ============================================================================
//  mdlOutputs  — called every real-time tick
//
//  Per-tick sequence:
//    1. Detect servo-enable rising / falling edge → SV_ON / SV_OFF frame
//    2. Non-blocking servo-ready wait: count ticks instead of Sleep()
//    3. Every ioRefreshTicks: issue a dedicated LIO_RD / IO_RD frame and
//       cache the result in ctx->ioInputCache
//    4. Build and send the cyclic motion frame (CSP / CSV / CST + IO_WR)
//    5. Read response; on USB error hold last output values (no pipe reset)
// ============================================================================
static void mdlOutputs(SimStruct *S, int_T /*tid*/)
{
    if (!ssGetIWork(S)[IWORK_INIT_DONE]) return;

    ECM_SFUNC_DATA *ctx =
        static_cast<ECM_SFUNC_DATA*>(ssGetPWork(S)[PWORK_CTX_IDX]);
    if (!ctx) return;

    // ---- Read inputs --------------------------------------------------------
    const real_T   *enableIn  = (const real_T*  )ssGetInputPortSignal(S, INPORT_ENABLE);
    const int32_T  *targetCmd = (const int32_T* )ssGetInputPortSignal(S, INPORT_TARGET_CMD);
    const uint32_T *ioOut     = (const uint32_T*)ssGetInputPortSignal(S, INPORT_IO_OUT);

    bool enable   = (enableIn[0] != 0.0);
    bool servoWas = (ssGetIWork(S)[IWORK_SERVO_ON] != 0);
    int  tick     = ssGetIWork(S)[IWORK_TICK];

    // =========================================================================
    // STEP 9 — Servo ON: rising edge only
    //   Replaced Sleep(1000) with a non-blocking tick-counting wait.
    //   On the rising edge: send SV_ON frame, record the current tick, set
    //   the SVON_WAIT flag, and return immediately — no blocking.
    //   On every subsequent tick while SVON_WAIT is set: check whether
    //   (tick - svOnTick) * sampleTime >= ECM_SVON_SETTLE_S (1.0 s).
    //   Until the settle time has elapsed, issue GET_STATUS keep-alive frames
    //   so the USB link stays active, then return without sending motion.
    //   Once the settle time has elapsed, clear SVON_WAIT and fall through
    //   to normal cyclic motion on this same tick.
    // =========================================================================
    if (enable && !servoWas)
    {
        // Rising edge — send SV_ON and start the non-blocking settle timer
        ClearCmdData(ctx);
        for (int i = 1; i < DEF_MA_MAX - 1; i++)
            if (IsMotionSlave(ctx->slaveType[i - 1]))
                ctx->cmdBuf[i].CMD = SV_ON;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));

        ssGetIWork(S)[IWORK_SERVO_ON]  = 1;
        ssGetIWork(S)[IWORK_SVON_WAIT] = 1;
        ssGetIWork(S)[IWORK_SVON_TICK] = tick;
        ssGetIWork(S)[IWORK_TICK]      = tick + 1;
        return;  // Skip motion this tick — settle timer just started
    }

    if (ssGetIWork(S)[IWORK_SVON_WAIT])
    {
        // Still inside the servo-ready settling window — check elapsed time
        int    svOnTick = ssGetIWork(S)[IWORK_SVON_TICK];
        real_T elapsed  = static_cast<real_T>(tick - svOnTick) * ctx->sampleTime;

        if (elapsed < ECM_SVON_SETTLE_S)
        {
            // Not ready yet — send a GET_STATUS keep-alive to maintain USB
            // traffic, update tick, and return without sending motion commands.
            ClearCmdData(ctx);
            ctx->cmdBuf[0].CMD = GET_STATUS;
            ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
            memset(ctx->rspBuf, 0, sizeof(ctx->rspBuf));
            ECMUSBRead(reinterpret_cast<unsigned char*>(ctx->rspBuf), sizeof(ctx->rspBuf));
            ssGetIWork(S)[IWORK_TICK] = tick + 1;
            return;
        }

        // Settle time has elapsed — clear wait flag and continue to motion
        ssGetIWork(S)[IWORK_SVON_WAIT] = 0;
    }

    // =========================================================================
    // STEP 11 — Servo OFF: falling edge only
    // =========================================================================
    if (!enable && servoWas)
    {
        ClearCmdData(ctx);
        for (int i = 1; i < DEF_MA_MAX - 1; i++)
            if (IsMotionSlave(ctx->slaveType[i - 1]))
                ctx->cmdBuf[i].CMD = SV_OFF;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
        ssGetIWork(S)[IWORK_SERVO_ON] = 0;
        // Fall through to read feedback after SV_OFF
    }

    // =========================================================================
    // STEP 13 — Read Digital Inputs (ECM local I/O + IO slaves)
    //   Issued as a dedicated frame every ioRefreshTicks steps.
    //   ECM local inputs  → slot 0  with LIO_RD command
    //   IO slave inputs   → slot i  with IO_RD  command
    //   Result cached in ctx->ioInputCache; held between refresh frames.
    // =========================================================================
    if (tick % ctx->ioRefreshTicks == 0)
    {
        ClearCmdData(ctx);
        ctx->cmdBuf[0].CMD = LIO_RD;   // ECM local inputs
        for (int i = 1; i < DEF_MA_MAX - 1; i++)
            if (IsIOSlave(ctx->slaveType[i - 1]))
                ctx->cmdBuf[i].CMD = IO_RD;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));

        memset(ctx->rspBuf, 0, sizeof(ctx->rspBuf));
        ECMUSBRead(reinterpret_cast<unsigned char*>(ctx->rspBuf), sizeof(ctx->rspBuf));

        // Cache ECM local inputs from slot 0
        ctx->ioInputCache = ctx->rspBuf[0].Data1;
    }

    // =========================================================================
    // STEP 10 — Cyclic motion frame
    //   Slot 0          : GET_STATUS (control / FIFO status)
    //   Slots 1..N-2    : CSP / CSV / CST per motion slave, IO_WR per IO slave
    //   Last slot (N-1) : reserved for SDO (left as GET_STATUS here)
    // =========================================================================
    ClearCmdData(ctx);
    ctx->cmdBuf[0].CMD = GET_STATUS;

    if (enable)
    {
        for (int i = 1; i < DEF_MA_MAX - 1; i++)
        {
            BYTE st = ctx->slaveType[i - 1];
            int  si = i - 1;   // user-facing slave index (0-based)

            if (IsMotionSlave(st) && si < ctx->numSlaves)
            {
                // Drive mode is fixed at initialisation; driveMode selects
                // which cyclic command and PDO object is used each tick.
                // STEP slaves have no torque PDO; CST_MODE is guarded here
                // as a safety net (already downgraded to CSP at DRIVE_MODE time).
                int dm = ctx->driveMode;
                if (st == SLAVE_STEP && dm == CST_MODE) dm = CSP_MODE;

                switch (dm)
                {
                case CSP_MODE:
                    ctx->cmdBuf[i].CMD   = CSP;
                    ctx->cmdBuf[i].Data1 = static_cast<unsigned int>(targetCmd[si]);
                    break;
                case CSV_MODE:
                    ctx->cmdBuf[i].CMD   = CSV;
                    ctx->cmdBuf[i].Data1 = static_cast<unsigned int>(targetCmd[si]);
                    break;
                case CST_MODE:
                    ctx->cmdBuf[i].CMD   = CST;
                    ctx->cmdBuf[i].Data1 = static_cast<unsigned int>(targetCmd[si]);
                    break;
                default:
                    ctx->cmdBuf[i].CMD = GET_STATUS;
                    break;
                }
            }
            else if (IsIOSlave(st))
            {
                // =========================================================
                // STEP 14 — Write Digital Outputs (IO slave)
                // =========================================================
                ctx->cmdBuf[i].CMD   = IO_WR;
                ctx->cmdBuf[i].Data1 = ioOut[0];
            }
        }

        // ECM local outputs in slot 0 alongside GET_STATUS
        ctx->cmdBuf[0].Data1 = ioOut[0];
    }

    // =========================================================================
    // Write motion frame → device, read feedback ← device
    //   Response frame layout per slot (motion slaves):
    //     Data1 – actual position  (CiA 402 object 0x6064)
    //     Data2 – CiA 402 status word (object 0x6041)
    //     Data3 – actual velocity  (object 0x606C, USE_16B only)
    //
    //   Response frame layout slot 0:
    //     Parm  & 0xFF   – current EtherCAT state
    //     Data2 & 0xFFFF – USB FIFO remaining count
    //
    //    On USB write or read failure, skip updating output ports so
    //   Simulink holds the last good values ("hold last" strategy).
    //   Do NOT call ECM_ResetDevice / ECMUSBRecover here — a pipe reset
    //   takes several milliseconds and will cause a real-time overrun.
    //   ECMUSBRecover() is only called in mdlTerminate (outside the RT loop).
    // =========================================================================
    if (!ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf)))
    {
        // Write failed — hold all outputs at last known values, advance tick
        ssGetIWork(S)[IWORK_TICK] = tick + 1;
        return;
    }

    memset(ctx->rspBuf, 0, sizeof(ctx->rspBuf));
    if (!ECMUSBRead(reinterpret_cast<unsigned char*>(ctx->rspBuf), sizeof(ctx->rspBuf)))
    {
        // Read failed — hold all outputs at last known values, advance tick
        ssGetIWork(S)[IWORK_TICK] = tick + 1;
        return;
    }

    // -------------------------------------------------------------------------
    // Publish outputs
    // -------------------------------------------------------------------------
    int32_T  *actualPos  = (int32_T* )ssGetOutputPortSignal(S, OUTPORT_ACTUAL_POS);
    uint32_T *statusWord = (uint32_T*)ssGetOutputPortSignal(S, OUTPORT_STATUS_WORD);
    int32_T  *actualVel  = (int32_T* )ssGetOutputPortSignal(S, OUTPORT_ACTUAL_VEL);
    real_T   *ecmState   = (real_T*  )ssGetOutputPortSignal(S, OUTPORT_ECM_STATE);
    real_T   *fifoCount  = (real_T*  )ssGetOutputPortSignal(S, OUTPORT_FIFO_COUNT);
    uint32_T *ioInputs   = (uint32_T*)ssGetOutputPortSignal(S, OUTPORT_IO_IN);

    for (int i = 1; i < DEF_MA_MAX - 1; i++)
    {
        int  si = i - 1;   // user-facing slave index
        if (si >= ctx->numSlaves) break;
        BYTE st = ctx->slaveType[si];

        if (IsMotionSlave(st))
        {
            // Data1 = actual position  (0x6064)
            actualPos [si] = static_cast<int32_T>(ctx->rspBuf[i].Data1);

            // Data2 = CiA 402 status word (0x6041)
            statusWord[si] = ctx->rspBuf[i].Data2;

            // Data3 = actual velocity (0x606C)
            // Only valid in 16-byte packet mode; zeroed in 12-byte mode
            // because the Data3 field does not exist in that variant.
#ifdef USE_16B
            actualVel[si] = static_cast<int32_T>(ctx->rspBuf[i].Data3);
#else
            actualVel[si] = 0;   // not available in 12-byte packet variant
#endif
        }
    }

    // Slot 0: EtherCAT state and FIFO remaining count
    // respData[0].Data2 low 16 bits = FIFO remaining count
    ecmState [0] = static_cast<real_T>(ctx->rspBuf[0].Parm  & 0xFFu);
    fifoCount[0] = static_cast<real_T>(ctx->rspBuf[0].Data2 & 0xFFFFu);

    // IO inputs from the dedicated LIO_RD / IO_RD refresh frame (cached)
    ioInputs[0] = ctx->ioInputCache;

    // Update IWork
    ssGetIWork(S)[IWORK_STATE_IDX] = static_cast<int>(ctx->rspBuf[0].Parm & 0xFFu);
    ssGetIWork(S)[IWORK_TICK]      = tick + 1;
}

// ============================================================================
//  mdlTerminate  — safe shutdown sequence
//
//  Mirrors vendor sample steps 11, 15, 16:
//    Step 11 – Servo OFF on all drive/HSP/STEP slaves
//    Step 15 – Set state → INIT
//    Step 16 – CloseECMUSB
// ============================================================================
static void mdlTerminate(SimStruct *S)
{
    ECM_SFUNC_DATA *ctx =
        static_cast<ECM_SFUNC_DATA*>(ssGetPWork(S)[PWORK_CTX_IDX]);
    if (!ctx) return;

    // =========================================================================
    // STEP 11 — Servo OFF on all drive/HSP/STEP slaves
    // =========================================================================
    if (ssGetIWork(S)[IWORK_SERVO_ON])
    {
        ClearCmdData(ctx);
        for (int i = 1; i < DEF_MA_MAX - 1; i++)
            if (IsMotionSlave(ctx->slaveType[i - 1]))
                ctx->cmdBuf[i].CMD = SV_OFF;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
        Sleep(200);
    }

    // =========================================================================
    //   Pipe recovery at shutdown only — safe here because we are
    //   outside the real-time loop. If any USB error occurred during the
    //   run, ECMUSBRecover() clears the stall before the INIT transition.
    // =========================================================================
    ECMUSBRecover();

    // =========================================================================
    // STEP 15 — Return to INIT state
    // =========================================================================
    if (ssGetIWork(S)[IWORK_INIT_DONE])
    {
        ClearCmdData(ctx);
        ctx->cmdBuf[0].CMD   = SET_STATE;
        ctx->cmdBuf[0].Data1 = STATE_INIT;
        ECMUSBWrite(reinterpret_cast<unsigned char*>(ctx->cmdBuf), sizeof(ctx->cmdBuf));
        WaitForResponse(ctx, 0, 1, STATE_INIT, 500);
    }

    // =========================================================================
    // STEP 16 — Close USB
    // =========================================================================
    CloseECMUSB();
    free(ctx);
    ssGetPWork(S)[PWORK_CTX_IDX] = nullptr;
}

// ============================================================================
//  Simulink MEX registration — must appear last
// ============================================================================
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif