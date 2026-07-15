// ============================================================================
// NEXTWUSBLib.h
// Public header for the NEXTWUSBLib DLL
// USB EtherCAT EC-01M Master Module Driver — Windows API Implementation
//
// The ECM-SK device enumerates as a USB HID class device (VID 16C0 / PID 05DF),
// but uses the standard Windows WinUSB.sys driver interface. 
//
// Supports both 12-byte (DEF_MA_MAX=42) and 16-byte (DEF_MA_MAX=32)
// transData packet variants. Define USE_16B before including to select 16B.
//
// Author : Generated for NEXTW EC-01M
// Platform: Windows (WinUSB / SetupAPI)
// ============================================================================
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------------------
// DLL export/import macro
// --------------------------------------------------------------------------
#ifdef NEXTWUSBLIB_EXPORTS
#   define NEXTWUSB_API __declspec(dllexport)
#else
#   define NEXTWUSB_API __declspec(dllimport)
#endif

// --------------------------------------------------------------------------
// Core DLL API
// --------------------------------------------------------------------------
NEXTWUSB_API bool  __stdcall OpenECMUSB   (void);
NEXTWUSB_API void  __stdcall CloseECMUSB  (void);
NEXTWUSB_API bool  __stdcall ECMUSBWrite  (unsigned char *data, unsigned long dwLength);
NEXTWUSB_API bool  __stdcall ECMUSBRead   (unsigned char *data, unsigned long dwLength);
NEXTWUSB_API void  __stdcall ECMUSBRecover(void); // Closes and reopens the HID device handle to clear any stalled state.

#ifdef __cplusplus
} // extern "C"
#endif

// --------------------------------------------------------------------------
// EtherCAT State-machine values
// --------------------------------------------------------------------------
#define NIC_INIT           0
#define STATE_INIT         1
#define STATE_PRE_OP       2
#define STATE_SAFE_OP      4
#define STATE_OPERATIONAL  8

// --------------------------------------------------------------------------
// Command codes
// --------------------------------------------------------------------------
#define GET_STATUS    0x00
#define SET_STATE     0x01
#define SET_AXIS      0x02
#define SET_DC        0x03
#define SET_EX        0x04
#define SET_FIFO      0x05
#define DRIVE_MODE    0x06
#define SDO_RD        0x07
#define SDO_WR        0x08

#define ALM_CLR       0x10
#define SV_ON         0x11
#define SV_OFF        0x12
#define IO_RD         0x13
#define IO_WR         0x14
#define CSP           0x15
#define CSV           0x16
#define CST           0x17
#define GO_HOME       0x18
#define ABORT_HOME    0x19
#define LIO_RD        0x21
#define LIO_WR        0x22
#define SW_RESET      0xBB

// --------------------------------------------------------------------------
// Drive / sync modes
// --------------------------------------------------------------------------
#define HOMING_MODE   6
#define CSP_MODE      8
#define CSV_MODE      9
#define CST_MODE      10

#define FREERUN       0x00
#define DCSYNC        0x01

// --------------------------------------------------------------------------
// Slave types
// --------------------------------------------------------------------------
#define SLAVE_DRIVE   0x0
#define SLAVE_IO      0x1
#define SLAVE_HSP     0x2
#define SLAVE_STEP    0x3
#define SLAVE_NONE    0xF

// Keep legacy names compatible with sample code
#define DRIVE   SLAVE_DRIVE
#define IO      SLAVE_IO
#define HSP     SLAVE_HSP
#define STEP    SLAVE_STEP
// NOTE: "None" macro omitted here to avoid conflicts; use SLAVE_NONE instead.

// --------------------------------------------------------------------------
// Array dimensions
// --------------------------------------------------------------------------
#ifdef USE_16B
#   define DEF_MA_MAX  32
#else
#   define DEF_MA_MAX  42
#endif

// --------------------------------------------------------------------------
// Transfer packet structures
// --------------------------------------------------------------------------
#pragma pack(push, 1)

#ifdef USE_16B
typedef struct {
    unsigned short CMD;
    unsigned short Parm;
    unsigned int   Data1;
    unsigned int   Data2;
    unsigned int   Data3;
} transData;                        // 16 bytes
#else
typedef struct {
    unsigned short CMD;
    unsigned short Parm;
    unsigned int   Data1;
    unsigned int   Data2;
} transData;                        // 12 bytes
#endif

#pragma pack(pop)