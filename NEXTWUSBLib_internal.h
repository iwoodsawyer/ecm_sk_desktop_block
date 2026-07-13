// ============================================================================
// NEXTWUSBLib_internal.h
// Internal definitions shared between NEXTWUSBLib.cpp and helper modules.
// NOT part of the public API.
// ============================================================================
#pragma once

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <SetupAPI.h>
#include <winusb.h>
#include <cfgmgr32.h>
#include <devpkey.h>
#include <cstdio>
#include <cstring>
#include <cassert>

// --------------------------------------------------------------------------
// EC-01M USB Device Identity
// Vendor ID  : 0x04B4  (Cypress Semiconductor — typical for EC-01M)
// Product ID : 0x00F1  (EC-01M bulk-transfer device)
//
// !! Replace with the real VID/PID from your device's INF or Device Manager !!
// --------------------------------------------------------------------------
#define ECM_USB_VID              0x04B4
#define ECM_USB_PID              0x00F1

// WinUSB Device Interface GUID — must match the INF / OS descriptor GUID
// {A5DCBF10-6530-11D2-901F-00C04FB951ED} is the generic WinUSB GUID;
// replace with the device-specific GUID from your INF if available.
static const GUID ECM_DEVICE_INTERFACE_GUID = {
    0xA5DCBF10, 0x6530, 0x11D2,
    { 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED }
};

// --------------------------------------------------------------------------
// USB pipe endpoint addresses for bulk transfers
// Adjust IN/OUT addresses to match the actual device endpoints
// (discoverable via WinUSB_QueryPipe at runtime — see OpenECMUSB impl.)
// --------------------------------------------------------------------------
#define ECM_PIPE_OUT_DEFAULT     0x01   // Bulk OUT  EP1
#define ECM_PIPE_IN_DEFAULT      0x81   // Bulk IN   EP1

// --------------------------------------------------------------------------
// Timeout for USB transfers (milliseconds)
// --------------------------------------------------------------------------
#define ECM_USB_TIMEOUT_MS       3000

// --------------------------------------------------------------------------
// Internal device state (one global instance — not thread-safe by design;
// wrap with a mutex if concurrent access is needed)
// --------------------------------------------------------------------------
typedef struct _ECM_DEVICE_CTX {
    HANDLE          hFile;          // CreateFile handle
    WINUSB_INTERFACE_HANDLE hWinUsb;// WinUSB interface handle
    UCHAR           pipeOut;        // Bulk-OUT pipe address
    UCHAR           pipeIn;         // Bulk-IN  pipe address
    BOOL            isOpen;         // TRUE while device is open
} ECM_DEVICE_CTX;

// --------------------------------------------------------------------------
// Logging helper (writes to OutputDebugString; replace with your logger)
// --------------------------------------------------------------------------
static inline void ECM_Log(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    OutputDebugStringA(buf);
    // Optionally also print to console:
    // printf("%s", buf);
}