// ============================================================================
// NEXTWUSBLib_internal.h
// Internal definitions shared between NEXTWUSBLib.cpp and helper modules.
// NOT part of the public API.
// ============================================================================
#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>      // HidD_GetHidGuid, HidD_GetAttributes, HidD_GetProductString
#include <hidusage.h>
#include <devpkey.h>
#include <cstdarg>       // va_list, va_start, va_end
#include <cstdio>
#include <cstring>
#include <cassert>

// --------------------------------------------------------------------------
// EC-01M USB Device Identity
// Vendor ID  : 0x16C0  (Van Ooijen Technische Informatica — V-USB shared VID)
// Product ID : 0x05DF  (Generic HID device — not mice/keyboards/joysticks)
//
// The ECM-SK enumerates as a USB HID class device under HidUsb.sys.
// No Zadig / driver replacement is needed — Windows installs HidUsb.sys
// automatically on first connection.
// --------------------------------------------------------------------------
#define ECM_USB_VID              0x16C0
#define ECM_USB_PID              0x05DF

// --------------------------------------------------------------------------
// HID Report ID
// V-USB / ECM-SK firmware uses Report ID 0 (no explicit report ID byte in
// the HID descriptor). Windows HID API still requires a Report ID prefix
// byte in every WriteFile / ReadFile buffer; use 0x00 for Report ID 0.
// --------------------------------------------------------------------------
#define ECM_HID_REPORT_ID        0x00

// --------------------------------------------------------------------------
// Timeout for USB transfers (milliseconds).
// Applied to both ECMUSBRead and ECMUSBWrite via WaitForSingleObject.
// The device handle is opened with FILE_FLAG_OVERLAPPED so that both
// ReadFile and WriteFile can be issued with an OVERLAPPED struct and
// cancelled via CancelIoEx if this deadline is exceeded.  A stalled or
// disconnected device cannot block the real-time thread beyond this limit.
// --------------------------------------------------------------------------
#define ECM_USB_TIMEOUT_MS       3000

// --------------------------------------------------------------------------
// Internal device state (one global instance)
// --------------------------------------------------------------------------
typedef struct _ECM_DEVICE_CTX {
    HANDLE   hFile;                  // CreateFile handle to the HID device
    BOOL     isOpen;                 // TRUE while device is open
    HANDLE   hEventRead;             // Manual-reset event for overlapped reads.
                                     // Allocated once in OpenECMUSB, freed in
                                     // ECM_CloseHandle. ResetEvent() before each
                                     // ReadFile call to avoid per-tick kernel alloc.
    HANDLE   hEventWrite;            // Manual-reset event for overlapped writes.
                                     // Same lifetime and usage as hEventRead.
    WCHAR    devicePath[MAX_PATH];   // Exact path used when the device was opened.
                                     // Cached so ECMUSBRecover can reopen the same
                                     // physical device without re-enumerating —
                                     // critical when multiple ECM-SK modules are
                                     // connected simultaneously.
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
}