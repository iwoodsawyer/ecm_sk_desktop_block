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
#include <hidclass.h>
#include <hidusage.h>
#include <hidsdi.h> 
#include <devpkey.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cassert>
#include "mex.h"

// --------------------------------------------------------------------------
// EC-01M USB Device Identity
// Vendor ID  : 0x16C0  (Van Ooijen Technische Informatica — V-USB shared VID)
// Product ID : 0x05DF  (Generic HID device — not mice/keyboards/joysticks)
//
// The ECM-SK device enumerates as a USB HID class device (VID 16C0/PID 05DF),
// but can also interface via the standard Windows WinUSB.sys driver.
// WinUSB provides direct bulk endpoint access without HID report framing,
// making it more suitable for high-throughput streaming when the device
// firmware supports it (i.e. has configured bulk IN/OUT endpoints).
// --------------------------------------------------------------------------
#define ECM_USB_VID              0x16C0
#define ECM_USB_PID              0x05DF

// --------------------------------------------------------------------------
// WinUSB Device Interface GUID
// Must match the INF / OS descriptor GUID. The generic WinUSB GUID below
// {A5DCBF10-6530-11D2-901F-00C04FB951ED} works for many devices;
// replace with the device-specific GUID from your INF if available.
// --------------------------------------------------------------------------

// HID Interface GUID
//static const GUID GUID_DEVINTERFACE_HID = {
//    0x4d1e55b2, 0xf16f, 0x11cf,
//    { 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 }
//};

// USB Device Interface GUID
static const GUID GUID_DEVINTERFACE_USB_DEVICE = {
    0xa5dcbf10, 0x6530, 0x11d2,
    { 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed }
};

// USB Device Class GUID
static const GUID GUID_DEVCLASS_USB_DEVICE = {
    0x88bae032, 0x5a81, 0x49f0,
    { 0xbc, 0x3d, 0xa4, 0xff, 0x13, 0x82, 0x16, 0xd6 }
};

// --------------------------------------------------------------------------
// USB pipe endpoint addresses for bulk transfers
// Adjust IN/OUT addresses to match the actual device endpoints
// (discoverable via WinUsb_QueryPipe at runtime — see OpenECMUSB impl.)
// These are compile-time defaults used if pipe discovery fails.
// --------------------------------------------------------------------------
#define ECM_PIPE_OUT_DEFAULT     0x01   // Bulk OUT  EP1
#define ECM_PIPE_IN_DEFAULT      0x81   // Bulk IN   EP1

// --------------------------------------------------------------------------
// Timeout for USB transfers (milliseconds)
// Applied to bulk transfer operations via WinUsb_SetPipePolicy(...,
// PIPE_TRANSFER_TIMEOUT, ...). The WinUSB driver enforces this timeout
// internally, preventing a stalled or disconnected device from blocking
// the caller indefinitely. This is critical for real-time systems where
// timing predictability is required.
// --------------------------------------------------------------------------
#define ECM_USB_TIMEOUT_MS       3000

// --------------------------------------------------------------------------
// Internal device state (one global instance)
// NOT thread-safe by design; wrap with a mutex/critical section if
// concurrent access from multiple threads is needed. The intended usage
// pattern assumes all I/O is synchronized by the calling real-time loop
// (e.g. mdlOutputs / mdlUpdate in Simulink S-Function model).
//
// CRITICAL DESIGN NOTES FOR REAL-TIME OPERATION:
// ─────────────────────────────────────────────
// 1. PERSISTENT RESOURCE ALLOCATION
//    All resource allocation (file handles, WinUSB interfaces) happens ONCE
//    in OpenECMUSB and persists until CloseECMUSB. This avoids per-tick kernel
//    object allocation overhead that would otherwise introduce scheduling
//    jitter into real-time control loops. Resources are reused across multiple
//    transfer cycles, not recreated for each call.
//
// 2. DEVICE PATH CACHING
//    The devicePath is cached at open time (before CreateFile) so that
//    ECMUSBRecover() can reopen the same PHYSICAL device without re-enumerating.
//    This is ESSENTIAL when multiple ECM-SK modules are connected simultaneously.
//    
//    Why caching matters:
//    - SetupAPI enumeration returns devices in order (index 0, 1, 2, ...)
//    - If device A is at index 0 and device B at index 1, and device A is
//      disconnected, device B may shift to index 0 on re-enumeration
//    - Re-enumeration without path caching could return a DIFFERENT physical
//      device, breaking device identity and desynchronizing protocol traffic
//    - Caching the exact path prevents this silent device identity loss
//
// 3. HOLD-LAST RECOVERY STRATEGY
//    Transient failures (timeout, stall) do NOT close or reset pipes from
//    within ECMUSBWrite/ECMUSBRead. Instead, the last valid data is held and
//    retried in the next cycle. Only ECMUSBRecover() (called at shutdown) 
//    performs explicit reset operations. This prevents real-time loop
//    disturbance from error handling.
//
// 4. COMPILE-TIME ENDPOINT DEFAULTS
//    If WinUsb_QueryPipe fails at open time, compile-time defaults
//    (ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT) are used. This allows
//    graceful degradation when device firmware doesn't expose standard
//    endpoint layouts.
// --------------------------------------------------------------------------
typedef struct _ECM_DEVICE_CTX {
    HANDLE                  hFile;      // CreateFile handle to the USB device.
                                        // Allocated once in OpenECMUSB,
                                        // persists until CloseECMUSB to avoid
                                        // per-tick kernel allocation overhead.
    
    WINUSB_INTERFACE_HANDLE hWinUsb;    // WinUSB interface handle (obtained
                                        // from WinUsb_Initialize). Persists
                                        // for the lifetime of the device.
                                        // Used for all bulk transfer and
                                        // policy operations.
										
	WINUSB_INTERFACE_HANDLE hWinUsbIf1; // WinUSB Interface 1 handle 
	                                    // (Scenario A only) obtained by:
                                        //  WinUsb_GetAssociatedInterface.
    
    UCHAR                   pipeOut;    // Bulk-OUT pipe address (0x01..0x0F).
                                        // Discovered at open time via
                                        // WinUsb_QueryPipe. Cached for reuse
                                        // across all ECMUSBWrite calls.
                                        // Used in ECMUSBWrite operations.
    
    UCHAR                   pipeIn;     // Bulk-IN pipe address (0x81..0x8F).
                                        // Discovered at open time via
                                        // WinUsb_QueryPipe. Cached for reuse
                                        // across all ECMUSBRead calls.
                                        // Used in ECMUSBRead operations.
    
    BOOL                    isOpen;     // TRUE while device is open and both
                                        // hFile and hWinUsb are valid.
                                        // Serves as the primary state flag
                                        // for device availability.

    BOOL                    foundViaHID;// Set TRUE by ECM_FindDevicePath()
                                        // when the device was discovered via 
                                        // the HID class GUID (SEARCH 2) AND 
                                        // ECM_FindHIDDeviceGetCompositePath()
										// successfully re-derived a WinUSB 
										// composite path for the same device.

    WCHAR                   devicePath[MAX_PATH];
                                        // Exact device path used when the
                                        // device was opened. Cached at open
                                        // time (BEFORE CreateFile) so
                                        // ECMUSBRecover can reopen the same
                                        // physical device without re-enumerating.
                                        // CRITICAL when multiple ECM-SK modules
                                        // are connected: re-enumeration might
                                        // return a different unit at a different
                                        // enumeration index, breaking device
                                        // identity and protocol synchronization.
                                        // 
                                        // Intentionally NOT cleared by
                                        // CloseECMUSB so ECMUSBRecover can
                                        // read it after calling CloseECMUSB
                                        // to reopen the same device.
} ECM_DEVICE_CTX;

// --------------------------------------------------------------------------
// Logging helper (writes to OutputDebugString; replace with your logger)
// Safe for real-time use (no allocation or blocking I/O if no debugger
// is attached). OutputDebugStringA is synchronous and should be kept
// out of the main I/O loop for production builds to avoid timing jitter.
//
// The logging design follows these principles:
// - No per-call kernel object allocation
// - Buffer-based formatting (stack allocation, not heap)
// - Synchronous output (no queuing or threading overhead)
// - Optional: can be wrapped with compile-time guards for production
//   builds to eliminate logging overhead entirely
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
    mexPrintf("%s", buf);
}