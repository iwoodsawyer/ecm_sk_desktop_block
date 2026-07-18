// ============================================================================
// NEXTWUSBLib.cpp
// Windows API implementation of the NEXTWUSBLib DLL transport layer.
// Interfaces with the ECM-SK EtherCAT Master Module via the standard
// Windows WinUSB.sys driver.
//
// Device identity:
//   VID 0x16C0  (Van Ooijen Technische Informatica — shared V-USB VID)
//   PID 0x05DF  (Generic HID — not mice/keyboards/joysticks)
//
// The ECM-SK device enumerates as a USB COMPOSITE device with two interfaces:
//   Interface 0 : HID class    → HIDClass.sys  (Windows default driver)
//   Interface 1 : Vendor class → WinUSB.sys    (bulk data transport)
//
// The device is initially recognised by Windows as a HID device because
// Interface 0 carries a HID descriptor.  This WinUSB implementation
// side-steps HIDClass entirely and communicates directly with the vendor
// bulk endpoints on Interface 1, providing raw frame access without the
// Report-ID framing overhead of the HID protocol.
//
// Scenario A — Dual-Interface Composite Device:
//   When ECM_FindDevicePath locates the device via the HID class GUID
//   (SEARCH 2), it means Windows has bound HIDClass.sys to Interface 0.
//   The DLL then performs a "Scenario A pivot":
//     1. Re-derives the composite USB root path from the HID child path.
//     2. Opens the composite root with CreateFileW.
//     3. Calls WinUsb_Initialize  → obtains hWinUsb  (Interface 0 handle).
//     4. Calls WinUsb_GetAssociatedInterface(hWinUsb, 0, &hWinUsbIf1)
//        → obtains hWinUsbIf1 (Interface 1 handle).
//     5. All pipe discovery and data I/O run against hWinUsbIf1.
//   "Associated index 0" in step 4 means the SECOND physical interface
//   (Interface 1), not Interface 0. The index is zero-based relative to
//   the interface after the default.
//
//   Prerequisite for Scenario A:
//     WinUSB.sys must be bound to Interface 1 before this DLL can claim it.
//     This can be achieved by:
//       (a) Firmware Microsoft OS 2.0 Descriptor — Windows 8+ auto-loads
//           WinUSB.sys on the vendor interface without any INF.
//       (b) A vendor co-installer INF that was run once at device setup.
//       (c) Zadig (zadig.akeo.ie) — manually assign WinUSB to Interface 1.
//     This DLL does NOT write an INF or install any driver; it assumes the
//     correct driver binding is already in place.
//
// Protocol Framing & Buffer Sizes:
//   WinUSB bulk transfers do NOT require Report ID framing (unlike HID).
//   The caller passes a raw buffer of arbitrary size directly to the endpoint.
//   Actual protocol frame sizes depend on the packet variant:
//   - 12-byte variant: typically 12*42 = 504 bytes per transfer
//   - 16-byte variant: typically 16*32 = 512 bytes per transfer
//   These sizes should match the device firmware's expected frame format.
//
// Device Discovery & Multi-Device Critical System:
//   Three strategies are employed in sequence:
//   1. GUID-based search: SetupAPI enumerates interfaces registered under
//      GUID_DEVINTERFACE_USB_DEVICE (preferred when INF/MS-OS-Descriptor
//      registers the device directly as WinUSB).
//   2. HID fallback + Scenario A pivot: If GUID search fails, the device is
//      located via the HID class GUID. Once found, the WinUSB composite root
//      path is re-derived and Interface 1 is claimed via
//      WinUsb_GetAssociatedInterface.
//   3. All-USB fallback: catch-all via hardware ID scan across every present
//      USB device regardless of class or registered interface GUID.
//
//   *** CRITICAL FOR MULTIPLE ECM-SK UNITS ***
//   The device path is cached at open time so ECMUSBRecover() can reopen the
//   same PHYSICAL device without re-enumerating. When multiple ECM-SK modules
//   are connected simultaneously, re-enumeration returns devices in order
//   (index 0, 1, 2, ...). If one device disconnects, remaining devices may
//   shift indices. Re-enumerating without caching the path could silently
//   reconnect to a DIFFERENT physical device, breaking device identity and
//   protocol synchronization. Path caching prevents this disaster.
//   The foundViaHID flag is also cached so ECMUSBRecover() restores the
//   correct dual-handle Scenario A state without re-enumerating.
//
// Transfer Model Comparison:
//   WinUSB synchronous model vs HID overlapped I/O:
//   - WinUSB: WinUsb_WritePipe/ReadPipe block until complete or timeout.
//     Simpler implementation, no overlapped I/O complexity.
//   - HID: WriteFile/ReadFile with overlapped I/O, manual-reset events,
//     WaitForSingleObject, GetOverlappedResult orchestration.
//   Trade-off: WinUSB trades implementation complexity for timeout
//   predictability and simpler error handling. Both models are suitable
//   for real-time operation when timeout policies are configured.
//
// Real-Time Resource Management:
//   All resource allocation happens ONCE at OpenECMUSB time and persists
//   until CloseECMUSB. This avoids per-tick kernel object creation overhead.
//   - File handle created once, reused across all transfers
//   - WinUSB Interface 0 handle (hWinUsb) initialised once
//   - WinUSB Interface 1 handle (hWinUsbIf1) obtained once — Scenario A only
//   - Pipe addresses cached at discovery time, no per-call lookups
//   - Device path and foundViaHID flag cached at open time for recovery
//
//   The hold-last recovery strategy (in caller's mdlOutputs) ensures transient
//   failures do NOT trigger expensive reset operations inside the real-time loop.
//   Only ECMUSBRecover() at shutdown performs explicit resets.
//
// Pipe Policies for Real-Time Operation:
//   - PIPE_TRANSFER_TIMEOUT: 3000 ms
//     Prevents indefinite stalls on broken/disconnected devices.
//   - AUTO_FLUSH on OUT pipe
//     Sends zero-length packets (ZLP) when transfers are exact multiples of
//     MaxPacketSize. Important for bulk endpoints to ensure the device
//     recognizes frame boundaries on fixed-size protocol messages.
//   - ALLOW_PARTIAL_READS on IN pipe
//     Allows short transfers to complete immediately rather than waiting
//     for a full buffer. Critical for variable-length or delimited messages.
//   - IGNORE_SHORT_PACKETS: FALSE
//     Ensures app is notified of partial transfers, critical for protocol
//     boundary validation and frame synchronization.
//
// Endpoint Discovery & Fallback:
//   At open time, WinUsb_QueryPipe locates the first Bulk-OUT and Bulk-IN
//   endpoints on the data interface (Interface 1 for Scenario A, Interface 0
//   otherwise). Their addresses are cached for reuse. If discovery fails
//   (e.g., device firmware issues), compile-time defaults are used as a
//   fallback (ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT). This allows
//   graceful degradation when device firmware doesn't expose endpoints
//   in the expected order.
//
// Build requirements:
//   - Link: winusb.lib, setupapi.lib, cfgmgr32.lib, hid.lib
//   - SDK:  Windows SDK 8.1 or later
//   - CRT:  MSVCRT / UCRT
//
// Compilation (MSVC):
//   cl /nologo /EHsc /W3 /DNEXTWUSBLIB_EXPORTS /LD NEXTWUSBLib.cpp
//      /link winusb.lib setupapi.lib cfgmgr32.lib hid.lib
//      /DEF:NEXTWUSBLib.def /OUT:NEXTWUSBLib.dll
// ============================================================================

#define NEXTWUSBLIB_EXPORTS
#include "NEXTWUSBLib.h"
#include "NEXTWUSBLib_internal.h"

// ============================================================================
// ECM_DEVICE_CTX  —  central state block for the transport layer
// ============================================================================
// One global instance (g_Dev) is allocated at DLL load time and persists
// for the lifetime of the process. All public API functions (OpenECMUSB,
// CloseECMUSB, ECMUSBWrite, ECMUSBRead, ECMUSBRecover) operate exclusively
// through this structure — there is no per-call heap allocation in the
// real-time path.
//
// Lifetime contract:
//   Allocated : DLL load  (static initialiser below)
//   Populated : OpenECMUSB()
//   Consumed  : ECMUSBWrite() / ECMUSBRead()   (real-time path)
//   Drained   : CloseECMUSB() / ECMUSBRecover()
//
// Scenario A handle ownership diagram:
//
//   CreateFileW(composite USB root path)
//           │
//           ▼
//         hFile  ──────────────────────────────────────────────┐
//           │                                                   │
//   WinUsb_Initialize(hFile)                                   │
//           │                                                   │
//           ▼                                                   │
//         hWinUsb   (Interface 0 — HID, no bulk endpoints)     │
//           │                                                   │
//   WinUsb_GetAssociatedInterface(hWinUsb, 0, &hWinUsbIf1)     │
//           │  ↑ associated-index 0 = second physical iface    │
//           │    i.e. Interface 1 (vendor / bulk)               │
//           ▼                                                   │
//         hWinUsbIf1  (Interface 1 — vendor, bulk pipes here)  │
//           │                                                   │
//   ECM_QueryPipes(hWinUsbIf1)  ──► pipeOut / pipeIn           │
//   ECM_ApplyPipePolicies(hWinUsbIf1)                          │
//           │                                                   │
//   ECMUSBWrite / ECMUSBRead  use  ECM_DataHandle()            │
//     → returns hWinUsbIf1  when Scenario A active             │
//     → returns hWinUsb     when Scenario A NOT active ◄───────┘
//
// Teardown order (MUST be reverse of acquisition):
//   1. WinUsb_Free(hWinUsbIf1)  ← free child handle first
//   2. WinUsb_Free(hWinUsb)     ← then the parent handle
//   3. CloseHandle(hFile)        ← finally the OS handle
//
// Non-Scenario-A (standard WinUSB, SEARCH 1 / SEARCH 3):
//   hWinUsbIf1 = NULL, foundViaHID = FALSE.
//   ECM_DataHandle() returns hWinUsb; no Interface 1 acquisition needed.
//   Teardown: WinUsb_Free(hWinUsb) → CloseHandle(hFile).
//
// Field-by-field documentation:
//
//   hFile
//   ─────
//   Win32 file handle from CreateFileW on the composite USB root path.
//   Flags: GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
//          FILE_FLAG_OVERLAPPED (mandatory — WinUSB uses this for internal
//          timeout enforcement via IOCTLs even in "synchronous" pipe calls).
//   Must be closed with CloseHandle() only AFTER both WinUsb_Free() calls.
//   Value: INVALID_HANDLE_VALUE until OpenECMUSB succeeds.
//
//   hWinUsb  —  WinUSB Interface 0 handle
//   ───────────────────────────────────────
//   Obtained by WinUsb_Initialize(hFile, &hWinUsb).
//   Always refers to the DEFAULT (first) USB interface — Interface 0.
//
//   Standard device (Scenario A NOT active):
//     Interface 0 IS the data interface. All pipe I/O uses this handle.
//     ECM_DataHandle() returns this handle.
//
//   Composite HID device (Scenario A active):
//     Interface 0 is the HID interface — it carries INTERRUPT endpoints for
//     HID reports, NOT the bulk endpoints needed for EtherCAT frames.
//     This handle must NOT be used for pipe I/O; it is held open solely to
//     anchor hWinUsbIf1 (WinUSB requires the parent handle to remain open
//     for the lifetime of the associated interface handle).
//     ECM_DataHandle() returns hWinUsbIf1 instead.
//
//   Must be freed with WinUsb_Free(hWinUsb) AFTER WinUsb_Free(hWinUsbIf1).
//   Value: NULL until OpenECMUSB successfully calls WinUsb_Initialize.
//
//   hWinUsbIf1  —  WinUSB Interface 1 handle  [SCENARIO A ONLY]
//   ─────────────────────────────────────────────────────────────
//   Obtained by: WinUsb_GetAssociatedInterface(hWinUsb, 0, &hWinUsbIf1)
//
//   "Associated index 0" maps to the SECOND physical USB interface
//   (Interface 1), NOT Interface 0. The index is zero-based relative
//   to the interface AFTER the default:
//     Associated index 0  →  Interface 1  (vendor/bulk — used here)
//     Associated index 1  →  Interface 2  (if present)
//     ...and so on.
//
//   Interface 1 is the vendor-class interface where WinUSB.sys is loaded.
//   It carries the bulk endpoints required for EtherCAT frame transfer:
//     Bulk-OUT endpoint  →  host-to-device command frames
//     Bulk-IN  endpoint  →  device-to-host response frames
//
//   Prerequisite: WinUSB.sys MUST already be bound to Interface 1 before
//   this handle can be obtained. Use Zadig, a co-installer INF, or ensure
//   the firmware carries a Microsoft OS Descriptor that auto-loads WinUSB.
//
//   When Scenario A is NOT active (foundViaHID == FALSE), this field is
//   NULL for the entire session and ECM_DataHandle() never returns it.
//
//   Teardown: MUST be freed with WinUsb_Free(hWinUsbIf1) BEFORE freeing
//   hWinUsb. Freeing the parent handle first while hWinUsbIf1 is still
//   open causes undefined driver state and potential kernel resource leaks.
//   Value: NULL at init; set in OpenECMUSB step 5 (foundViaHID path only).
//
//   pipeOut  —  Bulk-OUT endpoint address (host → device)
//   ───────────────────────────────────────────────────────
//   Cached by ECM_QueryPipes() by scanning all endpoints on the data
//   interface and selecting the first where:
//     PipeType == UsbdPipeTypeBulk  AND  (PipeId & 0x80) == 0
//   Typical value: 0x01. Fallback: ECM_PIPE_OUT_DEFAULT.
//   Used directly in WinUsb_WritePipe() on every ECMUSBWrite() call.
//   Not re-queried per call — cached once, valid for the session lifetime.
//
//   pipeIn  —  Bulk-IN endpoint address (device → host)
//   ──────────────────────────────────────────────────────
//   Cached by ECM_QueryPipes() by scanning all endpoints on the data
//   interface and selecting the first where:
//     PipeType == UsbdPipeTypeBulk  AND  (PipeId & 0x80) != 0
//   Bit 7 set (0x80) indicates IN direction per USB spec.
//   Typical value: 0x81. Fallback: ECM_PIPE_IN_DEFAULT.
//   Used directly in WinUsb_ReadPipe() on every ECMUSBRead() call.
//   Not re-queried per call — cached once, valid for the session lifetime.
//
//   isOpen  —  device-ready gate
//   ──────────────────────────────
//   Set TRUE only after ALL of the following succeed in OpenECMUSB:
//     1. ECM_FindDevicePath       — device located, path and HID flag cached
//     2. CreateFileW              — OS handle acquired
//     3. WinUsb_Initialize        — Interface 0 handle acquired
//     4. GetAssociatedInterface   — Interface 1 acquired (Scenario A only)
//     5. ECM_QueryPipes           — pipeOut / pipeIn cached (or defaults used)
//     6. ECM_ApplyPipePolicies    — timeout / flush / partial-read configured
//   Guards all API entry points: Write, Read, Recover, Close each return
//   immediately (no-op or false) when isOpen is FALSE.
//   OpenECMUSB is a no-op if isOpen is already TRUE (prevents double-open).
//
//   foundViaHID  —  Scenario A activation flag
//   ─────────────────────────────────────────────
//   Set TRUE by ECM_FindDevicePath() when the device was discovered via the
//   HID class GUID (SEARCH 2) AND a WinUSB composite path was successfully
//   derived for the same physical device.
//
//   When TRUE, OpenECMUSB performs the extra Scenario A step (step 5:
//   WinUsb_GetAssociatedInterface) and ECM_DataHandle() returns hWinUsbIf1
//   for all subsequent pipe I/O.
//
//   When FALSE (standard WinUSB — SEARCH 1 or SEARCH 3):
//     hWinUsbIf1 is never set; ECM_DataHandle() returns hWinUsb.
//
//   Intentionally persisted across CloseECMUSB so ECMUSBRecover() knows
//   to re-acquire Interface 1 after reopening. Without this flag, Recover
//   would use Interface 0 for a device that originally required Interface 1,
//   silently breaking the bulk data path on the next real-time cycle.
//
//   devicePath  —  cached Win32 device path (multi-device identity anchor)
//   ──────────────────────────────────────────────────────────────────────
//   Stores the exact composite USB root path string returned by
//   ECM_FindDevicePath() and passed to CreateFileW. The path uniquely
//   identifies a single physical USB device instance by its port location,
//   so it remains stable as long as the device stays on the same port.
//   Path format: \\?\usb#vid_16c0&pid_05df#<serial_or_port>#{dee824ef-...}
//
//   *** CRITICAL FOR MULTI-DEVICE SYSTEMS ***
//   When multiple ECM-SK modules are connected simultaneously, the SetupAPI
//   enumeration index (0, 1, 2 ...) can shift if one device disconnects —
//   a re-enumeration would then return a DIFFERENT physical device at the
//   same index. By caching this exact path string BEFORE CreateFileW,
//   ECMUSBRecover() bypasses enumeration entirely and reopens the SAME
//   physical device regardless of how many other units are connected.
//   Populated: inside OpenECMUSB, immediately after ECM_FindDevicePath()
//              returns and BEFORE the CreateFileW call.
//   Cleared:   NEVER by CloseECMUSB (intentional — ECMUSBRecover reads it).
//              Only zeroed by the static initialiser at DLL load.
// ============================================================================

// ============================================================================
// Module-level state (one global instance, not thread-safe by design)
// ============================================================================
// Initialization values have specific meaning:
//   hFile        = INVALID_HANDLE_VALUE : No file handle yet (safe initial state)
//   hWinUsb      = NULL                 : No WinUSB Interface 0 handle yet
//   hWinUsbIf1   = NULL                 : No WinUSB Interface 1 handle yet
//                                         (NULL = Scenario A not active)
//   pipeOut      = 0                    : 0 is invalid endpoint address
//                                         (see ECM_PIPE_OUT_DEFAULT)
//   pipeIn       = 0                    : 0 is invalid endpoint address
//                                         (see ECM_PIPE_IN_DEFAULT)
//   isOpen       = FALSE                : Device not yet opened
//   foundViaHID  = FALSE                : Scenario A not active at init
//   devicePath[0]= 0                    : Empty path string
//
// Resources are allocated at OpenECMUSB time and persist until CloseECMUSB
// to avoid per-tick kernel allocation overhead that would introduce scheduling
// jitter in the real-time cycle. The devicePath and foundViaHID flag are
// cached at open time (BEFORE CreateFile) to support multi-device systems
// and correct Scenario A recovery (see struct documentation above).
static ECM_DEVICE_CTX g_Dev = {
    INVALID_HANDLE_VALUE,   // hFile
    NULL,                   // hWinUsb    (Interface 0 handle)
    NULL,                   // hWinUsbIf1 (Interface 1 handle — Scenario A only)
    0,                      // pipeOut
    0,                      // pipeIn
    FALSE,                  // isOpen
    FALSE,                  // foundViaHID
    { 0 }                   // devicePath
};

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL  ECM_FindDevicePath              (WCHAR *pathBuf, DWORD pathBufLen,
                                              BOOL  *pFoundViaHID);
static BOOL  ECM_FindDevicePathByVidPid_WinUSB (const WCHAR *vidPidStr,
                                                WCHAR *pathBuf,
                                                DWORD  pathBufLen);
static BOOL  ECM_FindDevicePathByVidPid_InClass(HDEVINFO    hDevInfo,
                                                const WCHAR *vidPidStr,
                                                WCHAR       *pathBuf,
                                                DWORD        pathBufLen,
                                                const WCHAR *className);
static BOOL  ECM_FindDevicePathByVidPid_AllUSB (const WCHAR *vidPidStr,
                                                WCHAR *pathBuf,
                                                DWORD  pathBufLen);
static BOOL  ECM_FindHIDDeviceGetCompositePath (const WCHAR *hidPath,
                                                WCHAR       *winusbPathBuf,
                                                DWORD        winusbPathBufLen);
static BOOL  ECM_QueryPipes                  (WINUSB_INTERFACE_HANDLE hIface);
static void  ECM_ApplyPipePolicies           (WINUSB_INTERFACE_HANDLE hIface);
static void  ECM_ResetDevice                 (void);

// ============================================================================
// ECM_DataHandle  —  returns the correct WinUSB handle for pipe I/O
// ============================================================================
// Single indirection point that keeps all call sites (ECMUSBWrite,
// ECMUSBRead, ECMUSBRecover) clean — no scattered if/else per function.
//
// When Scenario A is active (hWinUsbIf1 != NULL):
//   Returns hWinUsbIf1 — the Interface 1 (vendor/bulk) handle.
//   hWinUsb (Interface 0 / HID) must NOT be used for pipe I/O because
//   Interface 0 only carries HID interrupt endpoints, not the bulk
//   endpoints required for EtherCAT frame transfer.
//
// When Scenario A is NOT active (hWinUsbIf1 == NULL):
//   Returns hWinUsb — the sole interface handle on a standard WinUSB device.
// ============================================================================
static inline WINUSB_INTERFACE_HANDLE ECM_DataHandle(void)
{
    return (g_Dev.hWinUsbIf1 != NULL) ? g_Dev.hWinUsbIf1 : g_Dev.hWinUsb;
}

// ============================================================================
//  OpenECMUSB
//  Locates the ECM-SK device by VID/PID, opens an exclusive file handle,
//  initializes WinUSB, discovers bulk endpoints, and configures transfer
//  policies. Returns true on success, false on any failure.
//
// High-level algorithm (Scenario A aware):
//   1. Search for device path by VID/PID
//      - SEARCH 1 (WinUSB GUID)  → standard path, foundViaHID = FALSE
//      - SEARCH 2 (HID GUID)     → Scenario A pivot, foundViaHID = TRUE
//      - SEARCH 3 (all USB)      → catch-all,        foundViaHID = FALSE
//   2. Cache the exact device path AND foundViaHID flag BEFORE opening
//      (critical for multi-device recovery — see ECMUSBRecover)
//   3. CreateFileW on the composite USB root path
//   4. WinUsb_Initialize → Interface 0 handle (hWinUsb)
//   5. Scenario A only: WinUsb_GetAssociatedInterface(hWinUsb, 0, &hWinUsbIf1)
//      → Interface 1 handle (hWinUsbIf1). This is the vendor interface
//        carrying the bulk pipes. "Associated index 0" = Interface 1.
//   6. ECM_QueryPipes on ECM_DataHandle() — queries Interface 1 for
//      Scenario A, Interface 0 otherwise
//   7. ECM_ApplyPipePolicies on ECM_DataHandle() — configures timeouts,
//      auto-flush, partial reads, and short-packet notification
//
// Device Path Caching (CRITICAL FOR MULTI-DEVICE SYSTEMS):
//   The exact device path is cached BEFORE CreateFile so that ECMUSBRecover()
//   can reopen the same physical device without re-enumerating. This is
//   essential when multiple ECM-SK modules are connected simultaneously.
//   Re-enumeration without caching could return a different device at a
//   different index, silently breaking device identity.
//
// Resource Allocation Strategy:
//   All resources (file handle, WinUSB interface handles, pipe addresses)
//   are allocated once here and persist until CloseECMUSB. This one-time
//   allocation avoids per-tick kernel overhead that would introduce
//   real-time scheduling jitter.
// ============================================================================
bool __stdcall OpenECMUSB(void)
{
    if (g_Dev.isOpen)
    {
        ECM_Log("[ECM] OpenECMUSB: already open.\n");
        return true;
    }

    // ----- 1. Locate the device path ----------------------------------------
    // ECM_FindDevicePath tries three strategies in order (WinUSB GUID, HID
    // GUID + Scenario A pivot, all-USB fallback) and sets pFoundViaHID to
    // TRUE when the Scenario A dual-handle path must be used.
    WCHAR devicePath[MAX_PATH] = { 0 };
    BOOL  foundViaHID          = FALSE;

    if (!ECM_FindDevicePath(devicePath, MAX_PATH, &foundViaHID))
    {
        ECM_Log("[ECM] OpenECMUSB: ECM-SK device (VID_%04X&PID_%04X) not found.\n",
                ECM_USB_VID, ECM_USB_PID);
        return false;
    }
    ECM_Log("[ECM] Device path: %ls  (foundViaHID=%d)\n",
            devicePath, (int)foundViaHID);

    // ----- 2. Cache the path and HID-detection flag -------------------------
    // CRITICAL FOR MULTI-DEVICE SYSTEMS. Store the exact composite path so
    // that ECMUSBRecover() can reopen the same physical device without
    // re-enumerating. Re-enumeration might return a different unit at a
    // different enumeration index when multiple ECM-SK modules are connected
    // simultaneously, silently breaking device identity and desynchronizing
    // protocol communication.
    // foundViaHID is also cached here: ECMUSBRecover() must know whether to
    // re-acquire Interface 1 (Scenario A) after reopening, without repeating
    // the full device discovery sequence.
    wcsncpy_s(g_Dev.devicePath, MAX_PATH, devicePath, _TRUNCATE);
    g_Dev.foundViaHID = foundViaHID;

    // ----- 3. Open a file handle to the composite device root ---------------
    // For Scenario A the path here is the COMPOSITE (USB root) device path,
    // not the HID interface child path — this allows WinUsb_Initialize to
    // see all interfaces of the composite device.
    // FILE_SHARE_READ | FILE_SHARE_WRITE allows other processes to probe
    // device properties, reducing contention. Exclusive access is enforced
    // at the pipe level by WinUSB internally.
    // FILE_FLAG_OVERLAPPED is mandatory — WinUSB uses it for timeout
    // enforcement via IOCTLs even when pipe calls appear synchronous.
    g_Dev.hFile = CreateFileW(
        devicePath,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (g_Dev.hFile == INVALID_HANDLE_VALUE)
    {
        DWORD err = GetLastError();
        if (err == ERROR_ACCESS_DENIED)
            ECM_Log("[ECM] OpenECMUSB: access denied — device may be open by "
                    "another process or driver.\n");
        else
            ECM_Log("[ECM] OpenECMUSB: CreateFile failed: 0x%08X\n", err);
        return false;
    }

    // ----- 4. Initialise WinUSB — acquire Interface 0 handle ----------------
    // WinUsb_Initialize wraps the file handle and exposes a clean API for
    // pipe operations. It always returns a handle to Interface 0 (the default
    // interface). On a standard (non-composite) device this is the only
    // interface and carries the bulk pipes. On a composite HID device
    // (Scenario A) this is the HID interface; the bulk pipes live on
    // Interface 1 which is acquired in step 5 below.
    // On success g_Dev.hWinUsb is valid and must be freed by WinUsb_Free()
    // AFTER WinUsb_Free(hWinUsbIf1) and BEFORE CloseHandle(hFile).
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        DWORD err = GetLastError();
        ECM_Log("[ECM] OpenECMUSB: WinUsb_Initialize failed: 0x%08X\n", err);
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    // ----- 5. Scenario A: acquire Interface 1 (vendor / bulk interface) -----
    // Only entered when the device was discovered via the HID class GUID,
    // meaning Windows bound HIDClass.sys to Interface 0. Interface 1 is the
    // vendor-class interface where WinUSB.sys is loaded.
    //
    // WinUsb_GetAssociatedInterface(hWinUsb, AssocIfaceIndex, &hIf1):
    //   AssocIfaceIndex = 0  →  Interface 1  (first AFTER the default)
    //   AssocIfaceIndex = 1  →  Interface 2  (second AFTER the default)
    //   ...and so on.
    // "Associated index 0" does NOT mean Interface 0 — it means the first
    // interface associated after (i.e. beyond) the default Interface 0.
    //
    // If this call fails the device is either:
    //   (a) A simple single-interface device (no Interface 1 exists)
    //   (b) WinUSB.sys is not loaded on Interface 1 (driver not installed)
    //       → Use Zadig to install WinUSB on Interface 1, then retry.
    // In both cases we log a detailed diagnostic and fall back to using
    // Interface 0 (pipe discovery below may then find no bulk endpoints).
    if (foundViaHID)
    {
        ECM_Log("[ECM] OpenECMUSB: Scenario A — device found via HID. "
                "Claiming Interface 1 via WinUsb_GetAssociatedInterface...\n");

        if (!WinUsb_GetAssociatedInterface(g_Dev.hWinUsb, 0, &g_Dev.hWinUsbIf1))
        {
            DWORD err = GetLastError();
            ECM_Log("[ECM] OpenECMUSB: WinUsb_GetAssociatedInterface failed: 0x%08X\n"
                    "[ECM]   Possible causes:\n"
                    "[ECM]   (a) Device has only one interface (no Interface 1)\n"
                    "[ECM]   (b) WinUSB.sys not bound to Interface 1\n"
                    "[ECM]       Remediation:\n"
                    "[ECM]         1. Open Zadig (zadig.akeo.ie)\n"
                    "[ECM]         2. Options → List All Devices\n"
                    "[ECM]         3. Select device VID_%04X / PID_%04X\n"
                    "[ECM]         4. Ensure Interface 1 is selected\n"
                    "[ECM]         5. Set driver to WinUSB and click Install\n"
                    "[ECM]         6. Restart this application\n"
                    "[ECM]   Falling back to Interface 0 (bulk pipes may not exist).\n",
                    err, ECM_USB_VID, ECM_USB_PID);
            // hWinUsbIf1 remains NULL — ECM_DataHandle() will return hWinUsb
        }
        else
        {
            ECM_Log("[ECM] OpenECMUSB: Interface 1 handle acquired (hWinUsbIf1). "
                    "All pipe I/O will use Interface 1.\n");
        }
    }

    // ----- 6. Discover bulk pipe addresses ------------------------------------
    // Query the endpoint configuration on whichever interface carries the
    // bulk pipes. ECM_DataHandle() returns hWinUsbIf1 (Interface 1) when
    // Scenario A succeeded, or hWinUsb (Interface 0) in all other cases.
    // If discovery fails (e.g., device firmware issues), fall back to
    // compile-time defaults. This allows graceful degradation when device
    // firmware doesn't expose standard endpoint layouts or exhibits issues
    // during enumeration.
    WINUSB_INTERFACE_HANDLE hData = ECM_DataHandle();

    if (!ECM_QueryPipes(hData))
    {
        // Fall back to compile-time defaults if pipe discovery fails
        ECM_Log("[ECM] OpenECMUSB: Pipe discovery failed; using default endpoints "
                "0x%02X / 0x%02X.\n",
                ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT);
        g_Dev.pipeOut = ECM_PIPE_OUT_DEFAULT;
        g_Dev.pipeIn  = ECM_PIPE_IN_DEFAULT;
    }

    // ----- 7. Configure transfer policies ------------------------------------
    // All policies are configured once at open time and persist for all
    // transfers. This one-time configuration avoids per-call policy setup
    // overhead. Applied to the data handle (Interface 1 for Scenario A,
    // Interface 0 otherwise). Each policy and its rationale is documented
    // inside ECM_ApplyPipePolicies.
    ECM_ApplyPipePolicies(hData);

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] OpenECMUSB: OK (OUT=0x%02X, IN=0x%02X, timeout=%lu ms, "
            "Scenario A=%s).\n",
            g_Dev.pipeOut, g_Dev.pipeIn, ECM_USB_TIMEOUT_MS,
            (g_Dev.hWinUsbIf1 != NULL) ? "YES (Interface 1)" : "NO (Interface 0)");
    return true;
}

// ============================================================================
//  CloseECMUSB
//  Releases WinUSB resources and closes the device handle, resetting
//  internal state to allow subsequent OpenECMUSB calls.
//
//  Scenario A teardown order (IMPORTANT — reverse of acquisition):
//    1. WinUsb_Free(hWinUsbIf1)  — Interface 1 child handle freed first.
//       MUST precede freeing hWinUsb. Freeing the parent (hWinUsb) while
//       the child (hWinUsbIf1) is still open causes undefined driver state
//       and potential kernel resource leaks.
//    2. WinUsb_Free(hWinUsb)     — Interface 0 / parent handle freed second.
//       WinUsb_Free is required; leaking hWinUsb prevents the device from
//       being accessed by other applications after this DLL releases it.
//    3. CloseHandle(hFile)        — OS file handle closed last.
//       Must follow both WinUsb_Free calls.
//
//  NOTE: g_Dev.devicePath and g_Dev.foundViaHID are intentionally NOT
//  cleared here so that ECMUSBRecover() can reopen the same composite
//  device path and correctly restore the Scenario A dual-handle state
//  without re-enumerating.
//
//  Hold-Last Strategy Context:
//  The hold-last recovery strategy (repeating last valid command on failure)
//  is applied by the CALLER in mdlOutputs. This library does NOT implement
//  hold-last; it only signals failure to the caller. The caller (EC01M_SFunction)
//  is responsible for maintaining the last valid command buffer and resubmitting
//  it on the next cycle if ECMUSBWrite/Read fails. This prevents the real-time
//  loop from being stalled by explicit resets or recovery operations.
// ============================================================================
void __stdcall CloseECMUSB(void)
{
    if (!g_Dev.isOpen)
        return;

    // Free Interface 1 handle first (Scenario A — MUST precede hWinUsb free).
    // Leaking hWinUsbIf1 or freeing hWinUsb first causes undefined driver state.
    if (g_Dev.hWinUsbIf1 != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsbIf1);
        g_Dev.hWinUsbIf1 = NULL;
        ECM_Log("[ECM] CloseECMUSB: Interface 1 handle (Scenario A) freed.\n");
    }

    // Free the base WinUSB handle (Interface 0 or sole interface).
    // WinUsb_Free is required; leaking hWinUsb prevents the device from
    // being accessed by other applications.
    if (g_Dev.hWinUsb != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsb);
        g_Dev.hWinUsb = NULL;
    }

    // Close the underlying file handle. Must be called AFTER both
    // WinUsb_Free() calls — closing the file while WinUSB handles are
    // still live causes I/O cancellation races in the kernel driver.
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    // Clear pipe addresses and open state
    g_Dev.pipeOut = 0;
    g_Dev.pipeIn  = 0;
    g_Dev.isOpen  = FALSE;
    // g_Dev.devicePath and g_Dev.foundViaHID intentionally preserved —
    // ECMUSBRecover() reads both to reopen the same physical device on the
    // correct interface without re-enumerating.
    ECM_Log("[ECM] CloseECMUSB: device closed.\n");
}

// ============================================================================
//  ECMUSBWrite
//  Writes `dwLength` bytes from `data` to the ECM-SK bulk-OUT endpoint.
//
//  Buffer Size & Protocol Framing:
//  The caller is responsible for providing exactly the correct frame size
//  (dwLength must match the protocol's expected frame size, typically
//  12*42=504 or 16*32=512 bytes depending on packet variant). No frame ID
//  or length prefix is added — the data is sent raw to the endpoint.
//
//  Interface Handle:
//  Uses ECM_DataHandle() to select the correct WinUSB interface handle
//  automatically. For Scenario A (composite HID device) this is hWinUsbIf1
//  (Interface 1 — vendor/bulk); for standard WinUSB devices this is hWinUsb
//  (Interface 0). No conditional logic is required at the call site.
//
//  Transfer Model:
//  WinUsb_WritePipe blocks until complete or timeout (configured via
//  SetPipePolicy PIPE_TRANSFER_TIMEOUT during OpenECMUSB). This synchronous
//  model is suitable for real-time applications where predictable timing
//  is more important than overlapped I/O complexity.
//
//  Error Handling & Hold-Last Strategy:
//  Returns TRUE on success (full write), FALSE on timeout or partial write.
//  The caller applies a hold-last strategy on failure (do NOT reset the pipe
//  from within this function); ECMUSBRecover is called at controlled shutdown
//  only. Transient failures are handled by the caller in mdlOutputs by
//  repeating the last valid command on the next cycle.
// ============================================================================
bool __stdcall ECMUSBWrite(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBWrite: invalid state or parameters.\n");
        return false;
    }

    // Perform the bulk write via the correct interface handle (Interface 1
    // for Scenario A, Interface 0 otherwise). WinUsb_WritePipe internally
    // uses overlapped I/O with the timeout configured in OpenECMUSB, so
    // this call will not block indefinitely if the device is stalled or
    // disconnected.
    ULONG bytesWritten = 0;
    BOOL ok = WinUsb_WritePipe(
        ECM_DataHandle(),           // Interface 1 (Scenario A) or Interface 0
        g_Dev.pipeOut,
        data,
        (ULONG)dwLength,
        &bytesWritten,
        NULL);  // NULL = use internal overlapped I/O (timeout-bounded)

    // Enforce full write; any partial or timed-out write is an error
    // in the protocol layer. The hold-last strategy ensures the last
    // valid command is repeated by the caller, preventing loss of
    // synchronization. We do NOT reset pipes here — that's left for
    // ECMUSBRecover() at controlled shutdown time.
    if (!ok || bytesWritten != (ULONG)dwLength)
    {
        DWORD err = GetLastError();
        if (err == ERROR_SEM_TIMEOUT)
        {
            ECM_Log("[ECM] ECMUSBWrite: timed out after %lu ms — "
                    "device stalled or disconnected? (hold last).\n",
                    ECM_USB_TIMEOUT_MS);
        }
        else
        {
            ECM_Log("[ECM] ECMUSBWrite failed: Err=0x%08X "
                    "(wrote %lu / %lu bytes, hold last).\n",
                    err, bytesWritten, dwLength);
        }

        // Return false to signal caller to use hold-last strategy.
        // Do NOT reset pipes here — that would block the real-time loop.
        return false;
    }

    return true;
}

// ============================================================================
//  ECMUSBRead
//  Reads `dwLength` bytes into `data` from the ECM-SK bulk-IN endpoint.
//
//  Buffer Size & Protocol Framing:
//  The caller must provide a buffer sized exactly for the expected frame
//  (dwLength must match the protocol's frame size, typically 12*42=504 or
//  16*32=512 bytes). The raw frame data is placed directly into the buffer
//  with no length prefix or framing indicators.
//
//  Interface Handle:
//  Uses ECM_DataHandle() to select the correct WinUSB interface handle
//  automatically. For Scenario A (composite HID device) this is hWinUsbIf1
//  (Interface 1 — vendor/bulk); for standard WinUSB devices this is hWinUsb
//  (Interface 0). No conditional logic is required at the call site.
//
//  Transfer Model:
//  WinUsb_ReadPipe blocks until complete, the timeout expires (configured
//  via SetPipePolicy during OpenECMUSB), or a short packet is detected
//  (ALLOW_PARTIAL_READS configured, IGNORE_SHORT_PACKETS disabled).
//  This synchronous model is suitable for real-time applications where
//  timing predictability matters more than implementation complexity.
//
//  Full-Frame Enforcement:
//  Returns TRUE only on success (full read matching dwLength exactly).
//  Any short read or timeout is treated as a fatal synchronization error
//  and returns FALSE. The caller applies hold-last strategy to recover.
//  This strict enforcement ensures protocol frame boundaries are never
//  silently violated by stale or partial data leaking to the caller.
// ============================================================================
bool __stdcall ECMUSBRead(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBRead: invalid state or parameters.\n");
        return false;
    }

    // Zero-initialise the destination buffer so stale data never leaks
    // to the caller if the transfer fails or returns partial data. This
    // prevents information leakage from previous operations and ensures
    // clean state even on error paths.
    memset(data, 0, dwLength);

    // Perform the bulk read via the correct interface handle (Interface 1
    // for Scenario A, Interface 0 otherwise). WinUsb_ReadPipe internally
    // uses overlapped I/O with the timeout configured in OpenECMUSB, so
    // this call will not block indefinitely if the device is stalled or
    // disconnected.
    ULONG bytesRead = 0;
    BOOL ok = WinUsb_ReadPipe(
        ECM_DataHandle(),           // Interface 1 (Scenario A) or Interface 0
        g_Dev.pipeIn,
        data,
        (ULONG)dwLength,
        &bytesRead,
        NULL);  // NULL = use internal overlapped I/O (timeout-bounded)

    // Enforce full-frame reads. Returning success on a short read breaks
    // the fixed-size structure assumptions of the protocol and would allow
    // stale or partial data to leak through. Partial reads are treated as
    // fatal synchronization errors — the caller applies hold-last strategy.
    if (!ok || bytesRead != (ULONG)dwLength)
    {
        DWORD err = GetLastError();
        if (err == ERROR_SEM_TIMEOUT)
        {
            ECM_Log("[ECM] ECMUSBRead: timed out after %lu ms — "
                    "device stalled or disconnected? (hold last).\n",
                    ECM_USB_TIMEOUT_MS);
        }
        else
        {
            ECM_Log("[ECM] ECMUSBRead failed: Err=0x%08X "
                    "(read %lu / %lu bytes, hold last).\n",
                    err, bytesRead, dwLength);
        }

        // Return false to signal caller to use hold-last strategy.
        // Do NOT reset pipes here — that would block the real-time loop.
        return false;
    }

    return true;
}

// ============================================================================
//  ECMUSBRecover
//  Closes the WinUSB handles to flush any stalled I/O state, then reopens
//  them using the cached g_Dev.devicePath — bypassing re-enumeration to
//  guarantee the same physical device is targeted even when multiple ECM-SK
//  modules are connected simultaneously.
//
//  Scenario A recovery sequence (when foundViaHID == TRUE):
//    Teardown  (reverse acquisition order — CRITICAL):
//      1. WinUsb_Free(hWinUsbIf1)  — child handle freed before parent
//      2. WinUsb_Free(hWinUsb)     — parent handle freed second
//      3. CloseHandle(hFile)        — OS handle closed last
//    Re-open:
//      4. CreateFileW(cached devicePath)
//      5. WinUsb_Initialize        → new hWinUsb  (Interface 0)
//      6. WinUsb_GetAssociatedInterface(0) → new hWinUsbIf1 (Interface 1)
//      7. ECM_ApplyPipePolicies on ECM_DataHandle()
//
//  Standard recovery sequence (when foundViaHID == FALSE):
//    Teardown:
//      1. WinUsb_Free(hWinUsb)
//      2. CloseHandle(hFile)
//    Re-open:
//      3. CreateFileW(cached devicePath)
//      4. WinUsb_Initialize        → new hWinUsb
//      5. ECM_ApplyPipePolicies on hWinUsb
//
//  *** TIMING CRITICAL — NOT REAL-TIME SAFE ***
//  This function can take 10-100 milliseconds (order of tens of ms) due to:
//  - WinUsb_Free: ~1-5 ms (driver resource cleanup)
//  - CloseHandle: ~1-5 ms (kernel object cleanup)
//  - CreateFileW: ~5-20 ms (driver stack re-engagement, even with cached path)
//  - WinUsb_Initialize: ~5-50 ms (driver re-initialization)
//  MUST NOT be called from the real-time loop (mdlOutputs). Should only be
//  called at controlled shutdown time in mdlTerminate. Calling from within
//  the real-time cycle would cause real-time overruns and thread starvation.
//  The handle will be closed again by CloseECMUSB before mdlTerminate returns.
//
//  CRITICAL FOR MULTI-DEVICE SYSTEMS:
//  Uses the cached device path to reopen WITHOUT re-enumerating. This ensures
//  the same physical device is targeted. Re-enumeration would query all devices
//  in order and could return a different device if enumeration order changes.
// ============================================================================
void __stdcall ECMUSBRecover(void)
{
    if (!g_Dev.isOpen)
        return;

    ECM_Log("[ECM] ECMUSBRecover: closing WinUSB handles to flush stalled I/O...\n");
    ECM_Log("[ECM]   WARNING: This operation can take 10-100ms (NOT real-time safe).\n");

    // ---- Teardown: reverse acquisition order --------------------------------
    // Interface 1 child handle MUST be freed before the Interface 0 parent.
    // Freeing hWinUsb first while hWinUsbIf1 is still open causes undefined
    // driver state and potential kernel resource leaks.
    if (g_Dev.hWinUsbIf1 != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsbIf1);
        g_Dev.hWinUsbIf1 = NULL;
        ECM_Log("[ECM] ECMUSBRecover: Interface 1 handle freed.\n");
    }

    // Free the WinUSB Interface 0 / sole-interface handle to close any
    // stalled pipes and release driver resources.
    if (g_Dev.hWinUsb != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsb);
        g_Dev.hWinUsb = NULL;
    }

    // Close the OS file handle after all WinUSB handles have been freed.
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    // g_Dev.devicePath and g_Dev.foundViaHID are preserved — intentionally
    // not cleared so we can reopen the correct device on the correct
    // interface below without re-enumerating.

    // ---- Reopen using cached composite path (no re-enumeration) ------------
    // Best-effort at shutdown; failure is non-fatal. This reopen WITHOUT
    // re-enumeration ensures the same physical device is targeted when
    // multiple ECM-SK modules are connected simultaneously. Using the cached
    // path avoids silent device identity loss that could occur if enumeration
    // order changes between the original open and this recovery call.
    g_Dev.hFile = CreateFileW(
        g_Dev.devicePath,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (g_Dev.hFile == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM] ECMUSBRecover: reopen failed: 0x%08X "
                "(device disconnected?)\n", GetLastError());
        return;
    }

    // Reinitialise WinUSB Interface 0 for the reopened file handle
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        ECM_Log("[ECM] ECMUSBRecover: WinUsb_Initialize failed: 0x%08X\n",
                GetLastError());
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return;
    }

    // ---- Scenario A: re-acquire Interface 1 if we originally used it -------
    // g_Dev.foundViaHID was cached at OpenECMUSB time and tells us whether
    // the Scenario A dual-handle path must be restored. Without this flag
    // we would incorrectly use Interface 0 for a device that requires
    // Interface 1 for bulk pipe I/O, silently breaking the data path.
    if (g_Dev.foundViaHID)
    {
        ECM_Log("[ECM] ECMUSBRecover: Scenario A — re-acquiring Interface 1...\n");

        if (!WinUsb_GetAssociatedInterface(g_Dev.hWinUsb, 0, &g_Dev.hWinUsbIf1))
        {
            ECM_Log("[ECM] ECMUSBRecover: WinUsb_GetAssociatedInterface failed: "
                    "0x%08X. Falling back to Interface 0.\n", GetLastError());
            // hWinUsbIf1 stays NULL; ECM_DataHandle() returns hWinUsb.
            // Pipe I/O on Interface 0 may fail if it has no bulk endpoints.
        }
        else
        {
            ECM_Log("[ECM] ECMUSBRecover: Interface 1 re-acquired successfully.\n");
        }
    }

    // ---- Reconfigure pipe policies on the data handle ----------------------
    // Reapply all pipe policies configured in OpenECMUSB. They are lost when
    // the WinUSB handles are freed and must be restored before the next I/O.
    // Applied to ECM_DataHandle() — Interface 1 for Scenario A, Interface 0
    // otherwise. Each policy and its rationale is documented inside
    // ECM_ApplyPipePolicies.
    ECM_ApplyPipePolicies(ECM_DataHandle());

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] ECMUSBRecover: handle reopened successfully using cached path "
            "(Scenario A=%s, multi-device identity preserved).\n",
            (g_Dev.hWinUsbIf1 != NULL) ? "YES (Interface 1)" : "NO (Interface 0)");
}

// ============================================================================
// ============================================================================
//  INTERNAL HELPERS
// ============================================================================
// ============================================================================

// ============================================================================
// ECM_ApplyPipePolicies
// ============================================================================
// PURPOSE:
//   Applies all WinUSB pipe policies required for real-time operation to
//   the given interface handle. Extracted as a shared helper to eliminate
//   duplication between OpenECMUSB (step 7) and ECMUSBRecover (after
//   handle re-acquisition). Policies are lost when WinUSB handles are freed
//   and must be re-applied after every handle re-acquisition.
//
// PARAMETER:
//   [in] hIface - The WinUSB interface handle to configure. Must be the DATA
//                 handle — i.e. ECM_DataHandle() — so that policies target
//                 Interface 1 (Scenario A) or Interface 0 (standard path).
//                 Passing the wrong handle (e.g. hWinUsb when Scenario A is
//                 active) would configure the HID interface instead of the
//                 bulk data interface.
//
// POLICIES APPLIED (all non-fatal on failure — driver defaults used):
//
//   PIPE_TRANSFER_TIMEOUT  : ECM_USB_TIMEOUT_MS (3000 ms) on OUT and IN
//     Prevents indefinite stalls on broken/disconnected devices.
//     Applied to both pipes independently.
//
//   AUTO_FLUSH (OUT pipe)  : TRUE
//     Sends a zero-length packet (ZLP) when a transfer is an exact multiple
//     of the endpoint's MaxPacketSize. This ensures the device recognises
//     frame boundaries on fixed-size protocol messages (504 or 512 bytes)
//     and prevents data from being buffered indefinitely in the USB stack.
//
//   ALLOW_PARTIAL_READS (IN pipe) : TRUE
//     Allows short transfers to complete immediately rather than waiting
//     for the full buffer to fill. Critical for protocols with variable-
//     length or delimited messages that don't always fill the buffer.
//     Without this, short reads would time out waiting for more data.
//
//   IGNORE_SHORT_PACKETS (IN pipe) : FALSE
//     Ensures the application is notified when a short packet is received.
//     Critical for protocol frame boundary validation and ensures fixed-size
//     frame assumptions are enforced at the USB level. If a frame is shorter
//     than expected the caller is notified immediately rather than silently
//     accepting partial data.
// ============================================================================
static void ECM_ApplyPipePolicies(WINUSB_INTERFACE_HANDLE hIface)
{
    // Set timeout for both pipes. If SetPipePolicy fails we continue with
    // the driver default timeout (non-fatal) because WinUSB provides internal
    // timeout mechanisms even if this explicit policy call fails.
    ULONG timeout = ECM_USB_TIMEOUT_MS;
    if (!WinUsb_SetPipePolicy(hIface, g_Dev.pipeOut,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
    {
        ECM_Log("[ECM] SetPipePolicy(OUT, TIMEOUT) failed: 0x%08X "
                "(non-fatal — using driver default).\n", GetLastError());
    }

    if (!WinUsb_SetPipePolicy(hIface, g_Dev.pipeIn,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
    {
        ECM_Log("[ECM] SetPipePolicy(IN, TIMEOUT) failed: 0x%08X "
                "(non-fatal — using driver default).\n", GetLastError());
    }

    // Enable auto-flush on OUT pipe to send ZLP when transfer is a multiple
    // of the max-packet-size. This ensures the device recognizes frame
    // boundaries on fixed-size protocol messages and prevents data from
    // being buffered indefinitely. Non-fatal if unsupported by device.
    UCHAR autoFlush = TRUE;
    if (!WinUsb_SetPipePolicy(hIface, g_Dev.pipeOut,
                              AUTO_FLUSH, sizeof(UCHAR), &autoFlush))
    {
        ECM_Log("[ECM] SetPipePolicy(OUT, AUTO_FLUSH) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }

    // Allow partial reads on IN pipe so short transfers complete immediately
    // rather than waiting for a full buffer. Critical for protocols with
    // variable-length or delimited messages that don't fill the entire buffer.
    // Without this, short reads would timeout waiting for more data.
    UCHAR allowPartial = TRUE;
    if (!WinUsb_SetPipePolicy(hIface, g_Dev.pipeIn,
                              ALLOW_PARTIAL_READS, sizeof(UCHAR), &allowPartial))
    {
        ECM_Log("[ECM] SetPipePolicy(IN, ALLOW_PARTIAL_READS) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }

    // Disable IGNORE_SHORT_PACKETS (set to FALSE) so the app is notified when
    // a short packet is received. This is critical for protocol frame boundary
    // validation and ensures fixed-size protocol frame assumptions are validated
    // at the USB level. If a frame is shorter than expected, we know immediately.
    UCHAR ignoreShort = FALSE;
    if (!WinUsb_SetPipePolicy(hIface, g_Dev.pipeIn,
                              IGNORE_SHORT_PACKETS, sizeof(UCHAR), &ignoreShort))
    {
        ECM_Log("[ECM] SetPipePolicy(IN, IGNORE_SHORT_PACKETS) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }
}

// ============================================================================
// ECM_FindHIDDeviceGetCompositePath
// ============================================================================
// PURPOSE:
//   Given confirmation that the device was found via the HID class GUID,
//   locates the parent COMPOSITE USB device path that WinUSB can open
//   directly (\\?\usb#vid_16c0&pid_05df#...#{dee824ef-...}).
//
//   This is the key Scenario A bridge function. When the device is found via
//   the HID GUID it means the OS has enumerated the device as a composite
//   device where:
//     Interface 0 — HID class   (HIDClass.sys, path = \\?\hid#vid_...#mi_00#...)
//     Interface 1 — Vendor class (WinUSB.sys,   path = \\?\usb#vid_...#{dee...})
//
//   Strategy:
//     Delegate to ECM_FindDevicePathByVidPid_WinUSB() which enumerates
//     GUID_DEVINTERFACE_USB_DEVICE for the target VID/PID and returns the
//     composite USB root path. This path is suitable for CreateFileW followed
//     by WinUsb_Initialize + WinUsb_GetAssociatedInterface(0) to reach
//     Interface 1 where the bulk endpoints live.
//
//   Why not use the hidPath directly?
//     The HID path (\\?\hid#...#mi_00#...#{4d1e55b2-...}) is the CHILD
//     interface path for Interface 0. Opening it with CreateFileW gives
//     access only to the HID interface — WinUsb_Initialize on it cannot
//     reach Interface 1. The USB root (composite parent) path must be used.
//
// PARAMETERS:
//   [in]  hidPath          - HID device path from SetupAPI (for logging only;
//                            the actual composite path is re-derived via
//                            ECM_FindDevicePathByVidPid_WinUSB)
//   [out] winusbPathBuf    - Buffer to receive composite USB root path
//   [in]  winusbPathBufLen - Size of winusbPathBuf in characters (MAX_PATH min)
//
// RETURN VALUE:
//   TRUE  - Composite WinUSB path found and written to winusbPathBuf
//   FALSE - No matching USB composite device found; WinUSB.sys may not be
//           installed on Interface 1 — see Zadig remediation steps in log
// ============================================================================
static BOOL ECM_FindHIDDeviceGetCompositePath(
    const WCHAR *hidPath,
    WCHAR       *winusbPathBuf,
    DWORD        winusbPathBufLen)
{
    ECM_Log("[ECM] ECM_FindHIDDeviceGetCompositePath: deriving composite "
            "WinUSB path from HID path: %ls\n", hidPath);

    // Build the VID/PID search token (same format as main search)
    WCHAR vidPidStr[64];
    swprintf_s(vidPidStr, 64, L"VID_%04X&PID_%04X", ECM_USB_VID, ECM_USB_PID);

    // Delegate to the WinUSB enumerator — it searches GUID_DEVINTERFACE_USB_DEVICE
    // for the target VID/PID and returns the composite USB root path that
    // CreateFileW + WinUsb_Initialize can open. This is the path for the
    // composite parent device, from which WinUsb_GetAssociatedInterface(0)
    // can then reach Interface 1 (the vendor/bulk interface).
    BOOL ok = ECM_FindDevicePathByVidPid_WinUSB(vidPidStr,
                                                winusbPathBuf,
                                                winusbPathBufLen);
    if (ok)
    {
        ECM_Log("[ECM] ECM_FindHIDDeviceGetCompositePath: composite path "
                "found: %ls\n", winusbPathBuf);
    }
    else
    {
        ECM_Log("[ECM] ECM_FindHIDDeviceGetCompositePath: FAILED — "
                "no USB composite device found for VID_%04X&PID_%04X.\n"
                "[ECM] This means WinUSB.sys is NOT installed on Interface 1.\n"
                "[ECM] Remediation:\n"
                "[ECM]   1. Open Zadig (zadig.akeo.ie)\n"
                "[ECM]   2. Options → List All Devices\n"
                "[ECM]   3. Select device VID_%04X / PID_%04X\n"
                "[ECM]   4. Ensure Interface 1 is selected in the dropdown\n"
                "[ECM]   5. Set driver to WinUSB and click Install Driver\n"
                "[ECM]   6. Restart this application\n",
                ECM_USB_VID, ECM_USB_PID,
                ECM_USB_VID, ECM_USB_PID);
    }
    return ok;
}

// ============================================================================
// ECM_FindDevicePath
// ============================================================================
// PURPOSE:
//   Main device discovery function. Locates the ECM-SK USB device by Vendor
//   ID and Product ID using three searches in priority order, and sets
//   *pFoundViaHID to indicate whether Scenario A (dual-handle composite)
//   must be activated in OpenECMUSB.
//
//   SEARCH 1 — WinUSB devices  (primary target)
//   ─────────────────────────────────────────────
//   Enumerates all interfaces registered under GUID_DEVINTERFACE_USB_DEVICE
//   using DIGCF_PRESENT | DIGCF_DEVICEINTERFACE. This is the correct approach
//   for devices bound to WinUSB.sys because WinUSB registers each device
//   instance under this interface GUID when the INF installs it, or when the
//   firmware carries a Microsoft OS Descriptor. Returns the composite USB root
//   path directly — no Scenario A pivot needed.
//   *pFoundViaHID = FALSE.
//   Device paths start with \\?\usb#vid_...
//
//   SEARCH 2 — HID class devices + Scenario A pivot
//   ──────────────────────────────────────────────────
//   Enumerates all HID class interfaces using the GUID returned by
//   HidD_GetHidGuid(). Covers devices that enumerate as standard HID (VID
//   16C0 / PID 05DF is a V-USB generic HID VID/PID). When a match is found
//   via the HID GUID, it triggers the Scenario A pivot:
//     ECM_FindHIDDeviceGetCompositePath() re-derives the WinUSB composite
//     root path for the same physical device so OpenECMUSB can open it and
//     acquire Interface 1 via WinUsb_GetAssociatedInterface.
//   *pFoundViaHID = TRUE → OpenECMUSB will call GetAssociatedInterface.
//   Device paths returned start with \\?\usb#vid_... (composite root).
//
//   SEARCH 3 — All USB devices  (catch-all fallback)
//   ──────────────────────────────────────────────────
//   Enumerates every present USB device (DIGCF_PRESENT, enumerator "USB")
//   regardless of class or interface GUID. Reaches devices with no class
//   driver or no registered device interface. If the hardware ID matches it
//   attempts to obtain a GUID_DEVINTERFACE_USB_DEVICE interface path; if no
//   path is obtainable it logs a diagnostic and returns FALSE.
//   *pFoundViaHID = FALSE.
//   Device paths start with \\?\usb#...
//
// PARAMETERS:
//   [out] pathBuf      - Buffer to receive the device path (MAX_PATH min)
//   [in]  pathBufLen   - Size of pathBuf in characters
//   [out] pFoundViaHID - Set TRUE when Scenario A must be activated:
//                        device was found via HID GUID and the composite
//                        WinUSB path was successfully derived.
//                        Set FALSE for SEARCH 1 and SEARCH 3 results.
//
// RETURN VALUE:
//   TRUE  - Device found; pathBuf populated and *pFoundViaHID set correctly
//   FALSE - Device not found via any strategy; check log output for diagnostics
//
// TROUBLESHOOTING (check log output):
//   All three searches return NOT FOUND
//     → Device not connected, or actual VID/PID differs from ECM_USB_VID/PID
//     → Check Device Manager and compare "AllUSB[n]" log lines
//   SEARCH 2 match found but Scenario A pivot fails
//     → WinUSB.sys not on Interface 1; use Zadig (see log for steps)
//   Match found in SEARCH 3 but no interface path
//     → Device has no driver; use Zadig to install WinUSB on Interface 1
//   SEARCH 1 succeeds but CreateFile later fails with ACCESS_DENIED
//     → Another process owns the WinUSB handle; verify no other instances running
//
// REAL-TIME SAFETY:
//   Uses SetupAPI — NOT real-time safe. Call ONLY during initialisation
//   (mdlStart), never from the real-time control loop (mdlOutputs).
// ============================================================================
static BOOL ECM_FindDevicePath(WCHAR *pathBuf, DWORD pathBufLen,
                               BOOL  *pFoundViaHID)
{
    *pFoundViaHID = FALSE;

    ECM_Log("[ECM] ECM_FindDevicePath: Searching for VID_%04X&PID_%04X\n",
            ECM_USB_VID, ECM_USB_PID);

    WCHAR vidPidStr[64];
    swprintf_s(vidPidStr, 64, L"VID_%04X&PID_%04X", ECM_USB_VID, ECM_USB_PID);

    // ===== SEARCH 1: WinUSB devices (primary — device may be WinUSB-bound) ==
    ECM_Log("[ECM] SEARCH 1: WinUSB Devices (GUID_DEVINTERFACE_USB_DEVICE)\n");
    if (ECM_FindDevicePathByVidPid_WinUSB(vidPidStr, pathBuf, pathBufLen))
    {
        ECM_Log("[ECM] Found via SEARCH 1 (WinUSB — no Scenario A needed).\n");
        *pFoundViaHID = FALSE;
        return TRUE;
    }

    // ===== SEARCH 2: HID class devices + Scenario A pivot ===================
    // Device was found via HID GUID — triggers Scenario A. The HID path is
    // a child interface path (Interface 0); ECM_FindHIDDeviceGetCompositePath
    // re-derives the composite USB root path so Interface 1 can be reached.
    ECM_Log("[ECM] SEARCH 2: HID Class Devices (Scenario A pivot)\n");
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);
    HDEVINFO hHidInfo = SetupDiGetClassDevsW(
        &hidGuid, NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hHidInfo != INVALID_HANDLE_VALUE)
    {
        WCHAR hidPath[MAX_PATH] = { 0 };

        if (ECM_FindDevicePathByVidPid_InClass(
                hHidInfo, vidPidStr, hidPath, MAX_PATH, L"HID"))
        {
            ECM_Log("[ECM] Found via SEARCH 2 (HID). "
                    "Activating Scenario A — deriving WinUSB composite path.\n");

            // Pivot: get the composite USB root path from the HID child path.
            // The hidPath is the HID interface child path (Interface 0);
            // we need the USB root (composite parent) path for WinUsb_Initialize
            // so that WinUsb_GetAssociatedInterface can then reach Interface 1.
            if (ECM_FindHIDDeviceGetCompositePath(hidPath, pathBuf, pathBufLen))
            {
                *pFoundViaHID = TRUE;
                ECM_Log("[ECM] Scenario A pivot SUCCEEDED. "
                        "WinUSB composite root path ready for OpenECMUSB.\n");
                return TRUE;
            }
            else
            {
                // Composite path not available. WinUSB.sys is not on Interface 1.
                // Detailed Zadig remediation steps already logged inside
                // ECM_FindHIDDeviceGetCompositePath. Return FALSE to caller.
                ECM_Log("[ECM] Scenario A FAILED: WinUSB.sys not bound to "
                        "Interface 1. See above for remediation steps.\n");
                return FALSE;
            }
        }
    }
    else
    {
        ECM_Log("[ECM]   HID enumeration failed: 0x%08X\n", GetLastError());
    }

    // ===== SEARCH 3: All USB devices — catch-all fallback ===================
    ECM_Log("[ECM] SEARCH 3: All USB Devices (catch-all fallback)\n");
    if (ECM_FindDevicePathByVidPid_AllUSB(vidPidStr, pathBuf, pathBufLen))
    {
        ECM_Log("[ECM] Found via SEARCH 3 (All USB fallback — no Scenario A).\n");
        *pFoundViaHID = FALSE;
        return TRUE;
    }

    ECM_Log("[ECM] DEVICE NOT FOUND via any search strategy.\n");
    return FALSE;
}

// ============================================================================
// ECM_FindDevicePathByVidPid_WinUSB
// ============================================================================
// PURPOSE:
//   Searches for a WinUSB device matching the target VID/PID by enumerating
//   all device interfaces registered under GUID_DEVINTERFACE_USB_DEVICE
//   (the standard interface GUID assigned to devices using the WinUSB driver).
//
//   Unlike HID enumeration (which uses a separate class GUID), WinUSB devices
//   are registered under GUID_DEVINTERFACE_USB_DEVICE when the INF installs
//   WinUSB.sys as the function driver, or when the firmware carries a
//   Microsoft OS Descriptor that causes Windows 8+ to auto-load WinUSB.
//   This is also the GUID used to find the composite USB root path during the
//   Scenario A pivot in ECM_FindHIDDeviceGetCompositePath.
//
// PARAMETERS:
//   [in]  const WCHAR *vidPidStr   - VID/PID search string, e.g. "VID_16C0&PID_05DF"
//   [out] WCHAR *pathBuf           - Buffer to receive device path if found
//   [in]  DWORD pathBufLen         - Size of pathBuf in characters (MAX_PATH minimum)
//
// RETURN VALUE:
//   TRUE  - Device found as WinUSB; path written to pathBuf
//   FALSE - Not found, or enumeration error
//
// DEVICE PATH FORMAT:
//   \\?\usb#vid_16c0&pid_05df#0x0001#{dee824ef-729b-4a0e-9c14-b7117d33a817}
//
// CRITICAL NOTES:
//   - Uses DIGCF_PRESENT | DIGCF_DEVICEINTERFACE (correct for interface GUIDs)
//   - Hardware ID is matched before attempting to retrieve the device path
//   - The device info set is always destroyed before returning
// ============================================================================
static BOOL ECM_FindDevicePathByVidPid_WinUSB(
    const WCHAR *vidPidStr,
    WCHAR       *pathBuf,
    DWORD        pathBufLen)
{
    ECM_Log("[ECM]   Searching WinUSB devices (GUID_DEVINTERFACE_USB_DEVICE)...\n");

    // Enumerate all present devices that expose a USB device interface.
    // DIGCF_DEVICEINTERFACE is mandatory for interface-GUID queries;
    // DIGCF_PRESENT skips disconnected devices.
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        &GUID_DEVINTERFACE_USB_DEVICE,
        NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hDevInfo == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM]   WinUSB enumeration failed: 0x%08X\n", GetLastError());
        return FALSE;
    }

    // Prepare case-insensitive search string once, outside the loop
    WCHAR searchUpper[64];
    wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
    _wcsupr_s(searchUpper, 64);

    BOOL found = FALSE;
    SP_DEVICE_INTERFACE_DATA ifData = { 0 };
    ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    // Walk every interface registered under GUID_DEVINTERFACE_USB_DEVICE
    for (DWORD idx = 0;
         SetupDiEnumDeviceInterfaces(hDevInfo, NULL,
                                     &GUID_DEVINTERFACE_USB_DEVICE,
                                     idx, &ifData);
         idx++)
    {
        // ---- Step 1: get buffer size for interface detail, then allocate ----
        // Two-call pattern required by SetupDiGetDeviceInterfaceDetailW:
        // first call with NULL buffer returns the required size;
        // second call with allocated buffer fills in the detail including
        // DevicePath and populates devData for the hardware ID query.
        DWORD requiredDetailSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0,
                                         &requiredDetailSize, NULL);
        if (requiredDetailSize == 0)
            continue;

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredDetailSize);
        if (!pDetail)
        {
            ECM_Log("[ECM]   WinUSB: memory allocation failed\n");
            break;
        }
        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        SP_DEVINFO_DATA devData = { 0 };
        devData.cbSize = sizeof(SP_DEVINFO_DATA);

        BOOL detailOk = SetupDiGetDeviceInterfaceDetailW(
            hDevInfo, &ifData,
            pDetail, requiredDetailSize,
            NULL, &devData);   // devData populated here for hardware ID lookup

        if (!detailOk)
        {
            ECM_Log("[ECM]   WinUSB[%lu]: GetDeviceInterfaceDetailW failed: 0x%08X\n",
                    idx, GetLastError());
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        // ---- Step 2: read the hardware ID from the registry and match VID/PID
        WCHAR hwId[512] = { 0 };
        if (!SetupDiGetDeviceRegistryPropertyW(
                hDevInfo, &devData, SPDRP_HARDWAREID,
                NULL, (PBYTE)hwId, sizeof(hwId), NULL))
        {
            ECM_Log("[ECM]   WinUSB[%lu]: no hardware ID, skipping\n", idx);
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        ECM_Log("[ECM]   WinUSB[%lu]: %ls\n", idx, hwId);

        WCHAR hwIdUpper[512];
        wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
        _wcsupr_s(hwIdUpper, 512);

        if (wcsstr(hwIdUpper, searchUpper) == NULL)
        {
            ECM_Log("[ECM]           -> No match\n");
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        // ---- Step 3: VID/PID matched — copy the composite device path -------
        ECM_Log("[ECM]           -> MATCH! Path: %ls\n", pDetail->DevicePath);

        size_t pathLen = wcslen(pDetail->DevicePath) + 1;
        if (pathLen <= (size_t)pathBufLen)
        {
            wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
            found = TRUE;
            ECM_Log("[ECM]           SUCCESS - WinUSB device path copied\n");
        }
        else
        {
            ECM_Log("[ECM]           ERROR: path too long (%zu chars)\n", pathLen);
        }

        HeapFree(GetProcessHeap(), 0, pDetail);

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    ECM_Log("[ECM]   WinUSB search result: %s\n", found ? "FOUND" : "NOT FOUND");
    return found;
}

// ============================================================================
// ECM_FindDevicePathByVidPid_InClass
// ============================================================================
// PURPOSE:
//   Searches for a USB device in a specific device class (HID in the current
//   call graph) by matching Vendor ID and Product ID in the hardware ID
//   registry property.
//
//   In SEARCH 2 this function is called with the HID device info set. On
//   a match it returns the HID child interface path (\\?\hid#vid_...#mi_00#...)
//   which the caller (ECM_FindDevicePath) then passes to
//   ECM_FindHIDDeviceGetCompositePath to derive the composite USB root path
//   needed for Scenario A.
//
// PARAMETERS:
//   [in]  HDEVINFO hDevInfo        - Device info set (ownership transferred;
//                                    this function always calls
//                                    SetupDiDestroyDeviceInfoList on it)
//   [in]  const WCHAR *vidPidStr   - VID/PID search string, e.g. "VID_16C0&PID_05DF"
//   [out] WCHAR *pathBuf           - Buffer to receive the device path
//   [in]  DWORD pathBufLen         - Size of pathBuf in characters (MAX_PATH minimum)
//   [in]  const WCHAR *className   - Label used in log output (e.g. "HID")
//
// RETURN VALUE:
//   TRUE  - Device found; path written to pathBuf
//   FALSE - Not found in this class, or error occurred
//
// BEHAVIOR:
//   1. Iterates all SP_DEVINFO_DATA entries in the set
//   2. Reads SPDRP_HARDWAREID from the registry for each entry
//   3. Case-insensitive substring match against vidPidStr
//   4. On match: calls SetupDiEnumDeviceInterfaces with the class-appropriate
//      GUID (GUID_DEVINTERFACE_HID for HID, GUID_DEVINTERFACE_USB_DEVICE as
//      fallback) to obtain an interface path
//   5. Allocates a detail buffer, retrieves DevicePath, copies to pathBuf
//
// CRITICAL NOTES:
//   - hDevInfo is always destroyed before this function returns
//   - Caller must supply a DIGCF_DEVICEINTERFACE-capable info set
//   - For HID devices the returned path is the child interface path, NOT the
//     composite USB root path. ECM_FindHIDDeviceGetCompositePath must be used
//     to convert it to a WinUSB-openable composite root path.
// ============================================================================
static BOOL ECM_FindDevicePathByVidPid_InClass(
    HDEVINFO    hDevInfo,
    const WCHAR *vidPidStr,
    WCHAR       *pathBuf,
    DWORD        pathBufLen,
    const WCHAR *className)
{
    ECM_Log("[ECM]   Searching in %ls class...\n", className);

    SP_DEVINFO_DATA devData = { 0 };
    devData.cbSize = sizeof(SP_DEVINFO_DATA);

    // Build the uppercase search token once, outside the enumeration loop
    WCHAR searchUpper[64];
    wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
    _wcsupr_s(searchUpper, 64);

    BOOL found       = FALSE;
    int  deviceIndex = 0;

    for (DWORD idx = 0; SetupDiEnumDeviceInfo(hDevInfo, idx, &devData); idx++)
    {
        WCHAR hwId[512] = { 0 };
        if (!SetupDiGetDeviceRegistryPropertyW(
                hDevInfo, &devData, SPDRP_HARDWAREID,
                NULL, (PBYTE)hwId, sizeof(hwId), NULL))
            continue;

        ECM_Log("[ECM]     Device[%lu]: %ls\n", idx, hwId);
        deviceIndex++;

        // Case-insensitive VID/PID match
        WCHAR hwIdUpper[512];
        wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
        _wcsupr_s(hwIdUpper, 512);

        if (wcsstr(hwIdUpper, searchUpper) == NULL)
        {
            ECM_Log("[ECM]           -> No match\n");
            continue;
        }

        ECM_Log("[ECM]           -> MATCH! Getting device path...\n");

        // Determine the primary interface GUID for this class.
        // For HID, the primary GUID is GUID_DEVINTERFACE_HID; fallback to
        // GUID_DEVINTERFACE_USB_DEVICE if the primary fails. This handles
        // composite devices where the HID interface may register under both.
        GUID *primaryGuid  = (className[0] == L'H')
            ? (GUID *)&GUID_DEVINTERFACE_HID
            : (GUID *)&GUID_DEVINTERFACE_USB_DEVICE;
        GUID *fallbackGuid = (className[0] == L'H')
            ? (GUID *)&GUID_DEVINTERFACE_USB_DEVICE
            : (GUID *)&GUID_DEVINTERFACE_HID;

        SP_DEVICE_INTERFACE_DATA ifData = { 0 };
        ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData, primaryGuid, 0, &ifData))
        {
            ECM_Log("[ECM]           SetupDiEnumDeviceInterfaces (primary) failed: "
                    "0x%08X; trying fallback GUID\n", GetLastError());
            if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData, fallbackGuid, 0, &ifData))
            {
                ECM_Log("[ECM]           Fallback GUID also failed: 0x%08X\n",
                        GetLastError());
                continue;
            }
        }

        // Determine buffer size required for interface detail (two-call pattern)
        DWORD requiredDetailSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &requiredDetailSize, NULL);
        if (requiredDetailSize == 0)
        {
            ECM_Log("[ECM]           Failed to get interface detail size\n");
            continue;
        }

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredDetailSize);
        if (!pDetail)
        {
            ECM_Log("[ECM]           Memory allocation failed\n");
            break;
        }
        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                             pDetail, requiredDetailSize,
                                             NULL, NULL))
        {
            ECM_Log("[ECM]           Found path: %ls\n", pDetail->DevicePath);

            size_t pathLen = wcslen(pDetail->DevicePath) + 1;
            if (pathLen <= (size_t)pathBufLen)
            {
                wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
                found = TRUE;
                ECM_Log("[ECM]           SUCCESS - Device path copied\n");
            }
            else
            {
                ECM_Log("[ECM]           ERROR: path too long (%zu chars)\n", pathLen);
            }
        }
        else
        {
            ECM_Log("[ECM]           GetDeviceInterfaceDetailW failed: 0x%08X\n",
                    GetLastError());
        }

        HeapFree(GetProcessHeap(), 0, pDetail);

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    ECM_Log("[ECM]   %ls class: %d devices examined, result: %s\n",
            className, deviceIndex, found ? "FOUND" : "NOT FOUND");
    return found;
}

// ============================================================================
// ECM_FindDevicePathByVidPid_AllUSB
// ============================================================================
// PURPOSE:
//   Last-resort search across ALL present USB devices regardless of class or
//   driver. Enumerates with DIGCF_PRESENT (no DIGCF_DEVICEINTERFACE) to reach
//   devices that are not registered with any particular interface GUID, then
//   attempts to obtain a usable device path via GUID_DEVINTERFACE_USB_DEVICE.
//
//   This search covers devices that have no class driver, are listed as
//   "Unknown Device", or whose INF did not register a device interface.
//
// PARAMETERS:
//   [in]  const WCHAR *vidPidStr   - VID/PID search string, e.g. "VID_16C0&PID_05DF"
//   [out] WCHAR *pathBuf           - Buffer to receive the device path
//   [in]  DWORD pathBufLen         - Size of pathBuf in characters (MAX_PATH minimum)
//
// RETURN VALUE:
//   TRUE  - Device found and a usable path was obtained
//   FALSE - Device not found, or found but no interface path available
//
// NOTES:
//   Because many devices visible via DIGCF_PRESENT do not register a device
//   interface, this search may locate the hardware ID but still fail to obtain
//   a CreateFile-able path. In that case the function logs a diagnostic message
//   and returns FALSE rather than returning a path that cannot be opened.
//   This is the SEARCH 3 fallback — no Scenario A pivot is performed here.
// ============================================================================
static BOOL ECM_FindDevicePathByVidPid_AllUSB(
    const WCHAR *vidPidStr,
    WCHAR       *pathBuf,
    DWORD        pathBufLen)
{
    ECM_Log("[ECM]   Searching all USB devices (DIGCF_PRESENT, enumerator=USB)...\n");

    // DIGCF_PRESENT only — no DIGCF_DEVICEINTERFACE.
    // This reaches devices with no registered interface GUID.
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        NULL, L"USB", NULL,
        DIGCF_PRESENT);

    if (hDevInfo == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM]   All-USB enumeration failed: 0x%08X\n", GetLastError());
        return FALSE;
    }

    WCHAR searchUpper[64];
    wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
    _wcsupr_s(searchUpper, 64);

    BOOL found     = FALSE;
    int  totalSeen = 0;

    SP_DEVINFO_DATA devData = { 0 };
    devData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD idx = 0; SetupDiEnumDeviceInfo(hDevInfo, idx, &devData); idx++)
    {
        WCHAR hwId[512] = { 0 };
        if (!SetupDiGetDeviceRegistryPropertyW(
                hDevInfo, &devData, SPDRP_HARDWAREID,
                NULL, (PBYTE)hwId, sizeof(hwId), NULL))
            continue;

        ECM_Log("[ECM]   AllUSB[%lu]: %ls\n", idx, hwId);
        totalSeen++;

        WCHAR hwIdUpper[512];
        wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
        _wcsupr_s(hwIdUpper, 512);

        if (wcsstr(hwIdUpper, searchUpper) == NULL)
            continue;

        ECM_Log("[ECM]           -> MATCH! Attempting to retrieve interface path...\n");

        // Attempt to get a device interface path via GUID_DEVINTERFACE_USB_DEVICE.
        // This may fail if the device has no registered interface (e.g., no INF),
        // in which case we report what we found but cannot return a path.
        SP_DEVICE_INTERFACE_DATA ifData = { 0 };
        ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData,
                                         &GUID_DEVINTERFACE_USB_DEVICE, 0, &ifData))
        {
            ECM_Log("[ECM]           SetupDiEnumDeviceInterfaces failed: 0x%08X\n"
                    "[ECM]           Device present but has no registered USB interface path.\n"
                    "[ECM]           Install WinUSB on Interface 1 via Zadig to proceed.\n",
                    GetLastError());
            // Hardware ID matched but no path obtainable — keep searching
            // in case a second enumeration entry for the same hardware has
            // a path (composite devices can have multiple entries).
            continue;
        }

        // Two-call pattern to get interface detail size, then fill buffer
        DWORD requiredDetailSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &requiredDetailSize, NULL);
        if (requiredDetailSize == 0)
        {
            ECM_Log("[ECM]           Failed to get interface detail size\n");
            continue;
        }

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredDetailSize);
        if (!pDetail)
        {
            ECM_Log("[ECM]           Memory allocation failed\n");
            break;
        }
        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                             pDetail, requiredDetailSize,
                                             NULL, NULL))
        {
            ECM_Log("[ECM]           Found path: %ls\n", pDetail->DevicePath);

            size_t pathLen = wcslen(pDetail->DevicePath) + 1;
            if (pathLen <= (size_t)pathBufLen)
            {
                wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
                found = TRUE;
                ECM_Log("[ECM]           SUCCESS - All-USB device path copied\n");
            }
            else
            {
                ECM_Log("[ECM]           ERROR: path too long (%zu chars)\n", pathLen);
            }
        }
        else
        {
            ECM_Log("[ECM]           GetDeviceInterfaceDetailW failed: 0x%08X\n",
                    GetLastError());
        }

        HeapFree(GetProcessHeap(), 0, pDetail);

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    ECM_Log("[ECM]   All-USB search: %d devices examined, result: %s\n",
            totalSeen, found ? "FOUND" : "NOT FOUND");
    return found;
}

// ============================================================================
//  ECM_QueryPipes
//  Queries all endpoints on the given interface handle and locates the first
//  Bulk-OUT and Bulk-IN pipes, storing their addresses in g_Dev.pipeOut and
//  g_Dev.pipeIn.
//
//  Now takes an explicit interface handle parameter (hIface) instead of
//  always using g_Dev.hWinUsb. This is essential for Scenario A — when the
//  device was found via HID, the bulk endpoints live on Interface 1
//  (hWinUsbIf1), NOT on Interface 0 (hWinUsb / HID interface). Querying
//  Interface 0 in Scenario A would find only HID interrupt endpoints and
//  return no bulk pipes.
//
// Algorithm:
//   1. WinUsb_QueryInterfaceSettings returns descriptor info (endpoint count)
//   2. For each endpoint, WinUsb_QueryPipe returns its type and address
//   3. First Bulk-OUT (address & 0x80 == 0) becomes pipeOut
//   4. First Bulk-IN  (address & 0x80 != 0) becomes pipeIn
//   5. Return TRUE if both pipes were found, FALSE otherwise
//
// Notes:
//   - Bulk-OUT pipe addresses are typically 0x01, 0x02, etc.
//   - Bulk-IN  pipe addresses are typically 0x81, 0x82, etc. (bit 7 = 1)
//   - The high bit (0x80) indicates direction per USB spec: 0 = OUT, 1 = IN
//   - Caller passes ECM_DataHandle() to ensure the correct interface is queried
//
// Return value:
//   TRUE  — both Bulk-OUT and Bulk-IN pipes were found and cached
//   FALSE — discovery failed (caller will apply compile-time defaults)
// ============================================================================
static BOOL ECM_QueryPipes(WINUSB_INTERFACE_HANDLE hIface)
{
    // Query the active interface descriptor to get the endpoint count.
    // AlternateInterfaceNumber 0 = the default (active) alternate setting.
    USB_INTERFACE_DESCRIPTOR ifDesc = { 0 };
    if (!WinUsb_QueryInterfaceSettings(hIface, 0, &ifDesc))
    {
        ECM_Log("[ECM] ECM_QueryPipes: WinUsb_QueryInterfaceSettings failed: "
                "0x%08X\n", GetLastError());
        return FALSE;
    }

    ECM_Log("[ECM] ECM_QueryPipes: Interface has %d endpoints.\n",
            (int)ifDesc.bNumEndpoints);

    BOOL foundOut = FALSE, foundIn = FALSE;

    // Iterate through all endpoints in the interface
    for (UCHAR i = 0; i < ifDesc.bNumEndpoints; i++)
    {
        WINUSB_PIPE_INFORMATION pipeInfo = { 0 };
        if (!WinUsb_QueryPipe(hIface, 0, i, &pipeInfo))
        {
            ECM_Log("[ECM] ECM_QueryPipes: WinUsb_QueryPipe[%d] failed: 0x%08X\n",
                    (int)i, GetLastError());
            continue;
        }

        ECM_Log("[ECM]   Pipe[%d]: type=%d (0=control,1=iso,2=bulk,3=int) "
                "addr=0x%02X maxPkt=%u\n",
                (int)i,
                (int)pipeInfo.PipeType,
                (unsigned)pipeInfo.PipeId,
                (unsigned)pipeInfo.MaximumPacketSize);

        // Only process bulk endpoints — ignore control, isochronous, interrupt
        if (pipeInfo.PipeType == UsbdPipeTypeBulk)
        {
            // Bit 7 of PipeId determines direction per USB spec:
            //   0 = OUT  (host → device)  e.g. 0x01, 0x02
            //   1 = IN   (device → host)  e.g. 0x81, 0x82
            if ((pipeInfo.PipeId & 0x80) == 0 && !foundOut)
            {
                g_Dev.pipeOut = pipeInfo.PipeId;
                foundOut = TRUE;
                ECM_Log("[ECM]   -> Assigned as Bulk-OUT pipe (host→device).\n");
            }
            else if ((pipeInfo.PipeId & 0x80) != 0 && !foundIn)
            {
                g_Dev.pipeIn = pipeInfo.PipeId;
                foundIn = TRUE;
                ECM_Log("[ECM]   -> Assigned as Bulk-IN pipe (device→host).\n");
            }
        }
    }

    // Return success only if both pipes were found. If only one direction
    // was found the device firmware may be non-standard or the wrong
    // interface was queried — caller will fall back to compile-time defaults.
    if (!foundOut || !foundIn)
    {
        ECM_Log("[ECM] ECM_QueryPipes: Incomplete — found OUT=%d IN=%d\n",
                foundOut, foundIn);
        return FALSE;
    }

    return TRUE;
}