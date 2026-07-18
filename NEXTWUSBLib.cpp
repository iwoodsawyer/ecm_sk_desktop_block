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
// How WinUSB reaches Interface 1 on a composite HID device
// ──────────────────────────────────────────────────────────
// Windows models composite USB devices using usbccgp.sys as the composite
// parent driver. Each interface is exposed as its own child device node with
// a distinct hardware ID containing "&mi_NN" (where NN is the interface
// number). When WinUSB.sys is bound to Interface 1 — via a co-installer INF,
// a Zadig installation, or a Microsoft OS 2.0 Descriptor in the firmware —
// Windows registers that interface node under GUID_DEVINTERFACE_USB_DEVICE
// with a path of the form:
//
//   \\?\usb#vid_16c0&pid_05df&mi_01#...#{dee824ef-729b-4a0e-9c14-b7117d33a817}
//
// Note the "&mi_01" component — this is the Interface 1 child device path,
// NOT the composite root. Opening this path with CreateFileW and calling
// WinUsb_Initialize yields a handle DIRECTLY to Interface 1. The handle
// already points at the vendor/bulk interface; WinUsb_GetAssociatedInterface
// is NOT needed and is NOT used.
//
// SEARCH 1 (GUID_DEVINTERFACE_USB_DEVICE) finds this "&mi_01" path via its
// hardware ID "VID_16C0&PID_05DF&MI_01", which contains the VID/PID match
// token. This is the correct and complete path for WinUSB communication.
//
// This DLL does NOT write an INF or install any driver. Prerequisite:
//   WinUSB.sys must already be bound to Interface 1. Options:
//     (a) Firmware Microsoft OS 2.0 Descriptor — Windows 8+ auto-loads WinUSB
//         on the vendor interface without any INF or user action.
//     (b) Zadig (zadig.akeo.ie) — select the device, choose Interface 1,
//         install WinUSB driver. One-time operation per machine.
//     (c) A vendor co-installer INF previously run at device setup.
//
// Protocol Framing & Buffer Sizes:
//   WinUSB bulk transfers do NOT require Report ID framing (unlike HID).
//   The caller passes a raw buffer of arbitrary size directly to the endpoint.
//   Actual protocol frame sizes depend on the packet variant:
//   - 12-byte variant: typically 12*42 = 504 bytes per transfer
//   - 16-byte variant: typically 16*32 = 512 bytes per transfer
//   These sizes should match the device firmware's expected frame format.
//
// Device Discovery:
//   Two strategies are employed in sequence:
//   1. GUID-based search: SetupAPI enumerates interfaces registered under
//      GUID_DEVINTERFACE_USB_DEVICE with DIGCF_PRESENT|DIGCF_DEVICEINTERFACE.
//      On a correctly configured device this finds the "&mi_01" WinUSB
//      interface path directly. This is the primary and preferred path.
//   2. All-USB fallback: If GUID search fails, all present USB devices are
//      enumerated via DIGCF_PRESENT|DIGCF_DEVICEINTERFACE with the USB
//      enumerator. Hardware ID strings are scanned for "VID_16C0&PID_05DF".
//      This catches devices whose interface GUID registration may be
//      non-standard but whose WinUSB interface is accessible.
//
//   *** CRITICAL FOR MULTIPLE ECM-SK UNITS ***
//   The device path is cached at open time so ECMUSBRecover() can reopen the
//   same PHYSICAL device without re-enumerating. When multiple ECM-SK modules
//   are connected simultaneously, re-enumeration returns devices in order
//   (index 0, 1, 2, ...). If one device disconnects, remaining devices may
//   shift indices. Re-enumerating without caching the path could silently
//   reconnect to a DIFFERENT physical device, breaking device identity and
//   protocol synchronization. Path caching prevents this disaster.
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
//   - WinUSB interface handle initialised once, reused for all operations
//   - Pipe addresses cached at discovery time, no per-call lookups
//   - Device path cached at open time for recovery without re-enumeration
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
//   endpoints. Their addresses are cached for reuse. If discovery fails
//   (e.g., device firmware issues), compile-time defaults are used as a
//   fallback (ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT). This allows
//   graceful degradation when device firmware doesn't expose endpoints
//   in the expected order.
//
// Build requirements:
//   - Link: winusb.lib, setupapi.lib, cfgmgr32.lib
//   - SDK:  Windows SDK 8.1 or later
//   - CRT:  MSVCRT / UCRT
//
// Compilation (MSVC):
//   cl /nologo /EHsc /W3 /DNEXTWUSBLIB_EXPORTS /LD NEXTWUSBLib.cpp
//      /link winusb.lib setupapi.lib cfgmgr32.lib
//      /DEF:NEXTWUSBLib.def /OUT:NEXTWUSBLib.dll
// ============================================================================

#define NEXTWUSBLIB_EXPORTS
#include "NEXTWUSBLib.h"
#include "NEXTWUSBLib_internal.h"

// ============================================================================
// ECM_DEVICE_CTX  —  central state block for the transport layer
// ============================================================================
// One global instance (g_Dev) is allocated at DLL load time and persists
// for the lifetime of the process. All public API functions operate
// exclusively through this structure — there is no per-call heap allocation
// in the real-time path.
//
// Lifetime contract:
//   Allocated : DLL load  (static initialiser below)
//   Populated : OpenECMUSB()
//   Consumed  : ECMUSBWrite() / ECMUSBRead()   (real-time path)
//   Drained   : CloseECMUSB() / ECMUSBRecover()
//
// Handle ownership and teardown order:
//
//   CreateFileW("\\?\usb#vid_16c0&pid_05df&mi_01#...#{dee824ef-...}")
//           │
//           ▼
//         hFile
//           │
//   WinUsb_Initialize(hFile)
//           │
//           ▼
//         hWinUsb  ←── points DIRECTLY at Interface 1 (vendor/bulk)
//           │           because the path already encodes "&mi_01"
//           │
//   ECM_QueryPipes(hWinUsb)     ──► pipeOut / pipeIn
//   ECM_ApplyPipePolicies(hWinUsb)
//           │
//   ECMUSBWrite / ECMUSBRead  use  hWinUsb directly
//
// Teardown order:
//   1. WinUsb_Free(hWinUsb)
//   2. CloseHandle(hFile)
//
// Field documentation:
//
//   hFile
//   ─────
//   Win32 file handle from CreateFileW on the WinUSB Interface 1 path.
//   Flags: GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
//          FILE_FLAG_OVERLAPPED (mandatory — WinUSB uses this for internal
//          timeout enforcement via IOCTLs even in "synchronous" pipe calls).
//   Must be closed with CloseHandle() only AFTER WinUsb_Free(hWinUsb).
//   Value: INVALID_HANDLE_VALUE until OpenECMUSB succeeds.
//
//   hWinUsb  —  WinUSB interface handle (Interface 1)
//   ──────────────────────────────────────────────────
//   Obtained by WinUsb_Initialize(hFile, &hWinUsb).
//   Because hFile is opened on the "&mi_01" device path, hWinUsb refers
//   directly to Interface 1 (the vendor/bulk interface). No associated
//   interface lookup is required.
//   Carries all bulk endpoints:
//     pipeOut — Bulk-OUT (host → device), EtherCAT command frames
//     pipeIn  — Bulk-IN  (device → host), EtherCAT response frames
//   Must be freed with WinUsb_Free() BEFORE CloseHandle(hFile).
//   Value: NULL until OpenECMUSB successfully calls WinUsb_Initialize.
//
//   pipeOut  —  Bulk-OUT endpoint address (host → device)
//   ───────────────────────────────────────────────────────
//   Cached by ECM_QueryPipes() selecting the first endpoint where:
//     PipeType == UsbdPipeTypeBulk  AND  (PipeId & 0x80) == 0
//   Typical value: 0x01. Fallback: ECM_PIPE_OUT_DEFAULT.
//   Used directly in WinUsb_WritePipe() on every ECMUSBWrite() call.
//   Cached once at open time; valid for the full session lifetime.
//
//   pipeIn  —  Bulk-IN endpoint address (device → host)
//   ──────────────────────────────────────────────────────
//   Cached by ECM_QueryPipes() selecting the first endpoint where:
//     PipeType == UsbdPipeTypeBulk  AND  (PipeId & 0x80) != 0
//   Bit 7 set (0x80) indicates IN direction per USB spec.
//   Typical value: 0x81. Fallback: ECM_PIPE_IN_DEFAULT.
//   Used directly in WinUsb_ReadPipe() on every ECMUSBRead() call.
//   Cached once at open time; valid for the full session lifetime.
//
//   isOpen  —  device-ready gate
//   ──────────────────────────────
//   Set TRUE only after ALL of the following succeed in OpenECMUSB:
//     1. ECM_FindDevicePath     — device located, path cached
//     2. CreateFileW            — OS handle acquired
//     3. WinUsb_Initialize      — WinUSB handle acquired
//     4. ECM_QueryPipes         — pipeOut / pipeIn cached (or defaults used)
//     5. ECM_ApplyPipePolicies  — timeout / flush / partial-read configured
//   Set FALSE immediately at the START of ECMUSBRecover(), and only
//   restored to TRUE at the end of a fully successful reopen. This prevents
//   ECMUSBWrite/Read from passing NULL handles to WinUSB APIs if recovery
//   partially fails (e.g. device disconnected), which would cause an
//   Access Violation in the host process.
//   Guards all API entry points: Write, Read, Recover, Close each return
//   immediately (no-op or false) when isOpen is FALSE.
//
//   devicePath  —  cached Win32 device path (multi-device identity anchor)
//   ──────────────────────────────────────────────────────────────────────
//   Stores the exact WinUSB Interface 1 path returned by ECM_FindDevicePath()
//   and passed to CreateFileW. The path uniquely identifies the single
//   physical USB interface instance by its port location, so it remains
//   stable as long as the device stays on the same physical port.
//
//   Path format example:
//     \\?\usb#vid_16c0&pid_05df&mi_01#6&1a2b3c4d&0&0001#{dee824ef-...}
//
//   *** CRITICAL FOR MULTI-DEVICE SYSTEMS ***
//   When multiple ECM-SK modules are connected simultaneously, the SetupAPI
//   enumeration index (0, 1, 2 ...) can shift if one device disconnects —
//   a re-enumeration would then return a DIFFERENT physical device at the
//   same index. By caching this exact path string BEFORE CreateFileW,
//   ECMUSBRecover() bypasses enumeration entirely and reopens the SAME
//   physical device regardless of how many other units are connected.
//   Populated: inside OpenECMUSB, immediately after ECM_FindDevicePath()
//              and BEFORE the CreateFileW call.
//   Cleared:   NEVER by CloseECMUSB (intentional — ECMUSBRecover reads it).
//              Only zeroed by the static initialiser at DLL load.
// ============================================================================

// ============================================================================
// Module-level state (one global instance, not thread-safe by design)
// ============================================================================
// Initialization values:
//   hFile         = INVALID_HANDLE_VALUE : No file handle yet
//   hWinUsb       = NULL                 : No WinUSB handle yet
//   pipeOut       = 0                    : Invalid endpoint (see ECM_PIPE_OUT_DEFAULT)
//   pipeIn        = 0                    : Invalid endpoint (see ECM_PIPE_IN_DEFAULT)
//   isOpen        = FALSE                : Device not yet opened
//   devicePath[0] = 0                    : Empty path string
//
// Resources are allocated at OpenECMUSB time and persist until CloseECMUSB
// to avoid per-tick kernel allocation overhead that would introduce scheduling
// jitter in the real-time cycle. devicePath is cached BEFORE CreateFile to
// support multi-device recovery (see struct documentation above).
static ECM_DEVICE_CTX g_Dev = {
    INVALID_HANDLE_VALUE,   // hFile
    NULL,                   // hWinUsb
    0,                      // pipeOut
    0,                      // pipeIn
    FALSE,                  // isOpen
    { 0 }                   // devicePath
};

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL  ECM_FindDevicePath                (WCHAR *pathBuf,
                                                DWORD  pathBufLen);
static BOOL  ECM_FindDevicePathByVidPid_WinUSB (const WCHAR *vidPidStr,
                                                WCHAR       *pathBuf,
                                                DWORD        pathBufLen);
static BOOL  ECM_FindDevicePathByVidPid_AllUSB (const WCHAR *vidPidStr,
                                                WCHAR       *pathBuf,
                                                DWORD        pathBufLen);
static BOOL  ECM_MatchHardwareId               (HDEVINFO         hDevInfo,
                                                SP_DEVINFO_DATA *pDevData,
                                                const WCHAR     *searchUpper);
static BOOL  ECM_QueryPipes                    (void);
static void  ECM_ApplyPipePolicies             (void);

// ============================================================================
//  OpenECMUSB
//  Locates the ECM-SK device by VID/PID, opens a file handle to the WinUSB
//  Interface 1 device node, initializes WinUSB, discovers bulk endpoints,
//  and configures transfer policies. Returns true on success, false on any
//  failure.
//
// High-level algorithm:
//   1. Search for device path (SEARCH 1: WinUSB GUID, SEARCH 2: all-USB)
//   2. Cache the exact device path BEFORE opening (critical for multi-device)
//   3. CreateFileW on the Interface 1 device path
//   4. WinUsb_Initialize → hWinUsb (points directly at Interface 1)
//   5. ECM_QueryPipes — locate Bulk-OUT and Bulk-IN endpoint addresses
//   6. ECM_ApplyPipePolicies — configure timeouts and protocol behaviour
//
// Why no GetAssociatedInterface:
//   The path from SEARCH 1 already encodes "&mi_01" — it is the Interface 1
//   child device node, not the composite root. WinUsb_Initialize on this
//   path returns hWinUsb pointing directly at Interface 1. No secondary
//   handle acquisition step is needed or correct.
//
// Device Path Caching (CRITICAL FOR MULTI-DEVICE SYSTEMS):
//   Cached BEFORE CreateFile so ECMUSBRecover() can reopen the same physical
//   device without re-enumerating. Re-enumeration with multiple units
//   connected could return a different unit at a different index, silently
//   breaking device identity.
//
// Resource Allocation Strategy:
//   All resources (file handle, WinUSB handle, pipe addresses) are allocated
//   once here and persist until CloseECMUSB. This one-time allocation avoids
//   per-tick kernel overhead that would introduce real-time scheduling jitter.
// ============================================================================
bool __stdcall OpenECMUSB(void)
{
    if (g_Dev.isOpen)
    {
        ECM_Log("[ECM] OpenECMUSB: already open.\n");
        return true;
    }

    // ----- 1. Locate the WinUSB Interface 1 device path ---------------------
    // ECM_FindDevicePath tries SEARCH 1 (GUID_DEVINTERFACE_USB_DEVICE) then
    // SEARCH 2 (all-USB fallback with DIGCF_DEVICEINTERFACE). Both searches
    // target the "&mi_01" Interface 1 node where WinUSB.sys is loaded.
    WCHAR devicePath[MAX_PATH] = { 0 };

    if (!ECM_FindDevicePath(devicePath, MAX_PATH))
    {
        ECM_Log("[ECM] OpenECMUSB: ECM-SK device (VID_%04X&PID_%04X) not found.\n",
                ECM_USB_VID, ECM_USB_PID);
        ECM_Log("[ECM]   Ensure WinUSB.sys is installed on Interface 1:\n");
        ECM_Log("[ECM]     1. Open Zadig (zadig.akeo.ie)\n");
        ECM_Log("[ECM]     2. Options -> List All Devices\n");
        ECM_Log("[ECM]     3. Select device VID_%04X / PID_%04X, Interface 1\n",
                ECM_USB_VID, ECM_USB_PID);
        ECM_Log("[ECM]     4. Set driver to WinUSB and click Install\n");
        return false;
    }
    ECM_Log("[ECM] Device path: %ls\n", devicePath);

    // ----- 2. Cache the path before opening ---------------------------------
    // CRITICAL FOR MULTI-DEVICE SYSTEMS. Store the exact path so that
    // ECMUSBRecover() can reopen the same physical device without
    // re-enumerating. Re-enumeration might return a different unit at a
    // different enumeration index when multiple ECM-SK modules are connected
    // simultaneously, silently breaking device identity and desynchronizing
    // protocol communication.
    wcsncpy_s(g_Dev.devicePath, MAX_PATH, devicePath, _TRUNCATE);

    // ----- 3. Open a file handle to the Interface 1 device node -------------
    // The path encodes "&mi_01" — this is the WinUSB Interface 1 child node,
    // not the composite root. WinUsb_Initialize on this handle gives direct
    // access to the vendor/bulk interface.
    // FILE_SHARE_READ | FILE_SHARE_WRITE: allows other processes to probe
    //   device properties while we hold the WinUSB pipe-level lock.
    // FILE_FLAG_OVERLAPPED: mandatory — WinUSB uses overlapped IOCTLs
    //   internally for timeout enforcement even in synchronous pipe calls.
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

    // ----- 4. Initialise WinUSB ---------------------------------------------
    // WinUsb_Initialize wraps the file handle. Because hFile was opened on
    // the "&mi_01" Interface 1 path, hWinUsb refers directly to the vendor
    // interface carrying the bulk endpoints. No GetAssociatedInterface call
    // is needed — the path already selects the correct interface.
    // hWinUsb must be freed with WinUsb_Free() BEFORE CloseHandle(hFile).
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        DWORD err = GetLastError();
        ECM_Log("[ECM] OpenECMUSB: WinUsb_Initialize failed: 0x%08X\n", err);
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    // ----- 5. Discover bulk pipe addresses ----------------------------------
    // Query the Interface 1 endpoint configuration to locate Bulk-OUT and
    // Bulk-IN pipes. If discovery fails (e.g., device firmware issues),
    // fall back to compile-time defaults. This allows graceful degradation
    // when device firmware doesn't expose standard endpoint layouts.
    if (!ECM_QueryPipes())
    {
        ECM_Log("[ECM] OpenECMUSB: Pipe discovery failed; using default endpoints "
                "0x%02X / 0x%02X.\n",
                ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT);
        g_Dev.pipeOut = ECM_PIPE_OUT_DEFAULT;
        g_Dev.pipeIn  = ECM_PIPE_IN_DEFAULT;
    }

    // ----- 6. Configure transfer policies -----------------------------------
    // Configured once at open time; persist for all transfers. Avoids
    // per-call policy overhead. Each policy and its rationale is documented
    // inside ECM_ApplyPipePolicies.
    ECM_ApplyPipePolicies();

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] OpenECMUSB: OK (OUT=0x%02X, IN=0x%02X, timeout=%lu ms).\n",
            g_Dev.pipeOut, g_Dev.pipeIn, ECM_USB_TIMEOUT_MS);
    return true;
}

// ============================================================================
//  CloseECMUSB
//  Releases WinUSB resources and closes the device handle.
//
//  Teardown order (MUST follow acquisition order in reverse):
//    1. WinUsb_Free(hWinUsb)  — releases WinUSB driver resources.
//       WinUsb_Free is required; leaking hWinUsb prevents the device from
//       being accessed by other applications after this DLL releases it.
//    2. CloseHandle(hFile)    — releases the OS kernel handle.
//       Must follow WinUsb_Free; closing hFile while hWinUsb is live causes
//       I/O cancellation races in the kernel driver stack.
//
//  NOTE: g_Dev.devicePath is intentionally NOT cleared here so that
//  ECMUSBRecover() can reopen the same device path without re-enumerating.
//
//  Hold-Last Strategy Context:
//  The hold-last recovery strategy (repeating last valid command on failure)
//  is applied by the CALLER in mdlOutputs. This library only signals failure
//  to the caller. The caller (EC01M_SFunction) maintains the last valid
//  command buffer and resubmits it on the next cycle if Write/Read fails.
//  This prevents the real-time loop from being stalled by resets.
// ============================================================================
void __stdcall CloseECMUSB(void)
{
    if (!g_Dev.isOpen)
        return;

    // Free the WinUSB handle before closing the file handle.
    // WinUsb_Free is required; leaking hWinUsb prevents the device from
    // being accessed by other applications.
    if (g_Dev.hWinUsb != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsb);
        g_Dev.hWinUsb = NULL;
    }

    // Close the underlying file handle. Must follow WinUsb_Free() —
    // closing hFile while hWinUsb is live causes I/O cancellation races.
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    g_Dev.pipeOut = 0;
    g_Dev.pipeIn  = 0;
    g_Dev.isOpen  = FALSE;
    // g_Dev.devicePath intentionally preserved — ECMUSBRecover() reads it
    // to reopen the same physical device without re-enumerating.
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

    // Perform the bulk write. WinUsb_WritePipe internally uses overlapped I/O
    // with the timeout configured in OpenECMUSB, so this call will not block
    // indefinitely if the device is stalled or disconnected.
    ULONG bytesWritten = 0;
    BOOL ok = WinUsb_WritePipe(
        g_Dev.hWinUsb,
        g_Dev.pipeOut,
        data,
        (ULONG)dwLength,
        &bytesWritten,
        NULL);  // NULL = use internal overlapped I/O (timeout-bounded)

    // Enforce full write; any partial or timed-out write is an error in the
    // protocol layer. The hold-last strategy ensures the last valid command
    // is repeated by the caller, preventing loss of synchronization.
    // Do NOT reset pipes here — that's left for ECMUSBRecover() at shutdown.
    if (!ok || bytesWritten != (ULONG)dwLength)
    {
        DWORD err = GetLastError();
        if (err == ERROR_SEM_TIMEOUT)
            ECM_Log("[ECM] ECMUSBWrite: timed out after %lu ms — "
                    "device stalled or disconnected? (hold last).\n",
                    ECM_USB_TIMEOUT_MS);
        else
            ECM_Log("[ECM] ECMUSBWrite failed: Err=0x%08X "
                    "(wrote %lu / %lu bytes, hold last).\n",
                    err, bytesWritten, dwLength);
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

    // Perform the bulk read. WinUsb_ReadPipe internally uses overlapped I/O
    // with the timeout configured in OpenECMUSB, so this call will not block
    // indefinitely if the device is stalled or disconnected.
    ULONG bytesRead = 0;
    BOOL ok = WinUsb_ReadPipe(
        g_Dev.hWinUsb,
        g_Dev.pipeIn,
        data,
        (ULONG)dwLength,
        &bytesRead,
        NULL);  // NULL = use internal overlapped I/O (timeout-bounded)

    // Enforce full-frame reads. Returning success on a short read breaks the
    // fixed-size structure assumptions of the protocol and would allow stale
    // or partial data to leak through. Do NOT reset pipes here — that would
    // block the real-time loop.
    if (!ok || bytesRead != (ULONG)dwLength)
    {
        DWORD err = GetLastError();
        if (err == ERROR_SEM_TIMEOUT)
            ECM_Log("[ECM] ECMUSBRead: timed out after %lu ms — "
                    "device stalled or disconnected? (hold last).\n",
                    ECM_USB_TIMEOUT_MS);
        else
            ECM_Log("[ECM] ECMUSBRead failed: Err=0x%08X "
                    "(read %lu / %lu bytes, hold last).\n",
                    err, bytesRead, dwLength);
        return false;
    }

    return true;
}

// ============================================================================
//  ECMUSBRecover
//  Closes the WinUSB handle to flush any stalled I/O state, then reopens it
//  using the cached g_Dev.devicePath — bypassing re-enumeration to guarantee
//  the same physical device is targeted.
//
//  *** isOpen IS SET FALSE AT THE TOP — CRITICAL SAFETY RULE ***
//  g_Dev.isOpen is set FALSE immediately after the guard check, BEFORE any
//  handles are freed. This guarantees that if recovery fails at any point
//  (device disconnected, WinUsb_Initialize fails, etc.) and the function
//  returns early, ECMUSBWrite/Read will see isOpen==FALSE on the next
//  real-time cycle and return false immediately rather than passing a NULL
//  or stale handle to WinUSB APIs — which would cause an Access Violation
//  and crash the host process (MATLAB/Simulink). isOpen is only restored to
//  TRUE at the very end of a fully successful reopen sequence.
//
//  *** TIMING CRITICAL — NOT REAL-TIME SAFE ***
//  This function can take 10-100 milliseconds due to:
//  - WinUsb_Free: ~1-5 ms (driver resource cleanup)
//  - CloseHandle: ~1-5 ms (kernel object cleanup)
//  - CreateFileW: ~5-20 ms (driver stack re-engagement, even cached path)
//  - WinUsb_Initialize: ~5-50 ms (driver re-initialization)
//  MUST NOT be called from the real-time loop (mdlOutputs). Call ONLY from
//  mdlTerminate at controlled shutdown. The handle is closed again by
//  CloseECMUSB before mdlTerminate returns.
//
//  CRITICAL FOR MULTI-DEVICE SYSTEMS:
//  Uses the cached device path to reopen WITHOUT re-enumerating. This ensures
//  the same physical device is targeted. Re-enumeration could return a
//  different device if enumeration order has changed.
// ============================================================================
void __stdcall ECMUSBRecover(void)
{
    if (!g_Dev.isOpen)
        return;

    // *** Mark closed IMMEDIATELY — before any handle operations ***
    // If any step below fails and we return early, ECMUSBWrite/Read will
    // see isOpen==FALSE and return false safely rather than calling WinUSB
    // with a NULL handle, which would cause an Access Violation crash.
    // isOpen is only set back to TRUE at the end of a successful reopen.
    g_Dev.isOpen = FALSE;

    ECM_Log("[ECM] ECMUSBRecover: flushing stalled I/O (isOpen set FALSE)...\n");
    ECM_Log("[ECM]   WARNING: This operation can take 10-100ms "
            "(NOT real-time safe).\n");

    // ---- Teardown (reverse acquisition order) --------------------------------
    // Free the WinUSB handle before closing the file handle to avoid I/O
    // cancellation races in the kernel driver stack.
    if (g_Dev.hWinUsb != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsb);
        g_Dev.hWinUsb = NULL;
    }

    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    // g_Dev.devicePath is preserved — not cleared so we can reopen the
    // same physical device below without re-enumerating.

    // ---- Reopen using cached path (no re-enumeration) -----------------------
    // This reopen targets the exact same physical Interface 1 device node that
    // was opened originally. Using the cached "&mi_01" path avoids silent
    // device identity loss that could occur if enumeration order changes.
    g_Dev.hFile = CreateFileW(
        g_Dev.devicePath,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (g_Dev.hFile == INVALID_HANDLE_VALUE)
    {
        // Device is disconnected or path is no longer valid.
        // isOpen remains FALSE — Write/Read will fail safely on next cycle.
        ECM_Log("[ECM] ECMUSBRecover: reopen failed: 0x%08X "
                "(device disconnected? isOpen stays FALSE).\n",
                GetLastError());
        return;
    }

    // Reinitialise WinUSB on the reopened file handle.
    // As in OpenECMUSB, the "&mi_01" path gives direct access to Interface 1.
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        ECM_Log("[ECM] ECMUSBRecover: WinUsb_Initialize failed: 0x%08X "
                "(isOpen stays FALSE).\n", GetLastError());
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        // isOpen remains FALSE — safe for the next real-time cycle.
        return;
    }

    // Reapply all pipe policies. They are lost when the WinUSB handle is
    // freed and must be restored before the next I/O. Uses cached pipeOut
    // and pipeIn addresses — no re-query needed (addresses are stable for
    // the same physical device on the same port).
    ECM_ApplyPipePolicies();

    // Only set isOpen TRUE here — full reopen succeeded.
    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] ECMUSBRecover: reopen succeeded (multi-device identity "
            "preserved, isOpen=TRUE).\n");
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
//   g_Dev.hWinUsb. Extracted as a shared helper to eliminate duplication
//   between OpenECMUSB and ECMUSBRecover. Policies are lost when the WinUSB
//   handle is freed and must be re-applied after every handle re-acquisition.
//
// POLICIES APPLIED (all non-fatal on failure — driver defaults used):
//
//   PIPE_TRANSFER_TIMEOUT : ECM_USB_TIMEOUT_MS (3000 ms) on OUT and IN
//     Prevents indefinite stalls on broken or disconnected devices.
//     Applied to both pipes independently.
//
//   AUTO_FLUSH (OUT pipe) : TRUE
//     Sends a zero-length packet (ZLP) when a transfer is an exact multiple
//     of the endpoint's MaxPacketSize. Ensures the device recognises frame
//     boundaries on fixed-size protocol messages (504 or 512 bytes) and
//     prevents data from being buffered indefinitely in the USB stack.
//
//   ALLOW_PARTIAL_READS (IN pipe) : TRUE
//     Allows short transfers to complete immediately rather than waiting for
//     the full buffer to fill. Critical for protocols with variable-length
//     or delimited messages. Without this, short reads would time out.
//
//   IGNORE_SHORT_PACKETS (IN pipe) : FALSE
//     Ensures the application is notified when a short packet is received.
//     Critical for protocol frame boundary validation. If a frame is shorter
//     than expected the caller is notified immediately rather than silently
//     accepting partial data.
// ============================================================================
static void ECM_ApplyPipePolicies(void)
{
    // Timeout on both pipes. Non-fatal if SetPipePolicy fails — WinUSB
    // provides internal timeout mechanisms via driver defaults.
    ULONG timeout = ECM_USB_TIMEOUT_MS;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
        ECM_Log("[ECM] SetPipePolicy(OUT, TIMEOUT) failed: 0x%08X "
                "(non-fatal).\n", GetLastError());

    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
        ECM_Log("[ECM] SetPipePolicy(IN, TIMEOUT) failed: 0x%08X "
                "(non-fatal).\n", GetLastError());

    // Auto-flush on OUT: send ZLP at exact multiples of MaxPacketSize.
    // Ensures device recognises frame boundaries; prevents stale buffering.
    UCHAR autoFlush = TRUE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                              AUTO_FLUSH, sizeof(UCHAR), &autoFlush))
        ECM_Log("[ECM] SetPipePolicy(OUT, AUTO_FLUSH) failed: 0x%08X "
                "(non-fatal).\n", GetLastError());

    // Allow partial reads on IN: short transfers complete immediately.
    // Without this, short reads wait for a full buffer and then time out.
    UCHAR allowPartial = TRUE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              ALLOW_PARTIAL_READS, sizeof(UCHAR), &allowPartial))
        ECM_Log("[ECM] SetPipePolicy(IN, ALLOW_PARTIAL_READS) failed: 0x%08X "
                "(non-fatal).\n", GetLastError());

    // Disable IGNORE_SHORT_PACKETS: notify app on short frames so protocol
    // frame boundary violations are detected immediately, not silently ignored.
    UCHAR ignoreShort = FALSE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              IGNORE_SHORT_PACKETS, sizeof(UCHAR), &ignoreShort))
        ECM_Log("[ECM] SetPipePolicy(IN, IGNORE_SHORT_PACKETS) failed: 0x%08X "
                "(non-fatal).\n", GetLastError());
}

// ============================================================================
// ECM_MatchHardwareId
// ============================================================================
// PURPOSE:
//   Reads the SPDRP_HARDWAREID registry property for a device node and
//   performs a case-insensitive substring search for searchUpper across ALL
//   strings in the REG_MULTI_SZ list.
//
// REG_MULTI_SZ format:
//   SPDRP_HARDWAREID returns a REG_MULTI_SZ — a sequence of null-terminated
//   wide strings followed by a final double-null terminator:
//     "USB\VID_16C0&PID_05DF&MI_01\0USB\VID_16C0&PID_05DF\0\0"
//   The first string is the most specific ID (includes &MI_01); subsequent
//   strings are progressively less specific. We iterate all strings in the
//   list to ensure a match is found even if the VID/PID token appears in a
//   less-specific entry.
//
// PARAMETERS:
//   [in]  hDevInfo     - Device info set (not destroyed by this function)
//   [in]  pDevData     - Pointer to the device info data for this node
//   [in]  searchUpper  - Upper-case VID/PID search token, e.g. "VID_16C0&PID_05DF"
//
// RETURN VALUE:
//   TRUE  - searchUpper was found as a substring in any hardware ID string
//   FALSE - No match, or registry property could not be read
// ============================================================================
static BOOL ECM_MatchHardwareId(
    HDEVINFO         hDevInfo,
    SP_DEVINFO_DATA *pDevData,
    const WCHAR     *searchUpper)
{
    // Buffer large enough for a typical REG_MULTI_SZ hardware ID list.
    // 512 wide chars = 1024 bytes, sufficient for all standard USB IDs.
    WCHAR hwIdBuf[512] = { 0 };
    DWORD requiredSize = 0;
    if (!SetupDiGetDeviceRegistryPropertyW(
            hDevInfo, pDevData, SPDRP_HARDWAREID,
            NULL, (PBYTE)hwIdBuf, sizeof(hwIdBuf), &requiredSize))
    {
        return FALSE;
    }

    // Walk the REG_MULTI_SZ list. Each entry is a null-terminated string;
    // the list ends with an empty string (double null terminator).
    // We upper-case and substring-search each entry independently.
    const WCHAR* end = (const WCHAR*)((PBYTE)hwIdBuf + requiredSize);
    for (const WCHAR *p = hwIdBuf; p < end && *p != L'\0'; p += wcslen(p) + 1))
    {
        ECM_Log("[ECM]     HW-ID: %ls\n", p);

        WCHAR entry[256];
        wcsncpy_s(entry, 256, p, _TRUNCATE);
        _wcsupr_s(entry, 256);

        if (wcsstr(entry, searchUpper) != NULL)
            return TRUE;
    }

    return FALSE;
}

// ============================================================================
// ECM_FindDevicePath
// ============================================================================
// PURPOSE:
//   Main device discovery entry point. Locates the WinUSB Interface 1 device
//   node for the ECM-SK by VID/PID using two searches in priority order.
//
//   SEARCH 1 — GUID_DEVINTERFACE_USB_DEVICE  (primary)
//   ────────────────────────────────────────────────────
//   Enumerates all interfaces registered under GUID_DEVINTERFACE_USB_DEVICE
//   with DIGCF_PRESENT | DIGCF_DEVICEINTERFACE. This finds the "&mi_01"
//   Interface 1 child node directly when WinUSB.sys is bound to Interface 1
//   (via INF, Zadig, or MS OS Descriptor). The hardware ID of this node is:
//     USB\VID_16C0&PID_05DF&MI_01\...
//   which contains the VID/PID match token. Path format:
//     \\?\usb#vid_16c0&pid_05df&mi_01#...#{dee824ef-...}
//
//   SEARCH 2 — All USB devices  (fallback)
//   ────────────────────────────────────────
//   Enumerates all present USB devices with DIGCF_PRESENT |
//   DIGCF_DEVICEINTERFACE and the "USB" enumerator. This catches devices
//   whose Interface 1 WinUSB node may be registered under a non-standard
//   GUID or whose enumeration order differs from SEARCH 1. The same
//   hardware ID match and device interface path retrieval logic is used.
//   NOTE: DIGCF_DEVICEINTERFACE is required here so that
//   SetupDiEnumDeviceInterfaces can retrieve a CreateFile-able path.
//
// PARAMETERS:
//   [out] pathBuf    - Buffer to receive the device path (MAX_PATH minimum)
//   [in]  pathBufLen - Size of pathBuf in characters
//
// RETURN VALUE:
//   TRUE  - Device found; pathBuf populated and ready for CreateFileW
//   FALSE - Not found via either strategy; see log for diagnostics
//
// TROUBLESHOOTING:
//   Both searches return NOT FOUND:
//     → WinUSB.sys not installed on Interface 1; use Zadig
//     → Device not connected, or VID/PID differs from ECM_USB_VID/PID
//   SEARCH 1/2 succeeds but CreateFile later fails with ACCESS_DENIED:
//     → Another process holds the WinUSB handle; close other instances
//
// REAL-TIME SAFETY:
//   Uses SetupAPI — NOT real-time safe. Call ONLY during initialisation
//   (mdlStart), never from the real-time control loop (mdlOutputs).
// ============================================================================
static BOOL ECM_FindDevicePath(WCHAR *pathBuf, DWORD pathBufLen)
{
    ECM_Log("[ECM] ECM_FindDevicePath: Searching for VID_%04X&PID_%04X\n",
            ECM_USB_VID, ECM_USB_PID);

    WCHAR vidPidStr[64];
    swprintf_s(vidPidStr, 64, L"VID_%04X&PID_%04X&MI_01", ECM_USB_VID, ECM_USB_PID); 

    // ===== SEARCH 1: GUID_DEVINTERFACE_USB_DEVICE (primary) =================
    ECM_Log("[ECM] SEARCH 1: GUID_DEVINTERFACE_USB_DEVICE\n");
    if (ECM_FindDevicePathByVidPid_WinUSB(vidPidStr, pathBuf, pathBufLen))
    {
        ECM_Log("[ECM] Found via SEARCH 1.\n");
        return TRUE;
    }

    // ===== SEARCH 2: All USB devices with DIGCF_DEVICEINTERFACE (fallback) ==
    ECM_Log("[ECM] SEARCH 2: All USB devices (fallback)\n");
    if (ECM_FindDevicePathByVidPid_AllUSB(vidPidStr, pathBuf, pathBufLen))
    {
        ECM_Log("[ECM] Found via SEARCH 2.\n");
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
//   all device interfaces registered under GUID_DEVINTERFACE_USB_DEVICE.
//
//   For a composite device with WinUSB bound to Interface 1, this GUID
//   enumerates the "&mi_01" child node whose hardware ID contains the
//   VID/PID match token. The returned path is directly usable by CreateFileW
//   and WinUsb_Initialize to obtain an Interface 1 handle.
//
// PARAMETERS:
//   [in]  vidPidStr  - Upper-case VID/PID token, e.g. "VID_16C0&PID_05DF"
//   [out] pathBuf    - Buffer to receive device path if found
//   [in]  pathBufLen - Size of pathBuf in characters (MAX_PATH minimum)
//
// RETURN VALUE:
//   TRUE  - Device found; path written to pathBuf
//   FALSE - Not found, or enumeration error
//
// EXAMPLE PATH RETURNED:
//   \\?\usb#vid_16c0&pid_05df&mi_01#6&1a2b3c4d&0&0001#{dee824ef-...}
//
// NOTES:
//   - Uses DIGCF_PRESENT | DIGCF_DEVICEINTERFACE (required for interface GUIDs)
//   - REG_MULTI_SZ hardware ID is iterated fully via ECM_MatchHardwareId()
//   - Device info set is always destroyed before returning
// ============================================================================
static BOOL ECM_FindDevicePathByVidPid_WinUSB(
    const WCHAR *vidPidStr,
    WCHAR       *pathBuf,
    DWORD        pathBufLen)
{
    ECM_Log("[ECM]   Searching GUID_DEVINTERFACE_USB_DEVICE...\n");

    // DIGCF_DEVICEINTERFACE is mandatory for interface-GUID queries.
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

    // Build the uppercase search token once, outside the enumeration loop
    WCHAR searchUpper[64];
    wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
    _wcsupr_s(searchUpper, 64);

    BOOL found = FALSE;
    SP_DEVICE_INTERFACE_DATA ifData = { 0 };
    ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    for (DWORD idx = 0;
         SetupDiEnumDeviceInterfaces(hDevInfo, NULL,
                                     &GUID_DEVINTERFACE_USB_DEVICE,
                                     idx, &ifData);
         idx++)
    {
        // Two-call pattern: first call returns required buffer size,
        // second call fills the detail struct including DevicePath and
        // populates devData for the hardware ID query.
        DWORD reqSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &reqSize, NULL);
        if (reqSize == 0)
            continue;

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, reqSize);
        if (!pDetail)
        {
            ECM_Log("[ECM]   Memory allocation failed\n");
            break;
        }
        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        SP_DEVINFO_DATA devData = { 0 };
        devData.cbSize = sizeof(SP_DEVINFO_DATA);

        if (!SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                              pDetail, reqSize,
                                              NULL, &devData))
        {
            ECM_Log("[ECM]   WinUSB[%lu]: GetDeviceInterfaceDetailW failed: "
                    "0x%08X\n", idx, GetLastError());
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        // Search all strings in the REG_MULTI_SZ hardware ID list.
        // ECM_MatchHardwareId logs each entry and returns TRUE on first match.
        ECM_Log("[ECM]   WinUSB[%lu]: checking hardware IDs...\n", idx);
        if (!ECM_MatchHardwareId(hDevInfo, &devData, searchUpper))
        {
            ECM_Log("[ECM]   WinUSB[%lu]: no match\n", idx);
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        ECM_Log("[ECM]   WinUSB[%lu]: MATCH! Path: %ls\n",
                idx, pDetail->DevicePath);

        size_t pathLen = wcslen(pDetail->DevicePath) + 1;
        if (pathLen <= (size_t)pathBufLen)
        {
            wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
            found = TRUE;
        }
        else
        {
            ECM_Log("[ECM]   ERROR: path too long (%zu chars)\n", pathLen);
        }

        HeapFree(GetProcessHeap(), 0, pDetail);
        if (found) break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    ECM_Log("[ECM]   WinUSB GUID search: %s\n", found ? "FOUND" : "NOT FOUND");
    return found;
}

// ============================================================================
// ECM_FindDevicePathByVidPid_AllUSB
// ============================================================================
// PURPOSE:
//   Fallback search across all present USB devices. Enumerates with both
//   DIGCF_PRESENT and DIGCF_DEVICEINTERFACE so that SetupDiEnumDeviceInterfaces
//   can successfully retrieve CreateFile-able interface paths.
//
//   This is SEARCH 2. It catches WinUSB Interface 1 nodes that may be
//   registered under a non-standard GUID or that SEARCH 1 missed for any
//   reason. The same VID/PID hardware ID match and path retrieval logic
//   as SEARCH 1 is used.
//
//   IMPORTANT — why DIGCF_DEVICEINTERFACE is required here:
//   SetupDiEnumDeviceInterfaces requires the info set to have been created
//   with DIGCF_DEVICEINTERFACE. Without it, the call returns
//   ERROR_NO_MORE_ITEMS for every device regardless of whether a device
//   interface path exists. The previous DIGCF_PRESENT-only approach was
//   therefore non-functional for path retrieval.
//
// PARAMETERS:
//   [in]  vidPidStr  - Upper-case VID/PID token, e.g. "VID_16C0&PID_05DF"
//   [out] pathBuf    - Buffer to receive the device path
//   [in]  pathBufLen - Size of pathBuf in characters (MAX_PATH minimum)
//
// RETURN VALUE:
//   TRUE  - Device found and a usable path was obtained
//   FALSE - Not found, or found but no interface path available
// ============================================================================
static BOOL ECM_FindDevicePathByVidPid_AllUSB(
    const WCHAR *vidPidStr,
    WCHAR       *pathBuf,
    DWORD        pathBufLen)
{
    ECM_Log("[ECM]   Searching all USB devices "
            "(DIGCF_PRESENT | DIGCF_DEVICEINTERFACE, enumerator=USB)...\n");

    // DIGCF_DEVICEINTERFACE is required so SetupDiEnumDeviceInterfaces can
    // retrieve interface paths. Without this flag it always fails with
    // ERROR_NO_MORE_ITEMS, making path retrieval impossible.
    // DIGCF_PRESENT skips disconnected devices.
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        NULL, L"USB", NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

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
        totalSeen++;

        // Search all strings in the REG_MULTI_SZ hardware ID list.
        if (!ECM_MatchHardwareId(hDevInfo, &devData, searchUpper))
            continue;

        ECM_Log("[ECM]   AllUSB[%lu]: MATCH! Retrieving interface path...\n",
                idx);

        // Retrieve the GUID_DEVINTERFACE_USB_DEVICE interface path for this
        // node. Because the info set was created with DIGCF_DEVICEINTERFACE,
        // this call will succeed if the device has a registered interface path.
        SP_DEVICE_INTERFACE_DATA ifData = { 0 };
        ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData,
                                         &GUID_DEVINTERFACE_USB_DEVICE,
                                         0, &ifData))
        {
            // Device matched by hardware ID but has no WinUSB interface path.
            // This means WinUSB.sys is not installed on this interface.
            ECM_Log("[ECM]   AllUSB[%lu]: no USB device interface path "
                    "(0x%08X) — WinUSB.sys not installed on this interface.\n"
                    "[ECM]   Use Zadig to install WinUSB on Interface 1.\n",
                    idx, GetLastError());
            continue;
        }

        // Two-call pattern for interface detail
        DWORD reqSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &reqSize, NULL);
        if (reqSize == 0)
        {
            ECM_Log("[ECM]   AllUSB[%lu]: failed to get detail size\n", idx);
            continue;
        }

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, reqSize);
        if (!pDetail)
        {
            ECM_Log("[ECM]   Memory allocation failed\n");
            break;
        }
        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                             pDetail, reqSize,
                                             NULL, NULL))
        {
            ECM_Log("[ECM]   AllUSB[%lu]: path: %ls\n",
                    idx, pDetail->DevicePath);

            size_t pathLen = wcslen(pDetail->DevicePath) + 1;
            if (pathLen <= (size_t)pathBufLen)
            {
                wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
                found = TRUE;
            }
            else
            {
                ECM_Log("[ECM]   ERROR: path too long (%zu chars)\n", pathLen);
            }
        }
        else
        {
            ECM_Log("[ECM]   AllUSB[%lu]: GetDeviceInterfaceDetailW failed: "
                    "0x%08X\n", idx, GetLastError());
        }

        HeapFree(GetProcessHeap(), 0, pDetail);
        if (found) break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    ECM_Log("[ECM]   All-USB search: %d devices examined, %s\n",
            totalSeen, found ? "FOUND" : "NOT FOUND");
    return found;
}

// ============================================================================
//  ECM_QueryPipes
//  Queries all endpoints on g_Dev.hWinUsb (Interface 1) and locates the
//  first Bulk-OUT and Bulk-IN pipes, storing their addresses in g_Dev.pipeOut
//  and g_Dev.pipeIn.
//
// Algorithm:
//   1. WinUsb_QueryInterfaceSettings returns the interface descriptor
//      (endpoint count for alternate setting 0)
//   2. For each endpoint, WinUsb_QueryPipe returns its type and address
//   3. First Bulk-OUT (PipeId & 0x80 == 0) becomes pipeOut
//   4. First Bulk-IN  (PipeId & 0x80 != 0) becomes pipeIn
//   5. Return TRUE if both pipes were found, FALSE otherwise
//
// Notes:
//   - Bulk-OUT pipe addresses: typically 0x01, 0x02, etc.
//   - Bulk-IN  pipe addresses: typically 0x81, 0x82, etc. (bit 7 = IN)
//   - Bit 7 of the address indicates direction per USB spec: 0=OUT, 1=IN
//   - Querying g_Dev.hWinUsb is correct because this handle always refers
//     to the Interface 1 WinUSB node (the "&mi_01" path was opened)
//
// Return value:
//   TRUE  — both Bulk-OUT and Bulk-IN pipes found and cached in g_Dev
//   FALSE — discovery failed; caller applies compile-time defaults
// ============================================================================
static BOOL ECM_QueryPipes(void)
{
    // Query the active interface descriptor (alternate setting 0).
    USB_INTERFACE_DESCRIPTOR ifDesc = { 0 };
    if (!WinUsb_QueryInterfaceSettings(g_Dev.hWinUsb, 0, &ifDesc))
    {
        ECM_Log("[ECM] ECM_QueryPipes: WinUsb_QueryInterfaceSettings failed: "
                "0x%08X\n", GetLastError());
        return FALSE;
    }

    ECM_Log("[ECM] ECM_QueryPipes: Interface has %d endpoints.\n",
            (int)ifDesc.bNumEndpoints);

    BOOL foundOut = FALSE, foundIn = FALSE;

    for (UCHAR i = 0; i < ifDesc.bNumEndpoints; i++)
    {
        WINUSB_PIPE_INFORMATION pipeInfo = { 0 };
        if (!WinUsb_QueryPipe(g_Dev.hWinUsb, 0, i, &pipeInfo))
        {
            ECM_Log("[ECM] ECM_QueryPipes: WinUsb_QueryPipe[%d] failed: "
                    "0x%08X\n", (int)i, GetLastError());
            continue;
        }

        ECM_Log("[ECM]   Pipe[%d]: type=%d (0=ctrl,1=iso,2=bulk,3=int) "
                "addr=0x%02X maxPkt=%u\n",
                (int)i, (int)pipeInfo.PipeType,
                (unsigned)pipeInfo.PipeId,
                (unsigned)pipeInfo.MaximumPacketSize);

        // Only process bulk endpoints — ignore control, isochronous, interrupt
        if (pipeInfo.PipeType == UsbdPipeTypeBulk)
        {
            // Bit 7 of PipeId: 0 = OUT (host→device), 1 = IN (device→host)
            if ((pipeInfo.PipeId & 0x80) == 0 && !foundOut)
            {
                g_Dev.pipeOut = pipeInfo.PipeId;
                foundOut = TRUE;
                ECM_Log("[ECM]   -> Assigned Bulk-OUT (host→device).\n");
            }
            else if ((pipeInfo.PipeId & 0x80) != 0 && !foundIn)
            {
                g_Dev.pipeIn = pipeInfo.PipeId;
                foundIn = TRUE;
                ECM_Log("[ECM]   -> Assigned Bulk-IN  (device→host).\n");
            }
        }
    }

    if (!foundOut || !foundIn)
    {
        ECM_Log("[ECM] ECM_QueryPipes: Incomplete — OUT=%d IN=%d. "
                "Check WinUSB is bound to the correct interface.\n",
                foundOut, foundIn);
        return FALSE;
    }

    return TRUE;
}