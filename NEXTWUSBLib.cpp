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
// The ECM-SK device typically enumerates as a USB HID class device (VID 16C0/
// PID 05DF). This WinUSB implementation provides direct access to bulk
// endpoints without the framing overhead of HID reports, suitable when the
// device firmware supports bulk streaming.
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
//   Two strategies are employed in sequence:
//   1. GUID-based search: SetupAPI enumerates interfaces registered with
//      ECM_DEVICE_INTERFACE_GUID (preferred when INF registers the device).
//   2. VID/PID fallback: If GUID search fails, hardware ID strings are
//      scanned for "VID_16C0&PID_05DF" to locate the device. This allows
//      matching even when the device is not registered with a known GUID.
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
//   - WinUSB interface initialized once, reused for all operations
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
// Module-level state (one global instance, not thread-safe by design)
// ============================================================================
// Initialization values have specific meaning:
//   hFile = INVALID_HANDLE_VALUE     : No file handle yet (safe initial state)
//   hWinUsb = NULL                   : No WinUSB interface yet
//   pipeOut = 0                      : 0 is invalid endpoint address (see ECM_PIPE_OUT_DEFAULT)
//   pipeIn = 0                       : 0 is invalid endpoint address (see ECM_PIPE_IN_DEFAULT)
//   isOpen = FALSE                   : Device not yet opened
//   devicePath[0] = 0                : Empty path string
//
// Resources are allocated at OpenECMUSB time and persist until CloseECMUSB
// to avoid per-tick kernel allocation overhead that would introduce scheduling
// jitter in the real-time cycle. The devicePath is cached at open time
// (BEFORE CreateFile) to support multi-device systems (see above).
static ECM_DEVICE_CTX g_Dev = { INVALID_HANDLE_VALUE, NULL, 0, 0, FALSE, { 0 } };

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL  ECM_FindDevicePath (WCHAR *pathBuf, DWORD pathBufLen);
static BOOL  ECM_FindDevicePathByVidPid(WCHAR *pathBuf, DWORD pathBufLen);
static BOOL  ECM_QueryPipes     (void);
static void  ECM_ResetDevice    (void);

// ============================================================================
//  OpenECMUSB
//  Locates the ECM-SK device by VID/PID, opens an exclusive file handle,
//  initializes WinUSB, discovers bulk endpoints, and configures transfer
//   policies. Returns true on success, false on any failure.
//
// High-level algorithm:
//   1. Search for device path by VID/PID comparison
//   2. Cache the exact device path BEFORE opening (critical for multi-device)
//   3. CreateFile the device path
//   4. WinUsb_Initialize to obtain the WinUSB interface handle
//   5. WinUsb_QueryPipe to locate Bulk-OUT and Bulk-IN endpoints
//   6. WinUsb_SetPipePolicy to configure timeouts and protocol behavior
//
// Device Path Caching (CRITICAL FOR MULTI-DEVICE SYSTEMS):
//   The exact device path is cached BEFORE CreateFile so that ECMUSBRecover()
//   can reopen the same physical device without re-enumerating. This is
//   essential when multiple ECM-SK modules are connected simultaneously.
//   Re-enumeration without caching could return a different device at a
//   different index, silently breaking device identity.
//
// Resource Allocation Strategy:
//   All resources (file handle, WinUSB interface, pipe addresses) are
//   allocated once here and persist until CloseECMUSB. This one-time
//   allocation avoids per-tick kernel overhead that would introduce
//   real-time scheduling jitter.
//
// ============================================================================
bool __stdcall OpenECMUSB(void)
{
    if (g_Dev.isOpen)
    {
        ECM_Log("[ECM] OpenECMUSB: already open.\n");
        return true;
    }

    // ----- 1. Locate the device path ----------------------------------------
    WCHAR devicePath[MAX_PATH] = { 0 };

    // Scan all USB devices and match VID/PID by hardware-ID string comparison. 
	// This is slower but more flexible when the device is not pre-registered 
	// with a GUID.
    if (!ECM_FindDevicePathByVidPid(devicePath, MAX_PATH))
    {
        ECM_Log("[ECM] OpenECMUSB: ECM-SK device (VID_%04X&PID_%04X) not found.\n",
                ECM_USB_VID, ECM_USB_PID);
        return false;
    }
    ECM_Log("[ECM] Device path: %ls\n", devicePath);

    // ----- 2. Cache the exact path now, before opening ----------------------
    // CRITICAL FOR MULTI-DEVICE SYSTEMS. Store the exact path so that
    // ECMUSBRecover() can reopen the same physical device without re-enumerating.
    // Re-enumeration might return a different unit at a different enumeration
    // index when multiple ECM-SK modules are connected simultaneously, silently
    // breaking device identity and desynchronizing protocol communication.
    wcsncpy_s(g_Dev.devicePath, MAX_PATH, devicePath, _TRUNCATE);

    // ----- 3. Open a file handle to the device --------------------------------
    // FILE_SHARE_READ | FILE_SHARE_WRITE allows other processes to probe the
    // device properties, reducing contention. The actual data path (WinUSB)
    // provides exclusive access at the pipe level.
    // FILE_FLAG_OVERLAPPED is used internally by WinUSB for timeout enforcement.
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

    // ----- 4. Initialise WinUSB -----------------------------------------------
    // WinUsb_Initialize wraps the file handle and exposes a clean API for
    // pipe operations. On success, g_Dev.hWinUsb is valid and must be freed
    // by WinUsb_Free before the file handle is closed.
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        DWORD err = GetLastError();
        ECM_Log("[ECM] OpenECMUSB: WinUsb_Initialize failed: 0x%08X\n", err);
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    // ----- 5. Discover bulk pipe addresses ------------------------------------
    // Query the device's endpoint configuration to locate Bulk-OUT and Bulk-IN
    // pipes. If discovery fails (e.g., device firmware issues), fall back to
    // compile-time defaults. This allows graceful degradation when device
    // firmware doesn't expose standard endpoint layouts or exhibits issues
    // during enumeration.
    if (!ECM_QueryPipes())
    {
        // Fall back to compile-time defaults if pipe discovery fails
        ECM_Log("[ECM] OpenECMUSB: Pipe discovery failed; using default endpoints "
                "0x%02X / 0x%02X.\n",
                ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT);
        g_Dev.pipeOut = ECM_PIPE_OUT_DEFAULT;
        g_Dev.pipeIn  = ECM_PIPE_IN_DEFAULT;
    }

    // ----- 6. Configure transfer policies ------------------------------------
    // These policies are configured once at open time and persist for all
    // transfers. This one-time configuration avoids per-call policy setup
    // overhead. Each policy is documented with its rationale.

    // Set timeout for both pipes. If SetPipePolicy fails, we continue with
    // default timeout (non-fatal) as the WinUSB driver provides internal
    // timeout mechanisms even if this explicit policy setting fails.
    ULONG timeout = ECM_USB_TIMEOUT_MS;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
    {
        ECM_Log("[ECM] OpenECMUSB: SetPipePolicy(OUT, timeout) failed: 0x%08X "
                "(non-fatal — using driver default).\n", GetLastError());
    }

    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout))
    {
        ECM_Log("[ECM] OpenECMUSB: SetPipePolicy(IN, timeout) failed: 0x%08X "
                "(non-fatal — using driver default).\n", GetLastError());
    }

    // Enable auto-flush on OUT pipe to send ZLP when transfer is a multiple of
    // the max-packet-size. This ensures the device recognizes frame boundaries
    // on fixed-size protocol messages and prevents data from being buffered
    // indefinitely. Non-fatal if unsupported by device.
    UCHAR autoFlush = TRUE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                              AUTO_FLUSH, sizeof(UCHAR), &autoFlush))
    {
        ECM_Log("[ECM] OpenECMUSB: SetPipePolicy(OUT, AUTO_FLUSH) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }

    // Allow partial reads on IN pipe so short transfers complete immediately
    // rather than waiting for a full buffer. Critical for protocols with
    // variable-length or delimited messages that don't fill the entire buffer.
    // Without this, short reads would timeout waiting for more data.
    UCHAR allowPartial = TRUE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              ALLOW_PARTIAL_READS, sizeof(UCHAR), &allowPartial))
    {
        ECM_Log("[ECM] OpenECMUSB: SetPipePolicy(IN, ALLOW_PARTIAL_READS) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }

    // Disable IGNORE_SHORT_PACKETS (set to FALSE) so the app is notified when
    // a short packet is received. This is critical for protocol frame boundary
    // validation and ensures fixed-size protocol frame assumptions are validated
    // at the USB level. If a frame is shorter than expected, we know immediately.
    UCHAR ignoreShort = FALSE;
    if (!WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                              IGNORE_SHORT_PACKETS, sizeof(UCHAR), &ignoreShort))
    {
        ECM_Log("[ECM] OpenECMUSB: SetPipePolicy(IN, IGNORE_SHORT_PACKETS) failed: 0x%08X "
                "(non-fatal — device may not support this).\n", GetLastError());
    }

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] OpenECMUSB: OK (OUT=0x%02X, IN=0x%02X, timeout=%lu ms).\n",
            g_Dev.pipeOut, g_Dev.pipeIn, ECM_USB_TIMEOUT_MS);
    return true;
}

// ============================================================================
//  CloseECMUSB
//  Releases WinUSB resources and closes the device handle, resetting
//  internal state to allow subsequent OpenECMUSB calls.
//
//  NOTE: g_Dev.devicePath is intentionally NOT cleared here so that
//  ECMUSBRecover can reopen the same device path after calling this function.
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

    // Free the WinUSB interface before closing the file handle.
    // WinUsb_Free is required; leaking hWinUsb prevents the device from
    // being accessed by other applications.
    if (g_Dev.hWinUsb != NULL)
    {
        WinUsb_Free(g_Dev.hWinUsb);
        g_Dev.hWinUsb = NULL;
    }

    // Close the underlying file handle
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    // Clear pipe addresses and state
    g_Dev.pipeOut = 0;
    g_Dev.pipeIn  = 0;
    g_Dev.isOpen  = FALSE;
    // g_Dev.devicePath intentionally preserved — ECMUSBRecover reads it after
    // calling this to reopen the same physical device.
    ECM_Log("[ECM] CloseECMUSB: device closed.\n");
}

// ============================================================================
//  ECMUSBWrite
//  Writes `dwLength` bytes from `data` to the EC-01M bulk-OUT endpoint.
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
//
// ============================================================================
bool __stdcall ECMUSBWrite(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBWrite: invalid state or parameters.\n");
        return false;
    }

    // Perform the bulk write. WinUsb_WritePipe internally uses overlapped I/O
    // with the timeout configured above, so this call will not block
    // indefinitely if the device is stalled or disconnected.
    ULONG bytesWritten = 0;
    BOOL ok = WinUsb_WritePipe(
        g_Dev.hWinUsb,
        g_Dev.pipeOut,
        data,
        (ULONG)dwLength,
        &bytesWritten,
        NULL);  // NULL = use default overlapped I/O (timeout-bounded)

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
                    "device stalled or disconnected? (hold last).\n", ECM_USB_TIMEOUT_MS);
        }
        else
        {
            ECM_Log("[ECM] ECMUSBWrite failed: Err=0x%08X (wrote %lu / %lu bytes, hold last).\n",
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
//  Reads `dwLength` bytes into `data` from the EC-01M bulk-IN endpoint.
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
//
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
    // with the timeout configured above, so this call will not block
    // indefinitely if the device is stalled or disconnected.
    ULONG bytesRead = 0;
    BOOL ok = WinUsb_ReadPipe(
        g_Dev.hWinUsb,
        g_Dev.pipeIn,
        data,
        (ULONG)dwLength,
        &bytesRead,
        NULL);  // NULL = use default overlapped I/O (timeout-bounded)

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
                    "device stalled or disconnected? (hold last).\n", ECM_USB_TIMEOUT_MS);
        }
        else
        {
            ECM_Log("[ECM] ECMUSBRead failed: Err=0x%08X (read %lu / %lu bytes, hold last).\n",
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
//  Closes the WinUSB handle to flush any stalled I/O state, then reopens it
//  using the cached g_Dev.devicePath — bypassing re-enumeration to guarantee
//  the same physical device is targeted even when multiple ECM-SK modules
//  are connected simultaneously.
//
//  *** TIMING CRITICAL — NOT REAL-TIME SAFE ***
//  This function can take 10-100 milliseconds (order of tens of ms) due to:
//  - WinUsb_Free: ~1-5 ms (driver resource cleanup)
//  - CloseHandle: ~1-5 ms (kernel object cleanup)
//  - CreateFileW: ~5-20 ms (re-enumeration, even with cached path)
//  - WinUsb_Initialize: ~5-50 ms (driver re-initialization)
//
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

    ECM_Log("[ECM] ECMUSBRecover: closing WinUSB handle to flush stalled I/O...\n");
    ECM_Log("[ECM]   WARNING: This operation can take 10-100ms (NOT real-time safe).\n");

    // Free the WinUSB interface to close any stalled pipes
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

    // g_Dev.devicePath is preserved — it is intentionally not cleared so we
    // can reopen here without re-enumerating.

    // Attempt to reopen using the exact path from the original open — best
    // effort at shutdown, failure is non-fatal. This reopen WITHOUT re-enumeration
    // ensures the same physical device is targeted when multiple ECM-SK modules
    // are connected simultaneously. Using the cached path avoids silent device
    // identity loss that could occur with re-enumeration.
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

    // Reinitialise WinUSB for the reopened handle
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        ECM_Log("[ECM] ECMUSBRecover: WinUsb_Initialize failed: 0x%08X\n",
                GetLastError());
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return;
    }

    // Reconfigure pipe policies as in OpenECMUSB (replicate settings)
    ULONG timeout = ECM_USB_TIMEOUT_MS;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                         PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                         PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);

    UCHAR autoFlush = TRUE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                         AUTO_FLUSH, sizeof(UCHAR), &autoFlush);

    UCHAR allowPartial = TRUE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                         ALLOW_PARTIAL_READS, sizeof(UCHAR), &allowPartial);

    UCHAR ignoreShort = FALSE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                         IGNORE_SHORT_PACKETS, sizeof(UCHAR), &ignoreShort);

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] ECMUSBRecover: handle reopened successfully using cached path "
            "(multi-device identity preserved).\n");
}

// [Rest of internal helpers remain the same but with enhanced comments...]

// ============================================================================
// ============================================================================
//  INTERNAL HELPERS
// ============================================================================
// ============================================================================

// ============================================================================
// ECM_FindDevicePathByVidPid_InClass
// ============================================================================
// PURPOSE:
//   Helper function that searches for a USB device in a specific device class
//   (HID or USB) by matching Vendor ID and Product ID.
//
// PARAMETERS:
//   [in]  HDEVINFO hDevInfo        - Device info set from SetupDiGetClassDevsW
//   [in]  const WCHAR *vidPidStr   - VID/PID search string (e.g., "VID_16C0&PID_05DF")
//   [out] WCHAR *pathBuf           - Buffer to store device path if found
//   [in]  DWORD pathBufLen         - Size of pathBuf in characters (must be MAX_PATH)
//   [in]  const WCHAR *className   - Device class name for logging ("HID" or "USB")
//
// RETURN VALUE:
//   TRUE  - Device found and path copied to pathBuf
//   FALSE - Device not found in this class, or error occurred
//
// BEHAVIOR:
//   1. Enumerates all devices in the specified class (HID or USB)
//   2. For each device, retrieves the hardware ID from registry
//   3. Compares hardware ID against the search string (case-insensitive)
//   4. When a match is found:
//      - Attempts to get device interface using primary GUID
//      - Falls back to alternate GUID if primary fails
//      - Retrieves the device path (e.g., \\?\hid#vid_16c0&pid_05df#...)
//      - Copies path to output buffer if it fits
//   5. Provides detailed logging for debugging
//
// CRITICAL NOTES:
//   - For HID devices: Uses GUID_DEVINTERFACE_HID as primary GUID
//   - For USB devices: Uses GUID_DEVINTERFACE_USB_DEVICE as primary GUID
//   - Caller MUST provide sufficient buffer (MAX_PATH minimum)
//   - Device info set (hDevInfo) is cleaned up by this function
//   - Function logs every step for troubleshooting device enumeration
//   - Hardware ID matching is case-insensitive using _wcsupr_s
//
// EXAMPLE HARDWARE IDs (from Windows registry):
//   HID device:   USB\VID_16C0&PID_05DF&REV_0100&MI_00
//   USB device:   USB\VID_16C0&PID_05DF\0x0001
//
// LOG OUTPUT EXAMPLES:
//   [ECM]   Searching in HID class...
//   [ECM]     Device[0]: USB\VID_16C0&PID_05DF&...
//   [ECM]           -> ✓ MATCH! Getting device path...
//   [ECM]           Found path: \\?\hid#vid_16c0&pid_05df#...
//   [ECM]           ✓✓✓ SUCCESS - Device path copied
//   [ECM]   HID class: 1 devices, result: ✓ FOUND
static BOOL ECM_FindDevicePathByVidPid_InClass(
    HDEVINFO hDevInfo, 
    const WCHAR *vidPidStr, 
    WCHAR *pathBuf, 
    DWORD pathBufLen,
    const WCHAR *className)
{
    ECM_Log("[ECM]   Searching in %ls class...\n", className);
    
    SP_DEVINFO_DATA devData = { 0 };
    devData.cbSize = sizeof(SP_DEVINFO_DATA);

    BOOL found = FALSE;
    int deviceIndex = 0;
    
    for (DWORD idx = 0; SetupDiEnumDeviceInfo(hDevInfo, idx, &devData); idx++)
    {
        WCHAR hwId[512] = { 0 };
        DWORD requiredSize = 0;
        
        if (!SetupDiGetDeviceRegistryPropertyW(
                hDevInfo, &devData, SPDRP_HARDWAREID,
                NULL, (PBYTE)hwId, sizeof(hwId), &requiredSize))
        {
            continue;
        }

        ECM_Log("[ECM]     Device[%d]: %ls\n", idx, hwId);
        deviceIndex++;

        // Check for VID/PID match (case-insensitive)
        WCHAR hwIdUpper[512];
        wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
        _wcsupr_s(hwIdUpper, 512);

        WCHAR searchUpper[64];
        wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
        _wcsupr_s(searchUpper, 64);

        if (wcsstr(hwIdUpper, searchUpper) == NULL)
        {
            ECM_Log("[ECM]           -> No match\n");
            continue;
        }

        ECM_Log("[ECM]           -> ✓ MATCH! Getting device path...\n");

        // Found the matching device; now get its interface path
        SP_DEVICE_INTERFACE_DATA ifData = { 0 };
        ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        // For HID devices, use HID GUID; for USB, use USB DEVICE GUID
        GUID *searchGuid = (className[0] == L'H') ? 
            (GUID*)&GUID_DEVINTERFACE_HID : 
            (GUID*)&GUID_DEVINTERFACE_USB_DEVICE;

        if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData, searchGuid, 0, &ifData))
        {
            DWORD err = GetLastError();
            ECM_Log("[ECM]           SetupDiEnumDeviceInterfaces failed: 0x%08X\n", err);
            
            // Try alternate GUID
            GUID *altGuid = (className[0] == L'H') ? 
                (GUID*)&GUID_DEVINTERFACE_USB_DEVICE : 
                (GUID*)&GUID_DEVINTERFACE_HID;
                
            if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData, altGuid, 0, &ifData))
            {
                ECM_Log("[ECM]           Alternate GUID also failed\n");
                continue;
            }
        }

        // Get interface detail size
        DWORD requiredDetailSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &requiredDetailSize, NULL);

        if (requiredDetailSize == 0)
        {
            ECM_Log("[ECM]           Failed to get interface detail size\n");
            continue;
        }

        // Allocate space for interface detail
        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredDetailSize);
        
        if (!pDetail)
        {
            ECM_Log("[ECM]           Memory allocation failed\n");
            break;
        }

        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        
        // Get the actual interface detail (device path)
        if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                             pDetail, requiredDetailSize,
                                             NULL, NULL))
        {
            ECM_Log("[ECM]           Found path: %ls\n", pDetail->DevicePath);
            
            // Verify path length
            size_t pathLen = wcslen(pDetail->DevicePath) + 1;
            if (pathLen <= (size_t)pathBufLen)
            {
                wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
                found = TRUE;
                ECM_Log("[ECM]           ✓✓✓ SUCCESS - Device path copied\n");
            }
            else
            {
                ECM_Log("[ECM]           ERROR: Path too long\n");
            }
        }
        else
        {
            DWORD err = GetLastError();
            ECM_Log("[ECM]           GetDeviceInterfaceDetailW failed: 0x%08X\n", err);
        }
        
        HeapFree(GetProcessHeap(), 0, pDetail);

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    
    ECM_Log("[ECM]   %ls class: %d devices, result: %s\n", 
            className, deviceIndex, found ? "✓ FOUND" : "✗ NOT FOUND");
    
    return found;
}

// ============================================================================
// ECM_FindDevicePathByVidPid
// ============================================================================
// PURPOSE:
//   Main device discovery function. Searches for an EC-01M USB device by
//   Vendor ID and Product ID across all device classes (HID, USB, and generic).
//   
//   The device may be enumerated as either HID or USB depending on driver
//   installation, so this function tries multiple search strategies.
//
// PARAMETERS:
//   [out] WCHAR *pathBuf   - Buffer to store device path if found
//   [in]  DWORD pathBufLen - Size of pathBuf in characters (must be MAX_PATH)
//
// RETURN VALUE:
//   TRUE  - Device found; path is in pathBuf
//   FALSE - Device not found; try troubleshooting steps
//
// SEARCH STRATEGY (in order):
//   
//   SEARCH 1: HID CLASS DEVICES (most likely)
//   ─────────────────────────────────────────
//   - Searches for device in HID (Human Interface Device) class
//   - Uses HID class GUID obtained from HidD_GetHidGuid()
//   - Your EC-01M device is typically enumerated here
//   - Success means device path starts with \\?\hid#...
//
//   SEARCH 2: USB DEVICE CLASS (WinUSB scenario)
//   ──────────────────────────────────────────────
//   - Searches for device in USB Device class
//   - Uses GUID_DEVCLASS_USB_DEVICE
//   - Only if WinUSB driver is pre-installed and device re-enumerated
//   - Success means device path starts with \\?\usb#...
//
//   SEARCH 3: ALL USB DEVICES (fallback)
//   ────────────────────────────────────
//   - Generic enumeration of all USB devices
//   - Uses minimal SetupAPI flags (DIGCF_PRESENT only)
//   - Last resort; provides diagnostic output but no device path
//
// CRITICAL PARAMETERS (in NEXTWUSBLib_internal.h):
//   ECM_USB_VID = 0x16C0   (Vendor ID)
//   ECM_USB_PID = 0x05DF   (Product ID)
//   Search string: "VID_16C0&PID_05DF" (case-insensitive)
//
// DEVICE PATH FORMAT:
//   HID path:     \\?\hid#vid_16c0&pid_05df#0x0001#{GUID}
//   USB path:     \\?\usb#vid_16c0&pid_05df#0x0001
//
// RETURN VALUE INTERPRETATION:
//   TRUE with pathBuf filled
//     → Device found! Ready to call CreateFileW(pathBuf, ...)
//   
//   FALSE with detailed log output
//     → Device not found; check log output for:
//        * Which search succeeded (tells you device class)
//        * What devices were found (tells you VID/PID mismatch)
//        * Which SetupAPI calls failed and why (error codes)
//
// LOG OUTPUT STRUCTURE:
//   [ECM] ECM_FindDevicePathByVidPid: Searching for VID_16C0&PID_05DF
//   [ECM] SEARCH 1: HID Class Devices
//   [ECM]   Searching in HID class...
//   [ECM]     Device[0]: USB\VID_16C0&PID_05DF&REV_0100&MI_00
//   [ECM]           -> ✓ MATCH! Getting device path...
//   [ECM]           Found path: \\?\hid#vid_16c0&pid_05df#0x0001#...
//   [ECM]           ✓✓✓ SUCCESS - Device path copied
//   [ECM]   HID class: 1 devices, result: ✓ FOUND
//
// TROUBLESHOOTING:
//   
//   ✗ All searches return "NOT FOUND"
//   → Device is not connected, or VID/PID is different
//   → Check Device Manager: what devices are visible?
//   → Share the "Device[X]" VID/PID from SEARCH 3 fallback output
//
//   ✗ SetupDiGetClassDevsW fails: 0x0000000D (ERROR_INVALID_DATA)
//   → GUID enumeration issue; try fallback search
//   → This function retries with different GUIDs automatically
//
//   ✗ HidD_GetHidGuid fails or returns wrong GUID
//   → HID driver not installed; WinUSB driver may be in use instead
//   → Check SEARCH 2 output
//
//   ✗ Device found in SEARCH 3 but not in SEARCH 1 or 2
//   → Device enumerated as generic USB; no proper driver class
//   → May need to install HID or WinUSB driver
//   → Use Zadig tool to select correct driver
//
// REAL-TIME SAFETY:
//   This function uses SetupAPI which is NOT real-time safe.
//   Call this ONLY during initialization, NOT in the real-time control loop.
//   Device path discovery should happen once in mdlStart, then cached.
//
// INTEGRATION:
//   Called from OpenECMUSB() in mdlStart phase
//   Device path is cached in g_Dev.devicePath
//   Subsequent calls use CreateFileW(g_Dev.devicePath, ...) without re-search
static BOOL ECM_FindDevicePathByVidPid(WCHAR *pathBuf, DWORD pathBufLen)
{
    ECM_Log("[ECM] ECM_FindDevicePathByVidPid: Searching for VID_%04X&PID_%04X\n", 
            ECM_USB_VID, ECM_USB_PID);
    
    WCHAR vidPidStr[64];
    swprintf_s(vidPidStr, 64, L"VID_%04X&PID_%04X", ECM_USB_VID, ECM_USB_PID);

    // ===== SEARCH 1: HID CLASS (most likely - your device is here!) =====
    ECM_Log("[ECM] SEARCH 1: HID Class Devices\n");
    
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);
    
    HDEVINFO hHidInfo = SetupDiGetClassDevsW(
        &hidGuid, NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hHidInfo != INVALID_HANDLE_VALUE)
    {
        if (ECM_FindDevicePathByVidPid_InClass(
            hHidInfo, vidPidStr, pathBuf, pathBufLen, L"HID"))
        {
            return TRUE;  // Found in HID class!
        }
    }
    else
    {
        ECM_Log("[ECM]   HID enumeration failed: 0x%08X\n", GetLastError());
    }

    // ===== SEARCH 2: USB CLASS =====
    ECM_Log("[ECM] SEARCH 2: USB Device Class\n");
    
    HDEVINFO hUsbInfo = SetupDiGetClassDevsW(
        &GUID_DEVCLASS_USB_DEVICE, NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hUsbInfo != INVALID_HANDLE_VALUE)
    {
        if (ECM_FindDevicePathByVidPid_InClass(
            hUsbInfo, vidPidStr, pathBuf, pathBufLen, L"USB"))
        {
            return TRUE;  // Found in USB class
        }
    }
    else
    {
        ECM_Log("[ECM]   USB enumeration failed: 0x%08X\n", GetLastError());
    }

    // ===== SEARCH 3: ALL USB DEVICES (fallback) =====
    ECM_Log("[ECM] SEARCH 3: All USB Devices (by name)\n");
    
    HDEVINFO hAllUsb = SetupDiGetClassDevsW(
        NULL, L"USB", NULL,
        DIGCF_PRESENT);

    if (hAllUsb != INVALID_HANDLE_VALUE)
    {
        SP_DEVINFO_DATA devData = { 0 };
        devData.cbSize = sizeof(SP_DEVINFO_DATA);
        
        int deviceIndex = 0;
        for (DWORD idx = 0; SetupDiEnumDeviceInfo(hAllUsb, idx, &devData); idx++)
        {
            WCHAR hwId[512] = { 0 };
            if (!SetupDiGetDeviceRegistryPropertyW(
                    hAllUsb, &devData, SPDRP_HARDWAREID,
                    NULL, (PBYTE)hwId, sizeof(hwId), NULL))
                continue;

            ECM_Log("[ECM]   Device[%d]: %ls\n", idx, hwId);
            deviceIndex++;

            WCHAR hwIdUpper[512];
            wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
            _wcsupr_s(hwIdUpper, 512);

            WCHAR searchUpper[64];
            wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
            _wcsupr_s(searchUpper, 64);

            if (wcsstr(hwIdUpper, searchUpper) != NULL)
            {
                ECM_Log("[ECM]     ✓ MATCH FOUND\n");
            }
        }
        SetupDiDestroyDeviceInfoList(hAllUsb);
        ECM_Log("[ECM]   Total devices found: %d\n", deviceIndex);
    }

    ECM_Log("[ECM] ✗✗✗ DEVICE NOT FOUND in any search\n");
    return FALSE;
}

// ============================================================================
//  ECM_QueryPipes
//  Queries all endpoints on interface 0 and locates the first Bulk-OUT and
//  Bulk-IN pipes, storing their addresses in g_Dev.pipeOut and g_Dev.pipeIn.
//
// Algorithm:
//   1. WinUsb_QueryInterfaceSettings returns descriptor info (endpoint count)
//   2. For each endpoint, WinUsb_QueryPipe returns its type and address
//   3. First Bulk-OUT (address & 0x80 == 0) becomes pipeOut
//   4. First Bulk-IN (address & 0x80 != 0) becomes pipeIn
//   5. Return TRUE if both pipes were found, FALSE otherwise
//
// Notes:
//   - Bulk-OUT pipe addresses are typically 0x01, 0x02, etc.
//   - Bulk-IN pipe addresses are typically 0x81, 0x82, etc. (bit 7 = 1)
//   - The high bit (0x80) indicates direction: 0 = OUT, 1 = IN
//
// Return value:
//   TRUE if both Bulk-OUT and Bulk-IN pipes were found and cached, 
//   FALSE if discovery failed (caller will use compile-time defaults).
// ============================================================================
static BOOL ECM_QueryPipes(void)
{
    // Query the active interface descriptor
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

    // Iterate through all endpoints in the interface
    for (UCHAR i = 0; i < ifDesc.bNumEndpoints; i++)
    {
        WINUSB_PIPE_INFORMATION pipeInfo = { 0 };
        if (!WinUsb_QueryPipe(g_Dev.hWinUsb, 0, i, &pipeInfo))
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

        // Check if this pipe is a bulk endpoint
        if (pipeInfo.PipeType == UsbdPipeTypeBulk)
        {
            // Bit 7 of address determines direction: 0 = OUT, 1 = IN
            if ((pipeInfo.PipeId & 0x80) == 0 && !foundOut)
            {
                g_Dev.pipeOut = pipeInfo.PipeId;
                foundOut = TRUE;
                ECM_Log("[ECM]   -> Assigned as Bulk-OUT pipe.\n");
            }
            else if ((pipeInfo.PipeId & 0x80) != 0 && !foundIn)
            {
                g_Dev.pipeIn = pipeInfo.PipeId;
                foundIn = TRUE;
                ECM_Log("[ECM]   -> Assigned as Bulk-IN pipe.\n");
            }
        }
    }

    // Return success only if both pipes were found
    if (!foundOut || !foundIn)
    {
        ECM_Log("[ECM] ECM_QueryPipes: Incomplete — found OUT=%d IN=%d\n",
                foundOut, foundIn);
        return FALSE;
    }

    return TRUE;
}