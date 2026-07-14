// ============================================================================
// NEXTWUSBLib.cpp
// Windows HID API implementation of the NEXTWUSBLib transport layer.
// Interfaces with the ECM-SK EtherCAT Master Module via the standard
// Windows HID class driver (HidUsb.sys).
//
// Device identity:
//   VID 0x16C0  (Van Ooijen Technische Informatica — shared V-USB VID)
//   PID 0x05DF  (Generic HID — not mice/keyboards/joysticks)
//
// The ECM-SK enumerates as a USB HID class device. Windows automatically
// binds HidUsb.sys on first connection.
//
// HID Report framing:
//   Every WriteFile / ReadFile call on a HID device must prefix the payload
//   with a 1-byte Report ID. The ECM-SK uses a single report (ID = 0x00),
//   so the wire buffer is always:
//
//     [ 0x00 | cmdBuf[0] ... cmdBuf[DEF_MA_MAX-1] ]
//      ^-- Report ID (1 byte)   ^-- protocol payload (sizeof(transData)*DEF_MA_MAX)
//
//   WriteFile total = 1 + sizeof(transData)*DEF_MA_MAX bytes
//   ReadFile  total = 1 + sizeof(transData)*DEF_MA_MAX bytes
//
//   12-byte variant: 1 + 12*42 = 505 bytes
//   16-byte variant: 1 + 16*32 = 513 bytes
//
// Overlapped I/O model:
//   The handle is opened with FILE_FLAG_OVERLAPPED. Under the Windows API,
//   ALL ReadFile and WriteFile calls on an overlapped handle MUST supply a
//   valid non-NULL OVERLAPPED structure — passing NULL causes the call to fail
//   with ERROR_INVALID_PARAMETER or produces undefined I/O driver behaviour
//   because the kernel and Win32 completion signalling mechanisms are put out
//   of phase. Both ECMUSBRead and ECMUSBWrite therefore use an OVERLAPPED
//   struct whose hEvent is a manual-reset event allocated once at open time
//   (g_Dev.hEventRead / g_Dev.hEventWrite) and reset with ResetEvent() before
//   each transfer. This eliminates per-tick kernel object allocation overhead
//   that would otherwise introduce jitter into the real-time cycle.
//
// Build requirements:
//   - Link: hid.lib, setupapi.lib
//   - SDK:  Windows SDK 8.1 or later
//   - CRT:  MSVCRT / UCRT
//
// Compilation (MSVC):
//   cl /nologo /EHsc /W3 /DNEXTWUSBLIB_EXPORTS /LD NEXTWUSBLib.cpp
//      /link hid.lib setupapi.lib
//      /DEF:NEXTWUSBLib.def /OUT:NEXTWUSBLib.dll
// ============================================================================

#define NEXTWUSBLIB_EXPORTS
#include "NEXTWUSBLib.h"
#include "NEXTWUSBLib_internal.h"

// ============================================================================
// HID report buffer size
//   1 byte Report ID prefix + full protocol frame
//   Computed at compile time from the active packet variant.
// ============================================================================
#define ECM_FRAME_BYTES    (sizeof(transData) * DEF_MA_MAX)
#define ECM_HID_BUF_BYTES  (1 + ECM_FRAME_BYTES)

// ============================================================================
// Module-level state
// ============================================================================
static ECM_DEVICE_CTX g_Dev = { INVALID_HANDLE_VALUE, FALSE, NULL, NULL, { 0 } };

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL ECM_FindHidDevicePath(WCHAR *pathBuf, DWORD pathBufLen);
static void ECM_CloseHandle(void);

// ============================================================================
//  OpenECMUSB
//  Locates the ECM-SK HID device by VID/PID, opens an exclusive overlapped
//  file handle, allocates the shared read/write OVERLAPPED events, and
//  caches the device path for use by ECMUSBRecover.
//  Returns true on success, false on any failure.
// ============================================================================
bool __stdcall OpenECMUSB(void)
{
    if (g_Dev.isOpen)
    {
        ECM_Log("[ECM] OpenECMUSB: already open.\n");
        return true;
    }

    // ----- 1. Locate the HID device path by VID/PID -------------------------
    WCHAR devicePath[MAX_PATH] = { 0 };
    if (!ECM_FindHidDevicePath(devicePath, MAX_PATH))
    {
        ECM_Log("[ECM] OpenECMUSB: ECM-SK device (VID_%04X&PID_%04X) not found.\n",
                ECM_USB_VID, ECM_USB_PID);
        return false;
    }
    ECM_Log("[ECM] HID device path: %ls\n", devicePath);

    // Cache the exact path now, before opening, so ECMUSBRecover can reopen
    // the same physical device without re-enumerating.  This is critical when
    // multiple ECM-SK modules are connected: re-enumeration might return a
    // different unit at a different enumeration index.
    wcsncpy_s(g_Dev.devicePath, MAX_PATH, devicePath, _TRUNCATE);

    // ----- 2. Open the HID device -------------------------------------------
    // FILE_FLAG_OVERLAPPED is required so that both ECMUSBRead and ECMUSBWrite
    // can supply a valid OVERLAPPED struct to ReadFile/WriteFile and enforce
    // ECM_USB_TIMEOUT_MS via WaitForSingleObject + CancelIoEx.
    // Under the Windows API, ALL I/O on an overlapped handle MUST use a
    // non-NULL OVERLAPPED — passing NULL would cause ERROR_INVALID_PARAMETER
    // or silent undefined behaviour from the I/O manager.
    g_Dev.hFile = CreateFileW(
        devicePath,
        GENERIC_READ | GENERIC_WRITE,
        0,                      // exclusive: do not share — prevents other
                                // processes from interfering with EtherCAT traffic
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,   // required for timeout-bounded read and write
        NULL);

    if (g_Dev.hFile == INVALID_HANDLE_VALUE)
    {
        DWORD err = GetLastError();
        if (err == ERROR_ACCESS_DENIED)
            ECM_Log("[ECM] OpenECMUSB: access denied — device may be open by "
                    "another process, or this is a protected system HID device.\n");
        else
            ECM_Log("[ECM] OpenECMUSB: CreateFile failed: 0x%08X\n", err);
        return false;
    }

    // ----- 3. Allocate shared OVERLAPPED events -----------------------------
    // One manual-reset event per transfer direction, allocated here and freed
    // in ECM_CloseHandle.  Using persistent events and calling ResetEvent()
    // before each transfer avoids CreateEvent/CloseHandle overhead inside the
    // real-time cycle, which would otherwise introduce kernel scheduling jitter
    // at every tick.  The events must be manual-reset: calling GetOverlappedResult
    // after WaitForSingleObject requires the event to still be signalled so that
    // GetOverlappedResult can observe it; an auto-reset event can be cleared by
    // the wait before GetOverlappedResult reads it, causing a deadlock.
    g_Dev.hEventRead  = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_Dev.hEventWrite = CreateEvent(NULL, TRUE, FALSE, NULL);

    if (g_Dev.hEventRead == NULL || g_Dev.hEventWrite == NULL)
    {
        ECM_Log("[ECM] OpenECMUSB: CreateEvent failed: 0x%08X\n", GetLastError());
        if (g_Dev.hEventRead)  { CloseHandle(g_Dev.hEventRead);  g_Dev.hEventRead  = NULL; }
        if (g_Dev.hEventWrite) { CloseHandle(g_Dev.hEventWrite); g_Dev.hEventWrite = NULL; }
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    // ----- 4. Configure HID input buffer depth ------------------------------
    // HidD_SetNumInputBuffers controls how many input reports Windows queues
    // internally before they are overwritten. A small queue (2) prevents stale
    // reports accumulating between real-time ticks.
    if (!HidD_SetNumInputBuffers(g_Dev.hFile, 2))
    {
        ECM_Log("[ECM] HidD_SetNumInputBuffers failed: 0x%08X (non-fatal)\n",
                GetLastError());
        // Non-fatal — default buffer count will be used
    }

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] OpenECMUSB: OK — ECM-SK HID device opened successfully.\n");
    ECM_Log("[ECM]   Frame size : %u bytes (Report ID + %u payload)\n",
            (unsigned)ECM_HID_BUF_BYTES, (unsigned)ECM_FRAME_BYTES);
    return true;
}

// ============================================================================
//  CloseECMUSB
//  Closes the HID device handle, frees the shared OVERLAPPED events, and
//  resets internal state.
// ============================================================================
void __stdcall CloseECMUSB(void)
{
    if (!g_Dev.isOpen)
        return;

    ECM_CloseHandle();
    ECM_Log("[ECM] CloseECMUSB: device closed.\n");
}

// ============================================================================
//  ECMUSBWrite
//  Sends one full protocol frame to the ECM-SK via a HID Output report.
//
//  HID framing: the caller passes a raw protocol buffer of ECM_FRAME_BYTES.
//  This function prepends the 1-byte Report ID (0x00) into a stack-allocated
//  intermediate buffer and calls WriteFile. The caller's buffer is NOT
//  modified.
//
//  Overlapped I/O: WriteFile is issued with the shared g_Dev.hEventWrite
//  OVERLAPPED event (reset before the call). If the write does not complete
//  immediately, WaitForSingleObject enforces ECM_USB_TIMEOUT_MS before
//  giving up and cancelling. This mirrors the read path and ensures a stalled
//  interrupt-OUT endpoint cannot block the real-time thread.
//
//  dwLength must equal ECM_FRAME_BYTES exactly (sizeof(transData)*DEF_MA_MAX).
//  Returns true on success, false on failure (hold-last strategy applied by
//  the caller in mdlOutputs).
// ============================================================================
bool __stdcall ECMUSBWrite(const unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBWrite: invalid state or parameters.\n");
        return false;
    }

    if (dwLength != (unsigned long)ECM_FRAME_BYTES)
    {
        ECM_Log("[ECM] ECMUSBWrite: unexpected length %lu (expected %lu).\n",
                dwLength, (unsigned long)ECM_FRAME_BYTES);
        return false;
    }

    // Build the HID output report buffer: [Report ID | payload]
    // Stack allocation is safe — ECM_HID_BUF_BYTES is at most 513 bytes.
    unsigned char hidBuf[ECM_HID_BUF_BYTES];
    hidBuf[0] = ECM_HID_REPORT_ID;            // Report ID prefix
    memcpy(hidBuf + 1, data, ECM_FRAME_BYTES); // protocol payload

    // Reset the shared write event before issuing the transfer so that any
    // signal left over from a previous tick does not cause a false-completion.
    ResetEvent(g_Dev.hEventWrite);

    OVERLAPPED ov  = { 0 };
    ov.hEvent      = g_Dev.hEventWrite;

    // lpNumberOfBytesWritten must be NULL when using an overlapped handle —
    // passing a non-NULL pointer here can produce erroneous byte counts on
    // some Windows versions. Actual count is retrieved via GetOverlappedResult.
    DWORD bytesWritten = 0;
    BOOL  ok = WriteFile(g_Dev.hFile, hidBuf, (DWORD)ECM_HID_BUF_BYTES,
                         NULL, &ov);

    if (!ok)
    {
        DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING)
        {
            ECM_Log("[ECM] ECMUSBWrite: WriteFile failed immediately: 0x%08X\n", err);
            // Do NOT close or reset the handle — caller applies hold-last strategy.
            // ECMUSBRecover() is called at shutdown in mdlTerminate.
            return false;
        }

        // Wait up to ECM_USB_TIMEOUT_MS for the write to complete.
        // Using WaitForSingleObject + GetOverlappedResult(bWait=FALSE) rather
        // than GetOverlappedResult(bWait=TRUE) because the latter blocks
        // indefinitely on a manual-reset event if I/O never completes — which
        // would reintroduce the very stall this overlapped model was designed
        // to prevent.
        DWORD waitResult = WaitForSingleObject(g_Dev.hEventWrite, ECM_USB_TIMEOUT_MS);
        if (waitResult != WAIT_OBJECT_0)
        {
            // Cancel the pending write and drain its completion so the
            // OVERLAPPED and its event are safe to reuse next tick.
            CancelIoEx(g_Dev.hFile, &ov);
            GetOverlappedResult(g_Dev.hFile, &ov, &bytesWritten, TRUE); // drain
            if (waitResult == WAIT_TIMEOUT)
                ECM_Log("[ECM] ECMUSBWrite: timed out after %u ms — "
                        "device stalled or disconnected?\n", ECM_USB_TIMEOUT_MS);
            else
                ECM_Log("[ECM] ECMUSBWrite: WaitForSingleObject error: 0x%08X\n",
                        GetLastError());
            return false;
        }

        // I/O completed within the timeout — retrieve the byte count.
        if (!GetOverlappedResult(g_Dev.hFile, &ov, &bytesWritten, FALSE))
        {
            ECM_Log("[ECM] ECMUSBWrite: GetOverlappedResult failed: 0x%08X\n",
                    GetLastError());
            return false;
        }
    }
    else
    {
        // WriteFile completed synchronously (can happen even on an overlapped
        // handle when data is already buffered). Retrieve the byte count the
        // same way for consistency.
        GetOverlappedResult(g_Dev.hFile, &ov, &bytesWritten, FALSE);
    }

    if (bytesWritten != (DWORD)ECM_HID_BUF_BYTES)
    {
        ECM_Log("[ECM] ECMUSBWrite: short write — sent %lu / %lu bytes.\n",
                bytesWritten, (unsigned long)ECM_HID_BUF_BYTES);
        return false;
    }

    return true;
}

// ============================================================================
//  ECMUSBRead
//  Receives one full protocol frame from the ECM-SK via a HID Input report.
//
//  HID framing: ReadFile returns [Report ID (1 byte) | payload]. This function
//  validates the Report ID, then copies only the payload into the caller's
//  buffer of ECM_FRAME_BYTES. The Report ID byte is consumed internally.
//
//  Overlapped I/O: ReadFile is issued with the shared g_Dev.hEventRead
//  OVERLAPPED event (reset before the call). WaitForSingleObject enforces
//  ECM_USB_TIMEOUT_MS; on timeout CancelIoEx cancels the pending read and
//  drains its completion before returning false, so the event and OVERLAPPED
//  are safe to reuse on the next tick without kernel object reallocation.
//
//  dwLength must equal ECM_FRAME_BYTES exactly.
//  Returns true on success, false on timeout or failure.
// ============================================================================
bool __stdcall ECMUSBRead(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBRead: invalid state or parameters.\n");
        return false;
    }

    if (dwLength != (unsigned long)ECM_FRAME_BYTES)
    {
        ECM_Log("[ECM] ECMUSBRead: unexpected length %lu (expected %lu).\n",
                dwLength, (unsigned long)ECM_FRAME_BYTES);
        return false;
    }

    // Zero-initialise the destination buffer so stale data never leaks
    memset(data, 0, dwLength);

    // Read into an intermediate HID buffer that includes the Report ID byte
    unsigned char hidBuf[ECM_HID_BUF_BYTES];
    memset(hidBuf, 0, sizeof(hidBuf));

    // Reset the shared read event before issuing the transfer so that a
    // signal left over from a previous tick does not cause a false-completion.
    ResetEvent(g_Dev.hEventRead);

    OVERLAPPED ov = { 0 };
    ov.hEvent     = g_Dev.hEventRead;

    DWORD bytesRead = 0;
    BOOL  ok = ReadFile(g_Dev.hFile, hidBuf, (DWORD)ECM_HID_BUF_BYTES,
                        NULL, &ov); // lpNumberOfBytesRead = NULL on overlapped handle

    if (!ok)
    {
        DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING)
        {
            ECM_Log("[ECM] ECMUSBRead: ReadFile failed immediately: 0x%08X\n", err);
            return false;
        }

        // Wait up to ECM_USB_TIMEOUT_MS for the report to arrive
        DWORD waitResult = WaitForSingleObject(g_Dev.hEventRead, ECM_USB_TIMEOUT_MS);
        if (waitResult != WAIT_OBJECT_0)
        {
            // Cancel the pending read and drain its completion so the shared
            // event and OVERLAPPED are safe to reuse on the next tick.
            CancelIoEx(g_Dev.hFile, &ov);
            GetOverlappedResult(g_Dev.hFile, &ov, &bytesRead, TRUE); // drain
            if (waitResult == WAIT_TIMEOUT)
                ECM_Log("[ECM] ECMUSBRead: timed out after %u ms — "
                        "device stalled or disconnected?\n", ECM_USB_TIMEOUT_MS);
            else
                ECM_Log("[ECM] ECMUSBRead: WaitForSingleObject error: 0x%08X\n",
                        GetLastError());
            return false;
        }

        // I/O completed within the timeout — retrieve the byte count.
        if (!GetOverlappedResult(g_Dev.hFile, &ov, &bytesRead, FALSE))
        {
            ECM_Log("[ECM] ECMUSBRead: GetOverlappedResult failed: 0x%08X\n",
                    GetLastError());
            return false;
        }
    }
    else
    {
        // ReadFile completed synchronously (data was already queued in the
        // HID input buffer). Retrieve the byte count for the check below.
        GetOverlappedResult(g_Dev.hFile, &ov, &bytesRead, FALSE);
    }

    if (bytesRead != (DWORD)ECM_HID_BUF_BYTES)
    {
        ECM_Log("[ECM] ECMUSBRead: short read — got %lu / %lu bytes.\n",
                bytesRead, (unsigned long)ECM_HID_BUF_BYTES);
        return false;
    }

    // Validate Report ID — should always be ECM_HID_REPORT_ID (0x00)
    if (hidBuf[0] != ECM_HID_REPORT_ID)
    {
        ECM_Log("[ECM] ECMUSBRead: unexpected Report ID 0x%02X (expected 0x%02X).\n",
                (unsigned)hidBuf[0], (unsigned)ECM_HID_REPORT_ID);
        // Non-fatal for Report ID 0 devices — some firmware echoes 0x00
        // regardless. Copy payload anyway and let caller decide.
    }

    // Strip the Report ID byte — copy only the protocol payload to caller
    memcpy(data, hidBuf + 1, ECM_FRAME_BYTES);
    return true;
}

// ============================================================================
//  ECMUSBRecover
//  Closes the HID handle to flush any stalled I/O state, then reopens it
//  using the cached g_Dev.devicePath — bypassing re-enumeration to guarantee
//  the same physical device is targeted even when multiple ECM-SK modules
//  are connected simultaneously.
//  Intended for use in mdlTerminate (shutdown) only; CloseHandle + CreateFile
//  can take tens of milliseconds and must NOT be called from the RT loop.
//  The handle will be closed again by CloseECMUSB before mdlTerminate returns.
// ============================================================================
void __stdcall ECMUSBRecover(void)
{
    if (!g_Dev.isOpen)
        return;

    ECM_Log("[ECM] ECMUSBRecover: closing HID handle to flush stalled I/O...\n");

    // ECM_CloseHandle frees the file handle AND both OVERLAPPED events.
    // g_Dev.devicePath is preserved — it is intentionally not cleared by
    // ECM_CloseHandle so we can reopen here without re-enumerating.
    ECM_CloseHandle();

    // Attempt to reopen using the exact path from the original open — best
    // effort at shutdown, failure is non-fatal.
    g_Dev.hFile = CreateFileW(
        g_Dev.devicePath,
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,   // consistent with OpenECMUSB
        NULL);

    if (g_Dev.hFile == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM] ECMUSBRecover: reopen failed: 0x%08X "
                "(device disconnected?)\n", GetLastError());
        return;
    }

    // Reallocate the OVERLAPPED events for the reopened handle
    g_Dev.hEventRead  = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_Dev.hEventWrite = CreateEvent(NULL, TRUE, FALSE, NULL);

    if (g_Dev.hEventRead == NULL || g_Dev.hEventWrite == NULL)
    {
        ECM_Log("[ECM] ECMUSBRecover: CreateEvent failed: 0x%08X\n", GetLastError());
        if (g_Dev.hEventRead)  { CloseHandle(g_Dev.hEventRead);  g_Dev.hEventRead  = NULL; }
        if (g_Dev.hEventWrite) { CloseHandle(g_Dev.hEventWrite); g_Dev.hEventWrite = NULL; }
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return;
    }

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] ECMUSBRecover: handle reopened successfully.\n");
}

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

// ----------------------------------------------------------------------------
//  ECM_CloseHandle
//  Closes the raw file handle, frees both shared OVERLAPPED events, and
//  resets device state. Internal use only.
//  NOTE: g_Dev.devicePath is intentionally NOT cleared here so that
//  ECMUSBRecover can reopen the same device path after calling this function.
// ----------------------------------------------------------------------------
static void ECM_CloseHandle(void)
{
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }

    // Free the shared OVERLAPPED events
    if (g_Dev.hEventRead != NULL)
    {
        CloseHandle(g_Dev.hEventRead);
        g_Dev.hEventRead = NULL;
    }
    if (g_Dev.hEventWrite != NULL)
    {
        CloseHandle(g_Dev.hEventWrite);
        g_Dev.hEventWrite = NULL;
    }

    g_Dev.isOpen = FALSE;
    // g_Dev.devicePath intentionally preserved — ECMUSBRecover reads it after
    // calling ECM_CloseHandle to reopen the same physical device.
}

// ----------------------------------------------------------------------------
//  ECM_FindHidDevicePath
//  Enumerates all HID class devices using the HID GUID from HidD_GetHidGuid,
//  opens each one briefly to read its VID/PID via HidD_GetAttributes, and
//  returns the device path of the first device matching ECM_USB_VID/PID.
//
//  This is the correct approach for HID class devices — the generic WinUSB
//  GUID search used in earlier revisions of this code does not apply here.
//
//  If the matched device path is longer than pathBufLen, the function logs
//  a clear error and returns FALSE instead of silently truncating — a
//  truncated path would cause a confusing CreateFile failure downstream.
// ----------------------------------------------------------------------------
static BOOL ECM_FindHidDevicePath(WCHAR *pathBuf, DWORD pathBufLen)
{
    // Get the HID class GUID — always {4D1E55B2-F16F-11CF-88CB-001111000030}
    // but calling HidD_GetHidGuid is the correct portable way to obtain it.
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);

    // Enumerate all present HID device interfaces
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        &hidGuid, NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hDevInfo == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM] SetupDiGetClassDevsW (HID) failed: 0x%08X\n", GetLastError());
        return FALSE;
    }

    SP_DEVICE_INTERFACE_DATA ifData = { 0 };
    ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    BOOL found = FALSE;

    for (DWORD idx = 0; !found; idx++)
    {
        // Enumerate the next HID interface
        if (!SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &hidGuid, idx, &ifData))
        {
            if (GetLastError() == ERROR_NO_MORE_ITEMS)
                break;  // exhausted all HID devices
            continue;   // skip devices with enumeration errors
        }

        // Query the required buffer size for the detail struct
        DWORD requiredSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &requiredSize, NULL);
        if (requiredSize == 0)
            continue;

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredSize);
        if (!pDetail)
            break;

        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        BOOL detailOk = SetupDiGetDeviceInterfaceDetailW(
                            hDevInfo, &ifData,
                            pDetail, requiredSize, NULL, NULL);
        if (!detailOk)
        {
            HeapFree(GetProcessHeap(), 0, pDetail);
            continue;
        }

        // Briefly open the device to read its VID/PID via HidD_GetAttributes.
        // Use FILE_SHARE_READ | FILE_SHARE_WRITE so we don't block other
        // processes while probing.
        HANDLE hProbe = CreateFileW(
            pDetail->DevicePath,
            0,                  // no read/write access needed just to query attrs
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            NULL, OPEN_EXISTING, 0, NULL);

        if (hProbe != INVALID_HANDLE_VALUE)
        {
            HIDD_ATTRIBUTES attrs = { 0 };
            attrs.Size = sizeof(HIDD_ATTRIBUTES);

            if (HidD_GetAttributes(hProbe, &attrs))
            {
                ECM_Log("[ECM] HID probe: VID=%04X PID=%04X path=%ls\n",
                        (unsigned)attrs.VendorID, (unsigned)attrs.ProductID,
                        pDetail->DevicePath);

                if (attrs.VendorID  == (USHORT)ECM_USB_VID &&
                    attrs.ProductID == (USHORT)ECM_USB_PID)
                {
                    // Verify the path fits in the caller's buffer before
                    // copying.  If not, log a clear diagnostic and abort
                    // rather than silently truncating — a truncated path
                    // passed to CreateFileW produces a confusing downstream
                    // failure rather than a clear "path too long" error.
                    size_t pathLen = wcslen(pDetail->DevicePath) + 1; // +1 for NUL
                    if (pathLen > (size_t)pathBufLen)
                    {
                        ECM_Log("[ECM] ECM_FindHidDevicePath: device path too long "
                                "(%zu WCHARs, buffer is %lu) — aborting.\n",
                                pathLen, pathBufLen);
                        CloseHandle(hProbe);
                        HeapFree(GetProcessHeap(), 0, pDetail);
                        break;  // return FALSE — let caller handle it
                    }
                    wcsncpy_s(pathBuf, pathBufLen,
                              pDetail->DevicePath, _TRUNCATE);
                    found = TRUE;
                    ECM_Log("[ECM] ECM-SK found: VID_%04X&PID_%04X\n",
                            ECM_USB_VID, ECM_USB_PID);
                }
            }
            CloseHandle(hProbe);
        }

        HeapFree(GetProcessHeap(), 0, pDetail);
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);

    if (!found)
        ECM_Log("[ECM] ECM_FindHidDevicePath: no device matched "
                "VID_%04X&PID_%04X.\n", ECM_USB_VID, ECM_USB_PID);

    return found;
}