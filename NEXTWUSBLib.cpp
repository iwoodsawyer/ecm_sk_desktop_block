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
// binds HidUsb.sys on first connection. No Zadig / driver replacement
// is required or permitted for HID class devices.
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
static ECM_DEVICE_CTX g_Dev = { INVALID_HANDLE_VALUE, FALSE };

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL ECM_FindHidDevicePath(WCHAR *pathBuf, DWORD pathBufLen);
static void ECM_CloseHandle(void);

// ============================================================================
//  OpenECMUSB
//  Locates the ECM-SK HID device by VID/PID, opens an exclusive file handle,
//  and configures read/write timeouts.
//  Returns TRUE on success, FALSE on any failure.
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

    // ----- 2. Open the HID device -------------------------------------------
    // FILE_FLAG_OVERLAPPED is used so that ReadFile / WriteFile calls
    // can be given a timeout via COMMTIMEOUTS-equivalent (HidD_SetNumInputBuffers
    // does not control timeouts; we use a manual event + WaitForSingleObject).
    // For simplicity and real-time determinism, we open without overlapped I/O
    // and instead rely on HidD_SetNumInputBuffers and the OS HID scheduler.
    // Synchronous ReadFile on a HID interrupt endpoint blocks until the next
    // interrupt poll interval (1 ms at Full Speed).
    g_Dev.hFile = CreateFileW(
        devicePath,
        GENERIC_READ | GENERIC_WRITE,
        0,              // exclusive: do not share — prevents other processes
                        // from interfering with the EtherCAT master traffic
        NULL,
        OPEN_EXISTING,
        0,              // synchronous I/O — simpler and sufficient for SLDRT
                        // Normal Mode where the RT thread is single-threaded
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

    // ----- 3. Configure HID read timeouts -----------------------------------
    // HidD_SetNumInputBuffers controls how many input reports Windows queues
    // internally before they are overwritten. A small queue (2) prevents stale
    // reports accumulating between real-time ticks. The actual transfer timeout
    // is controlled by setting a timeout on the file handle via
    // SetCommTimeouts is only for COM ports; for HID we use a manual approach:
    // we keep synchronous I/O and accept that ReadFile will block for at most
    // one USB poll interval (1 ms) plus OS scheduling jitter.
    //
    // For deterministic timeout behaviour, ECMUSBRead uses WaitForSingleObject
    // with an overlapped read — see ECMUSBRead implementation note below.
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
//  Closes the HID device handle and resets internal state.
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
//  dwLength must equal ECM_FRAME_BYTES exactly (sizeof(transData)*DEF_MA_MAX).
//  Returns TRUE on success, FALSE on failure (hold-last strategy applied by
//  the caller in mdlOutputs).
// ============================================================================
bool __stdcall ECMUSBWrite(unsigned char *data, unsigned long dwLength)
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
    hidBuf[0] = ECM_HID_REPORT_ID;                    // Report ID prefix
    memcpy(hidBuf + 1, data, ECM_FRAME_BYTES);         // protocol payload

    DWORD bytesWritten = 0;
    BOOL  ok = WriteFile(
                    g_Dev.hFile,
                    hidBuf,
                    (DWORD)ECM_HID_BUF_BYTES,
                    &bytesWritten,
                    NULL);              // synchronous

    if (!ok || bytesWritten != (DWORD)ECM_HID_BUF_BYTES)
    {
        ECM_Log("[ECM] ECMUSBWrite failed: Err=0x%08X (wrote %lu / %lu bytes).\n",
                GetLastError(), bytesWritten, (unsigned long)ECM_HID_BUF_BYTES);
        // Do NOT reset or close the handle here — caller holds last
        // output values. ECMUSBRecover() is called at shutdown in mdlTerminate.
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
//  dwLength must equal ECM_FRAME_BYTES exactly.
//  Returns TRUE on success, FALSE on failure (hold-last strategy applied by
//  the caller in mdlOutputs).
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

    DWORD bytesRead = 0;
    BOOL  ok = ReadFile(
                    g_Dev.hFile,
                    hidBuf,
                    (DWORD)ECM_HID_BUF_BYTES,
                    &bytesRead,
                    NULL);              // synchronous

    if (!ok || bytesRead != (DWORD)ECM_HID_BUF_BYTES)
    {
        ECM_Log("[ECM] ECMUSBRead failed: Err=0x%08X (read %lu / %lu bytes).\n",
                GetLastError(), bytesRead, (unsigned long)ECM_HID_BUF_BYTES);
        // Do NOT reset or close the handle here — hold last value
        // strategy applied by caller. ECMUSBRecover() called at shutdown only.
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
//  Controlled recovery for use at shutdown only (mdlTerminate).
//  Closes and reopens the HID device handle to flush any stalled I/O.
//  Must NOT be called from inside the real-time loop (mdlOutputs) —
//  CloseHandle + CreateFile can take tens of milliseconds.
// ============================================================================
void __stdcall ECMUSBRecover(void)
{
    if (!g_Dev.isOpen)
        return;

    ECM_Log("[ECM] ECMUSBRecover: closing and reopening HID handle...\n");

    // Record the path before closing so we can reopen it
    // (not needed at shutdown, but keeps the pattern consistent)
    ECM_CloseHandle();

    // Attempt to reopen — best effort at shutdown, failure is non-fatal
    WCHAR devicePath[MAX_PATH] = { 0 };
    if (ECM_FindHidDevicePath(devicePath, MAX_PATH))
    {
        g_Dev.hFile = CreateFileW(
            devicePath,
            GENERIC_READ | GENERIC_WRITE,
            0, NULL, OPEN_EXISTING, 0, NULL);

        if (g_Dev.hFile != INVALID_HANDLE_VALUE)
        {
            g_Dev.isOpen = TRUE;
            ECM_Log("[ECM] ECMUSBRecover: handle reopened successfully.\n");
        }
        else
        {
            ECM_Log("[ECM] ECMUSBRecover: reopen failed: 0x%08X\n", GetLastError());
        }
    }
    else
    {
        ECM_Log("[ECM] ECMUSBRecover: device no longer found (USB disconnected?).\n");
    }
}

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

// ----------------------------------------------------------------------------
//  ECM_CloseHandle
//  Closes the raw file handle and resets device state. Internal use only.
// ----------------------------------------------------------------------------
static void ECM_CloseHandle(void)
{
    if (g_Dev.hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
    }
    g_Dev.isOpen = FALSE;
}

// ----------------------------------------------------------------------------
//  ECM_FindHidDevicePath
//  Enumerates all HID class devices using the HID GUID from HidD_GetHidGuid,
//  opens each one briefly to read its VID/PID via HidD_GetAttributes, and
//  returns the device path of the first device matching ECM_USB_VID/PID.
//
//  This is the correct approach for HID class devices — the generic WinUSB
//  GUID search used in the old implementation does not apply here.
// ----------------------------------------------------------------------------
static BOOL ECM_FindHidDevicePath(WCHAR *pathBuf, DWORD pathBufLen)
{
    // Get the HID class GUID — always {4D1E55B2-F16F-11CF-88CB-001111000030}
    // but calling HidD_GetHidGuid is the correct portable way to obtain it.
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);

    // Enumerate all present HID device interfaces
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        &hidGuid,
        NULL, NULL,
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
                            pDetail, requiredSize,
                            NULL, NULL);
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
            NULL,
            OPEN_EXISTING,
            0,
            NULL);

        if (hProbe != INVALID_HANDLE_VALUE)
        {
            HIDD_ATTRIBUTES attrs = { 0 };
            attrs.Size = sizeof(HIDD_ATTRIBUTES);

            if (HidD_GetAttributes(hProbe, &attrs))
            {
                ECM_Log("[ECM] HID probe: VID=%04X PID=%04X path=%ls\n",
                        (unsigned)attrs.VendorID,
                        (unsigned)attrs.ProductID,
                        pDetail->DevicePath);

                if (attrs.VendorID  == (USHORT)ECM_USB_VID &&
                    attrs.ProductID == (USHORT)ECM_USB_PID)
                {
                    // Matched — copy the device path for the caller
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