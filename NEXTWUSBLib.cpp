// ============================================================================
// NEXTWUSBLib.cpp
// Windows API implementation of the NEXTWUSBLib DLL.
// Interfaces with the USB EtherCAT EC-01M Master Module via WinUSB.
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
// Module-level state
// ============================================================================
static ECM_DEVICE_CTX g_Dev = { INVALID_HANDLE_VALUE, NULL, 0, 0, FALSE };

// ============================================================================
// Forward declarations of internal helpers
// ============================================================================
static BOOL  ECM_FindDevicePath (WCHAR *pathBuf, DWORD pathBufLen);
static BOOL  ECM_FindDevicePathByVidPid(WCHAR *pathBuf, DWORD pathBufLen);
static BOOL  ECM_QueryPipes     (void);
static void  ECM_ResetDevice    (void);

// ============================================================================
//  OpenECMUSB
//  Opens the WinUSB device, queries its endpoints, and sets transfer policy.
//  Returns TRUE on success, FALSE on any failure.
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

    // First attempt: match via the Device Interface GUID (preferred when INF
    // registers the WinUSB class with a known GUID).
    if (!ECM_FindDevicePath(devicePath, MAX_PATH))
    {
        // Fallback: scan all WinUSB-class devices and match VID/PID by
        // hardware-ID string comparison.
        ECM_Log("[ECM] GUID search failed; trying VID/PID fallback.\n");
        if (!ECM_FindDevicePathByVidPid(devicePath, MAX_PATH))
        {
            ECM_Log("[ECM] OpenECMUSB: device not found.\n");
            return false;
        }
    }
    ECM_Log("[ECM] Device path: %ls\n", devicePath);

    // ----- 2. Open a file handle to the device --------------------------------
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
        ECM_Log("[ECM] CreateFile failed: 0x%08X\n", GetLastError());
        return false;
    }

    // ----- 3. Initialise WinUSB -----------------------------------------------
    if (!WinUsb_Initialize(g_Dev.hFile, &g_Dev.hWinUsb))
    {
        ECM_Log("[ECM] WinUsb_Initialize failed: 0x%08X\n", GetLastError());
        CloseHandle(g_Dev.hFile);
        g_Dev.hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    // ----- 4. Discover bulk pipe addresses ------------------------------------
    if (!ECM_QueryPipes())
    {
        // Fall back to compile-time defaults if pipe discovery fails
        ECM_Log("[ECM] Pipe discovery failed; using default endpoints 0x%02X / 0x%02X.\n",
                ECM_PIPE_OUT_DEFAULT, ECM_PIPE_IN_DEFAULT);
        g_Dev.pipeOut = ECM_PIPE_OUT_DEFAULT;
        g_Dev.pipeIn  = ECM_PIPE_IN_DEFAULT;
    }

    // ----- 5. Configure transfer policies ------------------------------------
    // Set per-pipe timeouts
    ULONG timeout = ECM_USB_TIMEOUT_MS;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                         PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                         PIPE_TRANSFER_TIMEOUT, sizeof(ULONG), &timeout);

    // Enable auto-flush on OUT pipe (send ZLP when transfer is a multiple of
    // the max-packet-size — important for bulk endpoints)
    UCHAR autoFlush = TRUE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeOut,
                         AUTO_FLUSH, sizeof(UCHAR), &autoFlush);

    // Allow partial reads on IN pipe so short transfers don't time out
    UCHAR allowPartial = TRUE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                         ALLOW_PARTIAL_READS, sizeof(UCHAR), &allowPartial);

    // Let WinUSB signal a short packet so the app can detect it
    UCHAR ignoreShort = FALSE;
    WinUsb_SetPipePolicy(g_Dev.hWinUsb, g_Dev.pipeIn,
                     IGNORE_SHORT_PACKETS, sizeof(UCHAR), &ignoreShort);

    g_Dev.isOpen = TRUE;
    ECM_Log("[ECM] OpenECMUSB: OK (OUT=0x%02X, IN=0x%02X).\n",
            g_Dev.pipeOut, g_Dev.pipeIn);
    return true;
}

// ============================================================================
//  CloseECMUSB
//  Releases WinUSB resources and closes the device handle.
// ============================================================================
void __stdcall CloseECMUSB(void)
{
    if (!g_Dev.isOpen)
        return;

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

    g_Dev.pipeOut = 0;
    g_Dev.pipeIn  = 0;
    g_Dev.isOpen  = FALSE;
    ECM_Log("[ECM] CloseECMUSB: device closed.\n");
}

// ============================================================================
//  ECMUSBWrite
//  Writes `dwLength` bytes from `data` to the EC-01M bulk-OUT endpoint.
//  Returns TRUE on success.
// ============================================================================
bool __stdcall ECMUSBWrite(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBWrite: invalid state or parameters.\n");
        return false;
    }

    ULONG bytesWritten = 0;
    BOOL  ok = WinUsb_WritePipe(
                    g_Dev.hWinUsb,
                    g_Dev.pipeOut,
                    data,
                    (ULONG)dwLength,
                    &bytesWritten,
                    NULL);          // NULL = synchronous (overlapped handle from CreateFile
                                    //        is used internally by WinUSB)

    // Exact length enforcement
    if (!ok || bytesWritten != (ULONG)dwLength)
    {
        ECM_Log("[ECM] ECMUSBWrite failed: Err=0x%08X (wrote %lu / %lu bytes).\n",
                GetLastError(), bytesWritten, dwLength);
        
        // Do NOT reset pipes here — hold last value strategy applied
        // by the caller; ECMUSBRecover() is called at shutdown only.
        return false;
    }

    return true;
}

// ============================================================================
//  ECMUSBRead
//  Reads `dwLength` bytes into `data` from the EC-01M bulk-IN endpoint.
//  Returns TRUE on success.
// ============================================================================
bool __stdcall ECMUSBRead(unsigned char *data, unsigned long dwLength)
{
    if (!g_Dev.isOpen || data == NULL || dwLength == 0)
    {
        ECM_Log("[ECM] ECMUSBRead: invalid state or parameters.\n");
        return false;
    }

    // Zero-initialise the destination buffer so stale data never leaks
    memset(data, 0, dwLength);

    ULONG bytesRead = 0;
    BOOL  ok = WinUsb_ReadPipe(
                    g_Dev.hWinUsb,
                    g_Dev.pipeIn,
                    data,
                    (ULONG)dwLength,
                    &bytesRead,
                    NULL);

    // Enforce full-frame reads. Returning success on a short read 
    // breaks the fixed-size structure assumptions of the protocol.
    if (!ok || bytesRead != (ULONG)dwLength)
    {
        ECM_Log("[ECM] ECMUSBRead failed: Err=0x%08X (read %lu / %lu bytes).\n",
                GetLastError(), bytesRead, dwLength);

        // Do NOT reset pipes here — hold last value strategy applied
        // by the caller; ECMUSBRecover() is called at shutdown only.
        return false;
    }

    // ECM_Log("[ECM] ECMUSBRead: %lu bytes received.\n", bytesRead);
    return true;
}

// ----------------------------------------------------------------------------
//  ECM_ResetDevice
// ----------------------------------------------------------------------------
static void ECM_ResetDevice(void)
{
    if (!g_Dev.isOpen) return;

    // Correct USB error recovery sequence: Abort pending I/O, then Reset to clear stall
    WinUsb_AbortPipe(g_Dev.hWinUsb, g_Dev.pipeOut);
    WinUsb_AbortPipe(g_Dev.hWinUsb, g_Dev.pipeIn);
    
    WinUsb_ResetPipe(g_Dev.hWinUsb, g_Dev.pipeOut);
    WinUsb_ResetPipe(g_Dev.hWinUsb, g_Dev.pipeIn);
    
    ECM_Log("[ECM] Device pipes aborted and reset.\n");
}

// ============================================================================
//  ECMUSBRecover
//  Public wrapper around ECM_ResetDevice for use at controlled shutdown only
//  (i.e. mdlTerminate). Must NOT be called from inside the real-time loop
//  (mdlOutputs) as WinUsb_AbortPipe / WinUsb_ResetPipe can take several
//   milliseconds and will cause a real-time overrun.
// ============================================================================
void __stdcall ECMUSBRecover(void)
{
    ECM_ResetDevice();
}

// ============================================================================
// ============================================================================
//  INTERNAL HELPERS
// ============================================================================
// ============================================================================

// ----------------------------------------------------------------------------
//  ECM_FindDevicePath
//  Uses SetupAPI to enumerate device interfaces registered with
//  ECM_DEVICE_INTERFACE_GUID and returns the path of the first match.
// ----------------------------------------------------------------------------
static BOOL ECM_FindDevicePath(WCHAR *pathBuf, DWORD pathBufLen)
{
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        &ECM_DEVICE_INTERFACE_GUID,
        NULL, NULL,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hDevInfo == INVALID_HANDLE_VALUE)
    {
        ECM_Log("[ECM] SetupDiGetClassDevsW failed: 0x%08X\n", GetLastError());
        return FALSE;
    }

    SP_DEVICE_INTERFACE_DATA ifData = { 0 };
    ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    BOOL found = FALSE;
    for (DWORD idx = 0; ; idx++)
    {
        if (!SetupDiEnumDeviceInterfaces(hDevInfo, NULL,
                                         &ECM_DEVICE_INTERFACE_GUID,
                                         idx, &ifData))
        {
            if (GetLastError() == ERROR_NO_MORE_ITEMS)
                break;
            continue;
        }

        // Query required buffer size
        DWORD requiredSize = 0;
        SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                         NULL, 0, &requiredSize, NULL);

        PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
            (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                GetProcessHeap(), HEAP_ZERO_MEMORY, requiredSize);
        if (!pDetail)
            break;

        pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                              pDetail, requiredSize,
                                              NULL, NULL))
        {
            wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
            found = TRUE;
        }
        HeapFree(GetProcessHeap(), 0, pDetail);

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return found;
}

// ----------------------------------------------------------------------------
//  ECM_FindDevicePathByVidPid
//  Fallback: enumerate all USB devices and find one whose hardware ID
//  contains the expected VID and PID strings.
// ----------------------------------------------------------------------------
static BOOL ECM_FindDevicePathByVidPid(WCHAR *pathBuf, DWORD pathBufLen)
{
    // Build the search substring, e.g. L"VID_04B4&PID_00F1"
    WCHAR vidPidStr[64];
    swprintf_s(vidPidStr, 64, L"VID_%04X&PID_%04X", ECM_USB_VID, ECM_USB_PID);

    // Enumerate all present device interfaces in the USB class
    // Use the WinUSB class GUID {88BAE032-5A81-49f0-BC3D-A4FF138216D6}
    static const GUID GUID_DEVCLASS_WINUSB = {
        0x88BAE032, 0x5A81, 0x49F0,
        { 0xBC, 0x3D, 0xA4, 0xFF, 0x13, 0x82, 0x16, 0xD6 }
    };

    HDEVINFO hDevInfo = SetupDiGetClassDevsW(
        NULL, L"USB", NULL,
        DIGCF_PRESENT | DIGCF_ALLCLASSES | DIGCF_DEVICEINTERFACE);

    if (hDevInfo == INVALID_HANDLE_VALUE)
        return FALSE;

    SP_DEVINFO_DATA devData = { 0 };
    devData.cbSize = sizeof(SP_DEVINFO_DATA);

    BOOL found = FALSE;
    for (DWORD idx = 0; SetupDiEnumDeviceInfo(hDevInfo, idx, &devData); idx++)
    {
        WCHAR hwId[512] = { 0 };
        if (!SetupDiGetDeviceRegistryPropertyW(
                hDevInfo, &devData, SPDRP_HARDWAREID,
                NULL, (PBYTE)hwId, sizeof(hwId), NULL))
            continue;

        // Case-insensitive substring match for VIDxxxx&PIDxxxx
        WCHAR hwIdUpper[512];
        wcsncpy_s(hwIdUpper, 512, hwId, _TRUNCATE);
        _wcsupr_s(hwIdUpper, 512);

        WCHAR searchUpper[64];
        wcsncpy_s(searchUpper, 64, vidPidStr, _TRUNCATE);
        _wcsupr_s(searchUpper, 64);

        if (wcsstr(hwIdUpper, searchUpper) == NULL)
            continue;

        // Found the matching device; now find its interface path
        // Re-enumerate as a device interface so we can get DevicePath
        // Try the generic WinUSB GUID first, then fall back to the USB GUID
        const GUID *guids[] = { &ECM_DEVICE_INTERFACE_GUID, &GUID_DEVCLASS_WINUSB };
        for (int g = 0; g < 2 && !found; g++)
        {
            SP_DEVICE_INTERFACE_DATA ifData = { 0 };
            ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

            if (!SetupDiEnumDeviceInterfaces(hDevInfo, &devData,
                                             guids[g], 0, &ifData))
                continue;

            DWORD requiredSize = 0;
            SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                             NULL, 0, &requiredSize, NULL);

            PSP_DEVICE_INTERFACE_DETAIL_DATA_W pDetail =
                (PSP_DEVICE_INTERFACE_DETAIL_DATA_W)HeapAlloc(
                    GetProcessHeap(), HEAP_ZERO_MEMORY, requiredSize);
            if (!pDetail)
                break;

            pDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
            if (SetupDiGetDeviceInterfaceDetailW(hDevInfo, &ifData,
                                                  pDetail, requiredSize,
                                                  NULL, NULL))
            {
                wcsncpy_s(pathBuf, pathBufLen, pDetail->DevicePath, _TRUNCATE);
                found = TRUE;
            }
            HeapFree(GetProcessHeap(), 0, pDetail);
        }

        if (found)
            break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return found;
}

// ----------------------------------------------------------------------------
//  ECM_QueryPipes
//  Queries all alternate settings / endpoints on interface 0 and
//  locates the first Bulk-OUT and Bulk-IN pipes.
// ----------------------------------------------------------------------------
static BOOL ECM_QueryPipes(void)
{
    USB_INTERFACE_DESCRIPTOR ifDesc = { 0 };
    if (!WinUsb_QueryInterfaceSettings(g_Dev.hWinUsb, 0, &ifDesc))
    {
        ECM_Log("[ECM] WinUsb_QueryInterfaceSettings failed: 0x%08X\n",
                GetLastError());
        return FALSE;
    }

    ECM_Log("[ECM] Interface: %d endpoints.\n", (int)ifDesc.bNumEndpoints);

    BOOL foundOut = FALSE, foundIn = FALSE;

    for (UCHAR i = 0; i < ifDesc.bNumEndpoints; i++)
    {
        WINUSB_PIPE_INFORMATION pipeInfo = { 0 };
        if (!WinUsb_QueryPipe(g_Dev.hWinUsb, 0, i, &pipeInfo))
        {
            ECM_Log("[ECM]   QueryPipe[%d] failed: 0x%08X\n", (int)i,
                    GetLastError());
            continue;
        }

        ECM_Log("[ECM]   Pipe[%d]: type=%d addr=0x%02X maxPkt=%u\n",
                (int)i,
                (int)pipeInfo.PipeType,
                (unsigned)pipeInfo.PipeId,
                (unsigned)pipeInfo.MaximumPacketSize);

        if (pipeInfo.PipeType == UsbdPipeTypeBulk)
        {
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

    return (foundOut && foundIn);
}
