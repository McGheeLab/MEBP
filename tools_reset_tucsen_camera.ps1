# tools_reset_tucsen_camera.ps1
#
# Re-enumerate the Tucsen camera on USB - the software equivalent of unplugging
# and replugging it.
#
# WHY THIS EXISTS
# ---------------
# The Tucsen camera can end up present and healthy to Windows (Status OK,
# Problem CM_PROB_NONE, driver TUUSB3 bound) while TUCAM_Api_Init still reports
# "0 camera(s)". Measured on ME3B V1 from the application's own log, timed
# against the "library loaded" line beside each call:
#
#     healthy    ~2.1 s  -> 1 camera
#     wedged    ~29.1 s  -> 0 cameras
#
# It gets into that state after a stop/start of the camera - not every time,
# which is what makes it confusing. Nothing in software can claim it again; the
# USB device has to re-enumerate. This script does that without anyone having to
# reach behind the rig.
#
# USAGE
#   Right-click -> "Run with PowerShell" as Administrator, or:
#     powershell -ExecutionPolicy Bypass -File tools_reset_tucsen_camera.ps1
#
# Requires elevation (PnP device control does). It refuses politely rather than
# failing halfway, matching tools_install_tucsen_sdk.ps1.
#
# NOTE: this file is deliberately pure ASCII. PowerShell 5.1 reads a BOM-less
# .ps1 as ANSI, and a UTF-8 dash decodes as a byte that terminates a string and
# breaks the parse - which cost a session once already.

[CmdletBinding()]
param(
    [switch]$SkipVerify
)

$ErrorActionPreference = 'Stop'

$TUCSEN_VID = 'VID_5453'

function Test-Elevated {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    $pr = New-Object Security.Principal.WindowsPrincipal($id)
    return $pr.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

Write-Host ""
Write-Host "Tucsen camera reset" -ForegroundColor Cyan
Write-Host "-------------------"

if (-not (Test-Elevated)) {
    Write-Host ""
    Write-Host "This needs Administrator rights (it disables and re-enables a USB device)." -ForegroundColor Yellow
    Write-Host "Right-click PowerShell -> Run as administrator, then run:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "    powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`"" -ForegroundColor White
    Write-Host ""
    Write-Host "Or just unplug the camera's USB cable, wait two seconds, and plug it back in." -ForegroundColor Yellow
    Write-Host "That does exactly the same thing." -ForegroundColor Yellow
    exit 1
}

# Close-the-app check. The SDK cannot re-claim a device while a process still
# holds TUCam.dll, so resetting underneath a running MEBP just wastes the reset.
$mebp = Get-CimInstance Win32_Process -Filter "Name like 'python%'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like '*main.py*' }
if ($mebp) {
    Write-Host ""
    Write-Host "MEBP still appears to be running (PID $($mebp.ProcessId))." -ForegroundColor Yellow
    Write-Host "Close it first - the reset will not stick while it holds the SDK." -ForegroundColor Yellow
    exit 1
}

$devs = @(Get-PnpDevice -ErrorAction SilentlyContinue |
          Where-Object { $_.InstanceId -like "*$TUCSEN_VID*" })
if ($devs.Count -eq 0) {
    Write-Host "No Tucsen device found on USB ($TUCSEN_VID)." -ForegroundColor Red
    Write-Host "Check the cable and the camera's power." -ForegroundColor Red
    exit 2
}

foreach ($d in $devs) {
    Write-Host ""
    Write-Host ("Device : {0}" -f $d.FriendlyName)
    Write-Host ("Id     : {0}" -f $d.InstanceId)
    Write-Host ("Status : {0}" -f $d.Status)

    try {
        $arr = (Get-PnpDeviceProperty -InstanceId $d.InstanceId `
                -KeyName 'DEVPKEY_Device_LastArrivalDate' -ErrorAction Stop).Data
        Write-Host ("Last enumerated: {0}" -f $arr)
    } catch { }

    Write-Host ""
    Write-Host "Disabling..." -NoNewline
    Disable-PnpDevice -InstanceId $d.InstanceId -Confirm:$false
    Write-Host " ok"
    Start-Sleep -Seconds 3
    Write-Host "Enabling..." -NoNewline
    Enable-PnpDevice -InstanceId $d.InstanceId -Confirm:$false
    Write-Host " ok"
    Start-Sleep -Seconds 3

    $after = Get-PnpDevice -InstanceId $d.InstanceId
    Write-Host ("Status now: {0}" -f $after.Status)
    try {
        $arr2 = (Get-PnpDeviceProperty -InstanceId $d.InstanceId `
                 -KeyName 'DEVPKEY_Device_LastArrivalDate' -ErrorAction Stop).Data
        Write-Host ("Last enumerated: {0}   <- this must have just changed" -f $arr2)
    } catch { }
}

if ($SkipVerify) { exit 0 }

# Ask the SDK itself, which is the only answer that matters: Windows reporting
# "OK" is exactly what it reported while the camera was unusable.
Write-Host ""
Write-Host "Asking the SDK how many cameras it can see..." -ForegroundColor Cyan
$repo = Split-Path -Parent $PSCommandPath
$py = @'
import sys, time
sys.path.insert(0, sys.argv[1])
from gui.widgets.tucam_backend import TUCamBackend
t0 = time.time()
be = TUCamBackend()
ok = be.open("0")
dt = time.time() - t0
be.release()
print("open=%s in %.1fs" % (ok, dt))
print("RESULT: %s" % ("camera is back" if ok else "still not claimable"))
if not ok:
    print("A healthy init answers in about 2s; ~29s means it is still wedged.")
    print("Try a physical unplug/replug, or a different USB3 port.")
sys.exit(0 if ok else 3)
'@
$tmp = Join-Path $env:TEMP "tucsen_verify.py"
Set-Content -Path $tmp -Value $py -Encoding utf8
try {
    & python $tmp $repo
    $code = $LASTEXITCODE
} catch {
    Write-Host "Could not run the SDK check (python not on PATH?): $_" -ForegroundColor Yellow
    $code = 0
} finally {
    Remove-Item $tmp -ErrorAction SilentlyContinue
}
exit $code
