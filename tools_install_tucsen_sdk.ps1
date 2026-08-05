<#
.SYNOPSIS
    Install the Tucsen camera USB driver (+ optionally the TUCam application)
    so MEBP can drive a Tucsen Libra / Dhyana / Aries / FL camera.

.DESCRIPTION
    One-time per-machine setup. The MEBP 'tucam' backend talks to TUCam.dll,
    which is ALREADY VENDORED in this repo at "DLLs\tucsen dlls\TUCam.dll" -
    so the app does not need the TUCam application at all. What it does need is
    the USB DRIVER, without which the camera never enumerates and the SDK
    reports 0 cameras no matter how correct the software is.

    Steps, each verified:

      1. Installs "Tucam Camera Driver" (validly signed by FUZHOU TUCSEN
         PHOTONICS CO., LTD.) - this is the part that makes the camera appear.
      2. Optionally installs the TUCam application (~360 MB), which gives a
         vendor reference viewer for sanity-checking the camera outside MEBP,
         and a second copy of the SDK under Program Files.
      3. Verifies: driver present, vendored DLL loads, and the SDK reports how
         many cameras it can actually see.

    MUST BE RUN AS ADMINISTRATOR (per-machine driver install).

    Both installers live in the vendor bundle the camera shipped with:
      "Driver ...\Tucam Camera Driver V2.1.6.1.exe"
      "Software...\TUCam_FV2.0.8.0_PV2.0.124.0 Setup.zip"
    (the vendor's folder names contain non-ASCII text; both installers are
    located by glob, so the exact folder name does not matter.)

.PARAMETER Bundle
    Path to the vendor zip (e.g. E:\Dhyana&FL&Libra16-18-22-25&Aries16_20251120.zip)
    OR to an already-extracted folder containing the two installers above.

.PARAMETER DriverExe
    Path to an already-extracted "Tucam Camera Driver*.exe" (skips -Bundle).

.PARAMETER WithApp
    Also install the full TUCam application (~360 MB). Off by default: the app
    is NOT required for MEBP, only useful as a vendor reference viewer.

.PARAMETER SkipDriver
    Skip the driver install (verification only).

.EXAMPLE
    # Right-click PowerShell -> Run as administrator (it opens in system32, so
    # pass full paths), driver only:
    powershell -ExecutionPolicy Bypass -File C:\dev\MEBP\tools_install_tucsen_sdk.ps1 `
        -Bundle "E:\Dhyana&FL&Libra16-18-22-25&Aries16_20251120.zip"

.EXAMPLE
    # Driver + the vendor TUCam viewer:
    powershell -ExecutionPolicy Bypass -File C:\dev\MEBP\tools_install_tucsen_sdk.ps1 `
        -Bundle "E:\Dhyana&FL&Libra16-18-22-25&Aries16_20251120.zip" -WithApp

.NOTES
    Signatures, checked before this script was written:
      * Tucam Camera Driver V2.1.6.1.exe  - Authenticode VALID,
        CN="FUZHOU TUCSEN PHOTONICS CO., LTD."   (SHA256
        9131826137DE175870AFB84F145F74B83E536BAA14F836635D8793B1EAA4964B)
      * TUCam.dll v2.0.8.0 (Xintu Photonics) - UNSIGNED. That is fine: it is a
        user-mode DLL, where signing is not enforced. Only a KERNEL driver
        signature is enforced, and the driver above is validly signed with a
        current certificate.

    !! Contrast with this repo's Nikon Ti experience: that body's driver was a
    2012 SHA-1 kernel driver which Windows 11 refused to load under HVCI
    (Memory Integrity), needing Core Isolation turned off. The Tucsen driver is
    currently signed, so it is NOT expected to hit that. If the device still
    fails to start after this script, check the CodeIntegrity event log before
    assuming a software fault - and note that a device stuck at
    "Unknown USB Device (Port Reset Failed) / Code 43" with VID_0000&PID_0001 is
    a device that failed USB enumeration ENTIRELY (power/cable/port), which no
    driver install can fix.
#>
[CmdletBinding()]
param(
    [string]$Bundle,
    [string]$DriverExe,
    [switch]$WithApp,
    [switch]$SkipDriver
)

$ErrorActionPreference = 'Stop'

$DriverSha256 = '9131826137DE175870AFB84F145F74B83E536BAA14F836635D8793B1EAA4964B'
$RepoDll = Join-Path $PSScriptRoot 'DLLs\tucsen dlls\TUCam.dll'

function Say($msg, $colour = 'Gray') { Write-Host $msg -ForegroundColor $colour }
function Ok($msg)   { Say "  [OK]   $msg" 'Green' }
function Bad($msg)  { Say "  [FAIL] $msg" 'Red' }
function Warn($msg) { Say "  [WARN] $msg" 'Yellow' }
function Info($msg) { Say "         $msg" 'DarkGray' }

# -- Preconditions --------------------------------------------------
$id = [Security.Principal.WindowsIdentity]::GetCurrent()
if (-not (New-Object Security.Principal.WindowsPrincipal($id)).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Bad 'This script must run as Administrator (per-machine driver install).'
    Say '       Close this window, start PowerShell via right-click -> Run as administrator, and re-run.' 'Yellow'
    exit 1
}
Say "Running elevated as $($id.Name)`n" 'Cyan'

$work = Join-Path $env:TEMP 'mebp_tucsen'
New-Item -ItemType Directory -Force -Path $work | Out-Null

# -- 1. Locate the installers ---------------------------------------
Say '=== 1. Locate installers ===' 'Cyan'

$appZip = $null

if ($DriverExe) {
    if (-not (Test-Path $DriverExe)) { Bad "driver exe not found: $DriverExe"; exit 1 }
    Ok "driver exe: $DriverExe"
}
elseif ($Bundle) {
    if (-not (Test-Path $Bundle)) { Bad "bundle not found: $Bundle"; exit 1 }
    if ((Get-Item $Bundle).PSIsContainer) {
        $found = Get-ChildItem -Path $Bundle -Recurse -Filter 'Tucam Camera Driver*.exe' -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $found) { Bad "no 'Tucam Camera Driver*.exe' under $Bundle"; exit 1 }
        $DriverExe = $found.FullName
        $appFound = Get-ChildItem -Path $Bundle -Recurse -Filter 'TUCam_*Setup*.zip' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($appFound) { $appZip = $appFound.FullName }
    }
    else {
        Info "extracting installers from $Bundle ..."
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        $zip = [IO.Compression.ZipFile]::OpenRead($Bundle)
        try {
            foreach ($e in $zip.Entries) {
                if ($e.Name -like 'Tucam Camera Driver*.exe') {
                    $DriverExe = Join-Path $work $e.Name
                    [IO.Compression.ZipFileExtensions]::ExtractToFile($e, $DriverExe, $true)
                }
                elseif ($WithApp -and $e.Name -like 'TUCam_*Setup*.zip') {
                    $appZip = Join-Path $work $e.Name
                    Info "extracting $($e.Name) ($([math]::Round($e.Length/1MB)) MB) ..."
                    [IO.Compression.ZipFileExtensions]::ExtractToFile($e, $appZip, $true)
                }
            }
        }
        finally { $zip.Dispose() }
        if (-not $DriverExe) { Bad 'no Tucam Camera Driver exe inside the bundle'; exit 1 }
    }
    Ok "driver exe: $DriverExe"
    if ($appZip) { Ok "app package: $appZip" }
}
elseif (-not $SkipDriver) {
    Bad 'Pass -Bundle <vendor zip or folder> or -DriverExe <path>, or use -SkipDriver to verify only.'
    exit 1
}

# -- 2. Verify the driver package before running it -----------------
if ($DriverExe) {
    Say "`n=== 2. Verify driver package ===" 'Cyan'
    $sig = Get-AuthenticodeSignature $DriverExe
    if ($sig.Status -eq 'Valid') {
        Ok "Authenticode Valid - $($sig.SignerCertificate.Subject -replace ',.*','')"
    }
    else {
        Warn "Authenticode status: $($sig.Status) (expected Valid for the Tucsen driver)"
    }
    $hash = (Get-FileHash $DriverExe -Algorithm SHA256).Hash
    if ($hash -eq $DriverSha256) {
        Ok 'SHA256 matches the version this script was written against (V2.1.6.1)'
    }
    else {
        Warn "SHA256 $hash differs from the recorded V2.1.6.1 hash - a different driver build; proceeding."
    }
}

# -- 3. Install the driver ------------------------------------------
if ($SkipDriver) {
    Say "`n=== 3. Driver install SKIPPED (-SkipDriver) ===" 'Yellow'
}
else {
    Say "`n=== 3. Install driver ===" 'Cyan'
    Info 'Inno Setup silent install (/VERYSILENT /NORESTART) ...'
    $p = Start-Process -FilePath $DriverExe `
        -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART' `
        -Wait -PassThru
    if ($p.ExitCode -eq 0) { Ok 'driver installer exit 0' }
    elseif ($p.ExitCode -eq 3010) { Ok 'driver installer exit 3010 (reboot recommended)' }
    else {
        Bad "driver installer exit code $($p.ExitCode)"
        Info 'Re-run interactively (double-click the exe) to see its own error dialog.'
        exit 1
    }
}

# -- 4. Optionally install the TUCam application ---------------------
if ($WithApp) {
    Say "`n=== 4. Install TUCam application (optional) ===" 'Cyan'
    if (-not $appZip) {
        Warn 'no TUCam application package found; skipping (MEBP does not need it).'
    }
    else {
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        $z = [IO.Compression.ZipFile]::OpenRead($appZip)
        $setup = $null
        try {
            foreach ($e in $z.Entries) {
                if ($e.Name -like '*Setup*.exe') {
                    $setup = Join-Path $work $e.Name
                    Info "extracting $($e.Name) ($([math]::Round($e.Length/1MB)) MB) ..."
                    [IO.Compression.ZipFileExtensions]::ExtractToFile($e, $setup, $true)
                    break
                }
            }
        }
        finally { $z.Dispose() }
        if (-not $setup) {
            Warn 'no Setup exe inside the app package; skipping.'
        }
        else {
            Info 'Installing TUCam (this is a large install and may take several minutes) ...'
            # Vendor installer type is not guaranteed; try silent, fall back to
            # interactive rather than reporting a false failure.
            $p2 = Start-Process -FilePath $setup `
                -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART' `
                -Wait -PassThru -ErrorAction SilentlyContinue
            if ($p2 -and ($p2.ExitCode -eq 0 -or $p2.ExitCode -eq 3010)) {
                Ok "TUCam application installed (exit $($p2.ExitCode))"
            }
            else {
                Warn "silent install returned $($p2.ExitCode); run it interactively if you want the viewer:"
                Info $setup
            }
        }
    }
}

# -- 5. Verify ------------------------------------------------------
Say "`n=== 5. Verify ===" 'Cyan'

# 5a. Vendored SDK (what MEBP actually loads).
if (Test-Path $RepoDll) {
    $v = (Get-Item $RepoDll).VersionInfo
    Ok "vendored SDK present: TUCam.dll $($v.FileVersion) ($($v.CompanyName))"
}
else {
    Bad "vendored SDK MISSING: $RepoDll"
    Info 'MEBP loads this copy; without it the Tucsen backend is unavailable.'
}

# 5b. Any TUCam install (informational only).
$installed = Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
    'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*' `
    -ErrorAction SilentlyContinue |
    Where-Object { $_.DisplayName -match 'TUCam|Tucsen|Tucam' } |
    Select-Object -ExpandProperty DisplayName -ErrorAction SilentlyContinue
if ($installed) { foreach ($n in $installed) { Ok "installed: $n" } }
else { Info 'no Tucsen entry in Add/Remove Programs (fine - MEBP uses the vendored DLL).' }

# 5c. USB device state.
$cams = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
    Where-Object { $_.FriendlyName -match 'Tucsen|Libra|Dhyana|Aries|TUCam' }
if ($cams) {
    foreach ($c in $cams) {
        if ($c.Status -eq 'OK') { Ok "device: $($c.FriendlyName) [$($c.Status)]" }
        else { Warn "device: $($c.FriendlyName) [$($c.Status)] $($c.ProblemDescription)" }
    }
}
else {
    Warn 'no Tucsen-named USB device is present.'
    $broken = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.Status -ne 'OK' -and $_.InstanceId -like 'USB\*' }
    foreach ($b in $broken) {
        Info "in error: $($b.FriendlyName) - $($b.InstanceId)"
    }
    Info 'A device showing VID_0000&PID_0001 / "Port Reset Failed" (Code 43) failed USB'
    Info 'enumeration outright - that is power/cable/port, not a driver problem. Try a'
    Info 'different (rear, USB3) port, the supplied cable, and the external PSU if the'
    Info 'model has one.'
}

# 5d. Ask the SDK itself how many cameras it can see - the ground truth.
Say "`n--- SDK camera count (ground truth) ---" 'Cyan'
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) {
    Info 'python not on PATH; skipping. Run this from the repo to check:'
    Info '  python -c "from gui.widgets.tucam_backend import TUCamBackend as B; print(B.enumerate())"'
}
else {
    Push-Location $PSScriptRoot
    try {
        $out = & python -c "from gui.widgets.tucam_backend import TUCamBackend as B; print('CAMERAS=' + repr(B.enumerate()))" 2>&1
        $line = ($out | Select-String 'CAMERAS=').ToString()
        if ($line -match 'CAMERAS=\[\]') {
            Warn 'the SDK loads but reports 0 cameras attached.'
            Info 'Software is fine; the camera is not reachable yet (see 5c above).'
        }
        elseif ($line) {
            $count = ($line -replace '.*CAMERAS=', '')
            Ok "SDK reports: $count"
            Say "`nNext: MEBP -> Hardware Setup -> Cameras -> Detect Cameras, assign the" 'Green'
            Say 'Tucsen source to a slot, then calibrate its um/px for the Microscope role.' 'Green'
        }
        else {
            Warn 'could not read a camera count from the backend:'
            $out | ForEach-Object { Info $_ }
        }
    }
    catch { Warn "backend check failed: $_" }
    finally { Pop-Location }
}

Say "`nDone. A reboot is recommended if the driver was just installed.`n" 'Cyan'
