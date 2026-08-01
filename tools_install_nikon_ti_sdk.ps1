<#
.SYNOPSIS
    Install the Nikon Ti SDK redistributable + USB driver for a Ti / Ti-E body.

.DESCRIPTION
    One-time per-machine setup so MEBP can drive the microscope body (filter
    cubes, focus, objectives). Does three things, then verifies each:

      1. Installs TiSDKRedist64 (the redistributable Nikon Ti SDK published on
         micro-manager.org — no Nikon developer registration needed). This
         self-registers NikonTi.dll as a COM server, which is what the MEBP
         'nikon_ti' backend talks to, and stages the micusb USB driver.
      2. Installs the micusb driver into the Windows driver store, which is
         what clears the "Code 28 - drivers not installed" state on the body's
         USB device.
      3. Verifies: DLL present, COM ProgID registered, device no longer in
         error.

    MUST BE RUN AS ADMINISTRATOR (per-machine MSI + driver install).

.PARAMETER Zip
    Path to an already-downloaded TiSDKRedist64 zip. Downloaded if omitted.

.PARAMETER SkipDriver
    Install the SDK only; leave the USB driver alone.

.EXAMPLE
    # Right-click PowerShell -> Run as administrator. It opens in system32, so
    # pass the FULL path (or cd to the repo first):
    powershell -ExecutionPolicy Bypass -File C:\dev\MEBP\tools_install_nikon_ti_sdk.ps1

.NOTES
    Ti-E is first-generation Ti, so the 4.4.x SDK is the correct kit; a Ti2 body
    needs Nikon's separate Ti2 SDK instead.

    ⚠ The driver catalog (micusb.cat) is signed by Nikon but with a SHA-1
    certificate that expired in 2012 (it is counter-timestamped, so the
    signature itself still verifies). Windows 11 may still refuse to load it.
    If step 2 fails, that is the reason, and the fallback is Nikon's current
    Ti Setup Tool from their Software Developer Toolkit site.
#>
[CmdletBinding()]
param(
    [string]$Zip,
    [switch]$SkipDriver
)

$ErrorActionPreference = 'Stop'

$Url    = 'https://micro-manager.org/media/files/TiSDKRedist64-4.4.1.714.zip'
$Sha256 = '706500BA50451EAC61E552C2B7AB08FE335B66BA001C48B1B833F98D0A818950'
$DllPath = 'C:\Program Files\Nikon\Shared\Bin\NikonTi.dll'
$DriverDir = 'C:\Program Files\Nikon\Shared\Drivers'
$ProgId = 'Nikon.TiScope.NikonTi'

function Say($msg, $colour = 'Gray') { Write-Host $msg -ForegroundColor $colour }
function Ok($msg)   { Say "  [OK]   $msg" 'Green' }
function Bad($msg)  { Say "  [FAIL] $msg" 'Red' }
function Warn($msg) { Say "  [WARN] $msg" 'Yellow' }

# ── Preconditions ──────────────────────────────────────────────────
$id = [Security.Principal.WindowsIdentity]::GetCurrent()
if (-not (New-Object Security.Principal.WindowsPrincipal($id)).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Bad 'This script must run as Administrator (per-machine MSI + driver install).'
    Say '       Close this window, start PowerShell via right-click -> Run as administrator, and re-run.' 'Yellow'
    exit 1
}
Say "Running elevated as $($id.Name)`n" 'Cyan'

$work = Join-Path $env:TEMP 'mebp_tisdk'
New-Item -ItemType Directory -Force -Path $work | Out-Null

# ── 1. Obtain + verify the package ─────────────────────────────────
Say '=== 1. Package ===' 'Cyan'
if (-not $Zip) {
    $Zip = Join-Path $work 'TiSDKRedist64-4.4.1.714.zip'
    if (Test-Path $Zip) {
        Ok "using cached download: $Zip"
    } else {
        Say "  downloading $Url"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri $Url -OutFile $Zip -UseBasicParsing -TimeoutSec 600
        Ok "downloaded ($('{0:N1}' -f ((Get-Item $Zip).Length / 1MB)) MB)"
    }
}
if (-not (Test-Path $Zip)) { Bad "zip not found: $Zip"; exit 1 }

$hash = (Get-FileHash $Zip -Algorithm SHA256).Hash
if ($hash -ne $Sha256) {
    Warn "SHA256 mismatch - expected $Sha256, got $hash"
    Warn 'Continuing, but verify you trust the source.'
} else {
    Ok 'SHA256 matches the known-good package'
}

$extract = Join-Path $work 'extract'
Remove-Item $extract -Recurse -Force -ErrorAction SilentlyContinue
Expand-Archive -Path $Zip -DestinationPath $extract -Force
$msi = Get-ChildItem $extract -Filter '*.msi' | Select-Object -First 1
if (-not $msi) { Bad 'no MSI inside the archive'; exit 1 }
Ok "MSI: $($msi.Name)"

# ── 2. Install the SDK ─────────────────────────────────────────────
Say "`n=== 2. Installing the Ti SDK ===" 'Cyan'
$log = Join-Path $work 'install.log'
$p = Start-Process msiexec -ArgumentList @(
    '/i', "`"$($msi.FullName)`"", '/qn', '/norestart', '/l*v', "`"$log`"") `
    -Wait -PassThru -NoNewWindow
if ($p.ExitCode -eq 0) {
    Ok 'MSI installed (exit 0)'
} elseif ($p.ExitCode -eq 3010) {
    Ok 'MSI installed (exit 3010 - reboot recommended)'
} else {
    Bad "msiexec exit code $($p.ExitCode). Log: $log"
    Say '       (1603 while elevated usually means a conflicting existing install)' 'Yellow'
    exit 1
}

if (Test-Path $DllPath) {
    $v = (Get-Item $DllPath).VersionInfo.FileVersion
    Ok "NikonTi.dll present (v$v)"
} else {
    Bad "expected $DllPath - not found"
}

# ── 3. USB driver ──────────────────────────────────────────────────
if (-not $SkipDriver) {
    Say "`n=== 3. USB driver (clears Code 28) ===" 'Cyan'
    $inf = Get-ChildItem $DriverDir -Filter 'micusb.inf' -ErrorAction SilentlyContinue |
           Select-Object -First 1
    if (-not $inf) {
        Warn "no micusb.inf under $DriverDir - skipping driver step"
    } else {
        # /install binds it to matching present hardware, not just the store.
        $out = & pnputil /add-driver "$($inf.FullName)" /install 2>&1 | Out-String
        $out.Trim() -split "`r?`n" | ForEach-Object { Say "       $_" }
        if ($LASTEXITCODE -eq 0) {
            Ok 'driver added to the driver store'
        } else {
            Warn "pnputil exit $LASTEXITCODE - the SHA-1/2012 signature may be refused by Windows 11"
            Say '       Fallback: install Nikon''s current Ti Setup Tool, or point Device Manager' 'Yellow'
            Say "       at $DriverDir manually (Update Driver -> Browse)." 'Yellow'
        }
    }
}

# ── 4. Verify ──────────────────────────────────────────────────────
Say "`n=== 4. Verification ===" 'Cyan'

$clsKey = "Registry::HKEY_CLASSES_ROOT\$ProgId"
if (Test-Path $clsKey) {
    Ok "COM ProgID registered: $ProgId"
} else {
    $any = Get-ChildItem 'Registry::HKEY_CLASSES_ROOT' -ErrorAction SilentlyContinue |
           Where-Object { $_.PSChildName -like 'Nikon.TiScope*' } |
           Select-Object -First 5 -ExpandProperty PSChildName
    if ($any) {
        Warn "$ProgId not found, but these are registered: $($any -join ', ')"
    } else {
        Bad 'no Nikon.TiScope ProgIDs registered - the DLL did not self-register'
    }
}

$dev = Get-PnpDevice -ErrorAction SilentlyContinue |
       Where-Object { $_.InstanceId -like '*VID_04B0*' }
if ($dev) {
    foreach ($d in $dev) {
        if ($d.Status -eq 'OK') { Ok "device '$($d.FriendlyName)' status OK" }
        else { Warn "device '$($d.FriendlyName)' status $($d.Status) (Problem $($d.Problem))" }
    }
} else {
    Warn 'no Nikon-VID USB device found - is the body powered on and connected?'
}

$check = Join-Path $PSScriptRoot 'tools_microscope_hw_check.py'
Say "`nNext, verify the body responds:" 'Cyan'
Say "      python `"$check`" nikon_ti" 'Cyan'
Say '      (add --focus once the sample is clear of the objective)' 'Cyan'
