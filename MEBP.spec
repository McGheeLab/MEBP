# -*- mode: python ; coding: utf-8 -*-
"""
MEBP PyInstaller Spec File
===========================
Builds a --onedir executable for the MEBP Bioprinter application.

Usage:
    pyinstaller MEBP.spec

Output:
    dist/MEBP/          — distributable folder
    dist/MEBP/MEBP      — the executable (MEBP.exe on Windows)

Notes:
    - Uses --onedir so that config/, settings.json, and print_records/
      persist between runs (--onefile extracts to a temp dir).
    - ToupCam SDK (toupcam.dll / libtoupcam.dylib) is NOT bundled.
      Users with ToupTek cameras must install ToupView or provide
      the SDK separately.
"""

import sys
from pathlib import Path

block_cipher = None

# ── Paths ──────────────────────────────────────────────────────────
PROJECT_ROOT = Path(SPECPATH)

# ── Data files to bundle ───────────────────────────────────────────
# (source_path, dest_relative_to_bundle)
# These are placed inside the _internal/ directory alongside the
# collected Python modules, so relative paths resolve via os.chdir().
datas = [
    # Hardware protocol definitions (CRITICAL — required for stage control)
    (str(PROJECT_ROOT / 'config' / 'controllers'), 'config/controllers'),
    # Hardware catalogs (needles, syringes, cameras, objectives)
    (str(PROJECT_ROOT / 'config' / 'hardware'), 'config/hardware'),
    # Sample print files (user-editable)
    (str(PROJECT_ROOT / 'config' / 'prints'), 'config/prints'),
    # Root-level config files
    (str(PROJECT_ROOT / 'config' / 'prtcfg.json'), 'config'),
    (str(PROJECT_ROOT / 'config' / 'xy_diagnostic_profile.json'), 'config'),
    # Reference images
    (str(PROJECT_ROOT / 'PrintImages'), 'PrintImages'),
    # ANDOR Zyla SDK3 runtime DLLs (atcore.dll + companions). Bundled so the
    # Andor backend (gui/widgets/andor_backend.py) can find them at runtime;
    # _find_andor_dll_dir() looks in DLLs/zyla dlls/ first.
    (str(PROJECT_ROOT / 'DLLs' / 'zyla dlls'), 'DLLs/zyla dlls'),
    # Default config (if present)
    (str(PROJECT_ROOT / 'Default.json'), '.'),
]

# Filter out data entries whose source doesn't exist
datas = [(src, dst) for src, dst in datas if Path(src).exists()]

# ── Hidden imports ─────────────────────────────────────────────────
# Modules that PyInstaller's static analysis may miss
hiddenimports = [
    # PySide6 modules used via dynamic import
    'PySide6.QtCore',
    'PySide6.QtGui',
    'PySide6.QtWidgets',
    'PySide6.QtOpenGL',
    'PySide6.QtOpenGLWidgets',
    # Scientific stack
    'numpy',
    'numpy.core._methods',
    'numpy.lib.format',
    'cv2',
    'scipy',
    'scipy.interpolate',
    'scipy.spatial',
    # Hardware / IO
    'serial',
    'serial.tools',
    'serial.tools.list_ports',
    # .nd3 container (HDF5). h5py's own PyInstaller hook pulls its extension
    # modules; these two are ours and are reached through lazy imports.
    'h5py',
    'SupportClasses.ND3',
    'SupportClasses.ND3Export',
    # LabLink client — vendored stdlib-only subpackage (v7.17). Imported
    # lazily by LabLinkService so the app starts without it; PyInstaller's
    # static analysis therefore does not see it.
    'SupportClasses.lablink',
    'SupportClasses.lablink.protocol',
    'SupportClasses.lablink.fsutil',
    'SupportClasses.lablink.client',
    'SupportClasses.lablink.session_client',
    # Xbox controller
    'pygame',
    # App modules that may be imported dynamically
    'SupportClasses.XboxController',
    'SupportClasses.SimulatedCamera',
    'SupportClasses.XYStageSimulator',
    'SupportClasses.ZPStageSimulator',
    'SupportClasses.VisionDetector',
    'SupportClasses.ImageStitcher',
    'SupportClasses.PickAndPlaceManager',
    'SupportClasses.XYDebugLogger',
]

# ── Optional: ANDOR Zyla backend (pylablib) ────────────────────────
# pylablib is only present on rigs with an Andor camera. Collect it fully when
# available so PyInstaller bundles its submodules/DLL loaders; skip silently
# otherwise (the backend is lazily guarded — the app runs without it).
try:
    from PyInstaller.utils.hooks import collect_all as _collect_all
    _pll_datas, _pll_bin, _pll_hidden = _collect_all('pylablib')
    datas += _pll_datas
    hiddenimports += _pll_hidden
    _extra_binaries = _pll_bin
except Exception:
    _extra_binaries = []

# ── Excludes ───────────────────────────────────────────────────────
excludes = [
    'tkinter',
    '_tkinter',
    'test',
    'unittest',
    'pytest',
    'IPython',
    'jupyter',
    'notebook',
    'matplotlib',       # Not used by MEBP; saves ~30 MB
    # Heavy packages dragged in as transitive deps but not used by MEBP
    'tensorflow',       # ~765 MB
    'torch',            # ~284 MB
    'torchaudio',
    'torchvision',
    'pyarrow',          # ~114 MB
    # NOTE: numba / llvmlite / pandas are intentionally NOT excluded — pylablib
    # (the Andor Zyla SDK3 backend) imports all three eagerly, so the bundled
    # Zyla path needs them present. They add ~150 MB to the bundle.
    'sklearn',          # ~16 MB
    'scikit-learn',
    'skimage',          # ~14 MB
    'scikit-image',
    'PIL',              # ~11 MB (Pillow — MEBP uses OpenCV, not PIL)
    'lxml',             # ~9 MB
    'grpc',             # ~18 MB
    'grpcio',
    'google',
    'h5py',
    'tables',
    'sqlalchemy',
    'jinja2',
    'babel',
    'docutils',
    'sphinx',
    'setuptools',
    'pip',
    'wheel',
    'pkg_resources',
]

# ── Analysis ───────────────────────────────────────────────────────
a = Analysis(
    [str(PROJECT_ROOT / 'main.py')],
    pathex=[str(PROJECT_ROOT)],
    binaries=_extra_binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

# ── PYZ (compressed Python archive) ───────────────────────────────
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

# ── EXE ────────────────────────────────────────────────────────────
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,  # --onedir: binaries collected separately
    name='MEBP',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,  # No terminal window (GUI app)
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    # macOS .app icon (set to None or provide .icns file)
    icon=None,
)

# ── COLLECT (--onedir bundle) ──────────────────────────────────────
coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='MEBP',
)

# ── macOS .app bundle (optional) ───────────────────────────────────
# Uncomment to create MEBP.app instead of a plain folder.
# app = BUNDLE(
#     coll,
#     name='MEBP.app',
#     icon=None,
#     bundle_identifier='com.lab.mebp',
#     info_plist={
#         'NSHighResolutionCapable': True,
#         'CFBundleShortVersionString': '7.3.5',
#     },
# )
