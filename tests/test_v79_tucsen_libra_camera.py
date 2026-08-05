"""
v7.9.x — Tucsen (TUCam SDK) camera backend tests.

Covers ``gui/widgets/tucam_backend.py`` and its wiring into the CameraWidget
multi-backend system, for the Tucsen Libra 25 evaluated as an alternate
MICROSCOPE camera.

Strategy (mirrors the existing ``FakeToupCam`` / ``FakeAndorCam`` suites): a
``FakeTUCamLib`` stands in for the real ``TUCam.dll`` at the ctypes boundary, so
**the production backend code runs unmodified** — device open, buffer alloc,
capture start, the reader thread, ``Buf_WaitForFrame``, the frame decode and the
mono→BGR8 conversion. Only the DLL is substituted, which is the one thing that
needs a camera attached.

That matters because the frame decode is where a real camera bug would live: the
row-pitch handling, the in-band header offset, the channel/bit-depth
interpretation and the µs↔ms exposure conversion are all exercised here against
values the fake reports the way the SDK does.
"""

from __future__ import annotations

import ctypes
import threading
import time
import unittest
from ctypes import POINTER, byref, c_char_p, c_ubyte, c_void_p, create_string_buffer
from pathlib import Path

import numpy as np

from gui.widgets import tucam_backend as tb
from gui.widgets.tucam_backend import TUCAM_FRAME, TUCamBackend

SUCCESS = tb.TUCAMRET_SUCCESS
FAIL = 0x80000000


def _deref(arg):
    """Recover the struct behind a ``ctypes.byref()`` argument."""
    return arg._obj


def _v(x):
    """Unwrap a ctypes scalar (c_int/c_double/...) to a Python number.

    The backend passes scalars as real ctypes objects, and ``int(c_int(7))``
    raises — so the fake must read ``.value`` the way the C ABI would.
    """
    return x.value if hasattr(x, "value") else x


# ════════════════════════════════════════════════════════════════════
#  Fake TUCam.dll
# ════════════════════════════════════════════════════════════════════

class FakeTUCamLib:
    """Minimal stand-in for TUCam.dll at the ctypes call boundary."""

    def __init__(self, *, cam_count=1, model=b"Libra 25", width=640, height=480,
                 channels=1, elem_bytes=2, depth=16, header=0, pitch=None,
                 resolutions=None, props=None, fill=None):
        self.cam_count = cam_count
        self.model = model
        self.width, self.height = width, height
        self.channels, self.elem_bytes, self.depth = channels, elem_bytes, depth
        self.header = header
        self.pitch = pitch          # None → tight rows
        # No resolution capability unless a list is supplied: the decode tests
        # need width/height to stay exactly what they asked for, and a live
        # resolution capability would have auto-pick overwrite them.
        self.resolutions = list(resolutions) if resolutions else []
        self.res_index = 0
        # Geometry follows the CURRENT resolution index, not the write history:
        # a real camera reports its actual size via Buf_Alloc whether or not we
        # just wrote the capability (the backend legitimately skips a no-change
        # write, see _capa_set).
        if self.resolutions:
            self.width, self.height = self._dims(0)
        self.fill = fill            # callable(np_view) to paint the frame
        # Supported properties: id → [min, max, default, step, current]
        self.props = props if props is not None else {
            tb.TUIDP_EXPOSURETM: [0.1, 1000.0, 30.0, 0.1, 30.0],   # ms
            tb.TUIDP_GLOBALGAIN: [0.0, 100.0, 0.0, 1.0, 0.0],
            tb.TUIDP_GAMMA: [1.0, 200.0, 100.0, 1.0, 100.0],
        }
        self.capas = {tb.TUIDC_ATEXPOSURE: [0, 1, 1, 1, 1]}
        self.res_capa_id = 7
        if self.resolutions:
            self.capas[self.res_capa_id] = [
                0, len(self.resolutions) - 1, 0, 1, 0]

        self.api_inits = 0
        self.api_uninits = 0
        self.opens = 0
        self.closes = 0
        self.cap_starts = 0
        self.cap_stops = 0
        self.buf_allocs = 0
        self.buf_releases = 0
        self.aborts = 0
        self.frames_served = 0
        self.prop_writes: list[tuple[int, float]] = []
        self.capa_writes: list[tuple[int, int]] = []
        self._buf = None
        self._stop = threading.Event()
        self.wait_delay = 0.002

    def _dims(self, index):
        w, h = self.resolutions[index].split("x")
        return int(w), int(h)

    # ── API lifecycle ─────────────────────────────────────────────
    def TUCAM_Api_Init(self, pinit, timeout=None):
        self.api_inits += 1
        _deref(pinit).uiCamCount = self.cam_count
        return SUCCESS

    def TUCAM_Api_Uninit(self):
        self.api_uninits += 1
        return SUCCESS

    def TUCAM_Dev_GetInfo(self, handle, pvi):
        vi = _deref(pvi)
        if vi.nID not in tb._INFO_MODEL_CANDIDATES:
            return FAIL
        # Only the first candidate answers, so the probe order is exercised.
        if vi.nID != tb._INFO_MODEL_CANDIDATES[0]:
            return FAIL
        ctypes.memmove(vi.pText, self.model + b"\x00", len(self.model) + 1)
        return SUCCESS

    def TUCAM_Dev_Open(self, popen):
        op = _deref(popen)
        if op.uiIdxOpen >= self.cam_count:
            return FAIL
        self.opens += 1
        op.hIdxTUCam = 0xBEEF0000 + int(op.uiIdxOpen)
        return SUCCESS

    def TUCAM_Dev_Close(self, handle):
        self.closes += 1
        return SUCCESS

    # ── Properties ────────────────────────────────────────────────
    def TUCAM_Prop_GetAttr(self, handle, pattr):
        attr = _deref(pattr)
        spec = self.props.get(int(attr.idProp))
        if spec is None:
            return FAIL
        attr.dbValMin, attr.dbValMax, attr.dbValDft, attr.dbValStep = spec[:4]
        return SUCCESS

    def TUCAM_Prop_GetValue(self, handle, pid, pval, chan=None):
        spec = self.props.get(int(_v(pid)))
        if spec is None:
            return FAIL
        _deref(pval).value = float(spec[4])
        return SUCCESS

    def TUCAM_Prop_SetValue(self, handle, pid, val, chan=None):
        spec = self.props.get(int(_v(pid)))
        if spec is None:
            return FAIL
        v = float(_v(val))
        spec[4] = v
        self.prop_writes.append((int(_v(pid)), v))
        return SUCCESS

    # ── Capabilities ──────────────────────────────────────────────
    def TUCAM_Capa_GetAttr(self, handle, pattr):
        attr = _deref(pattr)
        spec = self.capas.get(int(attr.idCapa))
        if spec is None:
            return FAIL
        attr.nValMin, attr.nValMax, attr.nValDft, attr.nValStep = \
            [int(x) for x in spec[:4]]
        return SUCCESS

    def TUCAM_Capa_GetValue(self, handle, cid, pval):
        spec = self.capas.get(int(_v(cid)))
        if spec is None:
            return FAIL
        _deref(pval).value = int(spec[4])
        return SUCCESS

    def TUCAM_Capa_SetValue(self, handle, cid, val):
        spec = self.capas.get(int(_v(cid)))
        if spec is None:
            return FAIL
        v = int(_v(val))
        spec[4] = v
        self.capa_writes.append((int(_v(cid)), v))
        if int(_v(cid)) == self.res_capa_id:
            self.res_index = int(v)
            self.width, self.height = self._dims(self.res_index)
        return SUCCESS

    def TUCAM_Capa_GetValueText(self, handle, pvt):
        vt = _deref(pvt)
        if int(vt.nID) != self.res_capa_id:
            return FAIL
        i = int(vt.dbValue)
        if not (0 <= i < len(self.resolutions)):
            return FAIL
        text = self.resolutions[i].encode()
        ctypes.memmove(vt.pText, text + b"\x00", len(text) + 1)
        return SUCCESS

    # ── Buffer / capture ──────────────────────────────────────────
    def _row_bytes(self):
        tight = self.width * self.channels * self.elem_bytes
        return tight if self.pitch is None else self.pitch

    def TUCAM_Buf_Alloc(self, handle, pframe):
        self.buf_allocs += 1
        f = _deref(pframe)
        img = self._row_bytes() * self.height
        self._buf = create_string_buffer(self.header + img)
        f.pBuffer = ctypes.cast(self._buf, POINTER(c_ubyte))
        f.usHeader = self.header
        f.usOffset = 0
        f.usWidth = self.width
        f.usHeight = self.height
        f.uiWidthStep = self._row_bytes()
        f.ucDepth = self.depth
        f.ucChannels = self.channels
        f.ucElemBytes = self.elem_bytes
        f.uiImgSize = img
        return SUCCESS

    def TUCAM_Buf_Release(self, handle):
        self.buf_releases += 1
        self._buf = None
        return SUCCESS

    def TUCAM_Cap_Start(self, handle, mode):
        self.cap_starts += 1
        self._stop.clear()
        return SUCCESS

    def TUCAM_Cap_Stop(self, handle):
        self.cap_stops += 1
        return SUCCESS

    def TUCAM_Buf_AbortWait(self, handle):
        self.aborts += 1
        self._stop.set()
        return SUCCESS

    def TUCAM_Buf_WaitForFrame(self, handle, pframe, timeout=None):
        if self._stop.is_set() or self._buf is None:
            return FAIL
        time.sleep(self.wait_delay)
        f = _deref(pframe)
        # Refresh geometry (a real SDK re-reports it each frame).
        f.usWidth, f.usHeight = self.width, self.height
        f.uiWidthStep = self._row_bytes()
        f.ucChannels, f.ucElemBytes, f.ucDepth = (
            self.channels, self.elem_bytes, self.depth)
        f.uiImgSize = self._row_bytes() * self.height
        f.usHeader = self.header
        f.pBuffer = ctypes.cast(self._buf, POINTER(c_ubyte))

        dtype = np.uint16 if self.elem_bytes >= 2 else np.uint8
        row_elems = self._row_bytes() // self.elem_bytes
        view = np.frombuffer(
            self._buf, dtype=dtype, count=row_elems * self.height,
            offset=self.header).reshape(self.height, row_elems)
        if self.fill is not None:
            self.fill(view)
        else:
            view[:, :] = 1000 if dtype == np.uint16 else 100
        self.frames_served += 1
        return SUCCESS


class _TUCamFixture(unittest.TestCase):
    """Installs a FakeTUCamLib as the module-level SDK handle."""

    #: OS device names the fake pretends Windows reports (see _install).
    os_names = ["Libra 25"]

    def _install(self, **kw):
        lib = FakeTUCamLib(**kw)
        self._saved = (tb._lib, tb._lib_error, tb._api_refs, tb._api_cam_count,
                       tb._os_model_cache)
        tb._lib = lib
        tb._lib_error = None
        tb._api_refs = 0
        tb._api_cam_count = 0
        # Pin the OS model-name lookup. Without this the real registry is read,
        # so a fake-SDK test would inherit whatever camera is physically
        # attached -- making the suite's result depend on the bench.
        tb._os_model_cache = list(self.os_names)
        self.addCleanup(self._restore)
        return lib

    def _restore(self):
        (tb._lib, tb._lib_error, tb._api_refs, tb._api_cam_count,
         tb._os_model_cache) = self._saved

    def _open(self, lib, **kw):
        be = TUCamBackend()
        self.addCleanup(be.release)
        self.assertTrue(be.open("0", **kw), "open() should succeed with the fake SDK")
        return be

    def _first_frame(self, be, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            ok, frame = be.read()
            if ok and frame is not None:
                return frame
            time.sleep(0.01)
        self.fail("no frame arrived from the reader thread")


# ════════════════════════════════════════════════════════════════════
#  Availability / discovery
# ════════════════════════════════════════════════════════════════════

class TestAvailabilityAndDiscovery(unittest.TestCase):

    def test_vendored_dll_is_present_in_repo(self):
        """The SDK is vendored so the app works without the TUCam install."""
        root = Path(__file__).resolve().parent.parent
        dll = root / "DLLs" / "tucsen dlls" / "TUCam.dll"
        self.assertTrue(dll.is_file(), f"missing vendored SDK: {dll}")
        self.assertGreater(dll.stat().st_size, 1_000_000)

    def test_finder_locates_the_vendored_dll(self):
        found = tb._find_tucam_dll()
        self.assertIsNotNone(found)
        self.assertTrue(found.lower().endswith("tucam.dll"))

    def test_enumerate_returns_empty_without_sdk(self):
        saved = (tb._lib, tb._lib_error)
        tb._lib, tb._lib_error = None, "forced: no SDK"
        try:
            self.assertEqual(TUCamBackend.enumerate(), [])
            be = TUCamBackend()
            self.assertFalse(be.open("0"))
            self.assertFalse(be.isOpened())
            self.assertEqual(be.read(), (False, None))
            be.release()          # must not raise
        finally:
            tb._lib, tb._lib_error = saved

    def test_dims_from_text_parsing(self):
        self.assertEqual(tb._dims_from_text("2048x2048"), (2048, 2048))
        self.assertEqual(tb._dims_from_text("1920 X 1080"), (1920, 1080))
        self.assertEqual(tb._dims_from_text("960*540"), (960, 540))
        self.assertIsNone(tb._dims_from_text("High Speed"))
        self.assertIsNone(tb._dims_from_text(""))

    def test_sdk_config_dir_is_not_the_tracked_dll_directory(self):
        """The SDK WRITES into the config path it is handed.

        It drops one XML per physical camera (named with that unit's serial)
        into that directory. Pointing it at `DLLs/tucsen dlls/` — which is
        tracked in git — litters the repo with machine-local per-unit state that
        can be committed by accident, so the config path must be somewhere else
        and writable.
        """
        cfg = Path(tb._sdk_config_dir()).resolve()
        dll_dir = Path(tb._find_tucam_dll() or ".").resolve().parent
        self.assertNotEqual(cfg, dll_dir,
                            "SDK would write per-unit files into the repo's DLL dir")
        self.assertTrue(cfg.is_dir(), f"config dir not created: {cfg}")
        # Must be writable — the SDK cannot persist its parameter set otherwise.
        probe = cfg / ".write_probe"
        try:
            probe.write_text("x", encoding="utf-8")
        finally:
            probe.unlink(missing_ok=True)

    def test_sdk_config_dir_is_gitignored(self):
        """Per-unit SDK state must never reach a commit."""
        root = Path(__file__).resolve().parent.parent
        ignore = (root / ".gitignore").read_text(encoding="utf-8")
        self.assertIn("config/hardware/tucsen/", ignore)


class TestEnumeration(_TUCamFixture):

    def test_enumerate_reports_model_from_sdk(self):
        lib = self._install(cam_count=2, model=b"Libra 25")
        devs = TUCamBackend.enumerate()
        self.assertEqual(len(devs), 2)
        self.assertEqual(devs[0]["id"], "0")
        self.assertEqual(devs[0]["displayname"], "Libra 25")

    def test_enumerate_releases_the_api_reference(self):
        lib = self._install()
        TUCamBackend.enumerate()
        self.assertEqual(tb._api_refs, 0)
        self.assertEqual(lib.api_uninits, 1)

    def test_open_out_of_range_index_refuses(self):
        self._install(cam_count=1)
        be = TUCamBackend()
        self.assertFalse(be.open("5"))
        self.assertEqual(tb._api_refs, 0, "API ref must not leak on refusal")

    def test_open_rejects_non_numeric_id(self):
        self._install()
        be = TUCamBackend()
        self.assertFalse(be.open("not-an-index"))


# ════════════════════════════════════════════════════════════════════
#  Streaming + frame decode
# ════════════════════════════════════════════════════════════════════

class TestStreamLifecycle(_TUCamFixture):

    def test_open_starts_stream_in_sdk_order(self):
        lib = self._install()
        be = self._open(lib)
        self.assertEqual(lib.opens, 1)
        self.assertEqual(lib.buf_allocs, 1)
        self.assertEqual(lib.cap_starts, 1)
        self.assertTrue(be.isOpened())

    def test_release_tears_down_in_the_required_order(self):
        lib = self._install()
        be = self._open(lib)
        self._first_frame(be)
        be.release()
        # AbortWait must precede Cap_Stop/Buf_Release so the reader is not
        # blocked in the SDK when its buffer is freed.
        self.assertGreaterEqual(lib.aborts, 1)
        self.assertEqual(lib.cap_stops, 1)
        self.assertEqual(lib.buf_releases, 1)
        self.assertEqual(lib.closes, 1)
        self.assertFalse(be.isOpened())

    def test_reader_thread_is_joined_on_release(self):
        lib = self._install()
        be = self._open(lib)
        self._first_frame(be)
        reader = be._reader
        self.assertIsNotNone(reader)
        be.release()
        self.assertFalse(reader.is_alive(), "reader thread outlived release()")

    def test_double_release_is_safe(self):
        lib = self._install()
        be = self._open(lib)
        be.release()
        be.release()
        self.assertEqual(lib.closes, 1)

    def test_read_before_open_returns_false(self):
        self._install()
        be = TUCamBackend()
        self.assertEqual(be.read(), (False, None))


class TestFrameDecode(_TUCamFixture):

    def test_mono16_frame_becomes_bgr8(self):
        lib = self._install(width=64, height=48, channels=1, elem_bytes=2)
        be = self._open(lib)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (48, 64, 3))
        self.assertEqual(frame.dtype, np.uint8)

    def test_padded_row_pitch_is_honoured(self):
        """A padded row stride must not shear the image.

        The left half is bright and the right half dark; if uiWidthStep were
        ignored the boundary would walk across rows.
        """
        w, h = 64, 32
        tight = w * 2
        pad = tight + 32                      # 16 extra uint16 per row

        def fill(view):
            view[:, :] = 0
            view[:, : w // 2] = 60000

        lib = self._install(width=w, height=h, channels=1, elem_bytes=2,
                            pitch=pad, fill=fill)
        be = self._open(lib)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (h, w, 3))
        gray = frame[:, :, 0].astype(int)
        # Every row must show the same bright/dark split.
        for row in range(h):
            self.assertGreater(gray[row, : w // 2].min(), 200,
                               f"row {row} left half not bright")
            self.assertLess(gray[row, w // 2:].max(), 55,
                            f"row {row} right half not dark")

    def test_in_band_header_offset_is_skipped(self):
        """Image data starts at pBuffer + usHeader, not at pBuffer."""
        w, h = 32, 16

        def fill(view):
            view[:, :] = 40000

        lib = self._install(width=w, height=h, channels=1, elem_bytes=2,
                            header=64, fill=fill)
        be = self._open(lib)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (h, w, 3))
        # A uniform source must decode uniform; reading at the wrong offset
        # would drag zeroed header bytes into the first row.
        self.assertEqual(int(frame[:, :, 0].min()), int(frame[:, :, 0].max()))

    def test_colour_24bit_frame_passes_through(self):
        lib = self._install(width=32, height=16, channels=3, elem_bytes=1,
                            depth=8)
        be = self._open(lib)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (16, 32, 3))
        self.assertEqual(frame.dtype, np.uint8)

    def test_four_channel_frame_drops_alpha(self):
        lib = self._install(width=16, height=8, channels=4, elem_bytes=1,
                            depth=8)
        be = self._open(lib)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (8, 16, 3))

    def test_frame_geometry_is_taken_from_the_frame_not_assumed(self):
        """The reported size follows the SDK's own header fields."""
        lib = self._install(width=100, height=50)
        be = self._open(lib)
        self._first_frame(be)
        self.assertEqual(be.get_resolution(), (100, 50))

    def test_read_returns_a_copy(self):
        lib = self._install(width=16, height=8)
        be = self._open(lib)
        a = self._first_frame(be)
        a[:] = 7
        ok, b = be.read()
        self.assertTrue(ok)
        self.assertFalse(np.array_equal(a, b),
                         "read() handed out a reference to its own buffer")


# ════════════════════════════════════════════════════════════════════
#  Exposure units — the µs (app) ↔ ms (SDK) boundary
# ════════════════════════════════════════════════════════════════════

class TestExposureUnits(_TUCamFixture):

    def test_get_exposure_converts_ms_to_us(self):
        lib = self._install()
        lib.props[tb.TUIDP_EXPOSURETM][4] = 25.0          # 25 ms
        be = self._open(lib)
        self.assertAlmostEqual(be.get_exposure_time(), 25_000.0, places=3)

    def test_put_exposure_converts_us_to_ms(self):
        lib = self._install()
        be = self._open(lib)
        self.assertTrue(be.put_exposure_time(50_000))     # 50 ms
        self.assertAlmostEqual(lib.props[tb.TUIDP_EXPOSURETM][4], 50.0,
                               places=6)

    def test_exposure_round_trips(self):
        lib = self._install()
        be = self._open(lib)
        be.put_exposure_time(12_345.0)
        self.assertAlmostEqual(be.get_exposure_time(), 12_345.0, places=3)

    def test_reported_range_is_the_sdk_range_in_us(self):
        lib = self._install()
        lib.props[tb.TUIDP_EXPOSURETM][:3] = [0.5, 2000.0, 30.0]   # ms
        be = self._open(lib)
        lo, hi, dft = be.get_exposure_time_range()
        self.assertAlmostEqual(lo, 500.0)
        self.assertAlmostEqual(hi, 2_000_000.0)
        self.assertAlmostEqual(dft, 30_000.0)

    def test_put_exposure_rejects_garbage(self):
        lib = self._install()
        be = self._open(lib)
        self.assertFalse(be.put_exposure_time("bright"))
        self.assertFalse(be.put_exposure_time(None))


# ════════════════════════════════════════════════════════════════════
#  Property gating — an unverified ID must fail closed
# ════════════════════════════════════════════════════════════════════

class TestPropertyGating(_TUCamFixture):

    def test_unsupported_property_reads_none_and_writes_nothing(self):
        """The module's property IDs are not hardware-verified, so an ID this
        camera lacks must degrade to 'unavailable' — never to a wrong write."""
        lib = self._install(props={tb.TUIDP_EXPOSURETM:
                                   [0.1, 1000.0, 30.0, 0.1, 30.0]})
        be = self._open(lib)
        self.assertIsNone(be.get_contrast())
        self.assertIsNone(be.get_brightness())
        self.assertFalse(be.put_contrast(50))
        self.assertFalse(be.put_brightness(10))
        self.assertEqual(
            [pid for pid, _ in lib.prop_writes if pid != tb.TUIDP_EXPOSURETM],
            [], "wrote to a property the camera does not implement")

    def test_unsupported_property_range_is_none(self):
        lib = self._install(props={})
        be = self._open(lib)
        self.assertIsNone(be.get_gamma_range())
        self.assertIsNone(be.get_contrast_range())
        self.assertIsNone(be.get_exposure_time_range())

    def test_setter_clamps_into_the_sdk_declared_range(self):
        lib = self._install()
        lib.props[tb.TUIDP_GLOBALGAIN][:3] = [0.0, 10.0, 0.0]
        be = self._open(lib)
        self.assertTrue(be.put_exposure_gain(999))
        self.assertLessEqual(lib.props[tb.TUIDP_GLOBALGAIN][4], 10.0)
        self.assertTrue(be.put_exposure_gain(-50))
        self.assertGreaterEqual(lib.props[tb.TUIDP_GLOBALGAIN][4], 0.0)

    def test_auto_exposure_via_capability(self):
        lib = self._install()
        be = self._open(lib)
        self.assertTrue(be.set_auto_exposure(False))
        self.assertIs(be.get_auto_exposure(), False)
        self.assertTrue(be.set_auto_exposure(True))
        self.assertIs(be.get_auto_exposure(), True)

    def test_diagnostics_lists_only_implemented_ids(self):
        lib = self._install()
        be = self._open(lib)
        text = be.diagnostics()
        self.assertIn("TUCam diagnostics", text)
        self.assertIn("Libra 25", text)
        self.assertIn("PROPERTIES", text)
        self.assertIn("CAPABILITIES", text)
        # Scope to the PROPERTIES block: the two sections both format ids as
        # "  N:", so an unscoped check can be satisfied by a capability.
        props = text.split("PROPERTIES", 1)[1].split("CAPABILITIES", 1)[0]
        self.assertIn(f"  {tb.TUIDP_EXPOSURETM:>3}:", props)
        # A property the camera does not implement must not be listed.
        self.assertNotIn(f"  {tb.TUIDP_BLACKLEVEL:>3}:", props)


# ════════════════════════════════════════════════════════════════════
#  Resolution — discovered, not assumed
# ════════════════════════════════════════════════════════════════════

class TestResolution(_TUCamFixture):

    def test_resolution_list_discovered_from_sdk_value_text(self):
        lib = self._install(resolutions=("1920x1080", "960x540", "640x480"))
        be = self._open(lib)
        self.assertEqual(be.get_resolution_list(),
                         [(1920, 1080), (960, 540), (640, 480)])

    def test_auto_picks_largest_within_preview_cap(self):
        lib = self._install(resolutions=("4096x4096", "1024x1024", "512x512"))
        be = self._open(lib)
        # 1024 is the largest at or under the 1280 preview cap.
        self.assertEqual(be.get_resolution(), (1024, 1024))

    def test_all_resolutions_above_cap_picks_smallest(self):
        lib = self._install(resolutions=("4096x4096", "2048x2048"))
        be = self._open(lib)
        self.assertEqual(be.get_resolution(), (2048, 2048))

    def test_explicit_resolution_index_honoured(self):
        lib = self._install(resolutions=("1920x1080", "960x540", "640x480"))
        be = self._open(lib, resolution_index=0)
        self.assertEqual(be.get_resolution(), (1920, 1080))
        self.assertEqual(be.get_eSize(), 0)

    def test_set_resolution_index_restarts_the_stream(self):
        lib = self._install(resolutions=("1920x1080", "960x540", "640x480"))
        be = self._open(lib)
        self._first_frame(be)
        starts, allocs = lib.cap_starts, lib.buf_allocs
        self.assertTrue(be.set_resolution_index(0))
        self.assertEqual(be.get_resolution(), (1920, 1080))
        # Buffer is sized for the old geometry, so it must be rebuilt.
        self.assertEqual(lib.cap_stops, 1)
        self.assertEqual(lib.buf_releases, 1)
        self.assertEqual(lib.cap_starts, starts + 1)
        self.assertEqual(lib.buf_allocs, allocs + 1)
        frame = self._first_frame(be)
        self.assertEqual(frame.shape, (1080, 1920, 3))

    def test_no_resolution_capability_falls_back_to_frame_size(self):
        """A camera whose capabilities don't describe resolutions still streams."""
        lib = self._install(width=800, height=600)   # no resolutions supplied
        self.assertNotIn(lib.res_capa_id, lib.capas)
        be = self._open(lib)
        self._first_frame(be)
        self.assertEqual(be.get_resolution_list(), [(800, 600)])
        self.assertIsNone(be.get_eSize())
        self.assertFalse(be.set_resolution_index(1))


# ════════════════════════════════════════════════════════════════════
#  Mono display scaling (shared with the Andor path)
# ════════════════════════════════════════════════════════════════════

class TestMonoDisplayScaling(_TUCamFixture):

    def test_shares_the_andor_conversion_function(self):
        """One implementation, so a Libra/Zyla A/B compares sensors not code."""
        from gui.widgets import andor_backend, mono_display
        self.assertIs(tb._mono_to_bgr8, mono_display._mono_to_bgr8)
        self.assertIs(andor_backend._mono_to_bgr8, mono_display._mono_to_bgr8)

    def test_auto_scale_default_on(self):
        lib = self._install()
        be = self._open(lib)
        self.assertTrue(be.get_display_auto_scale())

    def test_toggle_off_freezes_last_auto_levels(self):
        lib = self._install(width=32, height=16)
        be = self._open(lib)
        self._first_frame(be)
        self.assertIsNotNone(be._last_auto_levels)
        be.set_display_auto_scale(False)
        lo, hi = be.get_display_levels()
        self.assertGreater(hi, lo, "frozen levels must stay a valid window")

    def test_manual_levels_clamp_and_stay_ordered(self):
        lib = self._install()
        be = self._open(lib)
        be.set_display_auto_scale(False)
        self.assertTrue(be.put_display_black(500))
        self.assertTrue(be.put_display_white(4000))
        self.assertEqual(be.get_display_levels(), (500, 4000))
        # White below black must not invert the window.
        be.put_display_white(100)
        lo, hi = be.get_display_levels()
        self.assertGreater(hi, lo)
        # Out-of-range clamps to the 16-bit span.
        be.put_display_black(-5)
        be.put_display_white(999_999)
        lo, hi = be.get_display_levels()
        self.assertGreaterEqual(lo, 0)
        self.assertLessEqual(hi, 65535)

    def test_manual_levels_reject_garbage(self):
        lib = self._install()
        be = self._open(lib)
        self.assertFalse(be.put_display_black("dark"))
        self.assertFalse(be.put_display_white(None))

    def test_manual_levels_change_the_rendered_image(self):
        def fill(view):
            view[:, :] = 800
        lib = self._install(width=32, height=16, fill=fill)
        be = self._open(lib)
        self._first_frame(be)
        be.set_display_auto_scale(False)
        be.put_display_black(0)
        be.put_display_white(65535)
        dark = self._fresh(be)
        be.put_display_black(700)
        be.put_display_white(900)
        bright = self._fresh(be)
        self.assertLess(int(dark[:, :, 0].mean()), int(bright[:, :, 0].mean()),
                        "display levels had no effect on the rendered frame")

    def _fresh(self, be, timeout=3.0):
        """Wait for a frame produced AFTER this call."""
        with be._lock:
            be._frame = None
        be._frame_ready.clear()
        return self._first_frame(be, timeout)


# ════════════════════════════════════════════════════════════════════
#  Readback + API refcounting
# ════════════════════════════════════════════════════════════════════

class TestSettingsReadback(_TUCamFixture):

    def test_get_settings_reports_device_values(self):
        lib = self._install(model=b"Libra 25")
        lib.props[tb.TUIDP_EXPOSURETM][4] = 15.0
        be = self._open(lib)
        self._first_frame(be)
        st = be.get_settings()
        self.assertEqual(st["model"], "Libra 25")
        self.assertEqual(st["device_id"], "0")
        self.assertAlmostEqual(st["exposure_us"], 15_000.0, places=3)
        self.assertEqual(st["channels"], 1)
        self.assertEqual(st["elem_bytes"], 2)

    def test_settings_expose_both_mono_and_legacy_andor_keys(self):
        """`andor_*` are the historical names of the shared display-scale
        channel; the settings dialog and hw_controls persistence gate on them."""
        lib = self._install()
        be = self._open(lib)
        st = be.get_settings()
        for k in ("mono_auto_scale", "mono_scale_lo", "mono_scale_hi",
                  "andor_auto_scale", "andor_scale_lo", "andor_scale_hi"):
            self.assertIn(k, st)
        self.assertEqual(st["mono_auto_scale"], st["andor_auto_scale"])
        self.assertEqual(st["mono_scale_lo"], st["andor_scale_lo"])


class TestApiRefcounting(_TUCamFixture):

    def test_two_cameras_share_one_api_init(self):
        """Api_Init is process-global; Uninit while another camera streams would
        kill that stream, so the reference is counted."""
        lib = self._install(cam_count=2)
        a = TUCamBackend()
        b = TUCamBackend()
        self.addCleanup(a.release)
        self.addCleanup(b.release)
        self.assertTrue(a.open("0"))
        self.assertTrue(b.open("1"))
        self.assertEqual(lib.api_inits, 1)
        self.assertEqual(lib.api_uninits, 0)
        a.release()
        self.assertEqual(lib.api_uninits, 0, "Uninit'd while a camera was open")
        self.assertTrue(b.isOpened())
        b.release()
        self.assertEqual(lib.api_uninits, 1)


# ════════════════════════════════════════════════════════════════════
#  Identity + CameraWidget wiring
# ════════════════════════════════════════════════════════════════════

class TestIdentityWiring(unittest.TestCase):

    def test_identity_round_trip(self):
        from gui.widgets import camera_identity as ci
        ident, name = ci.identity_for_source(("tucam", "0"), [])
        self.assertEqual(ident, "tucam:0")
        self.assertEqual(name, "Tucsen Camera")
        self.assertEqual(ci.source_for_identity("tucam:0", []), ("tucam", "0"))

    def test_identity_is_distinct_from_other_backends(self):
        from gui.widgets import camera_identity as ci
        self.assertNotEqual(ci.identity_for_source(("tucam", "0"), [])[0],
                            ci.identity_for_source(("andor", "0"), [])[0])


class TestHardwareConfirmedMapping(_TUCamFixture):
    """Pins the facts measured on the real Libra 25 (2026-08-04).

    The original property table was wrong in a way that mattered: it put
    EXPOSURETM at 4, which is TEMPERATURE, so "set exposure" would have written a
    cooling setpoint. The GetAttr gate could not catch it because property 4
    exists. These tests exist so that mapping cannot silently regress.
    """

    def test_exposure_and_temperature_are_different_properties(self):
        self.assertNotEqual(tb.TUIDP_EXPOSURETM, tb.TUIDP_TEMPERATURE)

    def test_property_enum_is_sequential_as_measured(self):
        self.assertEqual(
            (tb.TUIDP_GLOBALGAIN, tb.TUIDP_EXPOSURETM, tb.TUIDP_BRIGHTNESS,
             tb.TUIDP_BLACKLEVEL, tb.TUIDP_TEMPERATURE, tb.TUIDP_GAMMA,
             tb.TUIDP_CONTRAST),
            (0, 1, 2, 3, 4, 8, 9))

    def test_auto_exposure_capability_is_the_boolean_one(self):
        """Capability 3 (range 0..1) drives auto-exposure; 8 is auto-LEVELS.

        Confirmed on hardware by consequence: enabling 3 made the exposure
        property self-adjust; enabling 8 left it pinned.
        """
        self.assertEqual(tb.TUIDC_ATEXPOSURE, 3)
        self.assertNotEqual(tb.TUIDC_ATEXPOSURE, tb.TUIDC_ATLEVELS)

    def test_setting_exposure_never_touches_temperature(self):
        lib = self._install(props={
            tb.TUIDP_EXPOSURETM: [0.0063, 5.75e6, 5.23, 0.0063, 5.23],
            tb.TUIDP_TEMPERATURE: [500.0, 1000.0, 500.0, 1.0, 0.375],
        })
        be = self._open(lib)
        self.assertTrue(be.put_exposure_time(30_000))
        written = [pid for pid, _ in lib.prop_writes]
        self.assertIn(tb.TUIDP_EXPOSURETM, written)
        self.assertNotIn(tb.TUIDP_TEMPERATURE, written,
                         "exposure write landed on the temperature property")

    def test_temperature_outside_declared_range_reports_unknown(self):
        """The Libra 25 declares 500..1000 but reads 0.375 — don't invent a °C."""
        lib = self._install(props={
            tb.TUIDP_TEMPERATURE: [500.0, 1000.0, 500.0, 1.0, 0.375],
        })
        be = self._open(lib)
        self.assertIsNone(be.get_temperature())

    def test_temperature_inside_declared_range_is_reported(self):
        lib = self._install(props={
            tb.TUIDP_TEMPERATURE: [-50.0, 50.0, 0.0, 0.1, -12.5],
        })
        be = self._open(lib)
        self.assertAlmostEqual(be.get_temperature(), -12.5)

    def test_redundant_capability_write_is_not_sent(self):
        """🐞 On a real Libra 25, writing auto_exposure=0 while it was ALREADY 0
        reset the exposure to the 6.3 us sensor minimum — a black preview.

        `hardware_setup._apply_hw_controls` restores auto_exposure from persisted
        hw_controls on every camera start, and that value came FROM the camera,
        so it normally matches — which would have blacked out the live view on
        every startup. A no-change write must therefore never reach the SDK.
        """
        lib = self._install()
        be = self._open(lib)
        lib.capa_writes.clear()

        # Current value is 1 (the fake's default); writing 1 must be skipped...
        self.assertTrue(be.set_auto_exposure(True))
        self.assertEqual(lib.capa_writes, [],
                         "a no-change capability write reached the SDK")
        # ...while a real transition must go through.
        self.assertTrue(be.set_auto_exposure(False))
        self.assertEqual(lib.capa_writes, [(tb.TUIDC_ATEXPOSURE, 0)])
        self.assertIs(be.get_auto_exposure(), False)

        # And re-asserting the new value is skipped too.
        lib.capa_writes.clear()
        self.assertTrue(be.set_auto_exposure(False))
        self.assertEqual(lib.capa_writes, [])

    def test_restoring_the_persisted_auto_exposure_value_is_a_no_op(self):
        """The exact shape of the startup restore that triggered the bug."""
        lib = self._install()
        be = self._open(lib)
        lib.capa_writes.clear()
        be.set_auto_exposure(be.get_auto_exposure())   # persisted == current
        self.assertEqual(lib.capa_writes, [])

    def test_binning_labels_are_not_mistaken_for_resolutions(self):
        """Capability 37 on the Libra 25 is labelled '1x1Normal'/'2x2Bin_Sum'.

        A looser WxH pattern would happily read those as 1x1 and 2x2 and offer
        them as capture resolutions.
        """
        self.assertIsNone(tb._dims_from_text("1x1Normal"))
        self.assertIsNone(tb._dims_from_text("2x2Bin_Sum"))
        # ...while the real resolution labels, which carry a suffix too, parse.
        self.assertEqual(tb._dims_from_text("5200x4096(Resolution)"), (5200, 4096))
        self.assertEqual(tb._dims_from_text("2600x2048(Sensitive)"), (2600, 2048))

    def test_inf_string_cleanup(self):
        """Windows stores the model as an INF reference; the name is last."""
        self.assertEqual(
            tb._clean_inf_string(
                "@oem81.inf,%vid_5453&pid_e437.devicedesc%;Libra 25"),
            "Libra 25")
        self.assertEqual(tb._clean_inf_string("Libra 25"), "Libra 25")
        self.assertEqual(tb._clean_inf_string(""), "")

    def test_model_falls_back_to_the_os_name(self):
        """This SDK build returns no model text, so the OS supplies the label."""
        lib = self._install(cam_count=1)
        # The fake answers the SDK text id only when given an open handle;
        # enumerate() has none, so the OS name must be used.
        self.assertEqual(TUCamBackend.enumerate()[0]["displayname"], "Libra 25")

    def test_model_falls_back_to_index_when_nothing_knows(self):
        self.os_names = []
        self.addCleanup(lambda: setattr(self, "os_names", ["Libra 25"]))
        lib = self._install(cam_count=1)
        self.assertEqual(TUCamBackend.enumerate()[0]["displayname"], "Tucsen #0")

    def test_settings_report_unknown_frame_format_before_first_frame(self):
        """'0 ch, 0 byte/px' would read like a real answer in the readout."""
        lib = self._install()
        be = TUCamBackend()
        self.addCleanup(be.release)
        be._lib = lib
        st = be.get_settings()
        self.assertIsNone(st["channels"])
        self.assertIsNone(st["elem_bytes"])


class TestSourceListVisibility(_TUCamFixture):
    """Does a Tucsen camera actually SHOW UP as a selectable source?

    This pins the operator-visible symptom end to end: Hardware Setup ->
    Cameras builds its per-slot source combos from
    ``CameraManager.available_sources``, which is derived from the camera
    widget's combo, which is populated from the detection probe. A break
    anywhere along that chain shows up as "there is no Tucsen option", which is
    indistinguishable at a glance from "no camera is attached" — so both states
    are asserted here.
    """

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    @staticmethod
    def _sources(mgr):
        s = mgr.available_sources
        return s() if callable(s) else s

    def _detect(self):
        from gui.widgets.camera_manager import CameraManager
        mgr = CameraManager()
        # opencv_indices=[] skips the slow real UVC probe; we only care about
        # whether the Tucsen entry is produced.
        mgr.detect_cameras(opencv_indices=[])
        return self._sources(mgr)

    def test_no_tucsen_source_when_no_camera_attached(self):
        """With 0 cameras the SDK reports nothing, so no option is offered.

        This is the expected state when the camera is unplugged or its driver
        is not installed — NOT a wiring fault.
        """
        self._install(cam_count=0)
        data = [d for _, d in self._detect()]
        self.assertFalse(
            [d for d in data if isinstance(d, (tuple, list)) and d[0] == "tucam"],
            "offered a Tucsen source with no camera attached")

    def test_tucsen_source_appears_when_a_camera_is_present(self):
        lib = self._install(cam_count=1, model=b"Libra 25")
        srcs = self._detect()
        tucsen = [(t, d) for t, d in srcs
                  if isinstance(d, (tuple, list)) and d[0] == "tucam"]
        self.assertEqual(len(tucsen), 1,
                         f"no Tucsen source offered; got {[t for t, _ in srcs]}")
        label, data = tucsen[0]
        self.assertIn("Libra 25", label, "label should carry the SDK's model name")
        self.assertEqual(data, ("tucam", "0"))

    def test_multiple_tucsen_cameras_each_get_a_source(self):
        self._install(cam_count=2, model=b"Libra 25")
        data = [d for _, d in self._detect()
                if isinstance(d, (tuple, list)) and d[0] == "tucam"]
        self.assertEqual(data, [("tucam", "0"), ("tucam", "1")])


class TestCameraWidgetWiring(_TUCamFixture):
    """The widget-level contract, with the real backend behind the fake SDK."""

    @classmethod
    def setUpClass(cls):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _widget(self, lib):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(camera_label="Test", show_controls=False)
        self.addCleanup(w.stop)
        w._start_tucam("0")
        # is_running is a property on CameraWidget, not a method.
        self.assertTrue(w.is_running, "widget did not start the Tucsen backend")
        return w

    def test_start_adopts_the_tucam_backend(self):
        lib = self._install()
        w = self._widget(lib)
        self.assertEqual(w._backend_type, "tucam")
        self.assertIsNotNone(w._tucam)

    def test_capabilities_report_tucam_source(self):
        lib = self._install(model=b"Libra 25")
        w = self._widget(lib)
        caps = w.hardware_capabilities()
        self.assertEqual(caps["source"], "tucam")
        self.assertTrue(caps["controllable"])
        self.assertEqual(caps["device_name"], "Libra 25")
        self.assertIn("exposure_us", caps["controls"])
        # Mono display-scale controls are offered, as for the Zyla.
        for k in ("andor_auto_scale", "andor_scale_lo", "andor_scale_hi"):
            self.assertIn(k, caps["controls"])

    def test_capabilities_omit_unimplemented_controls(self):
        lib = self._install(props={tb.TUIDP_EXPOSURETM:
                                   [0.1, 1000.0, 30.0, 0.1, 30.0]})
        w = self._widget(lib)
        ctrls = w.hardware_capabilities()["controls"]
        self.assertIn("exposure_us", ctrls)
        self.assertNotIn("contrast", ctrls)
        self.assertNotIn("brightness", ctrls)

    def test_get_hw_settings_labels_the_source(self):
        lib = self._install()
        w = self._widget(lib)
        st = w.get_hw_settings()
        self.assertEqual(st["source"], "tucam")

    def test_hw_setters_reach_the_backend(self):
        lib = self._install()
        w = self._widget(lib)
        self.assertTrue(w.set_hw_exposure_us(20_000))
        self.assertAlmostEqual(lib.props[tb.TUIDP_EXPOSURETM][4], 20.0,
                               places=6)
        self.assertTrue(w.set_hw_exposure_gain(5))
        self.assertTrue(w.set_hw_gamma(120))

    def test_display_scale_delegates_reach_the_backend(self):
        lib = self._install()
        w = self._widget(lib)
        self.assertTrue(w.set_hw_andor_auto_scale(False))
        self.assertTrue(w.set_hw_andor_scale_lo(300))
        self.assertTrue(w.set_hw_andor_scale_hi(9000))
        self.assertEqual(w._tucam.get_display_levels(), (300, 9000))

    def test_display_scale_delegates_false_when_not_a_mono_camera(self):
        from gui.widgets.camera_widget import CameraWidget
        w = CameraWidget(camera_label="Test", show_controls=False)
        self.addCleanup(w.stop)
        self.assertFalse(w.set_hw_andor_auto_scale(True))
        self.assertFalse(w.set_hw_andor_scale_lo(0))

    def test_stop_releases_the_stream_and_resets_backend(self):
        lib = self._install()
        w = self._widget(lib)
        w.stop()
        self.assertEqual(lib.closes, 1)
        self.assertIsNone(w._tucam)
        self.assertEqual(w._backend_type, "opencv",
                         "stale backend type would mislabel the next source")

    def test_set_capture_resolution_picks_nearest(self):
        lib = self._install(resolutions=("1920x1080", "960x540", "640x480"))
        w = self._widget(lib)
        actual = w.set_capture_resolution(960, 540)
        self.assertEqual(actual, (960, 540))

    def test_grab_frame_publishes_a_raw_frame(self):
        lib = self._install(width=32, height=16)
        w = self._widget(lib)
        deadline = time.time() + 3.0
        while time.time() < deadline and w.get_current_frame() is None:
            w._grab_frame()
            time.sleep(0.01)
        frame = w.get_current_frame()
        self.assertIsNotNone(frame, "no frame reached the widget's raw cache")
        self.assertEqual(frame.shape, (16, 32, 3))

    def test_log_hw_settings_includes_tucam(self):
        lib = self._install()
        w = self._widget(lib)
        st, text = w.log_hw_settings()
        self.assertEqual(st["source"], "tucam")
        self.assertIn("display scale", text)
        self.assertNotIn("no controllable camera", text)


if __name__ == "__main__":
    unittest.main()
