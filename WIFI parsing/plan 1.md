Remote Compute Link — send microscope images to a second computer, get results back
Context
MEBP runs everything in one Python process on one machine: camera capture, OpenCV analysis, mosaic stitching, and all motion control. Several workloads are genuinely heavy — spheroid detection on a 12 000 px mosaic is seconds of OpenCV, mosaic registration holds hundreds of MB — and the planned spheroid classifier needs a GPU this machine does not have.

A second computer is available in the same room. The goal is to send microscope images to it, run processing there (GPU/ML inference, heavy OpenCV, and existing third-party tools such as MATLAB/ImageJ), and get results — including processed images — back into MEBP for display.

Operator constraints, confirmed:

University Wi-Fi (UAWiFi); no private router permitted.
Same room, but a cable between the machines is not practical.
On-demand only — the operator triggers each job. No continuous frame stream.
The far end runs a Python worker script.
Admin on both machines; a ~$20 USB Wi-Fi adapter is acceptable; new Python dependencies are acceptable.
The "on-demand only" answer is what shapes this design. It removes the hardest part of the problem — a continuous stream would have needed drop-stale-frame backpressure, ack gating, and careful bandwidth budgeting. A request/response job channel needs none of that.

Verified facts
Network (measured on this machine)
Fact	Value	Consequence
Ethernet	Intel I226-V 2.5 GbE, disconnected	Unused; fastest option if a cable ever becomes acceptable
Wi-Fi	Intel AX211 Wi-Fi 6E, 5 GHz channel 149, 802.11ax, 80% signal, TX 459 / RX 344 Mbps PHY	Practical TCP ≈ 200–270 Mbps one-way before contention
IP / subnet	10.134.186.83/18 on UAWiFi 2, gateway 10.134.128.1	~16 000 hosts share this subnet
Firewall profile	Public, enabled	Inbound blocked by default
ARP neighbour cache	4 on-subnet peers all Unreachable, all-zero MAC; only the gateway resolves	Strong evidence of wireless client isolation
SoftAP capability	Two Microsoft Wi-Fi Direct Virtual Adapter interfaces; icssvc (Mobile Hotspot) present, Manual/Stopped; SharedAccess running	Mobile Hotspot is available
The most consequential finding is client isolation. Four distinct peers failed to ARP while the gateway answered. If that holds, two machines on UAWiFi cannot reach each other at all and no application code fixes it. Confirm against the actual second machine before writing code — it is a two-minute test and it decides the link.

Codebase
Fact	Location	Consequence
Zero networking code exists	socket appears once, for gethostname() in a log line (SupportClasses/XYStage.py:26,295)	Greenfield
Camera grab	QTimer, 15 fps default (gui/widgets/camera_widget.py:236,690)	—
Latest-frame box	_current_frame is rebound, never mutated, under _frame_lock, with monotonic _frame_seq (camera_widget.py:1258-1260)	A handed-out array is immutable by construction
GUI-thread cost is a known hazard	set_throttled docstring (camera_widget.py:1345-1353) records that 15 fps of GUI-thread conversion "can starve the event loop"	Never encode on the GUI thread
Andor converts mono16 → BGR8 inside its reader thread	gui/widgets/andor_backend.py:458,469	No 16-bit data exists downstream today. The "lossless 16-bit" problem is moot for v1
cv2.imencode releases the GIL (measured)	Encoding 3664×2748 in a thread left another thread's scheduling p95 at 1.69 ms vs 1.55 ms idle — statistically zero	A plain threading.Thread costs the Qt loop nothing
numpy pinned ==2.4.6	requirements.txt:5 (capped by numba ← pylablib ← Andor Zyla)	Any new dependency must not drag numpy forward
Measured payload sizes (real mosaic tiles, cv2 5.0.0)
Payload	Encode	Size	On a 200 Mbps link
1832×1374 tile, JPEG q85	5.9 ms	318 KiB	13 ms
3664×2748 full-res, JPEG q85	18 ms	~1.2 MB	48 ms
6000² mosaic, PNG lossless	1.65 s	38 MB	~1.5 s
For on-demand jobs, bandwidth is a non-issue. A single tile round-trips in well under a second; the worst realistic case is a large mosaic at a couple of seconds.

Decision 1 — the link
The link and the protocol are independent decisions. The software must work over any IP route so the link can change without touching MEBP.

Recommended: a private Wi-Fi link between the two machines — one runs Windows Mobile Hotspot / Wi-Fi Direct, the other joins. Private 192.168.137.0/24, no router, no campus involvement, no client isolation. The host can also share its internet connection so the remote box keeps internet.

Add the ~$20 USB Wi-Fi adapter you approved, and put the hotspot on that. On a single radio Windows pins the SoftAP to the channel the client connection already uses — here congested campus channel 149 — so the private link would both time-share the radio and contend with every other device on that AP. A dedicated adapter on a clean channel avoids both, and leaves the built-in AX211 on UAWiFi for internet.

Fallbacks, in order:

Direct TCP over UAWiFi — test first because it costs five minutes, but the ARP evidence says it will fail.
Tailscale / ZeroTier overlay — traverses client isolation and works across buildings, but when hole-punching fails it relays via an internet server. Absurd for two machines in the same room, but it works and needs no policy exception.
Direct 2.5 GbE cable — excluded by you. Recorded only because it is ~10× faster, costs ~$10, and is a drop-in change if the constraint relaxes.
⚠ Check the campus policy on peer-to-peer Wi-Fi before relying on a hotspot. Some universities run wireless intrusion prevention that de-authenticates SoftAPs.

Decision 2 — the protocol: HTTP over the private link, stdlib only
Given on-demand-only traffic, HTTP is the natural fit and needs no new dependency on either machine: http.server.ThreadingHTTPServer on the worker, urllib.request in MEBP.

MEBP is the HTTP client; the worker LISTENS. This is a deliberate inversion with two real benefits: no inbound firewall rule is needed on the print-control machine (the Public-profile inbound block only affects listeners), and the machine driving a printer exposes no network service at all. The one firewall rule lives on the machine that isn't controlling hardware, and the worker binds the hotspot address specifically (192.168.137.1), never 0.0.0.0.

Why HTTP beats the alternatives for this traffic pattern:

HTTP (stdlib)	ZeroMQ (pyzmq)	Raw socket
New dependency	none, either end	pyzmq both ends + PyInstaller collect_all	none
Framing	Content-Length, done for you	done for you	hand-rolled — the desync bug lives here
Reconnect	free (stateless requests)	automatic	hand-rolled backoff
Debuggable	curl / browser	needs custom tooling	hexdump
MATLAB / ImageJ	webread / openStream, no glue	needs JeroMQ jar	manual byte-swapping
Long-job progress	async job + poll (standard REST)	native	hand-rolled
ZeroMQ's decisive advantages — CONFLATE drop-latest semantics and transparent reconnect across a live stream — only matter for streaming, which you ruled out. Without a stream, it is a dependency on two machines buying very little. Raw sockets buy nothing over HTTP and put the highest-risk code (framing, resync, sendall timeouts leaving unknown bytes on the wire) in our hands.

This is not a dead end for streaming. The safety-critical work lives in a transport-independent protocol module. Adding a stream later means an MJPEG endpoint (multipart/x-mixed-replace, ~80 lines of stdlib) or a ZeroMQ PUB channel, reusing the entire metadata and coordinate contract unchanged.

Endpoints
GET  /hello                 → worker identity, handler name+version, capabilities,
                              numpy/cv2/torch versions, protocol version, clock
POST /jobs                  → body = JSON header + binary payload(s); returns 202 + job_id
GET  /jobs/{id}             → {state: queued|running|done|error, progress, message}
GET  /jobs/{id}/result      → JSON results + optional image payload(s)
DELETE /jobs/{id}           → cancel
Async submit-then-poll rather than one blocking request, because a mosaic job runs for tens of seconds. Polling fits MEBP's existing pattern exactly: the GUI already ticks every ~300 ms via _tick → _update_status → page.on_status_update() (gui/app.py:2296,2633), so progress rendering is free and needs no new timer.

Request/response body = Content-Type: application/octet-stream with a small fixed prefix: <4s magic><B version><I header_len><I payload_len>, then UTF-8 JSON, then raw bytes. JSON header overhead is ~450 B against a 318 KiB payload — 0.13% — which buys readability in a hexdump, jsondecode in MATLAB, and diffable tests. Payload stays outside the JSON; base64 would cost +33% and a full extra encode pass for nothing. Pickle is banned — remote code execution on a network service, and Python-only.

The metadata contract (the safety-critical part)
A pixel coordinate is meaningless without the frame's full coordinate context. This repo has already paid for that lesson twice: a mosaic persisted without its applied shift caused a 0.68 mm targeting error, and the fluorescence path once applied rotation but silently lost the flip. Every job header therefore carries complete context, and the far end must be able to tell "measured as zero" from "never measured".

Highest-value design decision: ship a 2×2 affine derived by calling the real mapping function, rather than shipping rotation_deg/flip_x/flip_y and letting the far end re-derive it.

def affine_from_pixel_mapper(mapper, w: int, h: int) -> dict:
    """2x3 affine equivalent to CameraManager.pixel_to_stage_offset.

    Derived BY CALLING the click path, so the far end's one-line µm conversion
    cannot drift from the operator's own click→stage map. pixel_to_stage_offset
    is exactly linear (scale · parity · rotation), so three probe calls recover
    it exactly."""
The far end then does, in any language: stage_um = [x_um, y_um] + A @ (p - origin_px). One line, no convention knowledge, nothing to get wrong. This is the same discipline SpheroidDetector.back_project_px already enforces as "the one and only implementation, pinned against MosaicWellRemap."

Header fields that each prevent a specific known bug:

Field	Why
optics.um_per_px = effective_um_per_px(idx, live_w)	The Andor is calibrated at two resolutions (3.227 @1024, 1.320 @2048). Shipping the stored value is a documented 2× error
optics.um_per_px_calibrated	The 1.67 seed default is > 0 and passes a naive check. Mirrors is_um_per_px_calibrated (camera_manager.py:462)
stage.valid	get_xy_position returns (None, None, None) when disconnected. None must never serialize to 0.0
stage.age_s	Position comes from a ~300 ms poller. During a 5 mm/s jog that is 1.5 mm of staleness
orientation.applied_to_pixels: false	Frames are RAW by design (camera_widget.py:1254-1257). Stated explicitly so a wrong assumption violates a checkable contract
mosaic.shift_um + shift_recorded	Exactly the field whose omission caused the 0.68 mm error. shift_recorded: false ⇒ coordinates are not motion-safe
coord_frame on results — required, no default	A missing coordinate frame must be a hard error, never an assumed identity
Shared validator, used by both ends:

def validate_frame_header(hdr: dict) -> list[str]:
    """Refusal reasons for converting pixels → stage µm. [] means usable.
    e.g. ['µm/px is the uncalibrated 1.67 seed default',
          'stage position unavailable (XY disconnected)',
          'stage position is 0.9 s old']"""
Safety invariants
A remote result can never command motion. Mirror the v7.8 precedent exactly: a new PROV_REMOTE alongside PROV_LIVE/PROV_MOSAIC/PROV_CONFIRMED, drawn dashed, upgraded to PROV_CONFIRMED only by a live operator click. Config result_motion_policy defaults to "never". Extend the existing dashed-ring test at gui/widgets/live_target_picker.py:474.
Hard refusal, never a warning, for any result with motion_safe: false, missing coord_frame, um_per_px_calibrated: false, or mosaic.shift_recorded: false — surfaced as an operator-readable string using the existing refuse_reason / DetectionReport.summary() idiom so the operator learns why.
The link is an accessory that can never stall a print. No PrintManager import, no callback registration, no shared lock. Every socket has a timeout; every thread is daemon=True; closeEvent stops it best-effort inside try/except.
Structural, not conventional, isolation. ComputeLinkClient.__init__ takes plain callables, never objects — it never holds CameraManager, StageController, or PrintManager. It is not a rule that it can't touch hardware; it has no reference through which to do so.
No Qt in SupportClasses/, no sockets on the GUI thread. Status reaches the GUI as an immutable frozen dataclass polled on the existing ~300 ms tick — the MicroscopeState pattern (SupportClasses/MicroscopeControl.py:79,1258) — never a cross-thread callback that could fire into a deleted Qt object.
The shared token is anti-misdirection, not security. It stops you sending into the wrong machine on a shared subnet. The docstring must say so, and the link must not carry sensitive data over an untrusted network.
New files
Path	Contents	Qt?
SupportClasses/ComputeLinkProtocol.py	framing, codecs, build_job_header, affine_from_pixel_mapper, validate_frame_header	no
SupportClasses/ComputeLinkClient.py	HTTP client, worker thread, frozen LinkState, job submit/poll	no
SupportClasses/ComputeLinkConfigStore.py	config/hardware/compute_link.json, $MEBP_COMPUTE_LINK_DIR, atomic write, get_store()	no
gui/compute_link_source.py	make_image_provider / make_meta_provider — the only place that knows CameraManager	no widgets
gui/widgets/compute_link_panel.py	status card; on_status_update() renders the snapshot	yes
gui/dialogs/compute_link_settings_dialog.py	host/port/token/quality/handler — nothing persisted until OK	yes
tools_compute_worker.py	the far-end program	no
tools_compute_link_check.py	bench diagnostic: /hello, RTT, throughput, semantic round-trip; prints the exact netsh advfirewall rule without running it	no
tests/test_v79_compute_link_protocol.py	framing round-trip, oversize/desync, uint16 PNG round-trip	—
tests/test_v79_compute_link_metadata.py	affine vs pixel_to_stage_offset parity sweep, refusal flags, None ≠ 0	—
tests/test_v79_compute_link_safety.py	PROV_REMOTE dashed; missing coord_frame never reaches a pick; shift_recorded:false refuses	—
Config store goes in config/hardware/, not HardwareConfig — for exactly the reason SupportClasses/MicroscopeConfigStore.py's docstring gives (lines 4-9): a hardware-setup file loaded from another machine must never carry another rig's machine-specific values. An IP address and a shared token are properties of this rig in this room.

Far-end handler interface — the whole extension surface
def process(images: list["np.ndarray"], meta: dict,
            progress: Callable[[float, str], None]) -> dict:
    """BGR uint8 in, plain dict out.
    {"coord_frame": "frame_px" | "stage_um" | "mosaic_px",   # REQUIRED
     "detections": [{"px":[x,y], "r_px":…, "label":…, "score":…}, …],
     "images": [("overlay", ndarray, "jpeg")],   # optional, sent back
     "notes": str}
    torch/cupy/whatever is the handler's business — the transport imports none."""
Three shipped handlers, selected by --handler:

echo (default) — returns geometry round-trip plus decoded shape/dtype/checksum. This is the verification handler: it proves link, codec and coordinate math with no ML installed, and is the first thing anyone runs.
opencv_spheroid — returns dicts shaped exactly like SpheroidDetector.SpheroidDetection.to_dict() so results are drop-in compatible with the existing survey panel. No new result schema invented.
subprocess — --handler subprocess --cmd "matlab -batch run_handler". Writes payload + header to a temp dir (temp + os.replace, so a half-written file is never read), runs the command, reads back JSON + optional images. This is how MATLAB/ImageJ participate with zero networking code on their side.
tools_compute_worker.py must not import MEBP — it runs on a machine with torch and no MEBP checkout. It carries an embedded copy of the codec, and a test forces both the embedded and SupportClasses copies and asserts byte-identical headers, so the duplication is drift-proof rather than aspirational.

Phased implementation
Phase	Work	Existing files touched
0	Bench: confirm client isolation with the real second machine; bring up the hotspot; confirm ping both ways	none
1	ComputeLinkProtocol.py + tests. Framing, codecs, affine, validator. No sockets, no dependency	none
2	ComputeLinkConfigStore.py + tests	none
3	ComputeLinkClient.py + tools_compute_worker.py with echo; loopback on one machine over 127.0.0.1	none
4	tools_compute_link_check.py; prove the two machines talk before touching the GUI	none
5	Real image source: gui/compute_link_source.py; metadata + affine wired to the real pixel_to_stage_offset	camera_manager.py (+1 method)
6	Status panel + settings dialog; construct in app.py, stop in closeEvent; register_section("compute_link", …)	app.py (6 lines), context_sections.py (2 lines)
7	opencv_spheroid + subprocess handlers; result overlay with PROV_REMOTE safety gates	one camera view widget
Phases 0–4 are shippable, testable, and revertible without touching a single existing file. Total edits to existing files across the whole feature: ~4 methods, ~40 lines, in 4 files.

requirements.txt gets a comment block only, in the style of the pymmcore entry — no new dependency on either machine.

Verification
Network (Phase 0, before any code):

From this machine, ping <peer> over UAWiFi. Expect failure — that confirms client isolation and justifies the hotspot.
Bring up Mobile Hotspot on the USB adapter; join from the second machine; ping both directions. Both must succeed.
iperf3 or tools_compute_link_check.py --bandwidth — expect ≥100 Mbps.
Link (Phase 4): 4. python tools_compute_worker.py --bind 192.168.137.1 --handler echo on the far machine; python tools_compute_link_check.py --peer 192.168.137.1 here. Must print handler identity, RTT, and throughput. 5. Semantic round-trip — the test that matters. Send a synthetic frame with a marker at a known pixel; assert the returned center_um matches what MEBP computes locally through CameraManager.pixel_to_stage_offset to within 0.5 µm. Until this passes, the UI shows "connected but unverified" and results are never eligible for motion. A ping proves nothing; this proves the coordinate contract. 6. Clock-skew check — compare timestamps. Two machines with different NTP state make every age gate nonsense (either everything looks stale or nothing ever does). Report DEGRADED above 2 s skew.

Integration (Phases 5–7): 7. Capture a real microscope frame, send it to the echo handler, confirm the returned geometry matches effective_um_per_px for the live resolution — then change the ToupCam eSize and confirm it tracks (this is the 2× scale bug). 8. Run opencv_spheroid on a real well mosaic; confirm detections render dashed as PROV_REMOTE, that "Go to" is disabled for them, and that clicking one live upgrades it to PROV_CONFIRMED. 9. Pull the far machine's power mid-job. MEBP must show an error and stay fully usable; nothing may stall, and no motion may occur.

Tests: python -m unittest discover tests -p "test_v79_compute_link*.py", plus the existing camera/spheroid/live-picker suites as regression.

Explicitly out of scope, and why
Continuous frame streaming. Ruled out by the on-demand answer. The protocol module is transport-independent, so adding MJPEG or a ZeroMQ PUB channel later reuses all the metadata and coordinate work.
16-bit lossless transport. No 16-bit data exists downstream today — andor_backend.py:458 converts mono16 → BGR8 inside the reader thread. Adding it needs a new AndorBackend.read_raw(), a separate change with its own bench verification. The protocol reserves png/uint16 and a test pins the round-trip so the door stays open.
The far end in a control loop. Converts an accessory into a dependency.
Encryption. The token is anti-misdirection only.
Auto-discovery / mDNS. On a hotspot the address is deterministically 192.168.137.1. Type it once.