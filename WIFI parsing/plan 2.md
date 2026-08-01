Remote Real-Time Image Processing Service (multi-client, WiFi-only)
Context
The operator wants camera frames streamed from MEBP machines to a central processing computer for real-time analysis, with results streamed back into the software. Constraints gathered:

All machines are on university WiFi — no personal router allowed, no cables, and WiFi client isolation may block direct device-to-device traffic.
2–3 machines may send frames simultaneously; clients are mixed (MEBP instances and potentially other programs/languages).
Frames must be bit-exact (lossless compression like zstd is acceptable; lossy JPEG is not).
1–5 fps per client; round-trip latency is soft (~0.5–2 s).
Bandwidth reality: a 2048×2048 mono16 frame is 8 MB raw, ~3–4 MB zstd-compressed. At 1–2 fps × 3 clients that's ~50–200 Mbit/s aggregate — feasible on decent WiFi, so per-client pacing and an optional downscale knob are part of the design.

Note: per the operator's instruction, no project files were read during planning. The MEBP-side integration is designed as a standalone new module with one thin attachment point; exact wiring into the camera layer will be confirmed against the real code at implementation time.

Chosen strategy: Central HTTP processing server + Tailscale overlay
Why HTTP (FastAPI/uvicorn) instead of ZeroMQ/WebRTC/MQTT:

"Requests from multiple machines" is literally request/response — POST a frame, get results in the reply. Correlation, multiplexing, and backpressure come free.
Mixed clients: callable from any language, or curl, with zero client libraries.
One TCP port, trivially testable, easy shared-secret auth.
At 1–5 fps with soft latency, HTTP overhead is irrelevant.
Why Tailscale (free tier) on all machines:

University WiFi has two independent problems: client isolation (WiFi↔WiFi traffic often dropped at the AP) and DHCP churn (IPs change). Tailscale gives every machine a stable private IP + MagicDNS name (http://proc-pc:8077/...) and encrypted transport, with no router and nothing installed on the network.
When the network permits direct peer traffic, Tailscale connections run at full LAN speed; when isolation blocks it, traffic falls back to Tailscale's DERP relays — still functional, but bandwidth-limited → the client's fps/downscale knobs are the pressure valve. tailscale status shows direct-vs-relayed per peer, which the verification phase checks explicitly.
Config keeps the server address as a plain string, so if the campus network turns out to allow direct traffic, pointing at the raw LAN IP works with zero code change.
Rejected alternatives
Direct Ethernet — ruled out by the no-cable constraint.
Plain WiFi LAN only — breaks silently under client isolation and IP churn; kept only as a config-level option, not the foundation.
ZeroMQ ROUTER/DEALER — fine technically, but worse fit for mixed-language clients and adds a framing protocol HTTP already provides.
GStreamer/WebRTC video — lossy; disqualified by fidelity requirement.
Cloud relay/MQTT — needless external dependency, wrong bandwidth profile.
API (documented in a README so non-MEBP clients can integrate)
POST /process
Body: zstd-compressed raw frame bytes.
Headers (or multipart JSON part): X-Frame-Meta JSON — frame_id, client_id, shape, dtype, timestamps, optional context (stage XY µm, well, camera role), optional downscale factor applied client-side.
Auth: X-Api-Token shared secret (the service sits on a semi-public network).
Response: JSON — frame_id echoed, processing time, and an application-defined results payload (detections, measurements, …).
GET /health — liveness + server load (lets clients adapt their fps).
Server runs uvicorn with a small worker pool; 2–3 concurrent clients is trivial. The actual CV code plugs into one function: process(frame: np.ndarray, meta) -> dict (ships as an echo/timing stub).

Components to build
Processing PC — remote_processor/ standalone service (new, outside MEBP): FastAPI app + zstd decode → np.frombuffer(...).reshape(shape) → pluggable process() hook → JSON response. Deps: fastapi, uvicorn, zstandard, numpy. Run from a terminal.

MEBP PC — SupportClasses/RemoteProcessingLink.py (new, GUI-free per house convention): a daemon worker thread that

accepts frames via non-blocking submit(frame, context) — drops the frame if a request is already in flight (latest-wins; pacing enforced here, never queued),
zstd-compresses and POSTs, stores the reply as an immutable snapshot exposed via latest_result() for GUI polling (matches the app's established poll-a-snapshot pattern; avoids cross-thread callback hazards),
tracks liveness from /health + request outcomes → a "remote processing: connected / stale / relayed" status string.
Config (e.g. config/remote_processing.json): server URL, token, target fps, downscale, enabled flag — default disabled → zero behavior change.
MEBP integration point (deferred detail — needs file access at implementation): a timer grabbing the current raw cached camera frame at the configured fps → submit(). v1 surfaces results via latest_result() + a log line; where they appear in the UI is a follow-up once the loop works.

Setup README: Tailscale install/login on each machine, token generation, firewall allowance for the server port, curl smoke-test example.

Verification
Loopback (one machine): server + synthetic-frame client on 127.0.0.1 — round trip works, decompressed frame is checksum-identical, in-flight drop logic holds when the hook is artificially slowed.
Reachability decision test (10 min, two machines): try plain-LAN curl to the server first (answers whether the campus WiFi even needs the overlay), then via Tailscale; record tailscale status direct-vs-relayed.
Throughput: 8 MB synthetic mono16 frames from 2 clients concurrently at the target fps; measure achieved fps + RTT; if relayed, confirm the downscale/fps knobs bring it inside budget.
End-to-end: MEBP live (or simulated camera) + one extra client machine hitting the same server concurrently; results correlate by frame_id, GUI stays responsive, killing the server → stale indicator, no crash, auto-recovery when it returns.