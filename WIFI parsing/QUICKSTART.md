# Implementing a LabLink client — receive files, process them, send results back

This is the protocol reference for writing your own client in **any language or
on any device**: MATLAB, ImageJ/Fiji, LabVIEW, an Arduino/ESP32, a microscope
vendor's scripting console, a Raspberry Pi, a web page.

The whole protocol is plain HTTP with a JSON body and raw bytes. If your
environment can do an HTTP request and compute a SHA-256, it can participate.
No client library exists or is needed.

The pattern this document builds toward — a **worker** that watches for new
files, processes them, and returns results — is at the end in §7.

---

## 1. The five calls

Base URL is whatever the server prints at startup, e.g. `http://100.74.228.41:8765`.
Every request except `GET /hello` needs the header `X-Lablink-Token: <token>`.

| Call | Purpose |
|---|---|
| `GET /hello` | Is it alive? Returns version, clock, size limit. No token needed. |
| `GET /c/{channel}` | List files in a channel. Add `?since_seq=N` for "only what's new". |
| `GET /c/{channel}/{name}` | Download. Supports `Range: bytes=N-` for resume. |
| `PUT /c/{channel}/{name}` | Upload. Requires `Content-Length` and `X-Lablink-Sha256`. |
| `DELETE /c/{channel}/{name}` | Remove one file. |

A **channel** is a named mailbox. Use one per direction — e.g. `mebp-out` for
work arriving, `results` for what you send back. Channels are created on first
upload; listing one that has never been used returns an empty list, not an error.

### GET /hello

```bash
curl http://SERVER:8765/hello
```
```json
{"service":"lablink","protocol":1,"version":"1.0.0",
 "time":1785630506.009,"name":"McGheeLab-CellC","max_file_bytes":536870912}
```

Use `time` to check your clock against the server's, and `max_file_bytes` to
reject an oversize file locally before spending the bandwidth.

### GET /c/{channel} — list

```bash
curl -H "X-Lablink-Token: TOKEN" http://SERVER:8765/c/mebp-out
```
```json
{"channel":"mebp-out","seq":7,
 "files":[{"name":"mosaic_A1.png","size":9168091,
           "sha256":"b7544ee3002b…","seq":7,
           "uploaded":1785629736.2,
           "meta":{"plate":"nest-plastic-24","well":"A1"}}]}
```

`seq` is a per-channel counter that only ever increases. **This is how you poll
efficiently:** remember the highest `seq` you have handled, then ask for
`?since_seq=N` and you get only newer files, already sorted by `seq`.
`meta` is whatever JSON the sender attached — possibly `{}`.

### GET /c/{channel}/{name} — download

```bash
curl -H "X-Lablink-Token: TOKEN" -o mosaic_A1.png \
     http://SERVER:8765/c/mebp-out/mosaic_A1.png
```

The response carries `X-Lablink-Sha256`. **Verify it** — that is the entire
point of it being there. To resume a partial download, send
`Range: bytes=<bytes you already have>-` and append the `206` response to your
file. See §5 for the rules.

### PUT /c/{channel}/{name} — upload

```bash
SHA=$(shasum -a 256 result.csv | cut -d' ' -f1)      # macOS/Linux
curl -X PUT --data-binary @result.csv \
     -H "X-Lablink-Token: TOKEN" \
     -H "X-Lablink-Sha256: $SHA" \
     -H "Content-Type: application/octet-stream" \
     http://SERVER:8765/c/results/result.csv
```

Optionally add `-H 'X-Lablink-Meta: {"source":"matlab","job":"A1"}'` — any JSON
object up to 8 KB, ASCII only. It comes back verbatim in listings, so it is the
natural place for "which input produced this".

Response is `201` (stored) or `200` (identical file was already there), with the
record as JSON.

### DELETE /c/{channel}/{name}

```bash
curl -X DELETE -H "X-Lablink-Token: TOKEN" \
     http://SERVER:8765/c/results/old.csv
```

---

## 2. Rules you must follow

These are not style preferences; ignoring them produces errors or data loss.

**`Content-Length` is required on upload, and chunked encoding is not
supported.** Many HTTP libraries switch to `Transfer-Encoding: chunked` when
you hand them a stream. If yours does, you get `400`. Set an explicit
`Content-Length` (you know the file size) or send the bytes as one buffer.

**`X-Lablink-Sha256` is required and is verified.** The server hashes what
arrives and rejects a mismatch with `400`, storing nothing. This catches
truncation and corruption. Lowercase hex.

**Upload the same name twice and the second one is refused,** unless the bytes
are byte-for-byte identical (then it is a no-op `200`). The server never
silently overwrites and never invents `file(2).png`. So:

> **Give every file a unique name** — put a timestamp in it, e.g.
> `mosaic_20260803_141530.png`. This single habit avoids the whole problem.

To replace a file deliberately: `DELETE`, then `PUT`.

**Names must match `^[A-Za-z0-9][A-Za-z0-9._ -]{0,127}$`** — one path segment,
starting with a letter or digit. Spaces are allowed (percent-encode them in the
URL: `mosaic%20001.png`). Rejected: `/` `\` `..`, leading dot, trailing dot or
space, Windows device names (`CON`, `NUL`, `COM1`…), anything non-ASCII.

**Two names differing only in capitalisation cannot coexist** in one channel.
`Scan.png` and `scan.png` are the same file to Windows and macOS, so a client
downloading the channel into one folder could not keep both. The server refuses
the second one.

**A file is either fully there or not there at all.** There is no window in
which a listing shows a file whose bytes are still arriving, so you never need
to guess whether an upload finished.

---

## 3. Status codes

| Code | Meaning | What to do |
|---|---|---|
| `200` | OK; on `PUT`, an identical file already existed | Treat as success — this is what makes retrying safe |
| `201` | Stored | — |
| `206` | Partial content (your `Range` was honoured) | Append to your partial file |
| `400` | Bad checksum, bad name, missing `Content-Length`/sha, malformed meta | Fix the request; do not retry unchanged |
| `401` | Token wrong or missing | Fix the token; do not retry |
| `404` | No such file or endpoint | — |
| `409` | Name taken by different content, or a case-variant exists | Rename (use a timestamp) or `DELETE` first |
| `413` | Larger than the server's limit | Check `max_file_bytes` from `/hello` |
| `416` | `Range` start is past the end of the file | Your partial file is too long; discard and restart |
| `507` | Server disk nearly full | Alert a human |
| `5xx` | Server problem | Retry with backoff |

Errors are JSON: `{"error": "human-readable sentence"}`. The sentences are
written to be shown to an operator — surface them rather than replacing them
with your own text.

**Retry `5xx` and network errors; never retry `400`, `401`, `409`, `413`.**

---

## 4. Uploading robustly

1. Compute the SHA-256 of the file.
2. Optionally `GET /hello` and compare `size` against `max_file_bytes`.
3. `PUT` with `Content-Length` and `X-Lablink-Sha256`.
4. On success, confirm the returned `sha256` equals yours.
5. On a network error or `5xx`, **retry the whole upload** with backoff
   (1 s, 2 s, 4 s…). This is safe: if the first attempt actually landed, the
   retry returns `200` "already there" instead of duplicating.

Resumable *upload* is deliberately not supported — retrying a whole file is
simpler and, at these sizes, cheaper than the protocol it would require.

---

## 5. Downloading robustly (the part people get wrong)

Two failure modes must be handled **differently**, and conflating them is the
bug we hit ourselves:

- **Incomplete** (connection dropped mid-transfer): your file is *shorter* than
  expected. **Keep it and resume.** Deleting it throws away the progress.
- **Corrupt** (complete but wrong hash): discard and start over.

So the checksum alone cannot be your test — check the **length first**:

```
loop:
    have = size of partial file (0 if none)
    GET the file with "Range: bytes=<have>-" when have > 0

    determine total size:
        from Content-Range "bytes X-Y/TOTAL" on a 206
        else from Content-Length on a 200 (which restarts from zero)

    append (206) or overwrite (200) the body into the partial file

    if partial size < total:            # truncated
        if it grew this pass:  continue         # progress: keep going
        else:                  give up after a few fruitless passes
    if sha256(partial) != expected:     # complete but wrong
        delete partial; retry from zero
    move partial into place             # atomically — see below
    done
```

Three details that matter:

- **`416` means your partial is at or past the full length** — it is complete,
  or it is corrupt and too long. Verify the hash to find out which.
- **A `200` in reply to a `Range` request** means the server ignored the range.
  Truncate and start from zero; don't append.
- **Write into a temporary name, then rename into place.** If anything watches
  your output folder, it must never see a half-written file. `rename` within one
  filesystem is atomic on every OS. Keep the temp file in a *subdirectory* or
  give it an extension your watcher ignores.

---

## 6. Polling for new work

```
seq = 0                     # or load from your state file
loop forever:
    GET /c/mebp-out?since_seq=<seq>
    for each file in the response (already in seq order):
        download, verify, process
        seq = file.seq      # advance ONLY after success
        save seq
    sleep 5 seconds
```

Two rules learned the hard way:

- **Advance your cursor only over an unbroken run of successes.** If file 4
  fails and file 5 succeeds, do *not* set the cursor to 5 — file 4 would never
  be retried and would be lost silently. Keep processing later files if you
  like, but leave the cursor below the failure.
- **Re-list from `since_seq=0` occasionally** (on startup, and every few
  minutes). Skip anything you already have — compare size, and hash only if you
  need certainty. This self-heals a cursor that has somehow moved past a file.

There is no push notification and no long-poll. Poll every few seconds; a
listing is a small JSON document.

---

## 7. The worker pattern: receive → process → send back

This is the shape you want for MATLAB/ImageJ/Python analysis machines.

```
INPUT_CHANNEL  = "mebp-out"     # work arrives here
OUTPUT_CHANNEL = "results"      # results go back here

forever:
    for each new file in INPUT_CHANNEL (via since_seq):
        download and verify it
        result = your_analysis(file)               # the only part that is yours
        upload result to OUTPUT_CHANNEL, named after the input:
            e.g. "mosaic_20260803_141530_result.csv"
            with meta {"input": "<input name>", "input_sha256": "<hash>"}
        advance the cursor
    sleep
```

**Name the output after the input, and record the input's name and hash in
`meta`.** That is what makes a result traceable to what produced it — six
months later, "which scan produced this number" has an answer.

Two channels, one per direction, is not optional: a worker that read and wrote
the same channel would find its own output and process it forever.

### Minimal Python worker (stdlib only, ~40 lines)

Verified against a live server: it fetched a 9 MB mosaic, checked it, and
returned a result carrying the input's name and hash.

```python
import hashlib, json, os, time, urllib.parse, urllib.request

BASE, TOKEN = "http://100.74.228.41:8765", "YOUR-TOKEN"
IN_CH, OUT_CH = "mebp-out", "results"

def req(method, path, data=None, headers=None):
    r = urllib.request.Request(f"{BASE}{path}", data=data, method=method)
    r.add_header("X-Lablink-Token", TOKEN)
    for k, v in (headers or {}).items():
        r.add_header(k, v)
    return urllib.request.urlopen(r, timeout=60)

def process(name, blob):                      # <-- your analysis here
    return f"file,{name}\nbytes,{len(blob)}\n".encode()

seq = 0
while True:
    try:
        with req("GET", f"/c/{IN_CH}?since_seq={seq}") as r:
            listing = json.load(r)
        for rec in listing["files"]:
            with req("GET", f"/c/{IN_CH}/{urllib.parse.quote(rec['name'], safe='')}") as r:
                blob = r.read()
            if hashlib.sha256(blob).hexdigest() != rec["sha256"]:
                print("checksum mismatch, will retry:", rec["name"])
                break                          # do NOT advance the cursor
            out = process(rec["name"], blob)
            out_name = f"{os.path.splitext(rec['name'])[0]}_result.csv"
            req("PUT", f"/c/{OUT_CH}/{urllib.parse.quote(out_name, safe='')}",
                data=out,
                headers={"X-Lablink-Sha256": hashlib.sha256(out).hexdigest(),
                         "Content-Length": str(len(out)),
                         "X-Lablink-Meta": json.dumps(
                             {"input": rec["name"], "input_sha256": rec["sha256"]})}
                ).close()
            print("handled", rec["name"], "->", out_name)
            seq = rec["seq"]
    except Exception as exc:                   # a lab tool must not die on a hiccup
        print("cycle failed, retrying:", exc)
    time.sleep(5)
```

This reads the whole file into memory, which is fine up to tens of MB. For
larger files, stream to disk and use the resume logic in §5.

### MATLAB

```matlab
opts = weboptions('HeaderFields', {'X-Lablink-Token','YOUR-TOKEN'}, 'Timeout', 60);
listing = webread('http://SERVER:8765/c/mebp-out', opts);

for k = 1:numel(listing.files)
    rec = listing.files(k);
    websave(rec.name, ['http://SERVER:8765/c/mebp-out/' rec.name], opts);
    % ... your analysis, writing out.csv ...
    up = weboptions('HeaderFields', { ...
        'X-Lablink-Token','YOUR-TOKEN'; ...
        'X-Lablink-Sha256', lower(string(mlreportgen.utils.hash('out.csv'))); ...
        'Content-Type','application/octet-stream'}, ...
        'RequestMethod','put', 'Timeout',60);
    webwrite('http://SERVER:8765/c/results/out.csv', fileread('out.csv'), up);
end
```

Compute the hash however your MATLAB version allows (`Simulink.getFileChecksum`,
a `System.Security.Cryptography` call, or shelling out to `shasum -a 256`) — the
header is mandatory.

### Constrained devices (ESP32, Arduino, instrument consoles)

The protocol needs only: an HTTP client, `Content-Length`, and SHA-256. If the
device cannot hash, it cannot upload — hash on a companion machine instead, or
have the device write to a shared folder that a real computer relays.

Sending small files is easy: hold the payload in RAM, hash it, `PUT` it.
Downloading large files needs the `Range` logic in §5, which needs somewhere to
put a partial file.

---

## 8. Security — what this protocol does and does not protect

Read this before putting anything sensitive through it.

**The token is a misdirection guard, not authentication.** It stops you sending
files into the wrong machine on a shared network. It does not stop an attacker:
it is a single shared secret, and every holder can read, overwrite and delete
everything in every channel. There are no users, no permissions, no audit trail.

**There is no TLS.** Traffic — including the token — is plaintext on the wire.
Anyone who can observe the network can read your files and steal the token.

> **This is why the deployment runs over Tailscale.** WireGuard encrypts and
> authenticates the transport, which is what actually protects the traffic; the
> token then only distinguishes intended machines within the tailnet. **If you
> run this on a plain LAN, treat the contents as public to that network.**

**Do not put through it:** anything identifiable, anything confidential,
credentials, or anything you would not put on a USB stick left on a bench.

**`GET /hello` is unauthenticated** and returns the server's hostname, version
and clock. Lab hostnames are often owner-identifying, so anyone scanning the
segment gets free inventory. Pass `--label` to give the server a neutral name.

**Every token holder can read, overwrite and delete everything.** There are no
users, no per-channel permissions, and no audit trail — the access log records
requests, not identities. Treat the token as "who may use the lab exchange",
never as "who may see this file".

**Anyone with the token can fill the disk.** There is a per-file size cap and a
free-space guard, but no quota. `--ttl-hours` is the mitigation.

**The server accepts connections before checking the token,** so an
unauthenticated party who can reach the port can open connections and consume
threads. Do not expose the port to the internet — bind it to the Tailscale
interface (`--bind 100.x.y.z`) so it is not reachable from the local Wi-Fi at
all.

### Three rules for your own client (each one was a real bug here)

**1. Validate every server-supplied name before it touches the filesystem.**
The `name` in a listing arrives over the network. Joining it to your download
folder is not safe: on Windows, `Path("C:/inbox") / "//attacker/share/f"`
yields `\\attacker\share\f`, and merely calling `.exists()` on that makes
Windows offer your NTLM credentials to whoever answers. `C:/Windows/x` discards
your folder entirely, and `../../x` escapes it. Check the name against §2's
pattern **before** any `exists`, `stat`, `open` or `rename` — not just before
the write.

**2. Verify against the hash you asked for, not the one the response offers.**
`X-Lablink-Sha256` only proves the response is self-consistent. If you pinned a
hash from a listing, enforce *that*; if both are present and differ, treat it as
tampering and discard. Preferring the header lets a hostile server serve any
bytes and hash them to match.

**3. Do not let "incomplete" look like "verified".** If you cannot verify a
download — no hash available, or a `416` left you with a partial you cannot
check — discard it and refetch. Never rename an unverified file into place; a
consumer watching that folder cannot tell the difference.

Also: only advance your polling cursor over consecutive successes (§6), and
never trust a listing's field types — a malformed entry should be skipped, not
crash your loop.

---

## 9. Checklist for a new client

- [ ] `GET /hello` works and you compare the clock
- [ ] Token sent on every other request
- [ ] Percent-encode names in URLs (spaces!)
- [ ] `Content-Length` set explicitly on upload; not chunked
- [ ] `X-Lablink-Sha256` computed and sent on upload
- [ ] Downloaded bytes verified against `X-Lablink-Sha256`
- [ ] Unique output names (timestamps)
- [ ] `200` on upload treated as success, not an error
- [ ] `400`/`401`/`409`/`413` not retried; `5xx` and network errors retried with backoff
- [ ] Downloads assembled under a temp name and renamed into place
- [ ] Cursor advanced only over consecutive successes
- [ ] Periodic full re-list to self-heal
- [ ] Separate channels for input and output
- [ ] One cycle failing does not kill the process
