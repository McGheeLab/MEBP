# LabLink — a shared drop-box for lab machines

Move files between lab computers over the network without a shared drive, a
USB stick, or any software installation. One machine runs a small server;
the others send files to it and pick files up from it.

Typical use: MEBP saves a mosaic on the microscope PC, it appears on the
analysis PC, MATLAB processes it, and the results appear back on the
microscope PC — with no code on either side of the workflow. Each machine
just watches a folder.

**Requires nothing but Python 3.11+.** No `pip install`, on any machine. It
does not import or modify MEBP; it is a self-contained mini-project.

---

## 1. What this is, and what it is not

It **is** a robust file exchange:

- files are opaque bytes — images, CSVs, `.mat`, anything, with optional JSON metadata;
- every transfer is checksum-verified end to end;
- interrupted downloads resume where they left off;
- machines can start and stop whenever they like, in any order, with nothing lost or duplicated.

It is **not**:

- **secure.** The shared token stops you accidentally sending files into the
  wrong machine on a shared network. Traffic is plain HTTP: anyone on the same
  network segment who learns the token can read everything. **Do not put
  sensitive or identifiable data through it.**
- **a remote-processing service.** It moves files; what each machine does with
  them is your own workflow.
- **a sync tool with deletion.** Files only ever get added to a machine's inbox.
  Deleting a file on the server does not delete anyone's copy (see §9).

---

## 2. Prerequisites

Python 3.8 or newer on every machine. Nothing else — no `pip install`.

```powershell
python --version       # Windows
```
```bash
python3 --version      # macOS / Linux — note the "3"
```

**The command differs by platform.** On Windows it is `python`; on macOS and
Linux it is `python3` (a bare `python` usually does not exist and gives
`command not found`). Every example below shows the Windows form — substitute
`python3` on a Mac. Paths differ too: Windows `WIFI parsing\lablink_cli.py`,
macOS/Linux `"WIFI parsing/lablink_cli.py"` (the quotes matter, the folder
name has a space).

Copy this whole `WIFI parsing` folder to each machine, or pull the repo on
both. `tools_link_check.py` is deliberately standalone — a single file you can
copy by USB stick to test the network before anything else is set up.

---

## 3. Choosing the network link (do this first — about 10 minutes)

Campus Wi-Fi often blocks machine-to-machine traffic ("client isolation"), so
**test reachability before setting anything else up.** Work down this list and
stop at the first option that passes.

> Don't use `ping` for this. ICMP and TCP are filtered independently — ping can
> succeed where the port is blocked and vice versa. The check below tests the
> actual port the file exchange uses.

### Option A — plain campus Wi-Fi (try first; it costs 5 minutes)

On **machine A**:

```powershell
python tools_link_check.py --listen
```

It prints the exact command to run on the other machine. On **machine B**:

```powershell
python tools_link_check.py --check http://<machine-A-ip>:8765
```

`LINK CHECK: PASS` → you're done, use machine A's campus IP as the server
address. Be aware campus IPs change: if the link stops working after a reboot,
re-check machine A's address.

Measurements on UAWiFi suggest this will **time out** — the access point drops
device-to-device traffic. If so, continue to B.

### Option B — Windows Mobile Hotspot (recommended for two machines in one room)

On the machine that will run the server: **Settings → Network & Internet →
Mobile hotspot → On**. On the other machine, join that hotspot.

The server machine is then reachable at **`192.168.137.1`**. Re-run the check
from machine B with `--check http://192.168.137.1:8765`.

This is a private network — no campus involvement, no client isolation, and
the fastest option. Two notes:

- The hotspot shares the one Wi-Fi radio with the campus connection, so both
  slow down somewhat. For occasional file transfers this is fine; a ~$20 USB
  Wi-Fi adapter dedicated to the hotspot removes the contention entirely.
- The client machine leaves campus Wi-Fi to join the hotspot, so it loses
  internet unless the host shares its connection (Mobile hotspot settings).

### Option C — Tailscale (different rooms, more than two machines, or A and B failed)

> **This is what this lab ended up using.** Measured 2026-08-01, Windows
> server + MacBook client, both on UAWiFi: options A and B both failed (see
> the verified results at the end of this section), and Tailscale worked
> first try at **117 ms RTT, 1.5 MB/s up and 1.9 MB/s down** — a relayed
> connection, not direct, because the access point blocks the peer traffic
> Tailscale needs to punch through. That is fast enough for this workload:
> a 12 MB mosaic moves in about 8 seconds, a 40 MB one in under half a minute.

Install [Tailscale](https://tailscale.com/download) on both machines and sign
in with the same account (the free tier is sufficient). Each machine gets a
permanent `100.x.y.z` address that survives IP changes and works across
buildings. Use that address as the server URL.

Check whether traffic is direct or relayed:

```powershell
tailscale status
```

A peer marked `relay` routes through Tailscale's servers on the internet —
still functional, but much slower (a 40 MB mosaic may take a minute).
`direct` runs at full LAN speed.

### Whichever you chose

Record the server URL. Nothing in the code depends on which option you picked —
the address is just a configuration string, so you can change your mind later
without touching anything else.

### Verified results on this lab's hardware (2026-08-01)

Windows server (`McGheeLab-CellC`, Intel AX211) and a MacBook Pro client,
both associated to UAWiFi:

| Option | Result |
|---|---|
| **A. Plain UAWiFi** | **Failed — timed out.** The server was confirmed listening on `0.0.0.0:8765`, answering on its own LAN IP, with an all-profiles firewall rule allowing the port. The client still got no reply at all. A *timeout* rather than *refused* is the tell: the packets never arrive, so the access point is dropping device-to-device traffic. |
| **B. Windows Mobile Hotspot** | **Not possible on this hardware.** `netsh wlan show drivers` reports `Hosted network supported: No`, there are no Wi-Fi Direct adapters, and UAWiFi is WPA2-Enterprise, which commonly blocks connection sharing. |
| **C. Tailscale** | **Worked.** 117 ms RTT, 1.5 MB/s up, 1.9 MB/s down, checksum verified. Relayed rather than direct — expected, since the isolation that broke option A also prevents the direct hole-punch. |

If your access point behaves differently, options A and B are still worth the
five minutes: they are several times faster when they work.

---

## 4. Set up the server machine (once)

Pick any machine that is usually on. It does not need to be powerful.

**1. Allow the port through the firewall.** In an **Administrator** PowerShell:

```powershell
netsh advfirewall firewall add rule name="LabLink 8765" dir=in action=allow protocol=TCP localport=8765
```

(To undo later: `netsh advfirewall firewall delete rule name="LabLink 8765"`.)

**2. Choose a token.** Any hard-to-guess string; every machine uses the same one.

**3. Start the server:**

```powershell
cd "C:\path\to\WIFI parsing"
python lablink_server.py --root D:\lablink_data --token YOUR-TOKEN
```

Useful options:

| Option | Meaning |
|---|---|
| `--root DIR` | where files are stored (default: `lablink_data` beside the script) |
| `--port N` | default 8765 |
| `--ttl-hours N` | auto-delete files older than N hours (default: keep forever) |
| `--max-file-mb N` | reject larger uploads (default 512) |
| `--bind IP` | listen on one interface only, e.g. `192.168.137.1` |

The startup banner prints the exact `--url` clients should use, one line per
network address, labelling hotspot and Tailscale addresses. Leave the window
open (see §8 for running it in the background).

---

## 5. Check it works from another machine

```powershell
cd "C:\path\to\WIFI parsing"
$env:LABLINK_URL  = "http://192.168.137.1:8765"     # your server URL
$env:LABLINK_TOKEN = "YOUR-TOKEN"

python lablink_cli.py hello
```

You should see the server name, round-trip time and clock difference. Then
measure real throughput:

```powershell
python tools_link_check.py --check $env:LABLINK_URL --token $env:LABLINK_TOKEN --mb 8
```

---

## 6. Sending and fetching files by hand

Files live in **channels** — named mailboxes that keep different projects and
directions apart. A channel is created the first time something is put in it.

```powershell
python lablink_cli.py send mebp-out "C:\mosaics\mosaic 001.png"
python lablink_cli.py send mebp-out C:\mosaics\*.png --meta '{\"plate\":\"nest-24\"}'
python lablink_cli.py list mebp-out
python lablink_cli.py get  results analysis.csv --dest C:\incoming
python lablink_cli.py get  results --all        --dest C:\incoming
python lablink_cli.py delete mebp-out "mosaic 001.png" --yes
python lablink_cli.py watch results --dest C:\incoming     # poll and fetch new files
```

Every command accepts `--url` and `--token` if you would rather not use
environment variables, or `--config file.json` containing `{"url":…, "token":…}`.

Exit codes are 0 (ok), 1 (error), 2 (bad usage), so PowerShell scripts can
branch on `$LASTEXITCODE`.

---

## 7. Automatic folder sync (the normal way to use this)

The sync agent watches a local **outbox** folder and uploads anything that
appears in it, and downloads anything new from a channel into a local
**inbox** folder. Your workflow only ever touches ordinary folders.

Create a config:

```powershell
python lablink_sync.py --make-config C:\lablink\sync.json
```

Edit it:

```json
{
  "url": "http://192.168.137.1:8765",
  "token": "YOUR-TOKEN",
  "outbox": { "dir": "C:/lablink/outbox", "channel": "mebp-out" },
  "inbox":  { "dir": "C:/lablink/inbox",  "channel": "results" },
  "poll_seconds": 5,
  "state_file": "C:/lablink/sync_state.json"
}
```

Run it:

```powershell
python lablink_sync.py --config C:\lablink\sync.json
```

Either `outbox` or `inbox` may be `null` for a one-way agent. Stop it with
Ctrl+C any time; start it again whenever — nothing is lost or re-sent.

### Worked example: MEBP mosaic → MATLAB → back

```
  MICROSCOPE PC                    SERVER                    ANALYSIS PC
  ─────────────                    ──────                    ───────────
  MEBP saves a mosaic
    into  outbox\      ──upload──▶  [mebp-out]  ──download──▶  inbox\
                                                                  │
                                                        MATLAB reads inbox\,
                                                        writes results into
                                                                outbox\
    inbox\             ◀─download─  [results]   ◀──upload───  outbox\
       │
  MEBP / you open
  the results
```

Microscope PC config: `outbox → mebp-out`, `inbox → results`.
Analysis PC config: `outbox → results`, `inbox → mebp-out`.

Two rules that make this reliable:

1. **One channel per direction.** Never use the same channel for a machine's
   outbox and inbox — it would download its own uploads forever. The agent
   refuses to start if you do.
2. **Use unique file names**, e.g. include a timestamp (`mosaic_20260801_1423.png`).
   Re-sending a *different* file under a name already on the server is refused
   (see §9); unique names avoid the question entirely.

### What the agent guarantees

- **Never uploads a half-written file.** A file is sent only once its size and
  timestamp have stopped changing, so a mosaic still being written by MEBP or
  MATLAB is left alone until it is complete.
- **Never shows a half-written file.** Downloads are assembled in a hidden
  `.partial` subfolder and moved into place in one step, so a script watching
  the inbox can be completely naive.
- **Survives outages.** Server down, machine rebooted, cable pulled — cycles
  are retried, interrupted downloads resume, and the state file is only an
  optimisation: delete it and nothing is re-transferred or duplicated.
- **Self-heals.** On start-up, and every few minutes, the inbox is reconciled
  against the full channel listing, so anything that was somehow missed is
  picked up rather than lost.

---

## 8. Keeping the server and agents running

Minimised windows (simplest):

```powershell
Start-Process python -ArgumentList 'lablink_server.py','--token','YOUR-TOKEN' -WindowStyle Minimized
Start-Process python -ArgumentList 'lablink_sync.py','--config','C:\lablink\sync.json' -WindowStyle Minimized
```

To start automatically: **Task Scheduler → Create Task → Trigger: At log on →
Action: Start a program** — program `python`, arguments as above, "Start in"
set to this folder.

---

## 9. Rules worth knowing

**Same name, same content** → accepted as a no-op. This is what makes retries
after a crash safe.

**Same name, different content** → refused (HTTP 409). The server never
silently overwrites, and never invents `file(2).png` names that would confuse a
watcher. To replace a file deliberately:

```powershell
python lablink_cli.py delete mebp-out "mosaic 001.png" --yes
python lablink_cli.py send   mebp-out "C:\new\mosaic 001.png"
```

The sync agent reports a conflict clearly and keeps going with other files.

**Deleting is local to the server.** It stops the file being listed or
downloaded again; copies already delivered to an inbox stay put. Clean up with
`--ttl-hours` on the server, or the `delete` command.

**File names** may contain letters, digits, dots, underscores, spaces and
hyphens, must start with a letter or digit, and must be at most 128 characters.
Windows device names (`CON`, `NUL`, `COM1`…) are refused. The sync agent skips
files it cannot name and logs why once.

---

## 10. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `connection refused` | Nothing is listening. Is the server running? Right port? |
| `timed out` / no route | Campus Wi-Fi client isolation, or the firewall rule is missing on the **server** machine. Work through §3 again. |
| `[401] bad or missing token` | The tokens differ. Both sides need the same `--token` / `LABLINK_TOKEN`. |
| `[409]` on send | A different file already has that name. See §9. |
| `[413]` on send | Bigger than the server's limit. Restart the server with `--max-file-mb`. |
| Clock warning in `hello` | The two clocks differ by more than 2 s. Transfers are unaffected (ordering uses sequence numbers, not clocks), but file timestamps will look odd across machines. |
| Slow transfers (<2 MB/s) | On Tailscale, run `tailscale status` — a `relay` peer routes via the internet. Prefer a direct connection or the Mobile Hotspot. |
| A file never arrives | Check the sending agent's log for `CONFLICT` or `skipping`. The inbox reconcile pass also recovers anything missed within a few minutes. |
| Sync agent won't start | `outbox and inbox must use DIFFERENT channels` — see §7 rule 1. |
| Want a clean slate | Stop the agent, delete its `state_file`, restart. Safe: nothing is re-sent or re-downloaded. |
| `--meta is not valid JSON` in PowerShell | PowerShell strips the inner quotes before Python sees them. Escape them: `--meta '{\"plate\":\"nest-24\"}'`. In bash/zsh the plain form `'{"plate":"nest-24"}'` is correct. |
| `not a file` when sending | The file name is taken relative to your current folder. Use a full path, or `cd` to where the file is first. |

---

## 11. Tests

```powershell
cd "C:\path\to\WIFI parsing"
python -m unittest discover -s tests
```

Covers crash-safety (uploads interrupted at any point leave nothing visible),
checksum verification, download resume, collision rules, sequence-number
recovery, concurrent uploads, and the sync agent end to end against a real
server on localhost.

### Acceptance checklist for a new installation

1. `tools_link_check.py --check … --token …` reports **PASS**; note the MB/s.
2. Send a large (tens of MB) file; kill the **server** at about half way;
   restart it. The partial file must **not** appear in `list`. Send it again — it
   uploads cleanly.
3. Download that file; kill the **client** at about half way. The destination
   folder must be empty (the part sits in `.partial`). Run again — it resumes
   and the checksum verifies.
4. Run sync agents on both machines; drop ~20 mixed files into an outbox and
   restart the server while they transfer. All must arrive exactly once and be
   byte-identical.
5. Delete both `state_file`s and restart the agents. Nothing is re-downloaded
   and the server reports only duplicates.
