# AP_Syslink

Driver for the nRF51822 radio co-processor on Crazyflie 2.x.

## The system

A Crazyflie 2.x carries two MCUs on one board:

```
  GCS ── USB ── Crazyradio 2.0 ──── 2.4 GHz ESB ──── nRF51822 ── UART ── STM32
                  (nRF52840)                        (radio)     1Mbaud   (ArduPilot)
                                                        │
                                          also: power, button, charging
```

ArduPilot runs on the STM32 and has no radio of its own. The nRF51 owns the
radio and is reached over USART6 running a framed protocol called **syslink**.
To ArduPilot the nRF51 is a dumb, lossy, packet-oriented serial link that also
happens to report battery and button state.

This library owns that UART outright and demultiplexes syslink packet types out
to the rest of the vehicle. Nothing else may open USART6.

## Framing

1 Mbaud, 8N1, no hardware flow control on the STM32 side.

```
+-----------+------+-----+=============+-----+-----+
|   START   | TYPE | LEN | DATA        |   CKSUM   |
+-----------+------+-----+=============+-----+-----+
```

- `START` — two constant bytes, `0xBC 0xCF`
- `TYPE` — one byte, packet type
- `LEN` — one byte, length of `DATA`
- `CKSUM` — two-byte Fletcher-8 over `TYPE`, `LEN` and `DATA` (RFC 1146)

## Architecture

```
                    ┌──────────────────────────────────┐
                    │            AP_Syslink            │
   USART6  ────────►│  framing / Fletcher-8 / demux    │
   (SERIAL2,        │  own thread @ PRIORITY_UART      │
    protocol 51)    └───┬───────────┬──────────┬───────┘
                        │           │          │
                   0x0C/0x0D      0x13       0xF0 …
                        │           │          │
                        ▼           ▼          ▼
              ┌──────────────┐  ┌────────┐  ┌────────┐
              │ MAVLinkPort  │  │battery │  │ debug  │
              │RegisteredPort│  │        │  │ probe  │
              └──────┬───────┘  └────────┘  └────────┘
                     │
                GCS_MAVLINK (COMM_2)
```

### Why a RegisteredPort

`AP_SerialManager::RegisteredPort` is an `AP_HAL::UARTDriver` subclass whose
purpose is to present a non-UART transport to ArduPilot as a serial port.
`AP_Networking::Port` is the reference implementation and solves nearly the same
problem — MAVLink over a lossy, packetised, MTU-limited link. Two things come
for free from that path:

- `mavlink_packetise()` (`AP_HAL/utility/packetise.cpp`) returns exactly one
  MAVLink frame's worth of bytes from a `ByteBuffer`, handling v1/v2 and signed
  frames. That is the "one whole frame per chunk where it fits" requirement,
  already written.
- Overriding `txspace()` and `bw_in_bytes_per_second()` makes the GCS throttle
  itself: `txspace()` feeds `comm_send_lock()`/`HAVE_PAYLOAD_SPACE`, and the
  bandwidth hint paces parameter download (`GCS_Param.cpp`) and FTP bursts
  (`GCS_FTP.cpp`) — the two flows most likely to bury a 5-deep radio queue.

### Why SERIAL2_PROTOCOL must be Syslink, and must not be None

The obvious way to keep the GCS off the physical port is to leave it
unassigned. **That does not work on ChibiOS**, and the failure is silent.

`SerialProtocol_None` is `-1`, and `AP_SerialManager::init()` — which runs
before `init_ardupilot()`, so before this driver starts — calls
`uart->disable_rxtx()` on any `None` port. That sets both rx and tx lines to
`PAL_MODE_INPUT`, de-muxing them from the USART peripheral. **Nothing ever
restores the alternate-function muxing.** The only code that re-muxes is the
pin-inversion path in `AP_HAL_ChibiOS/UARTDriver.cpp`, which is compiled only
for F7/H7/F3/G4/L4 — and the Crazyflie is an STM32F405. A later `begin()`
starts the peripheral onto disconnected pins, and the link is dead for the rest
of the boot with no error anywhere.

So the port carries its own protocol value, `SerialProtocol_Syslink = 51`.
Anything that is not `None` and has no explicit case in the serial manager's
switch falls through to `default:`, which calls a harmless `begin()` and leaves
the pins alone. No other consumer claims the value, so nothing fights us for
the port.

This costs two small upstream deltas, both unavoidable:

- the enum value itself, and
- one matching entry in `SERIAL_PROTOCOL_VALUES` in `AP_OSD_ParamSetting.cpp`,
  which carries an unguarded
  `static_assert(SerialProtocol_NumProtocols == ARRAY_SIZE(SERIAL_PROTOCOL_VALUES))`.
  That file compiles on crazyflie2 even though `OSD_ENABLED` is 0, so the table
  must grow in lockstep with the enum or the build fails.

Value **50 is deliberately left free** because upstream 4.7 uses it for
`SerialProtocol_IOMCU`; taking 51 keeps that rebase clean.

The *virtual* port advertises the existing `SerialProtocol_MAVLink2`, so no
further enum values are needed.

With SERIAL2 carrying Syslink rather than MAVLink, the virtual port is expected
to land on `MAVLINK_COMM_2`
— the same channel the old in-GCS implementation used — so `SR2_*` stream rate
parameters carry over unchanged. Confirm this at bring-up rather than assuming.

### The driver thread must own the port

`begin()` must be called **from the driver thread**, not from `init()` on the
main thread. The ChibiOS `UARTDriver` records the calling thread in
`_uart_owner_thd` and then refuses reads from anyone else:

```c
uint32_t UARTDriver::_available()
{
    if (!_rx_initialised || _uart_owner_thd != chThdGetSelfX()) {
        return 0;
```

`_read()` likewise returns -1. Neither reports an error. **Writes are not
guarded at all**, so opening the port on the wrong thread yields a link that
transmits perfectly and receives absolutely nothing — which looks exactly like
dead hardware or an unresponsive peer.

This is why `init_port()` runs at the top of `thread_main()`. `AP_Torqeedo`
does the same thing, and says so in a comment on `init_internals()`.

A `SCHED_TASK` in the vehicle's scheduler table would sidestep this, since the
main loop would be both the opener and the reader. A dedicated thread is still
the better fit at 1 Mbaud: the main loop would have to absorb up to ~250 bytes
per iteration at 400 Hz against a 512-byte driver buffer, and this link stalls
the nRF51's main loop for ~2.6 ms per full-size chunk. The thread just has to
own the port.

## Packet types used

| Type | Name | Direction | Phase |
|------|------|-----------|-------|
| 0x01 | `RADIO_CHANNEL` | →nRF, echoed | 2 |
| 0x02 | `RADIO_DATARATE` | →nRF, echoed | 2 |
| 0x05 | `RADIO_ADDRESS` | →nRF, echoed | 2 |
| 0x07 | `RADIO_POWER` | →nRF, echoed | 2 |
| 0x0B | `RADIO_READY` | →nRF, echoed | 2 |
| 0x0C | `RADIO_MAVLINK` | both | 3 |
| 0x0D | `RADIO_MAVLINK_BROADCAST` | both | 6 |
| 0x0E | `RADIO_MAVLINK_SPACE` | nRF→ | 3 |
| 0x13 | `PM_BATTERY_STATE` | nRF→ | 4 |
| 0x14 | `PM_BATTERY_AUTOUPDATE` | →nRF | 4 |
| 0x15/0x16 | `PM_SHUTDOWN_REQUEST`/`_ACK` | both | later |
| 0xF0 | `DEBUG_PROBE` | request/response | 1 |

## Constraints that bite

**Chunks cap at 251 bytes, not 252.** One byte of the 252-byte ESB payload is an
on-air marker separating MAVLink traffic from the nRF51's own CRTP-derived
control packets. A longer chunk is **dropped, not truncated** — silently losing
the tail would corrupt a frame undetectably.

**A MAVLink v2 frame can exceed one chunk.** Up to 267 bytes unsigned, 280
signed. `FILE_TRANSFER_PROTOCOL` lands near 261, and FTP is how a GCS fetches
parameters and logs, so this is not an edge case. Oversized frames are split
across chunks; losing either half costs the frame. No fragment header is needed
or wanted — the far end feeds a byte-stream parser that resyncs on STX.

**Two independent backpressure mechanisms guard different things.**

- `SYSLINK_RADIO_MAVLINK_SPACE` (0x0E) reports free *radio* transmit slots,
  peaking at 5, sent unsolicited whenever the count changes. In telemetry mode
  the Crazyflie is a PRX, so chunks only leave in ack payloads when the ground
  station polls; if it stops polling the queue fills. Track this locally
  (decrement on send, refresh on report) and derive `txspace()` from it.
- The `NRF_FLOW_CTRL` line (PA4, `GPIO(62)`) is the nRF51's UART RTS. It
  reflects the nRF51's **UART receive FIFO**, not the radio queue — the nRF51
  keeps draining syslink when the radio queue is full and simply discards
  chunks, so this line never asserts for that condition.

Note PA4 is not an STM32 USART6 CTS-capable pin, so this cannot be hardware flow
control; it is polled as a GPIO. It is also RTS-only — there is no CTS in the
other direction. Because a permanently deasserted line would leave the link
silently dark, the gate has a timeout after which the driver transmits anyway
and counts the event.

**Broadcasts are not queued and consume no slot.** 0x0D transmits immediately
and cannot fail for lack of room, so it must bypass the 0x0E slot accounting
entirely. Telemetry and peer traffic interleave freely; the destination is
carried by the packet type, not by any mode state on either side.

**The link starts silent, and this is the single most important thing to get
right.** The nRF51 sends *nothing* over the UART — no battery, no RSSI, no
MAVLink, no echo — until it has received one syslink packet that passes **both**
checksum bytes. Until that gate lifts, a perfectly working nRF51 is
indistinguishable from a dead one, and there is no diagnostic output to look
for: the nRF51's `DEBUG_PRINT` compiles to nothing unless built for SEGGER RTT,
which goes over SWD, not the UART.

Any valid packet lifts it. The driver sends `RADIO_READY` (0x0B) and waits for
the echo, which is the definitive proof that baud rate, framing and checksum
are all correct. It re-arms only on a `SYSOFF` power-down, so in practice it is
a one-time handshake per boot.

Note this is a *different* gate from the 3-second radio deafness in requirement
7. That one gates radio reception only, not the UART, so waiting out the
timeout will not start the flow.

**Battery and RSSI share one enable.** The periodic RSSI report is emitted from
inside the same `enableBatteryAutoupdate` check as the battery packet, so
without `PM_BATTERY_AUTOUPDATE` (0x14) there is no RSSI either — which reads
like a broken link rather than a disabled feature.

**TYPE comes before LEN on the wire.** Swapping them produces a well-formed
frame that will never lift the gate, with no error reported anywhere. The
checksum starts both bytes at zero and covers `TYPE`, `LEN` and `DATA`, not the
start bytes. Two frames to check an implementation against:

```
BC CF 0B 00 0B 16      RADIO_READY, zero length
BC CF 14 00 14 28      PM_BATTERY_AUTOUPDATE, zero length
```

**The radio is gated off for the first 3 seconds** after boot until either
`RADIO_READY` (0x0B) arrives or the timeout expires. Sending it early shortens
startup.

**ArduPilot owns the radio configuration.** The nRF51's compiled-in defaults
(channel 80, address `E7E7E7E7E7`) are not what the radio ends up using — the
STM32 pushes stored channel, datarate and address at boot and those win. A
mismatch between vehicle and ground station looks exactly like a packet-format
failure, so verify both ends agree before debugging anything else. Each config
packet is echoed back by the nRF51; wait on the echo rather than blind-delaying.

**Battery quirks (0x13).**

- `ISET` is *charge* current in mA, not discharge. Mapping it to `current_amps`
  would report plausible nonsense in flight. Left unset (NaN).
- `TEMP` is the nRF51 **die** temperature — not the battery, not ambient. It
  reads above room temperature and climbs under radio load. Deliberately not
  forwarded as a battery temperature. It is also gated on `hasCharger` in the
  nRF51 firmware, so on Roadrunner/Bolt it would sit at a convincing 0 °C.
- The field is only present when the nRF51 is built with
  `PM_SYSLINK_INCLUDE_TEMP`, making the packet 17 bytes instead of 13. The
  parser accepts both lengths so a firmware rebuild does not silently fail.
- Nothing is sent until `PM_BATTERY_AUTOUPDATE` (0x14) is sent first.

**Throughput.** The UART is the bottleneck, not the radio. At 1 Mbaud the
ceiling is ~100 kB/s, but the nRF51's UART has no DMA — per-byte interrupt on a
16 MHz Cortex-M0, and transmission busy-waits. Budget ~50 kB/s sustained duplex.
Ample for telemetry; not a bulk data pipe.

## Throughput

Downlink rate is **polls per second times bytes per radio packet**. One poll
carries exactly one packet whatever its size, so a half-empty packet is a
halved link.

That makes packet occupancy the lever, not the UART, and not the assumed
bandwidth. A `LOG_DATA` frame is about 109 bytes; alone in a 251 byte packet it
wastes well over half of every poll. Packing whole frames until they no longer
fit roughly doubles bulk download at an unchanged poll rate — measured over a
run of `LOG_DATA` frames, 10 chunks become 5. `SYSL_OPTIONS` bit 2 controls it.
The cost is that one lost packet damages two frames instead of one, which
unicast's hardware ack and retry makes rare.

Three separate mechanisms pace traffic, and they apply to different things:

| Mechanism | Affects | Set by |
|---|---|---|
| `bw_in_bytes_per_second()` | parameter download, FTP bursts | `SYSL_BW` |
| `txspace()` from free slots | everything | radio queue depth |
| `have_flow_control()` | param burst clamp, `LOG_DATA` per call | `SYSL_OPTIONS` bit 3 |

**`SYSL_BW` does not affect log download.** `AP_Logger` paces `LOG_DATA` by
`HAVE_PAYLOAD_SPACE()` and a per-call message count, never by the bandwidth
hint, so raising it to speed up logs achieves nothing. It matters for
parameters and FTP.

`get_flow_control()` can report `FLOW_CONTROL_ENABLE`, which lifts two
throttles `GCS_MAVLINK::have_flow_control()` applies to links without it:
parameter streaming is clamped to 5 messages per burst, and
`AP_Logger::handle_log_sending()` drops from 10 `LOG_DATA` per call to 1.

**It is off by default, and enabling it took the link down in testing.** Ten
`LOG_DATA` per call at the rate `update_send()` runs produces far more than
this radio carries; the excess does not queue politely, it saturates the nRF51
and the connection is lost. `SYSL_OPTIONS` bit 3 opts in, and is only worth it
alongside a ground station polling fast enough to drain the result.

### Credit accounting, not absolute credit

`RADIO_MAVLINK_SPACE` describes the queue when the report was generated, which
is stale by the time it crosses the UART — chunks sent in the meantime are
still travelling and are not in it. Treating the value as an absolute credit
hands back slots that are already spoken for and the queue overflows silently.

Credit is therefore returned only on **proof of transmission**: a report
showing more free slots than the previous one. Simulated against a stale-report
pipeline, that removes the overflow entirely for about 3% fewer sends. Because
credit then depends on reports arriving, one slot is released anyway if no
report has been seen for 500 ms, so a lost report cannot stall the link
permanently.

The syslink transmit buffer must also hold a full queue of chunks — five packed
chunks is 1285 bytes — or `update()` gets authorised to queue chunks that
`send_packet()` then rejects.

## Phases

1. **Core** — UART ownership, framing/deframing, Fletcher-8, type demux, flow
   control gate, debug probe, `SYSL` logging. *(done)*
2. **Boot config** — `RADIO_READY` to lift the transmit gate, then
   `PM_BATTERY_AUTOUPDATE`, then channel/datarate/address/power from `SYSL_*`
   parameters. Every step but the autoupdate is echo-confirmed. *(done)*
3. **MAVLink telemetry** — `RegisteredPort` + `mavlink_packetise()` + 0x0E slot
   accounting. *(done)*

   Note the frame remainder is tracked explicitly rather than by re-running
   `mavlink_packetise()` after an oversized frame is split. Once the buffer
   starts mid-frame its first byte is payload, and roughly one time in 128 that
   byte is `0xFD` or `0xFE`, which `packetise()` reads as a frame header, judges
   incomplete, and returns 0 for — stranding the tail until unrelated later
   traffic happens to satisfy the bogus length. `FILE_TRANSFER_PROTOCOL` frames
   are oversized, so this presents as intermittent parameter and log download
   stalls.
4. **Battery** — 0x14 at init, parse 0x13, feed `AP_BattMonitor::handle_scripting()`.
5. **Cleanup** — migrate `CF_*` radio parameters into `SYSL_*`.
6. **P2P broadcast** — 0x0D, re-home the AI-deck mission-state broadcast.

## Debugging

`SYSLINK_DEBUG_PROBE` (0xF0) is the only real visibility into this link. The
nRF51 replies with 8 bytes: whether address/channel/datarate commands were
received, whether UART data was dropped, UART error flags and count, and two
syslink RX checksum error counters. Those, plus this driver's own counters, are
written to the `SYSL` log message at 1 Hz.

If the link is dead, check in this order:

1. `SYSL.RxP` climbing at all — if not, the nRF51 is not talking; check wiring
   and that nothing else claimed SERIAL2.
2. `SYSL.CkE` / probe `CKSUM1`,`CKSUM2` — framing or baud mismatch.
3. Probe `ADDR`/`CHAN`/`RATE` all 1 — configuration actually landed (phase 2).
4. Vehicle and ground station on the same channel and address.
