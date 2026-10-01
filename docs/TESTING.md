# Testing

airlow can be exercised end to end **without Bluetooth hardware**. `src/sim.rs` implements the same
`Backend` trait as the USB transport and simulates:

* **A controller**: HCI command/event handling, ACL buffers and credits, packet fragmentation,
  radio throughput, the automatic flush timeout (Flush Occurred events), stale events, auto-reconnects.
* **An AirPods-style sink**, modelled on logs from a real AirPods Pro: it opens SDP to the host and
  replays the real queries byte for byte, runs the AVRCP exchange, enforces the AVDTP state machine,
  pauses the stream if media is late, drops stale/duplicate RTP, and decodes every SBC frame with
  libsbc so tests assert on the audio that would actually be heard (pitch and amplitude).

The sink is deliberately strict: any protocol error is recorded and fails the test.

```
cargo test --release              # ~5 s, 32 tests
cargo test --release -- --nocapture --test-threads=1   # see measured numbers
airlow captest [seconds]          # no Bluetooth: checks WASAPI loopback capture on this PC
```

## What is covered
Pairing and reconnect, stale/late/incoming connection events, the full A2DP session (SDP, AVRCP, AVDTP,
SBC config), RTP continuity, SBC round trip, fragmentation (several MTUs), credit flow control,
auto-flush, dead-radio diagnostics, live capture path (10/30 ms capture bursts, idle silence
keep-alive), failure paths (page timeout, hang-up, rejected config), and wire-format helpers.

Every bug found on hardware or in review has a regression test. The suite was mutation-tested: 12
historical bugs were re-introduced one at a time and each was caught by at least one test.

## What it cannot tell you
* How a real AirPods *firmware* behaves beyond what the logs showed: the simulator encodes what we
  have observed, so it can be wrong where we have not looked.
* Real radio conditions, USB timing on a given machine, and the AirPods' own playback buffer.
* End-to-end latency. The "pipeline latency" printed by the live tests covers only our software
  (capture buffer to packet on the wire); it excludes USB, the radio and the sink's buffer.
