# E35 protocol v1

UART1 115200, 8N1. MCU TX B12 / RX B13. E35 is a transparent byte stream;
UART read boundaries and E35 radio packet boundaries are unrelated to frames.
Host commands remain ASCII plus CR/LF: START, STOP, CALIBRATE, DEBUG, JOY,u,d,t.

All MCU output: `COBS(version_type + payload + CRC16_LE) + 00`.
Version is the high nibble (1), type the low nibble. CRC-16/CCITT-FALSE:
poly 0x1021, init 0xFFFF, no reflection, xorout 0; checksum of ASCII 123456789 = 0x29B1.
CRC covers version/type and payload, before COBS. No native C structs are transmitted.
Multi-byte integers are little endian. Signed fields use two's complement.
CRC or length errors discard the frame, then restart at the next 00 delimiter.

## Type 1: control sample, 200 Hz

27 payload bytes; 30 decoded bytes including header/CRC; exactly 32 wire bytes.
Offsets below are relative to payload (not including version/type).

| Offset | Field | Encoding / displayed unit |
|---|---|---|
| 0 | control tick | uint16, 5 ms per tick |
| 2 | state/error | high nibble state, low nibble error |
| 3 | flags | bits listed below |
| 4 | yaw | int16 /100 degrees, wrapped -180..180 |
| 6 | yaw error | int16 /100 degrees |
| 8 | controller Z yaw rate | int16 /10 degrees/s, corrected sign/bias |
| 10,12 | left/right filtered speed | int16 /1000 m/s |
| 14 | turn command before slew | int8 /127, -1..1 |
| 15 | applied servo command offset from center | int8 /5 degrees |
| 16,17 | left/right motor duty | uint8 /5 percent |
| 18,20 | navigation X,Y | int16 /1000 m |
| 22 | monotonic path progress | uint16 /1000 m |
| 24 | projected path segment | uint8 |
| 25 | applied controller speed target | uint16 /1000 m/s |

Flags: bit0 calibrated; bit1 IMU valid; bit2 DEBUG; bit3 turn at limit;
bit4 steering slew active; bit5 motor duty at upper limit; bit6 reserved;
bit7 quantization clipped or nonfinite (nonfinite encodes as zero).
Servo offset is a software command, not measured wheel angle. Center is currently 99 degrees.
Duties describe controller outputs for the configured servo chassis.
State: INIT=0, ARMED=1, RUN=2, DONE=3, ERROR=4; error definitions in state_machine.h.
Unused accelerometer and gyro X/Y data are omitted from periodic telemetry.

Sample snapshot is taken after the control task and priority-0 output commit.
Only a bounded state/output copy masks interrupts; quantization, CRC and COBS run
in foreground. DMA sends persistent cache-aligned buffers. Only occupied cache
lines are cleaned. No fast-frame printf, dynamic allocation, or UART busy waits.
Commands and their RX/execution acknowledgements have foreground queue priority;
telemetry is coalesced under pressure, never replayed in a stale backlog.
Tick increments in PIT even when a sample/frame is skipped. A positive tick gap
can therefore mean a skipped firmware sample/send or link loss, not just radio loss.

## Type 2: events

UTF-8 event text, usually CRLF terminated, inside the same CRC/COBS envelope.
Examples: BOOT,PROTO1,5MS,200HZ; RX,START,RECEIVED; CMD,START,RUN,0;
CALIBRATION,PROGRESS,50. Existing command/calibration UI behavior is retained.

## Type 3: diagnostics, 1 Hz

32-byte payload: `<IiiIIIII` = full uint32 tick, signed raw left/right encoder
accumulators, peak control cycles, missed control ticks, TX queue drops,
TX errors, RX errors. Encoder accumulators retain the original raw electrical
signs (unlike the corrected wheel speeds); they are boot-relative and not reset
at each START. Peak control cycles include sensor/control work and snapshot,
not foreground packet encoding. Convert cycles using the 600 MHz core clock.
Diagnostics use 37 wire bytes, and events consume additional variable bandwidth.

## Time, logging, and outages

16-bit sample ticks wrap every 327.68 s. Host uses modulo differences within
a continuous segment. The first accepted sample is time_s=0. A >500 ms receive
outage, a BOOT event, or a detected backwards reset starts a new segment.
Never infer absolute time continuity across segments. Host wall time is receipt
time; time_s is the sample timeline and must be used for control plots.
Duplicate sample ticks are ignored. Missing samples and invalid frames are counted.

At 200 Hz, fast frames use 6400 B/s (55.56% of 11520 B/s available at 115200 8N1).
This is the UART budget, not a guarantee of actual E35 radio performance.

Every serial session automatically creates logs/YYYYMMDD_HHMMSS_microseconds:
- .csv: every valid unique control sample, physical units and flags, host time,
  relative sample time, tick, segment, and missing count;
- .jsonl: startup, calibration, acknowledgements, outgoing commands, diagnostics,
  parser statistics, receive outages/resumption, session closure;
- .bin: all received UART bytes, including damaged/partial frames, for replay.

Receive/parsing/logging run in the reader thread. Files use buffered writes and
flush at least once per second while data is arriving; normal disconnect closes
and flushes them. UI shows the latest sample at up to 30 Hz; UI clear does not
clear disk logs. A process/OS crash may lose the last buffered second of logging.
Disk errors are displayed explicitly. No MCU connection was required for offline
protocol, logging and GUI checks; wireless throughput/control timing need hardware.
