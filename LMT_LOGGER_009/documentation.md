/*
===============================================================================
LMT LOGGER FIRMWARE — MAINTAINER REFERENCE
===============================================================================

READ THIS FIRST before modifying recording, playback, BLE, or flash logic.

===============================================================================
TABLE OF CONTENTS
===============================================================================

  1) DATA FORMATS (PRIMARY REFERENCE)
     1.1 Frame20
     1.2 Flash Page Layout
     1.3 PageFooter
     1.4 Binary Dump Records

  2) BINARY DUMP PROTOCOL
     2.1 Record Framing
     2.2 Page Footer Records
     2.3 Frame Records
     2.4 Decoder Requirements

  3) BOOT-TIME RECOVERY MODEL

  4) RUNTIME ARCHITECTURE
     4.1 Run Modes
     4.2 Main Loop Dispatch

  5) RECORDING MODEL

  6) PLAYBACK MODEL (CRITICAL)

  7) BLE ARCHITECTURE AND CONSTRAINTS

  8) OUTPUT PLANE SEPARATION

  9) SAFE VS UNSAFE CHANGES

 10) NOTES TO FUTURE MAINTAINERS

 11) HOST / FRONTEND DECODER GUIDANCE

===============================================================================
1) DATA FORMATS (PRIMARY REFERENCE)
===============================================================================

This section defines the canonical on-wire and on-flash data structures.
These definitions are stable and should be treated as protocol contracts.

-------------------------------------------------------------------------------
1.1 Frame20 (ATOMIC DATA RECORD)
-------------------------------------------------------------------------------

Frame20 is the fundamental IMU sample unit written to flash and emitted during
playback and binary dump.

  struct Frame20 {
    int16_t q0, q1, q2, q3;   // Orientation quaternion (Q15)
    int16_t ax, ay, az;       // Raw accelerometer axes
    int16_t mx, my, mz;       // Raw magnetometer axes
  };

Properties:
  - Size:        exactly 20 bytes
  - Endianness:  little-endian
  - Alignment:   packed (no padding)

Quaternion encoding:
  - Stored in signed Q15 fixed-point
  - float_value = int16_value / 32767.0
  - q0 is the scalar component
  - q1–q3 are the vector components
  - Quaternion is normalized at capture time
  - Minor normalization error due to quantization is expected

Accelerometer / Magnetometer:
  - Raw sensor units as reported by ICM-20948 AGMT
  - No scaling, filtering, or compensation applied in firmware

-------------------------------------------------------------------------------
1.2 FLASH PAGE LAYOUT (256 BYTES)
-------------------------------------------------------------------------------

Each flash page is self-contained and append-only.

  +-----------------------------+
  | Frame20[0]                  |
  | Frame20[1]                  |
  | ...                         |
  | Frame20[N-1]                |  N ≤ FRAMES_PER_PAGE (12)
  | (unused space = 0xFF)       |
  +-----------------------------+
  | PageFooter (16 bytes)       |
  +-----------------------------+

Constants:
  - FRAMES_PER_PAGE = 12
  - sizeof(Frame20) = 20 bytes
  - 12 * 20 + 16 = 256 bytes

-------------------------------------------------------------------------------
1.3 PageFooter (END-OF-PAGE METADATA)
-------------------------------------------------------------------------------

  struct PageFooter {
    uint32_t magic;         // Must equal PAGE_MAGIC ('PAGE')
    uint16_t validFrames;   // Number of valid Frame20 entries
    uint16_t crc16;         // CRC-16-CCITT (diagnostic)
    uint32_t firstFrameID;  // Global ID of first frame in page
    uint32_t pageStartMs;   // millis() at first frame
  };

Properties:
  - Size:        exactly 16 bytes
  - Endianness:  little-endian
  - Padding:     none

Field semantics:

  magic
    - Marks the page as written
    - Flash scanning stops at first page where magic != PAGE_MAGIC

  validFrames
    - Number of valid Frame20 records at start of page
    - Range: 0 .. FRAMES_PER_PAGE

  crc16
    - CRC-16-CCITT (poly 0x1021, init 0xFFFF)
    - Computed over:
        validFrames * sizeof(Frame20)
        + offsetof(PageFooter, crc16)
    - Diagnostic only:
        * CRC failure does NOT invalidate the page
        * CRC warnings are reported during playback

  firstFrameID
    - Global, monotonically increasing frame ID
    - Per-frame ID reconstructed as:
        frameID = firstFrameID + frameIndexInPage

  pageStartMs
    - millis() captured at first frame of page
    - Wraparound (~49 days) is expected and acceptable

-------------------------------------------------------------------------------
1.4 BINARY DUMP RECORD TYPES
-------------------------------------------------------------------------------

Binary dump output consists of two record types:

  1) PAGE FOOTER RECORD
  2) FRAME RECORD

For each flash page, records are emitted in this order:

  [PAGE FOOTER]
  [FRAME]
  [FRAME]
  ...
  [FRAME]   (validFrames times)

===============================================================================
2) BINARY DUMP PROTOCOL
===============================================================================

-------------------------------------------------------------------------------
2.1 RECORD FRAMING (COMMON HEADER)
-------------------------------------------------------------------------------

All binary records share a 4-byte header:

  Offset  Size  Description
  ------  ----  ------------------------------------------
   0       1    Sync byte 0
   1       1    Sync byte 1
   2       2    Payload length (uint16, little-endian)

The payload immediately follows the header.

-------------------------------------------------------------------------------
2.2 PAGE FOOTER RECORD
-------------------------------------------------------------------------------

Emitted ONCE per page, before any frames from that page.

  Sync bytes:     0x56 0xAA
  Payload length: 16 bytes

  [0x56 0xAA] [0x10 0x00] [PageFooter bytes]

-------------------------------------------------------------------------------
2.3 FRAME RECORD
-------------------------------------------------------------------------------

Emitted once per stored IMU sample.

  Sync bytes:     0x55 0xAA
  Payload length: 20 bytes

  [0x55 0xAA] [0x14 0x00] [Frame20 bytes]

Total record size: 24 bytes

-------------------------------------------------------------------------------
2.4 DECODER REQUIREMENTS (MANDATORY)
-------------------------------------------------------------------------------

A correct host decoder MUST:

  - Treat input as a raw byte stream
  - Scan for sync bytes
  - Validate payload length
  - Decode only when a full record is present
  - Resynchronize on mismatch
  - Ignore ASCII outside binary records

BLE notifications and serial reads DO NOT align with records.

===============================================================================
3) BOOT-TIME RECOVERY MODEL
===============================================================================

At boot:

  1) Flash is scanned page-by-page from page 0
  2) Scan stops at first page where footer.magic != PAGE_MAGIC
  3) currentPage = pagesFound
  4) If at least one page exists:

       frameCounter =
         lastPage.firstFrameID + lastPage.validFrames

Guarantees:
  - Append-only logging
  - No frame ID reuse
  - Power-loss-safe recovery
  - Deterministic continuity across sessions

===============================================================================
4) RUNTIME ARCHITECTURE
===============================================================================

-------------------------------------------------------------------------------
4.1 RUN MODES (MUTUALLY EXCLUSIVE)
-------------------------------------------------------------------------------

Exactly ONE mode is active at any time:

  - MODE_IDLE
  - MODE_RECORDING
  - MODE_PLAYBACK

This prevents:
  - SPI bus contention
  - BLE starvation
  - Timing jitter
  - Undefined subsystem interaction

-------------------------------------------------------------------------------
4.2 MAIN LOOP DISPATCH
-------------------------------------------------------------------------------

loop() acts as a dispatcher:

  MODE_IDLE:
    - USB Serial command parsing
    - BLE UART command parsing (if enabled)
    - No IMU reads
    - No flash I/O

  MODE_RECORDING:
    - Read DMP FIFO
    - Build Frame20 records
    - Write full pages to flash

  MODE_PLAYBACK:
    - Read flash pages
    - Emit exactly ONE frame per loop iteration

===============================================================================
5) RECORDING MODEL
===============================================================================

Data flow:

  DMP FIFO
    → Quaternion + AGMT
    → Frame20
    → RAM page buffer
    → Flash page (256 bytes)

Recording guarantees:
  - Frames logged sequentially
  - Flash writes occur only when a page is full
  - Pages are atomic and CRC-protected
  - Recording never overlaps playback or command handling

===============================================================================
6) PLAYBACK MODEL (CRITICAL)
===============================================================================

Playback is intentionally loop-paced:

  - At most ONE frame emitted per loop() iteration
  - Page loading and frame emission are separated

This pacing is REQUIRED for reliable BLE UART behavior.

Violating this will:
  - Overrun BLE buffers
  - Mangle output
  - Collapse throughput
  - Risk SoftDevice asserts

===============================================================================
7) BLE ARCHITECTURE AND CONSTRAINTS
===============================================================================

BLE stack rules:

  - Bluefruit.begin() is called EXACTLY ONCE
  - All services are registered EXACTLY ONCE
  - BLE behavior is controlled ONLY via advertising

Safe operations:
  - Start / stop advertising
  - Modify advertised services

Unsafe operations (DO NOT DO):
  - Reinitializing BLE
  - Restarting services
  - Reconfiguring BLE during notifications

===============================================================================
8) OUTPUT PLANE SEPARATION
===============================================================================

There are THREE distinct output planes:

  CONTROL:
    - Low-rate, structured output
    - Status, help, identification
    - Routed via emitControl()

  EVENT:
    - Single-line state transitions
    - Mode changes, notifications
    - Routed via emitEvent()

  DATA:
    - High-rate streaming (frames, dumps)
    - Emitted directly to Serial / BLE UART

DATA output MUST NEVER pass through CONTROL or EVENT paths.

===============================================================================
9) SAFE VS UNSAFE CHANGES
===============================================================================

Safe:
  - Modifying frame contents
  - Adding commands
  - Adjusting IMU rates
  - Adding dump modes

Unsafe without deep review:
  - Restarting BLE
  - Merging output paths
  - Emitting multiple frames per loop
  - Flash I/O during BLE streaming

===============================================================================
10) NOTES TO FUTURE MAINTAINERS
===============================================================================

This firmware is intentionally stateful.

That state exists to:
  - Respect SoftDevice timing
  - Preserve BLE throughput
  - Avoid spontaneous resets
  - Keep IMU logging deterministic

If something looks redundant:
  Assume it enforces a hard-won invariant.

===============================================================================
11) HOST / FRONTEND DECODER GUIDANCE (SUMMARY)
===============================================================================

Host decoders MUST:

  - Treat data as binary
  - Never convert to UTF-8 strings
  - Buffer aggressively
  - Resynchronize on sync words
  - Decode only complete records

Debug tip:
  If you do not see "55 AA 14 00", you are not in binary mode.

===============================================================================
END OF DOCUMENT
===============================================================================
*/
