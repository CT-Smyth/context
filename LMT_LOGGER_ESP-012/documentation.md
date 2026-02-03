===============================================================================
LMT LOGGER FIRMWARE — ARCHITECTURE & PROTOCOL REFERENCE
===============================================================================

This document defines the architecture, data formats, runtime model, and
external interfaces of the LMT Logger firmware.

It is the canonical reference for maintainers, host-side tooling authors, and
future production firmware variants.

All formats and behaviors described here reflect the current implementation.

===============================================================================
OVERVIEW
===============================================================================

The LMT Logger firmware is a deterministic, append-only motion capture system
designed to bridge low-level human motion sensing with higher-level AI systems.

Its responsibilities are intentionally narrow:

  - Acquire IMU orientation and raw sensor data at a fixed rate
  - Persist that data reliably to flash
  - Expose multiple low-overhead extraction interfaces
  - Maintain continuity and recoverability across power cycles

Interpretation, inference, learning, and semantics are explicitly out of scope
and are delegated to host-side systems.

===============================================================================
SYSTEM ARCHITECTURE
===============================================================================

The firmware is structured as a set of tightly scoped modules coordinated by a
single orchestration layer (.ino).

Separation of concerns is strict:

  - LoggerCore      : data model, state machines, flash logic
  - LoggerCLI       : human command interface
  - LoggerBLE       : BLE UART transport
  - LoggerOTA       : Wi-Fi + OTA lifecycle
  - LoggerHTTP      : read-only HTTP extraction API
  - SPIFlash        : external / emulated flash abstraction
  - .ino            : hardware mapping, boot choreography, run-mode scheduling

The .ino owns **policy** (when things happen).
The modules own **mechanism** (how they happen).

===============================================================================
RUNTIME MODEL
===============================================================================

Exactly one run mode is active at all times:

  MODE_IDLE
    - CLI (USB + BLE)
    - OTA + HTTP (if enabled)
    - Live IMU probes
    - No recording or playback

  MODE_RECORDING
    - Fixed-interval IMU acquisition
    - Append-only flash writes
    - No CLI or playback

  MODE_PLAYBACK
    - Deterministic, loop-paced data emission
    - No recording or CLI

Modes are mutually exclusive by design. This prevents:
  - SPI contention
  - BLE buffer overruns
  - Timing jitter
  - Undefined subsystem interaction

===============================================================================
DATA FORMATS (PROTOCOL CONTRACTS)
===============================================================================

All formats below are stable unless explicitly versioned.

-------------------------------------------------------------------------------
Frame20 — Atomic Motion Sample
-------------------------------------------------------------------------------

struct Frame20 {
  int16_t q0, q1, q2, q3;   // Orientation quaternion (Q15)
  int16_t ax, ay, az;       // Raw accelerometer axes
  int16_t mx, my, mz;       // Raw magnetometer axes
};

Properties:
  - Size        : exactly 20 bytes
  - Endianness : little-endian
  - Alignment  : packed, no padding

Quaternion:
  - Signed Q15 fixed-point
  - float = value / 32767.0
  - q0 is scalar component
  - q1–q3 are vector components
  - Normalized at capture time

Accel / Mag:
  - Raw ICM-20948 units
  - No scaling or compensation applied

-------------------------------------------------------------------------------
Flash Page Layout (256 bytes)
-------------------------------------------------------------------------------

Each flash page is self-contained and append-only.

  +-----------------------------+
  | Frame20[0]                  |
  | Frame20[1]                  |
  | ...                         |
  | Frame20[N-1]                |  N ≤ FRAMES_PER_PAGE (12)
  | (unused = 0xFF)             |
  +-----------------------------+
  | PageFooter (16 bytes)       |
  +-----------------------------+

Constants:
  - FLASH_PAGE_SIZE   = 256
  - FRAMES_PER_PAGE   = 12
  - sizeof(Frame20)   = 20
  - sizeof(PageFooter)= 16

-------------------------------------------------------------------------------
PageFooter — End-of-Page Metadata
-------------------------------------------------------------------------------

struct PageFooter {
  uint32_t magic;         // 'PAGE'
  uint16_t validFrames;   // 0..FRAMES_PER_PAGE
  uint16_t crc16;         // CRC-16-CCITT (diagnostic)
  uint32_t firstFrameID;  // global frame ID of first frame
  uint32_t pageStartMs;   // millis() at first frame
};

CRC:
  - Polynomial: 0x1021
  - Init: 0xFFFF
  - Computed over:
      validFrames * sizeof(Frame20)
      + offsetof(PageFooter, crc16)

CRC is diagnostic only:
  - CRC failure does NOT invalidate a page
  - CRC status is reported during playback and HTTP streaming

-------------------------------------------------------------------------------
Flash Layout Notes
-------------------------------------------------------------------------------

  - Pages are written sequentially from page 0
  - Scan stops at first page where footer.magic != 'PAGE'
  - Logging is append-only
  - No in-place modification of logged data

Reserved tail region:
  - Last 256 pages of flash
  - Used for indexed 256-byte storage elements
  - Slot[0] is virtual (MCU serial)
  - Slots[1..] stored physically

===============================================================================
BOOT-TIME RECOVERY MODEL
===============================================================================

At boot:

  1) Flash is scanned page-by-page from page 0
  2) Scan stops at first invalid footer.magic
  3) currentPage = pagesFound
  4) frameCounter reconstructed as:
       lastPage.firstFrameID + lastPage.validFrames

Guarantees:
  - Power-loss safety
  - No frame ID reuse
  - Deterministic continuity across sessions

===============================================================================
RECORDING MODEL
===============================================================================

Data flow:

  IMU (DMP + AGMT)
    → Frame20
    → RAM page buffer
    → Flash page (atomic write)

Properties:
  - Fixed-interval sampling (policy owned by .ino)
  - Flash writes only when a page is full
  - No flash I/O during BLE streaming or playback

===============================================================================
PLAYBACK MODEL (CRITICAL)
===============================================================================

Playback is intentionally **loop-paced**:

  - At most ONE frame emitted per loop() iteration
  - Page loading and frame emission are separated

This pacing is REQUIRED to:
  - Preserve BLE reliability
  - Avoid TX buffer overruns
  - Maintain deterministic behavior

Violating this invariant will break BLE and destabilize the system.

===============================================================================
INTERFACES
===============================================================================

-------------------------------------------------------------------------------
CLI (USB + BLE)
-------------------------------------------------------------------------------

Human-oriented control interface.

  - Commands are line-oriented
  - Synchronous execution
  - Routed through CONTROL and EVENT planes

-------------------------------------------------------------------------------
BLE UART
-------------------------------------------------------------------------------

Nordic-style NUS service.

  - Initialized once
  - Services registered once
  - Enabled/disabled via advertising only
  - No buffering guarantees
  - RX feeds CLI only in MODE_IDLE

-------------------------------------------------------------------------------
OTA
-------------------------------------------------------------------------------

ArduinoOTA-based firmware updates.

  - Wi-Fi enabled only while OTA active
  - Credentials stored in flash slots
  - Blocking connect during startup (intentional)
  - HTTP server started alongside OTA

-------------------------------------------------------------------------------
HTTP API (Read-Only)
-------------------------------------------------------------------------------

Active only while OTA is enabled.

Endpoints:

  GET /id
    - Returns device identifier (ASCII)

  GET /flash
    - Streams entire recorded log
    - Chunked binary response
    - Pages streamed sequentially

Each page is preceded by:

struct FlashPageHeader {
  uint32_t magic;        // 'LMTP'
  uint32_t pageIndex;
  uint16_t pageSize;     // 256
  uint16_t validFrames;
  uint16_t crc16;
  uint16_t flags;        // bit0: footer valid, bit1: CRC ok
};

Logging may continue concurrently with streaming.

===============================================================================
OUTPUT PLANES
===============================================================================

There are three strictly separated output planes:

  CONTROL
    - Status, help, structured output
    - Routed via emitControl()

  EVENT
    - Single-line state transitions
    - Routed via emitEvent()

  DATA
    - High-rate binary or frame streams
    - Written directly to Serial / BLE

DATA output must never pass through CONTROL or EVENT paths.

===============================================================================
SAFE VS UNSAFE CHANGES
===============================================================================

Safe:
  - Adding commands
  - Adjusting IMU rates
  - Extending frame contents (with versioning)
  - Adding new extraction endpoints

Unsafe without deep review:
  - Reinitializing BLE
  - Emitting multiple frames per loop
  - Mixing output planes
  - Flash writes during playback or BLE streaming

===============================================================================
NOTES TO FUTURE MAINTAINERS
===============================================================================

This firmware is intentionally stateful.

That state exists to enforce timing, ordering, and safety constraints that are
not obvious until violated.

If something appears redundant or conservative:
  assume it encodes a hard-won invariant.

===============================================================================
END OF DOCUMENT
===============================================================================

_______________________________________________________________________________

===============================================================================
LMT LOGGER — BINARY FORMAT REFERENCE
===============================================================================

This document defines the binary data formats emitted by the LMT Logger firmware.

It is intended for authors of host-side decoders, data ingestion pipelines,
analysis tools, and long-term archival systems.

This document is authoritative for all binary interfaces unless explicitly
versioned otherwise.

===============================================================================
SCOPE
===============================================================================

This specification covers:

  - Binary playback over Serial and BLE UART
  - Binary flash export over HTTP (/flash)
  - On-flash page structure (for offline decoding)

It does NOT cover:
  - ASCII CLI output
  - Human-readable status messages
  - Transport-layer framing (USB, BLE, TCP)

Decoders MUST treat all inputs as raw byte streams.

===============================================================================
GENERAL CONVENTIONS
===============================================================================

Byte order:
  - All multi-byte values are LITTLE-ENDIAN

Alignment:
  - All structures are PACKED
  - No padding bytes are inserted

Integer types:
  - uint16_t = 2 bytes
  - uint32_t = 4 bytes
  - int16_t  = 2 bytes (two’s complement)

Unless stated otherwise:
  - All sizes are exact
  - All fields are mandatory

===============================================================================
CORE DATA STRUCTURES
===============================================================================

-------------------------------------------------------------------------------
Frame20 — Atomic Motion Sample
-------------------------------------------------------------------------------

Frame20 is the fundamental motion record used everywhere:
  - on flash
  - in playback
  - in binary dumps

Structure (20 bytes):

  Offset  Size  Type     Name
  ------  ----  -------  --------------------------------
   0       2    int16    q0   (quaternion scalar)
   2       2    int16    q1
   4       2    int16    q2
   6       2    int16    q3
   8       2    int16    ax   (accelerometer X)
  10       2    int16    ay
  12       2    int16    az
  14       2    int16    mx   (magnetometer X)
  16       2    int16    my
  18       2    int16    mz

Properties:
  - Size: exactly 20 bytes
  - Quaternion encoding: signed Q15
      float = value / 32767.0
  - Accel / Mag: raw sensor units

-------------------------------------------------------------------------------
PageFooter — End-of-Page Metadata
-------------------------------------------------------------------------------

Stored at the end of each flash page.

Structure (16 bytes):

  Offset  Size  Type     Name
  ------  ----  -------  -------------------------------
   0       4    uint32   magic        ('PAGE' = 0x50414745)
   4       2    uint16   validFrames
   6       2    uint16   crc16
   8       4    uint32   firstFrameID
  12       4    uint32   pageStartMs

CRC:
  - CRC-16-CCITT
  - Polynomial: 0x1021
  - Initial value: 0xFFFF
  - Computed over:
      validFrames * sizeof(Frame20)
      + offsetof(PageFooter, crc16)

CRC is DIAGNOSTIC ONLY.
CRC failure does not invalidate the page.

===============================================================================
BINARY PLAYBACK STREAM (SERIAL / BLE)
===============================================================================

Binary playback emits a stream of records.
Records are NOT aligned to read boundaries and may be split arbitrarily.

-------------------------------------------------------------------------------
Record Framing (Common)
-------------------------------------------------------------------------------

All binary records share a 4-byte header:

  Offset  Size  Type     Description
  ------  ----  -------  -------------------------------
   0       1    uint8    Sync byte 0
   1       1    uint8    Sync byte 1
   2       2    uint16   Payload length (bytes)

Payload immediately follows the header.

Decoders MUST:
  - Scan for sync bytes
  - Validate payload length
  - Decode only when full payload is available
  - Resynchronize on mismatch

-------------------------------------------------------------------------------
Record Type: PAGE FOOTER RECORD
-------------------------------------------------------------------------------

Emitted once per page, before any frames from that page.

Header:
  Sync bytes:     0x56 0xAA
  Payload length: 16

Payload:
  - PageFooter structure (16 bytes)

Full record size:
  4 + 16 = 20 bytes

-------------------------------------------------------------------------------
Record Type: FRAME RECORD
-------------------------------------------------------------------------------

Emitted once per stored frame.

Header:
  Sync bytes:     0x55 0xAA
  Payload length: 20

Payload:
  - Frame20 structure (20 bytes)

Full record size:
  4 + 20 = 24 bytes

-------------------------------------------------------------------------------
Playback Ordering
-------------------------------------------------------------------------------

For each flash page:

  [PAGE FOOTER RECORD]
  [FRAME RECORD]
  [FRAME RECORD]
  ...
  [FRAME RECORD]   (validFrames times)

No other binary records are emitted during playback.

===============================================================================
HTTP FLASH EXPORT STREAM (/flash)
===============================================================================

The HTTP /flash endpoint streams the entire recorded log as a chunked binary
HTTP response.

Transport:
  - Content-Type: application/octet-stream
  - Transfer-Encoding: chunked

The payload is a pure binary stream with no delimiters other than chunk framing.

-------------------------------------------------------------------------------
FlashPageHeader — HTTP Stream Page Header
-------------------------------------------------------------------------------

Each flash page is preceded by a fixed-size header.

Structure (16 bytes):

  Offset  Size  Type     Name
  ------  ----  -------  -------------------------------
   0       4    uint32   magic        ('LMTP' = 0x4C4D5450)
   4       4    uint32   pageIndex
   8       2    uint16   pageSize     (always 256)
  10       2    uint16   validFrames
  12       2    uint16   crc16
  14       2    uint16   flags

Flags bitfield:
  bit 0 (0x0001): PageFooter present and valid
  bit 1 (0x0002): CRC check passed

-------------------------------------------------------------------------------
HTTP Stream Ordering
-------------------------------------------------------------------------------

For each page, sequentially from page 0 to currentPage - 1:

  [FlashPageHeader]
  [256 bytes raw flash page data]

No compression or transformation is applied.

Logging may continue concurrently with streaming.

===============================================================================
ON-FLASH DATA (OFFLINE DECODING)
===============================================================================

Offline decoding (e.g. from a raw flash image) uses the same structures:

  - Flash is read in 256-byte pages
  - Each page ends with a PageFooter
  - Scan stops at first page where footer.magic != 'PAGE'

This allows deterministic reconstruction without external metadata.

===============================================================================
DECODER REQUIREMENTS (MANDATORY)
===============================================================================

A correct decoder MUST:

  - Treat all inputs as raw binary
  - Never assume read alignment equals record alignment
  - Never assume records arrive contiguously
  - Resynchronize on sync byte mismatch
  - Validate payload lengths
  - Handle CRC failures gracefully
  - Ignore any ASCII text outside binary mode

BLE notifications and USB serial reads may split records arbitrarily.

===============================================================================
COMMON FAILURE MODES (HOST SIDE)
===============================================================================

If decoding fails, check:

  - Are you treating the stream as UTF-8 text? (This is wrong.)
  - Are you assuming fixed read sizes? (This is wrong.)
  - Are you ignoring sync bytes? (This is wrong.)
  - Are you emitting more than one frame per loop on firmware? (Bug.)

Debug heuristic:
  - If you never see 55 AA 14 00, you are not in binary frame mode.

===============================================================================
VERSIONING
===============================================================================

This format is currently unversioned.

Any future incompatible change MUST:
  - Introduce explicit version fields, OR
  - Use new sync bytes, OR
  - Be documented in a superseding version of this document

===============================================================================
END OF DOCUMENT
===============================================================================