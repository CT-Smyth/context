#include "rdts_decode.h"
#include <string.h>

/*
 * RDTS wire-format constants
 */
#define RDTS_FIXED_HEADER_LEN 8   // version..reserved
#define RDTS_TIME_LEN         8   // uint64_t unix ms

/*
 * Upper plausibility bound for Unix time in milliseconds.
 * Year 2100-01-01 UTC ≈ 4102444800000 ms.
 *
 * This is decode-level validation:
 * values above this cannot represent real time in this protocol.
 */
#define RDTS_MAX_UNIX_MS 4102444800000ULL

rdts_decode_result_t rdts_decode_packet(
  const uint8_t *payload,
  uint8_t len,
  rdts_packet_t *out)
{
  if (!payload || !out)
    return RDTS_DECODE_ERR_INTERNAL;

  /*
   * Minimum possible length:
   * fixed header + unix time
   *
   * Note: MAC may be omitted only when RDTS_FLAG_NOAUTH is set.
   */
  if (len < RDTS_FIXED_HEADER_LEN + RDTS_TIME_LEN)
    return RDTS_DECODE_ERR_LEN;

  memset(out, 0, sizeof(rdts_packet_t));

  const uint8_t *p = payload;

  out->version     = p[0];
  out->addr_mode   = p[1];
  out->addr_count  = p[2];
  out->window_len  = p[3];
  out->mode        = p[4];
  out->flags       = p[5];
  out->reserved    = p[6] | (p[7] << 8);
  p += RDTS_FIXED_HEADER_LEN;

  /* ---- Structural protocol checks ---- */

  if (out->version != 1)
    return RDTS_DECODE_ERR_VERSION;

  if (out->addr_mode > RDTS_ADDR_LIST)
    return RDTS_DECODE_ERR_ADDR_MODE;

  if (out->addr_count > RDTS_MAX_ADDRS)
    return RDTS_DECODE_ERR_ADDR_COUNT;

  if (out->reserved != 0)
    return RDTS_DECODE_ERR_RESERVED;

  /* ---- master_unix_ms ---- */

  if ((p + RDTS_TIME_LEN) > (payload + len))
    return RDTS_DECODE_ERR_LEN;

  memcpy(&out->master_unix_ms, p, RDTS_TIME_LEN);
  p += RDTS_TIME_LEN;

  /*
   * Decode-level plausibility: reject values that cannot represent Unix time in ms.
   * Year 2100-01-01 UTC ~= 4102444800000 ms since epoch.
   */
  if (out->master_unix_ms > 4102444800000ULL)
    return RDTS_DECODE_ERR_TIME_RANGE;

  /* ---- addr_list ---- */

  size_t addr_bytes = out->addr_count * sizeof(uint32_t);
  if ((p + addr_bytes) > (payload + len))
    return RDTS_DECODE_ERR_LEN;

  memcpy(out->addr_list, p, addr_bytes);
  p += addr_bytes;

  /* ---- MAC handling ----
   *
   * If NOAUTH flag is NOT set, MAC is required.
   * If NOAUTH flag IS set, MAC may be absent; in that case out->mac is zeroed.
   */
  bool have_mac = ((p + RDTS_MAC_LEN) <= (payload + len));

  if (!(out->flags & RDTS_FLAG_NOAUTH)) {
    // Authenticated/canonical mode: MAC must be present
    if (!have_mac)
      return RDTS_DECODE_ERR_MAC;

    memcpy(out->mac, p, RDTS_MAC_LEN);
  } else {
    // Noauth/test mode: accept missing MAC
    if (have_mac) {
      memcpy(out->mac, p, RDTS_MAC_LEN);
    } else {
      memset(out->mac, 0, RDTS_MAC_LEN);
    }
  }

  return RDTS_DECODE_OK;
}

// rdts_decode_result_t rdts_decode_packet(
//   const uint8_t *payload,
//   uint8_t len,
//   rdts_packet_t *out)
// {
//   if (!payload || !out)
//     return RDTS_DECODE_ERR_INTERNAL;

//   /*
//    * Minimum possible length:
//    * fixed header + unix time
//    * (MAC may be omitted in test / unsecured mode)
//    */
//   if (len < RDTS_FIXED_HEADER_LEN + RDTS_TIME_LEN)
//     return RDTS_DECODE_ERR_LEN;

//   memset(out, 0, sizeof(rdts_packet_t));

//   const uint8_t *p = payload;

//   out->version     = p[0];
//   out->addr_mode   = p[1];
//   out->addr_count  = p[2];
//   out->window_len  = p[3];
//   out->mode        = p[4];
//   out->flags       = p[5];
//   out->reserved    = p[6] | (p[7] << 8);
//   p += RDTS_FIXED_HEADER_LEN;

//   /* ---- Structural protocol checks ---- */

//   if (out->version != 1)
//     return RDTS_DECODE_ERR_VERSION;

//   if (out->addr_mode > RDTS_ADDR_LIST)
//     return RDTS_DECODE_ERR_ADDR_MODE;

//   if (out->addr_count > RDTS_MAX_ADDRS)
//     return RDTS_DECODE_ERR_ADDR_COUNT;

//   if (out->reserved != 0)
//     return RDTS_DECODE_ERR_RESERVED;

//   /* ---- master_unix_ms ---- */

//   if ((p + RDTS_TIME_LEN) > (payload + len))
//     return RDTS_DECODE_ERR_LEN;

//   memcpy(&out->master_unix_ms, p, RDTS_TIME_LEN);
//   p += RDTS_TIME_LEN;

//   /*
//    * Plausibility check:
//    * This value must be representable as Unix epoch time.
//    * Extremely large values indicate uninitialized memory,
//    * width/packing mismatches, or protocol violations.
//    */
//   if (out->master_unix_ms > RDTS_MAX_UNIX_MS)
//     return RDTS_DECODE_ERR_TIME_RANGE;

//   /* ---- addr_list ---- */

//   size_t addr_bytes = out->addr_count * sizeof(uint32_t);
//   if ((p + addr_bytes) > (payload + len))
//     return RDTS_DECODE_ERR_LEN;

//   memcpy(out->addr_list, p, addr_bytes);
//   p += addr_bytes;

//   /* ---- MAC (optional) ---- */

//   if ((p + RDTS_MAC_LEN) <= (payload + len)) {
//     memcpy(out->mac, p, RDTS_MAC_LEN);
//   } else {
//     memset(out->mac, 0, RDTS_MAC_LEN);
//   }

//   return RDTS_DECODE_OK;
// }

const char *rdts_decode_result_str(rdts_decode_result_t res)
{
  switch (res) {
    case RDTS_DECODE_OK:
      return "OK";
    case RDTS_DECODE_ERR_LEN:
      return "LEN";
    case RDTS_DECODE_ERR_VERSION:
      return "VERSION";
    case RDTS_DECODE_ERR_ADDR_MODE:
      return "ADDR_MODE";
    case RDTS_DECODE_ERR_ADDR_COUNT:
      return "ADDR_COUNT";
    case RDTS_DECODE_ERR_MAC:
      return "MAC";
    case RDTS_DECODE_ERR_RESERVED:
      return "RESERVED";
    case RDTS_DECODE_ERR_TIME_RANGE:
      return "TIME_RANGE";
    case RDTS_DECODE_ERR_INTERNAL:
      return "INTERNAL";
    default:
      return "UNKNOWN";
  }
}