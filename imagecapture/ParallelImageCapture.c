#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "ov5640.h"
#include "imagecapture/StateMachine.h"

#define IMAGECAPTURE_CODE(width, pclk, vsync, href) \
    { \
        /* 0 */ pio_encode_wait_gpio(0, vsync), \
        /* 1 */ pio_encode_wait_gpio(1, vsync), \
        /* .wrap_target */  \
        /* 2 */ pio_encode_wait_gpio(1, href), \
        /* 3 */ pio_encode_wait_gpio(1, pclk), \
        /* 4 */ pio_encode_in(pio_pins, width), \
        /* 5 */ pio_encode_wait_gpio(0, pclk), \
        /* .wrap */ \
    }

void common_hal_imagecapture_parallelimagecapture_construct(rp2pio_statemachine_obj_t *self, mcu_pin_obj_t *data, uint8_t data_clock, uint8_t vertical_sync, uint8_t horizontal_reference) {
    if (data_clock > 31 || vertical_sync > 31 || horizontal_reference > 31)
        printf("Error: pio_encode_wait_gpio does not support gpio n° > to 31");

    uint16_t imagecapture_code[] = IMAGECAPTURE_CODE(DATA_COUNT, data_clock, vertical_sync, horizontal_reference);

    common_hal_rp2pio_statemachine_construct(self,
        imagecapture_code, MP_ARRAY_SIZE(imagecapture_code),
        clock_get_hz(clk_sys), // full speed (4 instructions per loop -> max pclk 30MHz @ 120MHz)
        0, 0, // init
        NULL, 0, // may_exec
        NULL, 0, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // out pins
        data, DATA_COUNT, // in pins
        PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // in pulls
        NULL, 0, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // set pins
        #if DEBUG_STATE_MACHINE
        &pin_GPIO26, 3, PIO_PINMASK32_FROM_VALUE(7), PIO_PINMASK32_FROM_VALUE(7), // sideset pins
        #else
        NULL, 0, false, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // sideset pins
        #endif
        false, // No sideset enable
        NULL, PULL_NONE, // jump pin
        PIO_PINMASK_OR3(PIO_PINMASK_FROM_PIN(vertical_sync), PIO_PINMASK_FROM_PIN(horizontal_reference), PIO_PINMASK_FROM_PIN(data_clock)),
        // wait gpio pins
        true, // exclusive pin use
        false, 32, false, // out settings
        false, // wait for txstall
        true, 32, true,  // in settings
        false, // Not user-interruptible.
        2, 5, // wrap settings
        PIO_ANY_OFFSET,
        PIO_FIFO_TYPE_DEFAULT,
        PIO_MOV_STATUS_DEFAULT, PIO_MOV_N_DEFAULT);
}

size_t common_hal_imagecapture_parallelimagecapture_singleshot_capture(rp2pio_statemachine_obj_t *self, uint8_t *buff, size_t buff_size, bool isjpg) {
    PIO pio = self->pio;
    uint sm = self->state_machine;
    uint8_t offset = rp2pio_statemachine_program_offset(self);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    transfer_in(self, buff, buff_size, 4, false);

    pio_sm_set_enabled(pio, sm, false);

    if (isjpg) {
        for (size_t i = 0; i < buff_size - 1; i++) {
            if (buff[i] == 0xFF && buff[i + 1] == 0xD9) {
                // Return the length up to and including the marker
                printf("New image of size [%d]\n", i + 2);
                return i + 2;
            }
        }
        
        // Troubleshooting diagnostics
        printf("Image capture failed\n");
        // Print some header bytes
        printf("buffer head: ");
        for (size_t j = 0; j < 16 && j < buff_size; j++) printf("%02x ", buff[j]);
        printf("\n");
        // Print some tail bytes
        printf("buffer tail: ");
        for (size_t j = (buff_size > 16 ? buff_size - 16 : 0); j < buff_size; j++) printf("%02x ", buff[j]);
        printf("\n");
        // Look for JPEG start marker
        int start_idx = -1;
        for (size_t i = 0; i + 1 < buff_size; i++) {
            if ((uint8_t)buff[i] == 0xFF && (uint8_t)buff[i + 1] == 0xD8) {
                start_idx = i;
                break;
            }
        }
        if (start_idx >= 0) {
            printf("Found JPEG SOI at index %d\n", start_idx);
        } else {
            printf("JPEG SOI (0xFF 0xD8) not found in buffer\n");
        }
        // Basic stats
        printf("requested buf size: %d, fifo_depth: %u, actual_frequency: %u\n", (int)buff_size, self->fifo_depth, (unsigned)self->actual_frequency);
        // Check if buffer appears all zeros
        bool all_zero = true;
        for (size_t i = 0; i < buff_size; i++) {
            if (buff[i] != 0) { all_zero = false; break; }
        }
        printf("buffer all zero: %s\n", all_zero ? "yes" : "no");
        return 0;
    }
    return buff_size;
}

// ── Continuous (streaming) capture ──────────────────────────────────────────
// The PIO free-runs the sensor into a power-of-two ring buffer via a perpetual,
// write-address-wrapping DMA, so frames arrive at the sensor's native rate
// independent of when the host asks. `continuous_latest` extracts the newest
// complete JPEG (scanning back from the DMA write head for EOI then SOI). This
// decouples capture latency from the per-request protocol, unlike singleshot.
static int s_chan = -1;
static uint8_t *s_ring = NULL;
static size_t s_mask = 0;   // ring_size - 1 (ring size is a power of two)

void common_hal_imagecapture_parallelimagecapture_continuous_start(
    rp2pio_statemachine_obj_t *self, uint8_t *ring, uint32_t ring_size_bits) {
    PIO pio = self->pio;
    uint sm = self->state_machine;
    s_ring = ring;
    s_mask = ((size_t)1 << ring_size_bits) - 1;

    // Clear the ring before the perpetual writer starts. Otherwise it still holds
    // the previous stream session's bytes (or, after a RES switch, a differently
    // sized frame), and `continuous_latest` can lock onto a stale SOI/EOI pair and
    // hand back an old/torn frame on the first reads after STREAM 1 — the "corrupt
    // right after calibration" case. Zeroed bytes contain no JPEG markers, so no
    // frame is returned until a fresh one has actually been written.
    memset(ring, 0, s_mask + 1);

    uint8_t offset = rp2pio_statemachine_program_offset(self);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));   // align to the next VSYNC once

    if (s_chan < 0) s_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(s_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, self->rx_dreq);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, ring_size_bits);   // wrap the WRITE address
    const volatile void *rxf = (const volatile void *)&pio->rxf[sm];
    dma_channel_configure(s_chan, &c, ring, rxf, 0xFFFFFFFFu, true);   // perpetual
    pio_sm_set_enabled(pio, sm, true);
}

size_t common_hal_imagecapture_parallelimagecapture_continuous_latest(uint8_t *out, size_t out_max) {
    if (s_chan < 0 || !s_ring) return 0;
    const size_t size = s_mask + 1;
    // Keep this many bytes of clearance between the frame we return and the live
    // write head, to absorb the writes that land while we scan + copy.
    const size_t MARGIN = 2048;

    // Newest complete frame: scan back from the DMA write head for EOI, then SOI.
    // (JPEG byte-stuffing escapes every 0xFF in entropy data as 0xFF00, so a real
    // FF D9 / FF D8 never appears mid-scan — the first ones found are the true
    // frame boundaries. The ring was zeroed at STREAM start, so no stale markers.)
    uint32_t waddr0 = dma_hw->ch[s_chan].write_addr;
    size_t head0 = ((size_t)(waddr0 - (uint32_t)(uintptr_t)s_ring)) & s_mask;

    size_t eoi = SIZE_MAX;
    for (size_t k = 2; k < size; k++) {
        size_t p = (head0 + size - k) & s_mask;
        if (s_ring[p] == 0xD9 && s_ring[(p + size - 1) & s_mask] == 0xFF) { eoi = p; break; }
    }
    if (eoi == SIZE_MAX) return 0;
    size_t soi = SIZE_MAX;
    for (size_t k = 2; k < size; k++) {
        size_t p = (eoi + size - k) & s_mask;
        if (s_ring[p] == 0xFF && s_ring[(p + 1) & s_mask] == 0xD8) { soi = p; break; }
    }
    if (soi == SIZE_MAX) return 0;

    size_t len = (eoi + 1 + size - soi) & s_mask;
    if (len == 0 || len >= size || len > out_max) return 0;

    // Tear guard. The perpetual writer overwrites this frame's SOI once it advances
    // `size - len` bytes past the frame's EOI (a full lap of the ring back to SOI).
    // The 32 KiB ring barely holds one VGA JPEG, so that runway is small and the
    // free-running DMA laps into the frame we're copying → the classic half-gray /
    // tinted torn frame. Only hand the frame back while the write head is still
    // inside the runway (with MARGIN to spare) BEFORE and AFTER the copy; otherwise
    // bail and let the host's CAP retry loop try again a moment later. A torn frame
    // is thus dropped, never displayed — at the cost of an occasional retry.
    size_t runway = size - len;                    // bytes after EOI before SOI is hit
    size_t d0 = (head0 + size - eoi) & s_mask;     // bytes written since this EOI
    if (d0 + MARGIN >= runway) return 0;

    for (size_t k = 0; k < len; k++) out[k] = s_ring[(soi + k) & s_mask];

    uint32_t waddr1 = dma_hw->ch[s_chan].write_addr;
    size_t head1 = ((size_t)(waddr1 - (uint32_t)(uintptr_t)s_ring)) & s_mask;
    size_t d1 = (head1 + size - eoi) & s_mask;     // advance during the copy
    if (d1 < d0 || d1 + MARGIN >= runway) return 0;  // wrapped, or lapped into the frame

    return len;
}

void common_hal_imagecapture_parallelimagecapture_continuous_stop(rp2pio_statemachine_obj_t *self) {
    if (s_chan >= 0) { dma_channel_abort(s_chan); dma_channel_unclaim(s_chan); s_chan = -1; }
    s_ring = NULL;
    pio_sm_set_enabled(self->pio, self->state_machine, false);
}