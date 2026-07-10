void common_hal_imagecapture_parallelimagecapture_construct(rp2pio_statemachine_obj_t *self, mcu_pin_obj_t *data, uint8_t data_clock, uint8_t vertical_sync, uint8_t horizontal_reference);
int common_hal_imagecapture_parallelimagecapture_singleshot_capture(rp2pio_statemachine_obj_t *self, uint8_t *buff, size_t buff_size, bool isjpg);

// Continuous (streaming) capture: perpetual ring-buffer DMA at the sensor rate.
// `ring` must be aligned to (1 << ring_size_bits) bytes.
void common_hal_imagecapture_parallelimagecapture_continuous_start(rp2pio_statemachine_obj_t *self, uint8_t *ring, uint32_t ring_size_bits);
size_t common_hal_imagecapture_parallelimagecapture_continuous_latest(uint8_t *out, size_t out_max);
void common_hal_imagecapture_parallelimagecapture_continuous_stop(rp2pio_statemachine_obj_t *self);