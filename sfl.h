#ifndef SFL_H
#define SFL_H

#include <stdint.h>

void sfl_initialize();
uint8_t sfl_process_measurements();
uint8_t sfl_fusion(uint32_t systick_value, uint32_t systick_reload);
void sfl_magnetic_calibration();

#endif
