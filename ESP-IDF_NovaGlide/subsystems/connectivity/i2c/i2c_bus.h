#pragma once
#include "driver/i2c_master.h"

void i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_get(void);
