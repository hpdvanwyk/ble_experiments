/*
Copyright (c) 2019 Hendrik van Wyk
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef OW_TEMP_H
#define OW_TEMP_H

#include "utility/onewire.h"
#include "utility/hires.h"
#include "app_timer.h"

typedef struct
{
    int16_t                    temperature;
    sBSPACMonewireSerialNumber serial;
} ow_temp_reading_t;

typedef void (*temp_callback)(ow_temp_reading_t* reading);
typedef struct
{
    sBSPACMonewireBus bus_config;
    hBSPACMonewireBus bus;
    int               external_power;
    uint32_t          power_pin;
    app_timer_id_t    timer_id;
    ow_temp_reading_t reading;
    temp_callback     callback;
} oo_temp_reader_t;

int read_one_wire_temp(oo_temp_reader_t* reader, temp_callback callback);

void one_wire_init(oo_temp_reader_t* reader, app_timer_id_t timer_id, int dq_pin, uint32_t pwr_pin);

#endif //OW_TEMP_H