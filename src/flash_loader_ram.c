//------------------------------------------------------------------------------
//
// Copyright (c) 2008-2015 IAR Systems
//
// Licensed under the Apache License, Version 2.0 (the "License")
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// $Revision: 38420 $
//
//------------------------------------------------------------------------------

#include "flash_loader.h"
#include "flash_loader_extra.h"
#include "device_config.h"

void strcopy(char *to, char *from);

static const device_t *device;

#if USE_ARGC_ARGV
const char* FlFindOption(char* option, int with_value, int argc, char const* argv[])
{
    int i;

    for (i = 0; i < argc; i++) {
        if (strcmp(option, argv[i]) == 0) {
            if (with_value) {
                if (i + 1 < argc)
                    return argv[i + 1]; // The next argument is the value.
                else
                    return 0; // The option was found but there is no value to return.
            } else {
                return argv[i]; // Return the flag argument itself just to get a non-zero pointer.
            }
        }
    }
    return 0;
}
#endif /*USE_ARGC_ARGV*/

#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size, uint32_t link_address, uint32_t flags, int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size, uint32_t link_address, uint32_t flags)
#endif /* USE_ARGC_ARGV */
{
    uint32_t result;
    //uint8_t ubuf[255];

    sys_nvic_init();
    sys_mpu_init();
    sys_clk_init();
    uptime_init();
    //sys_uart_init();

    if (2 > ((SRC->SBMR1>>8) & 0x7)) {
        device = &QuadFlash;
    }

#if USE_ARGC_ARGV
    if (FlFindOption("--eco", 0, argc, argv)) {
        device = &AdestoFlash;
    }
#endif

#if USE_ARGC_ARGV
    result = device->init(base_of_flash, argc, argv);
#else
    result = device->init(base_of_flash);
#endif /* USE_ARGC_ARGV */

    if (RESULT_ERROR != result) {
        if (FLAG_ERASE_ONLY & flags) {
            if (device->eraseall) {
                result = device->eraseall();
                if (RESULT_OK == result) {
                    return RESULT_ERASE_DONE;
                }
            }
        }
    }

    return result;
}

uint32_t FlashWrite(void *block_start, uint32_t offset_into_block, uint32_t count, char const *buffer)
{
    return device->program((uint32_t)block_start+offset_into_block, count, buffer);
}

uint32_t FlashErase(void *block_start, uint32_t block_size)
{
    return device->erase(block_start);
}

OPTIONAL_CHECKSUM
uint32_t FlashChecksum(void const *begin, uint32_t count)
{
    return Crc16((uint8_t const *)begin, count);
}

OPTIONAL_SIGNOFF
uint32_t FlashSignoff(void)
{
    uint32_t result = RESULT_OK;

    if (device->signoff) {
        result = device->signoff();
    }

    return result;
}

void strcopy(char *to, char* from)
{
    while (*to++ = *from++) ;
}

