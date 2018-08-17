#ifndef __FLASH_CONFIG_H__
#define __FLASH_CONFIG_H__
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

// You should create a copy of this file in your flash loader project
// and configure it as described below

// when this macro is non-zero, your FlashInit function should accept
// extra 'argc' and 'argv' arguments as specified by the function
// prototype in 'flash_loader.h'
#define USE_ARGC_ARGV               1

// You can customize the memory reserved for passing arguments to FlashInit
// through argc and argv.
#if USE_ARGC_ARGV
// This specifies the maximum allowed number of arguments in argv
#define MAX_ARGS                    5
// This specifies the maximum combined size of the arguments, including
// a trailing null for each argument
#define MAX_ARG_SIZE                64
#endif

// If this is true (non-zero), the parameter designating the code destination
// in flash operations will be a 'void *', otherwise it will be a uint32_t.
// Targets where void * is smaller than a code pointer should set this to 0.
#define CODE_ADDR_AS_VOID_PTR       1

#include <stdio.h>
#include <stdint.h>
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_flexspi.h"

#define SDR_CMD                         kFLEXSPI_Command_SDR
#define DDR_CMD                         kFLEXSPI_Command_DDR
#define SDR_RADDR                       kFLEXSPI_Command_RADDR_SDR
#define DDR_RADDR                       kFLEXSPI_Command_RADDR_DDR
#define SDR_CADDR                       kFLEXSPI_Command_CADDR_SDR
#define DDR_CADDR                       kFLEXSPI_Command_CADDR_DDR
#define SDR_MODE1                       kFLEXSPI_Command_MODE1_SDR
#define DDR_MODE1                       kFLEXSPI_Command_MODE1_DDR
#define SDR_MODE2                       kFLEXSPI_Command_MODE2_SDR
#define DDR_MODE2                       kFLEXSPI_Command_MODE2_DDR
#define SDR_MODE4                       kFLEXSPI_Command_MODE4_SDR
#define DDR_MODE4                       kFLEXSPI_Command_MODE4_DDR
#define SDR_MODE8                       kFLEXSPI_Command_MODE8_SDR
#define DDR_MODE8                       kFLEXSPI_Command_MODE8_DDR
#define SDR_WRITE                       kFLEXSPI_Command_WRITE_SDR
#define DDR_WRITE                       kFLEXSPI_Command_WRITE_DDR
#define SDR_READ                        kFLEXSPI_Command_READ_SDR
#define DDR_READ                        kFLEXSPI_Command_READ_DDR
#define SDR_LEARN                       kFLEXSPI_Command_LEARN_SDR
#define DDR_LEARN                       kFLEXSPI_Command_LEARN_DDR
#define SDR_DATSZ                       kFLEXSPI_Command_DATSZ_SDR
#define DDR_DATSZ                       kFLEXSPI_Command_DATSZ_DDR
#define SDR_DUMMY                       kFLEXSPI_Command_DUMMY_SDR
#define DDR_DUMMY                       kFLEXSPI_Command_DUMMY_DDR
#define SDR_DUMMY_RWDS                  kFLEXSPI_Command_DUMMY_RWDS_SDR
#define DDR_DUMMY_RWDS                  kFLEXSPI_Command_DUMMY_RWDS_DDR
#define JMP_ON_CS                       kFLEXSPI_Command_JUMP_ON_CS
#define STOP                            kFLEXSPI_Command_STOP

#define USE_1PAD                        kFLEXSPI_1PAD
#define USE_2PAD                        kFLEXSPI_2PAD
#define USE_4PAD                        kFLEXSPI_4PAD
#define USE_8PAD                        kFLEXSPI_8PAD


typedef struct {
#if USE_ARGC_ARGV
    uint32_t (*init)(void *base_of_flash, int argc, char const *argv[]);
#else
    uint32_t (*init)(void *base_of_flash);
#endif /* USE_ARGC_ARGV */
    uint32_t (*erase)(void *block_start);
    uint32_t (*program)(uint32_t addr, uint32_t count, char const *buffer);
    uint32_t (*eraseall)(void);
    uint32_t (*signoff)(void);
    uint32_t base;
} device_t;

extern const device_t QuadFlash;
extern const device_t AdestoFlash;

#endif
