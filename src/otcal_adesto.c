#include "device_config.h"

#define ECOXIP_RESET_GPIO               GPIO3
#define ECOXIP_RESET_PIN                15

#define ECOXIP_MFID                     0x43
#define ECOXIP_DEID                     0xa7

#define ECOXIP_PAGE_SIZE                (256)
#define ECOXIP_SECTOR_SIZE              (ECOXIP_PAGE_SIZE*16) // that's actually a block not a sector, different define with adesto
#define ECOXIP_BLOCK_SIZE               (ECOXIP_SECTOR_SIZE*64) // that's actually a sector not a block
#define ECOXIP_SIZE                     (ECOXIP_BLOCK_SIZE*16)
#define ECOXIP_BASE                     FlexSPI_AMBA_BASE
#define ECOXIP_FLEXSPI_CLOCK            kCLOCK_FlexSpi

// Number of dummy cycles for non-SPI (quad/octal) modes
#define ECO_DUMMYS                      14
#define ECO_CTRL_REG                    (((ECO_DUMMYS - 8) >> 1) | 0x10)

#define ECO_PROTECT                     0x7F
#define ECO_UNPROTECT                   0

#define ECO_LUT_LENGTH                  64
#define ECO_LUT_READDATA                0
#define ECO_LUT_READSTATUS              1
#define ECO_LUT_WRITESTATUS             2
#define ECO_LUT_WRITEENABLE             3
#define ECO_LUT_ERASEBLOCK              5
#define ECO_LUT_WRITEREGS               6
#define ECO_LUT_READID                  7
#define ECO_LUT_READREGS                8
#define ECO_LUT_PAGEPROGRAM             9
#define ECO_LUT_CHIPERASE               11

// EcoXiP command op codes
#define ECOCMD_WREG1                    0x01 // Write Status Register Byte 1
#define ECOCMD_PDATA                    0x02 // Program Data
#define ECOCMD_RREG1                    0x05 // Read Status Register Byte 1
#define ECOCMD_WABLE                    0x06 // Write Enable
#define ECOCMD_RARRY                    0x0B // Read Array
#define ECOCMD_ERA4K                    0x20 // 4Kbytes Erase
#define ECOCMD_UNPTT                    0x39 // Unprotect Sector
#define ECOCMD_RREGS                    0x65 // Read Status/Control Registers
#define ECOCMD_WREGS                    0x71 // Write Status/Control Registers
#define ECOCMD_ERALL                    0xc7 // Chip Erase
#define ECOCMD_RMDID                    0x9f // Read Manufacturer and Device ID
#define ECOCMD_SPIMD                    0xff // Reset to Standard SPI Mode

static uint32_t flash_base = 0x60000000;

static flexspi_device_config_t dcfg = {
    .flexspiRootClk = 60000000, /* 60MHZ SPI serial clock */
    .isSck2Enabled = false,
    .flashSize = ECOXIP_SIZE/1024,
    .CSIntervalUnit = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval = 2,
    .CSHoldTime = 3,
    .CSSetupTime = 3,
    .dataValidTime = 0,
    .columnspace = 0,
    .enableWordAddress = 0,
    .AWRSeqIndex = 0,
    .AWRSeqNumber = 0,
    .ARDSeqIndex = ECO_LUT_READDATA,
    .ARDSeqNumber = 1,
    .AHBWriteWaitUnit = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

static const uint32_t xlut[ECO_LUT_LENGTH] = {
    [4*ECO_LUT_WRITEENABLE]    = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_WABLE, STOP, 0, 0),
    // Read Status Register Byte 1
    [4*ECO_LUT_READSTATUS]     = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_RREG1, SDR_READ,  USE_1PAD, 0x01),
    [4*ECO_LUT_READSTATUS +1]  = FLEXSPI_LUT_SEQ(STOP, 0, 0, 0, 0, 0),
    // Write Status Register Byte 1
    [4*ECO_LUT_WRITESTATUS]    = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_WREG1, SDR_WRITE, USE_1PAD, 0x01),
    [4*ECO_LUT_WRITESTATUS +1] = FLEXSPI_LUT_SEQ(STOP, 0, 0, 0, 0, 0),
    // Write Status/Control Register
    [4*ECO_LUT_WRITEREGS]      = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_WREGS, SDR_RADDR, USE_1PAD, 0x08),
    [4*ECO_LUT_WRITEREGS +1]   = FLEXSPI_LUT_SEQ(SDR_WRITE, USE_1PAD, 0x01, STOP, 0, 0),
    // Read Manufactor ID
    [4*ECO_LUT_READID]         = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_RMDID, SDR_READ,  USE_1PAD, 0x01),
    [4*ECO_LUT_READID +1]      = FLEXSPI_LUT_SEQ(STOP, 0, 0, 0, 0, 0),
    // Read
    [4*ECO_LUT_READDATA]       = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_RARRY, SDR_RADDR, USE_1PAD, 0x20),
    [4*ECO_LUT_READDATA +1]    = FLEXSPI_LUT_SEQ(SDR_DUMMY, USE_1PAD, 0x08,         SDR_READ,  USE_1PAD, 0x08),
    [4*ECO_LUT_READDATA +2]    = FLEXSPI_LUT_SEQ(STOP, 0, 0, 0, 0, 0),
    // Erase Block 4KBytes
    [4*ECO_LUT_ERASEBLOCK]     = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_ERA4K, SDR_RADDR, USE_1PAD, 0x20),
    [4*ECO_LUT_ERASEBLOCK +1]  = FLEXSPI_LUT_SEQ(STOP, 0, 0, 0, 0, 0),
    // Page Program
    [4*ECO_LUT_PAGEPROGRAM]    = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_PDATA, SDR_RADDR, USE_1PAD, 0x20),
    [4*ECO_LUT_PAGEPROGRAM +1] = FLEXSPI_LUT_SEQ(SDR_WRITE, USE_1PAD, 0x08, STOP, 0, 0),
    // Chip Erase
    [4*ECO_LUT_CHIPERASE]      = FLEXSPI_LUT_SEQ(SDR_CMD,   USE_1PAD, ECOCMD_ERALL, STOP, 0, 0),
};


// Perform JEDEC reset
static void _jedec_reset(void)
{
    uint32_t i;
    gpio_pin_config_t jreset_pin_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};

    CLOCK_EnableClock(kCLOCK_Iomuxc);
    // Configure the 3 pins used in JEDEC reset as GPIOs
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_06_GPIO3_IO06, 1);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_08_GPIO3_IO08, 1);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_07_GPIO3_IO07, 1);

    // Set the direction of 3 pins used in JEDEC reset to output
    GPIO_PinInit(GPIO3, 6, &jreset_pin_config); // CS
    GPIO_PinInit(GPIO3, 8, &jreset_pin_config); // SI/IO0
    GPIO_PinInit(GPIO3, 7, &jreset_pin_config); // SCK

    // Perform a reset sequence:
    // CS goes low 4 times with alternating values of SOUT
    // SCK is drive low or high and must stay in one state
    GPIO_WritePinOutput(GPIO3, 7, 0); // set SCK low
    for (i = 0; i < 4; i++) {
        // drive CS low
        GPIO_WritePinOutput(GPIO3, 6, 0);
        delay_us(100);
        // drive SI low or high: alternate its state every iteration
        GPIO_WritePinOutput(GPIO3, 8, (i&1));
        delay_us(100);
        // drive CS high; SI state will be captured on the CS rising edge
        GPIO_WritePinOutput(GPIO3, 6, 1);
        delay_us(100);
    }
}

static status_t _wait_busy(void)
{
    status_t status;
    flexspi_transfer_t xfer;
    uint32_t rdat;

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Read;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_READSTATUS;
    xfer.data = &rdat;
    xfer.dataSize = 1;

    do {
        status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
        if (status != kStatus_Success) {
            return status;
        }
    } while (rdat & 0x01);

    return status;
}

static status_t _get_id(uint32_t *mid)
{
    status_t status;
    flexspi_transfer_t xfer;
    uint8_t tmp[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Read;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_READID;
    xfer.data = (uint32_t*)tmp;
    xfer.dataSize = 12;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    if (status != kStatus_Success) {
        return status;
    }

    *mid = (tmp[7]<<8 | tmp[8]);

    return status;
}

static status_t _write_en(void)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Command;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_WRITEENABLE;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _read_status(uint32_t *datout)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Read;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_WRITESTATUS;
    xfer.data = datout;
    xfer.dataSize = 1;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _write_status(uint32_t datin)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Write;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_WRITESTATUS;
    xfer.data = &datin;
    xfer.dataSize = 1;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _write_regs(uint32_t offset, uint32_t datin)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = offset;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Write;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_WRITEREGS;
    xfer.data = &datin;
    xfer.dataSize = 1;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _protect(uint32_t cmd)
{
    status_t status;

    status = _write_en();
    if (status != kStatus_Success) {
        return status;
    }

    status = _write_status(cmd);
    if (status != kStatus_Success) {
        return status;
    }

    status = _wait_busy();
    return status;
}

static status_t _erase4k(uint32_t addr)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = addr;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Command;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_ERASEBLOCK;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _program(uint32_t addr, uint32_t prog_size, uint32_t *dat)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = addr;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Write;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_PAGEPROGRAM;
    xfer.data = dat;
    xfer.dataSize = prog_size;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

static status_t _erase_chip(void)
{
    status_t status;
    flexspi_transfer_t xfer;

    xfer.deviceAddress = 0;
    xfer.port = kFLEXSPI_PortA1;
    xfer.cmdType = kFLEXSPI_Command;
    xfer.SeqNumber = 1;
    xfer.seqIndex = ECO_LUT_CHIPERASE;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &xfer);
    return status;
}

#if USE_ARGC_ARGV
static uint32_t init(void *base_of_flash, int argc, char const *argv[])
#else
static uint32_t init(void *base_of_flash)
#endif /* USE_ARGC_ARGV */
{
    status_t status;
    flexspi_config_t xcfg;
    uint32_t mdid;
    uint32_t chip_size;

    flash_base = (uint32_t)base_of_flash;

    _jedec_reset();
    delay_ms(10);
    flexspi_pin_init();

    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 18);   /* Set PLL3 PFD0 clock 480MHZ. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0x3); /* Choose PLL3 PFD0 clock as flexspi source clock. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 7);   /* flexspi clock 60M */

    /*Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&xcfg);

    /*Set AHB buffer size for reading data through AHB bus. */
    xcfg.ahbConfig.enableAHBPrefetch = true;
    /*Allow AHB read start address do not follow the alignment requirement. */
    //xcfg.ahbConfig.enableReadAddressOpt = true;
    /* enable diff clock and DQS */
    //xcfg.enableSckBDiffOpt = true;
    xcfg.rxSampleClock = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    xcfg.enableCombination = true;
    FLEXSPI_Init(FLEXSPI, &xcfg);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(FLEXSPI, &dcfg, kFLEXSPI_PortA1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(FLEXSPI, 0, xlut, ECO_LUT_LENGTH);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(FLEXSPI);

    status = _get_id(&mdid);
    if (status != kStatus_Success) {return RESULT_ERROR;}

    if (ECOXIP_MFID != ((mdid>>8) & 0xFF)) {return RESULT_ERROR;}

    if (ECOXIP_DEID != (mdid&0xFF)) {return RESULT_ERROR;}

    status = _protect(ECO_UNPROTECT);
    if (status != kStatus_Success) {return RESULT_ERROR;}

    chip_size = 4*1024*1024;
    sprintf(LAYOUT_OVERRIDE_BUFFER, "%d 0x%X\0", (chip_size)/ECOXIP_SECTOR_SIZE, ECOXIP_SECTOR_SIZE);

    return RESULT_OK | OVERRIDE_LAYOUT;
}

static uint32_t erase(void *block_start)
{
    status_t status;
    uint32_t flash_offset;

    flash_offset = (uint32_t)block_start - (uint32_t)flash_base;
    status = _write_en();
    if (status != kStatus_Success) {return RESULT_ERROR;}
    status = _erase4k(flash_offset);
    if (status != kStatus_Success) {return RESULT_ERROR;}
    status = _wait_busy();
    if (status != kStatus_Success) {return RESULT_ERROR;}
    return RESULT_OK;
}

static uint32_t program(uint32_t addr, uint32_t cnt, char const *dat)
{
    status_t status;
    uint32_t flash_offset, prog_size;

    flash_offset = addr - flash_base;

    while (cnt > 0) {
        prog_size = (cnt > ECOXIP_PAGE_SIZE ? ECOXIP_PAGE_SIZE : cnt);
        status = _write_en();
        if (status != kStatus_Success) {return RESULT_ERROR;}
        status = _program(flash_offset, prog_size, (uint32_t *)dat);
        if (status != kStatus_Success) {return RESULT_ERROR;}
        status = _wait_busy();
        if (status != kStatus_Success) {return RESULT_ERROR;}
        cnt -= prog_size;
        flash_offset += prog_size;
        dat += prog_size;
    }

    return RESULT_OK;
}

static uint32_t eraseall(void)
{
    status_t status;

    status = _write_en();
    if (status != kStatus_Success) {return RESULT_ERROR;}
    status = _erase_chip();
    if (status != kStatus_Success) {return RESULT_ERROR;}
    status = _wait_busy();
    if (status != kStatus_Success) {return RESULT_ERROR;}
    return RESULT_OK;
}

static uint32_t signoff(void)
{
    return RESULT_OK;
}

const device_t AdestoFlash = {
    .init = init,
    .erase = erase,
    .program = program,
    .eraseall = eraseall,
    .signoff = signoff
};

