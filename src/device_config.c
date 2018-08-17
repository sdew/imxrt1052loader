#include "device_config.h"

__IO uint32_t g_uptime = 0;
static uint32_t fac_us;

void SysTick_Handler(void)
{
    g_uptime++;
}

void uptime_init(void)
{
    uint32_t prioritygroup = 0;
    uint32_t PreemptPriority = TICK_INT_PRIORITY;
    uint32_t SubPriority = 0;

    /*Configure the SysTick to have interrupt in 1ms time basis*/
    SysTick_Config(SystemCoreClock /1000);

    /*Configure the SysTick IRQ priority */
    prioritygroup = NVIC_GetPriorityGrouping();

    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
    fac_us = SystemCoreClock /1000000;
}

uint32_t uptime_get(void)
{
    return g_uptime;
}

void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    ticks = us *fac_us;
    //lock();
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told)
                tcnt += (told -tnow);
            else
                tcnt += (reload +told -tnow);
            told = tnow;
            if(tcnt >= ticks) break;
        }
    };
    //unlock();
}

void delay_ms(uint32_t ms)
{
    while (ms > 0) {
        delay_us(1000);
        ms--;
    }
}

/* MPU configuration. */
void sys_mpu_init(void)
{
    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {
        SCB_DisableICache();
    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {
        SCB_DisableDCache();
    }

    /* Disable MPU */
    ARM_MPU_Disable();

    /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable, SubRegionDisable, Size)
     * API in core_cm7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
     *      Use MACROS defined in core_cm7.h: ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribtue    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner noncache
     *     1             1           0           0              Normal             shareable       outer and inner noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write back write/read acllocate
     *     2             x           0           0              Device              not shareable
     *  Above are normal use settings, if your want to see more details or want to config different inner/outter cache policy.
     *  please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in core_cm7.h.
     */

    /* Region 0 setting: Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(0, 0xC0000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

    /* Region 1 setting: Memory with Device type, not shareable,  non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

/* Region 2 setting */
#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
    /* Setting Memory with Normal type, not shareable, outer/inner write back. */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x60000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_512MB);
#else
    /* Setting Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x60000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);
#endif

    /* Region 3 setting: Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(3, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

    /* Region 4 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(4, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_128KB);

    /* Region 5 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(5, 0x20000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_128KB);

    /* Region 6 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(6, 0x20200000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

    /* The define sets the cacheable memory to shareable,
     * this suggestion is referred from chapter 2.2.1 Memory regions,
     * types and attributes in Cortex-M7 Devices, Generic User Guide */
#if defined(SDRAM_IS_SHAREABLE)
    /* Region 7 setting: Memory with Normal type, shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 1, 1, 1, 0, ARM_MPU_REGION_SIZE_32MB);
#else
    /* Region 7 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32MB);
#endif

    /* Region 8 setting, set last 8MB of SDRAM can't be accessed by cache, glocal variables which are not expected to be
     * accessed by cache can be put here */
    /* Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(8, 0x81800000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_8MB);

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable I cache and D cache */
    SCB_EnableDCache();
    SCB_EnableICache();
}


#define AKXTAL_24M          24000000U
#define AKXTAL_32K          32768U
#define AKCORE_CLOCK        600000000U
#define AKCORE_DIV          ((AKCORE_CLOCK/AKXTAL_24M)*4)

const clock_arm_pll_config_t arm_pll_cfg = {
    .loopDivider = AKCORE_DIV, /* PLL loop divider, Fout = Fin * 50 */
};
const clock_sys_pll_config_t sys_pll_cfg = {
    .loopDivider = 1,   /* PLL loop divider, Fout = Fin * ( 20 + loopDivider*2 + numerator / denominator ) */
    .numerator = 0,     /* 30 bit numerator of fractional loop divider */
    .denominator = 1,   /* 30 bit denominator of fractional loop divider */
};
const clock_usb_pll_config_t usb1_pll_cfg = {
    .loopDivider = 0,   /* PLL loop divider, Fout = Fin * 20 */
};

void sys_clk_init(void)
{
    /* Init RTC OSC clock frequency. */
    CLOCK_SetRtcXtalFreq(AKXTAL_32K);
    /* Set XTAL 24MHz clock frequency. */
    CLOCK_SetXtalFreq(AKXTAL_24M);

    /* Setting PeriphClk2Mux and PeriphMux to provide stable clock before PLLs are initialed */
    CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 1); /* Set PERIPH_CLK2 MUX to OSC */
    CLOCK_SetMux(kCLOCK_PeriphMux, 1);     /* Set PERIPH_CLK MUX to PERIPH_CLK2 */

    /* Setting the VDD_SOC to 1.5V. It is necessary to config AHB to 600Mhz. */
    DCDC->REG3 = (DCDC->REG3 & (~DCDC_REG3_TRG_MASK)) | DCDC_REG3_TRG(0x12);
    /* Waiting for DCDC_STS_DC_OK bit is asserted */
    while (DCDC_REG0_STS_DC_OK_MASK != (DCDC_REG0_STS_DC_OK_MASK & DCDC->REG0));

    /* Init ARM PLL. */
    CLOCK_InitArmPll(&arm_pll_cfg);
    /* Init System PLL. */
#ifndef SKIP_SYSCLK_INIT
    CLOCK_InitSysPll(&sys_pll_cfg);
#endif
    /* Init Usb1 PLL. */
#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
    CLOCK_InitUsb1Pll(&usb1_pll_cfg);
#endif
    CLOCK_InitUsb1Pll(&usb1_pll_cfg);
    /* Enbale Audio PLL output. */
    CCM_ANALOG->PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_ENABLE_MASK;
    /* Enbale Video PLL output. */
    CCM_ANALOG->PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_ENABLE_MASK;
    /* Enable ENET PLL output. */
    CCM_ANALOG->PLL_ENET  |= CCM_ANALOG_PLL_ENET_ENABLE_MASK;
    CCM_ANALOG->PLL_ENET  |= CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_MASK;

    /* Set ARM_PODF. */
    CLOCK_SetDiv(kCLOCK_ArmDiv, 1);
    /* Set AHB_PODF. */
    CLOCK_SetDiv(kCLOCK_AhbDiv, 0);
    /* Set IPG_PODF. */
    CLOCK_SetDiv(kCLOCK_IpgDiv, 3);

    /* Set preperiph clock source. */
    CLOCK_SetMux(kCLOCK_PrePeriphMux, 3);
    /* Set periph clock source. */
    CLOCK_SetMux(kCLOCK_PeriphMux, 0);

    /* Set PERIPH_CLK2_PODF. */
    CLOCK_SetDiv(kCLOCK_PeriphClk2Div, 0);
    /* Set periph clock2 clock source. */
    CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 0);
    /* Set PERCLK_PODF. */
    CLOCK_SetDiv(kCLOCK_PerclkDiv, 1);
    /* Set per clock source. */
    CLOCK_SetMux(kCLOCK_PerclkMux, 0);

    /* Set USDHC1_PODF. */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 1);
    /* Set Usdhc1 clock source. */
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 0);
    /* Set USDHC2_PODF. */
    CLOCK_SetDiv(kCLOCK_Usdhc2Div, 1);
    /* Set Usdhc2 clock source. */
    CLOCK_SetMux(kCLOCK_Usdhc2Mux, 0);

#ifndef SKIP_SYSCLK_INIT
    /* Set SEMC_PODF. */
    CLOCK_SetDiv(kCLOCK_SemcDiv, 2);
    /* Set Semc alt clock source. */
    CLOCK_SetMux(kCLOCK_SemcAltMux, 0);
    /* Set Semc clock source. */
    CLOCK_SetMux(kCLOCK_SemcMux, 1);
#endif

    /* Set CSI_PODF. */
    CLOCK_SetDiv(kCLOCK_CsiDiv, 1);
    /* Set Csi clock source. */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);

#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
    /* Set FLEXSPI_PODF. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 1);
    /* Set Flexspi clock source. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0);
#endif

    /* Set LPSPI_PODF. */
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 4);
    /* Set Lpspi clock source. */
    CLOCK_SetMux(kCLOCK_LpspiMux, 2);

    /* Set TRACE_PODF. */
    CLOCK_SetDiv(kCLOCK_TraceDiv, 2);
    /* Set Trace clock source. */
    CLOCK_SetMux(kCLOCK_TraceMux, 2);

    /* Set SAI1_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, 3);
    /* Set SAI1_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai1Div, 1);
    /* Set Sai1 clock source. */
    CLOCK_SetMux(kCLOCK_Sai1Mux, 0);
    /* Set Sai2 clock source. */
    CLOCK_SetMux(kCLOCK_Sai2Mux, 0);
    /* Set SAI2_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai2Div, 1);
    /* Set SAI2_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai2PreDiv, 3);
    /* Set SAI3_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai3PreDiv, 3);
    /* Set SAI3_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai3Div, 1);
    /* Set Sai3 clock source. */
    CLOCK_SetMux(kCLOCK_Sai3Mux, 0);

    /* Set LPI2C_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 0);
    /* Set Lpi2c clock source. */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0);

    /* Set CAN_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_CanDiv, 1);
    /* Set Can clock source. */
    CLOCK_SetMux(kCLOCK_CanMux, 2);

    /* Set UART_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_UartDiv, 0);
    /* Set Uart clock source. */
    CLOCK_SetMux(kCLOCK_UartMux, 0);

    /* Set LCDIF_PRED. */
    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 1);
    /* Set LCDIF_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_LcdifDiv, 3);
    /* Set Lcdif pre clock source. */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 5);

    /* Set SPDIF0_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Spdif0PreDiv, 1);
    /* Set SPDIF0_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Spdif0Div, 7);
    /* Set Spdif clock source. */
    CLOCK_SetMux(kCLOCK_SpdifMux, 3);

    /* Set FLEXIO1_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, 1);
    /* Set FLEXIO1_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Flexio1Div, 7);
    /* Set Flexio1 clock source. */
    CLOCK_SetMux(kCLOCK_Flexio1Mux, 3);
    /* Set FLEXIO2_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Flexio2PreDiv, 1);
    /* Set FLEXIO2_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Flexio2Div, 7);
    /* Set Flexio2 clock source. */
    CLOCK_SetMux(kCLOCK_Flexio2Mux, 3);

    /* Set Pll3 sw clock source. */
    CLOCK_SetMux(kCLOCK_Pll3SwMux, 0);
    /* Set lvds1 clock source. */
    CCM_ANALOG->MISC1 = (CCM_ANALOG->MISC1 & (~CCM_ANALOG_MISC1_LVDS1_CLK_SEL_MASK)) | CCM_ANALOG_MISC1_LVDS1_CLK_SEL(0);

    /* Set SystemCoreClock variable. */
    SystemCoreClock = AKCORE_CLOCK;
}

void flexspi_pin_init(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_FLEXSPIB_DATA03, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_FLEXSPIB_DATA02, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_FLEXSPIB_DATA01, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_FLEXSPIB_DATA00, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_04_FLEXSPIB_SCLK,   1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_05_FLEXSPIA_DQS,    1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_06_FLEXSPIA_SS0_B,  1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_07_FLEXSPIA_SCLK,   1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_08_FLEXSPIA_DATA00, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_09_FLEXSPIA_DATA01, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_10_FLEXSPIA_DATA02, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_11_FLEXSPIA_DATA03, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_FLEXSPIB_DATA03, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_FLEXSPIB_DATA02, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_FLEXSPIB_DATA01, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_FLEXSPIB_DATA00, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_04_FLEXSPIB_SCLK,   0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_05_FLEXSPIA_DQS,    0x0130F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_06_FLEXSPIA_SS0_B,  0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_07_FLEXSPIA_SCLK,   0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_08_FLEXSPIA_DATA00, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_09_FLEXSPIA_DATA01, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_10_FLEXSPIA_DATA02, 0x10F1u);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_11_FLEXSPIA_DATA03, 0x10F1u);
}

#define AKNVIC_PRIORITYGROUP_0      ((uint32_t)0x00000007U)
#define AKNVIC_PRIORITYGROUP_1      ((uint32_t)0x00000006U)
#define AKNVIC_PRIORITYGROUP_2      ((uint32_t)0x00000005U)
#define AKNVIC_PRIORITYGROUP_3      ((uint32_t)0x00000004U)
#define AKNVIC_PRIORITYGROUP_4      ((uint32_t)0x00000003U)

void sys_nvic_init(void)
{
    NVIC_SetPriorityGrouping(AKNVIC_PRIORITYGROUP_4);
}

#define AKDEBUG_UART_TYPE       DEBUG_CONSOLE_DEVICE_TYPE_LPUART
#define AKDEBUG_UART_BASEADDR   (uint32_t)LPUART1
#define AKDEBUG_UART_INSTANCE   1U
#define AKDEBUG_UART_BAUDRATE   (115200U)

/* Get debug console frequency. */
static uint32_t sys_uart_freq(void)
{
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
       from application is use PLL3 source or OSC source */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) {/* PLL3 div6 80M */
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    } else {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
}

/* Initialize debug console. */
void sys_uart_init(void)
{
    lpuart_config_t lpuart1_config;
    uint32_t freq = sys_uart_freq();

    CLOCK_EnableClock(kCLOCK_Iomuxc);           /* iomuxc clock (iomuxc_clk_enable): 0x03u */

    IOMUXC_SetPinMux(
            IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 is configured as LPUART1_TX */
            0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
            IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 is configured as LPUART1_RX */
            0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
            IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 PAD functional properties : */
            0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                       Drive Strength Field: R0/6
                                                       Speed Field: medium(100MHz)
                                                       Open Drain Enable Field: Open Drain Disabled
                                                       Pull / Keep Enable Field: Pull/Keeper Enabled
                                                       Pull / Keep Select Field: Keeper
                                                       Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                       Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
            IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 PAD functional properties : */
            0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                       Drive Strength Field: R0/6
                                                       Speed Field: medium(100MHz)
                                                       Open Drain Enable Field: Open Drain Disabled
                                                       Pull / Keep Enable Field: Pull/Keeper Enabled
                                                       Pull / Keep Select Field: Keeper
                                                       Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                       Hyst. Enable Field: Hysteresis Disabled */


    CLOCK_EnableClock(kCLOCK_Lpuart1); //使能LPUART1时钟

    CLOCK_SetMux(kCLOCK_UartMux, 0); //设置UART时钟源为PLL3 80Mhz，PLL3/6=480/6=80MHz
    CLOCK_SetDiv(kCLOCK_UartDiv, 0); //设置UART时钟1分频，即UART时钟为80Mhz

    LPUART_GetDefaultConfig(&lpuart1_config); //先设置为默认配置，后面在根据实际情况配置

    //初始化NXP官方提供的debug console，此函数会重新初始化LPUART1，但是我们后面会
    //重新显示的初始化一次LPUART1，DbgConsole_Init()主要是给那些想要使用NXP官方
    //调试功能的开发者使用的，不需要使用的话就可以将下面代码注释掉
    //DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR,bound,BOARD_DEBUG_UART_TYPE,freq);

    lpuart1_config.baudRate_Bps  = 115200;                 //波特率
    lpuart1_config.dataBitsCount = kLPUART_EightDataBits;  //8位
    lpuart1_config.stopBitCount  = kLPUART_OneStopBit;     //1位停止位
    lpuart1_config.parityMode    = kLPUART_ParityDisabled; //无奇偶校验
    lpuart1_config.enableRx      = false;                  //使能接收
    lpuart1_config.enableTx      = true;                   //使能发送

    LPUART_Init(LPUART1, &lpuart1_config, freq);

    //LPUART中断设置
    //LPUART_EnableInterrupts(LPUART1,kLPUART_RxDataRegFullInterruptEnable); //使能接收中断
    //uint32_t group = NVIC_GetPriorityGrouping();
    //NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(group,5,0)); //抢占优先级5，子优先级0
    //EnableIRQ(LPUART1_IRQn); //使能LPUART1中断
}

#define LPUART_REC_LEN          200
uint8_t LPUART_RX_BUF[LPUART_REC_LEN];
uint16_t LPUART_RX_STA = 0;

void LPUART1_IRQHandler(void)
{
    uint8_t res = 0;

    if ((LPUART1->STAT) & kLPUART_RxDataRegFullFlag) { //接收中断
        res = LPUART1->DATA;                           //读取数据
        if ((LPUART_RX_STA & 0x8000) == 0) {           //接收未完成
            if (LPUART_RX_STA & 0x4000) {              //接收到了0x0d
                if (res != 0x0a)
                    LPUART_RX_STA = 0;                 //接收错误,重新开始
                else
                    LPUART_RX_STA |= 0x8000;           //接收完成了
            } else {                                   //还没收到0X0D
                if (res == 0x0d)
                    LPUART_RX_STA |= 0x4000;
                else {
                    LPUART_RX_BUF[LPUART_RX_STA&0X3FFF] = res;
                    LPUART_RX_STA++;
                    if (LPUART_RX_STA > (LPUART_REC_LEN-1))
                        LPUART_RX_STA = 0;             //接收数据错误,重新开始接收
                }
            }
        }
    }
    __DSB(); //数据同步屏蔽指令
}

void uart_send(uint8_t *dat)
{
    while (*dat != 0) {
        //while (!(LPUART1->STAT & (1<<23))) ;
        LPUART1->DATA = *(dat);
        while ((LPUART1->STAT&(1<<22))==0);
        dat++;
    }
}

//Override
#if _DLIB_FILE_DESCRIPTOR
#define UART_SEND_PROTOTYPE int fputc(int ch, FILE *f)
//#define UART_RECV_PROTOTYPE int fgetc(FILE *f)
#else
#define UART_SEND_PROTOTYPE int putchar(int ch)
//#define UART_RECV_PROTOTYPE int getchar()
#endif

UART_SEND_PROTOTYPE
{
    LPUART1->DATA = (uint8_t)ch;
    while ((LPUART1->STAT&(1<<22))==0);
    return ch;
}
/*
UART_RECV_PROTOTYPE
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(USART1);
}*/

