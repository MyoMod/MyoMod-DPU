/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v13.0
processor: MIMXRT1062xxxxB
package_id: MIMXRT1062DVL6B
mcu_data: ksdk2_0
processor_version: 14.0.1
board: MIMXRT1060-EVKC
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 5d593d64-b05a-48b6-982b-7f0904718eaf
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
      - 2: []
      - 3: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * GPIO2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO2'
- type: 'igpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'igpio_b1c1fa279aa7069dca167502b8589cb7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIO2'
- config_sets:
  - fsl_gpio:
    - enable_irq_comb_0_15: 'true'
    - gpio_interrupt_comb_0_15:
      - IRQn: 'GPIO2_Combined_0_15_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_comb_16_31: 'true'
    - gpio_interrupt_comb_16_31:
      - IRQn: 'GPIO2_Combined_16_31_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIO2_init(void) {
  /* Make sure, the clock gate for GPIO2 is enabled (e. g. in pin_mux.c) */
  /* Enable interrupt GPIO2_Combined_0_15_IRQn request in the NVIC. */
  EnableIRQ(GPIO2_GPIO_COMB_0_15_IRQN);
  /* Enable interrupt GPIO2_Combined_16_31_IRQn request in the NVIC. */
  EnableIRQ(GPIO2_GPIO_COMB_16_31_IRQN);
}

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'pit_ab54f91356454adb874dafbb69e655fd'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'false'
    - enableSharedInterrupt: 'true'
    - sharedInterrupt:
      - IRQn: 'PIT_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'ClocksTool_DefaultInit'
    - channels:
      - 0:
        - channel_id: 'CHANNEL_0'
        - channelNumber: '0'
        - enableChain: 'false'
        - timerPeriod: '10Hz'
        - startTimer: 'true'
        - enableInterrupt: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = false
};

static void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
  /* Set channel 0 period to 100 ms (7500000 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, PIT_CHANNEL_0_TICKS);
  /* Enable interrupts from channel 0. */
  PIT_EnableInterrupts(PIT_PERIPHERAL, PIT_CHANNEL_0, kPIT_TimerInterruptEnable);
  /* Enable interrupt PIT_IRQN request in the NVIC */
  EnableIRQ(PIT_IRQN);
  /* Start channel 0. */
  PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
}

/***********************************************************************************************************************
 * TMR1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'TMR1'
- type: 'qtmr'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'qtmr_460dd7aa3f3371843c2548acd54252b0'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'TMR1'
- config_sets:
  - fsl_qtmr:
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'ClocksTool_DefaultInit'
    - qtmr_channels:
      - 0:
        - channel_prefix_id: 'TimePresacler'
        - channel: 'kQTMR_Channel_0'
        - primarySource: 'kQTMR_ClockDivide_8'
        - secondarySource: 'kQTMR_Counter0InputPin'
        - countingMode: 'kQTMR_PriSrcRiseEdge'
        - enableMasterMode: 'true'
        - enableExternalForce: 'false'
        - faultFilterCount: '3'
        - faultFilterPeriod: '0'
        - debugMode: 'kQTMR_HaltCounter'
        - timerModeInit: 'timer'
        - timerMode:
          - freq_value_str: '1 ms'
        - dmaIntMode: 'polling'
      - 1:
        - channel_prefix_id: 'ms_Counter'
        - channel: 'kQTMR_Channel_1'
        - primarySource: 'kQTMR_ClockCounter0Output'
        - primarySourceFreq: '1 khz'
        - secondarySource: 'kQTMR_Counter0InputPin'
        - countingMode: 'kQTMR_CascadeCount'
        - enableMasterMode: 'true'
        - enableExternalForce: 'false'
        - faultFilterCount: '3'
        - faultFilterPeriod: '0'
        - debugMode: 'kQTMR_HaltCounter'
        - timerModeInit: 'timer'
        - timerMode:
          - freq_value_str: '65535'
        - dmaIntMode: 'polling'
    - interruptVector:
      - enable_irq: 'false'
      - interrupt:
        - IRQn: 'TMR1_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const qtmr_config_t TMR1_TimePresacler_config = {
  .primarySource = kQTMR_ClockDivide_8,
  .secondarySource = kQTMR_Counter0InputPin,
  .enableMasterMode = true,
  .enableExternalForce = false,
  .faultFilterCount = 0,
  .faultFilterPeriod = 0,
  .debugMode = kQTMR_HaltCounter
};
const qtmr_config_t TMR1_ms_Counter_config = {
  .primarySource = kQTMR_ClockCounter0Output,
  .secondarySource = kQTMR_Counter0InputPin,
  .enableMasterMode = true,
  .enableExternalForce = false,
  .faultFilterCount = 0,
  .faultFilterPeriod = 0,
  .debugMode = kQTMR_HaltCounter
};

static void TMR1_init(void) {
  /* Quad timer channel TimePresacler peripheral initialization */
  QTMR_Init(TMR1_PERIPHERAL, TMR1_TIMEPRESACLER_CHANNEL, &TMR1_TimePresacler_config);
  /* Setup the timer period of the channel */
  QTMR_SetTimerPeriod(TMR1_PERIPHERAL, TMR1_TIMEPRESACLER_CHANNEL, 18750U);
  /* Quad timer channel ms_Counter peripheral initialization */
  QTMR_Init(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL, &TMR1_ms_Counter_config);
  /* Setup the timer period of the channel */
  QTMR_SetTimerPeriod(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL, 65535U);
  /* Start the timer - select the timer counting mode */
  QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_TIMEPRESACLER_CHANNEL, kQTMR_PriSrcRiseEdge);
  /* Start the timer - select the timer counting mode */
  QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL, kQTMR_CascadeCount);
}

/***********************************************************************************************************************
 * GPIO4 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO4'
- type: 'igpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'igpio_b1c1fa279aa7069dca167502b8589cb7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIO4'
- config_sets:
  - fsl_gpio:
    - enable_irq_comb_0_15: 'true'
    - gpio_interrupt_comb_0_15:
      - IRQn: 'GPIO4_Combined_0_15_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_comb_16_31: 'false'
    - gpio_interrupt_comb_16_31:
      - IRQn: 'GPIO1_Combined_16_31_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIO4_init(void) {
  /* Make sure, the clock gate for GPIO4 is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector GPIO4_Combined_0_15_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(GPIO4_GPIO_COMB_0_15_IRQN, GPIO4_GPIO_COMB_0_15_IRQ_PRIORITY);
  /* Enable interrupt GPIO4_Combined_0_15_IRQn request in the NVIC. */
  EnableIRQ(GPIO4_GPIO_COMB_0_15_IRQN);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPIO2_init();
  PIT_init();
  TMR1_init();
  GPIO4_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
