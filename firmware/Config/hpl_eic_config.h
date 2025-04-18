/* Auto-generated config file hpl_eic_config.h */
#ifndef HPL_EIC_CONFIG_H
#define HPL_EIC_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// <h> Basic Settings
// <o> Clock Selection
// <i> Indicates which clock used, The EIC can be clocked either by GCLK_EIC when higher frequency than 32KHz is required for filtering or
// <i> either by CLK_ULP32K when power consumption is the priority.
// <0x0=> Clocked by GCLK
// <0x1=> Clocked by ULPOSC32K
// <id> eic_arch_cksel
#ifndef CONF_EIC_CKSEL
#define CONF_EIC_CKSEL 1
#endif

// <o> Pin Sampler frequency selection
// <i> Indicates the sampling rate of the EXTINT pin.
// <0x0=> The sampling rate is EIC clock
// <0x1=> The sampling rate is the prescaled clock
// <id> eic_arch_tickon
#ifndef CONF_EIC_TICKON
#define CONF_EIC_TICKON 0
#endif

// </h>

// <e> Non-Maskable Interrupt Control
// <id> eic_arch_nmi_ctrl
#ifndef CONF_EIC_ENABLE_NMI_CTRL
#define CONF_EIC_ENABLE_NMI_CTRL 0
#endif

// <q> Non-Maskable Interrupt Filter Enable
// <i> Indicates whether the mon-maskable interrupt filter is enabled or not
// <id> eic_arch_nmifilten
#ifndef CONF_EIC_NMIFILTEN
#define CONF_EIC_NMIFILTEN 0
#endif

// <y> Non-Maskable Interrupt Sense
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines non-maskable interrupt sense
// <id> eic_arch_nmisense
#ifndef CONF_EIC_NMISENSE
#define CONF_EIC_NMISENSE EIC_NMICTRL_NMISENSE_NONE_Val
#endif

// <q> Asynchronous Edge Detection Mode
// <i> Indicates the interrupt detection mode operated synchronously or asynchronousl
// <id> eic_arch_nmiasynch
#ifndef CONF_EIC_NMIASYNCH
#define CONF_EIC_NMIASYNCH 0
#endif
// </e>

// <e> Interrupt 0 Settings
// <id> eic_arch_enable_irq_setting0
#ifndef CONF_EIC_ENABLE_IRQ_SETTING0
#define CONF_EIC_ENABLE_IRQ_SETTING0 0
#endif

// <q> External Interrupt 0 Filter Enable
// <i> Indicates whether the external interrupt 0 filter is enabled or not
// <id> eic_arch_filten0
#ifndef CONF_EIC_FILTEN0
#define CONF_EIC_FILTEN0 0
#endif

// <q> External Interrupt 0 Debounce Enable
// <i> Indicates whether the external interrupt 0 debounce is enabled or not
// <id> eic_arch_debounce_enable0
#ifndef CONF_EIC_DEBOUNCE_ENABLE0
#define CONF_EIC_DEBOUNCE_ENABLE0 0
#endif

// <q> External Interrupt 0 Event Output Enable
// <i> Indicates whether the external interrupt 0 event output is enabled or not
// <id> eic_arch_extinteo0
#ifndef CONF_EIC_EXTINTEO0
#define CONF_EIC_EXTINTEO0 0
#endif

// <y> Input 0 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense0
#ifndef CONF_EIC_SENSE0
#define CONF_EIC_SENSE0 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 0 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 0 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch0
#ifndef CONF_EIC_ASYNCH0
#define CONF_EIC_ASYNCH0 0
#endif

// </e>

// <e> Interrupt 1 Settings
// <id> eic_arch_enable_irq_setting1
#ifndef CONF_EIC_ENABLE_IRQ_SETTING1
#define CONF_EIC_ENABLE_IRQ_SETTING1 0
#endif

// <q> External Interrupt 1 Filter Enable
// <i> Indicates whether the external interrupt 1 filter is enabled or not
// <id> eic_arch_filten1
#ifndef CONF_EIC_FILTEN1
#define CONF_EIC_FILTEN1 0
#endif

// <q> External Interrupt 1 Debounce Enable
// <i> Indicates whether the external interrupt 1 debounce is enabled or not
// <id> eic_arch_debounce_enable1
#ifndef CONF_EIC_DEBOUNCE_ENABLE1
#define CONF_EIC_DEBOUNCE_ENABLE1 0
#endif

// <q> External Interrupt 1 Event Output Enable
// <i> Indicates whether the external interrupt 1 event output is enabled or not
// <id> eic_arch_extinteo1
#ifndef CONF_EIC_EXTINTEO1
#define CONF_EIC_EXTINTEO1 0
#endif

// <y> Input 1 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense1
#ifndef CONF_EIC_SENSE1
#define CONF_EIC_SENSE1 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 1 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 1 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch1
#ifndef CONF_EIC_ASYNCH1
#define CONF_EIC_ASYNCH1 0
#endif

// </e>

// <e> Interrupt 2 Settings
// <id> eic_arch_enable_irq_setting2
#ifndef CONF_EIC_ENABLE_IRQ_SETTING2
#define CONF_EIC_ENABLE_IRQ_SETTING2 1
#endif

// <q> External Interrupt 2 Filter Enable
// <i> Indicates whether the external interrupt 2 filter is enabled or not
// <id> eic_arch_filten2
#ifndef CONF_EIC_FILTEN2
#define CONF_EIC_FILTEN2 0
#endif

// <q> External Interrupt 2 Debounce Enable
// <i> Indicates whether the external interrupt 2 debounce is enabled or not
// <id> eic_arch_debounce_enable2
#ifndef CONF_EIC_DEBOUNCE_ENABLE2
#define CONF_EIC_DEBOUNCE_ENABLE2 0
#endif

// <q> External Interrupt 2 Event Output Enable
// <i> Indicates whether the external interrupt 2 event output is enabled or not
// <id> eic_arch_extinteo2
#ifndef CONF_EIC_EXTINTEO2
#define CONF_EIC_EXTINTEO2 0
#endif

// <y> Input 2 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense2
#ifndef CONF_EIC_SENSE2
#define CONF_EIC_SENSE2 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 2 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 2 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch2
#ifndef CONF_EIC_ASYNCH2
#define CONF_EIC_ASYNCH2 0
#endif

// </e>

// <e> Interrupt 3 Settings
// <id> eic_arch_enable_irq_setting3
#ifndef CONF_EIC_ENABLE_IRQ_SETTING3
#define CONF_EIC_ENABLE_IRQ_SETTING3 1
#endif

// <q> External Interrupt 3 Filter Enable
// <i> Indicates whether the external interrupt 3 filter is enabled or not
// <id> eic_arch_filten3
#ifndef CONF_EIC_FILTEN3
#define CONF_EIC_FILTEN3 0
#endif

// <q> External Interrupt 3 Debounce Enable
// <i> Indicates whether the external interrupt 3 debounce is enabled or not
// <id> eic_arch_debounce_enable3
#ifndef CONF_EIC_DEBOUNCE_ENABLE3
#define CONF_EIC_DEBOUNCE_ENABLE3 0
#endif

// <q> External Interrupt 3 Event Output Enable
// <i> Indicates whether the external interrupt 3 event output is enabled or not
// <id> eic_arch_extinteo3
#ifndef CONF_EIC_EXTINTEO3
#define CONF_EIC_EXTINTEO3 0
#endif

// <y> Input 3 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense3
#ifndef CONF_EIC_SENSE3
#define CONF_EIC_SENSE3 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 3 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 3 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch3
#ifndef CONF_EIC_ASYNCH3
#define CONF_EIC_ASYNCH3 0
#endif

// </e>

// <e> Interrupt 4 Settings
// <id> eic_arch_enable_irq_setting4
#ifndef CONF_EIC_ENABLE_IRQ_SETTING4
#define CONF_EIC_ENABLE_IRQ_SETTING4 1
#endif

// <q> External Interrupt 4 Filter Enable
// <i> Indicates whether the external interrupt 4 filter is enabled or not
// <id> eic_arch_filten4
#ifndef CONF_EIC_FILTEN4
#define CONF_EIC_FILTEN4 0
#endif

// <q> External Interrupt 4 Debounce Enable
// <i> Indicates whether the external interrupt 4 debounce is enabled or not
// <id> eic_arch_debounce_enable4
#ifndef CONF_EIC_DEBOUNCE_ENABLE4
#define CONF_EIC_DEBOUNCE_ENABLE4 0
#endif

// <q> External Interrupt 4 Event Output Enable
// <i> Indicates whether the external interrupt 4 event output is enabled or not
// <id> eic_arch_extinteo4
#ifndef CONF_EIC_EXTINTEO4
#define CONF_EIC_EXTINTEO4 0
#endif

// <y> Input 4 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense4
#ifndef CONF_EIC_SENSE4
#define CONF_EIC_SENSE4 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 4 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 4 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch4
#ifndef CONF_EIC_ASYNCH4
#define CONF_EIC_ASYNCH4 0
#endif

// </e>

// <e> Interrupt 5 Settings
// <id> eic_arch_enable_irq_setting5
#ifndef CONF_EIC_ENABLE_IRQ_SETTING5
#define CONF_EIC_ENABLE_IRQ_SETTING5 1
#endif

// <q> External Interrupt 5 Filter Enable
// <i> Indicates whether the external interrupt 5 filter is enabled or not
// <id> eic_arch_filten5
#ifndef CONF_EIC_FILTEN5
#define CONF_EIC_FILTEN5 0
#endif

// <q> External Interrupt 5 Debounce Enable
// <i> Indicates whether the external interrupt 5 debounce is enabled or not
// <id> eic_arch_debounce_enable5
#ifndef CONF_EIC_DEBOUNCE_ENABLE5
#define CONF_EIC_DEBOUNCE_ENABLE5 0
#endif

// <q> External Interrupt 5 Event Output Enable
// <i> Indicates whether the external interrupt 5 event output is enabled or not
// <id> eic_arch_extinteo5
#ifndef CONF_EIC_EXTINTEO5
#define CONF_EIC_EXTINTEO5 0
#endif

// <y> Input 5 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense5
#ifndef CONF_EIC_SENSE5
#define CONF_EIC_SENSE5 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 5 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 5 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch5
#ifndef CONF_EIC_ASYNCH5
#define CONF_EIC_ASYNCH5 0
#endif

// </e>

// <e> Interrupt 6 Settings
// <id> eic_arch_enable_irq_setting6
#ifndef CONF_EIC_ENABLE_IRQ_SETTING6
#define CONF_EIC_ENABLE_IRQ_SETTING6 0
#endif

// <q> External Interrupt 6 Filter Enable
// <i> Indicates whether the external interrupt 6 filter is enabled or not
// <id> eic_arch_filten6
#ifndef CONF_EIC_FILTEN6
#define CONF_EIC_FILTEN6 0
#endif

// <q> External Interrupt 6 Debounce Enable
// <i> Indicates whether the external interrupt 6 debounce is enabled or not
// <id> eic_arch_debounce_enable6
#ifndef CONF_EIC_DEBOUNCE_ENABLE6
#define CONF_EIC_DEBOUNCE_ENABLE6 0
#endif

// <q> External Interrupt 6 Event Output Enable
// <i> Indicates whether the external interrupt 6 event output is enabled or not
// <id> eic_arch_extinteo6
#ifndef CONF_EIC_EXTINTEO6
#define CONF_EIC_EXTINTEO6 0
#endif

// <y> Input 6 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense6
#ifndef CONF_EIC_SENSE6
#define CONF_EIC_SENSE6 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 6 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 6 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch6
#ifndef CONF_EIC_ASYNCH6
#define CONF_EIC_ASYNCH6 0
#endif

// </e>

// <e> Interrupt 7 Settings
// <id> eic_arch_enable_irq_setting7
#ifndef CONF_EIC_ENABLE_IRQ_SETTING7
#define CONF_EIC_ENABLE_IRQ_SETTING7 0
#endif

// <q> External Interrupt 7 Filter Enable
// <i> Indicates whether the external interrupt 7 filter is enabled or not
// <id> eic_arch_filten7
#ifndef CONF_EIC_FILTEN7
#define CONF_EIC_FILTEN7 0
#endif

// <q> External Interrupt 7 Debounce Enable
// <i> Indicates whether the external interrupt 7 debounce is enabled or not
// <id> eic_arch_debounce_enable7
#ifndef CONF_EIC_DEBOUNCE_ENABLE7
#define CONF_EIC_DEBOUNCE_ENABLE7 0
#endif

// <q> External Interrupt 7 Event Output Enable
// <i> Indicates whether the external interrupt 7 event output is enabled or not
// <id> eic_arch_extinteo7
#ifndef CONF_EIC_EXTINTEO7
#define CONF_EIC_EXTINTEO7 0
#endif

// <y> Input 7 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense7
#ifndef CONF_EIC_SENSE7
#define CONF_EIC_SENSE7 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 7 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 7 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch7
#ifndef CONF_EIC_ASYNCH7
#define CONF_EIC_ASYNCH7 0
#endif

// </e>

// <e> Interrupt 8 Settings
// <id> eic_arch_enable_irq_setting8
#ifndef CONF_EIC_ENABLE_IRQ_SETTING8
#define CONF_EIC_ENABLE_IRQ_SETTING8 0
#endif

// <q> External Interrupt 8 Filter Enable
// <i> Indicates whether the external interrupt 8 filter is enabled or not
// <id> eic_arch_filten8
#ifndef CONF_EIC_FILTEN8
#define CONF_EIC_FILTEN8 0
#endif

// <q> External Interrupt 8 Debounce Enable
// <i> Indicates whether the external interrupt 8 debounce is enabled or not
// <id> eic_arch_debounce_enable8
#ifndef CONF_EIC_DEBOUNCE_ENABLE8
#define CONF_EIC_DEBOUNCE_ENABLE8 0
#endif

// <q> External Interrupt 8 Event Output Enable
// <i> Indicates whether the external interrupt 8 event output is enabled or not
// <id> eic_arch_extinteo8
#ifndef CONF_EIC_EXTINTEO8
#define CONF_EIC_EXTINTEO8 0
#endif

// <y> Input 8 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense8
#ifndef CONF_EIC_SENSE8
#define CONF_EIC_SENSE8 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 8 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 8 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch8
#ifndef CONF_EIC_ASYNCH8
#define CONF_EIC_ASYNCH8 0
#endif

// </e>

// <e> Interrupt 9 Settings
// <id> eic_arch_enable_irq_setting9
#ifndef CONF_EIC_ENABLE_IRQ_SETTING9
#define CONF_EIC_ENABLE_IRQ_SETTING9 0
#endif

// <q> External Interrupt 9 Filter Enable
// <i> Indicates whether the external interrupt 9 filter is enabled or not
// <id> eic_arch_filten9
#ifndef CONF_EIC_FILTEN9
#define CONF_EIC_FILTEN9 0
#endif

// <q> External Interrupt 9 Debounce Enable
// <i> Indicates whether the external interrupt 9 debounce is enabled or not
// <id> eic_arch_debounce_enable9
#ifndef CONF_EIC_DEBOUNCE_ENABLE9
#define CONF_EIC_DEBOUNCE_ENABLE9 0
#endif

// <q> External Interrupt 9 Event Output Enable
// <i> Indicates whether the external interrupt 9 event output is enabled or not
// <id> eic_arch_extinteo9
#ifndef CONF_EIC_EXTINTEO9
#define CONF_EIC_EXTINTEO9 0
#endif

// <y> Input 9 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense9
#ifndef CONF_EIC_SENSE9
#define CONF_EIC_SENSE9 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 9 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 9 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch9
#ifndef CONF_EIC_ASYNCH9
#define CONF_EIC_ASYNCH9 0
#endif

// </e>

// <e> Interrupt 10 Settings
// <id> eic_arch_enable_irq_setting10
#ifndef CONF_EIC_ENABLE_IRQ_SETTING10
#define CONF_EIC_ENABLE_IRQ_SETTING10 1
#endif

// <q> External Interrupt 10 Filter Enable
// <i> Indicates whether the external interrupt 10 filter is enabled or not
// <id> eic_arch_filten10
#ifndef CONF_EIC_FILTEN10
#define CONF_EIC_FILTEN10 0
#endif

// <q> External Interrupt 10 Debounce Enable
// <i> Indicates whether the external interrupt 10 debounce is enabled or not
// <id> eic_arch_debounce_enable10
#ifndef CONF_EIC_DEBOUNCE_ENABLE10
#define CONF_EIC_DEBOUNCE_ENABLE10 0
#endif

// <q> External Interrupt 10 Event Output Enable
// <i> Indicates whether the external interrupt 10 event output is enabled or not
// <id> eic_arch_extinteo10
#ifndef CONF_EIC_EXTINTEO10
#define CONF_EIC_EXTINTEO10 0
#endif

// <y> Input 10 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense10
#ifndef CONF_EIC_SENSE10
#define CONF_EIC_SENSE10 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 10 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 10 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch10
#ifndef CONF_EIC_ASYNCH10
#define CONF_EIC_ASYNCH10 0
#endif

// </e>

// <e> Interrupt 11 Settings
// <id> eic_arch_enable_irq_setting11
#ifndef CONF_EIC_ENABLE_IRQ_SETTING11
#define CONF_EIC_ENABLE_IRQ_SETTING11 1
#endif

// <q> External Interrupt 11 Filter Enable
// <i> Indicates whether the external interrupt 11 filter is enabled or not
// <id> eic_arch_filten11
#ifndef CONF_EIC_FILTEN11
#define CONF_EIC_FILTEN11 0
#endif

// <q> External Interrupt 11 Debounce Enable
// <i> Indicates whether the external interrupt 11 debounce is enabled or not
// <id> eic_arch_debounce_enable11
#ifndef CONF_EIC_DEBOUNCE_ENABLE11
#define CONF_EIC_DEBOUNCE_ENABLE11 0
#endif

// <q> External Interrupt 11 Event Output Enable
// <i> Indicates whether the external interrupt 11 event output is enabled or not
// <id> eic_arch_extinteo11
#ifndef CONF_EIC_EXTINTEO11
#define CONF_EIC_EXTINTEO11 0
#endif

// <y> Input 11 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense11
#ifndef CONF_EIC_SENSE11
#define CONF_EIC_SENSE11 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 11 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 11 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch11
#ifndef CONF_EIC_ASYNCH11
#define CONF_EIC_ASYNCH11 0
#endif

// </e>

// <e> Interrupt 12 Settings
// <id> eic_arch_enable_irq_setting12
#ifndef CONF_EIC_ENABLE_IRQ_SETTING12
#define CONF_EIC_ENABLE_IRQ_SETTING12 1
#endif

// <q> External Interrupt 12 Filter Enable
// <i> Indicates whether the external interrupt 12 filter is enabled or not
// <id> eic_arch_filten12
#ifndef CONF_EIC_FILTEN12
#define CONF_EIC_FILTEN12 0
#endif

// <q> External Interrupt 12 Debounce Enable
// <i> Indicates whether the external interrupt 12 debounce is enabled or not
// <id> eic_arch_debounce_enable12
#ifndef CONF_EIC_DEBOUNCE_ENABLE12
#define CONF_EIC_DEBOUNCE_ENABLE12 0
#endif

// <q> External Interrupt 12 Event Output Enable
// <i> Indicates whether the external interrupt 12 event output is enabled or not
// <id> eic_arch_extinteo12
#ifndef CONF_EIC_EXTINTEO12
#define CONF_EIC_EXTINTEO12 0
#endif

// <y> Input 12 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense12
#ifndef CONF_EIC_SENSE12
#define CONF_EIC_SENSE12 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 12 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 12 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch12
#ifndef CONF_EIC_ASYNCH12
#define CONF_EIC_ASYNCH12 0
#endif

// </e>

// <e> Interrupt 13 Settings
// <id> eic_arch_enable_irq_setting13
#ifndef CONF_EIC_ENABLE_IRQ_SETTING13
#define CONF_EIC_ENABLE_IRQ_SETTING13 1
#endif

// <q> External Interrupt 13 Filter Enable
// <i> Indicates whether the external interrupt 13 filter is enabled or not
// <id> eic_arch_filten13
#ifndef CONF_EIC_FILTEN13
#define CONF_EIC_FILTEN13 0
#endif

// <q> External Interrupt 13 Debounce Enable
// <i> Indicates whether the external interrupt 13 debounce is enabled or not
// <id> eic_arch_debounce_enable13
#ifndef CONF_EIC_DEBOUNCE_ENABLE13
#define CONF_EIC_DEBOUNCE_ENABLE13 0
#endif

// <q> External Interrupt 13 Event Output Enable
// <i> Indicates whether the external interrupt 13 event output is enabled or not
// <id> eic_arch_extinteo13
#ifndef CONF_EIC_EXTINTEO13
#define CONF_EIC_EXTINTEO13 0
#endif

// <y> Input 13 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense13
#ifndef CONF_EIC_SENSE13
#define CONF_EIC_SENSE13 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 13 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 13 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch13
#ifndef CONF_EIC_ASYNCH13
#define CONF_EIC_ASYNCH13 0
#endif

// </e>

// <e> Interrupt 14 Settings
// <id> eic_arch_enable_irq_setting14
#ifndef CONF_EIC_ENABLE_IRQ_SETTING14
#define CONF_EIC_ENABLE_IRQ_SETTING14 0
#endif

// <q> External Interrupt 14 Filter Enable
// <i> Indicates whether the external interrupt 14 filter is enabled or not
// <id> eic_arch_filten14
#ifndef CONF_EIC_FILTEN14
#define CONF_EIC_FILTEN14 0
#endif

// <q> External Interrupt 14 Debounce Enable
// <i> Indicates whether the external interrupt 14 debounce is enabled or not
// <id> eic_arch_debounce_enable14
#ifndef CONF_EIC_DEBOUNCE_ENABLE14
#define CONF_EIC_DEBOUNCE_ENABLE14 0
#endif

// <q> External Interrupt 14 Event Output Enable
// <i> Indicates whether the external interrupt 14 event output is enabled or not
// <id> eic_arch_extinteo14
#ifndef CONF_EIC_EXTINTEO14
#define CONF_EIC_EXTINTEO14 0
#endif

// <y> Input 14 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense14
#ifndef CONF_EIC_SENSE14
#define CONF_EIC_SENSE14 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 14 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 14 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch14
#ifndef CONF_EIC_ASYNCH14
#define CONF_EIC_ASYNCH14 0
#endif

// </e>

// <e> Interrupt 15 Settings
// <id> eic_arch_enable_irq_setting15
#ifndef CONF_EIC_ENABLE_IRQ_SETTING15
#define CONF_EIC_ENABLE_IRQ_SETTING15 0
#endif

// <q> External Interrupt 15 Filter Enable
// <i> Indicates whether the external interrupt 15 filter is enabled or not
// <id> eic_arch_filten15
#ifndef CONF_EIC_FILTEN15
#define CONF_EIC_FILTEN15 0
#endif

// <q> External Interrupt 15 Debounce Enable
// <i> Indicates whether the external interrupt 15 debounce is enabled or not
// <id> eic_arch_debounce_enable15
#ifndef CONF_EIC_DEBOUNCE_ENABLE15
#define CONF_EIC_DEBOUNCE_ENABLE15 0
#endif

// <q> External Interrupt 15 Event Output Enable
// <i> Indicates whether the external interrupt 15 event output is enabled or not
// <id> eic_arch_extinteo15
#ifndef CONF_EIC_EXTINTEO15
#define CONF_EIC_EXTINTEO15 0
#endif

// <y> Input 15 Sense Configuration
// <EIC_NMICTRL_NMISENSE_NONE_Val"> No detection
// <EIC_NMICTRL_NMISENSE_RISE_Val"> Rising-edge detection
// <EIC_NMICTRL_NMISENSE_FALL_Val"> Falling-edge detection
// <EIC_NMICTRL_NMISENSE_BOTH_Val"> Both-edges detection
// <EIC_NMICTRL_NMISENSE_HIGH_Val"> High-level detection
// <EIC_NMICTRL_NMISENSE_LOW_Val"> Low-level detection
// <i> This defines input sense trigger
// <id> eic_arch_sense15
#ifndef CONF_EIC_SENSE15
#define CONF_EIC_SENSE15 EIC_NMICTRL_NMISENSE_BOTH_Val
#endif

// <q> External Interrupt 15 Asynchronous Edge Detection Mode
// <i> Indicates the external interrupt 15 detection mode operated synchronously or asynchronousl
// <id> eic_arch_asynch15
#ifndef CONF_EIC_ASYNCH15
#define CONF_EIC_ASYNCH15 0
#endif

// </e>

// <h> Debouncer 0 Settings
// <o> Debouncer Frequency Selection
// <0x0=>Divided by 2
// <0x1=>Divided by 4
// <0x2=>Divided by 8
// <0x3=>Divided by 16
// <0x4=>Divided by 32
// <0x5=>Divided by 64
// <0x6=>Divided by 128
// <0x7=>Divided by 256
// <i> Select the debouncer low frequency clock for pins

//   <i> EXTINT[7:0].

// <id> eic_arch_prescaler0
#ifndef CONF_EIC_DPRESCALER0
#define CONF_EIC_DPRESCALER0 EIC_DPRESCALER_PRESCALER0(0x0)
#endif

// <o> Low frequency samples
// <0x0=>3
// <0x1=>7
// <i> Indicates the number of samples by the debouncer low frequency clock needed to validate a transition from
// <i> current pin state to next pin state in synchronous debouncing mode.
// <id> eic_arch_states0
#ifndef CONF_EIC_STATES0
#define CONF_EIC_STATES0 0x0
#endif

// </h>

// <h> Debouncer 1 Settings
// <o> Debouncer Frequency Selection
// <0x0=>Divided by 2
// <0x1=>Divided by 4
// <0x2=>Divided by 8
// <0x3=>Divided by 16
// <0x4=>Divided by 32
// <0x5=>Divided by 64
// <0x6=>Divided by 128
// <0x7=>Divided by 256
// <i> Select the debouncer low frequency clock for pins

//   <i> EXTINT[15:8].

// <id> eic_arch_prescaler1
#ifndef CONF_EIC_DPRESCALER1
#define CONF_EIC_DPRESCALER1 EIC_DPRESCALER_PRESCALER1(0x0)
#endif

// <o> Low frequency samples
// <0x0=>3
// <0x1=>7
// <i> Indicates the number of samples by the debouncer low frequency clock needed to validate a transition from
// <i> current pin state to next pin state in synchronous debouncing mode.
// <id> eic_arch_states1
#ifndef CONF_EIC_STATES1
#define CONF_EIC_STATES1 0x0
#endif

// </h>

/*#define CONFIG_EIC_EXTINT_MAP {10, PIN_PB10}, {11, PIN_PB11}, {12, PIN_PB12}, {13, PIN_PB13},*/

#define CONFIG_EIC_EXTINT_MAP \
{0,  PIN_PB16}, \
{1,  PIN_PB17}, \
{2,  PIN_PA02}, \
{3,  PIN_PA03}, \
{4,  PIN_PA20}, \
{5,  PIN_PA21}, \
{6,  PIN_PB06}, \
{7,  PIN_PB07}, \
{8,  PIN_PB24}, \
{9,  PIN_PB25}, \
{10, PIN_PC10}, \
{11, PIN_PA27}, \
{12, PIN_PB12}, \
{13, PIN_PB27}, \
{14, PIN_PB14}, \
{15, PIN_PB15}, \


// <<< end of configuration section >>>

#endif // HPL_EIC_CONFIG_H


// #define M1ENC0 PB30
// {14, PIN_PB30}
// #define M1ENC1 PB31 
// {15, PIN_PB31}
// #define M2ENC0 PB16
// {00, PIN_PB16}
// #define M2ENC1 PB03
// {03, PIN_PB30}
// #define M3ENC0 PA14
// {14, PIN_PA14}
// #define M3ENC1 PA15
// {15, PIN_PA15}
// #define M4ENC0 PB14
// {14, PIN_PB14}
// #define M4ENC1 PB15
// {15, PIN_PB15}
// #define M5ENC0 PA06
// {06, PIN_PA06}
// #define M5ENC1 PA07
// {07, PIN_PA07}
// #define M6ENC0 PB06
// {06, PIN_PB06}
// #define M6ENC1 PB09
// {09, PIN_PB09}

