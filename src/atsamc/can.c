// Serial over CAN emulation for STM32 boards.
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_SAMC21
#include "command.h" // DECL_CONSTANT_STR
#include "fasthash.h" // fasthash64
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "generic/canbus.h" // canbus_notify_tx, canbus_notify_rx
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT
#include "mcan.h"
#include "mcan_helper.h"

#if CONFIG_CANSERIAL
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PA24,PA25");
 #define GPIO_Rx GPIO('A', 24)
 #define GPIO_Tx GPIO('A', 25)
 #define SOC_CAN CAN0
 #define CAN_FUNCTION  6  // Alternative function mapping number
#endif

#define UID_BASE 0x0080A00C

#define CAN_MESSAGE_SIZE 8

/* size of our custom Rx and Tx Buffer Elements, in 32 bit words */
#define RAM_BUF_SIZE                  (MCAN_RAM_BUF_HDR_SIZE + CAN_MESSAGE_SIZE / 4)

#define RAM_ARRAY_SIZE_FILT_STD       (4u) // 4 standard Filters
#define RAM_ARRAY_SIZE_FILT_EXT       (0u) // No extended filters
#define RAM_FIFO_SIZE_RX0             (12u) // 12 Slots in the RX FIFO
/* no Rx FIFO 1 in our Message RAM */
#define RAM_ARRAY_SIZE_RX             (0u) // Using the FIFO.
/* no Tx Event FIFO in our Message RAM */
#define RAM_ARRAY_SIZE_TX             (0u) // Using the FIFO.
#define RAM_FIFO_SIZE_TX              (4u)

#define MSG_RAM_SIZE      ( \
      RAM_ARRAY_SIZE_FILT_STD * MCAN_RAM_FILT_STD_SIZE \
    + RAM_ARRAY_SIZE_FILT_EXT * MCAN_RAM_FILT_EXT_SIZE \
    + RAM_FIFO_SIZE_RX0 * RAM_BUF_SIZE \
    + RAM_ARRAY_SIZE_RX * RAM_BUF_SIZE \
    + RAM_ARRAY_SIZE_TX * RAM_BUF_SIZE \
    + RAM_FIFO_SIZE_TX * RAM_BUF_SIZE )

struct frame_desc
{
    uint32_t id;
    uint8_t data[64];
    uint8_t len;
    uint8_t buf_idx;
};

/* the Message RAM is allocated from within the SAMC21's RAM
 */
static uint32_t mcan_msg_ram[MSG_RAM_SIZE] __ALIGNED(4);
static struct mcan_set mcan;
static volatile bool rx_ded_buffer_data = false;
/**
 * \brief Handler for interrupt line 1 of MCANx.
 */
void CAN0_Handler(void)
{
    if (SOC_CAN->IR.reg & CAN_IR_RF0N) {
        canbus_notify_rx();
    }
    if (SOC_CAN->IR.reg & CAN_IR_TFE) {
        canbus_notify_tx();
    }
}

// Read the next CAN packet
int
canbus_read(uint32_t *id, uint8_t *data)
{
    struct mcan_msg_info msg = { .data = data };

    uint8_t msgs_remain = mcan_dequeue_received_msg(&mcan, 0, &msg);
    if (msgs_remain > 0)
        canbus_notify_rx();

    // Return packet
    *id = msg.id;
    return msg.data_len;
}

// Transmit a packet
int
canbus_send(uint32_t id, uint32_t len, uint8_t *data)
{
    mcan_enqueue_outgoing_msg(&mcan, id, len, data);
    return len;
}

// Setup the receive packet filter
void
canbus_set_filter(uint32_t id)
{
    mcan_filter_single_id(&mcan, 0, 0, id);
}

void
can_init(void)
{
    enable_pclock(CAN0_GCLK_ID, ID_CAN0);
    gpio_peripheral(GPIO_Rx, CAN_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION, 0);

    const struct mcan_config mcan_cfg = {
            .id = ID_CAN0,
            .regs = CAN0,

            .msg_ram = mcan_msg_ram,

            .array_size_filt_std = 0,
            .array_size_filt_ext = 0,
            .fifo_size_rx0 = RAM_FIFO_SIZE_RX0,
            .fifo_size_rx1 = 0,
            .array_size_rx = 0,
            .fifo_size_tx_evt = 0,
            .array_size_tx = 0,
            .fifo_size_tx = RAM_FIFO_SIZE_TX,

            .buf_size_rx_fifo0 = 8,
            .buf_size_rx_fifo1 = 0,
            .buf_size_rx = 8,
            .buf_size_tx = 8,

            /*
            using values from AT6493 (SAMC21 app note); the plus values are to add on what the MCAN driver subtracts back off
            */
            .bit_rate = 500000,
            .quanta_before_sp = 10 + 2,
            .quanta_after_sp = 3 + 1,
            .quanta_sync_jump = 3 + 1,

            /*
            AT6493 (SAMC21 app note) 'fast' values were unhelpfully the same as normal speed; these are for double (1MBit)
                    the maximum peripheral clock of 48MHz on the SAMC21 does restrict us from very high rates
            */
            .bit_rate_fd = 5000000,
            .quanta_before_sp_fd = 7 + 2,
            .quanta_after_sp_fd = 2 + 1,
            .quanta_sync_jump_fd = 2 + 1,
    };

    uint32_t mcan_msg_ram_size = ARRAY_SIZE(mcan_msg_ram);
    mcan_configure_msg_ram(&mcan_cfg, &mcan_msg_ram_size);
    mcan_set_mode(&mcan, MCAN_MODE_CAN);

    /*##-2- Configure the CAN Filter #######################################*/
    canbus_set_filter(0);

    /*##-3- Configure Interrupts #################################*/
    mcan.cfg.regs->IE.reg |= CAN_IE_RF0NE || CAN_IE_TFEE;
    mcan.cfg.regs->ILE.reg |= CAN_ILE_EINT0;
    armcm_enable_irq(CAN0_Handler, CAN0_IRQn, 0);

    // Convert unique 96-bit chip id into 48 bit representation
    uint64_t hash = fasthash64((uint8_t*)UID_BASE, 12, 0xA16231A7);
    canbus_set_uuid(&hash);
    mcan_enable(&mcan);
}
DECL_INIT(can_init);
