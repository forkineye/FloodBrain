/*
 * XNRF24L01.h
 *
 * Project: XNRF24L01
 * Copyright (c) 2014 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */ 

#ifndef XNRF24L01_H_
#define XNRF24L01_H_

#include "config.h"
#include <util/delay.h>
#include <avr/io.h>

/************************************************************************/
/* BEGIN REGISTER AND COMMAND DEFINITIONS                               */
/************************************************************************/

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NOACK  0xB0
#define NRF_NOP             0xFF
#define REGISTER_MASK       0x1F
#define ACTIVATE            0x50

/* Non-P omissions */
#define LNA_HCURR	0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/************************************************************************/
/* END REGISTER AND COMMAND DEFINITIONS                               */
/************************************************************************/

//TODO: Update this to support USART
/*! \brief Structure which defines some items needed for nRF and SPI control.
 *  \param spi              Pointer to the SPI module this nRF is connected to.
 *  \param spi_port         Pointer to the port which the SPI module resides.
 *  \param ss_port          Pointer to the port containing the Slave Select pin for this NRF.
 *  \param ss_pin           Slave Select pin number for this NRF.
 *  \param ce_port          Pointer to the port containing the Chip Enable pin.
 *  \param ce_pin           Chip Enable pin number.
 *  \param addr_width       Address width to configure.  Valid values are 3-5.
 *  \param payload_width    Default payload width for all Pipes.  Valid values are 0-32.
 *  \param confbits         Configuration bits to be written to the CONFIG register for initialization.
 */
typedef struct {
    SPI_t   *spi;
    PORT_t  *spi_port;
    PORT_t  *ss_port;
    uint8_t ss_pin;
    PORT_t  *ce_port;
    uint8_t ce_pin;
    uint8_t addr_width;
    uint8_t payload_width;
    uint8_t confbits;
} xnrf_config_t;

typedef enum {
    XNRF_250KBPS,
    XNRF_1MBPS,
    XNRF_2MBPS
} xnrf_datarate_t;

/************************************************************************/
/* INLINE FUCTIONS                                                      */
/************************************************************************/

/*! \brief Pulls the Slave Select line low and selects our nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_select(xnrf_config_t *config) {
    config->ss_port->OUTCLR = (1 << config->ss_pin);
}

/*! \brief Pulls the Slave Select line high and de-selects our nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_deselect(xnrf_config_t *config) {
    config->ss_port->OUTSET = (1 << config->ss_pin);
}

/*! \brief Pulls the Chip Enable line high and enables our nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
 static inline void xnrf_enable(xnrf_config_t *config) {
    config->ce_port->OUTSET = (1 << config->ce_pin);
 }

/*! \brief Pulls the Chip Enable line low and disables our nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
 static inline void xnrf_disable(xnrf_config_t *config) {
    config->ce_port->OUTCLR = (1 << config->ce_pin);
 }

/*! \brief Flushes the RX FIFO.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \return         Contents of the STATUS register.
 */
static inline uint8_t xnrf_flush_rx(xnrf_config_t *config) {
    xnrf_select(config);
    uint8_t status = xspi_transfer_byte(config->spi, FLUSH_RX);
    xnrf_deselect(config);
    return status;
}

/*! \brief Flushes the TX FIFO.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \return         Contents of the STATUS register. 
 */
static inline uint8_t xnrf_flush_tx(xnrf_config_t *config) {
    xnrf_select(config);
    uint8_t status = xspi_transfer_byte(config->spi, FLUSH_TX);
    xnrf_deselect(config);
    return status;
}

/*! \brief Retrieves contents of the STATUS register.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \return         Contents of the STATUS register.
 */
static inline uint8_t xnrf_get_status(xnrf_config_t *config) {
    xnrf_select(config);
    uint8_t status = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);
    return status;
}

/*! \brief Returns a single byte for the given register.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param reg      Register to query.
 *  \return         Contents of the register.
 */
static inline uint8_t xnrf_read_register(xnrf_config_t *config, uint8_t reg) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (R_REGISTER | (REGISTER_MASK & reg)));
    uint8_t result = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);
    return result;
}

/*! \brief Writes a single byte to the given register.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param reg      Register to write.
 *  \param val      Value to write to the register.
 */
static inline void xnrf_write_register(xnrf_config_t *config, uint8_t reg, uint8_t val) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (W_REGISTER | (REGISTER_MASK & reg)));
    xspi_transfer_byte(config->spi, val);
    xnrf_deselect(config);
}

/*! \brief Reads RX payload width for the top payload in RX FIFO
  * \param config   Pointer to a xnrf_config_t structure.
  */
static inline uint8_t xnrf_get_rx_width(xnrf_config_t *config) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, R_RX_PL_WID);
    uint8_t result = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);
    return result;
}

/*! \brief Powers up the nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_powerup (xnrf_config_t *config) {
    xnrf_write_register(config, CONFIG, xnrf_read_register(config, CONFIG) | (1 << PWR_UP));
}

/*! \brief Powers down the nRF.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_powerdown (xnrf_config_t *config) {
    xnrf_write_register(config, CONFIG, xnrf_read_register(config, CONFIG) & ~(1 << PWR_UP));
}

/*! \brief Configures up the nRF for TX mode.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_config_tx (xnrf_config_t *config) {
    xnrf_disable(config);
    _delay_us(5);   /* Datasheet section 6.1.7 - Tpece2csn */
    xnrf_write_register(config, CONFIG, xnrf_read_register(config, CONFIG) & ~(1 << PRIM_RX));
}

/*! \brief Configures the nRF for RX mode.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_config_rx (xnrf_config_t *config) {
    xnrf_disable(config);
    _delay_us(5);   /* Datasheet section 6.1.7 - Tpece2csn */
    xnrf_write_register(config, CONFIG, xnrf_read_register(config, CONFIG) | (1 << PRIM_RX));
}

/*! \brief Sets the nRF channel.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param channel  Desired channel number (1-127).  We don't check so stay in range.  We're embedded FFS, don't be a tard.
 */
static inline void xnrf_set_channel (xnrf_config_t *config, uint8_t channel) {
    xnrf_write_register(config, RF_CH, channel);
}

/*! \brief Retrieves an array of bytes for the given register.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param reg      Register to trigger the query.
 *  \param data     Address of a buffer to hold the returned data.
 *  \param len      Length of the data you're retrieving.
 */
static inline void xnrf_read_register_buffer(xnrf_config_t *config, uint8_t reg, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (R_REGISTER | (REGISTER_MASK & reg)));
    while (len--)
        *data++ = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);
}
    
/*! \brief Writes an array of bytes for the given register.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param reg      Register to trigger the write.
 *  \param data     Pointer to the data we are sending.
 *  \param len      Size of the data we are sending.
 */
static inline void xnrf_write_register_buffer(xnrf_config_t *config, uint8_t reg, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (W_REGISTER | (REGISTER_MASK & reg)));
    while (len--)
        xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);
}

/*! \brief Retrieves a payload.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param data     Pointer to a buffer to hold our data.
 *  \param len      Length of the payload you're retrieving.
 */
static inline void xnrf_read_payload(xnrf_config_t *config, volatile uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, R_RX_PAYLOAD);
    while (len--)
        *data++ = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);
}

/*! \brief Retrieves a payload.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param buffer   Pointer to a RingBuffer structure to hold our data.
 *  \param len      Length of the payload you're retrieving.
 */
static inline void xnrf_read_payload_buffer(xnrf_config_t *config, RingBuffer_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, R_RX_PAYLOAD);
    while (len--)
        RingBuffer_Insert(buffer, xspi_transfer_byte(config->spi, NRF_NOP));
    xnrf_deselect(config);
}

/*! \brief Writes a payload to be transmitted.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param data     Pointer to a buffer that holds our data to send.
 *  \param len      Size of the payload we are sending.
 */
static inline void xnrf_write_payload(xnrf_config_t *config, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD);
    while (len--)
        xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);
}

/*! \brief Writes a payload to be transmitted.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param buffer   Pointer to a RingBuffer structure that holds our data.
 *  \param len      Size of the payload we are sending.
 */
static inline void xnrf_write_payload_buffer(xnrf_config_t *config, RingBuffer_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD);
    while (len--)
        xspi_transfer_byte(config->spi, RingBuffer_Remove(buffer));
    xnrf_deselect(config);
}

/*! \brief Writes a payload to be transmitted, explicitly disabling ACKs.
 *  \note Use this if you need to send packets w/o ACKs when configured to receive dynamic payloads.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param data     Pointer to a buffer that holds our data to send.
 *  \param len      Size of the payload we are sending.
 */
static inline void xnrf_write_payload_noack(xnrf_config_t *config, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD_NOACK);
    while (len--)
        xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);
}

/*! \brief Writes a payload to be transmitted, explicitly disabling ACKs.
 *  \note Use this if you need to send packets w/o ACKs when configured to receive dynamic payloads.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param buffer   Pointer to a RingBuffer structure that holds our data.
 *  \param len      Size of the payload we are sending.
 */
static inline void xnrf_write_payload_buffer_noack(xnrf_config_t *config, RingBuffer_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD_NOACK);
    while (len--)
        xspi_transfer_byte(config->spi, RingBuffer_Remove(buffer));
    xnrf_deselect(config);
}

/*! \brief Sets the TX Address.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param address  Pointer to the address to set.
 */
static inline void xnrf_set_tx_address(xnrf_config_t *config, uint8_t *address) {
    xnrf_write_register_buffer(config, TX_ADDR, address, config->addr_width);
}

/** The address setters are all defined seperately to eliminate conditional bloat **/

/*! \brief Sets the RX Address for Pipe 0.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param address  Pointer to the address to set.
 */
static inline void xnrf_set_rx0_address(xnrf_config_t *config, uint8_t *address) {
    xnrf_write_register_buffer(config, RX_ADDR_P0, address, config->addr_width);
}

/*! \brief Sets the RX Address for Pipe 1.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param address  Pointer to the address to set.
 */
static inline void xnrf_set_rx1_address(xnrf_config_t *config, uint8_t *address) {
    xnrf_write_register_buffer(config, RX_ADDR_P1, address, config->addr_width);
}

/*! \brief Sets the RX Address for Pipe 2.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param lsb      Least Significant Byte of the address for Pipe 2.  Rest of address is shared with Pipe 1.
 */
static inline void xnrf_set_rx2_address(xnrf_config_t *config, uint8_t lsb) {
    xnrf_write_register(config, RX_ADDR_P2, lsb);
}

/*! \brief Sets the RX Address for Pipe 3.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param lsb      Least Significant Byte of the address for Pipe 3.  Rest of address is shared with Pipe 1.
 */
static inline void xnrf_set_rx3_address(xnrf_config_t *config, uint8_t lsb) {
    xnrf_write_register(config, RX_ADDR_P3, lsb);
}

/*! \brief Sets the RX Address for Pipe 4.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param lsb      Least Significant Byte of the address for Pipe 4.  Rest of address is shared with Pipe 1.
 */
static inline void xnrf_set_rx4_address(xnrf_config_t *config, uint8_t lsb) {
    xnrf_write_register(config, RX_ADDR_P4, lsb);
}

/*! \brief Sets the RX Address for Pipe 5.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param lsb      Least Significant Byte of the address for Pipe 5.  Rest of address is shared with Pipe 1.
 */
static inline void xnrf_set_rx5_address(xnrf_config_t *config, uint8_t lsb) {
    xnrf_write_register(config, RX_ADDR_P5, lsb);
}

/*! \brief Pulses the CE line for 10us to transmit the payload.
 *  \param config   Pointer to a xnrf_config_t structure.
 */
static inline void xnrf_pulse_tx(xnrf_config_t *config) {
    xnrf_enable(config);    /* Pulse the nRF to start TX */
    _delay_us(11);          /* -for 10us per datasheet */
    xnrf_disable(config);   /* End pulse */
}

/************************************************************************/
/* Called functions                                                     */
/************************************************************************/

/*! \brief Initializes XNRF by setting up SPI according to the config structure and settings initial radio parameters.
  *  \param config  Pointer to a xnrf_config_t structure.
  *  \param config  Pointer to a xspi_config_t structure.
  */
void xnrf_init(xnrf_config_t *xnrf_config, xspi_config_t *xspi_config);

/*! \brief Sets the air datarate.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param rate     Datarate to use.
 */
void xnrf_set_datarate(xnrf_config_t *config, xnrf_datarate_t rate);

/*! \brief Sets the TX/RX Address width.
 *  \param config   Pointer to a xnrf_config_t structure.
 *  \param width    Width of the address.  Valid values are 3-5.
 */
void xnrf_set_address_width(xnrf_config_t *config, uint8_t width);

#endif /* XNRF24L01_H_ */