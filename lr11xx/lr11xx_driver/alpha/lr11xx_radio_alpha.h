#ifndef __LR11XX_RADIO_ALPHA_H__
#define __LR11XX_RADIO_ALPHA_H__

#include "lr11xx_radio_types.h"
#include "lr11xx_types.h"

/*!
 * @brief Packet type values
 */
typedef enum
{
    LR11XX_RADIO_ALPHA_PACKET_NONE =
        0x00,  //!< State after cold start, Wi-Fi or GNSS capture, as the device has to be reconfigured
    LR11XX_RADIO_ALPHA_PACKET_GFSK    = 0x01,  //!< GFSK modulation
    LR11XX_RADIO_ALPHA_PACKET_LORA    = 0x02,  //!< LoRa modulation
    LR11XX_RADIO_ALPHA_PACKET_BPSK    = 0x03,  //!< BPSK modulation
    LR11XX_RADIO_ALPHA_PACKET_GMSK    = 0x04,  //!< GMSK modulation
    LR11XX_RADIO_ALPHA_PACKET_RANGING = 0x05,  //!< LoRa ranging mode
} lr11xx_radio_packet_types_alpha_t;

/*!
 * @brief Ramp-down delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-down time
 * of the power amplifier for BPSK operation.
 */
typedef enum
{
    LR11XX_RADIO_BPSK_RAMP_DOWN_DELAY_DEFAULT = 0x00,    //!< No optimization
    LR11XX_RADIO_BPSK_RAMP_DOWN_DELAY_100BPS  = 0x1D70,  //!< Optimal for 100bps
    LR11XX_RADIO_BPSK_RAMP_DOWN_DELAY_600BPS  = 0x04E1,  //!< Optimal for 600bps
} lr11xx_radio_bpsk_ramp_down_delay_t;

/*!
 * @brief Ramp-up delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-up time
 * of the power amplifier for BPSK operation.
 */
typedef enum
{
    LR11XX_RADIO_BPSK_RAMP_UP_DELAY_DEFAULT = 0x00,    //!< No optimization
    LR11XX_RADIO_BPSK_RAMP_UP_DELAY_100BPS  = 0x1306,  //!< Optimal for 100bps
    LR11XX_RADIO_BPSK_RAMP_UP_DELAY_600BPS  = 0x0325,  //!< Optimal for 600bps
} lr11xx_radio_bpsk_ramp_up_delay_t;

/*!
 * @brief Modulation configuration for BPSK packets
 */
typedef struct
{
    uint32_t                        bitrate;      //!< BPSK bitrate [bit/s]
    lr11xx_radio_gfsk_pulse_shape_t pulse_shape;  //!< BPSK pulse shape
} lr11xx_radio_modulation_param_bpsk_t;

/*!
 * @brief Modulation configuration for BPSK packets
 */
typedef struct
{
    uint32_t                        bitrate;      //!< GMSK bitrate [bit/s]
    lr11xx_radio_gfsk_pulse_shape_t pulse_shape;  //!< GMSK pulse shape
} lr11xx_radio_modulation_param_gmsk_t;

/*!
 * @brief Packet parameter configuration for BPSK packets
 */
typedef struct
{
    uint8_t                             payload_length;   //!< BPSK payload length [Bytes]
    lr11xx_radio_bpsk_ramp_up_delay_t   ramp_up_delay;    //!< Delay to fine tune ramp-up time [us]
    lr11xx_radio_bpsk_ramp_down_delay_t ramp_down_delay;  //!< Delay to fine tune ramp-down time [us]
    uint16_t                            bit_num;          //!< Used to ramp down the PA before the end of
                                                          //!< the payload length [bits]
} lr11xx_radio_packet_param_bpsk_t;

/*!
 * @brief GMSK Scrambling configurations
 */
typedef enum
{
    LR11XX_RADIO_GMSK_DCFREE_OFF       = 0x00,  //!< Whitening deactivated
    LR11XX_RADIO_GMSK_DCFREE_WHITENING = 0x01,  //!< Whitening enabled
} lr11xx_radio_gmsk_dc_free_t;

/*!
 * @brief GMSK Header Type configurations
 *
 * This parameter indicates whether or not the payload length is sent and read
 * over the air.
 *
 * If the payload length is known beforehand by both transmitter and receiver,
 * therefore there is no need to send it over the air. Otherwise, setting this
 * parameter to LR11XX_RADIO_GMSK_HEADER_TYPE_EXPLICIT will make the modem to
 * automatically prepand a byte containing the payload length to the the payload
 * on transmitter side. On receiver side, this first byte is read to set the
 * payload length to read.
 *
 * This configuration is only available for GMSK packet types.
 */
typedef enum
{
    LR11XX_RADIO_GMSK_HEADER_TYPE_IMPLICIT = 0x00,  //!< Payload length is not sent/read over the air
    LR11XX_RADIO_GMSK_HEADER_TYPE_EXPLICIT = 0x01,  //!< Payload length is sent/read over the air
} lr11xx_radio_gmsk_header_type_t;

/*!
 * @brief GMSK Preamble Detector Length configurations
 *
 * This parameter sets the minimum length of preamble bits to be received to
 * continue reception of incoming packet. If a packet with preamble length lower
 * than this value is being received, the reception stops without generating
 * IRQ.
 *
 * This parameter has no impact on TX operations.
 */
typedef enum
{
    LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_OFF    = 0x00,
    LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_8BITS  = 0x04,
    LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_16BITS = 0x05,
    LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_24BITS = 0x06,
    LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_32BITS = 0x07
} lr11xx_radio_gmsk_preamble_detect_length_t;

/*!
 * @brief GMSK Address Filtering configurations
 *
 * If Address Filtering is enabled but a wrong address is received, therefore
 * the reception is aborted and the address error flag of packet status is set.
 */
typedef enum
{
    LR11XX_RADIO_GMSK_ADDRESS_FILTERING_DISABLE      = 0x00,  //!< Filter deactivated
    LR11XX_RADIO_GMSK_ADDRESS_FILTERING_NODE_ADDRESS = 0x01,  //!< Filter on Node Address
    LR11XX_RADIO_GMSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES =
        0x02,  //!< Filtering on Node and Broadcast addresses
} lr11xx_radio_gmsk_address_filtering_t;

/*!
 * @brief GMSK Cyclic Redundancy Check configurations
 *
 * If this value is set to something other than CRC_OFF, a CRC is automatically
 * computed and added after the end of the payload on transmitter side. On
 * receiver side, the CRC check is automatically processed.
 */
typedef enum
{
    LR11XX_RADIO_GMSK_CRC_OFF        = 0x01,  //!< CRC check deactivated
    LR11XX_RADIO_GMSK_CRC_1BYTE      = 0x00,
    LR11XX_RADIO_GMSK_CRC_2BYTES     = 0x02,
    LR11XX_RADIO_GMSK_CRC_1BYTE_INV  = 0x04,
    LR11XX_RADIO_GMSK_CRC_2BYTES_INV = 0x06,
} lr11xx_radio_gmsk_crc_type_t;

/*!
 * @brief Packet parameter configuration for GMSK packet
 */
typedef struct
{
    uint16_t                                   preamble_len_in_bits;   //!< GMSK Preamble length [bits]
    lr11xx_radio_gmsk_preamble_detect_length_t preamble_detector;      //!< GMSK Preamble detection configuration
    uint8_t                                    sync_word_len_in_bits;  //!< GMSK Syncword length [bits]
    lr11xx_radio_gmsk_address_filtering_t      address_filtering;      //!< GMSK Address filtering configuration
    lr11xx_radio_gmsk_header_type_t            header_type;            //!< GMSK Header type configuration
    uint8_t                                    pld_len_in_bytes;       //!< GMSK Payload length [bytes]
    lr11xx_radio_gmsk_crc_type_t               crc_type;               //!< GMSK CRC configuration
    lr11xx_radio_gmsk_dc_free_t                dc_free;                //!< GMSK Whitening configuration
} lr11xx_radio_packet_param_gmsk_t;

typedef enum
{
    LR11XX_RF_FE_TEST_CASE_STANDBY = 0x00,  //!< Goes to standby RC mode
    LR11XX_RF_FE_TEST_CASE_GNSS_CAPTURE =
        0x01,  //!< Goes to GNSS capture mode with default GPS RF frequency (1.57542Ghz). Frequency can be changed
               //!< afterwards by calling lr11xx_radio_set_rf_freq command
    LR11XX_RF_FE_TEST_CASE_WIFI_CAPTURE_12_MHZ = 0x02,  //!< Goes to Wi-Fi capture mode with ADC 12MHz
    LR11XX_RF_FE_TEST_CASE_FS =
        0x03,  //!< Goes to FS mode, de-configures DIO1, starts PA self test (bpsk datatype=1, datarate=0)
    LR11XX_RF_FE_TEST_CASE_WIFI_CAPTURE_9_MHZ = 0x04,  //!< Goes to Wi-Fi capture mode with ADC 9MHz
    LR11XX_RF_FE_TEST_CASE_WIFI_CAPTURE_6_MHZ = 0x05,  //!< Goes to Wi-Fi capture mode with ADC 6MHz
    LR11XX_RF_FE_TEST_CASE_WIFI_CAPTURE_3_MHZ = 0x06,  //!< Goes to Wi-Fi capture mode with ADC 3MHz
    LR11XX_RF_FE_TEST_CASE_WIFI_DSP           = 0x07,  //!< Goes to Wi-Fi DSP mode (for power consumption measurement)
    LR11XX_RF_FE_TEST_CASE_GNSS_DSP           = 0x08,  //!< Goes to GNSS DSP mode (for power consumption measurement)
} lr11xx_radio_rf_fe_test_case_t;

/*!
 * @brief Set the modulation parameters for BPSK packets
 *
 * The command @ref lr11xx_radio_set_pkt_type must be called prior this one.
 *
 * @param [in] radio Radio abstraction
 *
 * @param [in] mod_params The structure of modulation configuration
 *
 * @see lr11xx_radio_set_pkt_type
 */
void lr11xx_radio_set_modulation_param_bpsk( const void*                                 radio,
                                             const lr11xx_radio_modulation_param_bpsk_t* mod_params );

/*!
 * @brief Set the modulation parameters for GMSK packets
 *
 * The command @ref lr11xx_radio_set_pkt_type must be called prior this one.
 *
 * @param [in] radio Radio abstraction
 *
 * @param [in] mod_params The structure of modulation configuration
 *
 * @see lr11xx_radio_set_pkt_type
 */
void lr11xx_radio_set_modulation_param_gmsk( const void*                                 radio,
                                             const lr11xx_radio_modulation_param_gmsk_t* mod_params );

/*!
 * @brief Set the packet parameters for BPSK packets
 *
 * The command @ref lr11xx_radio_set_pkt_type must be called prior this one.
 *
 * @param [in] radio Radio abstraction
 *
 * @param [in] pkt_params The structure of packet configuration
 *
 * @see lr11xx_radio_set_pkt_type, lr11xx_radio_set_modulation_param_bpsk
 */
void lr11xx_radio_set_packet_param_bpsk( const void* radio, const lr11xx_radio_packet_param_bpsk_t* pkt_params );

/*!
 * @brief Set the packet parameters for GMSK packets
 *
 * The command @ref lr11xx_radio_set_pkt_type must be called prior this one.
 *
 * @param [in] radio Radio abstraction
 *
 * @param [in] pkt_params The structure of packet configuration
 *
 * @see lr11xx_radio_set_pkt_type, lr11xx_radio_set_modulation_param_gmsk
 */
void lr11xx_radio_set_packet_param_gmsk( const void* radio, const lr11xx_radio_packet_param_gmsk_t* pkt_params );

/*!
 * @brief Enable the RF FE test mode
 *
 * @note Test case 9 for measuring RSSI path for GNSS has its own implementation provided by @ref
 * lr11xx_radio_read_gnss_rssi_test
 *
 * @param [in] radio Radio abstraction
 * @param [in] test_case Test case to be launched
 * @param [in] pkt_params Wi-Fi channel used (only valid for test cases 2, 4, 5, 6 )
 *
 * @see lr11xx_radio_read_gnss_rssi_test
 */
void lr11xx_radio_set_rf_fe_test( const void* radio, const lr11xx_radio_rf_fe_test_case_t test_case,
                                  uint8_t wifi_channel );

#endif  // __LR11XX_RADIO_ALPHA_H__
