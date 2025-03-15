#include <nuttx/bits.h>
#include <stdint.h>

/* NuttX SPI mode number for spi config as defined in OpenAlliance TC6 */
#define OA_TC6_SPI_MODE 0

#define CHUNK_DEFAULT_PAYLOAD_SIZE 64
/* #define CHUNK_DEFAULT_PAYLOAD_SIZE 32 */
#define CHUNK_DEFAULT_SIZE (CHUNK_DEFAULT_PAYLOAD_SIZE + 4)

/* nvc7410 registers, most defined in OpenAlliance TC6 */
#define IDVER_ADDR               0x0U
#define IDVER_MMS                0

#define PHY_ID_ADDR              0x1U
#define PHY_ID_MMS               0

#define SPICAP_ADDR              0x2U
#define SPICAP_MMS               0

#define RESET_ADDR               0x3U
#define RESET_MMS                0
#define RESET_SWRESET_MASK       BIT(0)
#define RESET_SWRESET_POS        0

#define CONFIG0_ADDR             0x4U
#define CONFIG0_MMS              0
#define CONFIG0_CPS_MASK         GENMASK(2, 0)
#define CONFIG0_CPS_POS          0

#define STATUS0_ADDR             0x8U
#define STATUS0_MMS              0
#define STATUS0_HDRE_MASK        BIT(5)
#define STATUS0_HDRE_POS         5
#define STATUS0_RESETC_MASK      BIT(6)
#define STATUS0_RESETC_POS       6

#define BUFSTS_ADDR              0xBU
#define BUFSTS_MMS               0

#define IMASK_ADDR               0xCU
#define IMASK_MMS                0

#define PHY_CONTROL_REG_ADDR     0xFF00U
#define PHY_CONTROL_REG_MMS      0
#define PHY_CONTROL_LCTL_POS     12

#define PHY_STATUS_REG_ADDR      0xFF01U
#define PHY_STATUS_REG_MMS       0

#define PHY_IDENTIFIER0_REG_ADDR 0xFF02U
#define PHY_IDENTIFIER0_REG_MMS  0

#define PHY_IDENTIFIER1_REG_ADDR 0xFF03U
#define PHY_IDENTIFIER1_REG_MMS  0

#define MAC_CONTROL0_REG_ADDR    0x0U
#define MAC_CONTROL0_REG_MMS     1
#define MAC_CONTROL0_FCSA_POS    8
#define MAC_CONTROL0_TXEN_POS    1
#define MAC_CONTROL0_RXEN_POS    0

/* DIO registers specific to ncv7410 */
#define DIO_CONFIG_REG_ADDR      0x0012U
#define DIO_CONFIG_REG_MMS       12
#define DIO0_FUNC_POS            1
#define DIO1_FUNC_POS            9
#define DIO0_OUT_VAL_POS         0
#define DIO1_OUT_VAL_POS         8
#define DIO_TRISTATE_FUNC        0x0
#define DIO_GPIO_FUNC            0x1
#define DIO_SFD_TX_FUNC          0x2
#define DIO_SFD_RX_FUNC          0x3
#define DIO_LINK_CTRL_FUNC       0x4
#define DIO_SFD_TXRX_FUNC        0xB
#define DIO_TXRX_FUNC            0xF

/* Control Transaction Protocol header bits as defined in OpenAlliance TC6 */
#define CTP_DNC_MASK  BIT(31)
#define CTP_DNC_POS   31
#define CTP_HDRB_MASK BIT(30)
#define CTP_HDRB_POS  30
#define CTP_WNR_MASK  BIT(29)
#define CTP_WNR_POS   29
#define CTP_AID_MASK  BIT(28)
#define CTP_AID_POS   28
#define CTP_MMS_MASK  GENMASK(27, 24)
#define CTP_MMS_POS   24
#define CTP_ADDR_MASK GENMASK(23, 8)
#define CTP_ADDR_POS  8
#define CTP_LEN_MASK  GENMASK(7, 1)
#define CTP_LEN_POS   1
#define CTP_P_MASK    BIT(0)
#define CTP_P_POS     0

/* Data Transaction Protocol HEADER bits as defined in OpenAlliance TC6 */
#define DTPH_DNC_MASK  BIT(31)
#define DTPH_DNC_POS   31
#define DTPH_SEQ_MASK  BIT(30)
#define DTPH_SEQ_POS   30
#define DTPH_NORX_MASK BIT(29)
#define DTPH_NORX_POS  29
#define DTPH_VS_MASK   GENMASK(23, 22)
#define DTPH_VS_POS    22
#define DTPH_DV_MASK   BIT(21)
#define DTPH_DV_POS    21
#define DTPH_SV_MASK   BIT(20)
#define DTPH_SV_POS    20
#define DTPH_SWO_MASK  GENMASK(19, 16)
#define DTPH_SWO_POS   16
#define DTPH_EV_MASK   BIT(14)
#define DTPH_EV_POS    14
#define DTPH_EBO_MASK  GENMASK(13, 8)
#define DTPH_EBO_POS   8
#define DTPH_TSC_MASK  GENMASK(7, 6)
#define DTPH_TSC_POS   6
#define DTPH_P_MASK    BIT(0)
#define DTPH_P_POS     0

/* Data Transaction Protocol FOOTER bits as defined in OpenAlliance TC6 */
#define DTPF_EXST_MASK BIT(31)
#define DTPF_EXST_POS  31
#define DTPF_HDRB_MASK BIT(30)
#define DTPF_HDRB_POS  30
#define DTPF_SYNC_MASK BIT(29)
#define DTPF_SYNC_POS  29
#define DTPF_RCA_MASK  GENMASK(28, 24)
#define DTPF_RCA_POS   24
#define DTPF_VS_MASK   GENMASK(23, 22)
#define DTPF_VS_POS    22
#define DTPF_DV_MASK   BIT(21)
#define DTPF_DV_POS    21
#define DTPF_SV_MASK   BIT(20)
#define DTPF_SV_POS    20
#define DTPF_SWO_MASK  GENMASK(19, 16)
#define DTPF_SWO_POS   16
#define DTPF_FD_MASK   BIT(15)
#define DTPF_FD_POS    15
#define DTPF_EV_MASK   BIT(14)
#define DTPF_EV_POS    14
#define DTPF_EBO_MASK  GENMASK(13, 8)
#define DTPF_EBO_POS   8
#define DTPF_RTSA_MASK BIT(7)
#define DTPF_RTSA_POS  7
#define DTPF_RTSP_MASK BIT(6)
#define DTPF_RTSP_POS  6
#define DTPF_TXC_MASK  GENMASK(5, 1)
#define DTPF_TXC_POS   1
#define DTPF_P_MASK    BIT(0)
#define DTPF_P_POS     0

static inline int tx_credits(uint32_t footer)
{
  return (footer & DTPF_TXC_MASK) >> DTPF_TXC_POS;
}

static inline int rx_available(uint32_t footer)
{
  return (footer & DTPF_RCA_MASK) >> DTPF_RCA_POS;
}

static inline int header_bad(uint32_t footer)
{
  return (footer & DTPF_HDRB_MASK) >> DTPF_HDRB_POS;
}

static inline int ext_status(uint32_t footer)
{
  return (footer & DTPF_EXST_MASK) >> DTPF_EXST_POS;
}

static inline int data_valid(uint32_t footer)
{
  return (footer & DTPF_DV_MASK) >> DTPF_DV_POS;
}

static inline int start_valid(uint32_t footer)
{
  return (footer & DTPF_SV_MASK) >> DTPF_SV_POS;
}

static inline int start_word_offset(uint32_t footer)
{
  return (footer & DTPF_SWO_MASK) >> DTPF_SWO_POS;
}

static inline int end_valid(uint32_t footer)
{
  return (footer & DTPF_EV_MASK) >> DTPF_EV_POS;
}

static inline int end_byte_offset(uint32_t footer)
{
  return (footer & DTPF_EBO_MASK) >> DTPF_EBO_POS;
}

static inline int frame_drop(uint32_t footer)
{
  return (footer & DTPF_FD_MASK) >> DTPF_FD_POS;
}

static inline int rx_frame_timestamp_added(uint32_t footer)
{
  return (footer & DTPF_RTSA_MASK) >> DTPF_RTSA_POS;
}

static inline int rx_frame_timestamp_parity(uint32_t footer)
{
  return (footer & DTPF_RTSP_MASK) >> DTPF_RTSP_POS;
}

static inline int mac_phy_sync(uint32_t footer)
{
  return (footer & DTPF_SYNC_MASK) >> DTPF_SYNC_POS;
}
