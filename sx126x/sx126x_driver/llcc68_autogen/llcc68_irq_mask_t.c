enum llcc68_irq_masks_e
{
    LLCC68_IRQ_NONE              = ( 0 << 0 ),
    LLCC68_IRQ_TX_DONE           = ( 1 << 0 ),
    LLCC68_IRQ_RX_DONE           = ( 1 << 1 ),
    LLCC68_IRQ_PREAMBLE_DETECTED = ( 1 << 2 ),
    LLCC68_IRQ_SYNC_WORD_VALID   = ( 1 << 3 ),
    LLCC68_IRQ_HEADER_VALID      = ( 1 << 4 ),
    LLCC68_IRQ_HEADER_ERROR      = ( 1 << 5 ),
    LLCC68_IRQ_CRC_ERROR         = ( 1 << 6 ),
    LLCC68_IRQ_CAD_DONE          = ( 1 << 7 ),
    LLCC68_IRQ_CAD_DETECTED      = ( 1 << 8 ),
    LLCC68_IRQ_TIMEOUT           = ( 1 << 9 ),
    LLCC68_IRQ_ALL               = LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE | LLCC68_IRQ_PREAMBLE_DETECTED |
                     LLCC68_IRQ_SYNC_WORD_VALID | LLCC68_IRQ_HEADER_VALID | LLCC68_IRQ_HEADER_ERROR |
                     LLCC68_IRQ_CRC_ERROR | LLCC68_IRQ_CAD_DONE | LLCC68_IRQ_CAD_DETECTED | LLCC68_IRQ_TIMEOUT,
};
