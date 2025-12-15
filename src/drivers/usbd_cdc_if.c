// In src/usbd_cdc_if.c

#include "drivers/usbd_cdc_if.h"
#include <stdbool.h>

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

// Circular buffer pointers
static volatile uint32_t tx_head = 0;
static volatile uint32_t tx_tail = 0;
static volatile bool g_vcp_connected = false;

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TxComplete_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TxComplete_FS // Add the new transmit complete callback
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  // Initialize buffer pointers
  tx_head = 0;
  tx_tail = 0;
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;
    case CDC_SET_COMM_FEATURE:
      break;
    case CDC_GET_COMM_FEATURE:
      break;
    case CDC_CLEAR_COMM_FEATURE:
      break;
    case CDC_SET_LINE_CODING:
      break;
    case CDC_GET_LINE_CODING:
      break;
    case CDC_SET_CONTROL_LINE_STATE:
      // The host sets the DTR (Data Terminal Ready) line when it opens the port.
      // We can use this to know if the VCP is connected.
      g_vcp_connected = (pbuf[0] & 0x01) ? true : false;
      break;
    case CDC_SEND_BREAK:
      break;
    default:
      break;
  }
  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

/**
  * @brief  CDC_TxComplete_FS
  *         Data transmitted callback
  *         This function is called when a transmission is complete.
  *         It checks the circular buffer for more data and starts a new transmission if available.
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TxComplete_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint32_t local_tx_tail = tx_tail; // Read volatile variable once for consistent check
  if (tx_head != local_tx_tail) {
    uint16_t len;
    // If head is behind tail, we can send all data up to the tail or the end of the buffer.
    if (tx_head < local_tx_tail) {
      len = local_tx_tail - tx_head;
    } else {
      // Data has wrapped around, send from head to the end of the buffer
      len = APP_TX_DATA_SIZE - tx_head;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &UserTxBufferFS[tx_head], len);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK) {
      tx_head = (tx_head + len) % APP_TX_DATA_SIZE;
    }
  }

  // If after that transmission, head and tail are different, it means we have more data to send
  // (likely from a wrap-around). We can trigger the next transmission immediately without waiting for another interrupt.
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState == 0 && tx_head != tx_tail) {
      CDC_TxComplete_FS(NULL, NULL, 0);
  }

  return (USBD_OK);
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  // Calculate the space available in the buffer. The buffer is full when there's 1 byte left.
  uint32_t buff_space;
  uint32_t local_tx_head = tx_head; // Read volatile variable once
  if (local_tx_head > tx_tail) {
      buff_space = local_tx_head - tx_tail - 1;
  } else {
      buff_space = APP_TX_DATA_SIZE - (tx_tail - local_tx_head) - 1;
  }

  if (Len > buff_space) {
      return USBD_FAIL; // Not enough space in the buffer
  }

  uint32_t next_tail = (tx_tail + Len) % APP_TX_DATA_SIZE;

  // Copy data to the circular buffer
  for (uint16_t i = 0; i < Len; i++) {
    UserTxBufferFS[(tx_tail + i) % APP_TX_DATA_SIZE] = Buf[i];
  }

  __disable_irq(); // --- Enter critical section ---
  tx_tail = next_tail; // Update tail *after* data is copied
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState == 0) {
      CDC_TxComplete_FS(NULL, NULL, 0);
  }
  __enable_irq(); // --- Exit critical section ---
  return USBD_OK;
}

/**
  * @brief  Checks if the USB VCP is connected to a host.
  * @retval true if connected, false otherwise.
  */
bool is_usb_vcp_connected(void) {
    return g_vcp_connected;
}