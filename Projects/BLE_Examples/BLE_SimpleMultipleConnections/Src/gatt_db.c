
#include <stdio.h>
#include <string.h>
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "osal.h"
#include "app_state.h"
#include "profile.h"
#include "gatt_db.h"
#include "MultipleConnections_config.h"

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*
UUIDs:
D973F2E0-B19E-11E2-9E96-0800200C9A66
D973F2E1-B19E-11E2-9E96-0800200C9A66
D973F2E2-B19E-11E2-9E96-0800200C9A66
*/
  
#define SRVC_UUID           0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9
#define TX_CHR_UUID         0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9
#define RX_CHR_UUID         0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9

const uint8_t Chat_service_uuid[16] = {SRVC_UUID};
const uint8_t Chat_TX_char_uuid[16] = {TX_CHR_UUID};
const uint8_t Chat_RX_char_uuid[16] = {RX_CHR_UUID};

BLE_GATT_SRV_CCCD_DECLARE(tx, NUM_LINKS, BLE_GATT_SRV_PERM_NONE,
                     BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);


/* Chat TX (notification), RX(write without response)  characteristics definition */
static const ble_gatt_chr_def_t chat_chars[] = {
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(TX_CHR_UUID),
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(tx),
            .descr_count = 1U,
        },
    },
    {
        .properties = BLE_GATT_SRV_CHAR_PROP_WRITE | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(RX_CHR_UUID),
    },
};

/* Chat Service definition */
static const ble_gatt_srv_def_t chat_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_128(SRVC_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)chat_chars,
       .chr_count = 2U,
   },
};

uint16_t TXCharHandle, RXCharHandle;


/*******************************************************************************
* Function Name  : Add_Chat_Service
* Description    : Add the Chat service. This service has one characteristic with
*                  notify property, that is used to send data to the client.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Chat_Service(void)
{
  uint8_t ret;
  
  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&chat_service);
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }
  TXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&chat_chars[0]);
  RXCharHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&chat_chars[1]);
  
  printf("Chat Service added.\nTX Char Handle %04X, RX Char Handle %04X\n",
         TXCharHandle, RXCharHandle);
  
  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Chat service.\n");
  return BLE_STATUS_ERROR ;
}
