
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "profile.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "procedures.h"

#define ADV_INTERVAL_MIN    ((uint16_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint16_t)(100/0.625))     // 100 ms

#define DEBUG 2

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if DEBUG > 1
#include <stdio.h>
#define PRINTF_DBG2(...) printf(__VA_ARGS__)
#else
#define PRINTF_DBG2(...)
#endif

#define PRINT_ADDDRESS(a)   PRINTF("0x%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0])


const char name[] = LOCAL_NAME;
#define NAME_LENGTH (sizeof(name)-1)

#define USE_SCAN_RESP_DATA 1

#if MAX_NUM_MASTERS

#if USE_SCAN_RESP_DATA
static uint8_t scan_resp_data[19] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH + 1, AD_TYPE_COMPLETE_LOCAL_NAME};
#define UUID_SERVICE_DATA_LEN   0
#else
#define UUID_SERVICE_DATA_LEN   19
#endif

static uint8_t adv_data[5+NAME_LENGTH+UUID_SERVICE_DATA_LEN] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH+1, AD_TYPE_COMPLETE_LOCAL_NAME};

#define NON_DISCOVERABLE_ADV_DATA_LEN   3               // Do not send name and UUID if not discoverable
#define DISCOVERABLE_ADV_DATA_LEN       sizeof(adv_data)

#endif /* MAX_NUM_MASTERS */


tBleStatus StartGeneralConnectionEstablishment(void)
{
  tBleStatus ret;
  
  ret = aci_gap_start_procedure(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC, LE_1M_PHY_BIT, 0, 0);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting scanning: 0x%02X\r\n", ret);
    return ret;
  }
  else {
    PRINTF("Scanning...\r\n");
  }
  
  return BLE_STATUS_SUCCESS;
}

#if MAX_NUM_MASTERS
tBleStatus ConfigureAdvertising(void)
{
  tBleStatus ret;
  
  adv_data[2] = FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED;    
  // Add name to advertising data
  Osal_MemCpy(adv_data+5,name,NAME_LENGTH);
#if !USE_SCAN_RESP_DATA    
  // Add service UUID to advertising data
  adv_data[5+NAME_LENGTH] = 18;
  adv_data[5+NAME_LENGTH+1] = AD_TYPE_128_BIT_UUID_SERVICE_DATA;
  Osal_MemCpy(adv_data+5+NAME_LENGTH+2,Chat_service_uuid,sizeof(Chat_service_uuid));
  adv_data[5+NAME_LENGTH+2+16] = SERVICE_DATA_TO_ADVERTISE; // Service data, to identify different kind of devices (0: slave-only device, 1: master slave)
#else
  scan_resp_data[0] = 18;
  scan_resp_data[1] = AD_TYPE_128_BIT_UUID_SERVICE_DATA;
  Osal_MemCpy(scan_resp_data+2,Chat_service_uuid,sizeof(Chat_service_uuid));
  scan_resp_data[18] = SERVICE_DATA_TO_ADVERTISE; // Service data, to identify different kind of devices (0: slave-only device, 1: master slave)
#endif
  
  ret = aci_gap_set_advertising_configuration(0x00, // Advertising handle
                                              0x02, // General discoverable mode
                                              0x0013, // Connectable, Scannable, Legacy
                                              ADV_INTERVAL_MIN,
                                              ADV_INTERVAL_MAX,
                                              ADV_CH_ALL,
                                              0, NULL, // No peer address
                                              ADV_NO_WHITE_LIST_USE,
                                              127, // No preference for TX power
                                              LE_1M_PHY, // Primary_Advertising_PHY (not used for legacy adv)
                                              0, // Secondary_Advertising_Max_Skip (not used for legacy adv)
                                              LE_1M_PHY, //  Secondary_Advertising_PHY (not used for legacy adv)
                                              0, // Advertising_SID (not used for legacy adv)
                                              0); // No scan request notification
  PRINTF("Advertising configuration (discoverable) 0x%02X\n", ret);
  if(ret)
    return ret;
  
#if USE_SCAN_RESP_DATA
  ret = aci_gap_set_scan_response_data(0x00, sizeof(scan_resp_data), scan_resp_data);
  if(ret)
    return ret;
#endif
  
  ret = aci_gap_set_advertising_data(0x00, // Advertising handle
                                     0x03, // Complete data
                                     DISCOVERABLE_ADV_DATA_LEN, adv_data);
  PRINTF("aci_gap_set_advertising_data 0x%02X\n", ret);
  
  return ret;
  
}

tBleStatus StartAdvertising()
{
  tBleStatus ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters = {
    .Advertising_Handle = 0,
    .Duration = 0,
    .Max_Extended_Advertising_Events = 0,
  };
  
  ret = aci_gap_set_advertising_enable(ENABLE,1,&Advertising_Set_Parameters);
  
  PRINTF("Enable advertising 0x%02X\n", ret);
  return ret;
}

#endif

