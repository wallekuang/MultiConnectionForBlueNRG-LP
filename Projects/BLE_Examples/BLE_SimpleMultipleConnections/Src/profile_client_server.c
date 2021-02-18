/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : profile.c
* Author             : AMS - RF  Application team
* Description        : This file define the procedure to connect and send data.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

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
#include "profile.h"
#include "gap_profile.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define SCAN_INTERVAL       ((uint16_t)(100/0.625))     // 100 ms
#define SCAN_WINDOW         ((uint16_t)(100/0.625))     // 100 ms
#define CONN_INTERVAL_MIN   ((uint16_t)(350/1.25))      // 350 ms
#define CONN_INTERVAL_MAX   ((uint16_t)(350/1.25))      // 350 ms
#define SUPERVISION_TIMEOUT ((uint16_t)(2000/10))       // 2000 ms
#define CE_LENGTH           ((uint16_t)(0/0.625))       // 0 ms

#define WRITE_INTERVAL_MS           2000

#define DEBUG         2

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


/* Private variables ---------------------------------------------------------*/

volatile int app_flags = 0;

static VTIMER_HandleType write_timer;

#if MAX_NUM_SLAVES

/* States of the state machine used to discover services, enable notifications and
  write the offset into the slaves.
*/
typedef enum{
  IDLE = 0,
  EXCHANGE_CONFIG,
  EXCHANGING_CONFIG,
  START_SERVICE_DISCOVERY,
  DISCOVERING_SERVICES,
  START_CHAT_CHAR_DISCOVERY,
  DISCOVERING_CHAT_CHAR,
  ENABLE_TX_CHAR_NOTIFICATIONS,
  ENABLING_TX_CHAR_NOTIFICATIONS,
  DONE,
} MasterState;

typedef enum{
  PAIRING_IDLE = 0,
  START_PAIRING,
  PAIRING,
  PAIRING_DONE
} MasterPairingState;

// Type of the structure used to store the state related to each server/slave
typedef struct _slave {
  uint8_t  address_type;
  uint8_t  address[6];
  uint16_t conn_handle;
  MasterState state;
  MasterState resume_state;
  MasterPairingState pairing_state;
  uint16_t Chat_serv_start_handle;
  uint16_t Chat_serv_end_handle;
  uint16_t tx_handle;
  uint16_t rx_handle;
}slave_device;

slave_device slaves[MAX_NUM_SLAVES];

#endif /* MAX_NUM_SLAVES */

#if MAX_NUM_MASTERS
// Type of the structure used to store the state related to each master
typedef struct _master {
  uint8_t  address_type;
  uint8_t  address[6];
  uint16_t conn_handle;
}master_device;

master_device masters[MAX_NUM_MASTERS];
#endif

uint8_t num_connected_slaves = 0;
uint8_t num_connected_masters = 0;

void SlaveInit(uint8_t slave_index);
void MasterInit(uint8_t master_index);


/* Private function prototypes -----------------------------------------------*/
void SendDataCB(void *param);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : address_is_set.
* Description    : Check if address is set, i.e. if it is different
*                  from 0x000000000000
* Input          : the address.
* Return         : TRUE if addres is set, FALSE otherwise.
*******************************************************************************/
uint8_t address_is_set(uint8_t address[6])
{
  int i;
  
  for(i = 0; i < 6; i++){
    if(address[i] != 0)
      break;
  }
  if(i == 6)
    return FALSE;
  else  
    return TRUE;
}


/*******************************************************************************
* Function Name  : Chat_DeviceInit.
* Description    : Init the CHAT device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t addr_len;
  uint8_t address[6];  
  uint8_t role = 0;
  
#if MAX_NUM_SLAVES
  role |= GAP_CENTRAL_ROLE;
#endif
#if MAX_NUM_MASTERS
  role |= GAP_PERIPHERAL_ROLE;
#endif
  
  uint8_t val = 1;
  aci_hal_write_config_data(0x30, 1, &val);
  
  /* Set the TX power to -2 dBm */
  aci_hal_set_tx_power_level(0, 25);
  
  /* Since we need to transfer notifications of 244 bytes in a single packet, the LL payload must be
   244 bytes for application data + 3 bytes for ATT header + 4 bytes for L2CAP header. */
   //ret = hci_le_write_suggested_default_data_length(251, 2120);
   //PRINTF("hci_le_write_suggested_default_data_length(): 0x%02x\r\n", ret);

  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00, strlen(name), STATIC_RANDOM_ADDR, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gap_init() --> SUCCESS\r\n");
  }

  aci_hal_read_config_data(0x80, &addr_len, address);
  PRINTF("Static random address: ");
  PRINT_ADDDRESS(address);
  PRINTF("\r\n");

  /* Set the device name */
  ret = Gap_profile_set_dev_name(0, strlen(name), (uint8_t *) name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error with Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
  aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
  
  ret = Add_Chat_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_Chat_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_Chat_Service() --> SUCCESS\r\n");
  }  
  
  aci_gap_set_authentication_requirement(NO_BONDING, MITM_PROTECTION_NOT_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0);
  
#if MAX_NUM_SLAVES
  
  for(int i = 0; i < MAX_NUM_SLAVES; i++)
    SlaveInit(i);
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, ACTIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  
  PRINTF("Scan configuration %02X\n", ret);
    
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  
  PRINTF("Connection configuration %02X\n", ret);
#endif  
  
#if MAX_NUM_MASTERS
  
  for(int i = 0; i < MAX_NUM_MASTERS; i++)
    MasterInit(i);
  
  ret = ConfigureAdvertising();
    
  PRINTF("Advertising configuration %02X\n", ret);
#endif  
  
  write_timer.callback = SendDataCB;
  
  HAL_VTIMER_StartTimerMs(&write_timer, WRITE_INTERVAL_MS);
  
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : App_SleepMode_Check.
* Description    : Check if the device can go to sleep. See sleep.h
* Input          : Requested sleep mdoe.
* Return         : Allowed sleep mode
*******************************************************************************/
PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

#if MAX_NUM_MASTERS
/*******************************************************************************
* Function Name  : MasterInit.
* Description    : Init the master state
* Input          : Index of the master
* Return         : none.
*******************************************************************************/
void MasterInit(uint8_t master_index)
{  
  Osal_MemSet(&masters[master_index], 0, sizeof(master_device));
}
#endif

#if MAX_NUM_SLAVES
/*******************************************************************************
* Function Name  : SlaveInit.
* Description    : Init the slave state
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void SlaveInit(uint8_t slave_index)
{  
  Osal_MemSet(&slaves[slave_index], 0, sizeof(slave_device));
}

/*******************************************************************************
* Function Name  : StartDiscovery.
* Description    : Begin discovery of the services for the selected slave
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void StartDiscovery(uint8_t slave_index)
{
  slaves[slave_index].state = EXCHANGE_CONFIG;
}

/*******************************************************************************
* Function Name  : StartPairing.
* Description    : Begin pairing for the selected slave
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void StartPairing(uint8_t slave_index, uint16_t Connection_Handle, MasterState resume_state)
{    
  if(slaves[slave_index].pairing_state != PAIRING_IDLE){
    // Pairing already started
    return;
  }
  
  slaves[slave_index].pairing_state = START_PAIRING;
  slaves[slave_index].resume_state = resume_state;
  slaves[slave_index].state = IDLE;
  PRINTF_DBG2("resume_state %d\n",resume_state);
}

/*******************************************************************************
* Function Name  : PerSlaveStateMachine.
* Description    : State machine handling the discovery of the services, setting
*                  of the client characteristic configuratino descriptors and
*                  writing into the characteristics.
* Input          : none
* Return         : none.
*******************************************************************************/
void PerSlaveStateMachine(void)
{
  tBleStatus ret;
  
  for(int i = 0; i < MAX_NUM_SLAVES; i++){
    
    switch(slaves[i].pairing_state){
    case START_PAIRING:
      
      PRINTF("aci_gap_send_pairing_req, force_rebond %d\r\n", FALSE);
      ret = aci_gap_send_pairing_req(slaves[i].conn_handle, FALSE);
      if(ret){
        PRINTF("Error starting pairing %02X\r\n", ret);
        // Retry later        
      }
      else  {
        PRINTF("Pairing started.\r\n");
        slaves[i].pairing_state = PAIRING;
      }
      break;
    default:
      break;
    }
    
    //PRINTF("slaves[%d].state: %d\r\n", i, slaves[i].state);
  
    switch(slaves[i].state){
      
    case EXCHANGE_CONFIG:
      {
        /* Exchange ATT MTU */        
        ret = aci_gatt_clt_exchange_config(slaves[i].conn_handle);
        PRINTF_DBG2("aci_gatt_clt_exchange_config(): 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = EXCHANGING_CONFIG;
        }
        else {
          slaves[i].state = START_SERVICE_DISCOVERY;
        }        
      }
      break;      
    case START_SERVICE_DISCOVERY:
      {
        /* Start discovery of all primary services */
        
        ret = aci_gatt_clt_disc_all_primary_services(slaves[i].conn_handle);
        PRINTF_DBG2("aci_gatt_clt_disc_all_primary_services(): 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_SERVICES;
        }
        else {
          slaves[i].state = IDLE;
        }      
      }
      break;
    case START_CHAT_CHAR_DISCOVERY:
      {
        /* Start characteristic discovery for Chat Service */
        ret = aci_gatt_clt_disc_all_char_of_service(slaves[i].conn_handle, slaves[i].Chat_serv_start_handle, slaves[i].Chat_serv_end_handle);
        PRINTF_DBG2("aci_gatt_clt_disc_all_char_of_service() for Chat service: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_CHAT_CHAR;
        }
        else {
          slaves[i].state = IDLE;
        }      
      }
      break;
    case ENABLE_TX_CHAR_NOTIFICATIONS:
      {
        /* Enable notifications for TX characteristic */
        
        static uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        
        // TODO: this is hard coded but should be discovered.
        slaves[i].tx_handle = 0x11;
        
        ret = aci_gatt_clt_write(slaves[i].conn_handle, slaves[i].tx_handle+2, 2, client_char_conf_data);
        PRINTF_DBG2("aci_gatt_clt_write() to enable TX characteristic notifications: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = ENABLING_TX_CHAR_NOTIFICATIONS;
        }
        else if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES){
          // Retry later
        }
        else {
          slaves[i].state = IDLE;
        }
      }
      break;
    default:
    	break;      
    }  
  }  
}
#endif

#if MAX_NUM_MASTERS
/*******************************************************************************
* Function Name  : get_master_index_from_conn_handle.
* Description    : Get the index in the 'slaves' array corresponding to the given
*                  connection handle.
* Input          : conn_handle Connection handle
* Return         : index of the slave. If -1, slave has not been found
*******************************************************************************/
int get_master_index_from_conn_handle(uint16_t conn_handle)
{
  int i;
  
  for(i = 0; i < MAX_NUM_MASTERS; i++){
    if(masters[i].conn_handle == conn_handle){
      return i;
    }
  }  
  return -1;
}
#endif

#if MAX_NUM_SLAVES
/*******************************************************************************
* Function Name  : get_slave_index_from_conn_handle.
* Description    : Get the index in the 'slaves' array corresponding to the given
*                  connection handle.
* Input          : conn_handle Connection handle
* Return         : index of the slave. If -1, slave has not been found
*******************************************************************************/
int get_slave_index_from_conn_handle(uint16_t conn_handle)
{
  int i;
  
  for(i = 0; i < MAX_NUM_SLAVES; i++){
    if(slaves[i].conn_handle == conn_handle){
      return i;
    }
  }  
  return -1;
}
#endif

static void DeviceStateMachine(void)
{
#if MAX_NUM_MASTERS  
  
  if(num_connected_masters < MAX_NUM_MASTERS && !APP_FLAG(ADVERTISING)){
    
    if(StartAdvertising() == BLE_STATUS_SUCCESS){
      APP_FLAG_SET(ADVERTISING);
    }    
  }
#endif
  
#if MAX_NUM_SLAVES 
  
  if(num_connected_slaves < MAX_NUM_SLAVES && !APP_FLAG(SCANNING)){
    
    if(StartGeneralConnectionEstablishment() == BLE_STATUS_SUCCESS){
      APP_FLAG_SET(SCANNING);
    }    
  }
#endif
  
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
  
  DeviceStateMachine();
  
#if MAX_NUM_SLAVES
  PerSlaveStateMachine();
#endif
  
}/* end APP_Tick() */

void SendDataCB(void *param)
{
  tBleStatus ret;  
  static uint32_t counter = 0;  
  uint8_t data_sent = FALSE;
  uint8_t data[CHARACTERISTIC_LEN] = {0};
  
  HOST_TO_LE_32(data,counter);

#if MAX_NUM_SLAVES
  
  for(int i = 0; i < MAX_NUM_SLAVES; i++){    
    slaves[i].rx_handle = 0x14; /// TODO: hard coded for the moment
    if(address_is_set(slaves[i].address) && slaves[i].rx_handle){
      data_sent = TRUE;
      ret = aci_gatt_clt_write_without_resp(slaves[i].conn_handle, slaves[i].rx_handle+1, sizeof(data), data);
      if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Data sent to slave %d (%d) 0x%04X\n", i, counter, slaves[i].conn_handle);
      else
        PRINTF("Error sending data to slave %d: 0x%02X (handle 0x%04X)\n", i, ret, slaves[i].conn_handle);
    }
  }
#endif
  
#if MAX_NUM_MASTERS
  for(int i = 0; i < MAX_NUM_MASTERS; i++){
    if(address_is_set(masters[i].address)){
      data_sent = TRUE;  
      ret =  aci_gatt_srv_notify(masters[i].conn_handle, TXCharHandle + 1, BLE_GATT_SRV_NOTIFY_FLAG_NOTIFICATION, CHARACTERISTIC_LEN, data);
      if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Data sent to master %d (%d) 0x%04X\n", i, counter, masters[i].conn_handle);
      else
        PRINTF("Error sending data to master %d: 0x%02X (handle 0x%04X)\n", i, ret, masters[i].conn_handle);
    }
  }
#endif
  
  if(data_sent)  
    counter++;
  
  HAL_VTIMER_StartTimerMs(&write_timer, WRITE_INTERVAL_MS);
}

/* ***************** BlueNRG-LP Stack Callbacks ********************************/

void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{  
  PRINTF_DBG2("hci_le_connection_complete_event, handle: 0x%04X, Status %d\r\n", Connection_Handle, Status);
   
   if(Status == 0){
     
#if MAX_NUM_SLAVES
     
     if(Role == 0x00) { // Master role 
       int slave_index;
       
       APP_FLAG_CLEAR(SCANNING);
       
       for(slave_index = 0; slave_index < MAX_NUM_SLAVES; slave_index++){
         
         if(!address_is_set(slaves[slave_index].address)){
           slaves[slave_index].address_type = Peer_Address_Type;
           memcpy(slaves[slave_index].address, Peer_Address, 6);         
           slaves[slave_index].conn_handle = Connection_Handle;
           
           PRINTF("Connected with slave ");
           PRINT_ADDDRESS(Peer_Address);
           PRINTF(" (slave %d)\r\n", slave_index);
           
           //StartDiscovery(slave_index);
           // TODO: for the moment, skip directly to the phase where notifications are enabled
           slaves[slave_index].state = ENABLE_TX_CHAR_NOTIFICATIONS;
           
           break;
         }
       }
       if(slave_index == MAX_NUM_SLAVES)
         PRINTF("\n\nNO SPACE TO STORE SLAVE\n\n");
       
       num_connected_slaves++;
       PRINTF("Connected slaves: %d\n", num_connected_slaves);
     
     }
#endif
#if MAX_NUM_MASTERS
     if(Role == 0x01) { // Slave role
       
       APP_FLAG_CLEAR(ADVERTISING);
       
       for(int master_index = 0; master_index < MAX_NUM_MASTERS; master_index++){
         
         if(!address_is_set(masters[master_index].address)){
           masters[master_index].address_type = Peer_Address_Type;
           memcpy(masters[master_index].address, Peer_Address, 6);         
           masters[master_index].conn_handle = Connection_Handle;
           
           PRINTF("Connected with master ");
           PRINT_ADDDRESS(Peer_Address);
           PRINTF("\r\n");
           
           break;
         }
       }
       
       num_connected_masters++;
       PRINTF("Connected masters: %d\n", num_connected_masters);
       
     }
#endif // MAX_NUM_MASTERS
     
   }   
   else if(Status == BLE_ERROR_UNKNOWN_CONNECTION_ID){
     PRINTF_DBG2("Connection canceled.\r\n");
   }

}/* end hci_le_connection_complete_event() */


void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{
  
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  int i;
  
  PRINTF("hci_disconnection_complete_event, Status 0x%02X, Handle 0x%04X, Reason 0x%02X\n", Status, Connection_Handle, Reason);
  
  if(Status != 0){
    return;
  }
    
#if MAX_NUM_SLAVES  
  i = get_slave_index_from_conn_handle(Connection_Handle);
  
  if(i >= 0){
    
    PRINTF("Disconnected from slave ");
    PRINT_ADDDRESS(slaves[i].address);
    PRINTF("\n");
    
    SlaveInit(i);
    
    num_connected_slaves--;
    PRINTF("Connected slaves: %d\n", num_connected_slaves);
    
    return;
  }
#endif
  
#if MAX_NUM_MASTERS
  
  i = get_master_index_from_conn_handle(Connection_Handle);
  
  if(i >= 0){ // If we arrive here, this condition should always be true   
    
    PRINTF("Disconnected from master ");
    PRINT_ADDDRESS(masters[i].address);
    PRINTF(", status 0x%02X, reason 0x%02X\r\n", Status, Reason);
    
    MasterInit(i);
    
    num_connected_masters--;    
    
    PRINTF("Connected masters: %d\n", num_connected_masters);
  }
  else {
    // We should not arrive here
    PRINTF("Disconnection: unexpected handle\n");
  }
#endif
  
}/* end hci_disconnection_complete_event() */

void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
#if MAX_NUM_SLAVES  
  
  int slave_index;
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);  
  if(slave_index < 0){    
    // Pairing as a slave
    PRINTF("Paired with master (0x%04X, 0x%02X, 0x%02X).\n", Connection_Handle, Status, Reason);
    
    return;
  }
  
  if(Status != 0){
    PRINTF("Pairing failed. Status %02X, Reason: %02X\r\n", Status, Reason);
    slaves[slave_index].state = DONE;
    slaves[slave_index].resume_state = IDLE;
    slaves[slave_index].pairing_state = PAIRING_IDLE;
    
    aci_gap_remove_bonded_device(slaves[slave_index].address_type, slaves[slave_index].address);
    
    aci_gap_terminate(slaves[slave_index].conn_handle, BLE_ERROR_TERMINATED_REMOTE_USER);
    
    return;
  }
  
  slaves[slave_index].state = slaves[slave_index].resume_state;
  slaves[slave_index].resume_state = IDLE;
  slaves[slave_index].pairing_state = PAIRING_DONE;
  
  PRINTF("Pairing complete (slave %d).\n", slave_index);
#else  
  PRINTF("Paired device\n");
#endif  
  
}

#if MAX_NUM_MASTERS
/*******************************************************************************
 * Function Name  : aci_gatt_srv_write_event.
 * Description    : This event occurs when a write request for an attribute is
 *                  received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_srv_write_event(uint16_t Connection_Handle,
                                 uint8_t Resp_Needed,
                                 uint16_t Attribute_Handle,
                                 uint16_t Data_Length,
                                 uint8_t Data[])
{
    uint8_t att_error = BLE_ATT_ERR_NONE;
    int i;
    
    //PRINTF_DBG2("aci_gatt_srv_write_event, handle 0x%04X\r\n",Attribute_Handle);
  
    i = get_master_index_from_conn_handle(Connection_Handle);  

    if(i < 0){ // This should not happen
      return;
    }
  
    if(Attribute_Handle == RXCharHandle + 1)
    {
      uint32_t counter;
    
      counter = LE_TO_HOST_32(Data);
    
      PRINTF("Write from client %d: %d (%d bytes)\n", i, counter, Data_Length);
    
    }
    
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_error, 0,  NULL);
    }
}

#endif

#if MAX_NUM_SLAVES

void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint16_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{ 
  int i;
  
  i = get_slave_index_from_conn_handle(Connection_Handle);  

  if(i < 0){ // This should not happen
    return;
  }
 
  if(Attribute_Handle == slaves[i].tx_handle + 1)
  {
    uint32_t counter;
    
    counter = LE_TO_HOST_32(Attribute_Value);
    
    PRINTF("Notification from server %d: %d (%d bytes)\n", i, counter, Attribute_Value_Length);
    
  }  
}

void aci_att_clt_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t Attribute_Data_Length,
                                           uint16_t Data_Length,
                                           uint8_t Attribute_Data_List[])
{
  int slave_index;
  
  //PRINTF_DBG2("aci_att_clt_read_by_group_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);  
  if(slave_index < 0) // This should not happen
    return;
  
  switch(slaves[slave_index].state){
    
  case DISCOVERING_SERVICES:
    if(Attribute_Data_Length == 20){ // Only 128bit UUIDs
      for(int i = 0; i < Data_Length; i += Attribute_Data_Length){
        if(memcmp(&Attribute_Data_List[i+4],Chat_service_uuid,16) == 0){
          memcpy(&slaves[slave_index].Chat_serv_start_handle, &Attribute_Data_List[i], 2);
          memcpy(&slaves[slave_index].Chat_serv_end_handle, &Attribute_Data_List[i+2], 2);
          PRINTF("Slave %d, Chat service handles: 0x%04X 0x%04X\r\n", slave_index, slaves[slave_index].Chat_serv_start_handle, slaves[slave_index].Chat_serv_end_handle);
        }
      }
    }
    break;
  default:
	break;
  }
  
}

void print_uuid(uint8_t *uuid)
{
  for(int i = 0; i < 16; i++)
    PRINTF("%02X",uuid[i]);
}

void aci_att_clt_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint16_t Data_Length,
                                     uint8_t Handle_Value_Pair_Data[])
{
  int slave_index;
  uint16_t handle;
  
  //PRINTF_DBG2("aci_att_clt_read_by_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);
  if(slave_index < 0) // This should not happen
    return;
  
  switch(slaves[slave_index].state){
  case DISCOVERING_CHAT_CHAR:
    for(int i = 0; i < Data_Length; i += Handle_Value_Pair_Length){
      if(Handle_Value_Pair_Length == 21){ // 128-bit UUID
        handle = LE_TO_HOST_16(&Handle_Value_Pair_Data[i]);
        //print_uuid(&Handle_Value_Pair_Data[i+5]);
        if(memcmp(&Handle_Value_Pair_Data[i+5], Chat_TX_char_uuid, 16) == 0){
          slaves[slave_index].tx_handle = handle;
          PRINTF("TX Char handle for slave %d: 0x%04X\r\n", slave_index, handle);
        }
        else if(memcmp(&Handle_Value_Pair_Data[i+5], Chat_RX_char_uuid, 16) == 0){
          slaves[slave_index].rx_handle = handle;
           PRINTF("RX Char Handle for slave %d: 0x%04X\r\n", slave_index, handle);
        }
      }
    }
    break;    
  }
}

#define INSUFFICIENT_ENCRYPTION 0x0F

void aci_gatt_clt_error_resp_event(uint16_t Connection_Handle,
                               uint8_t Req_Opcode,
                               uint16_t Attribute_Handle,
                               uint8_t Error_Code)
{
  int i;
  
  //PRINTF_DBG2("aci_gatt_clt_error_resp_event %04X %02X %04X %02X\n", Connection_Handle, Req_Opcode, Attribute_Handle, Error_Code);
  
  i = get_slave_index_from_conn_handle(Connection_Handle);  
  if(i < 0) // This should not happen
    return;
  
  if(Error_Code == INSUFFICIENT_ENCRYPTION){
    // Start pairing
    StartPairing(i, Connection_Handle, slaves[i].state - 1); // After pairing go one state back
    return;
  }  
}

void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  int i;
  
  i = get_slave_index_from_conn_handle(Connection_Handle);
  
  if(i < 0) // This should not happen
    return;
  
  if(Error_Code != BLE_STATUS_SUCCESS){
    PRINTF_DBG2("Procedure terminated with error 0x%02X (0x%04X).\r\n", Error_Code, slaves[i].conn_handle);
    slaves[i].state = DONE;
    return;
  }
  
  switch(slaves[i].state){    
  case EXCHANGING_CONFIG:
    PRINTF("Configuration exchanged (0x%04X).\r\n", slaves[i].conn_handle);    
    slaves[i].state = START_SERVICE_DISCOVERY;
    break;    
  case DISCOVERING_SERVICES:
    PRINTF_DBG2("Discovering services ended (0x%04X).\r\n", slaves[i].conn_handle);
    if(slaves[i].Chat_serv_start_handle != 0)
      slaves[i].state = START_CHAT_CHAR_DISCOVERY;
    else
      slaves[i].state = DONE;
    break;    
  case DISCOVERING_CHAT_CHAR:
    PRINTF_DBG2("Discovering Chat Service characteristics ended (0x%04X).\r\n", slaves[i].conn_handle);
    if(slaves[i].tx_handle != 0)
      slaves[i].state = ENABLE_TX_CHAR_NOTIFICATIONS;
    else 
      slaves[i].state = DONE;
    break;
  case ENABLING_TX_CHAR_NOTIFICATIONS:
    PRINTF("Notifications for TX Charac enabled (0x%04X).\r\n", slaves[i].conn_handle);
    slaves[i].state = DONE;
    break;
  default:
	break;
  }
}

void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
  uint8_t AD_len, AD_type;
  uint8_t i = 0;
  tBleStatus ret;
  
  while(i < Advertising_Report[0].Data_Length){
    AD_len = Advertising_Report[0].Data[i];
    AD_type = Advertising_Report[0].Data[i+1];    
    if(AD_type == AD_TYPE_128_BIT_UUID_SERVICE_DATA){
      // Search for Chat service UUID
      if(memcmp(&Advertising_Report[0].Data[i+2], Chat_service_uuid, sizeof(Chat_service_uuid))==0 && Advertising_Report[0].Data[i+18] == SERVICE_DATA_TO_SEARCH_FOR){
        // Device found!
        ret = aci_gap_create_connection(LE_1M_PHY_BIT, Advertising_Report[0].Address_Type, Advertising_Report[0].Address);        
        PRINTF("aci_gap_create_connection %02X\r\n", ret);
        return;
      }
    }
    i += AD_len+1;
  }
  
}

void hci_le_extended_advertising_report_event(uint8_t Num_Reports,
                                              Extended_Advertising_Report_t Extended_Advertising_Report[])
{
  Advertising_Report_t Advertising_Report;
  Advertising_Report.Address_Type = Extended_Advertising_Report[0].Address_Type;
  memcpy(Advertising_Report.Address, Extended_Advertising_Report[0].Address, 6);
  Advertising_Report.Data_Length = Extended_Advertising_Report[0].Data_Length;
  Advertising_Report.Data = Extended_Advertising_Report[0].Data;
  Advertising_Report.RSSI = Extended_Advertising_Report[0].RSSI;
  hci_le_advertising_report_event(1, &Advertising_Report);
}

#endif

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
}

void aci_gap_bond_lost_event(void)
{  
  PRINTF("aci_gap_bond_lost_event\r\n");
}

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  PRINTF("hci_le_data_length_change_event handle: 0x%04X, MaxTxOctets: %d, MaxTxTime: %d, MaxRxOctets: %d, MaxRxTime: %d.\r\n", Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t RX_MTU)
{
  PRINTF("aci_att_exchange_mtu_resp_event, handle: 0x%04X, RX_MTU: %d \r\n", Connection_Handle, RX_MTU);
}
