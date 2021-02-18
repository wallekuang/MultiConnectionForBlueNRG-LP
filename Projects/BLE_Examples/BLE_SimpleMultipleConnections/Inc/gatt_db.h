

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

#define CHARACTERISTIC_LEN  20

tBleStatus Add_Chat_Service(void);
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data);

extern uint16_t ChatServHandle, TXCharHandle, RXCharHandle;

extern const uint8_t Chat_service_uuid[16];
extern const uint8_t Chat_TX_char_uuid[16];
extern const uint8_t Chat_RX_char_uuid[16];

#endif /* _GATT_DB_H_ */
