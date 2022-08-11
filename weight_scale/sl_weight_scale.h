/***************************************************************************//**
 * @file
 * @brief Weight Scale GATT service
 *******************************************************************************

 ******************************************************************************/

#ifndef SL_WEIGHT_SCALE_H
#define SL_WEIGHT_SCALE_H

#include <stddef.h>
#include <stdint.h>

/**************************************************************************//**
 * Callback to handle connection closed event.
 * @param[in] reason Result code.
 * @param[in] connection Handle of the closed connection
 * @note To be implemented in user code.
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection);

/**************************************************************************//**
 * Send Weight Measurement characteristic indication to the client.
 * @param[in] connection Connection handle of the client.
 * @param[in] value Temperature value in milligram.
 *****************************************************************************/
sl_status_t sl_bt_ws_weight_measurement_indicate(uint8_t connection, int32_t value);

/**************************************************************************//**
 * Weight Measurement characteristic's CCCD has changed.
 * @param[in] connection Connection handle of the client.
 * @param[in] client_config Characteristic Client Configuration Flag.
 * @note To be implemented in user code.
 *****************************************************************************/
void sl_bt_ws_weight_measurement_indication_changed_cb(uint8_t connection,
                                                            sl_bt_gatt_client_config_flag_t client_config);

/**************************************************************************//**
 * Weight Measurement characteristic indication confirmed.
 * @param[in] connection Connection handle of the client.
 * @note To be implemented in user code.
 *****************************************************************************/
void sl_bt_ws_weight_measurement_indication_confirmed_cb(uint8_t connection);

/**************************************************************************//**
 * Bluetooth stack event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_ws_on_event(sl_bt_msg_t *evt);

uint8_t ws_nvm3_write_ws_offset(int32_t offset);
sl_status_t ws_write_ws_offset_to_gatt(void);
void ws_write_gatt_ws_offset_to_nvm3(uint8array *value);
uint8_t ws_nvm3_write_ws_divider(int32_t divider);
sl_status_t ws_write_ws_divider_to_gatt(void);
void ws_write_gatt_ws_divider_to_nvm3(uint8array *value);

#endif // SL_HEALTH_THERMOMETER_H
