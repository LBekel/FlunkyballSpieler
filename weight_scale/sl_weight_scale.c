/***************************************************************************//**
 * @file
 * @brief Weight Scale GATT service
 *******************************************************************************
 * LBekel
 ******************************************************************************/

#include <stdbool.h>
#include "em_common.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app_assert.h"
#include "sl_weight_scale.h"


/**************************************************************************//**
 * Initialize Weight scale module.
 *****************************************************************************/
static void weight_scale_init(void);

/**************************************************************************//**
 * Convert weight value to characteristic buffer.
 * @param[in] value weight value in milligram.
 * @param[out] buffer Buffer to hold GATT characteristics value.
 *****************************************************************************/
static void weight_measurement_val_to_buf(int32_t value, uint8_t *buffer);

/**************************************************************************//**
 * Callback to handle connection closed event.
 *****************************************************************************/
SL_WEAK void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
  (void)reason;
  (void)connection;
}

/**************************************************************************//**
 * Send weight Measurement characteristic indication to the client.
 *****************************************************************************/
sl_status_t sl_bt_ws_weight_measurement_indicate(uint8_t connection,
                                                      int32_t value)
{
  sl_status_t sc;
  uint8_t buf[5] = { 0 };
  weight_measurement_val_to_buf(value, buf);
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_weight_measurement,
    sizeof(buf),
    buf);
  return sc;
}

/**************************************************************************//**
 * weight Measurement characteristic's CCCD has changed.
 *****************************************************************************/
SL_WEAK void sl_bt_ws_weight_measurement_indication_changed_cb(uint8_t connection,
                                  sl_bt_gatt_client_config_flag_t client_config)
{
  (void)connection;
  (void)client_config;
}

/**************************************************************************//**
 * weight Measurement characteristic indication confirmed.
 *****************************************************************************/
SL_WEAK void sl_bt_ws_weight_measurement_indication_confirmed_cb(uint8_t connection)
{
  (void)connection;
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 *****************************************************************************/
void sl_bt_ws_on_event(sl_bt_msg_t *evt)
{
  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      weight_scale_init();
      break;

    case sl_bt_evt_connection_closed_id:
      sl_bt_connection_closed_cb(evt->data.evt_connection_closed.reason,
                                 evt->data.evt_connection_closed.connection);
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (gattdb_weight_measurement == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        // client characteristic configuration changed by remote GATT client
        if (sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)
        {
          sl_bt_ws_weight_measurement_indication_changed_cb(
            evt->data.evt_gatt_server_characteristic_status.connection,
            (sl_bt_gatt_client_config_flag_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags);
        }
        // confirmation of indication received from remove GATT client
        else if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)
        {
          sl_bt_ws_weight_measurement_indication_confirmed_cb(
            evt->data.evt_gatt_server_characteristic_status.connection);
        }
        else
        {
          app_assert(false,
                     "[E: 0x%04x] Unexpected status flag in evt_gatt_server_characteristic_status\n",
                     (int)evt->data.evt_gatt_server_characteristic_status.status_flags);
        }
      }
      break;

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Private function definitions

static void weight_scale_init(void)
{
    uint8_t res = hx711_basic_init();
    if (res)
    {
        printf("hx711 init failed\r\n");
    }
}

static void weight_measurement_val_to_buf(int32_t value, uint8_t *buffer)
{
  uint32_t tmp_value = ((uint32_t)value & 0x00ffffffu) \
                       | ((uint32_t)(-3) << 24);
  buffer[0] = 0;
  buffer[1] = tmp_value & 0xff;
  buffer[2] = (tmp_value >> 8) & 0xff;
  buffer[3] = (tmp_value >> 16) & 0xff;
  buffer[4] = (tmp_value >> 24) & 0xff;
}
