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
#include "driver_hx711_basic.h"
#include "nvm3_default.h"
#include "nvm3_default_config.h"

// Use the default nvm3 handle from nvm3_default.h
#define NVM3_DEFAULT_HANDLE nvm3_defaultHandle

#define WS_OFFSET_NVM3_KEY 0
#define WS_DIVIDER_NVM3_KEY 1

int32_t ws_nvm3_read_ws_offset(void);
int32_t ws_nvm3_read_ws_divider(void);

static int32_t ws_offset = 177437; //176930
static int32_t ws_divider = 2020; //1850

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
sl_status_t sl_bt_ws_weight_measurement_indicate(uint8_t connection, int32_t value)
{
    (void) value;

    sl_status_t sc = 0;
    uint8_t buf[5] = {0};
    int32_t weight = 0; //mg
    float tmp_c = 0.0; //kg
    volatile int32_t raw_voltage;
    volatile double voltage_v;

    if(hx711_basic_read((int32_t*) &raw_voltage, (double*) &voltage_v)==0)
    {
        (void) voltage_v;
        //weight = raw_voltage - 176930;
        weight = raw_voltage - ws_offset;
        //weight = weight / 1850; // 2021
        weight = weight / ws_divider;
        tmp_c = (float) weight / 1000;
        app_log_info("Weight: %5.3f kg\n\r", tmp_c);

        // Send weight measurement indication to connected client.
        weight_measurement_val_to_buf(weight, buf);
        sc = sl_bt_gatt_server_send_indication(connection,
                                               gattdb_weight_measurement,
                                               sizeof(buf),
                                               buf);
        //app_log_info("%d %d %d %d %d Size %d\n\r",buf[0],buf[1],buf[2],buf[3],buf[4],sizeof(buf));
    }
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
    int32_t temp_weight_offset = ws_nvm3_read_ws_offset();
    if(temp_weight_offset == 0)
    {
        ws_nvm3_write_ws_offset(ws_offset);
    }
    else
    {
        ws_offset = temp_weight_offset;
    }

    int32_t temp_weight_divider = ws_nvm3_read_ws_divider();
    if(temp_weight_divider == 0)
    {
        ws_nvm3_write_ws_divider(ws_divider);
    }
    else
    {
        ws_divider = temp_weight_divider;
    }
}

static void weight_measurement_val_to_buf(int32_t value, uint8_t *buffer)
{
  uint32_t tmp_value = ((uint32_t)value & 0x00ffffffu) \
                       | ((uint32_t)(-3) << 24);
  buffer[0] = 0;
  buffer[1] = value & 0xff;
  buffer[2] = (value >> 8) & 0xff;
  buffer[3] = (value >> 16) & 0xff;
  buffer[4] = (value >> 24) & 0xff;
}

/******************************************************************************
 * Write scale offset to NVM3
 ******************************************************************************/
uint8_t ws_nvm3_write_ws_offset(int32_t offset)
{
    ws_offset = offset;

    if(ECODE_NVM3_OK == nvm3_writeData(NVM3_DEFAULT_HANDLE, WS_OFFSET_NVM3_KEY, &offset, 4))
    {
        ws_offset = offset; //copy to global variable
        return 1;
    }
    return 0;
}

/******************************************************************************
 * Read scale offset from NVM3
 ******************************************************************************/
int32_t ws_nvm3_read_ws_offset(void)
{
    uint32_t type;
    int32_t offset;
    size_t len;
    Ecode_t err;

    err = nvm3_getObjectInfo(NVM3_DEFAULT_HANDLE, WS_OFFSET_NVM3_KEY, &type, &len);
    if(err != NVM3_OBJECTTYPE_DATA || type != NVM3_OBJECTTYPE_DATA)
    {
        return 0;
    }

    err = nvm3_readData(NVM3_DEFAULT_HANDLE, WS_OFFSET_NVM3_KEY, &offset, len);
    if(ECODE_NVM3_OK == err)
    {
        return offset;
    }
    return 0;
}

/******************************************************************************
 * Write and convert scale offset to local GATT
 ******************************************************************************/
sl_status_t ws_write_ws_offset_to_gatt(void)
{
    sl_status_t sc;
    uint8_t temp[4];
    temp[3] = ws_offset & 0xff;
    temp[2] = (ws_offset >> 8) & 0xff;
    temp[1] = (ws_offset >> 16) & 0xff;
    temp[0] = (ws_offset >> 24) & 0xff;
    sc = sl_bt_gatt_server_write_attribute_value(gattdb_scale_offset, 0, 4, temp);
    return sc;
}

/******************************************************************************
 * Write and convert GATT scale offset to NVM3 and back to GATT
 ******************************************************************************/
void ws_write_gatt_ws_offset_to_nvm3(uint8array *value)
{
    int32_t temp_scale_offset = 0;
    uint8_t var = value->len-1;
    for (uint8_t i = 0; i < value->len; i++)
    {
        temp_scale_offset += value->data[var]<<(8*i);
        var--;
    }
    ws_nvm3_write_ws_offset(temp_scale_offset);
    ws_write_ws_offset_to_gatt();
}

/******************************************************************************
 * Write scale divider to NVM3
 ******************************************************************************/
uint8_t ws_nvm3_write_ws_divider(int32_t divider)
{
    ws_divider = divider;

    if(ECODE_NVM3_OK == nvm3_writeData(NVM3_DEFAULT_HANDLE, WS_DIVIDER_NVM3_KEY, &divider, 4))
    {
        ws_divider = divider; //copy to global variable
        return 1;
    }
    return 0;

}

/******************************************************************************
 * Read scale divider from NVM3
 ******************************************************************************/
int32_t ws_nvm3_read_ws_divider(void)
{
    uint32_t type;
    int32_t divider;
    size_t len;
    Ecode_t err;

    err = nvm3_getObjectInfo(NVM3_DEFAULT_HANDLE, WS_DIVIDER_NVM3_KEY, &type, &len);
    if(err != NVM3_OBJECTTYPE_DATA || type != NVM3_OBJECTTYPE_DATA)
    {
        return 0;
    }

    err = nvm3_readData(NVM3_DEFAULT_HANDLE, WS_DIVIDER_NVM3_KEY, &divider, len);
    if(ECODE_NVM3_OK == err)
    {
        return divider;
    }
    return 0;
}

/******************************************************************************
 * Write and convert scale divider to local GATT
 ******************************************************************************/
sl_status_t ws_write_ws_divider_to_gatt(void)
{
    sl_status_t sc;
    uint8_t temp[4];
    temp[3] = ws_divider & 0xff;
    temp[2] = (ws_divider >> 8) & 0xff;
    temp[1] = (ws_divider >> 16) & 0xff;
    temp[0] = (ws_divider >> 24) & 0xff;
    sc = sl_bt_gatt_server_write_attribute_value(gattdb_scale_divider, 0, 4, temp);
    return sc;
}

/******************************************************************************
 * Write and convert GATT scale divider to NVM3 and back to GATT
 ******************************************************************************/
void ws_write_gatt_ws_divider_to_nvm3(uint8array *value)
{
    int32_t temp_scale_divider = 0;
    uint8_t var = value->len-1;
    for (uint8_t i = 0; i < value->len; i++)
    {
        temp_scale_divider += value->data[var]<<(8*i);
        var--;
    }
    ws_nvm3_write_ws_divider(temp_scale_divider);
    ws_write_ws_divider_to_gatt();
}
