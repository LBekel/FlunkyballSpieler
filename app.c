/***************************************************************************//**
 * @file
 * @brief Core application logic.
 ******************************************************************************/
#include <stdbool.h>
#include "em_common.h"
#include "sl_status.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_timer.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "sl_weight_scale.h"
#include "app.h"
#include "driver_hx711_basic.h"
//#include "nvm3_default.h"
#include "nvm3_default_config.h"
#include "nvm.h"

// Connection handle.
static uint8_t app_connection = 0;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Button state.
static volatile bool app_btn0_pressed = false;

// Periodic timer handle.
static sl_simple_timer_t app_ws_periodic_timer;

// Periodic timer callback.
static void app_ws_periodic_timer_cb(sl_simple_timer_t *timer, void *data);

sl_status_t gatt_write_team(uint8_t team);

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_bt_ws_on_event(evt);
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n\r",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      ws_write_ws_offset_to_gatt();
      ws_write_ws_divider_to_gatt();
      gatt_write_team(nvm_read_team());

      app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle, // advertising set handle
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);

      app_log_info("Started advertising\n\r");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("Connection opened\n\r");

#ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
      // Set remote connection power reporting - needed for Power Control
      sc = sl_bt_connection_set_remote_power_reporting(
        evt->data.evt_connection_opened.connection,
        sl_bt_connection_power_reporting_enable);
      app_assert_status(sc);
#endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
        app_log_info("Connection closed\n");

        // Generate data for advertising
        sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                   sl_bt_advertiser_general_discoverable);
        app_assert_status(sc);

        // Restart advertising after client has disconnected.
        sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                           sl_bt_advertiser_connectable_scannable);
        app_assert_status(sc);
        app_log_info("Started advertising\n");
      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
        if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_scale_offset)
        {
            ws_write_gatt_ws_offset_to_nvm3(&(evt->data.evt_gatt_server_attribute_value.value));
        }
        else if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_scale_divider)
        {
            ws_write_gatt_ws_divider_to_nvm3(&(evt->data.evt_gatt_server_attribute_value.value));
        }
        else if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_team)
        {
            nvm_store_team(evt->data.evt_gatt_server_attribute_value.value.data[0]);
        }
        else
        {
            printf("sl_bt_evt_gatt_server_attribute_value_id %d\r\n",evt->data.evt_gatt_server_attribute_value.value.data[0]);
            printf("sl_bt_evt_gatt_server_attribute_value_id %d\r\n",evt->data.evt_gatt_server_attribute_value.attribute);
        }
        break;


    case sl_bt_evt_gatt_server_characteristic_status_id:
      //app_log_info("characteristic_status\n\r",evt->data.evt_gatt_server_characteristic_status.status_flags);
      app_ws_periodic_timer_cb(&app_ws_periodic_timer, NULL);

      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
  (void)reason;
  (void)connection;
  sl_status_t sc;

  // Stop timer.
  sc = sl_simple_timer_stop(&app_ws_periodic_timer);
  app_assert_status(sc);
}


/**************************************************************************//**
 * Weight Measurement
 * Indication changed callback
 *
 * Called when indication of temperature measurement is enabled/disabled by
 * the client.
 *****************************************************************************/
void sl_bt_ws_weight_measurement_indication_changed_cb(uint8_t connection,
        sl_bt_gatt_client_config_flag_t client_config)
{
    sl_status_t sc;
    app_connection = connection;
    // Indication or notification enabled.
    if(sl_bt_gatt_disable != client_config)
    {
        // Start timer used for periodic indications.
//    sc = sl_simple_timer_start(&app_ws_periodic_timer,
//                               100,
//                               app_ws_periodic_timer_cb,
//                               NULL,
//                               true);
//    app_assert_status(sc);
        // Send first indication.
        app_ws_periodic_timer_cb(&app_ws_periodic_timer, NULL);
    }
    // Indications disabled.
    else
    {
        // Stop timer used for periodic indications.
        (void) sl_simple_timer_stop(&app_ws_periodic_timer);
    }
}

/**************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  // Button pressed.
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) {
      app_btn0_pressed = true;
    }
  }
  // Button released.
  else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
    if (&sl_button_btn0 == handle) {
      app_btn0_pressed = false;
    }
  }
}


/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic temperature measurements and indications.
 *****************************************************************************/
static void app_ws_periodic_timer_cb(sl_simple_timer_t *timer, void *data)
{
    (void) data;
    (void) timer;
    sl_status_t sc;
    // Send weight measurement indication to connected client.
    sc = sl_bt_ws_weight_measurement_indicate(app_connection, 0);
    if(sc)
    {
        app_log_warning("Failed to send weight measurement indication\n\r");
    }
}

#ifdef SL_CATALOG_CLI_PRESENT
void hello(sl_cli_command_arg_t *arguments)
{
  (void) arguments;
  bd_addr address;
  uint8_t address_type;
  sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
  app_assert_status(sc);
  app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n\r",-
               address_type ? "static random" : "public device",
               address.addr[5],
               address.addr[4],
               address.addr[3],
               address.addr[2],
               address.addr[1],
               address.addr[0]);
}
#endif // SL_CATALOG_CLI_PRESENT

/******************************************************************************
 * write team to GATT
 * @param[in] team number
 * @param[in] sl_status_t
 ******************************************************************************/
sl_status_t gatt_write_team(uint8_t team)
{
    sl_status_t sc;
    sc = sl_bt_gatt_server_write_attribute_value(gattdb_team, 0, sizeof(team), &team);
    return sc;
}
