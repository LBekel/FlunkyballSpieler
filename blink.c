/***************************************************************************//**
 * @file
 * @brief Blink examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_status.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_log.h"
#include "gatt_db.h"
#include "app_assert.h"
#include "sl_bluetooth.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifndef LED_INSTANCE
#define LED_INSTANCE               sl_led_led0
#endif

#ifndef TOOGLE_DELAY_MS
#define TOOGLE_DELAY_MS            500
#endif

#ifndef BLINK_TASK_STACK_SIZE
#define BLINK_TASK_STACK_SIZE      configMINIMAL_STACK_SIZE
#endif

#ifndef BLINK_TASK_PRIO
#define BLINK_TASK_PRIO            20
#endif

#ifndef EXAMPLE_USE_STATIC_ALLOCATION
#define EXAMPLE_USE_STATIC_ALLOCATION      0
#endif

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

static void blink_task(void *arg);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * Initialize blink example.
 ******************************************************************************/
void blink_init(void)
{
  TaskHandle_t xHandle = NULL;

#if (EXAMPLE_USE_STATIC_ALLOCATION == 1)

  static StaticTask_t xTaskBuffer;
  static StackType_t  xStack[BLINK_TASK_STACK_SIZE];

  app_log_info("start blink task\n\r");
  // Create Blink Task without using any dynamic memory allocation
  xHandle = xTaskCreateStatic(blink_task,
                              "blink task",
                              BLINK_TASK_STACK_SIZE,
                              ( void * ) NULL,
                              tskIDLE_PRIORITY + 1,
                              xStack,
                              &xTaskBuffer);

  // Since puxStackBuffer and pxTaskBuffer parameters are not NULL,
  // it is impossible for xHandle to be null. This check is for
  // rigorous example demonstration.
  EFM_ASSERT(xHandle != NULL);

#else

  BaseType_t xReturned = pdFAIL;

  // Create Blink Task using dynamic memory allocation
  xReturned = xTaskCreate(blink_task,
                          "blink task",
                          BLINK_TASK_STACK_SIZE,
                          ( void * ) NULL,
                          tskIDLE_PRIORITY + 1,
                          &xHandle);

  // Unlike task creation using static allocation, dynamic task creation can very likely
  // fail due to lack of memory. Checking the return value is relevant.
  EFM_ASSERT(xReturned == pdPASS);

#endif
}

/*******************************************************************************
 * Blink task.
 ******************************************************************************/
static void blink_task(void *arg)
{
  (void)&arg;

  //Use the provided calculation macro to convert milliseconds to OS ticks
  TickType_t xDelay = pdMS_TO_TICKS(TOOGLE_DELAY_MS);
  sl_status_t sc;
  uint8_t buf[1] = { 0 };
  size_t readlen = 0;
  while (1) {
    //Wait for specified delay
    vTaskDelay(xDelay);

    // Toggle led

    sc = sl_bt_gatt_server_read_attribute_value(gattdb_led_frequency, 0, sizeof(buf), &readlen, buf);
    app_assert_status(sc);
    if(buf[0]>0)
    {
       xDelay = pdMS_TO_TICKS(1000/buf[0]);
       sl_led_toggle(&LED_INSTANCE);
    }
    else
    {
        sl_led_turn_off(&LED_INSTANCE);
    }

  }
}
