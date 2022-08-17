/*******************************************************************************
 * @file test.c
 * @brief non volatile memory functions
 * Created on: 12.08.2022
 * Author: LBekel
 ******************************************************************************/

#include <stdbool.h>
#include "em_common.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app_assert.h"
#include "nvm3_default.h"
#include "nvm3_default_config.h"

// Use the default nvm3 handle from nvm3_default.h
#define NVM3_DEFAULT_HANDLE nvm3_defaultHandle

#define NVM_TEAM_KEY 2

/******************************************************************************
 * stores team in nvm
 * @param[in] team
 * @param[out] true on success
 ******************************************************************************/
uint8_t nvm_store_team(uint8_t team)
{
    //only team 1 and 2 allowed
    if((team == 1)||(team == 2))
    {
        if(ECODE_NVM3_OK == nvm3_writeData(NVM3_DEFAULT_HANDLE, NVM_TEAM_KEY, &team, 1))
        {
            return 1;
        }
    }
    return 0;
}


/******************************************************************************
 * read team from nvm
 * @param[out] team number, 0 on error
 ******************************************************************************/
uint8_t nvm_read_team(void)
{
    uint32_t type;
    uint8_t team;
    size_t len;
    Ecode_t err;

    err = nvm3_getObjectInfo(NVM3_DEFAULT_HANDLE, NVM_TEAM_KEY, &type, &len);
    if(err != NVM3_OBJECTTYPE_DATA || type != NVM3_OBJECTTYPE_DATA)
    {
        return 0;
    }

    err = nvm3_readData(NVM3_DEFAULT_HANDLE, NVM_TEAM_KEY, &team, len);
    if(ECODE_NVM3_OK == err)
    {
        return team;
    }
    return 0;
}
