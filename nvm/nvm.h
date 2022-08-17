/*******************************************************************************
 * @file nvm.h
 * @brief non volatile memory functions
 * Created on: 12.08.2022
 * Author: LBekel
 ******************************************************************************/

#ifndef NVM_NVM_H_
#define NVM_NVM_H_

uint8_t nvm_store_team(uint8_t team);
uint8_t nvm_read_team(void);

#endif /* NVM_NVM_H_ */
