#include "reflash.h"
#include "can.h"

#define BASE_ADDRESS 0x08000800

bool reflash_can_output(uint8_t can_id, uint8_t *buffer, uint8_t n) {
   CAN_TxHeaderTypeDef TxHeader = {
      .IDE = CAN_ID_STD,
      .StdId = can_id,
      .RTR = CAN_RTR_DATA,
      .DLC = n
   };
   
   bool status;
   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buffer, &TxMailbox) == 
       HAL_OK) {
      status = true;
   } else {
      status = false;
   }

   HAL_Delay(1);
}

uint8_t reflash_read_flash(uint16_t offset) {
   return *((void*)(BASE_ADDRESS + offset));
}

void reflash_write_flash(uint16_t offset, uint8_t data) {
   HAL_FLASH_Unlock();
   HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, BASE_ADDRESS + offset, data);
   HAL_FLASH_Lock();
}

reflash_generics_t reflash_generics = {
   .id = /* Fill this out */, 
   .writeable = true
};

reflash_data_t reflash_data_list[] = {
   /* Fill this out. */
};
