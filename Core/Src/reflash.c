#include "reflash.h"

void reflash_can_hook(uint8_t can_id, uint8_t *buffer, uint8_t n) {
   // See if message applies to us
   if (!(can_id == CAN_MESSAGE_ROLLCALL ||
         can_id == CAN_MESSAGE_ROLLCALL_RESPONSE ||
         can_id == CAN_MESSAGE_DATA_REQUEST ||
         can_id == CAN_MESSAGE_DATA_RESPONSE || can_id == CAN_MESSAGE_DONE)) {
      return; // Return if it doesn't
   }

   switch (can_id) {
   case CAN_MESSAGE_ROLLCALL:
      // TODO: Send back our device ID
      break;

   case CAN_MESSAGE_DATA_REQUEST:
      // TODO: Send all the data we have available
      break;

   case CAN_MESSAGE_WRITE_DATA:
      // TODO: Write the data we got sent
      break;
   }
}

// TODO: Make sure this works :)
uint64_t reflash_read_data(uint16_t data_id) {
   uint16_t data_list_index = 0;
   while (data_list_index < DATA_LIST_SIZE) {
      if (data_list[data_list_index].data_id == data_id)
         break;
      data_list_index++;
   }

   uint16_t offset = data_list[data_list_index].offset;
   uint16_t n_bytes = data_list[data_list_index].size;

   uint64_t data = 0;
   while (n_bytes > 0) {
      data |= reflash_read_flash(offset) << ((n_bytes - 1) * 8);
      n_bytes--;
   }

   return data;
}
