#ifndef REFLASH_H // !REFLASH_H
#define REFLASH_H
#include <stdbool.h>
#include <stdint.h>

#define CAN_MESSAGE_ROLLCALL 0
#define CAN_MESSAGE_ROLLCALL_RESPONSE 1
#define CAN_MESSAGE_DATA_REQUEST 2
#define CAN_MESSAGE_DATA_RESPONSE 3
#define CAN_MESSAGE_WRITE_DATA 4
#define CAN_MESSAGE_DONE 5

typedef struct reflash_data_s {
   uint16_t data_id;
   uint16_t offset;
   uint8_t size;
} reflash_data_t;

typedef struct reflash_generics_s {
   uint8_t id;
   bool writeable;
} reflash_generics_t;

extern reflash_generics_t reflash_generics;

extern reflash_data_t reflash_data_list[];
extern const uint16_t DATA_LIST_SIZE;

void reflash_can_hook(uint8_t can_id, uint8_t *buffer, uint8_t n);
extern void reflash_can_output(uint8_t can_id, uint8_t *buffer, uint8_t n);

extern uint8_t reflash_read_flash(uint16_t offset);
extern void reflash_write_flash(uint16_t offset, uint8_t data);

bool reflash_has_new_data();
uint64_t reflash_read_data(uint16_t data_id);

#endif // !REFLASH_H
