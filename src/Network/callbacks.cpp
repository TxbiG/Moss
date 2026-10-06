/** 
 @file callbacks.c
 @brief ENet callback functions
*/
#define ENET_BUILDING_LIB 1
#include <Moss/Moss_Network.h>

static ENetCallbacks callbacks = { malloc, free, abort };

int enet_initialize_with_callbacks(const ENetCallbacks * inits) {
   if (inits -> malloc != NULL || inits -> free != NULL)
   {
      if (inits -> malloc == NULL || inits -> free == NULL) { return -1; }
      callbacks.malloc = inits -> malloc;
      callbacks.free = inits -> free;
   }
      
   if (inits -> no_memory != NULL) { callbacks.no_memory = inits -> no_memory; }

   return Moss_Init_Network();
}
           
void* enet_malloc (size_t size) {
   void* memory = callbacks.malloc(size);
   if (memory == NULL) {callbacks.no_memory();}
   return memory;
}

void enet_free(void* memory) { callbacks.free(memory); }

