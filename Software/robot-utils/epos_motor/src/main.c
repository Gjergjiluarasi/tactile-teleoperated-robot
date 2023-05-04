#include <ecrt.h>
#include <stdio.h>

#define NUM_SLAVES 4

int main()
{
  ec_master_t adapter(
  ec_slave_config_t *sc;
  int ret, i;
  
  // Initialize the adapter
  ret = ecat_init(&adapter, 0, 0, NUM_SLAVES);
  if (ret < 0) {
    printf("Error initializing adapter\n");
    return -1;
  }

  // Configure the slaves
  for (i = 0; i < NUM_SLAVES; i++) {
    sc = ecat_get_slave_config(&adapter, i + 1);
    if (!sc) {
      printf("Error configuring slave %d\n", i + 1);
      return -1;
    }
  }

  // Start the adapter
  ret = ecat_start_adapter(&adapter);
  if (ret < 0) {
    printf("Error starting adapter\n");
    return -1;
  }

  // Check the state of the slaves
  for (i = 0; i < NUM_SLAVES; i++) {
    sc = ecat_get_slave_config(&adapter, i + 1);
    printf("Slave %d state: %s\n", i + 1, ecat_state_string(sc->state));
  }

  // Clean up and exit
  ecat_cleanup_adapter(&adapter);
  return 0;
}