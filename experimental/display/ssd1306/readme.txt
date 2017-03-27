Hook up the display by:
1. Calling display_job_init in main
2. Calling SetDisplayEvent from onEvent in lora_job.c
3. Calling SetDisplayStatus from statusfunc in lora_job.c

Make sure that the DISPLAY_RESET pin is set up as an output pin.
Verify that you have hooked up the SDA, SCL and DISPLAY_RESET to the correct pins on the EE02 module

