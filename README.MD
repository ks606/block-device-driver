### Commands for sbull_script:\
   load\
   unload\

Block device creation and control performs via user_cmd file, which locates here:\
   /sys/bus/sbull/drivers/sbulld/user_cmd\

### Available commands:\
   set_devsize=N - setting size (in 512-byte sectors) of block device\
   add_device - if performs before set_devsize command, then created block device have 100 MB default size\

### Examples:\
#### Creation of block device\
   echo set_devsize=100 > user_cmd\
   echo add_device > user_cmd\