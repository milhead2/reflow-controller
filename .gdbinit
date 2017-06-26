target extended-remote :3333
monitor reset halt
load
monitor reset init
b main
b _assert_failed
continue