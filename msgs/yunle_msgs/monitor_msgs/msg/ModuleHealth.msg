Header header
uint8 state
string moduleName
uint8 moduleType
int32 requestId

uint8 MODULE_OK = 0
uint8 MODULE_NOT_STARTED = 1
uint8 MODULE_DEVICE_NOT_FOUND = 2
uint8 MODULE_DEVICE_ERROR = 3

uint8 MODULE_WARNING = 16
uint8 MODULE_ERROR = 17

uint8 MODULE_TYPE_HARDWARE = 0
uint8 MODULE_TYPE_GENERAL = 1