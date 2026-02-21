This PlatformIO project runs on the XIAO to communicate with the AS5600 encoder over I2C.

Packet structure:
 - Sync word: Denotes the start of each packet (uint8_t, 4 bytes)
 - Header: The type of message ('S' for speed, 'E' for error) (char, 1 byte)
 - Payload:
    - Speed in deg/sec (float, 4 bytes), or
    - Error code ('I' for init error, 'C' for comm error) (char, 1 byte)