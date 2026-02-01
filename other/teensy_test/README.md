This PlatformIO project is for testing the UART state machine on the Teensy to communicate with the XIAO for encoder packets.

Packet structure:
 - Sync word: Denotes the start of each packet (uint8_t, 4 bytes)
 - Header: The type of message ('S' for speed, 'E' for error) (char, 1 byte)
 - Payload:
    - Speed in deg/sec (float, 4 bytes), or
    - Error code ('I' for init error, 'C' for comm error) (char, 1 byte)