import serial

CSRF_FRAMETYPE_LINK_STATISTICS = 0x14
CSRF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CSRF_FRAMETYPE_DEVICE_PING = 0x28
CSRF_FRAMETYPE_DEVICE_INFO = 0x29

def main():
	port = serial.Serial('/dev/tty.usbserial-130', 115200)

	while True:
		header = port.read(1)
		if header == b'\xC8':
			length = int.from_bytes(port.read(1), 'little')
			rest = port.read(length)

			ty, payload, crc = rest[0], rest[1:-1], rest[-1]
			if ty == CSRF_FRAMETYPE_LINK_STATISTICS:
				print('Link statistics:')
				uplink_rssi_ant1, payload = payload[0], payload[1:]
				uplink_rssi_ant2, payload = payload[0], payload[1:]
				uplink_quality, payload = payload[0], payload[1:]
				uplink_snr, payload = int.from_bytes(payload[0:1], 'little', signed=True), payload[1:]

				print(f'RSSI1/2: {-uplink_rssi_ant1:3}/{-uplink_rssi_ant2:3} Quality: {uplink_quality:2}%')
			elif ty == CSRF_FRAMETYPE_RC_CHANNELS_PACKED:
				packed_channels = int.from_bytes(payload, 'little')
				channels = []
				for i in range(16):
					channel = packed_channels & 0x7FF
					channels.append(channel)
					packed_channels >>= 11
				#print('Channels:')
				#print(channels)

if __name__ == '__main__':
	main()