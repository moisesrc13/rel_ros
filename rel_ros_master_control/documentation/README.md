# DXMR110-8K Device Register Map

For multi por support each port has 999 holding registers
41001-41999 Port 1
42001-42999 Port 2

- each single word record is 16 bits long (or 2 octets | 2 bytes)
- two word record is 32 bits long (or 4 octets | 4 bytes)
- from the data example `example-sensor-data.jpeg`. there are 4 octets,
where most significant is octet 0 > 1 > 2 > 3, this is
11111111 | 11111111 | 11111111 | 11111111
-----------------------------------------
octet 0  |  octet 1 |  octet 2 | octet 3

so the count goes from 0 > 31 bits this is 32 bits in total = two modbus registers of 16 bits or a 4 bytes (octets) word
