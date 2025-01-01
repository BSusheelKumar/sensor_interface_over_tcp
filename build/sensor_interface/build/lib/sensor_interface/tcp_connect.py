import struct

# Input values
supply_voltage = 12000      # Supply voltage in milli-volts
env_temp = 250              # Environment temperature in deci-Celsius
yaw = 100                   # Yaw in deci-degrees
pitch = -50                 # Pitch in deci-degrees
roll = 30                   # Roll in deci-degrees
command_id = 11

# Pack the data into a binary format using little-endian
packed_data = struct.pack('<BhhhhH', command_id, supply_voltage, env_temp, yaw, pitch, roll)

# Convert the packed data to hexadecimal string
hex_message = ''.join(f'{byte:02X}' for byte in packed_data)

# Add start and end markers
formatted_message = '$' + hex_message + '0D0A'

print(formatted_message)
