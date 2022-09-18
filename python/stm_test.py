import serial

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM10'
print(ser.open())
# if not ser.open():
#     print("Error opening port!")
#     exit(0)

f = open('instr.txt')
instr_list = eval(f.readline())

for instr in instr_list:
	if instr[0:2] == 'PS':
		print(instr[3:])
		ser.write(instr[3:].encode())
		exit(0)
print(instr_list)