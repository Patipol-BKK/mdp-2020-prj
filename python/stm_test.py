import serial

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM10'
print(ser.open())
# if not ser.open():
#     print("Error opening port!")
#     exit(0)
instr_list = ['PS|FR045']
# f = open('instr.txt')
# instr_list = eval(f.readline())

for instr in instr_list:
    if instr[0:2] == 'PS':
        print(instr[3:])
        ser.write(instr[3:].encode())
#         while True:
#             bytesToRead = ser.inWaiting()
#             raw_dat = ser.readline()
#             # dat = raw_dat.strip().decode()
# #             if dat != '':   
#             print(raw_dat)
            # if dat[0] == 'R':
            #     break
        # exit(0)
print(instr_list)

# while True:
#     bytesToRead = ser.inWaiting()
#     raw_dat = ser.readline()
#     # dat = raw_dat.strip().decode()
#     print(raw_dat)
#     # if dat != '':   
#     #     print(dat)
#     # if dat[0] == 'R':
#     #     exit(0)
#     #     break