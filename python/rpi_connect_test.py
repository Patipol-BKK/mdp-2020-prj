import socket

HOST = '192.168.28.28' # Enter IP or Hostname of your server
PORT = 25000 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

#Lets loop awaiting for your input
command = "hi".encode('utf-8')
s.send(command)
# reply = s.recv(1024)

s.close()
print("reply")
