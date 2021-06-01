# Simple TCP client in Python
import socket, time

ADDR = "10.1.1.11"
PORT = 1025
MESSAGE = b"Test %u"
DELAY = 1

def hex_str(bytes):
    return " ".join([("%02x" % int(b)) for b in bytes])

print("Send to TCP %s:%s" % (ADDR, PORT))
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.2)
sock.connect((ADDR, PORT))
count = 1
while True:
    msg = MESSAGE % count
    sock.sendall(msg)
    print("Tx %u: %s" % (len(msg), hex_str(msg)))
    count += 1
    try:
        data = sock.recv(1000)
    except:
        data = None
    if data:
        s = hex_str(data)
        print("Rx %u: %s\n" % (len(data), s))
    time.sleep(DELAY)
# EOF

