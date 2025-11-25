import json
import struct
import socket


class MessageManager():
    def __init__(self) -> None:
        pass

    def PackMessage(self, reqId, msgType, msg={}):
        msgLen = 0
        PACK_FMT_STR = '!BBHLH6s'
        jsonStr = json.dumps(msg)
        if (msg != {}):
            msgLen = len(jsonStr)
        rawMsg = struct.pack(PACK_FMT_STR, 0x5A, 0x01, reqId, msgLen, msgType, b'\x00\x00\x00\x00\x00\x00')
        # print("Message")
        # print("{:02X} {:02X} {:04X} {:08X} {:04X}".format(0x5A, 0x01, reqId, msgLen, msgType))

        if (msg != {}):
            rawMsg += bytearray(jsonStr, 'ascii')
            # print(msg)
        return rawMsg

    def UnpackMessage(self, so):
        PACK_FMT_STR = '!BBHLH6s'
        info = {}
        dataall = b''
        try:
            data = so.recv(16)
        except socket.timeout:
            print('timeout')
            so.close()
        jsonDataLen = 0
        # backReqNum = 0
        if len(data) < 16:
            print('pack head error')
            print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1')
            print(data)
            so.close()
        else:
            header = struct.unpack(PACK_FMT_STR, data)
            # print("{:02X} {:02X} {:04X} {:08X} {:04X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}       length: {}"
            # .format(header[0], header[1], header[2], header[3], header[4],
            # header[5][0], header[5][1], header[5][2], header[5][3], header[5][4], header[5][5],
            # header[3]))
            jsonDataLen = header[3]
            # backReqNum = header[4]
        dataall += data
        data = b''
        readSize = 1024
        try:
            while jsonDataLen > 0:
                recv = so.recv(1024)
                data += recv
                jsonDataLen -= len(recv)
                if jsonDataLen < readSize:
                    readSize = jsonDataLen
            # print(json.dumps(json.loads(data), indent=1))
            info = json.loads(data)
            dataall += data
            # print(' '.join('{:02X}'.format(x) for x in dataall))
        except socket.timeout:
            print('timeout')
        so.close()
        return info
