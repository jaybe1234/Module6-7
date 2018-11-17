import struct
# import serial
class coordinate:

    def __init__(self,id,ser):
        self.id = id
        self.position = 0
        self.verify = 0
        self.ser = ser

    def setZero(self):
        packet = [255,255,self.id,1,0,0,0,0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)
        self.position = 0

    def setZ(self):
        packet = [255, 255, self.id, 3, 0, 0, 0, 0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)
        self.position = 0


    def move(self,position):
        self.ser.reset_output_buffer()
        if position >= 0:
            direction = 1
        else:
            direction = 0
        packet = [255, 255, self.id,2,direction,int(abs(position)),int((position*100)%100),0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)
        # while self.verify == 0:
        #     self.wait()


    def wait(self):
        if(self.ser.in_waiting > 0):

            # Read data out of the buffer until a carraige return / new line is found
            serialString = self.ser.read()
            value = ord(serialString)
            if value == self.id:
                self.verify = 1

    def flushVerify(self):
        self.verfy = 0

    def down(self,posz):
        packet = [255, 255, self.id, 4,1,posz,0,0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)
    def downdrop(self,posz):
        packet = [255, 255, self.id, 6,1,posz,0,0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)

    def grab(self):
        packet = [255, 255, self.id, 4, 1, 0, 0, 0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)

    def put(self,state):
        packet = [255, 255, self.id, 4, state, 0, 0, 0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)

    def rotate(self,angle):
        if abs(int(angle)) < 250:
                packet = [255, 255, self.id, 5, abs(int(angle)), 0,0, 0]
        else:
                value1=abs(int(angle))-250
                packet = [255, 255, self.id, 5, 250, value1, 0, 0]
        for i in range(len(packet)):
            packet[i] = struct.pack('B', packet[i])
        for i in packet:
            self.ser.write(i)

