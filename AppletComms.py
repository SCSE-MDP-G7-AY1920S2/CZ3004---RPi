import time
import socket

class AppletComm(object):

    #Initialize the items required for IP Comms
    def __init__(self):
        self.ipAddress = '192.168.7.7' #IP Address of the RPi
        self.isEstablished = False
        self.portNum = 36126 #Ephemeral Port, configure on Applet too
        self.client = None
        self.connection = None

    #Good to have to check if connected
    def isConnected(self):
        return self.isEstablished

    def connect(self):
        while True:
            retry = False
            try:
                #Let's wait for connection
                print ('[APPLET_INFO] Waiting for socket connection from Applet on {0}, port {1}'.format(self.ipAddress, self.portNum))
                self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                #self.connection.setblocking(0)
                # self.connection.setsockopt(socket.SOL_SOCKET, socket.TCP_NODELAY, 1)
                self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                print("[APPLET INFO] Socket reused. Trying to bind...")
                self.connection.bind((self.ipAddress, self.portNum))
                print("[APPLET INFO] Socket binded.")
                self.connection.listen(1)
                #self.connection.setblocking(0)
                (self.client, self.clientAddr) = self.connection.accept() #Client is the socket to transmit
                print('[APPLET_ACCEPTED] Connected to Applet.')
                self.isEstablished = True
                retry = False

            except Exception as e:
                print('[APPLET_ERROR] Applet Connection Error: %s' % str(e))
                retry = True

            #When established, break the while(true)
            if (self.isEstablished):
                break

            #When not yet established, keep retrying
            print('[APPLET_INFO] Retrying Applet Connection')
            time.sleep(1)

    #Disconnect when done
    def disconnect(self):
        if not (self.client is None):
            print('[APPLET_CLOSE] Shutting down Applet Connection')
            self.client.close()
            print('[APPLET_CLOSE] Applet Connection Shut Down Successfully')

        if not (self.connection is None):
            print('[APPLET_CLOSE] Shutting down RPi Connection')
            self.connection.close()
            print('[APPLET_CLOSE] RPi Connection Shut Down Successfully')

        self.isEstablished = False

    #The fundamental trying to receive
    def read(self):
        try:
            dataRcvBytes = self.client.recv(2048) #Buffer is 2048 bytes, returned value is byte stream
            dataRcvBytes = dataRcvBytes.decode('utf-8')

            if (dataRcvBytes):
                print('[APPLET_INFO] Received:{} '.format(dataRcvBytes))
                return dataRcvBytes

            else:
                print('[APPLET_ERROR] Null transmission. Attempting to re-establish.')
                self.disconnect()
                self.connect()
                return dataRcvBytes

        except Exception as e:
            pass
            '''
            print('[APPLET_ERROR] Receiving Error: %s' % str(e))

            if ('Broken pipe' in str(e) or 'Connection reset by peer' in str(e)):
                print('[APPLET_ERROR] Communication pipe error. Attempting to re-establish.')
                self.disconnect()
                self.connect()
            '''
    #The fundamental trying to send
    def write(self, message):
        try:
            #Make sure there is a connection first before sending
            if (self.isEstablished):
                message = message + "\n"
                self.client.send(message.encode('utf-8')) #self.clientAddr)
                return

            #There is no connections. Send what?
            else:
                print('[APPLET_INVALID] No Socket Connections')

        except Exception as e:
            print('[APPLET_ERROR] Cannot send message: %s' % str(e))
