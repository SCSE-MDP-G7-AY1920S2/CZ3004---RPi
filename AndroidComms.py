#Refer to Python Documentation for 'socket' info

import time
from bluetooth import *

class AndroidComm(object):
    
    #Initialize client-server connection
    def __init__(self):
        self.clientSock = None
        self.serverSock = None
        self.isEstablished = False

    #Good to have to check if connected
    def isConnected(self):
        return self.isEstablished

    #Initialize the Bluetooth Connection
    def connect(self):
        while True:
            BTChannel = 3 #The Bluetooth Channel According to Pi
            retry = False
            try:
                self.serverSock = BluetoothSocket(3)
                self.serverSock.bind(('', BTChannel)) #Can try bluetooth.PORT_ANY or BTChannel also
                self.serverSock.listen(1) #Specify how many clients the thing will wait for
                self.port = self.serverSock.getsockname()[1] #Value returned is [host, port]. We need port

                uuid = '00001101-0000-1000-8000-00805F9B34FB' #I don't know why it's using this
                #uuid = '7121A3C0-2CDE-4BED-8253-E3F6C8BCBCAF' #From Android
                #uuid = '94f39d29-7d6d-437d-973b-fba39e49d4ee' #From Leon's Android

                #Advertise service referenced from PyBluez Documentation
                advertise_service(self.serverSock, 'MDP-07',
                    service_id = uuid,
                    service_classes = [uuid, SERIAL_PORT_CLASS],
                    profiles = [SERIAL_PORT_PROFILE])

                #Let's wait for connection
                print ("[BLUETOOTH_INFO] Waiting for connection of RFCOMM channel %d" % self.port) #Which might not be true since we can use PORT_ANY

                #Accept a connection
                (self.clientSock, clientInfo) = self.serverSock.accept()
                print('[BLUETOOTH_ACCEPTED] Accepted Bluetooth connection from ' + str(clientInfo))
                self.isEstablished = True
                retry = False

            #Exception Handling
            except Exception as e:
                print('[BLUETOOTH_ERROR] Bluetooth Connection Error: %s' % str(e))
                retry = True

            #When established, break the while(true)
            if (self.isEstablished):
                break

            #When not yet established, keep retrying
            print('[BLUETOOTH_INFO] Retrying Bluetooth Establishment')
            time.sleep(1.5)

    #We need to shutdown any disconnected connections
    def disconnect(self):
        try:
            #If there is an existing server socket
            if not (self.serverSock is None):
                print('[BLUETOOTH_CLOSE] Shutting down BT Server')
                self.serverSock.shutdown() #Needed to shutdown in a timely fashion
                self.serverSock.close()
                print('[BLUETOOTH_CLOSE] BT Server Shut Down Successfully')

            #If there is an existing client socket
            if not (self.clientSock is None):
                print('[BLUETOOTH_CLOSE] Shutting down BT Client')
                self.clientSock.shutdown() #Needed to shutdown in a timely fashion
                self.clientSock.close()
                print('[BLUETOOTH_CLOSE] BT Client Shut Down Successfully')

        except Exception as e:
            print('[ANDROID] Android Disconnect Error: %s' % str(e))
            pass
        
        #Mark connected as false
        self.isEstablished = False

    #The fundamental trying to send
    def write(self, message):
        try:
            #Make sure there is a connection first before sending
            if (self.isEstablished):
                #self.clientSock.send(str(message))
                # print('[BLUETOOTH_INFO] Sent: ' + message)
                self.clientSock.send(message.encode('utf-8'))
                return

            #There is no connections. Send what?
            else:
                print('[BLUETOOTH_INVALID] No Bluetooth Connections')
                return

        except BluetoothError as e:
            print('[BLUETOOTH_ERROR] Cannot send message: %s' % str(e))

    #The fundamental trying to receive
    def read(self):
        try:
            dataRcvBytes = self.clientSock.recv(2048) #Buffer is 2048 bytes, returned value is byte stream
            #print('[BLUETOOTH_INFO] Received: ' + dataRcvBytes.rstrip())
            print('[BLUETOOTH_INFO] Received: ' + dataRcvBytes.decode('utf-8'))
            return dataRcvBytes.decode('utf-8')

        except BluetoothError as e:
            print('[BLUETOOTH_ERROR] Receiving Error: %s' % str(e))
            if ('Connection reset by peer' in str(e)):
                print('[BLUETOOTH_ERROR] Connection dropped. Attempting to re-establish.')
                self.disconnect()
                self.connect()
