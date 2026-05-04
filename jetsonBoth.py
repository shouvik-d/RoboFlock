import serial
import time
import pynmea2
import meshtastic.serial_interface
from meshtastic import portnums_pb2 as portnums
from pubsub import pub
#from datetime import datetime
import struct


# nOTES, i have to configure the meshtastics appropriately for this all to work
#  channels 1, 2, and 3 are being used, with all being private. however, anything
#  sent to ch 1,2 will be displayed, nothing sent on ch 3 will be displayed
#  as it's position data only. (in the tom script we have to handle this data 
#  and map it).


# i'll have to figure out how to automatically determine the ports for the
#  gps and meshtastic which may not be fixed, the UART port for the hc-12
#  should be fixed though.
#gps_port = "/dev/ttyUSB0" # this is for the gps/neo m8p connected via usb to the Jetson
gps_port = "COM19" # this is for the gps/neo m8p connected via usb to the Jetson
gps_baud_rate = 9600     #NOTE JETSON USES 115200 BY DEFAULT...

mesh_port = "COM5" # just a guess rn for the meshtastic usb port
mesh_baud_rate = 9600

#hc12_port = "/dev/ttyTHS0" # this is for the hc12 connected via uart1 to the Jetson
hc12_port = "COM8"
hc12_baud_rate = 115200 

# have to determine the COM port automatically eventually using ports = serial.tools.list_ports.comports()
#  then we search for the associated description value "USB-Enhanced-SERIAL CH9102" for the meshtastic.
#  or can use the VID:PID (VID:PID=1A86:55D4) and SER combination (allows us to id exact expected meshtastic device)
#  (SER=5887017851 for the 3480/Robot mesthtastic, and SER=5887019247 for e074/Tom's meshtastic)
PORT = "COM5"
interface = meshtastic.serial_interface.SerialInterface(devPath=PORT, noNodes=True)

# defining the gps coordinates as global for now (prob better to pass in functions when needed but... eh)
latRobot = 0
lonRobot = 0
latBeacon = 0
lonBeacon = 0

# have to update these with the node ids of each meshtastic when the time comes
tomNodeID = 3145785460
beaconNodeID = 1819531008
robotNodeID = 1819554944

# name - node #    , user id  , user  
# d700 - 1819531008, !6c73d700, beacon
# 3480 - 1819554944, !6c743480, robot
# e074 - 3145785460, !bb80e074, tom

def getRobotGps():
    try:
        # initialize serial connection
        ser = serial.Serial(gps_port, gps_baud_rate, timeout=1)

        # we never get in here during normal operation, testing the code above to see if the .in_waiting is causing issues
        # try to get nmea sentence from usb serial port.
        # , I just changed it to > or EQUAL to zero (making it pointless tbh) but it works now
        if ser.in_waiting >= 0:
            global latRobot
            global lonRobot
    
            # Read the raw bytes and decode
            # line should be our raw nmea text sentence, we need to parse it with pynmea2 next to get the robot's lat and lon.
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                latRobot, lonRobot = parse_with_lib(line)
                if latRobot == None or lonRobot == None:
                    return
                else:
                    None
        time.sleep(0.01)
    # handle exceptions
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


# get the Beacon's gps coordinates via the hc12 connected to jetson's uart1 port
def getBeaconGps():
    try:
        # initialize serial connection
        ser = serial.Serial(hc12_port, hc12_baud_rate, timeout=1)

        # we never get in here during normal operation, testing the code above to see if the .in_waiting is causing issues
        # try to get nmea sentence from usb serial port.
        # , I just changed it to > or EQUAL to zero (making it pointless tbh) but it works now
        if ser.in_waiting >= 0:
            global latBeacon
            global lonBeacon
    
            # Read the raw bytes and decode
            # line should be our raw nmea text sentence, we need to parse it with pynmea2 next to get the robot's lat and lon.
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                latBeacon, lonBeacon = parse_with_lib(line)
                if latBeacon == None or lonBeacon == None:
                    return
                else:
                    None
        time.sleep(0.01)
    # handle exceptions
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


# parse the sentence with pynmea2 library to get lat and lon values,
#  note it's currently set to parse "GGA" sentences only.
# also note, the msg.latitude and msg.longitude return values as floats.
def parse_with_lib(sentence):
    try:
        msg = pynmea2.parse(sentence)
        if msg.sentence_type == "GGA":
            # using hdop to kick out inaccurate readings
            #  hopefully this handles the noisy data values
            #  10 is "moderate" according to wikipedia

            # changed this to 2.0 because the values we're gettting ucenter are around 1.1,
            #  and seem to max out at 5.0 according to the software, positions start to
            #  get drifty around 1.3 so may be best to set even lower after testing.
            if float(msg.horizontal_dil) > 2.0:
                return None, None
            else:
                return msg.latitude, msg.longitude
        else:                                       # added this else to handle the gntxt message that was messing things up
            return None, None
    except pynmea2.ParseError:
        print("[ERROR] Invalid NMEA:", sentence)
        return None, None


def sendPositions(lat1, lon1, lat2, lon2, destID):
    # seeing maybe conflicting info about the lat lon data type, docs say float
    # others say they need to be ints normed by 1e7 for example, lat * 1e7.
    #  i think the latter may be for spoofing position packets using Data packets
    #  which may be more applicable here (as the position packets are drawn to
    #  map within the meshtastic app which i don't care for but we'll see).
    try:
        robotLat = int(lat1 * 1e7)
        robotLon = int(lon1 * 1e7)
        beaconLat = int(lat2 * 1e7)
        beaconLon = int(lon2 * 1e7)

        # pack our data to send more efficiently to tom's tablet
        # "<iiii" represents four byte little endian encoding
        data = struct.pack("<iiii", robotLat, robotLon, beaconLat, beaconLon)
        print(f"DATA: {data}")
        # changed channel index to 2 from 3 (not sure if this is correct yet) (I THINK IT's correct now after consulting with docs)
        interface.sendData(data, destinationId=destID, channelIndex=2, portNum=portnums.PortNum.CUSTOM_APP, wantAck=True)
        #print(f"GPS SENT TO TOM: {robotLat}, {robotLon}, {beaconLat}, {beaconLon}")
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    except TypeError:
        print(f"type error in sendPositions")
        return



# so for tom, messages are sent by clicking the button on the webpage/map page,
#  when he clicks it calls a javascript function that calls a socket.emit function with a text message ("come to base", "return robot", etc.) 
#  which sends the text data to the "send_message" in the receiving script


# this is where we'll eventually handle different actions based on message contents
#  ex. stop robot movement if tom sends "stop robot", or begin return to base if tom sends "return to base"
def on_receive(packet, interface=None):
    # all i want is to display everything received by the jetson on the screen rn,
    #  eventually handle different actions based on received message contents
    print("something received")
    None

pub.subscribe(on_receive, "meshtastic.receive.text")



def heartbeat():
    while True:
        print("in heartbeat")
        getRobotGps()
        time.sleep(0.5)
        getBeaconGps()
        time.sleep(0.5)
        sendPositions(latRobot, lonRobot, latBeacon, lonBeacon, tomNodeID)
        time.sleep(4)

if __name__ == "__main__":
    import threading
    threading.Thread(target=heartbeat, daemon=True).start()



while True:
    None
    """
    getRobotGps()
    # added this to deal with incomplete nmea sentences being received by the beacon, 
    #  seems not enough time to decode the sentence in the beacon func if we don't delay.
    time.sleep(0.5)
    getBeaconGps()
    time.sleep(0.5) # changed this delay and above to 0.5 seems to have solved my type errors in sendPositions
    """