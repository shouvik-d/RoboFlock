from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
import time
import pynmea2
import meshtastic.serial_interface
#from meshtastic import portnums_pb2 as portnums
from pubsub import pub
from datetime import datetime
import struct


app = Flask(__name__)
socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")

PORT = "COM3"
interface = meshtastic.serial_interface.SerialInterface(devPath=PORT, noNodes=True)


@app.route("/")
def index():
    return render_template("index.html")

# if tom clicks a button, which triggers a send_message entry,
#  we send on the message to the other meshtastics
# this socketio.on receives the data send by socket.emit
#  with corresponding label/"event_name"
@socketio.on("send_message")
def handle_send_message(data):
    text = data.get("text", "")

    if text:
        print("[BUTTON]", text)
        # SEND OUT message to jetson and beacon
        #  over channel 2, this will be displayed on both screens.
        interface.sendText(text, channelIndex=2)


#defining a global boolean value to know when we receive first valid gps positions
first_valid_coordinates = False

# okay, think i need a port number to uniquely id gps receipt and other text message receipt.
def on_receive(packet, interface=None):
    global first_valid_coordinates
    print("in on receive")
    print(first_valid_coordinates)

    if packet['decoded']['portnum'] == "PRIVATE_APP":
        print(packet['decoded']['portnum'])
        print(packet.get('channel'))
        sentence = packet['decoded']['payload']
        print(sentence)

        if first_valid_coordinates:
            # unpacks our gps data in four byte little endian, matches the pack in the jetsonScript
            lat1, lon1 = struct.unpack("<iiii", sentence)
            #lat, lon = parse_with_lib(sentence)
            lat1 = lat1 / 1e7
            lon1 = lon1 / 1e7
            lat2 = lat2 / 1e7
            lon2 = lon2 / 1e7

            # send gps locations to gps_update function in the html file in order to visualize positions
            print("GPS RECEIVED ", lat1, lon1, lat2, lon2)
            socketio.emit("gps_update", {"lat1": lat1, "lon1": lon1, "lat2": lat2, "lon2": lon2})
        else:
            print("find first valid coordinates")
            lat1, lon1, lat2, lon2 = struct.unpack("<iiii", sentence)
            print(lat1, lon1)
            
            lat1 = lat1 / 1e7
            lon1 = lon1 / 1e7
            lat2 = lat2 / 1e7
            lon2 = lon2 / 1e7

            # just realizing, kinda sadly tbh, that the longitude coordinates can be negative, for ub in particular around -78
            #  so maybe just make this an and as the only time we expect both to be 0 is right after starting the jetson script
            if ((lat1 <= 10.0 and lon1 <= 10.0) or (lat2 <= 10.0 and lon2 <= 10.0)):
                #do nothing, don't send coordinates
                print("lat1 and lon1 <= 10.0 or lat2 and lon2 <= 10.0") 
                print(lat1, lon1, lat2, lon2)
                None
            else:
                first_valid_coordinates = True
                print("init markers  , first valid coordinates received")
                # changed the even from "init_gps_coordinates" to "gps_update"
                socketio.emit("gps_update", {"lat1": lat1, "lon1": lon1, "lat2": lat2, "lon2": lon2})
    else:
        # ain't great if we aren't gettting the position packets from the jetson, but do nothing for now
        None



pub.subscribe(on_receive, "meshtastic.receive")


def heartbeat():
    while True:
        time.sleep(5)

if __name__ == "__main__":
    import threading
    threading.Thread(target=heartbeat, daemon=True).start()
    socketio.run(app, host="127.0.0.1", port=5000, use_reloader=False, allow_unsafe_werkzeug=True)