from flask import Flask, render_template, request
from twilio.rest import Client
import serial
import threading
import time

#twilio stuff
account_sid = "ACac9b029c11f2cb1e9b98c50529464191"
auth_token = "a54cbe266686e1c4f9a8d6b550470dd0"
client = Client(account_sid, auth_token) #creates client
twilio_phone_number = "+18447342936" #phone number to send the message
recipient_phone_number = "+19167199871" #phone number to send the message to
message_armed = "System armed" #message body to send
message_disarm_now = "Motion Detected, http://192.168.1.21:5000 DISARM NOW"
message_disarm = "System Disarmed, you may now arm the system again http://192.168.1.21:5000"

#flask server
app = Flask(__name__)

#serial stuff
ser = serial.Serial('/dev/ttyS0', baudrate=38400, timeout=1)

#password to disarm
correct_disarm_password = "1234"
#password to arm
correct_arm_password = "4321"

def send_arm_message():
    message = client.messages.create(
        body=message_armed,
        from_=twilio_phone_number,
        to=recipient_phone_number
    )
    print("armed message sent")

def send_disarm_message():
    message = client.messages.create(
        body=message_disarm,
        from_=twilio_phone_number,
        to=recipient_phone_number
    )
    print("disarmed message sent")

def send_disarm_now_message():
    message = client.messages.create(
        body=message_disarm_now,
        from_=twilio_phone_number,
        to=recipient_phone_number
    )
    print("motion detected message sent")

def process_uart_data(data):

    if data == 42:
        send_arm_message()
        
        ser.close()
        ser.open()
        
    elif data == 41:
        send_disarm_now_message()
        
        ser.close()
        ser.open()
            
    else:
        print("Signal Error, Rereading")
        ser.close()
        ser.open()

def motion_thread():

    while True:
        print("uart is running")
        time.sleep(1)
        data = ser.read(4)  # Read 4 bytes (adjust if needed)
        if data:
            value_received = int.from_bytes(data, byteorder='big')  # Convert bytes to integer
            print("Received data:", value_received)
            process_uart_data(value_received)
            
@app.route('/')
def index():
    return render_template('index.html')

#sends signal to nucleo when password is entered
@app.route('/disarm', methods=['POST'])
def disarm():
    signal_to_send = b'9'
    password = request.form['password']
    if password == correct_disarm_password:
        send_disarm_message()
        ser.write(signal_to_send)
        print("disarm signal send")
        return "Disarmed"
    else:
        return "Incorrect Password"

#sends signal to nucleo when password is entered
@app.route('/arm', methods=['POST'])
def arm():
    signal_to_send = b'8'
    password = request.form['password']
    if password == correct_arm_password:
        send_arm_message()
        ser.write(signal_to_send)
        return "System Armed"
    else:
        return "Incorrect Password"

def flask_thread():
    if __name__ == '__main__':
        app.run(host='0.0.0.0')

#threading stuff
thread1 = threading.Thread(target=flask_thread)
thread2 = threading.Thread(target=motion_thread)

thread1.start()
thread2.start()