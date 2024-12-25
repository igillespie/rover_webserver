from flask import Flask, session, redirect, url_for, jsonify, request, render_template, Response 
from flask_socketio import SocketIO 
from flask_restful import Resource, Api, reqparse, abort
from flask_cors import CORS  # Add this import
import other_files
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import robot_sensors as sensors
import threading
from web_publishers.joystick_publisher import JoystickPublisher
# import time

data_sensors = None

def make_sensors():
    print("make_sensors called")
    global data_sensors
    data_sensors = sensors.robot_data.RobotSensors()
    print("Data sensors should be init'ed")
    data_sensors.spin()
               
def create_app():
    global data_sensors

    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'your_secret_key'

    

    api = Api(app)
    
    data_thread = threading.Thread(target=make_sensors)
    data_thread.start()

    # ROS2 Node setup
    joystick_publisher = JoystickPublisher()
    
    socketio = SocketIO(app)  # Allow all origins

    # Define routes
    @app.route("/")
    def home():
        try:
            return render_template("home.html")
        except Exception as e:
            return f"Error loading home page: {e}", 500
        
    @socketio.on('command')
    def handle_command(data):
        try:
            pass
            # for command in data:
            #     func = "other_files." + str(command) + "()"  # Convert the command to a function call
            #     exec(func)
        except Exception as e:
            print(f"Error handling command: {e}")

    # Emit frequent status updates (e.g., pan/tilt angles)
    def emmiter():
        global data_sensors
        while True:
            try:
                velocity = data_sensors.drive_state.get_velocity()
                print(f"Velocity: {velocity}")
                socketio.emit('status_update', {"speed_left": velocity[0], "speed_right": velocity[3], 
                                                "angleLfront": other_files.angleLfront, "angleRfront": other_files.angleRfront,
                                                "angleLback": other_files.angleLback, "angleRback": other_files.angleRback,
                                                "joint1": other_files.joint1, "joint2": other_files.joint2, 
                                                "joint0": other_files.joint0})
                socketio.sleep(0.1)
            except Exception as e:
                # print(f"Error emitting status update: {e}")
                pass

    @socketio.on('connect')
    def handle_connect():
        print("Client connected")
        socketio.start_background_task(emmiter)

    @socketio.on('disconnect')
    def handle_disconnect():
        print("Client disconnected")

    @socketio.on_error_default
    def default_error_handler(e):
        print(f'An error occurred: {e}')

    return app, socketio

def start_webserver():
    app, socketio = create_app()
    socketio.run(app, host="0.0.0.0", port=4999)

if __name__ == "__main__":
    start_webserver()