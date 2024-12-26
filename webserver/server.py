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
from ros_controller.ros_controller import ROSController
# import time


               
def create_app():

    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'your_secret_key'
    api = Api(app)
 
    socketio = SocketIO(app)  # Allow all origins

    # Initialize ROS controller
    ros_controller = ROSController()

    # Start ROS publisher and subscriber nodes
    #ros_controller.start_publisher_nodes()
    ros_controller.start_subscriber_nodes()
     # Spin ROS nodes in a separate thread to avoid blocking Flask
    ros_thread = threading.Thread(target=ros_controller.spin_nodes, daemon=True)
    ros_thread.start()

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
        while True:
            try:
                # Retrieve the latest drive_state data as-is
                drive_state_data = ros_controller.get_node("drive_state_subscriber").drive_state.get_fronts()

                # Emit the JSON data directly via Socket.IO
                if drive_state_data:
                    #print(f"Drive State Data: {drive_state_data}")  # Log to console
                    socketio.emit('drive_state', {"right_front": drive_state_data["right_front"]["velocity"], "left_front": drive_state_data["left_front"]["velocity"]})

                # Add a small delay to prevent high CPU usage
                socketio.sleep(0.1)
            except Exception as e:
                print(f"Error in emmiter: {e}")
                break  # Stop the loop if a critical error occurs

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

    return app, socketio, ros_controller

def start_webserver():
    app, socketio, ros_controller = create_app()
    try:
        socketio.run(app, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        pass
    finally:
        ros_controller.shutdown()

if __name__ == "__main__":
    start_webserver()