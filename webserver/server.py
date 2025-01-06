from flask import Flask, session, redirect, url_for, jsonify, request, render_template, Response 
from flask_socketio import SocketIO 
from flask_restful import Resource, Api, reqparse, abort
from flask_cors import CORS  # Add this import
# import other_files
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import robot_sensors as sensors
import threading
from ros_controller.ros_controller import ROSController
# import time

app = Flask(__name__, static_folder="static")

ros_controller = None
lock = threading.Lock()  # Ensure thread-safe access to cached images



def create_app():

   
    app.config['SECRET_KEY'] = 'your_secret_key'
    # app.config['DEBUG'] = True
    api = Api(app)
 
    socketio = SocketIO(app, cors_allowed_origins="*", logger=False, engineio_logger=False)

    # Initialize ROS controller
    global ros_controller
    ros_controller = ROSController()

    # Start ROS publisher and subscriber nodes
    ros_controller.start_publisher_nodes()
    ros_controller.start_subscriber_nodes()
    ros_controller.spin_all_nodes()

    # Define routes
    @app.route("/")
    def home():
        try:
            return render_template("home.html")
        except Exception as e:
            return f"Error loading home page: {e}", 500

    @app.route('/video_feed')
    def video_feed():
        """
        Flask route to provide an MJPEG stream of the camera frames.
        """
        def generate():
            while True:
                try:
                    # Access the MainCameraSubscriber node
                    # all = ros_controller.get_all_nodes()
                    # print(f"all nodes {all}")
                    main_camera_node = ros_controller.get_node("main_camera_subscriber")
                    if not main_camera_node:
                        print("MainCameraSubscriber node not found.")
                        break

                    #with lock:
                        # Get the latest cached image
                    cached_image = main_camera_node.get_cached_image()

                    if cached_image:
                        # Log the size of the cached image
                        #print(f"Cached image size: {len(cached_image.data)} bytes")
                        #print(f"Image timestamp: {cached_image.header.stamp.sec}.{cached_image.header.stamp.nanosec}")
                        
                        # Yield the JPEG image as an MJPEG frame
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + cached_image.data + b'\r\n')
                    else:
                        # Log that no image is available
                        #print("No cached image available.")
                        
                        # Yield a blank frame
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + b'' + b'\r\n')
                except Exception as e:
                    print(f"Error in video_feed: {e}")
                    break

        print("Starting MJPEG video stream...")
        response = Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        print("MJPEG video stream started.")
        return response
        
    @socketio.on('command')
    def handle_command(data):
        try:
            #print(f"data received {data}")
            
            if data.get('type') == 'joyStick':
                info = data.get('info', {})  # Safely get 'info' dictionary
                x = info.get('x')  # Get 'x' value from 'info'
                y = info.get('y')  # Get 'y' value from 'info'
                yaw = info.get('yaw')
                # Ensure joy_node is retrieved
                joy_node = ros_controller.get_node("joystick_publisher") if ros_controller else None

                if joy_node is not None:
                    if x is not None and y is not None and yaw is not None:  # Proceed if both 'x' and 'y' and 'yaw' are present
                        joy_node.publish_joystick_data(x, y, yaw)
                    else:  # Incomplete data: stop the robot
                        joy_node.publish_joystick_data(0, 0, 0)
                        print("Incomplete joystick info received. Publish stop")
                else:
                    print("Joystick publisher node is not available.")
            else:
                print("Unhandled command type received.")
        except Exception as e:
            print(f"Error handling command: {e}")

    # Emit frequent status updates (e.g., pan/tilt angles)
    def emitter():
        last_emitted_data = None  # Cache to store the last emitted data

        while True:
            try:
                # Retrieve the latest drive_state data as-is
                drive_state_data = ros_controller.get_node("drive_state_subscriber").drive_state.get_velocity()
                corner_state_data = ros_controller.get_node("corner_state_subscriber").corner_state
                distance_node = ros_controller.get_node("distance")

                # Build the data dictionary dynamically
                data_to_emit = {}

                if drive_state_data:
                    #print(f"drive_state_data {drive_state_data}")
                    data_to_emit["right_front"] = drive_state_data["drive_right_front"]
                    data_to_emit["left_front"] = drive_state_data["drive_left_front"]

                if corner_state_data:
                    data_to_emit.update({
                        "angleRback": corner_state_data[0]["position"],
                        "angleRfront": corner_state_data[1]["position"],
                        "angleLfront": corner_state_data[2]["position"],
                        "angleLback": corner_state_data[3]["position"]
                    })

                if distance_node:
                    distance = {
                        "distance": distance_node.distance
                    }
                    data_to_emit["distance"] = distance
                    
                else:
                    print("failed to find distance_node")

                # Emit only if there's data to send
                if data_to_emit and data_to_emit != last_emitted_data:
                    socketio.emit('update', data_to_emit)
                    last_emitted_data = data_to_emit

                
            except Exception as e:
                print(f"Error emitting data: {e}")
                pass

            # Add a small delay to prevent high CPU usage
            socketio.sleep(0.1)

        

    def slow_emitter(): 
        while True:
            try:
                
                battery_state_node = ros_controller.get_node("battery_state_subscriber")
                core_status_node = ros_controller.get_node("core_status")
                
                # Build the data dictionary dynamically
                data_to_emit = {}

                if battery_state_node:
                    # Extract battery state data (voltage and current)
                    battery_state = {
                        "voltage": battery_state_node.battery_state.get("voltage", None),
                        "current": battery_state_node.battery_state.get("current", None)
                    }
                    data_to_emit["battery_state"] = battery_state
                else:
                    print("failed to find battery_state_node")

                if core_status_node:
                    core_status = {
                        "voltage": core_status_node.core_status.get("voltage", None),
                        "cpu_temp": core_status_node.core_status.get("cpu_temperature", None)
                    }
                    data_to_emit["core_status"] = core_status
                    #print(f"core status: {data_to_emit['core_status']}")
                else:
                    print("failed to find battery_state_node")

                # Emit only if there's data to send
                if data_to_emit:
                    socketio.emit('slow_update', data_to_emit)
 
            except Exception as e:
                print(f"Error emitting data: {e}")
                pass
             # Add a small delay to prevent high CPU usage
            socketio.sleep(1.0)
       

    @socketio.on('connect')
    def handle_connect():
        print("Client connected")
        socketio.start_background_task(emitter)
        socketio.start_background_task(slow_emitter)

    @socketio.on('disconnect')
    def handle_disconnect(environ=None):
        print("Client disconnected")
        

    @socketio.on_error_default
    def default_error_handler(e):
        print(f'An error occurred: {e}')

    return app, socketio, ros_controller

import eventlet
import eventlet.wsgi

def start_webserver():
    app, socketio, ros_controller = create_app()
    try:
        eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 8080)), app)
    except KeyboardInterrupt:
        pass
    finally:
        ros_controller.shutdown()

if __name__ == "__main__":
    start_webserver()