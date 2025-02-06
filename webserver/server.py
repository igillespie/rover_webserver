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
# import base64
# import zlib
import time
import numpy as np
from PIL import Image
from skimage.metrics import structural_similarity as ssim  # For SSIM comparison
import io
from datetime import datetime


app = Flask(__name__, static_folder="static")

ros_controller = None
lock = threading.Lock()  # Ensure thread-safe access to cached images

#very basic pw protection
USERS = {"your_username": "your_password"}

previous_image = None
fps = 20  # Frames per second for image emiitter

# is_emitting = False
# previous_image_emit_count = 0

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
        if is_logged_in():
            try:
                return render_template("home.html")
            except Exception as e:
                return f"Error loading home page: {e}", 500
        else:
            try:
                return redirect("/login")
            except Exception as e:
                return f"Error loading login page: {e}", 500

    
    def is_logged_in():
        return session.get("logged_in", False)
        # return False

    @app.route("/login", methods=["POST", "GET"])
    def login():
        if request.method == "POST":
            username = request.form["username"]
            password = request.form["password"]
            if username in USERS and USERS[username] == password:
                session["logged_in"] = True
                return redirect("/")
            return render_template("login.html", error="Invalid username or password")
        else:
            return render_template("login.html")

    @app.route('/video_feed')
    def video_feed():
        def generate():
            prev_time = time.time()
            target_fps = 10  # Limit to 10 FPS
            frame_interval = 1.0 / target_fps
            main_camera_node = ros_controller.get_node("main_camera_subscriber")
            while True:
                current_time = time.time()
                elapsed_time = current_time - prev_time

                if elapsed_time >= frame_interval:
                    # Fetch and yield the frame
                    cached_image = main_camera_node.get_cached_image()
                    if cached_image:
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' +
                            cached_image.data + b'\r\n')
                    prev_time = current_time
                else:
                    time.sleep(frame_interval - elapsed_time)
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
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
                joy_node = ros_controller.get_node("joystick_publisher")

                if joy_node is not None:
                    if x is not None and y is not None and yaw is not None:  # Proceed if both 'x' and 'y' and 'yaw' are present
                        joy_node.publish_joystick_data(x, y, yaw)
                    else:  # Incomplete data: stop the robot
                        joy_node.publish_joystick_data(0, 0, 0)
                        print("Incomplete joystick info received. Publish stop")
                else:
                    print("Joystick publisher node is not available.")
            elif data.get('type') == 'turn_in_place':
                movement_command_publisher = ros_controller.get_node("movement_command_publisher")
                action = data.get("action")
                # Map actions to angle and speed
                if action == "left":
                    movement_command_publisher.publish_turn_in_place_command(20, 90) 
                elif action == "right":
                    movement_command_publisher.publish_turn_in_place_command(-20, 90)  
                else:
                    print(f"Unknown action: {action}")

            elif data.get('type') == 'move_distance':
                movement_command_publisher = ros_controller.get_node("movement_command_publisher") if ros_controller else None
                distance = data.get("distance")
                if distance < -3.0 or distance > 3.0:
                    print(f"Error: Distance value {distance} meters is out of range (-3 to 3 meters). Command ignored.")
                    return

                # Map actions to angle and speed
                movement_command_publisher.publish_move_distance_command(distance)

            elif data.get('type') == 'mast_control':
                
                mast = ros_controller.get_node("mast") if ros_controller else None
                pan_angle = data.get("pan_angle")
                tilt_angle = data.get("tilt_angle")
                if pan_angle == 0 and tilt_angle == 0:
                    mast.set_mast_pan_angle(150.0, 150.0) # Center it
                else:
                    mast.set_mast_pan_tilt_angles_incremental(pan_angle, tilt_angle)

            elif data.get('type') == 'direct_pan_angle':
                mast = ros_controller.get_node("mast") if ros_controller else None
                
                # Convert pan_angle to float safely
                pan_angle = data.get("pan_angle")
                try:
                    pan_angle = float(pan_angle) if pan_angle is not None else None
                except ValueError:
                    pan_angle = None
                    print("Error: pan_angle is not a valid number.")
                
                if pan_angle is not None and mast is not None:
                    mast.set_mast_pan_angle(pan_angle, mast.current_state.tilt_position)
                else:
                    print("Error: Could not set pan angle. Check values and try again.")

            elif data.get('type') == 'capture_img':

                handle_image_capture()

            else:
                print(f"Unhandled command type received. Received={data.get('type')}")
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
                mast_node = ros_controller.get_node("mast")

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

                if mast_node:
                    mast = {
                        "pan": mast_node.current_state.pan_position,
                        "tilt": mast_node.current_state.tilt_position
                    }
                    data_to_emit["mast"] = mast

                # Emit only if there's data to send
                if data_to_emit and data_to_emit != last_emitted_data:
                    socketio.emit('update', data_to_emit)
                    last_emitted_data = data_to_emit

                
            except Exception as e:
                print(f"Error emitting data: {e}")
                pass

            # Add a small delay to prevent high CPU usage
            socketio.sleep(0.5)

    
    def image_emitter():
        global previous_image
        while True:
            try:
                # Get the cached image from the camera node
                camera_node = ros_controller.get_node("main_camera_subscriber")
                compressed_image = camera_node.get_cached_image()
                
                # Convert compressed image bytes to a PIL Image
                pil_image = Image.open(io.BytesIO(compressed_image.data))
                
                # Resize the image
                resized_image = pil_image.resize((320, 180))
                
                # Convert the resized image to a NumPy array
                current_image = np.array(resized_image)
                
                # If there's a previous image, compute SSIM
                if previous_image is not None:
                    similarity, _ = ssim(
                        previous_image, 
                        current_image, 
                        full=True, 
                        multichannel=True, 
                        channel_axis=-1
                    )
                    if similarity > 0.95:  # Skip sending if images are very similar
                        socketio.sleep(1 / fps)  # Maintain FPS rate
                        continue
                
                # Update the previous image
                previous_image = current_image
                
                # Convert resized image back to JPEG bytes
                # jpeg_buffer = io.BytesIO()
                # resized_image.save(jpeg_buffer, format="JPEG")
                # jpeg_bytes = jpeg_buffer.getvalue()
                jpeg_bytes = bytes(compressed_image.data)
                
                # Emit the JPEG bytes to the client
                socketio.emit('image_update', jpeg_bytes)

            except Exception as e:
                print(f"Error emitting data: {e}")
            
            # Maintain the desired frame rate
            socketio.sleep(1 / fps)
            
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
                        #"voltage": battery_state_node.battery_state.get("voltage", None), #this voltage isn't very accurate from the INA260 and no one knows why
                        "current": battery_state_node.battery_state.get("current", None)
                    }
                    data_to_emit["battery_state"] = battery_state
                else:
                    print("failed to find battery_state_node")

                if core_status_node:
                    core_status = {
                        "voltage": core_status_node.core_status.get("voltage", None),
                        "cpu_temp": core_status_node.core_status.get("cpu_temperature", None),
                        "uptime": core_status_node.core_status.get("uptime", None)
                    }
                    data_to_emit["core_status"] = core_status
                    #print(f"core status: {data_to_emit['core_status']}")
                else:
                    print("failed to find core_status_node")

                # Emit only if there's data to send
                if data_to_emit:
                    socketio.emit('slow_update', data_to_emit)
 
            except Exception as e:
                print(f"Error emitting data: {e}")
                pass
             # Add a small delay to prevent high CPU usage
            socketio.sleep(1.0)

    def handle_image_capture():
        # Ensure the target directory exists
        save_directory = "/home/pi/captured_images/"
        os.makedirs(save_directory, exist_ok=True)  # Creates the directory if it doesn't exist

        # Get the camera node and cached image
        camera_node = ros_controller.get_node("main_camera_subscriber")
        compressed_image = camera_node.get_cached_image()
        
        # Get the JPEG bytes
        data = compressed_image.data
        
        # Generate the timestamped file name
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_name = f"{save_directory}{timestamp}.jpeg"
        
        # Write the data to the JPEG file
        with open(file_name, "wb") as f:
            f.write(data)

        print(f"Image saved as '{file_name}'")

    @socketio.on('connect')
    def handle_connect():
        print("Client connected")
        socketio.start_background_task(emitter)
        socketio.start_background_task(slow_emitter)
        socketio.start_background_task(image_emitter)

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