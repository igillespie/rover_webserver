document.addEventListener("DOMContentLoaded", function() {
    const socket = io();

    socket.on('connect', function() {
        console.log('Connected to server');
    });

    var Joy1 = new JoyStick('joyDiv', {
        internalFillColor: '#808080', // Dark gray color for the stick
        internalStrokeColor: '#A9A9A9', // Light gray border for the stick
        externalStrokeColor: '#D3D3D3',
    }, function(stickData) {

        //console.log("raw joy data " + JSON.stringify(stickData))
        let joyX = stickData.x / 100 * -1.0;  // Normalize the x-axis (-1 to 1)
        let joyY = stickData.y / 100;  // Normalize the y-axis (-1 to 1)
    
        // Apply a dead zone to prevent small movements from being registered
        const DEAD_ZONE = 0.05; // Adjust sensitivity (10% dead zone)
    
        joyX = Math.abs(joyX) > DEAD_ZONE ? joyX : 0;
        joyY = Math.abs(joyY) > DEAD_ZONE ? joyY : 0;
    
        //Optionally scale the values to make the joystick less sensitive
        const SCALE_FACTOR = 0.3; // Reduce to make it LESS sensitive
        joyX *= SCALE_FACTOR;
        joyY *= SCALE_FACTOR;

        
    
        // Emit the normalized and adjusted joystick data
        //Don't send while testing, just log
        //console.log("Processed joystick data: ", { x: joyX, y: joyY });
        socket.emit('command',  {"type": "joyStick", "info": {"x": joyX, "y": joyY}});
    });

    socket.on('slow_update', function(data) {
        updateBatterStatus(data);
        updateCoreStatus(data);
        updateOdometry(data);
    });
    socket.on('update', function(data) {

        const maxVelocity = 3; // unknown unit
        // const maxAngle = 1 // unknown unit
        document.documentElement.style.setProperty('--left-speed', `${Math.round(data.left_front*(-100/maxVelocity))}px`);
        document.documentElement.style.setProperty('--right-speed', `${Math.round(data.right_front*(-100/maxVelocity))}px`);
        document.getElementById('left-speeds').textContent = Math.round(data.left_front*(100/maxVelocity));
        document.getElementById('right-speeds').textContent = Math.round(data.right_front*(100/maxVelocity));
        color_wheel = ((maxVelocity - Math.abs(data.left_front))/maxVelocity)*120;
        document.documentElement.style.setProperty('--wheel-color', `hsl(${color_wheel}, 100%, 50%)`);

        data.angleLfront /= Math.PI;
        data.angleLfront *= 180;
        data.angleRfront /= Math.PI;
        data.angleRfront *= 180;
        data.angleLback /= Math.PI;
        data.angleLback *= 180;
        data.angleRback /= Math.PI;
        data.angleRback *= 180; 
    
        document.documentElement.style.setProperty('--angle-FL', `${data.angleLfront}deg`);
        document.documentElement.style.setProperty('--angle-FR', `${data.angleRfront}deg`);
        document.documentElement.style.setProperty('--angle-BL', `${data.angleLback}deg`);
        document.documentElement.style.setProperty('--angle-BR', `${data.angleRback}deg`);
        document.getElementById('FL-angle').textContent = Math.round(data.angleLfront);
        document.getElementById('FR-angle').textContent = Math.round(data.angleRfront);
        document.getElementById('BL-angle').textContent = Math.round(data.angleLback);
        document.getElementById('BR-angle').textContent = Math.round(data.angleRback);
    });

    function updateCoreStatus(data) {

        if (data.core_status) {
            // console.log("data " + JSON.stringify(data))
            let voltage = data.core_status.voltage
            let cpu_temp = data.core_status.cpu_temp

            let batteryVoltageElement = document.getElementById('core-status-voltage');
            let cpuElement = document.getElementById('core-status-cpu-temp');

            // Check if the elements exist and update their content
            if (batteryVoltageElement && cpuElement) {
                // Ensure the data is valid and update the text content
                if (typeof voltage === 'number' && !isNaN(voltage)) {
                    batteryVoltageElement.textContent = `Voltage: ${voltage.toFixed(2)} V`;
                } else {
                    batteryVoltageElement.textContent = 'Voltage: --';
                }

                if (typeof cpu_temp === 'number' && !isNaN(cpu_temp)) {
                    cpuElement.textContent = `CPU Temp: ${cpu_temp.toFixed(2)}˚`;
                } else {
                    cpuElement.textContent = 'CPU Temp: --˚';
                }

               
            } else {
                console.error('Core status elements not found in the DOM.');
            }  
        }
    }

    function updateBatterStatus(data) {
        if (data.battery_state) {
            // console.log('Battery State:', data.battery_state);
            // Example data for voltage and amps
            let voltage = data.battery_state.voltage; // Voltage in volts
            let amps = data.battery_state.current;     // Current in amperes

            // Get the HTML elements by their IDs
            let batteryVoltageElement = document.getElementById('battery-voltage');
            let batteryAmpsElement = document.getElementById('battery-amps');

            // Check if the elements exist and update their content
            if (batteryVoltageElement && batteryAmpsElement) {
                // Ensure the data is valid and update the text content
                if (typeof voltage === 'number' && !isNaN(voltage)) {
                    batteryVoltageElement.textContent = `Voltage: ${voltage.toFixed(2)} V`;
                } else {
                    batteryVoltageElement.textContent = 'Voltage: --';
                }

                if (typeof amps === 'number' && !isNaN(amps)) {
                    batteryAmpsElement.textContent = `Amps: ${amps.toFixed(2)} A`;
                } else {
                    batteryAmpsElement.textContent = 'Amps: --';
                }
            } else {
                console.error('Battery elements not found in the DOM.');
            }   
        } 
    }

    function updateOdometry(data) {
        if (data.distance) {
            // console.log('Battery State:', data.battery_state);
            // Example data for voltage and amps
            let distance = data.distance.distance; // Voltage in volts

            // Get the HTML elements by their IDs
            let odomDistance = document.getElementById('odom-distance');

            // Check if the elements exist and update their content
            if (odomDistance) {
                // Ensure the data is valid and update the text content
                if (typeof distance === 'number' && !isNaN(distance)) {
                    odomDistance.textContent = `Disance: ${distance.toFixed(2)} M`;
                } else {
                    odomDistance.textContent = 'Disance: -- M';
                }
            } else {
                console.error('Battery elements not found in the DOM.');
            }   
        } 
    }

    socket.on('disconnect', function() {
        console.log('Disconnected from server');
    });

    document.addEventListener('keydown', (event) => {
        if (event.key == 'ArrowUp') {
            socket.emit('command', ["forward"]);
        }
        if (event.key == 'ArrowDown') {
            socket.emit('command', ["backward"]);
        }
        if (event.key == 'ArrowLeft') {
            socket.emit('command', ["left"]);
        }
        if (event.key == 'ArrowRight') {
            socket.emit('command', ["right"]);
        }
        if (event.key == 'b') {
            socket.emit('command', ["stop"]);
        }
        if (event.key == '[') {
            socket.emit('command', ["spin_left"]);
        }
        if (event.key == ']') {
            socket.emit('command', ["spin_right"]);
        }
        if (event.key == 'w') {
            socket.emit('command', ["joint1_down"]);
        }
        if (event.key == 's') {
            socket.emit('command', ["joint1_up"]);
        }
        if (event.key == 'a') {
            socket.emit('command', ["joint0_left"]);
        }
        if (event.key == 'd') {
            socket.emit('command', ["joint0_right"]);
        }
        if (event.key == 'i') {
            socket.emit('command', ["joint2_down"]);
        }
        if (event.key == 'k') {
            socket.emit('command', ["joint2_up"]);
        };
    });

    document.addEventListener('keyup', (event) => {
        if (event.key == '[' || event.key == ']') {
            socket.emit('command', ["stop_spin", "stop"]);
        }
        if (event.key == "ArrowUp" || event.key == "ArrowDown" || event.key == "ArrowLeft" || event.key == "ArrowRight") {
            socket.emit('command', ["stop"]);
        }});

});