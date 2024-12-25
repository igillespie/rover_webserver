document.addEventListener("DOMContentLoaded", function() {
    const socket = io();

    socket.on('connect', function() {
        console.log('Connected to server');
    });

    var Joy1 = new JoyStick('joyDiv', {}, function(stickData) {
        // joy1IinputPosX.value = stickData.xPosition;
        // joy1InputPosY.value = stickData.yPosition;
        // joy1Direzione.value = stickData.cardinalDirection;
        // joy1X.value = stickData.x;
        // joy1Y.value = stickData.y;
        console.log(stickData);
        socket.emit('command',  {"type": "joyStick", "info": {"x": stickData.x, "": stickData.y}});
    });

    socket.on('status_update', function(data) {
        const maxVelocity = 3; // unknown unit
        const minVelocity = -3; // unknown unit
        document.documentElement.style.setProperty('--left-speed', `${Math.round(data.speed_left*(-100/maxVelocity))}px`);
        document.documentElement.style.setProperty('--right-speed', `${Math.round(data.speed_right*(100/maxVelocity))}px`);
        document.getElementById('left-speeds').textContent = Math.round(data.speed_left*(100/maxVelocity));
        document.getElementById('right-speeds').textContent = Math.round(data.speed_right*(-100/maxVelocity));
        document.documentElement.style.setProperty('--angle-FL', `${data.angleLfront}deg`);
        document.documentElement.style.setProperty('--angle-FR', `${data.angleRfront}deg`);
        document.documentElement.style.setProperty('--angle-BL', `${data.angleLback}deg`);
        document.documentElement.style.setProperty('--angle-BR', `${data.angleRback}deg`);
        color_wheel = ((maxVelocity - Math.abs(data.speed_left))/maxVelocity)*120;
        document.documentElement.style.setProperty('--wheel-color', `hsl(${color_wheel}, 100%, 50%)`);
        document.documentElement.style.setProperty('--joint1', `${data.joint1}deg`);
        document.documentElement.style.setProperty('--joint2', `${data.joint2}deg`);
        document.documentElement.style.setProperty('--joint0', `${data.joint0}deg`);
        document.getElementById('FL-angle').textContent = data.angleLfront;
        document.getElementById('FR-angle').textContent = data.angleRfront;
        document.getElementById('BL-angle').textContent = data.angleLback;
        document.getElementById('BR-angle').textContent = data.angleRback;
        document.getElementById('angle-1').textContent = data.joint1;
        document.getElementById('angle-2').textContent = data.joint2;
        document.getElementById('angle-0').textContent = data.joint0;
    });

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