document.addEventListener("DOMContentLoaded", function() {
    

    var Joy1 = new JoyStick('joyDiv', 
        {
          
            internalFillColor: '#808080', // Dark gray color for the stick
            internalStrokeColor: '#A9A9A9', // Light gray border for the stick
            externalStrokeColor: '#D3D3D3', // Lighter gray border for the external circle
            autoReturnToCenter: true,    // Enable auto-return to the center when released
        }, 
        function(stickData) {

            console.log("raw joy data " + JSON.stringify(stickData))
            let joyX = stickData.x / 100 * -1.0;  // Normalize the x-axis (-1 to 1)
            let joyY = stickData.y / 100;  // Normalize the y-axis (-1 to 1)
        
            // Apply a dead zone to prevent small movements from being registered
            const DEAD_ZONE = 0.05; // Adjust sensitivity (10% dead zone)
        
            joyX = Math.abs(joyX) > DEAD_ZONE ? joyX : 0;
            joyY = Math.abs(joyY) > DEAD_ZONE ? joyY : 0;
        
            // Optionally scale the values to make the joystick less sensitive
            // const SCALE_FACTOR = 0.3; // Reduce to make it LESS sensitive
            // joyX *= SCALE_FACTOR;
            // joyY *= SCALE_FACTOR;

            //console.log("Processed joystick data: ", { x: joyX, y: joyY });
        
            // Emit the normalized and adjusted joystick data
        
        });
});
