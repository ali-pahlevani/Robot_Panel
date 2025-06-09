console.log('control.js loaded');

let fullLift, fullLower; // Exportable functions

document.addEventListener('DOMContentLoaded', function() {
  console.log('DOM loaded');
  console.log('roslib:', typeof ROSLIB);
  console.log('nipplejs:', typeof nipplejs);
  console.log('Joystick container:', document.getElementById('joystick-container'));

  // Dynamically determine WebSocket URL
  var wsHost = window.location.hostname || 'localhost';
  var wsPort = 9090; // Match rosbridge_websocket port
  var wsUrl = 'ws://' + wsHost + ':' + wsPort;

  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: wsUrl
  });

  try {
    var joystick = nipplejs.create({
      zone: document.getElementById('joystick-container'),
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: '#44b0e0',
      size: 200
    });
    console.log('Joystick created:', joystick);
  } catch (e) {
    console.error('Joystick error:', e);
  }

  var cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/tri_cycle_controller/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  var forksTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/forks_position_controller/commands',
    messageType: 'std_msgs/Float64MultiArray'
  });

  var maxLinearSpeed = 0.5; // m/s
  var maxAngularSpeed = 1.0; // rad/s
  var currentForkPosition = 0.0; // Track commanded fork position (0.0 or 0.03)
  var isForkMoving = false; // Prevent concurrent movements

  joystick.on('move', function(event, data) {
    var linear = data.vector.y * maxLinearSpeed;
    var angular = -data.vector.x * maxAngularSpeed;
    var twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
    cmdVelTopic.publish(twist);
  });

  joystick.on('end', function() {
    var twist = new ROSLIB.Message({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    });
    cmdVelTopic.publish(twist);
  });

  // Smooth transition for fork movement
  function moveForks(targetPosition) {
    if (isForkMoving) {
      console.log('Fork movement already in progress');
      return Promise.reject(new Error('Fork movement already in progress'));
    }
    isForkMoving = true;
    return new Promise((resolve, reject) => {
      const duration = 1000; // 1 second
      const rate = 10; // 10 Hz
      const steps = duration / (1000 / rate); // 10 steps
      const stepSize = (targetPosition - currentForkPosition) / steps;
      let currentStep = 0;

      const interval = setInterval(() => {
        if (currentStep >= steps) {
          clearInterval(interval);
          currentForkPosition = targetPosition;
          isForkMoving = false;
          // Publish final position
          const message = new ROSLIB.Message({
            data: [targetPosition, targetPosition]
          });
          forksTopic.publish(message);
          console.log(`Fork movement completed to ${targetPosition}`);
          resolve();
          return;
        }

        const interpolatedPosition = currentForkPosition + stepSize * (currentStep + 1);
        const message = new ROSLIB.Message({
          data: [interpolatedPosition, interpolatedPosition]
        });
        forksTopic.publish(message);
        console.log(`Publishing fork position: [${interpolatedPosition}, ${interpolatedPosition}]`);
        currentStep++;
      }, 1000 / rate);
    });
  }

  fullLift = async () => {
    if (currentForkPosition === 0.03) {
      console.log('Forks already at full lift position');
      return { success: true, message: 'Forks already lifted' };
    }
    try {
      await moveForks(0.03);
      return { success: true, message: 'Forks lifted' };
    } catch (error) {
      console.error('Error lifting forks:', error);
      isForkMoving = false;
      return { success: false, error: error.message };
    }
  };

  fullLower = async () => {
    if (currentForkPosition === 0.0) {
      console.log('Forks already at full lower position');
      return { success: true, message: 'Forks already lowered' };
    }
    try {
      await moveForks(0.0);
      return { success: true, message: 'Forks lowered' };
    } catch (error) {
      console.error('Error lowering forks:', error);
      isForkMoving = false;
      return { success: false, error: error.message };
    }
  };

  // Export functions for index.js
  window.control = window.control || {};
  window.control.fullLift = fullLift;
  window.control.fullLower = fullLower;

  ros.on('connection', function() {
    console.log('ROS connected');
    document.getElementById('joystick-container').classList.remove('joystick-disabled');
  });

  ros.on('close', function() {
    console.log('ROS disconnected');
    document.getElementById('joystick-container').classList.add('joystick-disabled');
    isForkMoving = false; // Reset on disconnect
  });

  ros.on('error', function(error) {
    console.log('ROS connection error:', error);
    isForkMoving = false; // Reset on error
  });
});
