console.log('control.js loaded');

let fullLift, fullLower; // Exportable functions
let joystick = null; // Store joystick instance
let joystickTimer = null; // Timer for periodic Twist publishing
let joystickVector = null; // Store latest joystick vector

document.addEventListener('DOMContentLoaded', () => {
  console.log('DOM loaded');
  console.log('roslib:', typeof ROSLIB);
  console.log('nipplejs:', typeof nipplejs);
  console.log('Joystick container:', document.getElementById('joystick-container'));

  // Dynamically determine WebSocket URL
  const wsHost = window.location.hostname || 'localhost';
  const wsPort = 9090; // Match rosbridge_websocket port
  const wsUrl = 'ws://' + wsHost + ':' + wsPort;

  // Initialize ROS connection
  const ros = new ROSLIB.Ros({
    url: wsUrl
  });

  // Function to initialize or update joystick
  function initializeJoystick() {
    const container = document.getElementById('joystick-container');
    if (!container) {
      console.error('Joystick container not found');
      return;
    }
    // Get precise container width
    const containerWidth = container.getBoundingClientRect().width;
    const joystickSize = Math.min(containerWidth, 200); // Match container or cap at 200px

    // Destroy existing joystick
    if (joystick) {
      try {
        joystick.destroy();
        console.log('Destroyed existing joystick');
      } catch (e) {
        console.error('Error destroying joystick:', e);
      }
    }

    try {
      joystick = nipplejs.create({
        zone: container,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#44b0e0',
        size: joystickSize // Dynamic size based on container
      });
      console.log('Joystick created with size:', joystickSize);

      // Ensure container is interactive
      container.style.pointerEvents = 'auto';
      container.style.touchAction = 'manipulation'; // Match CSS

      // Attach event listeners
      joystick.on('move', (event, data) => {
        console.log('Joystick move:', data.vector);
        joystickVector = data.vector; // Store latest vector
        // Start periodic publishing if not already running
        if (!joystickTimer) {
          joystickTimer = setInterval(() => {
            if (joystickVector && ros.isConnected) {
              const linear = joystickVector.y * maxLinearSpeed;
              const angular = -joystickVector.x * maxAngularSpeed;
              const twist = new ROSLIB.Message({
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular }
              });
              cmdVelTopic.publish(twist);
              console.log('Periodic Twist published:', twist);
            } else if (!ros.isConnected) {
              console.warn('ROS not connected, cannot publish periodic Twist');
            }
          }, 100); // 10 Hz
        }
      });

      joystick.on('end', () => {
        console.log('Joystick end, resetting state');
        joystickVector = null; // Clear vector
        if (joystickTimer) {
          clearInterval(joystickTimer);
          joystickTimer = null;
          console.log('Cleared joystick timer');
        }
        if (ros.isConnected) {
          const twist = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
          });
          cmdVelTopic.publish(twist);
          console.log('Published zero Twist:', twist);
        } else {
          console.warn('ROS not connected, cannot publish zero Twist');
        }
        // Ensure container remains interactive
        container.style.pointerEvents = 'auto';
        // Log joystick state to debug
        console.log('Joystick after end:', joystick);
      });
    } catch (e) {
      console.error('Joystick creation error:', e);
    }
  }

  // Initialize joystick
  initializeJoystick();

  // Update joystick size on window resize with debounce
  let resizeTimeout;
  window.addEventListener('resize', () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(initializeJoystick, 50); // 50ms debounce
  });

  const cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/tri_cycle_controller/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  const forksTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/forks_position_controller/commands',
    messageType: 'std_msgs/Float64MultiArray'
  });

  const maxLinearSpeed = 0.5; // m/s
  const maxAngularSpeed = 1.0; // rad/s
  let currentForkPosition = 0.0; // Track commanded fork position (0.0 or 0.03)
  let isForkMoving = false; // Prevent concurrent movements

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
      console.log('Forks already at full lift position');
      return { success: true, message: 'Forks already lifted' };
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

  ros.on('connection', () => {
    console.log('ROS connected');
    document.getElementById('joystick-container').classList.remove('joystick-disabled');
    // Re-initialize joystick on connection to ensure interactivity
    initializeJoystick();
  });

  ros.on('close', () => {
    console.log('ROS disconnected');
    document.getElementById('joystick-container').classList.add('joystick-disabled');
    isForkMoving = false; // Reset on disconnect
  });

  ros.on('error', (error) => {
    console.log('ROS connection error:', error);
    isForkMoving = false; // Reset on error
  });
});