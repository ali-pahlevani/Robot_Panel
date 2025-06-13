console.log('control.js loaded');

let fullLift, fullLower; // Exportable functions
let joystick = null; // Store joystick instance
let joystickTimer = null; // Timer for periodic Twist publishing
let joystickVector = null; // Store latest joystick vector
let simTime = null; // Store latest simulation time from /clock
let useSimulationTime = false;

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

  // Subscribe to /clock for simulation time
  const clockTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/clock',
    messageType: 'rosgraph_msgs/Clock'
  });

  // Function to initialize or update joystick
  function initializeJoystick() {
    const container = document.getElementById('joystick-container');
    if (!container) {
      console.error('Joystick container not found');
      return;
    }
    const containerWidth = container.getBoundingClientRect().width;
    const joystickSize = Math.min(containerWidth, 200);

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
        size: joystickSize
      });
      console.log('Joystick created with size:', joystickSize);

      container.style.pointerEvents = 'auto';
      container.style.touchAction = 'manipulation';

      joystick.on('move', (event, data) => {
        console.log('Joystick move:', data.vector);
        joystickVector = data.vector;
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
          }, 100);
        }
      });

      joystick.on('end', () => {
        console.log('Joystick end, resetting state');
        joystickVector = null;
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
        container.style.pointerEvents = 'auto';
      });
    } catch (e) {
      console.error('Joystick creation error:', e);
    }
  }

  initializeJoystick();

  let resizeTimeout;
  window.addEventListener('resize', () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(initializeJoystick, 50);
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
  let currentForkPosition = 0.0;
  let isForkMoving = false;

  function moveForks(targetPosition) {
    if (isForkMoving) {
      console.log('Fork movement already in progress');
      return Promise.reject(new Error('Fork movement already in progress'));
    }
    isForkMoving = true;
    return new Promise((resolve, reject) => {
      const duration = 1000;
      const rate = 10;
      const steps = duration / (1000 / rate);
      const stepSize = (targetPosition - currentForkPosition) / steps;
      let currentStep = 0;

      const interval = setInterval(() => {
        if (currentStep >= steps) {
          clearInterval(interval);
          currentForkPosition = targetPosition;
          isForkMoving = false;
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

  window.control = window.control || {};
  window.control.fullLift = fullLift;
  window.control.fullLower = fullLower;
  window.control.startNavigation = startNavigation;
  window.control.stopNavigation = stopNavigation;

  ros.on('connection', () => {
    console.log('ROS connected');
    document.getElementById('joystick-container').classList.remove('joystick-disabled');
    initializeJoystick();
  });

  ros.on('close', () => {
    console.log('ROS disconnected');
    document.getElementById('joystick-container').classList.add('joystick-disabled');
    isForkMoving = false;
    simTime = null;
  });

  ros.on('error', (error) => {
    console.log('ROS connection error:', error);
    isForkMoving = false;
    simTime = null;
  });

  const initialPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/initialpose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  });

  const goalPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped'
  });

  const navigateToPoseClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose'
  });

  function getTopics() {
    return new Promise((resolve, reject) => {
      ros.getTopics((topics) => resolve(topics.topics), (error) => reject(error));
    });
  }

  async function startNavigation() {
    const simCheckbox = document.getElementById('simulation-checkbox');
    if (simCheckbox && simCheckbox.checked) {
      try {
        const topics = await getTopics();
        if (topics.includes('/clock')) {
          useSimulationTime = true;
          clockTopic.subscribe((message) => {
            simTime = message.clock;
            console.log('Received simulation time:', simTime);
          });
        } else {
          useSimulationTime = false;
        }
      } catch (error) {
        console.error('Error getting topics:', error);
        useSimulationTime = false;
      }
    } else {
      useSimulationTime = false;
    }
  }

  function stopNavigation() {
    clockTopic.unsubscribe();
    useSimulationTime = false;
    simTime = null; // Optional: reset simTime
  }

  function getTimestamp() {
    if (useSimulationTime && simTime) {
      console.log('Using simulation time:', simTime);
      return { sec: simTime.secs, nanosec: simTime.nsecs };
    } else {
      console.log('Using system time');
      const now = new Date();
      return {
        sec: Math.floor(now.getTime() / 1000),
        nanosec: (now.getTime() % 1000) * 1000000
      };
    }
  }

  function degreesToQuaternion(degrees) {
    const radians = degrees * (Math.PI / 180);
    return {
      x: 0.0,
      y: 0.0,
      z: Math.sin(radians / 2),
      w: Math.cos(radians / 2)
    };
  }

  function parsePoseInput(input) {
    const values = input.trim().split(/\s+/).map(parseFloat);
    if (values.length !== 3 || values.some(isNaN)) {
      return null;
    }
    return { x: values[0], y: values[1], headingDeg: values[2] };
  }

  document.getElementById('set-initial-pose-button').addEventListener('click', () => {
    const input = document.getElementById('initial-pose-input').value;
    const pose = parsePoseInput(input);
    if (!pose) {
      console.log('Invalid input for initial pose');
      return;
    }
    const { x, y, headingDeg } = pose;
    const orientation = degreesToQuaternion(headingDeg);
    const timestamp = getTimestamp();
    const msg = new ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: { sec: timestamp.secs, nanosec: timestamp.nsecs }
      },
      pose: {
        pose: {
          position: { x, y, z: 0.0 },
          orientation
        },
        covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
      }
    });
    if (ros.isConnected) {
      initialPoseTopic.publish(msg);
      console.log('Published initial pose:', msg);
    } else {
      console.log('ROS not connected');
    }
  });

  document.getElementById('set-target-button').addEventListener('click', () => {
    const input = document.getElementById('target-input').value;
    const pose = parsePoseInput(input);
    if (!pose) {
      console.log('Invalid input for target pose');
      return;
    }
    const { x, y, headingDeg } = pose;
    const orientation = degreesToQuaternion(headingDeg);
    const timestamp = getTimestamp();

    const goalPoseMsg = new ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: { sec: timestamp.secs, nanosec: timestamp.nsecs }
      },
      pose: {
        position: { x, y, z: 0.0 },
        orientation
      }
    });
    if (ros.isConnected) {
      goalPoseTopic.publish(goalPoseMsg);
      console.log('Published goal pose:', goalPoseMsg);
    } else {
      console.log('ROS not connected for goal pose');
    }

    if (ros.isConnected) {
      const goal = new ROSLIB.Goal({
        actionClient: navigateToPoseClient,
        goalMessage: {
          pose: {
            header: {
              frame_id: 'map',
              stamp: { sec: timestamp.secs, nanosec: timestamp.nsecs }
            },
            pose: {
              position: { x, y, z: 0.0 },
              orientation
            }
          }
        }
      });
      goal.send();
      console.log('Sent navigate_to_pose goal:', { x, y, headingDeg });
      goal.on('result', (result) => {
        console.log('Navigate to pose result:', result);
      });
    } else {
      console.log('ROS not connected for navigate_to_pose');
    }
  });
});