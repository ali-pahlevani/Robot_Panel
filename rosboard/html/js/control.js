console.log('control.js loaded');
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

  var maxLinearSpeed = 0.5; // m/s
  var maxAngularSpeed = 1.0; // rad/s

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

  ros.on('connection', function() {
    console.log('ROS connected');
    document.getElementById('joystick-container').classList.remove('joystick-disabled');
  });

  ros.on('close', function() {
    console.log('ROS disconnected');
    document.getElementById('joystick-container').classList.add('joystick-disabled');
  });

  ros.on('error', function(error) {
    console.log('ROS connection error:', error);
  });
});