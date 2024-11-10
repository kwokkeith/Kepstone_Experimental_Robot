// src/rosService.js
import ROSLIB from 'roslib';

export function startNode() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Coverage_Planner is connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge:', error);
  });
  ros.on('connection', function() {
    console.log('Coverage_Planner disconnected from ROSBridge');
  });

  //Define a ROS service client to start the node
  const startNodeService = new ROSLIB.Service({
    ros: ros,
    name: '/start_coverage_planner', // Replace with your actual service name
    serviceType: 'std_srvs/Trigger' // Replace with the appropriate service type
  });

  return startNodeService;
}

export function publishPoint() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Point publiisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the point publisher:', error);
  });
  ros.on('connection', function() {
    console.log('Point Publisher is disconnected from ROSBridge');
  });

  const pointPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/roi_points',
    messageType: 'std_msgs/String'
  });

  return pointPublisher;
}