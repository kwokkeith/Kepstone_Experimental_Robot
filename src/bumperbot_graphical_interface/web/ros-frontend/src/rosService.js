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

export function publishmapName({ mapName }) {
  // Initialize ROS connection 
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });
  
  ros.on('connection', function() {
    console.log('Map name publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the map name publisher:', error);
  });
  ros.on('connection', function() {
    console.log('Map name publisher is disconnected from ROSBridge');
  });

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/new_map_name',
    messageType: 'std_msgs/String'
  });

  // Create a ROS message
  const message = new ROSLIB.Message({ data: mapName });

  // Publish the message multiple times
  function publishMultipleTimes(times) {
    for (let i = 0; i < times; i++) {
      setTimeout(() => {
        topic.publish(message);
        console.log(`Published message ${i + 1} times`);
      }, i * 1000); // Delay of 1 second between each publish
    }
  }
  publishMultipleTimes(5);
}