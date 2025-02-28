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
  ros.on('close', function() {
    console.log('Coverage_Planner disconnected from ROSBridge');
  });

  //Define a ROS service client to start the node
  const startNodeService = new ROSLIB.Service({
    ros: ros,
    name: '/start_coverage_planner', 
    serviceType: 'std_srvs/Trigger' 
  });

  return startNodeService;
}

export function endNode() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('End_Coverage is connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge:', error);
  });
  ros.on('close', function() {
    console.log('End_Coverage disconnected from ROSBridge');
  });

  //Define a ROS service client to start the node
  const endNodeService = new ROSLIB.Service({
    ros: ros,
    name: '/stop_coverage_planner', 
    serviceType: 'std_srvs/Trigger' 
  });

  return endNodeService;
}

export function publishPoint() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Point publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the point publisher:', error);
  });
  ros.on('close', function() {
    console.log('Point Publisher is disconnected from ROSBridge');
  });

  const pointPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/roi_points',
    messageType: 'std_msgs/String'
  });

  return pointPublisher;
}

export function publishStartPoint() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Start Point publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the point publisher:', error);
  });
  ros.on('close', function() {
    console.log('Start Point Publisher is disconnected from ROSBridge');
  });

  const StartPointPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/start_point',
    messageType: 'std_msgs/String'
  });

  return StartPointPublisher;
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
  ros.on('close', function() {
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
        if(i=== times - 1){
          ros.close();
        }
      }, i * 1000); // Delay of 1 second between each publish
    }
  }
  publishMultipleTimes(3);
}

export function coverage_listener() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Coverage listener is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the coverage listener:', error);
  });
  ros.on('close', function() {
    console.log('Coverage listener is disconnected from ROSBridge');
  });

  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/canvas_messenger',
    messageType: 'std_msgs/String'
  });

  return listener;
}

export function publishEditState({editState}) {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Edit State publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the edit state publisher:', error);
  });
  ros.on('close', function() {
    console.log('Edit State Publisher is disconnected from ROSBridge');
  });

  const EditStatePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/edit_state',
    messageType: 'std_msgs/Bool'
  });

  // Create a ROS message
  const editStateMsg = new ROSLIB.Message({ data: editState });

  // Publish the message multiple times
  function publishMultipleTimes(times) {
    for (let i = 0; i < times; i++) {
      setTimeout(() => {
        EditStatePublisher.publish(editStateMsg);
        // console.log(`Published editstate ${i + 1} times`);
        if(i=== times - 1){
          ros.close();
        }
      }, i * 800); // Delay of 1 second between each publish
    }
  }
  publishMultipleTimes(4);
}

export function publishContourAngles({data: contourAnglesList}) {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('all BCD PolyContours publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the all BCD PolyContours publisher:', error);
  });
  ros.on('close', function() {
    console.log('all BCD PolyContours Publisher is disconnected from ROSBridge');
  });

  const contourAnglesPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/new_angle_array',
    messageType: 'std_msgs/String'
  });
  const contourAnglesMsg = new ROSLIB.Message({ data: contourAnglesList });

  function publishMultipleTimes(times) {
    for (let i = 0; i < times; i++) {
      setTimeout(() => {
        contourAnglesPublisher.publish(contourAnglesMsg);
        // console.log(`Published editstate ${i + 1} times`);
        if(i=== times - 1){
          ros.close();
        }
      }, i * 800); // Delay of 1 second between each publish
    }
  }
  publishMultipleTimes(2);
}

export function publishDbShutdownState({dbState}) {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('db State publisher is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the db state publisher:', error);
  });
  ros.on('close', function() {
    console.log('db State Publisher is disconnected from ROSBridge');
  });

  const dbStatePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/shutdown_bool',
    messageType: 'std_msgs/Bool'
  });

  // Create a ROS message
  const dbStateMsg = new ROSLIB.Message({ data: dbState });

  // Publish the message multiple times
  function publishMultipleTimes(times) {
    for (let i = 0; i < times; i++) {
      setTimeout(() => {
        dbStatePublisher.publish(dbStateMsg);
        // console.log(`Published editstate ${i + 1} times`);
        if(i=== times - 1){
          ros.close();
        }
      }, i * 800); // Delay of 1 second between each publish
    }
  }
  publishMultipleTimes(2);
}

export function callWriteWaypointsService({ waypointsList }) {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to ROSBridge for service call.');
    
    // Create the service client
    var serviceClient = new ROSLIB.Service({
      ros: ros,
      name: '/write_waypoints_to_txt_file',
      serviceType: 'bumperbot_graphical_interface/WriteWaypoints'
    });
    
    // Create the service request
    var request = new ROSLIB.ServiceRequest({
      waypoints_list: waypointsList
    });
    
    serviceClient.callService(request, function(result) {
      console.log("Service called, received result:", result);
      ros.close();
    });
  });

  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for service call:', error);
  });

  ros.on('close', function() {
    console.log('Disconnected from ROSBridge.');
  });
}

export function triggerStartCoverageService() {
  // Connect to ROS
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to ROSBridge.');

    // Create the service client
    var serviceClient = new ROSLIB.Service({
      ros: ros,
      name: '/start_coverage',
      serviceType: 'std_srvs/Trigger'
    });

    // Create the service request
    var request = new ROSLIB.ServiceRequest({});

    // Call the service
    serviceClient.callService(request, function(result) {
      console.log("Service called, received result:", result);
      ros.close();
    });
  });

  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for service call:', error);
  });

  ros.on('close', function() {
    console.log('Disconnected from ROSBridge.');
  });
}

export function triggerStopCoverageService() {
  // Connect to ROS
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to ROSBridge.');

    // Create the service client
    var serviceClient = new ROSLIB.Service({
      ros: ros,
      name: '/robot_controller/stop_robot_job',
      serviceType: 'std_srvs/Trigger'
    });

    // Create the service request
    var request = new ROSLIB.ServiceRequest({});

    // Call the service
    serviceClient.callService(request, function(result) {
      console.log("Service to stop called, received result:", result);
      ros.close();
    });
  });

  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for service call:', error);
  });

  ros.on('close', function() {
    console.log('Disconnected from ROSBridge.');
  });
}

export function sidebrush_speed_listener() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('sidebrush listener is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the sidebrush listener:', error);
  });
  ros.on('close', function() {
    console.log('sidebrush listener is disconnected from ROSBridge');
  });

  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/sidebrush_controller/sidebrush_speed',
    messageType: 'std_msgs/UInt32'
  });

  return listener;
}

export function sidebrush_position_listener() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('sidebrush listener is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the sidebrush listener:', error);
  });
  ros.on('close', function() {
    console.log('sidebrush listener is disconnected from ROSBridge');
  });

  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/sidebrush_controller/sidebrush_position',
    messageType: 'std_msgs/String'
  });

  return listener;
}

export function realsense_d455_listener_front() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('realsense_d455_listener is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the realsense_d455_listener:', error);
  });
  ros.on('close', function() {
    console.log('realsense_d455_listener is disconnected from ROSBridge');
  });

  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/depth_camera/color/image_compressed/compressed/', 
    messageType: 'sensor_msgs/CompressedImage'
  });

  // Throttling: allow messages only every ~17ms (60fps)
  let lastTimestamp = 0;
  const throttleInterval = 1000 / 30; // ~33.33ms

  const originalSubscribe = listener.subscribe.bind(listener);
  listener.subscribe = function(callback) {
    originalSubscribe((msg) => {
      const now = Date.now();
      if (now - lastTimestamp < throttleInterval) return;
      lastTimestamp = now;
      callback(msg);
    });
  };

  return listener;
}

export function realsense_d455_listener_rear() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('realsense_d455_listener_rear is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the realsense_d455_listener_rear:', error);
  });
  ros.on('close', function() {
    console.log('realsense_d455_listener_rear is disconnected from ROSBridge');
  });

  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/depth_camera/color/image_compressed/compressed/rear', //Fake name change for DV8
    messageType: 'sensor_msgs/CompressedImage'
  });
  // Throttling: allow messages only every ~17ms (60fps)
  let lastTimestamp = 0;
  const throttleInterval = 1000 / 30; // ~33.33ms

  const originalSubscribe = listener.subscribe.bind(listener);
  listener.subscribe = function(callback) {
    originalSubscribe((msg) => {
      const now = Date.now();
      if (now - lastTimestamp < throttleInterval) return;
      lastTimestamp = now;
      callback(msg);
    });
  };

  return listener;
}

export function battery_percentage_listener() {
  // Initialize ROS connection
  var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('battery_percentage_listener is now connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.error('Error connecting to ROSBridge for the battery_percentage_listener:', error);
  });
  ros.on('close', function() {
    console.log('battery_percentage_listener is disconnected from ROSBridge');
  });

  const battery_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/sam/battery_state', 
    messageType: 'sensor_msgs/BatteryState'
  });

  return battery_listener;
}
