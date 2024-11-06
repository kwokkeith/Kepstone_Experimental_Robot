var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server:', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Define a ROS service client to start the node
var startNodeService = new ROSLIB.Service({
    ros: ros,
    name: '/start_coverage_planner', // Replace with your actual service name
    serviceType: 'std_srvs/Trigger' // Replace with the appropriate service type
});

function startNode() {
    // Create a request object
    var request = new ROSLIB.ServiceRequest({});

    // Call the service to start the node
    startNodeService.callService(request, function(result) {
        console.log('Service call result:', result);
        console.log("Started coverage_planner_node");
    }, function(error) {
        console.error('Service call failed:', error);
    });
}
