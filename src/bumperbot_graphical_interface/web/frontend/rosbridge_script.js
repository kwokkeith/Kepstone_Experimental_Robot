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

// Subscribing to Topic
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/move_base/result',
    messageType : 'move_base_msgs/MoveBaseActionResult'
});

var isListening = false;

function startNode() {
    if (!isListening) {
        listener.subscribe(function(message) {
            console.log('Received message on ' + listener.name + ': ' + message.status.goal_id.id);
            //console.log(message); // Log the entire message object

        });
        isListening = true;
        console.log("Started listening on " + listener.name);
    }
}

function stopNode() {
    if (isListening) {
        listener.unsubscribe();
        isListening = false;
        console.log("Stopped listening on " + listener.name);
    }
}
