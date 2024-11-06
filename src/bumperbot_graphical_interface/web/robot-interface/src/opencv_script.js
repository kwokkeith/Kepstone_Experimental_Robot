const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const points = [];

// Load and display image on the canvas
const img = new Image();
img.src = 'my_world_map2.png'; // Replace with the actual image URL
img.onload = function() {
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
};

// Capture clicks for point selection
canvas.addEventListener('click', (event) => {
    if (points.length < 4) {
        const rect = canvas.getBoundingClientRect();
        const x = Math.round(event.clientX - rect.left);
        const y = Math.round(event.clientY - rect.top);
        points.push({ x, y });
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(x, y, 2, 0, 2 * Math.PI);
        ctx.fill();
        if (points.length === 4) {
            console.log("4 points selected", points);
        }
    }
});

// Send points to ROS when submit button is clicked
function submitPoints() {
    if (points.length !== 4) {
        alert("Please select 4 points.");
        return;
    } else {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    }

    // Create a ROS connection
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Replace with your ROSBridge WebSocket URL
    });

    // Define a ROS topic
    const pointPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/roi_points',
        messageType: 'std_msgs/String'
    });

    // Prepare points as a string for easier handling in ROS
    const pointsStr = points.map(p => `${p.x} ${p.y}`).join('\n');
    const message = new ROSLIB.Message({ data: pointsStr });

    // Publish points
    pointPublisher.publish(message);
    alert("Points submitted to ROS.");
    points.length=0;

    window.addEventListener('beforeunload', () => {
        pointPublisher.unsubscribe();
    });
}