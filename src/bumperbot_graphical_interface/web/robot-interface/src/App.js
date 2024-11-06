import React, { useEffect, useState, useRef} from 'react';
import './App.css';
import ROSLIB from 'roslib';


function App() {
  let startNodeService;
  const [points, setPoints] = useState([]);
  const pointsRef = useRef(points);
  const canvasRef = useRef(null);
  const imgRef = useRef(null);

  useEffect(() => {
    // Initialize ROS connection
    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
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
    startNodeService = new ROSLIB.Service({
      ros: ros,
      name: '/start_coverage_planner', // Replace with your actual service name
      serviceType: 'std_srvs/Trigger' // Replace with the appropriate service type
    });

      // Load and display image on the canvas
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.src = `${process.env.PUBLIC_URL}/my_world_map2.png`; // Replace with the actual image URL
    img.onload = function() {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    };

      // Capture clicks for point selection
    const handleCanvasClick = (event) => {
      console.log(pointsRef.current);
      if (pointsRef.current.length < 4) {
        const rect = canvas.getBoundingClientRect();
        const x = Math.round(event.clientX - rect.left);
        const y = Math.round(event.clientY - rect.top);
        pointsRef.current.push({ x, y });
        setPoints([...pointsRef.current]);
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(x, y, 2, 0, 2 * Math.PI);
        ctx.fill();
        if (pointsRef.current.length === 4) {
          console.log("4 points selected", pointsRef.current);
        }
      }
    };

    canvas.addEventListener('click', handleCanvasClick);

    // Cleanup function to close the ROS connection when the component unmounts
    return () => {
      canvas.removeEventListener('click', handleCanvasClick);
      ros.close();
    };
  }, []);





  const toggleMenu = () => {
    const menu = document.getElementById("menu");
    const menuButton = document.getElementById("menu-button");
    menu.classList.toggle("active"); // Toggle the 'active' class to show/hide the menu
    menuButton.classList.toggle("menu-open");
  };

  const toggleFullscreen = () => {
    const icon = document.getElementById("fullscreen-icon");
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().then(() => {
            icon.classList.replace("fa-maximize", "fa-minimize");
        }).catch((err) => {
            console.warn("Fullscreen mode not enabled:", err);
        });
    } else {
        document.exitFullscreen().then(() => {
            icon.classList.replace("fa-minimize", "fa-maximize");
        }).catch((err) => {
            console.warn("Fullscreen exit not enabled:", err);
        });
    }
  };

  const checkLogin = (event) => {
    event.preventDefault();
    const username = document.getElementById("username").value;
    const password = document.getElementById("password").value;

    if (username === "admin" && password === "password") {
        document.getElementById("login-page").style.display = "none";
        document.getElementById("main-page").style.display = "block";
        document.getElementById("menu-button").style.display = "block"; // Show menu button after login
        return false;
    } else {
        document.getElementById("login-error").style.display = "block";
        return false;
    }
  };

  const showPage = (page) => {
    const pages = document.querySelectorAll('.page');
    pages.forEach(p => p.style.display = 'none');
    document.getElementById(page + '-page').style.display = 'block';
    toggleMenu(); // Hide menu after selecting a page
  };

  const startNode = () => {
    if (startNodeService) {
      const request = new ROSLIB.ServiceRequest({});
      startNodeService.callService(request, function(result) {
        console.log('Service call result:', result);
      }, function(error) {
        console.error('Service call failed:', error);
      });
    }
  };

  const submitPoints = () => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const img = imgRef.current;
    
    if (pointsRef.current.length !== 4) {
      alert("Please select 4 points.");
      setPoints([]);
      pointsRef.current = [];
      return;
    } else {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    }

    // Create a ROS connection
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' 
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
    setPoints([]);
    pointsRef.current = [];

    window.addEventListener('beforeunload', () => {
        pointPublisher.unsubscribe();
    });
  };

  const localizeRobot = () => {
    alert("Local Localization Placeholder");
  };

  const adjustBrightness = () => {
    alert("Brightness Adjustment Placeholder");
  }


  return (
    <div>
      {/* Fixed Menu Button (Top Left) */}
      <div className="menu-button" id="menu-button">
        <i className="fa-solid fa-bars" id="menu-icon" onClick={toggleMenu}></i>
      </div>

      {/* Fixed Fullscreen Button (Top Right) */}
      <div className="fullscreen-button">
        <i className="fa-solid fa-minimize" id="fullscreen-icon" onClick={toggleFullscreen}></i>
      </div>

      {/* Login Page */}
      <div id="login-page" className="page">
        <h2>Login</h2>
        <form onSubmit={checkLogin}>
          <input type="text" placeholder="Username" id="username" defaultValue="admin" required />
          <input type="password" placeholder="Password" id="password" defaultValue="password" required />
          <button type="submit">Login</button>
        </form>
        <p id="login-error" style={{ color: 'red', display: 'none' }}>Incorrect login, please try again.</p>
      </div>

      {/* Main Page */}
      <div id="main-page" className="page" style={{ display: 'none' }}>
        <main>
          <h2>Robot Status</h2>
          <div className="status">
            <p>Battery: <span id="battery-status">100%</span></p>
            <p>Status: <span id="robot-status">Docking Station</span></p>
            <p>Health: <span id="robot-health">Full Health</span></p>
          </div>
        </main>
      </div>

      {/* Sliding Menu */}
      <div id="menu" className="slide-menu">
        <a href="#" onClick={() => showPage('main')}>Dashboard</a>
        <a href="#" onClick={() => showPage('saved-zones')}>Saved Zones</a>
        <a href="#" onClick={() => showPage('settings')}>Settings</a>
      </div>

      {/* Saved Zones Page */}
      <div id="saved-zones-page" className="page" style={{ display: 'none' }}>
        {/* Testing section */}
        <h2>Start the Coverage_Planner node first</h2>
        <button onClick={startNode}>start coverage_planner_node</button>

        {/* Section to interact with Coverage_planner */}
        <h2>Saved Zones</h2>
        <canvas id="canvas" width="300" height="300" ref={canvasRef}></canvas>
        <img src={`${process.env.PUBLIC_URL}/my_world_map2.png`} alt="Map" ref={imgRef} style={{ display: 'none' }} />
        <button onClick={submitPoints}>Submit Points</button>
      </div>

      {/* Settings Page */}
      <div id="settings-page" className="page" style={{ display: 'none' }}>
        <h2>Settings</h2>
        <button onClick={localizeRobot}>Local Localization</button>
        <button onClick={adjustBrightness}>Screen Brightness</button>
      </div>
    </div>
  );
}

export default App;
