// src/components/SavedZonesPage.js
import React, { useRef, useEffect, useState } from 'react';
import './SavedZonesPage.css';
import ROSLIB from 'roslib';
import {publishPoint, startNode} from '../rosService'; // Import the functions used from the rosService file

const SavedZonesPage = () => {
  let startNodeService;
  const canvasRef = useRef(null);
  const [points, setPoints] = useState([]);
  const imgRef = useRef(null); // Reference for the image so it only loads once

  useEffect(() => {
    const startNodeService = startNode();
    const pointPublisher = publishPoint();

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const img = new Image();

    // Load the image only once and store it in imgRef
    img.src = `${process.env.PUBLIC_URL}/my_world_map2.png`;
    img.onload = () => {
      imgRef.current = img;
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    };
  }, []);

  const handleCanvasClick = (event) => {
    if (points.length < 4) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      const rect = canvas.getBoundingClientRect();
      const x = Math.round(event.clientX - rect.left);
      const y = Math.round(event.clientY - rect.top);

      // Add the new point to the array
      const newPoints = [...points, { x, y }];
      setPoints(newPoints);

      // Redraw the image and all points
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(imgRef.current, 0, 0, canvas.width, canvas.height);
      newPoints.forEach(point => {
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(point.x, point.y, 2, 0, 2 * Math.PI); // Draws a small circle of radius 5
        ctx.fill();
      });
    }
  };

  const handleStartNode = () => {
    const startNodeService = startNode();
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

    handleStartNode();

    if (points.length !== 4) {
      alert("Please select 4 points.");
      return;
    }

    console.log("Submitted points:", points);

    // Clear the points and redraw only the image
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(imgRef.current, 0, 0, canvas.width, canvas.height);
    setPoints([]); // Reset all 4 points drawn

    // Prepare points as a string for easier handling in ROS
    const pointPublisher = publishPoint();
    const pointsStr = points.map(p=> `${p.x} ${p.y}`).join('\n');
    const message = new ROSLIB.Message({ data: pointsStr });

    //Publish the points to the ROS topic
    pointPublisher.publish(message);
    alert("Points submitted to ROS.");
  };

  return (
    <div id="saved-zones-page" className="page">
      <h2>Saved Zones</h2>
      <canvas ref={canvasRef} id="canvas" width="300" height="300" onClick={handleCanvasClick}></canvas>
      {/* <button onClick={handleStartNode}>Start Coverage_Planner Node</button> */}
      <button onClick={submitPoints}>Submit Points</button>
    </div>
  );
};

export default SavedZonesPage;
