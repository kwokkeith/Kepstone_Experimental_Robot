// src/components/CreateMapPage.js
import React, { useEffect, useRef, useState } from 'react';
import './CreateMapPage.css';
import ROSLIB from 'roslib';
import { coverage_listener, publishPoint, publishStartPoint} from '../rosService';

const CreateMapPage = ({ mapName }) => {
  // ==========================
  // React States
  // ==========================
  const imageRef = useRef(null);
  const canvasRef = useRef(null);
  const [points, setPoints] = useState([]);
  const [startpoints, setStartPoints] = useState(false);
  const [coverageListener, setcoverageListener] = useState('');
  const [showButtonContainer, setShowButtonContainer] = useState(false);

  // ==========================
  // React useEffect Hooks called whenever a dependent state changes
  // ==========================

  // Log the map name when the component loads
  useEffect(() => {
    if (mapName) {
      console.log(`Creating new map: ${mapName}`);
    }
  },[mapName] );

  // Draw the image in the canvas
  useEffect(() => {
    const image = imageRef.current;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    image.onload = () => {
      canvas.width = image.width;
      canvas.height = image.height;
      ctx.drawImage(image, 0, 0);

      // Check session storage and draw contours if data is present
      const storedCoverageListener = sessionStorage.getItem('coverageListener');
      if (storedCoverageListener) {
        handleCoverageListenerChange(JSON.parse(storedCoverageListener));
        setShowButtonContainer(true); // Show button container here
      }
    };
  }, []);

  useEffect(() => {
    const listener = coverage_listener();

    listener.subscribe(function(message) {
      console.log('Received message on coverage topic:', message.data);
      setcoverageListener(message.data);
      sessionStorage.setItem('coverageListener', JSON.stringify(message.data));
    });

    return () => {
      listener.unsubscribe();
    };
  }, []);

  // Call handleCoverageListenerChange whenever coverageListener changes
  useEffect(() => {
    handleCoverageListenerChange(coverageListener);
  }, [coverageListener]);

  useEffect(() => {
    // If 4 points are drawn, publish the points to the ROS topic
    if (points.length === 4){
      // Clear the canvas and redraw the image
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(imageRef.current, 0, 0, canvas.width, canvas.height);

      // Prepare points as a string for easier handling in ROS
      const pointPublisher = publishPoint();
      const pointsStr = points.map(p=> `${p.x} ${p.y}`).join('\n');
      const message = new ROSLIB.Message({ data: pointsStr });
      setPoints([]); // Reset all 4 points drawn
      setStartPoints(true);

      //Publish the points to the ROS topic under /roi_points
      pointPublisher.publish(message);
    } 
  }, [points]);

  useEffect(() => {
    if (startpoints && points.length === 1){
      // Clear the canvas and redraw the image
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(imageRef.current, 0, 0, canvas.width, canvas.height);

      const StartPointPublisher = publishStartPoint();
      const startPointsStr = points.map(p=> `${p.x} ${p.y}`).join('\n');
      const startPointsMessage = new ROSLIB.Message({ data: startPointsStr });
      setPoints([]);
      setStartPoints(false);
      StartPointPublisher.publish(startPointsMessage);
    }
  },[points]);

  // ==========================
  // FUNCTIONS
  // ==========================

  const handleCanvasClick = (event) => {
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
    ctx.drawImage(imageRef.current, 0, 0, canvas.width, canvas.height);
    newPoints.forEach(point => {
      ctx.fillStyle = 'red';
      ctx.beginPath();
      ctx.arc(point.x, point.y, 2, 0, 2 * Math.PI); // Draws a small circle of radius 5
      ctx.fill();
    });
    
  };

  const handleCoverageListenerChange = (newData) => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
  
    // Split the newData into separate contours
    const contoursData = newData.trim().split('],').map(contour => contour.replace('[', '').replace(']', '').trim());
  
    // Draw each contour separately
    ctx.strokeStyle = 'magenta';
    ctx.lineWidth = 1;
  
    contoursData.forEach(contourData => {
      const contours = contourData.split('\n').map(line => {
        const [x, y] = line.split(' ').map(Number);
        return { x, y };
      });
  
      if (contours.length > 0) {
        ctx.beginPath();
        ctx.moveTo(contours[0].x, contours[0].y);
        contours.forEach(point => {
          ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.stroke();
      }
    });
  };

  const handleClearData = () => {
    // Nothing here worked
    // sessionStorage.removeItem("coverageListener");
    // setcoverageListener('');
    // setShowClearButton(false);
    // const canvas = canvasRef.current;
    // const ctx = canvas.getContext('2d');
    // ctx.clearRect(0, 0, canvas.width, canvas.height);
  };
  const handleEdit = () => {

  };
  const handleSave = () => {
    
  };

  // ==========================
  // React rendered html component
  // ==========================

  return (
    <div className="create-map-page">
      <h2>{mapName}</h2>
      <img
        ref={imageRef}
        src={`${process.env.PUBLIC_URL}/my_world_map2.png`}
        alt="My World Map"
        className="my-world-map"
        style={{ display: 'none' }}
      />
      <canvas ref={canvasRef} className="my-world-canvas" onClick={handleCanvasClick}></canvas>
      {showButtonContainer && (
      <div className="button-container">
        <button onClick={handleClearData} className="cancel-btn">Cancel</button>
        <button onClick={handleEdit} className="edit-btn">Edit</button>
        <button onClick={handleSave} className="save-btn">Save</button>
      </div>
    )}
      <span className="loader"></span>
    </div>
  );
};

export default CreateMapPage;