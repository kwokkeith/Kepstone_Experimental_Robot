// src/components/CreateMapPage.js
import React, { useEffect, useRef, useState } from 'react';
import './CreateMapPage.css';
import { coverage_listener } from '../rosService';

const CreateMapPage = ({ mapName }) => {
  const imageRef = useRef(null);
  const canvasRef = useRef(null);
  const [coverageListener, setcoverageListener] = useState('');
  const [showClearButton, setShowClearButton] = useState(false);

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
        setShowClearButton(true);
      }
    };
  }, []);

  useEffect(() => {
    const listener = coverage_listener();

    listener.subscribe(function(message) {
      console.log('Received message on coverage topic:', message.data);
      setcoverageListener(message.data);
      sessionStorage.setItem('coverageListener', JSON.stringify(message.data));
      setShowClearButton(true);
    });

    return () => {
      listener.unsubscribe();
    };
  }, []);

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
  
  // Call handleCoverageListenerChange whenever coverageListener changes
  useEffect(() => {
    handleCoverageListenerChange(coverageListener);
  }, [coverageListener]);

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
      <canvas ref={canvasRef} className="my-world-canvas"></canvas>
      {showClearButton && (
        <button onClick={handleClearData}>Clear Data</button>
      )}
      <span class="loader"></span>
    </div>
  );
};

export default CreateMapPage;
