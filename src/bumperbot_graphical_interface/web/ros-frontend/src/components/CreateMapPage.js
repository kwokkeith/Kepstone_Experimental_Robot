// src/components/CreateMapPage.js
import React, { useEffect, useRef, useState } from 'react';
import './CreateMapPage.css';
import ROSLIB from 'roslib';
import { coverage_listener, publishPoint, publishStartPoint} from '../rosService';

const CreateMapPage = ({ mapName, showPage }) => {
  // ==========================
  // React States
  // ==========================

  // React States for creating map using ROS
  const imageRef = useRef(null);
  const canvasRef = useRef(null);
  const [points, setPoints] = useState([]);
  const [startpoints, setStartPoints] = useState(false);
  const [coverageListener, setcoverageListener] = useState('');
  const [showButtonContainer, setShowButtonContainer] = useState(false);
  const [createMapState, setCreateMapState] = useState(true);
  const [editMapState, setEditMapState] = useState(false);
  

  // React States for database fetching
  const [data, setData] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // ==========================
  // React useEffect Hooks called whenever a dependent state changes
  // ==========================

  // Log the map name when the component loads
  useEffect(() => {
    if (mapName) {
      console.log(`Creating new map: ${mapName}`);
      fetchData();
    }
  },[mapName] );

  // Function to fetch data from backend sqlite NOT ROS
  const fetchData = async () => {
    try {
      const response = await fetch('http://localhost:5000/api/data');
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      const result = await response.json();
      setData(result.data);
      setIsLoading(false);
    } catch (err) {
      setError(err.message);
      setIsLoading(false);
    }
  };

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
        setCreateMapState(false); // Disable map creation FOR post map creation
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

  const getPolygonBoundingCoordinates = (mapName) => {
    const map = data.find((item => item.map_name === mapName));
    return map ? map.polygonBounding_coordinates : null;
  };

  const getBcdPolygonContourCoordinates = (mapName) => {
    const map = data.find((item => item.map_name === mapName));
    return map ? map.bcdPolygonContour_coordinates : null;
  };

  const isPointInPolygon = (point, polygon) => {
    let isInside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const xi = polygon[i].x, yi = polygon[i].y;
        const xj = polygon[j].x, yj = polygon[j].y;

        const intersect = ((yi > point.y) !== (yj > point.y)) &&
            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
        if (intersect) isInside = !isInside;
    }
    return isInside;
  };

  const handleCanvasClick = (event) => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const rect = canvas.getBoundingClientRect();
    const x = Math.round(event.clientX - rect.left);
    const y = Math.round(event.clientY - rect.top);

    // Add the new point to the array
    const newPoints = [...points, { x, y }];
    setPoints(newPoints);
    if(createMapState){
      // Redraw the image and all points
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(imageRef.current, 0, 0, canvas.width, canvas.height);
      newPoints.forEach(point => {
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(point.x, point.y, 2, 0, 2 * Math.PI); // Draws a small circle of radius 5
        ctx.fill();
      });
    } else if(editMapState && !createMapState){
      const polygonCoordinates = getPolygonBoundingCoordinates(mapName);
      const bcdCleaningPathCoordinates = getBcdPolygonContourCoordinates(mapName);
      // console.log('Polygon coordinates:', polygonCoordinates); //Debugging
      if (polygonCoordinates) {
        const polygonPoints = polygonCoordinates.trim().split('\n').map(line => {
          const [px, py] = line.split(' ').map(Number);
          return { x: px, y: py };
        });

        // if (isPointInPolygon({ x, y }, polygonPoints)) {
        //   console.log('Point is inside polygonContour:', polygonCoordinates);
        // } else {
        //   console.log('Point is outside polygonContour.');
        // }
      }
      // CLEANING PATH COORDINATES ARE NOT THE SAME AS THE POLYGON BCD COORDINATES, redo this part
      if (bcdCleaningPathCoordinates) {
        const bcdCleaningPathLists = bcdCleaningPathCoordinates.split('],[').map(sublist => {
          return sublist.replace(/[\[\]]/g, '').trim().split('\n').map(line => {
            const [px, py] = line.split(' ').map(Number);
            return { x: px, y: py };
          });
        });
  
        let foundInBCDList = null;
        bcdCleaningPathLists.forEach((polygon, index) => {
          if (isPointInPolygon({ x, y }, polygon)) {
            foundInBCDList = index;
          }
        });
  
        if (foundInBCDList !== null) {
          // console.log(`Point is inside the cleaning path list at index: ${foundInBCDList}`);
          // Draw the points found in the list
          const listPoints = bcdCleaningPathLists[foundInBCDList];
          if (listPoints.length > 0) {
            ctx.beginPath();
            ctx.moveTo(listPoints[0].x, listPoints[0].y);
            listPoints.forEach(point => {
              ctx.lineTo(point.x, point.y);
            });
            ctx.closePath();
            ctx.strokeStyle = 'lime';
            ctx.lineWidth = 2;
            ctx.stroke();
          }
        
          // // Draw the points
          // listPoints.forEach(point => {
          //   ctx.fillStyle = 'green';
          //   ctx.beginPath();
          //   ctx.arc(point.x, point.y, 2, 0, 2 * Math.PI);
          //   ctx.fill();
          // });
        } else {
          console.log('Point is not inside any cleaning path list.');
        }
      }

    }
    
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

  const handleClearDataWrapper = async () => {
    if (!mapName) {
      console.error('Map name is not defined.');
      return;
    }

    try {
      const response = await fetch(`http://localhost:5000/api/maps/${encodeURIComponent(mapName)}`, {
        method: 'DELETE',
      });

      const result = await response.json();

      if (response.ok) {
        //alert(result.message);
        alert("Cancelled map creation successfully");
        sessionStorage.removeItem('coverageListener');
        showPage('main')
        // Optionally, refresh data after deletion
        fetchData();
      } else {
        console.error('Error deleting data:', result.error || result.message);
      }
    } catch (error) {
      console.error('Error:', error);
    }
  };
  
  const handleEdit = () => {
    // console.log('Edit button clicked');
    setEditMapState(true);
  };
  const handleSave = () => {
    sessionStorage.removeItem('coverageListener');
    showPage('main')
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
        <button onClick={handleClearDataWrapper} className="cancel-btn">Cancel</button>
        <button onClick={handleEdit} className="edit-btn">Edit</button>
        <button onClick={handleSave} className="save-btn">Save</button>
      </div>
    )}
      <span className="loader"></span>
    </div>
  );
};

export default CreateMapPage;