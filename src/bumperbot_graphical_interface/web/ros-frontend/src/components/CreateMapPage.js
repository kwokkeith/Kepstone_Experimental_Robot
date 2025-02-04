// src/components/CreateMapPage.js
import React, { useEffect, useRef, useState } from 'react';
import './CreateMapPage.css';
import ROSLIB from 'roslib';
import { coverage_listener, publishPoint, publishStartPoint, startNode, publishEditState, publishmapName, publishContourAngles, publishDbShutdownState} from '../rosService';

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
  const [contourAngles, setContourAngles] = useState({});
  

  // React States for database fetching
  const [configData, setConfigData] = useState([]);
  const [polyData, setPolyData] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // ==========================
  // React useEffect Hooks called whenever a dependent state changes
  // ==========================

  // Log the map name when the component loads
  useEffect(() => {
    if (mapName) {
      console.log(`Creating new map: ${mapName}`);
      fetchConfigData();
      fetchPolyData();
    }
  },[mapName]);

  // Function to fetch config data from sqlite db
  const fetchConfigData = async () => {
    try {
        const response = await fetch('http://localhost:5000/api/config');
      
        if (!response.ok) {
          throw new Error('Network response error');
        }

        // Parse the response as JSON
        const result = await response.json();

        setConfigData(result.data);
        setIsLoading(false);
    } catch (err) {
        setError(err.message);
        setIsLoading(false);
    }
  };

  const fetchPolyData = async () => {
    try {
        const response = await fetch('http://localhost:5000/api/bcdpolycontourdata');
      
        if (!response.ok) {
          throw new Error('Network response error');
        }

        // Parse the response as JSON
        const result = await response.json();

        setPolyData(result.data);
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
      // console.log('Received message on coverage topic:', message.data);
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
    const storedFourPointsSet = sessionStorage.getItem('fourPointsSet');
    if (!storedFourPointsSet) {    
      // If 4 points are drawn, publish the points to the ROS topic
      console.log(startpoints);
      if (!startpoints && points.length === 4){
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
        sessionStorage.setItem('fourPointsSet', JSON.stringify(true));
      } 
    }  
    
  }, [points]);

  //if starting 4 points alr set, then get and publish the start points
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

  // After your state hooks are defined:
  useEffect(() => {
    console.log('Contour Angles updated:', contourAngles);
    sessionStorage.setItem('contourAngles', JSON.stringify(contourAngles));
  }, [contourAngles]);

  // ==========================
  // FUNCTIONS
  // ==========================

  const getStartPoint = (mapName) => {
    const map = configData.find((item => item.map_name === mapName));
    return map ? map.start_point : null;
  };

  const getRoiPoints = (mapName) => {
    const map = configData.find((item => item.map_name === mapName));
    return map ? map.roi_points : null;
  };

  const getPolygonBoundingCoordinates = (mapName) => {
    const map = configData.find((item => item.map_name === mapName));
    return map ? map.polygonBounding_coordinates : null;
  };

  //TODO: CREATE A NEW FUNCTION FOR THIS 
  const getBcdPolygonContourCoordinates = (mapName) => {
    // Filter for matching mapName
    const items = polyData.filter(item => item.map_name === mapName);
    // Sort items based on polygon_index (or derive from polygon_id if not available)
    const sortedItems = items.sort((a, b) => {
      // Use polygon_index if available; otherwise use polygon_id-1
      const indexA = a.polygon_index !== undefined ? a.polygon_index : (a.polygon_id - 1);
      const indexB = b.polygon_index !== undefined ? b.polygon_index : (b.polygon_id - 1);
      return indexA - indexB;
    });
    // Map each sorted item to an array of coordinate strings
    return sortedItems.map(item =>
      item.bcdPolygonContour_coordinates
        .split('\n')
        .map(line => line.trim())
        .filter(line => line !== '')
    );
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
      const bcdCleaningPathCoordinates = getBcdPolygonContourCoordinates(mapName); // TODO: Rename variable from bcdCLeaning... to bcdPolyContour...
      sessionStorage.setItem('allBCDPolyContours', JSON.stringify(bcdCleaningPathCoordinates));
      
      if (bcdCleaningPathCoordinates) {
        const bcdCleaningPathLists = bcdCleaningPathCoordinates.map(sublist =>
          sublist.map(line => {
            const [px, py] = line.split(' ').map(Number);
            return { x: px, y: py };
          })
        );
  
        let foundInBCDList = null;
        bcdCleaningPathLists.forEach((polygon, index) => {
          if (isPointInPolygon({ x, y }, polygon)) {
            foundInBCDList = index;
          }
        });
        
        if (foundInBCDList !== null) {
          console.log(`Point is inside the cleaning path list at index: ${foundInBCDList}`);

          // Draw the contour found in the list
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

          setTimeout(() => {
            const angle = Number(prompt('Set new cleaning angle in degrees:'));

            setContourAngles(prev => {
              const updated = { ...prev };
              if (!updated[foundInBCDList]) {
                // If no entry exists for this polygon, create one with the new object
                updated[foundInBCDList] = [{ polygon_index: foundInBCDList, angle: angle }];
              } else {
                // Look for an existing object with the same polygon_index
                const index = updated[foundInBCDList].findIndex(item => item.polygon_index === foundInBCDList);
                if (index !== -1) {
                  // Replace the existing object with the new input
                  updated[foundInBCDList][index] = { polygon_index: foundInBCDList, angle: angle };
                } else {
                  // If it doesn't exist (unlikely, since the key corresponds to the index), push the new object
                  updated[foundInBCDList].push({ polygon_index: foundInBCDList, angle: angle });
                }
              }
              return updated;
            });
          }, 0);

        } else {
          console.log('Point is not inside any cleaning path list.');
        }
      }
      setPoints([]);
    }
    
  };

  const handleCoverageListenerChange = (newData) => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');

    // Draw each contour separately
    ctx.strokeStyle = 'magenta';
    ctx.lineWidth = 1;
  
    // Split the newData into separate contours
    const contoursData = newData.trim().split('],').map(contour => contour.replace('[', '').replace(']', '').trim());
  
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

  const handleEditStartNode = (mapName) => {
    const startNodeService = startNode();
    if (startNodeService) {
      const request = new ROSLIB.ServiceRequest({});
      startNodeService.callService(request, function(result) {
        // console.log('Service call result:', result);
        publishmapName({ mapName });
        startNodeService.ros.close(); // Close the ROS connection

        //Publish Edit State === FALSE only after the service call is successful
        setTimeout(() => {
          publishEditState({ editState: true });
        }, 1000);


      }, function(error) {
        console.error('Service call failed:', error);
        startNodeService.ros.close(); // Close the ROS connection
      });
    }
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
        sessionStorage.removeItem('fourPointsSet');
        sessionStorage.removeItem('allBCDPolyContours');
        showPage('main')
        // Optionally, refresh data after deletion
        fetchConfigData();
        fetchPolyData();
      } else {
        console.error('Error deleting data:', result.error || result.message);
      }
    } catch (error) {
      console.error('Error:', error);
    }
  };
  
  const handleEdit = async () => {
    // console.log('Edit button clicked');
    try {
      const response = await fetch(`http://localhost:5000/api/maps/${encodeURIComponent(mapName)}`, {
        method: 'DELETE',
      });

      const result = await response.json();

      if (response.ok) {
        setEditMapState(true);
        publishDbShutdownState({ dbState: true });

      } else {
        console.error('Error deleting data:', result.error || result.message);
      }
    } catch (error) {
      console.error('Error:', error);
    }

  };
  const handleSave = () => {
    sessionStorage.removeItem('coverageListener');
    fetchConfigData();
    fetchPolyData();
    showPage('main');
    publishDbShutdownState({ dbState: true });
  };

  const handleSaveEdit = () => {
    // Reset State back to initial state for handleCanvasClick
    setEditMapState(false);
    setStartPoints(false);
    sessionStorage.removeItem('coverageListener');
    sessionStorage.removeItem('fourPointsSet');
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(imageRef.current, 0, 0, canvas.width, canvas.height);

    handleEditStartNode(mapName); //TODO: Fix This so it does not launch db_publisher.cpp again 
    
    setTimeout(() => {
      //console.log("old roi points in the points array",points)
      // Get ROI_Points and StartPoints from the database that was set initially.
      const roi_points = getRoiPoints(mapName);
      const roiPointsArray = roi_points.trim().split('\n').map(line => {
        const [x, y] = line.split(' ').map(Number);
        return { x, y };
      });
      const newRoiPoints = [...points, ...roiPointsArray];
      
      setPoints(newRoiPoints);
      // console.log("new roi points in the points array",points)
      // console.log('ROI Points:', roiPointsArray);

      setTimeout(() => {
        const startpoint = getStartPoint(mapName);
        const startPointArray = startpoint.trim().split('\n').map(line => {
          const [x, y] = line.split(' ').map(Number);
          return { x, y };
        });
        const newStartPoint = [...points, ...startPointArray];
        setPoints(newStartPoint);
      },3000);

      // const bcdPolyContoursString = sessionStorage.getItem('allBCDPolyContours');
      const contourAngles = sessionStorage.getItem('contourAngles');

      setTimeout(() => {
        publishContourAngles({data: contourAngles});
      },4000)
    
    }, 5000);

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
        {editMapState && (
          <button onClick={handleSaveEdit} className = "save-edit-btn">Save Edit</button>
        )}
      </div>
    )}
      <span className="loader"></span>
    </div>
  );
};

export default CreateMapPage;