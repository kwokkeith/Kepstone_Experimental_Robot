import React, { useEffect, useRef, useState } from 'react';
import './MyWorldPage.css';
import plusIcon from '../assets/icons/play.svg';
import pauseIcon from '../assets/icons/pause.svg';
import stopIcon from '../assets/icons/stop.svg';
// import ROSLIB from 'roslib';
import { callWriteWaypointsService, triggerStartCoverageService, triggerStopCoverageService } from '../rosService';

const MyWorldPage = ({mapName}) => {
  const canvasRef = useRef(null);
  const imageRef = useRef(null);
  const [data, setData] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [contoursList, setContoursList] = useState([]);
  const [waypointsList, setWaypointsList] = useState([]);
  const [navigation_waypointsList, setNavigationWaypointsList] = useState([]);
  const [imageLoaded, setImageLoaded] = useState(false);
  const [isJobStarted, setIsJobStarted] = useState(false);
  const [isJobPaused, setIsJobPaused] = useState(false);

  // ==================================
  // useEffect Hooks for fetching data
  // ==================================
  
  useEffect(() => {
    const fetchConfigData = async () => {
      try {
        const response = await fetch('http://localhost:5000/api/config');
        if (!response.ok) {
          throw new Error(`HTTP error! Status: ${response.status}`);
        }
        const result = await response.json();
        setData(result.data);
      } catch (err) {
        setError(err.message);
      } finally {
        setIsLoading(false);
      }
    };
    
    fetchConfigData();
    
  }, []);

  // ==================================
  // useEffect Hooks for drawing on canvas
  // ==================================

  useEffect(() => {
    if (!data.length || !imageLoaded) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const image = imageRef.current;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw image
    canvas.width = image.width;
    canvas.height = image.height;
    ctx.drawImage(image, 0, 0);

    // Draw contours
    ctx.strokeStyle = 'magenta';
    ctx.lineWidth = 1.3;
    contoursList.forEach(contour => {
      ctx.beginPath();
      contour.forEach((point, index) => {
        if (index === 0) {
          ctx.moveTo(point.x, point.y);
        } else {
          ctx.lineTo(point.x, point.y);
        }
      });
      ctx.closePath();
      ctx.stroke();
    });

    // Draw waypoints
    ctx.strokeStyle = 'blue';
    ctx.lineWidth = 1;
    waypointsList.forEach(waypoint => {
      ctx.beginPath();
      waypoint.forEach((point, index) => {
        if (index === 0) {
          ctx.moveTo(point.x, point.y);
        } else {
          ctx.lineTo(point.x, point.y);
        }
      });
      ctx.closePath();
      ctx.stroke();
    });

  }, [data, imageLoaded, contoursList, waypointsList]);

  // ===========================================
  // useEffect Hooks for processing fetched data
  // ===========================================

  useEffect(() => {
    if (!data || data.length === 0) {
      console.warn('Data not loaded yet');
      return;
    }
    const entry = data.find(item => item.map_name === mapName);
    if (!entry) {
      console.error(`No entry found for map: ${mapName}`);
      return;
    }  
  
    // Process contours only if the string is non-empty
    if (entry.polygonBounding_coordinates && entry.polygonBounding_coordinates.trim() !== '') {
      const fetched_contours = entry.polygonBounding_coordinates.trim();
      const cleaned_contours = fetched_contours.endsWith(',')
        ? fetched_contours.slice(0, -1)
        : fetched_contours;
  
      const contours = cleaned_contours
        .replace(/^\[|\]$/g, '')
        .split('],[')
        .map(contourStr =>
          contourStr
            .split('\n')
            .filter(coord => coord.trim() !== '')
            .map(coord => {
              const [x, y] = coord.trim().split(' ').map(Number);
              return { x, y };
            })
        );
      setContoursList(contours);
    } else {
      console.warn('No polygonBounding_coordinates found.');
    }
  
    // Process waypoints only if the string is non-empty
    if (entry.cleaning_path_coordinates && entry.cleaning_path_coordinates.trim() !== '') {
      const fetched_waypoints = entry.cleaning_path_coordinates.trim();
      const cleaned_waypoints = fetched_waypoints.endsWith(',')
        ? fetched_waypoints.slice(0, -1)
        : fetched_waypoints;
  
      const waypoints = cleaned_waypoints
        .replace(/^\[|\]$/g, '')
        .split('],[')
        .map(waypointStr =>
          waypointStr
            .split('\n')
            .filter(coord => coord.trim() !== '')
            .map(coord => {
              const [x, y] = coord.trim().split(' ').map(Number);
              return { x, y };
            })
        );
      setWaypointsList(waypoints);
    } else {
      console.warn('No cleaning_path_coordinates found.');
    }
  
    // Process navigation waypoints
    if (entry.navigation_waypoints && entry.navigation_waypoints.trim() !== '') {
      const fetched_navigation_waypoints = entry.navigation_waypoints.trim();
      setNavigationWaypointsList(fetched_navigation_waypoints);
    } else {
      console.warn('No navigation_waypoints found.');
    }
  }, [data, mapName]);

  const handleImageLoad = () => {
    setImageLoaded(true);
  };

  const handleStartJobClick = () => {
    setIsJobStarted(true);
    setIsJobPaused(false);
    //TODO: Get navigation_waypoints from config table
    callWriteWaypointsService({ waypointsList: navigation_waypointsList });
    setTimeout(() => {
      triggerStartCoverageService();
    }, 4000);
  };

  const handlePauseJobClick = () => {
    setIsJobPaused(true);
  };
  
  const handleStopJobClick = () => {
    setIsJobStarted(false);
    setIsJobPaused(false);

    triggerStopCoverageService();
  };

  // =============================
  // React rendered html component
  // =============================

  return (
    <div className="my-world-container">
      <div className="my-world-left">
        <h2>{mapName}</h2>
        {isLoading && <p>Loading...</p>}
        {error && <p>Error: {error}</p>}
        <img
          ref={imageRef}
          alt="My World Map"
          className="my-world-map"
          style={{ display: 'none' }}
          src={`${process.env.PUBLIC_URL}/my_world_map2.png`}
          onLoad={handleImageLoad}
        />
        <canvas ref={canvasRef} className="my-world-canvas"></canvas>
      </div>
      <div className="my-world-right">
        {/* Insert additional text or content here */}
        <h1>Coverage Cleaning mode</h1>
        <div className = "bottom-center-right-buttons">
        {isJobStarted ? (
          <>
            <button className="pause-job-button" onClick={handlePauseJobClick}>
              <img src={pauseIcon} alt="pause" className="pause-icon" />
            </button>
            {isJobPaused ? (
              <button className="start-job-button" onClick={handleStartJobClick}>
                <img src={plusIcon} alt="start" className="start-icon" />
              </button>
            ) : (
              <button className="stop-job-button" onClick={handleStopJobClick}>
                <img src={stopIcon} alt="stop" className="stop-icon" />
              </button>
            )}
          </>
        ) : (
          <button className="start-job-button" onClick={handleStartJobClick}>
            <img src={plusIcon} alt="start" className="start-icon" />
          </button>
        )}
        </div>
      </div>
    </div>
  );
};

export default MyWorldPage;

