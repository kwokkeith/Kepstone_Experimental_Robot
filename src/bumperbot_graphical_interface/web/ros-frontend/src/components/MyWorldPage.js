import React, { useEffect, useRef, useState } from 'react';
import './MyWorldPage.css';

const MyWorldPage = () => {
  const canvasRef = useRef(null);
  const imageRef = useRef(null);
  const [data, setData] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [contoursList, setContoursList] = useState([]);
  const [waypointsList, setWaypointsList] = useState([]);
  const [imageLoaded, setImageLoaded] = useState(false);

  // ==================================
  // useEffect Hooks for fetching data
  // ==================================
  
  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch('http://localhost:5000/api/data');
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
    
    fetchData();
    
  }, []);

  // ==================================
  // useEffect Hooks for mapName
  // ==================================

  const mapName = data.length > 0 ? data[0].map_name : 'map_name';
  useEffect(() => {
    // Update sessionStorage when currentPage state changes
    sessionStorage.setItem('mapName', JSON.stringify(mapName));
  },[mapName])

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
    if (!data.length) return;

    // Process contours
    const fetched_contours = data[0].polygonBounding_coordinates.trim();
    const cleaned_contours = fetched_contours.endsWith(',') 
      ? fetched_contours.slice(0, -1)
      : fetched_contours;

    const contours = cleaned_contours
      .replace(/^\[|\]$/g, '')
      .split('],[')
      .map(contourStr => 
        contourStr.split('\n')
          .map(coord => {
            const [x, y] = coord.trim().split(' ').map(Number);
            return { x, y };
          })
      );

    setContoursList(contours);
    // console.log('Contours List:', contours); //Debugging tool to see the contours

    // Process waypoints
    const fetched_waypoints = data[0].cleaning_path_coordinates.trim();
    const cleaned_waypoints = fetched_waypoints.endsWith(',') 
      ? fetched_waypoints.slice(0, -1)
      : fetched_waypoints;

    const waypoints = cleaned_waypoints
      .replace(/^\[|\]$/g, '')
      .split('],[')
      .map(waypointStr => 
        waypointStr.split('\n')
          .map(coord => {
            const [x, y] = coord.trim().split(' ').map(Number);
            return { x, y };
          })
      );

    setWaypointsList(waypoints);
    // console.log('Waypoints List:', waypoints); //Debugging tool to see the waypoints
    
  }, [data]);

  const handleImageLoad = () => {
    setImageLoaded(true);
  };

  // =============================
  // React rendered html component
  // =============================

  return (
    <div className="my-world-page">
      <h2>{data.length > 0 ? data[0].map_name : 'Map Name'}</h2>
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
  );
};

export default MyWorldPage;

