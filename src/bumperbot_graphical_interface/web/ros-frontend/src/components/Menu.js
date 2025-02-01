// src/components/Menu.js
import React, { useState, useEffect } from 'react';
import './Menu.css';
import ROSLIB from 'roslib';
import { startNode, publishmapName, publishEditState } from '../rosService';

const Menu = ({ showPage, isOpen }) => {
  const [isMapsOpen, setIsMapsOpen] = useState(false);
  const [data, setData] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  const handleMapsClick = () => {
    setIsMapsOpen((prev) => !prev);
  };

  const handleStartNode = (mapName) => {
    const startNodeService = startNode();
    if (startNodeService) {
      const request = new ROSLIB.ServiceRequest({});
      startNodeService.callService(request, function(result) {
        console.log('Service call result:', result);
        publishmapName({ mapName });
        startNodeService.ros.close(); // Close the ROS connection

        //Publish Edit State === FALSE only after the service call is successful
        setTimeout(() => {
          publishEditState({ editState: false });
        }, 1000);


      }, function(error) {
        console.error('Service call failed:', error);
        startNodeService.ros.close(); // Close the ROS connection
      });
    }
  };

  useEffect(() => {
    const fetchData = async () => {
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
    
    fetchData();
    
  }, []);
  
  const handleCreateMapClick = () => {
    const dbmapNames = data.map(entry => entry.map_name);
    // console.log(dbmapNames); // Debugging
    const mapName = prompt("What is the name of the Map?");
    if (dbmapNames.includes(mapName)) {
      alert("Map name already exists. Please enter a new map name.");
    } else if (!mapName) {
      alert("Please enter a valid map name");
    } else if (mapName) {
      showPage('create-map', mapName); // Pass map name to showPage
      handleStartNode(mapName);
    }
  };
/*
  useEffect(() => {
    if (!isOpen) {
      setIsMapsOpen(false);
    }
  }, [isOpen]);
*/
 

  return (
    <div id="menu" className={`slide-menu ${isOpen ? 'active' : ''}`}>
      <a onClick={() => showPage('main')}>Dashboard</a>
      <a onClick={handleMapsClick}>Maps</a>
      
      {isMapsOpen && (
        <div className="sub-menu">
          <a onClick={() => showPage('my-world')}>My World</a>
          <a onClick={handleCreateMapClick}>
            <span className="plus-icon">+</span> Create Map
          </a>
        </div>
      )}
      
      <a onClick={() => showPage('settings')}>Settings</a>
    </div>
  );
};

export default Menu;
