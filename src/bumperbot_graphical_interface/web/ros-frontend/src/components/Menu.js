// src/components/Menu.js
import React, { useState, useEffect } from 'react';
import './Menu.css';
import ROSLIB from 'roslib';
import { startNode, publishmapName } from '../rosService';

const Menu = ({ showPage, isOpen }) => {
  const [isMapsOpen, setIsMapsOpen] = useState(false);

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
      }, function(error) {
        console.error('Service call failed:', error);
        startNodeService.ros.close(); // Close the ROS connection
      });
    }
  };

  const handleCreateMapClick = () => {
    const mapName = prompt("What is the name of the Map?");
    if (mapName) {
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