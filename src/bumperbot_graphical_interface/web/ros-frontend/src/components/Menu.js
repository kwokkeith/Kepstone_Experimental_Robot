// src/components/Menu.js
import React, { useState, useEffect } from 'react';
import './Menu.css';
import ROSLIB from 'roslib';
import { startNode } from '../rosService';

const Menu = ({ showPage, isOpen }) => {
  const [isMapsOpen, setIsMapsOpen] = useState(false);

  const handleMapsClick = () => {
    setIsMapsOpen((prev) => !prev);
  };

  useEffect(()=> {
    const startNodeService = startNode();
  },[]);

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

  const handleCreateMapClick = () => {
    const mapName = prompt("What is the name of the Map?");
    if (mapName) {
      showPage('create-map', mapName); // Pass map name to showPage
      handleStartNode();
    }
  };

  useEffect(() => {
    if (!isOpen) {
      setIsMapsOpen(false);
    }
  }, [isOpen]);

 

  return (
    <div id="menu" className={`slide-menu ${isOpen ? 'active' : ''}`}>
      <a href="#" onClick={() => showPage('main')}>Dashboard</a>
      <a href="#" onClick={handleMapsClick}>
        Maps
      </a>
      
      {isMapsOpen && (
        <div className="sub-menu">
          <a href="#" onClick={() => showPage('my-world')}>My World</a>
          <a href="#" onClick={handleCreateMapClick}>
            <span className="plus-icon">+</span> Create Map
          </a>
        </div>
      )}
      
      <a href="#" onClick={() => showPage('settings')}>Settings</a>
    </div>
  );
};

export default Menu;
