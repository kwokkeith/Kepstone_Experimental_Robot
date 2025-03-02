// components/SelectZone.js
import React, { useState, useEffect } from 'react';
import './SelectZone.css';
import leftchevron from '../assets/icons/chevron-left.svg';
import checkIcon from '../assets/icons/check.svg';
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button
import ROSLIB from 'roslib';
import { startNode, publishmapName, publishEditState } from '../rosService';

const SelectZone = ({showPage}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

    const [zones, setZones] = useState([]);
    const [zonesToDelete, setZonesToDelete] = useState([]);
    const [zoneSequence, setZoneSequence] = useState([]);

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

  const handleNewZoneClick = async () => {
    try {
      // Force re-fetching config data to avoid stale cache issues
      const response = await fetch(`http://localhost:5000/api/config?timestamp=${new Date().getTime()}`);
      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }
      const result = await response.json();
      const dbmapNames = result.data.map(entry => entry.map_name);
  
      const mapName = prompt("What is the name of the Map?");
      if (dbmapNames.includes(mapName)) {
        alert("Map name already exists. Please enter a new map name.");
      } else if (!mapName) {
        alert("Please enter a valid map name");
      } else {
        showPage('create-map', mapName); // Pass map name to showPage
        handleStartNode(mapName);
      }
    } catch (err) {
      alert(`Error fetching config: ${err.message}`);
    }
  };


  const handleZoneClick = (entry) => {
    // showPage('my-world', entry.map_name);
    setZoneSequence(prevSequence => [...prevSequence, entry.map_name]);
    console.log("Updated Zone Sequence:", [...zoneSequence, entry.map_name]);
  };

  const handleBackClick = () => {
    showPage('create-schedule');
  };

  const handleCheckClick = () => {
    showPage('create-schedule');
  };


  return (
    <div className="page-container">
      {/* Map Actions at the top */}
      <div className="map-actions">
  
        {/* New Zone Button */}
        <button className="new-zone-button" onClick={handleNewZoneClick}>
          <img src={plusIcon} alt="Plus" className="plus-icon" /> New Zone
        </button>
      </div>
  
      <div className="header-container">
        <h1>Select Zone</h1>
      </div>

      {/* Content Grid below Map Actions */}
      <div className="content-grid" id="grid-container">
        {isLoading ? (
          <p>Loading...</p>
        ) : error ? (
          <p>Error: {error}</p>
        ) : (
          data.map((entry, index) => {
            let imageSrc = null;
            try {
              imageSrc = require(`../../public/temp_zone/image_${entry.map_name}.png`);
            } catch (error) {
              // imageSrc remains null if image is not found
            }
            let zoneIndex = zoneSequence.indexOf(entry.map_name);
  
            if (imageSrc) {
              return (
                <div
                  key={index}
                  className={`grid-item ${
                    zonesToDelete.includes(entry.map_name) ? 'selected-zone' : ''
                  }`}
                  onClick={() => handleZoneClick(entry)}
                  style={{
                    backgroundImage: `url(${imageSrc})`,
                    backgroundSize: 'cover',
                    backgroundPosition: 'center',
                    cursor: 'pointer'
                  }}
                >
                  <h3>{entry.map_name}</h3>
                  {zoneIndex !== -1 && (
                    <div className="sequence-indicator">{zoneIndex + 1}</div>
                  )}
                </div>
              );
            } else {
              return (
                <div
                  key={index}
                  className={`grid-item ${
                    zonesToDelete.includes(entry.map_name) ? 'selected-zone' : ''
                  }`}
                  onClick={() => handleZoneClick(entry)}
                  style={{
                    backgroundColor: '#E0E8FC',
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    justifyContent: 'center',
                    cursor: 'pointer'
                  }}
                >
                  <p>Map image not found</p>
                  <h3>{entry.map_name}</h3>
                  {zoneIndex !== -1 && (
                    <div className="sequence-indicator">{zoneIndex + 1}</div>
                  )}
                </div>
              );
            }
          })
        )}
      </div>
      <div className='bottom-button-container'>
        <button className = "create-schedule-back-button bottom-button hover" onClick={handleBackClick}>
            <img src={leftchevron} alt="Back" />
          </button>
        <button className = "create-schedule-confirm-button bottom-button hover" onClick={handleCheckClick}>
          <img src={checkIcon} alt="Confirm" />
        </button>
      </div>
    </div>
  );
};

export default SelectZone;
