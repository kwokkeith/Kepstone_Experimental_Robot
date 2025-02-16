// components/CreateMapPage2.js
import React, { useState, useEffect } from 'react';
import './CreateMapPage2.css';
// import trashIcon from '../assets/icons/trash.svg'; // Import the trash icon
// import trashhoverIcon from '../assets/icons/trash-hover.svg'; // Import the trash icon
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button
import ROSLIB from 'roslib';
import { startNode, publishmapName, publishEditState } from '../rosService';

const CreateMapPage2 = ({showPage}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

    const [showConfirm, setShowConfirm] = useState(false);
    const [zonesToDelete, setZonesToDelete] = useState([]);
    const [zones, setZones] = useState([]);

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
      
    }, [zonesToDelete]);

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

  const handleTrashClick = () => {
    setShowConfirm(true);
    setZonesToDelete([]);
    if (showConfirm) {
      setZonesToDelete([]);
      setShowConfirm(false);
    }
  };

  const handleZoneClick = (entry) => {
    if (showConfirm) {
      // Toggle the zone in the deletion list (using map_name as identifier)
      setZonesToDelete((prev) =>
        prev.includes(entry.map_name)
          ? prev.filter((name) => name !== entry.map_name)
          : [...prev, entry.map_name]
      );
    } else {
      // Normal behavior when deletion mode is NOT active
      showPage('my-world', entry.map_name);
    }
  };

  const handleConfirmClick = async () => {
    // For demonstration, simply log the list
    console.log('Zones to delete:', zonesToDelete);

    for (let zone of zonesToDelete) {
      try {
        const response = await fetch(`http://localhost:5000/api/maps/${encodeURIComponent(zone)}`, {
          method: 'DELETE',
        });
  
        if (!response.ok) {
          console.error(`Failed to delete ${zone}:`, response.statusText);
        } else {
          const result = await response.json();
          console.log(`Deleted ${zone}:`, result);
        }
      } catch (error) {
        console.error(`Error deleting ${zone}:`, error);
      }
    }

    // You can then iterate through zonesToDelete and call your delete API for each.
    setShowConfirm(false);
    setZonesToDelete([]);
  };

  return (
    <div className="page-container">
      {/* Map Actions at the top */}
      <div className="map-actions">
        {showConfirm && (
          <button className="confirm-button" onClick={handleConfirmClick}>
            Confirm
          </button>
        )}
        {/* Trash Icon Button */}
        <button className="icon-button-trash-zone" onClick={handleTrashClick}>
          {/* <img src={trashIcon} alt="Trash" className="icon-trash-zone" /> */}
        </button>
  
        {/* New Zone Button */}
        <button className="new-zone-button" onClick={handleNewZoneClick}>
          <img src={plusIcon} alt="Plus" className="plus-icon" /> New Zone
        </button>
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
  
            if (imageSrc) {
              return (
                <div
                  key={index}
                  className={`grid-item ${
                    showConfirm && zonesToDelete.includes(entry.map_name) ? 'selected-zone' : ''
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
                  {/* Additional entry details can go here */}
                </div>
              );
            } else {
              return (
                <div
                  key={index}
                  className={`grid-item ${
                    showConfirm && zonesToDelete.includes(entry.map_name) ? 'selected-zone' : ''
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
                  {/* Additional entry details can go here */}
                </div>
              );
            }
          })
        )}
      </div>
    </div>
  );
};

export default CreateMapPage2;
