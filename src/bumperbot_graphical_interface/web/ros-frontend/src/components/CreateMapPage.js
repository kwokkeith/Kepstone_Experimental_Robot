// src/components/CreateMapPage.js
import React, { useEffect } from 'react';
import './CreateMapPage.css';

const CreateMapPage = ({ mapName }) => {
  // Log the map name when the component loads
  useEffect(() => {
    if (mapName) {
      console.log(`Creating new map: ${mapName}`);
    }
  }, [mapName]);

  return (
    <div className="create-map-page">
      <h2>{mapName}</h2>
      <p>Please wait...</p> {/* Display a simple "Please wait" message */}
    </div>
  );
};

export default CreateMapPage;
