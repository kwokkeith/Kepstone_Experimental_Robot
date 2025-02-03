// components/CreateMapPage2.js
import React from 'react';
import './CreateMapPage2.css';
import trashIcon from '../assets/icons/trash.svg'; // Import the trash icon
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button

const CreateMapPage2 = () => {
  const handleNewZoneClick = () => {
    console.log('New Zone button clicked');
    // Additional logic for creating a new zone can go here
  };

  const handleTrashClick = () => {
    console.log('Trash button clicked');
    // Additional logic for deleting or clearing zones can go here
  };

  return (
    <div className="map-actions">
      {/* Trash Icon Button */}
      <button className="icon-button" onClick={handleTrashClick}>
        <img src={trashIcon} alt="Trash" className="icon" />
      </button>

      {/* New Zone Button */}
      <button className="new-zone-button" onClick={handleNewZoneClick}>
        <img src={plusIcon} alt="Plus" className="plus-icon" /> New Zone
      </button>
    </div>
  );
};

export default CreateMapPage2;
