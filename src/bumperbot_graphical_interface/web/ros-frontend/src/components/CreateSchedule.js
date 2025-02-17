// components/CreateMapPage2.js
import React, { useState, useEffect } from 'react';
import './CreateSchedule.css';
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button


const CreateSchedule = ({showPage}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

    const [showConfirm, setShowConfirm] = useState(false);
    const [zonesToDelete, setZonesToDelete] = useState([]);

  
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
    <div className="create-schedule-container">

      <div className="create-schedule-middle-content">
        <div className="create-schedule-top-content">
        
        </div>
        <div className="create-schedule">

        </div>

        <div className="create-schedule-bottom-buuttons">

        </div>
        
      </div>

    </div>
  );
};

export default CreateSchedule;
