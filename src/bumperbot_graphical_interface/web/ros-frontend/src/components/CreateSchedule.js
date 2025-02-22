// components/CreateSchedule.js
import React, { useState, useEffect } from 'react';
import './CreateSchedule.css';
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button
import leftchevron from '../assets/icons/chevron-left.svg';
import checkIcon from '../assets/icons/check.svg';

const CreateSchedule = ({showPage, selectedDate}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

    const handleCreateScheduleBack = () => {
        // console.log("Back to Schedules");
        sessionStorage.removeItem('selected-date');
        showPage("schedules");

    };

    const handleCreateScheduleConfirm = () => {
      const scheduleName = document.querySelector('.text-input.schedule-form').value;
      const scheduleTime = document.querySelector('.time-picker.schedule-form').value;
      const scheduleRepeat = document.querySelector('.dropdown.schedule-form').value;

      console.log('Schedule Name:', scheduleName);
      console.log('Schedule Time:', scheduleTime);
      console.log('Schedule Repeat:', scheduleRepeat);

      if (scheduleName === "" || scheduleTime === "") {
        alert("Please fill out all fields");
        return;
      }
      console.log("Create Schedule");
    };

    const handleSelectZoneClick = () => {
      showPage("select-zone");
    };

  return (
    <div className="create-schedule-container">

      <div className="create-schedule-middle-content">
        <div className="create-schedule-top-content">
          <h1>Create Schedule</h1>
          <h1>{selectedDate}</h1>
        </div>
        <div className="form-container">
          <input type="text" placeholder="Schedule Name" className="text-input schedule-form" />
          <input type="time" className="time-picker schedule-form" />
          <button className="select-zone-button schedule-form" onClick = {handleSelectZoneClick}>Select Zones</button>
          <select className="dropdown schedule-form">
            <option value="NoRepeat">Does Not Repeat</option>
            <option value="Weekly">Weekly</option>
            <option value="Monthly">Monthly</option>
          </select>
        </div>

        <div className="create-schedule-bottom-buttons">
          <button className = "create-schedule-back-button bottom-button hover"onClick={handleCreateScheduleBack}>
            <img src={leftchevron} alt="Back" />
          </button>
          <button className = "create-schedule-confirm-button bottom-button hover" onClick={handleCreateScheduleConfirm}>
            <img src={checkIcon} alt="Confirm" />
          </button>
        </div>
        
      </div>

    </div>
  );
};

export default CreateSchedule;
