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
    const [scheduleName, setScheduleName] = useState('');
    const [scheduleTime, setScheduleTime] = useState('');
    const [repeatOption, setRepeatOption] = useState('NoRepeat');

    useEffect(() => {
      const storedName = JSON.parse(sessionStorage.getItem('schedule-name'));
      const storedTime = JSON.parse(sessionStorage.getItem('schedule-time'));
      const storedRepeat = JSON.parse(sessionStorage.getItem('repeat-option'));
  
      if (storedName) setScheduleName(storedName);
      if (storedTime) setScheduleTime(storedTime);
      if (storedRepeat) setRepeatOption(storedRepeat);
    }, []);

    const formatDateForDB = (dateString) => {
      const date = new Date(dateString);
      
      // Get day, month (0-indexed), and year
      const day = date.getDate().toString().padStart(2, '0');
      const month = (date.getMonth() + 1).toString().padStart(2, '0'); // +1 because months are 0-indexed
      const year = date.getFullYear();
      
      // Return in the format "DD-MM-YYYY" required by the database
      return `${day}-${month}-${year}`;
    };

    const formatTimeForDB = (timeString) => {
      return `${timeString}:00`;
    };

    const handleCreateScheduleBack = () => {
        // console.log("Back to Schedules");
        // sessionStorage.setItem('selected-date',"''");
        showPage("schedules");
        sessionStorage.removeItem('schedule-name');
        sessionStorage.removeItem('schedule-time');
        sessionStorage.removeItem('repeat-option');

    };

    const handleCreateScheduleConfirm = () => {
      const scheduleName = document.querySelector('.text-input.schedule-form').value;
      const scheduleTime = document.querySelector('.time-picker.schedule-form').value;
      const repeatOption = document.querySelector('.dropdown.schedule-form').value;
      const formattedDate = formatDateForDB(selectedDate);
      const formattedTime = formatTimeForDB(scheduleTime);

      if (scheduleName === "" || scheduleTime === "") {
        alert("Please fill out all fields");
        return;
      }
      console.log(scheduleName, formattedDate, formattedTime, repeatOption);
    };

    const handleSelectZoneClick = () => {
      showPage("select-zone");
      sessionStorage.setItem('schedule-name', JSON.stringify(scheduleName));
      sessionStorage.setItem('schedule-time', JSON.stringify(scheduleTime));
      sessionStorage.setItem('repeat-option', JSON.stringify(repeatOption));
    };

  return (
    <div className="create-schedule-container">

      <div className="create-schedule-middle-content">
        <div className="create-schedule-top-content">
          <h1>Create Schedule</h1>
          <h1>{selectedDate}</h1>
        </div>
        <div className="form-container">
          <input
            type="text"
            placeholder="Schedule Name"
            className="text-input schedule-form"
            value={scheduleName}
            onChange={e => setScheduleName(e.target.value)}
          />
          <input
            type="time"
            className="time-picker schedule-form"
            value={scheduleTime}
            onChange={e => setScheduleTime(e.target.value)}
          />
          <button className="select-zone-button schedule-form" onClick={handleSelectZoneClick}>
            Select Zones
          </button>
          <select
            className="dropdown schedule-form"
            value={repeatOption}
            onChange={e => setRepeatOption(e.target.value)}
          >
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
