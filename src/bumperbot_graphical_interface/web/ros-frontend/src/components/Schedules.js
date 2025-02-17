import React, { useState, useEffect } from 'react';
import './Schedules.css';
// import ROSLIB from 'roslib';
// import trashIcon from '../assets/icons/trash.svg';
import plusIcon from '../assets/icons/plus.svg'; 
import ROSLIB from 'roslib';
import { startNode, publishmapName, publishEditState } from '../rosService';
// import chevronLeftIcon from '../assets/icons/chevron-left.svg';
// import chevronRightIcon from '../assets/icons/chevron-right.svg';

const Schedules = ({showPage, showCreateSchedulePage}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

    const [date, setDate] = useState(new Date());
    const [selectedDate, setSelectedDate] = useState(date.toDateString());
    const today = new Date();
    today.setHours(0, 0, 0, 0);

    // ---------------
    // REACT WEBHOOKS
    // ---------------
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

    // ----------
    // Functions
    // ----------

    const handleScheduleLeftClick = () => {
      handleMonthChange(-1);
    }

    const handleScheduleRightClick = () => {
      handleMonthChange(1);
    }

    const handleMonthChange = (change) => {
      const newDate = new Date(date);
      newDate.setMonth(newDate.getMonth() + change);
      setDate(newDate);
    };

    const handleTrashClick = () => {
      console.log("Trash clicked");
    };

    const handleDateClick = (date) => {
      setSelectedDate(date.toDateString());
    };

    const renderCalendar = () => {
      const days = [];
      const prevLastDay = new Date(date.getFullYear(), date.getMonth(), 0).getDate();
      const totalMonthDay = new Date(date.getFullYear(), date.getMonth() + 1, 0).getDate();
      const startWeekDay = new Date(date.getFullYear(), date.getMonth(), 1).getDay(); // weekday for the 1st
      const totalCalendarDay = 6 * 7;
      
      for (let i = 0; i < totalCalendarDay; i++) {
        const day = i - startWeekDay + 1;
        if (i < startWeekDay) { 
          // Previous month days
          days.push(
            <div key={i} className="padding-day">
              {prevLastDay - startWeekDay + i + 1}
            </div>
          );
        } else if (day > totalMonthDay) { 
          // Next month days
          days.push(
            <div key={i} className="padding-day">
              {day - totalMonthDay}
            </div>
          );
        } else {
          // Current month days
          const currentDay = new Date(date.getFullYear(), date.getMonth(), day);
          const dayClass = selectedDate === currentDay.toDateString() ? 'selected-day' : 'current-day';
          days.push(
            <div key={i} className={dayClass} onClick={() => handleDateClick(currentDay)}>
              {day}
            </div>
          );
        }
      }
  
      return days;
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

    const handleNewSchedule = () => {
      console.log(selectedDate)
      showCreateSchedulePage('create-schedule', selectedDate);

    };

    const handleLeftClick = () => {
      console.log("left clicked");
    };

    const handleRightClick = () => {
      console.log("right clicked");
    };
  
    return (
      <div className="schedules-page-container">
      <div className="schedules-middle-content">
        <div className="schedules-middle-content-left">
          <div className="card">
            <div className="calendar-toolbar">
              <button className="prev month-btn" onClick={handleScheduleLeftClick}>
                <i className="fas fa-chevron-left"></i>
              </button>
              <div className="current-month">
                {date.toLocaleDateString("en-US", { month: 'short', year: 'numeric' })}
              </div>
              <button className="next month-btn" onClick={handleScheduleRightClick}>
                <i className="fas fa-chevron-right"></i>
              </button>
            </div>
            <div className="calendar">
              <div className="weekdays">
                <div className="weekday-name">Sun</div>
                <div className="weekday-name">Mon</div>
                <div className="weekday-name">Tue</div>
                <div className="weekday-name">Wed</div>
                <div className="weekday-name">Thu</div>
                <div className="weekday-name">Fri</div>
                <div className="weekday-name">Sat</div>
              </div>
              <div className="calendar-days">
                {renderCalendar()}
              </div>
            </div>
          </div>
        </div>
  
        <div className="schedules-middle-content-right">
          <div className = "schedules-action-buttons">
            <button className="icon-button-trash-zone" onClick={handleTrashClick}>
              {/* <img src={trashIcon} alt="Trash" className="icon-trash-zone" /> */}
            </button>
    
            <button className="new-zone-button" onClick={handleNewZoneClick}>
              <img src={plusIcon} alt="Plus" className="plus-icon" /> New Zone
            </button>
    
            <button className="new-zone-button" onClick={handleNewSchedule}>
              <img src={plusIcon} alt="Plus" className="plus-icon" /> New Schedule
            </button>
          </div>
          <div className = "schedules-schedule-tab">
            
          </div>
        </div>
      </div>
    </div>
    );
  };

export default Schedules;