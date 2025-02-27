// Import necessary modules and assets
import React, { useState, useEffect, useRef } from 'react';
import './MainPage.css';
import moveIcon from '../assets/icons/move.svg';
import mapIcon from '../assets/icons/map.svg';
import calendarIcon from '../assets/icons/calendar.svg';
import diagnosticsIcon from '../assets/icons/diagnostics.svg';
import batteryIcon from '../assets/icons/zap.svg';
import Header from './Header'; // Import the Header component
import renderIMG from '../assets/images/Render3d.png';
import {battery_percentage_listener} from '../rosService';

const MainPage = ({ showPage }) => { // Ensure showPage is received as a prop
  const [percentage, setPercentage] = useState(20); // Battery percentage
  const [circumference, setCircumference] = useState(0);
  const [strokeDashoffset, setStrokeDashoffset] = useState(0);
  const circleRef = useRef(null);
  const [nextSchedule, setNextSchedule] = useState('schedule1'); // Next schedule

  //Battery States
  const [batteryColor, setBatteryColor] = useState('#2BE1A9');
  const [prevPercentage, setPrevPercentage] = useState(percentage);
  const [batteryStatus, setBatteryStatus] = useState("Battery in use");

  // Only run with dv8 robot.
  // useEffect(() => {
  //   const battery_percentage_srv = battery_percentage_listener();
  //   const handleNewBatteryPercentage = (msg) => {
  //     const labeledMessage = `/sam/battery_state/percentage ${msg.data}`;
  //     setPercentage(msg.data);
  //   };

  //   battery_percentage_srv.subscribe(handleNewBatteryPercentage);

  //   return () => {
  //     battery_percentage_srv.unsubscribe(handleNewBatteryPercentage);
  //   };
  // }, []);
  
  useEffect(() => {
    if (circleRef.current) {
      const radius = circleRef.current.r.baseVal.value;
      const circumference = 2 * Math.PI * radius;
      setCircumference(circumference);
      setStrokeDashoffset(circumference - (percentage / 100) * circumference);
      circleRef.current.style.strokeDasharray = circumference;
      circleRef.current.style.strokeDashoffset = strokeDashoffset;
    }
  }, [percentage, circumference]);

  useEffect(() => {
    let newColor;
    if (percentage <= 20) {
      newColor = "#FF5255";
    } else if (percentage < 50) {
      newColor = "#F8E16A";
    } else {
      newColor = "#2BE1A9";
    }
    setBatteryColor(newColor);
  
    if (percentage > prevPercentage) {
      setBatteryStatus("Battery is charging");
    } else {
      setBatteryStatus("Battery in use");
    }
    setPrevPercentage(percentage);
  }, [percentage]);

  

  return (
    <div id="main-page" className="page">
      {/* Header Component */}
      <Header currentPage="MainPage" /> {/* Pass the currentPage prop */}

      {/* Middle Content */}
      <div className="middle-content">
        <div className="middle-content-left">
          <img src={renderIMG} alt="Render3d" />
        </div>

        <div className="middle-content-right-top">
          <div className="battery-status">
            <div className="progress-ring">
              <svg width="100%" height="100%">
                <circle
                  ref={circleRef}
                  cx="50%"
                  cy="50%"
                  r="40%"
                  stroke={batteryColor}
                />
              </svg>
              <div className="middle-icon">
                  <img src={batteryIcon} alt="Battery" />
              </div>
            </div>
            <div className="battery-text">
              <h3>{percentage}%</h3>
              <p>{batteryStatus}</p> 
              {/* TODO: Change to Battery Charging when read from rostopic */}
            </div>
          </div>
          {/* <p>Top Right</p> */}
        </div>

        <div className="middle-content-right-bottom">
          <div className = "next-schedule">
            <div className="schedule-icon">
              <img src={calendarIcon} alt="Calendar" />
            </div>
            <div className="schedule-text">
              <h3>{nextSchedule}</h3>
              <p>03/11/2024 18:00</p> 
              {/* TODO: Change to Battery Charging when read from rostopic */}
            </div>
          </div>
        </div>

      </div>

      {/* Function Selector at the Bottom */}
      <div className="function-selector-container">
        <div className="main-icons">

          {/* add the onClick later for each showPage */}
          <div className="icon-with-label hover">
            <div className="icon">
              <img src={moveIcon} alt="Move" />
            </div>
            <span className="icon-label">Move</span>
          </div>
          
          <div className="icon-with-label hover" onClick={() => showPage('create-map2')} style={{ cursor: 'pointer' }}>
            <div className="icon">
              <img src={mapIcon} alt="Zone" />
            </div>
            <span className="icon-label">Zones</span>
          </div>

          <div className="icon-with-label hover" onClick = {() => showPage('schedules')} style={{ cursor: 'pointer' }}>
            <div className="icon">
              <img src={calendarIcon} alt="Schedule" />
            </div>
            <span className="icon-label">Schedules</span>
          </div>

          <div className="icon-with-label hover" onClick={() => showPage('diagnostics')} style={{ cursor: 'pointer' }} >
            <div className="icon">
              <img src={diagnosticsIcon} alt="Diagnostics"/>
            </div>
            <span className="icon-label">Diagnostics</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default MainPage;
