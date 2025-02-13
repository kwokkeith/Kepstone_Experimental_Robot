// Import necessary modules and assets
import React, { useState, useEffect, useRef } from 'react';
import './MainPage.css';
import bellIcon from '../assets/icons/bell.svg';
import menuIcon from '../assets/icons/Menu.svg';
import slidersIcon from '../assets/icons/sliders.svg';
import moveIcon from '../assets/icons/move.svg';
import mapIcon from '../assets/icons/map.svg';
import calendarIcon from '../assets/icons/calendar.svg';
import diagnosticsIcon from '../assets/icons/diagnostics.svg';
import batteryIcon from '../assets/icons/zap.svg';

const MainPage = ({ showPage }) => {
  const [currentTime, setCurrentTime] = useState('');
  const [currentDate, setCurrentDate] = useState('');
  const [percentage, setPercentage] = useState(10); //USE THIS TO SET PERCENTAGE OF BATTERY
  const [circumference, setCircumference] = useState(0);
  const [strokeDashoffset, setStrokeDashoffset] = useState(0);
  const circleRef = useRef(null);

  useEffect(() => {
    const updateDateTime = () => {
      const now = new Date();
      const time = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
      const date = now.toLocaleDateString('en-GB', {
        day: '2-digit',
        month: 'long',
        year: 'numeric',
      });
      setCurrentTime(time);
      setCurrentDate(date);
    };

    updateDateTime();
    const timer = setInterval(updateDateTime, 1000);

    return () => clearInterval(timer);
  }, []);

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

  const strokeColor = percentage < 20 ? '#FF5255' : '#2BE1A9';

  return (
    <div id="main-page" className="page">
      {/* Top Section */}
      <header className="top-section">
        <div className="time-date-container">
          <div className="time-display">{currentTime}</div>
          <div className="separator"></div>
          <div className="date-display">{currentDate}</div>
        </div>

        <div className="top-icons">
          <div className="icon hover">
            <img src={slidersIcon} alt="Sliders" />
          </div>
          <div className="icon hover">
            <img src={bellIcon} alt="Notification" />
          </div>
          <div className="icon hover">
            <img src={menuIcon} alt="Menu" />
          </div>
        </div>
      </header>

      <div className="middle-content">

        <div className="middle-content-left">
          {/* <p>Map</p> */}
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
                  stroke={strokeColor}
                />
              </svg>
              <div className="middle-icon">
                  <img src={batteryIcon} alt="Battery" />
              </div>
            </div>
            <div className="battery-text">
              <h3>{percentage}%</h3>
              <p>Battery in use</p> 
              {/* TODO: Change to Battery Charging when read from rostopic */}
            </div>
          </div>
          {/* <p>Top Right</p> */}
        </div>

        <div className="middle-content-right-bottom">
          {/* <p>Bottom Right</p> */}
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

          <div className="icon-with-label hover">
            <div className="icon">
              <img src={calendarIcon} alt="Schedule" />
            </div>
            <span className="icon-label">Schedules</span>
          </div>

          <div className="icon-with-label hover">
            <div className="icon">
              <img src={diagnosticsIcon} alt="Diagnostics" />
            </div>
            <span className="icon-label">Diagnostics</span>
          </div>

        </div>
      </div>
    </div>
  );
};

export default MainPage;
