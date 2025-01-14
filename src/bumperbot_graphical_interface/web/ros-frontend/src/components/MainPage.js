// Import necessary modules and assets
import React, { useState, useEffect } from 'react';
import './MainPage.css';
import bellIcon from '../assets/icons/bell.svg';
import menuIcon from '../assets/icons/Menu.svg';
import slidersIcon from '../assets/icons/sliders.svg';

const MainPage = () => {
  // State variables for current time and date
  const [currentTime, setCurrentTime] = useState('');
  const [currentDate, setCurrentDate] = useState('');

  // Effect hook to update time and date dynamically
  useEffect(() => {
    const updateDateTime = () => {
      const now = new Date(); // Get the current date and time
      const time = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }); // Format time
      const date = now.toLocaleDateString('en-GB', {
        day: '2-digit',
        month: 'long',
        year: 'numeric',
      }); // Format date
      setCurrentTime(time); // Update state with formatted time
      setCurrentDate(date); // Update state with formatted date
    };

    updateDateTime(); // Set initial time and date on component mount
    const timer = setInterval(updateDateTime, 1000); // Update time every second

    return () => clearInterval(timer); // Clean up timer on component unmount
  }, []);

  return (
    <div id="main-page" className="page">
      {/* Top Section */}
      <header className="top-section">
        {/* Time and Date Display */}
        <div className="time-date-container">
          <div className="time-display">{currentTime}</div> {/* Display current time */}
          <div className="separator"></div> {/* Vertical separator between time and date */}
          <div className="date-display">{currentDate}</div> {/* Display current date */}
        </div>

        {/* Top-right Icons */}
        <div className="top-icons">
          <div className="icon">
            <img src={slidersIcon} alt="Sliders" /> {/* Sliders icon */}
          </div>
          <div className="icon">
            <img src={bellIcon} alt="Notification" /> {/* Notification icon */}
          </div>
          <div className="icon">
            <img src={menuIcon} alt="Menu" /> {/* Menu icon */}
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main>
        <h2>Robot Status</h2> {/* Main heading */}
        <div className="status">
          {/* Display robot status details */}
          <p>
            Battery: <span id="battery-status">100%</span>
          </p>
          <p>
            Status: <span id="robot-status">Docking Station</span>
          </p>
          <p>
            Health: <span id="robot-health">Full Health</span>
          </p>
        </div>
      </main>
    </div>
  );
};

export default MainPage;
