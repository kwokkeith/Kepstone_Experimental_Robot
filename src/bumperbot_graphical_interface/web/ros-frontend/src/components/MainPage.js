// Import necessary modules and assets
import React, { useState, useEffect } from 'react';
import './MainPage.css';
import bellIcon from '../assets/icons/bell.svg';
import menuIcon from '../assets/icons/Menu.svg';
import slidersIcon from '../assets/icons/sliders.svg';
import moveIcon from '../assets/icons/move.svg';
import mapIcon from '../assets/icons/map.svg';
import calendarIcon from '../assets/icons/calendar.svg';
import diagnosticsIcon from '../assets/icons/diagnostics.svg';

const MainPage = ({ showPage }) => {
  const [currentTime, setCurrentTime] = useState('');
  const [currentDate, setCurrentDate] = useState('');

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
          <div className="icon">
            <img src={slidersIcon} alt="Sliders" />
          </div>
          <div className="icon">
            <img src={bellIcon} alt="Notification" />
          </div>
          <div className="icon">
            <img src={menuIcon} alt="Menu" />
          </div>
        </div>
      </header>

      {/* Function Selector at the Bottom */}
      <div className="function-selector-container">
        <div className="main-icons">
          <div className="icon-with-label">
            <div className="icon">
              <img src={moveIcon} alt="Move" />
            </div>
            <span className="icon-label">Move</span>
          </div>
          <div className="icon-with-label">
            <div
              className="icon"
              onClick={() => showPage('create-map2')}
              style={{ cursor: 'pointer' }}
            >
              <img src={mapIcon} alt="Zone" />
            </div>
            <span className="icon-label">Zones</span>
          </div>
          <div className="icon-with-label">
            <div className="icon">
              <img src={calendarIcon} alt="Schedule" />
            </div>
            <span className="icon-label">Schedules</span>
          </div>
          <div className="icon-with-label">
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
