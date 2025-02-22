import React, { useState, useEffect } from 'react';
import './Header.css';
import bellIcon from '../assets/icons/bell.svg';
import menuIcon from '../assets/icons/Menu.svg';
import slidersIcon from '../assets/icons/sliders.svg';
import homeIcon from '../assets/icons/home.svg';

const Header = ({ currentPage = "Home", showPage }) => { // Pass showPage as prop
  // console.log("Current Page in Header:", currentPage); // Debugging

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
        <div className="icon hover" onClick={() => currentPage !== "MainPage" && showPage('main')}>
          {currentPage === "MainPage" ? (
            <img src={menuIcon} alt="Menu" />
          ) : (
            <img src={homeIcon} alt="Home" />
          )}
        </div>
      </div>
    </header>
  );
};

export default Header;
