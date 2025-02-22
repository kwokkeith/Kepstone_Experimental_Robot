// src/App.js
import React, { useEffect, useState } from 'react';
import './index.css'; // For global styles
import LoginPage from './components/LoginPage';
import MainPage from './components/MainPage';
import Menu from './components/Menu';
import SettingsPage from './components/SettingsPage';
import FullScreenToggle from './components/FullScreenToggle';
import MyWorldPage from './components/MyWorldPage';
import CreateMapPage from './components/CreateMapPage';
import CreateMapPage2 from './components/CreateMapPage2';
import Diagnostics from './components/Diagnostics';

import Header from './components/Header'; // Import Header
import Schedules from './components/Schedules';
import CreateSchedule from './components/CreateSchedule';
import SelectZone from './components/SelectZone';


function App() {
  // Gets the session stored value, (in case react re-renders)
  // ==========================
  // Persistent storage
  // ==========================
  const getInitialLoginState = () => {
    const storedLoginState = sessionStorage.getItem('isLoggedIn');
    return storedLoginState ? JSON.parse(storedLoginState) : false;
  };
  
  const getCurrentPageState = () => {
    const storedCurrentPageState = sessionStorage.getItem('currentPage');
    return storedCurrentPageState ? JSON.parse(storedCurrentPageState) : 'main';
  }

  const getMapNameState = () => {
    const storedMapNameState = sessionStorage.getItem('mapName');
    return storedMapNameState ? JSON.parse(storedMapNameState) : '';
  }

  const getSelectedDateState = () => {
    const storedSelectedDateState = sessionStorage.getItem('selected-date');
    return storedSelectedDateState ? JSON.parse(storedSelectedDateState) : '';
  }

  // ==========================
  // React States
  // ==========================
  const [isLoggedIn, setIsLoggedIn] = useState(getInitialLoginState);
  const [currentPage, setCurrentPage] = useState(getCurrentPageState);
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [mapName, setMapName] = useState(getMapNameState); //State to store map name
  const [selectedDate, setSelectedDate] = useState(getSelectedDateState);


  // ==========================
  // React useEffect Hooks called whenever a dependent state changes
  // ==========================
  useEffect(() => {
    // Update sessionStorage when isLoggedIn state changes
    sessionStorage.setItem('isLoggedIn', JSON.stringify(isLoggedIn));
  },[isLoggedIn])

  useEffect(() => {
    // Update sessionStorage when currentPage state changes
    sessionStorage.setItem('currentPage', JSON.stringify(currentPage));
  },[currentPage])

  useEffect(() => {
    // Update sessionStorage when currentPage state changes
    sessionStorage.setItem('mapName', JSON.stringify(mapName));
  },[mapName])

  useEffect(() => {
    // Update sessionStorage when currentPage state changes
    sessionStorage.setItem('selected-date', JSON.stringify(selectedDate));
  },[selectedDate])

  // ==========================
  // Callback functions 
  // ==========================
  const handleLoginSuccess = () => {
    setIsLoggedIn(true);
  };

  const showPage = (page, name = '') => {
    setCurrentPage(page);
    setMapName(name);     //Store map name if provided
    setIsMenuOpen(false); // Close menu after selecting a page
  };

  const toggleMenu = () => {
    setIsMenuOpen((prev) => !prev);
  };

  const showCreateSchedulePage = (page, dateSelected=selectedDate) => {
    setCurrentPage(page);
    setSelectedDate(dateSelected);
    setIsMenuOpen(false);
  }

  // ==========================
  // React rendered html component
  // ==========================
  return (
    <div className="App">
      <FullScreenToggle />
      {!isLoggedIn ? (
        <LoginPage onLoginSuccess={handleLoginSuccess} />
      ) : (
        <>
          {/* ✅ Add Header and pass `currentPage` & `showPage` */}
          <Header currentPage={currentPage} showPage={showPage} />
          <Menu showPage={showPage} isOpen={isMenuOpen} />
          <button onClick={toggleMenu} className="menu-toggle-btn">☰</button>
          {currentPage === 'main' && <MainPage showPage={showPage} />}
          {currentPage === 'settings' && <SettingsPage />}
          {currentPage === 'my-world' && <MyWorldPage mapName={mapName}/>}
          {currentPage === 'create-map' && <CreateMapPage mapName={mapName} showPage={showPage}/>} {/* Pass mapName to CreateMapPage */}
          {currentPage === 'create-map2' && <CreateMapPage2 showPage={showPage}/>}
          {currentPage === 'diagnostics' && <Diagnostics showPage={showPage}/>}
          {currentPage === 'schedules' && <Schedules showPage={showPage} showCreateSchedulePage={showCreateSchedulePage}/>}
          {currentPage === 'create-schedule' && <CreateSchedule showPage={showPage} selectedDate={selectedDate}/>}
          {currentPage === 'select-zone' && <SelectZone showPage={showPage}/>}

        </>
      )}
    </div>
  );
}

export default App;
