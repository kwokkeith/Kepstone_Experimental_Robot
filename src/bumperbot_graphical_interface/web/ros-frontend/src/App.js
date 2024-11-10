// src/App.js
import React, { useState } from 'react';
import './index.css'; // For global styles
import LoginPage from './components/LoginPage';
import MainPage from './components/MainPage';
import Menu from './components/Menu';
import SettingsPage from './components/SettingsPage';
import FullScreenToggle from './components/FullScreenToggle';
import MyWorldPage from './components/MyWorldPage';
import CreateMapPage from './components/CreateMapPage';

function App() {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [currentPage, setCurrentPage] = useState('main');
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [mapName, setMapName] = useState(''); //State to store map name

  const handleLoginSuccess = () => {
    setIsLoggedIn(true);
    setIsMenuOpen(true); // Show menu when login is successful
  };

  const showPage = (page, name = '') => {
    setCurrentPage(page);
    setMapName(name); //Store map name if provided
    setIsMenuOpen(false); // Close menu after selecting a page
  };

  const toggleMenu = () => {
    setIsMenuOpen((prev) => !prev);
  };

  return (
    <div className="App">
      <FullScreenToggle />
      {!isLoggedIn ? (
        <LoginPage onLoginSuccess={handleLoginSuccess} />
      ) : (
        <>
          <Menu showPage={showPage} isOpen={isMenuOpen} />
          <button onClick={toggleMenu} className="menu-toggle-btn">☰</button>
          {currentPage === 'main' && <MainPage />}
          {currentPage === 'settings' && <SettingsPage />}
          {currentPage === 'my-world' && <MyWorldPage />}
          {currentPage === 'create-map' && <CreateMapPage mapName={mapName}/>} {/* Pass mapName to CreateMapPage */}
        </>
      )}
    </div>
  );
}

export default App;
