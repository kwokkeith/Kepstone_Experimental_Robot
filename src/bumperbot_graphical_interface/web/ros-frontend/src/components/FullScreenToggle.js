// src/components/FullScreenToggle.js
import React, { useState } from 'react';
import './FullScreenToggle.css';

const FullScreenToggle = () => {
  const [isFullScreen, setIsFullScreen] = useState(false);

  const toggleFullScreen = () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen()
        .then(() => setIsFullScreen(true))
        .catch((err) => console.warn("Error enabling fullscreen mode:", err));
    } else {
      document.exitFullscreen()
        .then(() => setIsFullScreen(false))
        .catch((err) => console.warn("Error exiting fullscreen mode:", err));
    }
  };

  // Toggle this variable to `true` to hide the button
  const hideToggle = true; // Change to `false` to show the button

  return (
    <div className={`fullscreen-toggle ${hideToggle ? 'hidden' : ''}`}>
      <button onClick={toggleFullScreen} className="fullscreen-button">
        <i className={`fas ${isFullScreen ? 'fa-compress' : 'fa-expand'}`} />
      </button>
    </div>
  );
};

export default FullScreenToggle;