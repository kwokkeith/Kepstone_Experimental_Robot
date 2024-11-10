// src/components/SettingsPage.js
import React from 'react';
import './SettingsPage.css';

const SettingsPage = () => {
  const localizeRobot = () => {
    alert("Local Localization Placeholder");
  };

  const adjustBrightness = () => {
    alert("Brightness Adjustment Placeholder");
  };

  return (
    <div id="settings-page" className="page">
      <h2>Settings</h2>
      <button onClick={localizeRobot}>Local Localization</button>
      <button onClick={adjustBrightness}>Screen Brightness</button>
    </div>
  );
};

export default SettingsPage;
