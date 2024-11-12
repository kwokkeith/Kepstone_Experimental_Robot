// src/components/MainPage.js
import React from 'react';
import './MainPage.css';

const MainPage = () => (
  <div id="main-page" className="page">
    <main>
      <h2>Robot Status</h2>
      <div className="status">
        <p>Battery: <span id="battery-status">100%</span></p>
        <p>Status: <span id="robot-status">Docking Station</span></p>
        <p>Health: <span id="robot-health">Full Health</span></p>
      </div>
    </main>
  </div>
);

export default MainPage;
