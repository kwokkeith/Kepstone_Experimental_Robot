// src/components/MyWorldPage.js
import React from 'react';
import './MyWorldPage.css'; // Import the CSS file

const MyWorldPage = () => {
  return (
    <div className="my-world-page">
      <h2>My World</h2>
      <img src={`${process.env.PUBLIC_URL}/my_world_map1.png`} alt="My World Map" className="my-world-map" />
    </div>
  );
};

export default MyWorldPage;
