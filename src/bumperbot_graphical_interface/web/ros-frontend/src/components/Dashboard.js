// src/components/Dashboard.js
import React from 'react';
import RobotStatus from './RobotStatus';
import BatteryStatus from './BatteryStatus';
import SensorReadings from './SensorReadings';
import MapDisplay from './MapDisplay';
import WaypointSetter from './WaypointSetter';
import NavigationStatus from './NavigationStatus';
import SystemHealth from './SystemHealth';
import ErrorLog from './ErrorLog';
import Settings from './Settings';
import Notifications from './Notifications';
import FullScreenToggle from './FullScreenToggle';

const Dashboard = () => (
  <div>
    <h1>Robot Dashboard</h1>
    <RobotStatus />
    <BatteryStatus />
    <SensorReadings />
    <MapDisplay />
    <WaypointSetter />
    <NavigationStatus />
    <SystemHealth />
    <ErrorLog />
    <Settings />
    <Notifications />
    <FullScreenToggle />
  </div>
);

export default Dashboard;
