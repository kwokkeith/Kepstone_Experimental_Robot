import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import ros from '../rosService';

const BatteryStatus = () => {
  const [battery, setBattery] = useState(100);

  useEffect(() => {
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: '/battery_level', // Replace with actual topic
      messageType: 'std_msgs/Float32'
    });

    batteryTopic.subscribe((message) => {
      setBattery(message.data);
    });

    return () => {
      batteryTopic.unsubscribe();
    };
  }, []);

  return <div><h2>Battery Level: {battery}%</h2></div>;
};

export default BatteryStatus;
