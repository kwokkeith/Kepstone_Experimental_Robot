import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import ros from '../rosService';

const SensorReadings = () => {
  const [sensorData, setSensorData] = useState(0);

  useEffect(() => {
    const sensorTopic = new ROSLIB.Topic({
      ros,
      name: '/sensor_data', // Replace with actual topic
      messageType: 'std_msgs/Float32'
    });

    sensorTopic.subscribe((message) => {
      setSensorData(message.data);
    });

    return () => {
      sensorTopic.unsubscribe();
    };
  }, []);

  return <div><h2>Sensor Reading: {sensorData}</h2></div>;
};

export default SensorReadings;
