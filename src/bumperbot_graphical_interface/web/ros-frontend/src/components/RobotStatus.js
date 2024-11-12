import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import ros from '../rosService';

const RobotStatus = () => {
  const [status, setStatus] = useState('Unknown');

  useEffect(() => {
    const statusTopic = new ROSLIB.Topic({
      ros,
      name: '/robot_status', // Replace with actual topic
      messageType: 'std_msgs/String'
    });

    statusTopic.subscribe((message) => {
      setStatus(message.data);
    });

    return () => {
      statusTopic.unsubscribe();
    };
  }, []);

  return <div><h2>Robot Status: {status}</h2></div>;
};

export default RobotStatus;
