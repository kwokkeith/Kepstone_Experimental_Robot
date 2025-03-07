// components/CreateSchedule.js
import React, { useState, useEffect } from 'react';
import './CreateSchedule.css';
import plusIcon from '../assets/icons/plus.svg';  // Assuming you have a plus.svg icon, or replace it with a "+" character inside the button
import leftchevron from '../assets/icons/chevron-left.svg';
import checkIcon from '../assets/icons/check.svg';

const CreateSchedule = ({showPage, selectedDate}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);
    const [scheduleName, setScheduleName] = useState('');
    const [scheduleTime, setScheduleTime] = useState('');
    const [repeatOption, setRepeatOption] = useState('NoRepeat');
    const [zoneSequence, setZoneSequence] = useState([]);

    useEffect(() => {
      const storedName = JSON.parse(sessionStorage.getItem('schedule-name'));
      const storedTime = JSON.parse(sessionStorage.getItem('schedule-time'));
      const storedRepeat = JSON.parse(sessionStorage.getItem('repeat-option'));
      const storedZoneSequence = JSON.parse(sessionStorage.getItem('zoneSequence'));
  
      if (storedName) setScheduleName(storedName);
      if (storedTime) setScheduleTime(storedTime);
      if (storedRepeat) setRepeatOption(storedRepeat);
      if (storedZoneSequence) setZoneSequence(storedZoneSequence);
    }, []);

    function getUUID() {
      const v = "0123456789abcdef";
      let res = "";
  
      for (let i = 0; i < 8; i++) {
          res += v[Math.floor(Math.random() * 16)];
      }
      res += "-";
      for (let i = 0; i < 4; i++) {
          res += v[Math.floor(Math.random() * 16)];
      }
      res += "-4"; // UUID version 4
      for (let i = 0; i < 3; i++) {
          res += v[Math.floor(Math.random() * 16)];
      }
      res += "-";
      res += v[8 + Math.floor(Math.random() * 4)]; // UUID variant
      for (let i = 0; i < 3; i++) {
          res += v[Math.floor(Math.random() * 16)];
      }
      res += "-";
      for (let i = 0; i < 12; i++) {
          res += v[Math.floor(Math.random() * 16)];
      }
  
      return res;
    }

    const formatDateForDB = (dateString) => {
      const date = new Date(dateString);
      
      // Get day, month (0-indexed), and year
      const day = date.getDate().toString().padStart(2, '0');
      const month = (date.getMonth() + 1).toString().padStart(2, '0'); // +1 because months are 0-indexed
      const year = date.getFullYear();
      
      // Return in the format "DD-MM-YYYY" required by the database
      return `${day}-${month}-${year}`;
    };

    const formatTimeForDB = (timeString) => {
      return `${timeString}:00`;
    };

    const handleCreateScheduleBack = () => {
        // console.log("Back to Schedules");
        // sessionStorage.setItem('selected-date',"''");
        showPage("schedules");
        sessionStorage.removeItem('schedule-name');
        sessionStorage.removeItem('schedule-time');
        sessionStorage.removeItem('repeat-option');
        sessionStorage.removeItem('zoneSequence');

    };

    const handleCreateScheduleConfirm = () => {
      const scheduleName = document.querySelector('.text-input.schedule-form').value;
      const scheduleTime = document.querySelector('.time-picker.schedule-form').value;
      const repeatOption = document.querySelector('.dropdown.schedule-form').value;
      const formattedDate = formatDateForDB(selectedDate);
      const formattedTime = formatTimeForDB(scheduleTime);
  
      if (scheduleName === "" || scheduleTime === "" || zoneSequence.length === 0) {
          alert("Please fill out all fields");
          return;
      }
      // console.log(scheduleName, formattedDate, formattedTime, repeatOption);
  
      // zoneSequence.forEach((zone, index) => {
      //     console.log(`Index: ${index}, Zone: ${zone}`);
      // });
  
      fetchConfigIds(zoneSequence)
          .then(configIds => {
              // console.log('Config IDs:', configIds);
  
              const scheduleId = getUUID(); // Generate a unique schedule_id
  
              // Insert schedule into Schedule_Table
              return fetch('http://localhost:5000/api/schedule', {
                  method: 'POST',
                  headers: {
                      'Content-Type': 'application/json'
                  },
                  body: JSON.stringify({
                      schedule_id: scheduleId,
                      schedule_name: scheduleName,
                      start_time: formattedTime,
                      start_date: formattedDate,
                      recurrence: repeatOption
                  })
              })
              .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                return response.json();
              })
              .then(() => {
                  const jobId = getUUID(); // Generate a unique job_id

                  // Insert jobs and job links
                  const jobPromises = zoneSequence.map((zone, index) => {
                      const config_id = configIds.find(config => config.map_name === zone).config_id;
                      
  
                      // Insert job into Jobs_Table
                      return fetch('http://localhost:5000/api/jobs', {
                          method: 'POST',
                          headers: {
                              'Content-Type': 'application/json'
                          },
                          body: JSON.stringify({
                              job_id:jobId,
                              config_id,
                              sequence_no: index
                          })
                      })
                      .then(() => {
                          // Insert job order into Job_Order_Table
                          return fetch('http://localhost:5000/api/job_order', {
                              method: 'POST',
                              headers: {
                                  'Content-Type': 'application/json'
                              },
                              body: JSON.stringify({
                                  job_id:jobId,
                                  config_id
                              })
                          });
                      })
                      .then(() => {
                          // Insert schedule-job link into Schedule_Job_Link
                          return fetch('http://localhost:5000/api/schedule_job_link', {
                              method: 'POST',
                              headers: {
                                  'Content-Type': 'application/json'
                              },
                              body: JSON.stringify({
                                  schedule_id:scheduleId,
                                  sequence_no: index,
                                  job_id:jobId
                              })
                          });
                      })
                      .then(() => {
                          // Insert into ContainsTable
                          return fetch('http://localhost:5000/api/contains', {
                              method: 'POST',
                              headers: {
                                  'Content-Type': 'application/json'
                              },
                              body: JSON.stringify({
                                  job_id:jobId,
                                  sequence_no: index
                              })
                          });
                      });
                  });

                  return Promise.all(jobPromises)
                      .then(() => {
                        // Insert job status into JobStatus
                        return fetch('http://localhost:5000/api/job_status', {
                            method: 'POST',
                            headers: {
                                'Content-Type': 'application/json'
                            },
                            body: JSON.stringify({
                                job_id:jobId,
                                status: 'scheduled'
                            })
                        });
                    });
              });
          })
          .then(() => {
              console.log('All jobs and links inserted successfully');
          })
          .catch(error => {
              console.error('Error:', error);
          });

          showPage("schedules");
    };

    const fetchConfigIds = async (zoneSequence) => {
      try {
          const response = await fetch('http://localhost:5000/api/config_ids', {
              method: 'POST',
              headers: {
                  'Content-Type': 'application/json'
              },
              body: JSON.stringify({ zoneSequence })
          });
  
          if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
          }
  
          const contentType = response.headers.get('content-type');
          if (!contentType || !contentType.includes('application/json')) {
              throw new TypeError("Received non-JSON response");
          }
  
          const data = await response.json();
          return data.data;
      } catch (error) {
          console.error('Error:', error);
          throw error;
      }
    };

    const handleSelectZoneClick = () => {
      showPage("select-zone");
      sessionStorage.setItem('schedule-name', JSON.stringify(scheduleName));
      sessionStorage.setItem('schedule-time', JSON.stringify(scheduleTime));
      sessionStorage.setItem('repeat-option', JSON.stringify(repeatOption));
    };

  return (
    <div className="create-schedule-container">

      <div className="create-schedule-middle-content">
        <div className="create-schedule-top-content">
          <h1>Create Schedule</h1>
          <h1>{selectedDate}</h1>
        </div>
        <div className="form-container">
          <input
            type="text"
            placeholder="Schedule Name"
            className="text-input schedule-form"
            value={scheduleName}
            onChange={e => setScheduleName(e.target.value)}
          />
          <input
            type="time"
            className="time-picker schedule-form"
            value={scheduleTime}
            onChange={e => setScheduleTime(e.target.value)}
          />
          <button className="select-zone-button schedule-form" onClick={handleSelectZoneClick}>
            {zoneSequence.length > 0 ? "Zones Selected" : "Select Zones"}
          </button>
          <select
            className="dropdown schedule-form"
            value={repeatOption}
            onChange={e => setRepeatOption(e.target.value)}
          >
            <option value="NoRepeat">Does Not Repeat</option>
            <option value="Weekly">Weekly</option>
            <option value="Monthly">Monthly</option>
          </select>
        </div>

        <div className="create-schedule-bottom-buttons">
          <button className = "create-schedule-back-button bottom-button hover"onClick={handleCreateScheduleBack}>
            <img src={leftchevron} alt="Back" />
          </button>
          <button className = "create-schedule-confirm-button bottom-button hover" onClick={handleCreateScheduleConfirm}>
            <img src={checkIcon} alt="Confirm" />
          </button>
        </div>
        
      </div>

    </div>
  );
};

export default CreateSchedule;
