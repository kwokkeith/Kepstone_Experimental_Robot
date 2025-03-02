import React, { useState, useEffect } from 'react';
import './Diagnostics.css';
// import ROSLIB from 'roslib';
import { sidebrush_speed_listener, sidebrush_position_listener, realsense_d455_listener_front, realsense_d455_listener_rear} from '../rosService';

const Diagnostics = ({showPage}) => {
    const [data, setData] = useState([]);
    const [error, setError] = useState(null);
    const [isLoading, setIsLoading] = useState(true);
    const [activeTab, setActiveTab] = useState('sidebrush');
    const [sidebrushSpeed, setSidebrushSpeed] = useState(0);
    const [sidebrushPosition, setSidebrushPosition] = useState(0);
    const [sidebrushMessages, setSidebrushMessages] = useState([]);
    const [imageDataFront, setImageDataFront] = useState(null);
    const [imageDataRear, setImageDataRear] = useState(null);
    const [imageFrontMessages, setImageFrontMessages] = useState([]);
    const [imageRearMessages, setImageRearMessages] = useState([]);
    const [enlargedImage, setEnlargedImage] = useState(imageDataFront);
    const [currentCameraMode,setCurrentCameraMode] = useState('front');

    useEffect(() => {
      if (currentCameraMode == 'front') {
        setEnlargedImage(imageDataFront);
      }
      else if (currentCameraMode == 'rear') {
        setEnlargedImage(imageDataRear);
      }
    }, [imageDataFront, imageDataRear]);

    useEffect(() => {
      const fetchData = async () => {
        try {
          const response = await fetch('http://localhost:5000/api/config');
          if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
          }
          const result = await response.json();
          setData(result.data);
        } catch (err) {
          setError(err.message);
        } finally {
          setIsLoading(false);
        }
      };
      
      fetchData();
      
    }, []);

    useEffect(() => {
      const sidebrush_speed_srv = sidebrush_speed_listener();
      const sidebrush_position_srv = sidebrush_position_listener(); 
    
      const handleNewSpeedMessage = (msg) => {
        const labeledMessage = `/sidebrush_controller/sidebrush_speed returned: ${msg.data}`;
        setSidebrushSpeed(msg.data);
        setSidebrushMessages(prevMessages => [...prevMessages, labeledMessage]);
      };

      const handleNewPositionMessage = (msg) => {
        const labeledMessage = `/sidebrush_controller/sidebrush_position returned : ${msg.data}`;
        setSidebrushPosition(msg.data); 
        setSidebrushMessages(prevMessages => [...prevMessages, labeledMessage]);
      };
    
      sidebrush_speed_srv.subscribe(handleNewSpeedMessage);
      sidebrush_position_srv.subscribe(handleNewPositionMessage); 
    
      return () => {
        sidebrush_speed_srv.unsubscribe(handleNewSpeedMessage);
        sidebrush_position_srv.unsubscribe(handleNewPositionMessage);
      };
    }, []);

    useEffect(() => {
      const realsense_listener_srv = realsense_d455_listener_front();
    
      const handleNewImageMessage = (msg) => {
        const labeledMessage = `/depth_camera_front/color/image_raw/compressed received: ${msg.header.stamp}`;
  
        // Adjust MIME type based on msg.format content
        const mimeType = msg.format.includes('jpeg') ? 'image/jpeg' : msg.format;
        const imageUrl = `data:${mimeType};base64,${msg.data}`;
    
        setImageDataFront(imageUrl);
        setImageFrontMessages(prevMessages => [...prevMessages, labeledMessage]);
      };
    
      realsense_listener_srv.subscribe(handleNewImageMessage);
    
      return () => {
        realsense_listener_srv.unsubscribe(handleNewImageMessage);
      };
    }, []);

    useEffect(() => {
      const realsense_listener_rear_srv = realsense_d455_listener_rear();
    
      const handleNewImageMessage = (msg) => {
        const labeledMessage = `/depth_camera_rear/color/image_raw/compressed received: ${msg.header.stamp}`;
  
        // Adjust MIME type based on msg.format content
        const mimeType = msg.format.includes('jpeg') ? 'image/jpeg' : msg.format;
        const imageUrl = `data:${mimeType};base64,${msg.data}`;
    
        setImageDataRear(imageUrl);
        setImageRearMessages(prevMessages => [...prevMessages, labeledMessage]);
      };
    
      realsense_listener_rear_srv.subscribe(handleNewImageMessage);
    
      return () => {
        realsense_listener_rear_srv.unsubscribe(handleNewImageMessage);
      };
    }, []);

    const showContent = (tabID) => {
      setActiveTab(tabID);
    };
  
    return (
      <div className="diagnostics-page-container">
        <div className="diagnostics-middle-content">
          <div className="diagnostics-middle-content-left">
            <div className="pill-tabs">
              <div className={`pill-tab ${activeTab === 'sidebrush' ? 'active' : ''}`} onClick={() => showContent('sidebrush')}>Side Brush</div>
              <div className={`pill-tab ${activeTab === 'vacuum' ? 'active' : ''}`} onClick={() => showContent('vacuum')}>Vacuum</div>
              <div className={`pill-tab ${activeTab === 'rollerbrush' ? 'active' : ''}`} onClick={() => showContent('rollerbrush')}>Roller Brush</div>
              <div className={`pill-tab ${activeTab === 'camera' ? 'active' : ''}`} onClick={() => showContent('camera')}>Camera</div>
            </div>
          </div>
          <div className="diagnostics-middle-content-right">
            <div id="sidebrush" className="tab-content" style={{ display: activeTab === 'sidebrush' ? 'block' : 'none' }}>
              <div className="sidebrush-top">
                <div className="sidebrush-top-left">
                  {/* Content for top left */}
                </div>
                <div className="sidebrush-top-right">
                  {/* Content for top right */} 
                </div>
              </div>
                <div className="sidebrush-bottom bottom-scroll" id="sidebrush-bottom-scroll">
                  {/* Content for bottom half */}
                    {sidebrushMessages.map((message, index) => (
                    <div key={index}>{message}</div>
                    ))}
                </div>
              </div>
            <div id="vacuum" className="tab-content" style={{ display: activeTab === 'vacuum' ? 'block' : 'none' }}>
              Content for Tab 2
              </div>
            <div id="rollerbrush" className="tab-content" style={{ display: activeTab === 'rollerbrush' ? 'block' : 'none' }}>
              Content for Tab 3
              </div>
            <div
              id="camera"
              className="tab-content"
              style={{
                display: activeTab === 'camera' ? 'flex' : 'none',
                flexDirection: 'column',
                height: '100%',
              }}>
              
              {/* Enlarged view area */}
              <div
                className="camera-enlarged-view"
                style={{
                  flex: 1,
                  display: 'flex',
                  justifyContent: 'center',
                  alignItems: 'center',
                }}>
                {enlargedImage ? (
                  <img
                    src={enlargedImage}
                    alt="Enlarged Camera Output"
                    style={{ maxWidth: '100%', maxHeight: '80%', borderRadius: '10px' }}
                  />
                ) : (
                  <p style={{ color: '#fff' }}>No image data available</p>
                )}
              </div>
              
              {/* Thumbnails area */}
              <div
                className="camera-thumbnails"
                style={{
                  height: '30%',
                  display: 'flex',
                  flexDirection: 'row',
                }}>
                <div
                  style={{
                    flex: 1,
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                    cursor: 'pointer',
                  }}
                  onClick={() => setCurrentCameraMode('front')}>
                  <div
                    className="camera-screen-bottom-left"
                    style={{
                      flex: 1,
                      display: 'flex',
                      justifyContent: 'center',
                      alignItems: 'center',
                      width: '100%',
                    }}>
                    {imageDataFront ? (
                      <img
                        src={imageDataFront}
                        alt="CameraOutputFront"
                        style={{ height: '100%', maxWidth: '35%' , borderRadius: '6px'}}
                      />
                    ) : (
                      <p>No image data available</p>
                    )}
                  </div>
                  <p style={{ margin: '0.5rem 0', textAlign: 'center' }}>Front</p>
                </div>
                <div
                  style={{
                    flex: 1,
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                    cursor: 'pointer',
                  }}
                  onClick={() => setCurrentCameraMode('rear')}>
                  <div
                    className="camera-screen-bottom-right"
                    style={{
                      flex: 1,
                      display: 'flex',
                      justifyContent: 'center',
                      alignItems: 'center',
                      width: '100%',
                    }}>
                    {imageDataRear ? (
                      <img
                        src={imageDataRear}
                        alt="CameraOutputRear"
                        style={{ height: '100%', maxWidth: '35%', borderRadius: '6px' }}
                      />
                    ) : (
                      <p>No image data available</p>
                    )}
                  </div>
                  <p style={{ margin: '0.5rem 0', textAlign: 'center' }}>Rear</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  };

export default Diagnostics;
