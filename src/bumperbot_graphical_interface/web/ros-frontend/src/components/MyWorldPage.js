import React, { useEffect, useRef, useState } from 'react';
import './MyWorldPage.css'; // Import the CSS file

const MyWorldPage = () => {
  const canvasRef = useRef(null);
  const imageRef = useRef(null);
  const [data, setData] = useState([]);

  useEffect(() => {
    fetch('http://localhost:5000/api/data')
      .then((response) => response.json())
      .then((result) => setData(result.data))
      .catch((error) => console.error('Error fetching data:', error));

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const image = imageRef.current;

    image.onload = () => {
      // Set canvas size to match the image size
      canvas.width = image.width;
      canvas.height = image.height;

      // Draw the image on the canvas
      ctx.drawImage(image, 0, 0);

      // Define the contours
      const contours = [
        [
          { x: 78, y: 105 }, { x: 78, y: 114 }, { x: 104, y: 114 }, { x: 105, y: 107 },
          { x: 115, y: 108 }, { x: 115, y: 118 }, { x: 109, y: 123 }, { x: 78, y: 123 },
          { x: 78, y: 194 }, { x: 106, y: 193 }, { x: 106, y: 176 }, { x: 81, y: 162 },
          { x: 83, y: 142 }, { x: 92, y: 144 }, { x: 92, y: 156 }, { x: 116, y: 170 },
          { x: 116, y: 201 }, { x: 111, y: 205 }, { x: 78, y: 204 }, { x: 78, y: 209 },
          { x: 168, y: 209 }, { x: 196, y: 179 }, { x: 225, y: 179 }, { x: 227, y: 185 },
          { x: 222, y: 189 }, { x: 201, y: 189 }, { x: 181, y: 209 }, { x: 240, y: 209 },
          { x: 240, y: 105 }, { x: 236, y: 105 }, { x: 236, y: 145 }, { x: 231, y: 149 },
          { x: 225, y: 146 }, { x: 225, y: 112 }, { x: 202, y: 112 }, { x: 199, y: 105 }
        ],
        [
          { x: 167, y: 165 }, { x: 172, y: 182 }, { x: 168, y: 189 }, { x: 132, y: 189 },
          { x: 133, y: 163 }
        ],
        [
          { x: 177, y: 127 }, { x: 202, y: 126 }, { x: 204, y: 150 }, { x: 199, y: 154 },
          { x: 178, y: 153 }
        ]
      ];

      // Draw the contours
      ctx.strokeStyle = 'magenta';
      ctx.lineWidth = 2;
      contours.forEach(contour => {
        ctx.beginPath();
        ctx.moveTo(contour[0].x, contour[0].y);
        contour.forEach(point => {
          ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.stroke();
      });
    };
  }, []);

  return (
    <div className="my-world-page">
      <h2>My World</h2>
      <img
        ref={imageRef}
        src={`${process.env.PUBLIC_URL}/my_world_map2.png`}
        alt="My World Map"
        className="my-world-map"
        style={{ display: 'none' }}
      />
      <canvas ref={canvasRef} className="my-world-canvas"></canvas>
    </div>
  );
};

export default MyWorldPage;