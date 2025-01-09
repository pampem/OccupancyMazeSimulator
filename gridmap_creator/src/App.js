import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9098'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

const gridmap_topic = new ROSLIB.Topic({
  ros: ros,
  name: 'drone1/slam_gridmap',
  messageType: 'nav_msgs/OccupancyGrid'
});

const selectedGridmapPublisher = new ROSLIB.Topic({
  ros: ros,
  name: 'drone1/selected_gridmap',
  messageType: 'nav_msgs/OccupancyGrid'
});

const drone1GridmapTopic = new ROSLIB.Topic({
  ros: ros,
  name: 'drone1/gridmap',
  messageType: 'nav_msgs/OccupancyGrid'
});

export default function GridMap() {
  const [gridData, setGridData] = useState([]);
  const [selectedCells, setSelectedCells] = useState(new Set());
  const [gridInfo, setGridInfo] = useState(null);

  useEffect(() => {
    gridmap_topic.subscribe(message => {
      const width = message.info.width;
      const height = message.info.height;
      const data = message.data;
      const gridArray = [];

      for (let y = 0; y < height; y++) {
        const row = [];
        for (let x = 0; x < width; x++) {
          const index = x + y * width;
          row.push({ value: data[index], x: x, y: y });
        }
        gridArray.push(row);
      }

      setGridData(gridArray);
      setGridInfo(message.info);
    });

    // Subscribe to the drone1/gridmap topic
    fetchSelectedGridmap();

    return () => {
      gridmap_topic.unsubscribe();
      drone1GridmapTopic.unsubscribe();
    };
  }, []);

  function handleCellClick(x, y) {
    const key = `${x}-${y}`;
    const newSet = new Set(selectedCells);
    if (newSet.has(key)) {
      newSet.delete(key);
    } else {
      newSet.add(key);
    }
    setSelectedCells(newSet);
  }

  function confirmSelection() {
    publishSelectedCellsGridmap();
  }

  function publishSelectedCellsGridmap() {
    if (gridInfo) {
      const newGridData = new Array(gridInfo.width * gridInfo.height).fill(-1);
      selectedCells.forEach(key => {
        const [x, y] = key.split('-').map(Number);
        const index = x + y * gridInfo.width;
        newGridData[index] = 100;
      });
      const gridMessage = new ROSLIB.Message({
        header: {
          frame_id: 'odom',
          stamp: {
            secs: Math.floor(Date.now() / 1000),
            nsecs: (Date.now() % 1000) * 1e6
          }
        },
        info: gridInfo,
        data: newGridData
      });
      console.log("Publishing selected cells gridmap:", gridMessage);
      selectedGridmapPublisher.publish(gridMessage);
    }
  }

  function fetchSelectedGridmap() {
    // Unsubscribe previous subscription to avoid duplication
    drone1GridmapTopic.unsubscribe();
  
    // Subscribe to the topic
    drone1GridmapTopic.subscribe(message => {
      console.log('Received selected gridmap from drone1/gridmap:', message);
      const width = message.info.width;
      const height = message.info.height;
      const data = message.data;
      const gridArray = [];
      const newSelectedCells = new Set();
  
      for (let y = 0; y < height; y++) {
        const row = [];
        for (let x = 0; x < width; x++) {
          const index = x + y * width;
          row.push({ value: data[index], x, y });
          if (data[index] === 100) {
            newSelectedCells.add(`${x}-${y}`);
          }
        }
        gridArray.push(row);
      }
  
      // Update gridData and replace selectedCells with ROS-provided state
      setGridData(gridArray);
      setGridInfo(message.info);
      setSelectedCells(newSelectedCells); // Directly set the new state, overriding previous selections
    });
  }  

  return (
    <div className="grid-map">
      <h1 style={{ textAlign: 'center', marginBottom: '20px' }}>Gridmap Creator</h1>
      <div style={{ display: 'flex', flexDirection: 'column-reverse' }}>
        {gridData.map((row, rowIndex) => (
          <div key={rowIndex} className="grid-row" style={{ display: 'flex', flexDirection: 'row' }}>
            {row.map((cell, colIndex) => {
              const key = `${colIndex}-${rowIndex}`;
              const isSelected = selectedCells.has(key);
              const cellClass =
                isSelected && cell.value === 100
                  ? 'selected-occupied'
                  : isSelected
                  ? 'selected'
                  : cell.value === 100
                  ? 'occupied'
                  : 'free';
              return (
                <div
                  key={key}
                  onClick={() => handleCellClick(colIndex, rowIndex)}
                  className={`cell ${cellClass}`}
                />
              );
            })}
          </div>
        ))}
      </div>
      <button onClick={confirmSelection} className="button">
        Apply
      </button>
      <button onClick={fetchSelectedGridmap} className="button2">
        Fetch
      </button>
    </div>
  );
}
