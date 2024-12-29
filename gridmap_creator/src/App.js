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
  name: '/slam_gridmap',
  messageType: 'nav_msgs/OccupancyGrid'
});

const selectedGridmapPublisher = new ROSLIB.Topic({
  ros: ros,
  name: '/selected_gridmap',
  messageType: 'nav_msgs/OccupancyGrid'
});

const loadMapService = new ROSLIB.Service({
  ros: ros,
  name: '/map_server/load_map',
  serviceType: 'nav2_msgs/LoadMap'
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

    // Fetch the selected gridmap on component mount
    fetchSelectedGridmap();

    return () => {
      gridmap_topic.unsubscribe();
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
    const homeDir = process.env.REACT_APP_HOME_DIR;
    const mapUrl = `${homeDir}/.ros/save/saved_selected_gridmap.yaml`;
    const request = new ROSLIB.ServiceRequest({
      map_url: mapUrl
    });
    loadMapService.callService(request, function(result) {
      console.log('Service result:', result);

      if (result.result !== 0) {
        console.error('Failed to load map:', result.result);
        return;
      }

      const width = result.map.info.width;
      const height = result.map.info.height;
      const data = result.map.data;
      const gridArray = [];
      const newSelectedCells = new Set();

      for (let y = 0; y < height; y++) {
        const row = [];
        for (let x = 0; x < width; x++) {
          const index = x + y * width;
          row.push({ value: data[index], x: x, y: y });
          if (data[index] === 100) {
            newSelectedCells.add(`${x}-${y}`);
          }
        }
        gridArray.push(row);
      }

      setGridData(gridArray);
      setGridInfo(result.map.info);
      setSelectedCells(newSelectedCells);
    });
  }

  return (
    <div className="grid-map">
      <h1 style={{ textAlign: 'center', marginBottom: '20px' }}>Gridmap Creator</h1>
      <div style={{ display: 'flex', flexDirection: 'column' }}>
        {gridData.map((row, rowIndex) => (
          <div key={rowIndex} className="grid-row" style={{ display: 'flex', flexDirection: 'row-reverse' }}>
            {row.map((cell, colIndex) => {
              const key = `${colIndex}-${rowIndex}`;
              const isSelected = selectedCells.has(key);
              const cellClass = isSelected && cell.value === 100 ? 'selected-occupied' : isSelected ? 'selected' : cell.value === 100 ? 'occupied' : 'free';
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
