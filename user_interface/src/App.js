import { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

function App() {
  const [connected, setConnected] = useState(false);
  const [processingItems, setProcessingItems] = useState([]);
  const [completedItems, setCompletedItems] = useState([]);
  const rosRef = useRef(null);
  const topicSubscriberRef = useRef(null);
  const completedIdsRef = useRef(new Set()); // Track completed IDs to avoid duplicates

  // Initialize connection to ROSbridge
  useEffect(() => {
    // Create ros connection
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
      console.log('Connected to ROSbridge server');
      setConnected(true);
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROSbridge server:', error);
      setConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to ROSbridge server closed');
      setConnected(false);
    });

    rosRef.current = ros;

    // Cleanup function
    return () => {
      if (ros) {
        ros.close();
      }
    };
  }, []);

  // Subscribe to processing time events topic
  useEffect(() => {
    if (!rosRef.current || !connected) {
      return;
    }

    console.log('Setting up topic subscriber');
    
    // Create a subscriber for the processing time events topic
    const subscriber = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/processing_time_events',
      messageType: 'std_msgs/String'
    });

    // Subscribe to the topic
    subscriber.subscribe((message) => {
      try {
        const eventData = JSON.parse(message.data);
        console.log('Received event:', eventData);
        
        if (eventData.event_type === 'service_call') {
          // Create a stable ID that doesn't depend on timestamp
          const stableId = `${eventData.carrier_id}-${eventData.station_id}-${eventData.timestamp}`;
          
          // Add to processing items with a stable unique ID
          const newItem = {
            id: stableId,
            carrierId: eventData.carrier_id,
            stationId: eventData.station_id,
            processingTime: eventData.processing_time,
            startTime: Date.now(),
            timeRemaining: eventData.processing_time
          };
          
          setProcessingItems(prev => [...prev, newItem]);
        }
      } catch (error) {
        console.error('Error parsing event message:', error);
      }
    });

    topicSubscriberRef.current = subscriber;
    
    // Cleanup function
    return () => {
      if (subscriber) {
        subscriber.unsubscribe();
      }
    };
  }, [connected]);

  // Update countdown for processing items
  useEffect(() => {
    if (processingItems.length === 0) return;

    const interval = setInterval(() => {
      const now = Date.now();
      
      // Update remaining time for each item
      setProcessingItems(prevItems => {
        const updatedItems = prevItems.map(item => {
          const elapsedTime = now - item.startTime;
          const timeRemaining = Math.max(0, item.processingTime - elapsedTime);
          return {...item, timeRemaining};
        });
        
        // Move completed items
        const stillProcessing = [];
        const newlyCompleted = [];
        
        updatedItems.forEach(item => {
          if (item.timeRemaining > 0) {
            stillProcessing.push(item);
          } else if (!completedIdsRef.current.has(item.id)) {
            // Only add to completed if not already tracked
            completedIdsRef.current.add(item.id);
            newlyCompleted.push(item);
          }
        });
        
        if (newlyCompleted.length > 0) {
          setCompletedItems(prev => [...prev, ...newlyCompleted]);
        }
        
        return stillProcessing;
      });
    }, 100); // Update every 100ms for smoother countdown
    
    return () => clearInterval(interval);
  }, [processingItems]);

  return (
    <div className="App">
      <header className="App-header">
        <h1>ROS2 Processing Time Monitor</h1>
        <div className="connection-status">
          Status: {connected ? 
            <span className="connected">Connected to ROSbridge</span> : 
            <span className="disconnected">Disconnected</span>}
        </div>
      </header>
      
      <div className="visualization">
        <div className="processing-list">
          <h2>Currently Processing</h2>
          {processingItems.length === 0 ? (
            <p>No items currently processing</p>
          ) : (
            <ul>
              {processingItems.map(item => (
                <li key={item.id} className="processing-item">
                  <div>Carrier: {item.carrierId}, Station: {item.stationId}</div>
                  <div>Processing Time: {item.processingTime}ms</div>
                  <div className="progress-container">
                    <div 
                      className="progress-bar" 
                      style={{width: `${100 * (1 - item.timeRemaining / item.processingTime)}%`}}
                    />
                  </div>
                  <div>Time Remaining: {Math.ceil(item.timeRemaining)}ms</div>
                </li>
              ))}
            </ul>
          )}
        </div>
        
        <div className="completed-list">
          <h2>Completed Processes</h2>
          {completedItems.length === 0 ? (
            <p>No completed items yet</p>
          ) : (
            <ul>
              {completedItems.map(item => (
                <li key={item.id} className="completed-item">
                  <div>Carrier: {item.carrierId}, Station: {item.stationId}</div>
                  <div>Processing Time: {item.processingTime}ms</div>
                  <div>✓ Completed</div>
                </li>
              ))}
            </ul>
          )}
        </div>
      </div>
    </div>
  );
}

export default App;
