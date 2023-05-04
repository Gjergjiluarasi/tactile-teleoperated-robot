import logo from './logo.svg';
import './App.css';
import React, {useState} from "react";
import ConnectRobotPanel from './ConnectRobotPanel';
import MainPage from './MainPage';

// import {Button} from 'rsuite';
import SetPointsPanel from './SetPointsPanel';

function App() {
  // const [sensorData, getSensorData] = useState("");
  const [actuatorData, setActuatorData] = useState("");
  // const handleSensorChange = (event) => { get}
  const handleActuatorChange = (event)=> { setActuatorData(event.target.value)};

  const [robotIP, setRobotIP] = useState("");
  const handleRobotIPChange = (event)=> { setRobotIP(event.target.value)};

  // const [J1_des, setJ1_des] = useState("");
  // const [J2_des, setJ2_des] = useState("");
  // const [J3_des, setJ3_des] = useState("");
  // const [J4_des, setJ4_des] = useState("");
  // const handleJ1_desChange = (event)=> { setJ1_des(event.target.value)};
  // const handleJ2_desChange = (event)=> { setJ2_des(event.target.value)};
  // const handleJ3_desChange = (event)=> { setJ3_des(event.target.value)};
  // const handleJ4_desChange = (event)=> { setJ4_des(event.target.value)};
  
  return (
    <div className="App">
      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
        <a
          className="App-link"
          href="https://reactjs.org"
          target="_blank"
          rel="noopener noreferrer"
        >
          Learn React
        </a>
      </header>
      <body className="App-body">
        <h1> 6G-life Surgical Robot</h1>
        <label> {"Actuator input:  "} 
          <input 
            type = "text"
            value = {actuatorData}
            onChange = {handleActuatorChange}
          />
        </label>
        <h2> Actuator output: <br/> {actuatorData}</h2>
        <ConnectRobotPanel/>
        <SetPointsPanel/>
        <MainPage/>
      </body>
    </div>
  );
}

export default App;
