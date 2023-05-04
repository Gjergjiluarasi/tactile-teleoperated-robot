import React, {useState} from "react";
import Button from "./Button";

function ConnectRobotPanel(props) {
    const [robotIP, setRobotIP] = useState("");
    const handleRobotIPChange = (event)=> { setRobotIP(event.target.value)};
    return (
        <div className="App-ConnectRobotPanel">
            <label> {"Robot IP:  "}{props.connected}
            <input 
                type = "text"
                value = {robotIP}
                onChange = {handleRobotIPChange}
            />
            </label>
            <Button caption='Connect'
            onClick = {props.connectClicked}/>
            {/* <Button color="cyan" appearance="primary">
            Cyan</Button> */}
        </div>
    )
}

export default ConnectRobotPanel;