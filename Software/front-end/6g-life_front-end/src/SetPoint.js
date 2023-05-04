import React, {useState} from "react";

function SetPoint(props) {
    const [valueDesired, setValueDesired] = useState("");
    const handleValueDesiredChange = (event)=> { setValueDesired(event.target.value)};
    
    return (
        <label style={{fontSize: 20}}> {props.caption} {": "} 
              <input 
                type = "text"
                value = {valueDesired}
                onChange = {handleValueDesiredChange}
              />
        </label>      
    )
}

export default SetPoint