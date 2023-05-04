/*
This React component can send the button state to the C++ backend using the API endpoint.
It uses the fetch API to send a POST request to the C++ backend.
*/
import React, { useState } from 'react';

function ButtonComponent() {
    const [buttonState, setButtonState] = useState(false);

    function handleButtonClick() {
        // Toggle the button state
        const newButtonState = !buttonState;
        setButtonState(newButtonState);

        // Send the button state to the backend
        fetch('http://localhost:8080/button_state', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ button_state: newButtonState })
        })
        .then(response => response.json())
        .then(data => {
            if (!data.success) {
                console.error('Failed to update actuator value');
            }
        })
        .catch(error => {
            console.error('Failed to send button state to backend:', error);
        });
    }

    return (
        <div>
            <button onClick={handleButtonClick}>
                {buttonState ? 'ON' : 'OFF'}
            </button>
        </div>
    );
}

export default ButtonComponent;