/* 
This component fetches the latest actuator value from the backend every second and adds it to the chart data.
The labels array contains the timestamps for each data point, and the datasets array contains the actual data points.
The chart is rendered using the Line component from the react-chartjs-2 library.
*/
import React, { useState, useEffect } from 'react';
import { Line } from 'react-chartjs-2';

function ChartComponent() {
    const [data, setData] = useState({
        labels: [],
        datasets: [
            {
                label: 'Actuator Value',
                data: [],
                fill: false,
                borderColor: 'rgb(75, 192, 192)',
                tension: 0.1
            }
        ]
    });

    useEffect(() => {
        const intervalId = setInterval(() => {
            // Fetch the latest actuator value from the backend
            fetch('http://localhost:8080/actuator_value')
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // Add the new actuator value to the chart data
                    const newData = {
                        labels: [...data.labels, new Date().toLocaleTimeString()],
                        datasets: [
                            {
                                ...data.datasets[0],
                                data: [...data.datasets[0].data, data.actuator_value]
                            }
                        ]
                    };
                    setData(newData);
                } else {
                    console.error('Failed to fetch actuator value');
                }
            })
            .catch(error => {
                console.error('Failed to fetch actuator value from backend:', error);
            });
        }, 1000);

        return () => {
            clearInterval(intervalId);
        };
    }, []);

    return (
        <div>
            <Line data={data} />
        </div>
    );
}

export default ChartComponent;