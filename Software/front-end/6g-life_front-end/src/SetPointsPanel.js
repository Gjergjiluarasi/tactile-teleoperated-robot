import React from "react";
import SetPoint from "./SetPoint";

function SetPointsPanel(props) {
    const setPointsCaptions = ["J1(shaft)", "J2(wrist)", "J3(gripper left)", "J4(gripper right)"];
    const setPoints = setPointsCaptions.map((value, index) => {
        return <SetPoint caption={value} />
    });

    return (
        <div className="App-SetPointsPanel">
            {setPoints}
        </div>
    )
}

export default SetPointsPanel;