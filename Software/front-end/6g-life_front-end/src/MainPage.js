import React, {useState} from "react";
import ConnectRobotPanel from './ConnectRobotPanel';

class MainPage extends React.Component{
    constructor(props){
        super(props);
        this.state = {
            connected: "",
            J1: "",
            J2: "",
            J3: "",
            J4: ""
        };
    }
    operationHandler(operation){

    }
    clearHandler(){

    }
    connectHandler(){
        this.setState({
            connected: "Connected"
        });
    }
    render(){
        return <div className="App-ConnectRobotPanel">
            <ConnectRobotPanel
                connected={this.state.connected}
                operationClicked = {this.operationHandler.bind(this)}
                clearClicked = {this.clearHandler.bind(this)}
                connectClicked = {this.connectHandler.bind(this)}
            />
        </div>

    }
}

export default MainPage;