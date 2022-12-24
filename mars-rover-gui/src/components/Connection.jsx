import React, { Component } from "react";
import { Container } from "react-bootstrap"
import Alert from "react-bootstrap/Alert"
import Config from "../scripts/Config"

class Connection extends Component {
    state = {
        connected: false,
        ros: null,
    }

    constructor() {
        super();
        this.init_connection();
    }

    // function to initialize connection with ros:
    init_connection()
    {
        this.state.ros = new window.ROSLIB.Ros();
        console.log(this.state.ros)

        this.state.ros.on("connection", () => {
            console.log("Connection established");
            this.setState({connected: true});
        });

        this.state.ros.on("closed", ()=> {
            console.log("Connection is closed")
            this.setState({connected: false});

            setTimeout(() => {
                // it will try to reconnect every 3 econds if connection is not found
                try {
                    this.state.ros.connect("ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT);     // ip address(get using ifconfig):port number (usually 9090 for websocket conenctions)
                } catch(error) {
                    console.log("connection problem");
                }

            }, 3000);   // executed every 3000 miliseconds

        });

        try {
            this.state.ros.connect("ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT);     // ip address(get using ifconfig):port number (usually 9090 for websocket conenctions)
        } catch(error) {
            console.log("connection problem");
        }
        
    }

    render() {
        return (
            <div>
                <Alert className="text-center m-3"
                variant={this.state.connected? "success": "danger"}>{this.state.connected? "Robot Connected" : "Robot Disconnected"}</Alert>
            </div>
            
        );
    }
}

export default Connection