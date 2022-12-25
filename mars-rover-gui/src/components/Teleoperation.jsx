import React, { Component } from "react";
import { Joystick } from "react-joystick-component";
import Config from "../scripts/Config"

class Teleoperation extends Component {
    state = { ros: null }

    constructor() {
        super();
        this.init_connection();

        // need to bind the method if using state variables
        this.handleMove = this.handleMove.bind(this);  
        this.handleStop = this.handleStop.bind(this);     
    }

    // REPEAT FUNCTION FROM CONNECTION MODULE
    // function to initialize connection with ros:
    init_connection()
    {
        // this.setState({ros: new window.ROSLIB.Ros()})
        this.state.ros = new window.ROSLIB.Ros();
        console.log(this.state.ros)

        this.state.ros.on("connection", () => {
            console.log("Connection established in Teleoperation");
            this.setState({connected: true});
        });

        this.state.ros.on("closed", ()=> {
            console.log("Connection is closed in Teleoperation")
            this.setState({connected: false});

            setTimeout(() => {
                // it will try to reconnect every 3 econds if connection is not found
                try {
                    this.state.ros.connect("ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT);     // ip address(get using ifconfig):port number (usually 9090 for websocket conenctions)
                } catch(error) {
                    console.log("connection problem in Teleoperation");
                }

            }, Config.RECONNECTION_TIMER);   // executed every 3000 miliseconds

        });

        try {
            this.state.ros.connect("ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT);     // ip address(get using ifconfig):port number (usually 9090 for websocket conenctions)
        } catch(error) {
            console.log("connection problem in Teleoperation");
        }
        
    }
    // END REPEAT

    handleMove(event) {
        // create ros publisher on cmd_vel
        var cmd_vel = new window.ROSLIB.Topic({
            ros: this.state.ros, // roshandler
            name: Config.CMD_VEL_TOPIC,
            messageType: "geometry_msgs/Twist"

        })
        // create twist message and send to rosbridge
        var twist = new window.ROSLIB.Message({
            linear: {
                x: event.y,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: -event.x,
            }
        });

        // publish to message on cmd_vel topic
        cmd_vel.publish(twist);
    }

    handleStop(event) {
        // create ros publisher on cmd_vel
        var cmd_vel = new window.ROSLIB.Topic({
            ros: this.state.ros, // roshandler
            name: Config.CMD_VEL_TOPIC,
            messageType: "geomtery_msgs/Twist"

        })
        // create twist message and send to rosbridge
        var twist = new window.ROSLIB.Message({
            linear: {
                x: 0,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: 0,
            }
        });

        // publish to message on cmd_vel topic
        cmd_vel.publish(twist);

    }

    render() {
        return (
            <div>
                <Joystick
                    size={250}
                    baseColor="#EEEEEE"
                    stickColor="#BBBBBB"
                    move={this.handleMove}
                    stop={this.handleStop}
                ></Joystick>
            </div>
        );
    }
}

export default Teleoperation