import React, { Component } from "react";
import {Row, Col, Container, Button} from "react-bootstrap"
import Config from "../scripts/Config"
import * as Three from "three"

class RobotState extends Component {
    state = {
        x:0,
        y:0,
        orientation:0,
        linear_velocity:0,
        angular_velocity:0,
    };

    // REPEATING CONNECTION PART

    constructor() {
        super();
        this.init_connection();
    }

    // function to initialize connection with ros:
    init_connection()
    {
        // this.setState({ros: new window.ROSLIB.Ros()})
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

            }, Config.RECONNECTION_TIMER);   // executed every 3000 miliseconds

        });

        try {
            this.state.ros.connect("ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT);     // ip address(get using ifconfig):port number (usually 9090 for websocket conenctions)
        } catch(error) {
            console.log("connection problem");
        }
        
    }

    // END REPEAT CONNECTION

    componentDidMount(){
        this.getRobotState();
    }


    getRobotState() {

        // created pose subscriber
        var pose_subscriber = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: Config.ROBOT_COORDINATE_TOPIC,
            messageType: "geometry_msgs/PoseWithCovarianceStamped"
        });

        // pose callback
        pose_subscriber.subscribe((message) => {
            this.setState({x: message.pose.pose.position.x.toFixed(2)});
            this.setState({y: message.pose.pose.position.y.toFixed(2)});
            this.setState({orientation: this.getOrientationFromQuaternion(message.pose.pose.orientation).toFixed(2)});
        })


        // creating subscriber for velocities
        var velocity_subscriber = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: Config.ROBOT_VELOCITY_TOPIC,
            messageType: "nav_msgs/Odometry"
        })

        // callback function for velocity/odometry
        velocity_subscriber.subscribe((message) => {
            this.setState({linear_velocity : message.twist.twist.linear.x})
            this.setState({angular_velocity : message.twist.twist.angular.z})
        })

    }

    getOrientationFromQuaternion(ros_orientation_quaternion) {
        var q = new Three.Quaternion(
            ros_orientation_quaternion.x,
            ros_orientation_quaternion.y,
            ros_orientation_quaternion.z,
            ros_orientation_quaternion.w
        );

        // convert to itch and yaw
        var RPY = new Three.Euler().selfFromQuaternion(q);

        return RPY("_z") * (180 / Math.PI);
    }


    render() {
        return ( 
            <div>
                <Row>
                    <Col>
                        <h4 className="mt-4">Position</h4>
                        <p className="mt-4">x: {this.state.x}</p>
                        <p className="mt-4">y: {this.state.y}</p>

                        <h4 className="mt-4">Orientation: {this.state.orientation}</h4>
                    </Col>
                </Row>
                <Row>
                    <Col>
                        <h4 className="mt-4">Velocities:</h4>
                        <p className="mt-4">Linear Velocities: {this.state.linear_velocity}</p>
                        <p className="mt-4">Angular Velocities: {this.state.angular_velocity}</p>

                    </Col>
                </Row>
            </div>
         );
    }
}

export default RobotState