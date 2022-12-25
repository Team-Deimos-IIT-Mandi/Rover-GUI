import React, { Component } from "react";
import Connection from "./Connection";
import Teleoperation from "./Teleoperation"
import RobotState from "./RobotState";

import Maps from "./MapBox"

// import Maps from "./Maps";
import {Row, Col, Container} from "react-bootstrap"

class Home extends Component {
    state = {}

    render() {
        return ( 
            <div>
                <Container>
                    <h1 className="text-center mt-3">Robot Control bridge</h1>
                    
                    <Row>
                        <Col>
                            <Connection />
                        </Col>
                    </Row>
                    
                    <Row  className="mt-5 justify-content-center">
                        <Col xs={6} md={4}>
                            <h2>JoyStick</h2>
                            <Teleoperation className= "mt-3"/>
                        </Col>
                        <Col xs={12} md={8} >
                            <h2>MAP</h2>
                            {/* <MapContainer /> */}
                            <span>
                            <Maps />
                            </span>
                        </Col>
                    </Row>

                    <Row className="mt-5">
                        <Col>
                            <h2>Robot State</h2>
                            <RobotState />
                        </Col>
                    </Row>


                    {/* <Maps /> */}
                    
                    
                </Container>
            </div>
         );
    }
}

export default Home