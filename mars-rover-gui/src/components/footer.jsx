import React, { Component } from "react";
import { Container } from "react-bootstrap"

class Footer extends Component {
    state = {  }
    render() {
        return ( 
            <Container className = "text-center">
                <p>IIT Mandi Deimos Team &copy; 2022</p>
            </Container>
         );
    }
}

export default Footer