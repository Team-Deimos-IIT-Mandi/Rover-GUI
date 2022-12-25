import React, { Component } from "react";
import { Container } from "react-bootstrap"
import { Route, BrowserRouter as Router, Routes } from  "react-router-dom"
import Home from "./Home"
import LifeScience from "./LifeScience"
import RoboArm from "./RoboArm"

class Body extends Component {
    state = {  }
    render() {
        return ( 
            <Container>
            <Router>
                <Routes>
                    <Route path = "/"  element = {<Home />}></Route>
                    <Route path='/about' component={() => {
                        window.location.href = 'https://rover.iitmandi.co.in/';
                        return null;
                    }}></Route>

                    <Route path = "/lifescience"  element = {<LifeScience />}></Route>
                    <Route path='/roboarm' element = {<RoboArm />}></Route>                 
                </Routes>
            </Router>
            </Container>

         );
    }
}

export default Body