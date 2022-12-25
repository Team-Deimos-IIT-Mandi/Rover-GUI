import React, { Component } from "react";
import { Navbar, Nav, Container } from "react-bootstrap";

class Header extends Component {
  state = {}
  render() {
    return (
      <Container>
        <Navbar bg="dark" variant="dark" expand="lg" collapseOnSelect>
          <Container>
            <Navbar.Brand href="#home">Navbar</Navbar.Brand>
            <Nav className="me-auto">
              <Nav.Link href="/">Robot Control</Nav.Link>
              <Nav.Link href="https://rover.iitmandi.co.in/">About</Nav.Link>
              <Nav.Link href="/lifescience">Life Science</Nav.Link>
              <Nav.Link href="/roboarm">Robotic Arm</Nav.Link>
            </Nav>
          </Container>
        </Navbar>
      </Container>
    );
  }
}

export default Header