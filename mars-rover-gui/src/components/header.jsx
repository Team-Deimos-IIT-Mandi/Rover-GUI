import React, { Component } from "react";
import { Navbar, Nav, Container } from "react-bootstrap";

class Header extends Component {
    state = {  }
    render() {
        return ( 
            <Container>
      <Navbar bg="dark" variant="dark" expand = "lg" collapseOnSelect>
        <Container>
          <Navbar.Brand href="#home">Navbar</Navbar.Brand>
          <Nav className="me-auto">
            <Nav.Link href="/">Home</Nav.Link>
            <Nav.Link href="/">Features</Nav.Link>
            <Nav.Link href="/">Pricing</Nav.Link>
          </Nav>
        </Container>
      </Navbar>
    </Container>
         );
    }
}

export default Header