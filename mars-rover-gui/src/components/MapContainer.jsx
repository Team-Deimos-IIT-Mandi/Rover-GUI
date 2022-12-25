import React, { Component } from "react";
import {Row, Col, Container, Button} from "react-bootstrap"
import Api from "../scripts/apis"
// import {Map, } from "google-map-react"
import GoogleMapReact from 'google-map-react';


const AnyReactComponent = ({ text }) => <div>{text}</div>;


class MapContainer extends Component {
    state = {  
        center: {
            lat: 10.99835602,
            lng: 77.01502627
          },
          zoom: 11
      };

    // const {isLoaded} = useLoadScript({
    //     googleMapsApiKey = Api.GOOGLE_MAPS_API_KEY
    // })

    // const defaultProps = {
    //     center: {
    //       lat: 10.99835602,
    //       lng: 77.01502627
    //     },
    //     zoom: 11
    //   };

    render() {
        return ( 
            <div>
                <div style={{ height: '100vh', width: '100%' }}>
                    <GoogleMapReact
                        bootstrapURLKeys={{ key: Api.GOOGLE_MAPS_API_KEY }}
                        defaultCenter={this.state.center}
                        defaultZoom={this.state.zoom}
                    >
                        <AnyReactComponent
                        lat={59.955413}
                        lng={30.337844}
                        text="My Marker"
                        />
                    </GoogleMapReact>
                </div>
            </div>
         );
    }
}

export default MapContainer