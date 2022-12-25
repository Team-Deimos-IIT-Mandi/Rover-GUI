import React, { Component } from "react";
import {Map,  GoogleApiWrapper} from "google-map-react"

class Maps extends Component {
    state = { 
        long: "46",
        lat: "55"
     }
    render() {
        return ( 
            <div style="width: 100%"><iframe scrolling="no" marginheight="0" marginwidth="0" src="https://maps.google.com/maps?width=100%25&amp;height=600&amp;hl=en&amp;q=32,22+(My%20Business%20Name)&amp;t=&amp;z=14&amp;ie=UTF8&amp;iwloc=B&amp;output=embed" width="100%" height="600" frameborder="0"><a href="https://www.maps.ie/distance-area-calculator.html">measure acres/hectares on map</a></iframe></div>
         );
    }
}

export default Maps