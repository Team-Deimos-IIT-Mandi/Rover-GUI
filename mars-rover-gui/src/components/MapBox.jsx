import React, { Component } from 'react';
import ReactMapGl, {Marker, Popup} from 'react-map-gl'
import Geocoder from 'react-map-gl-geocoder'
import 'mapbox-gl/dist/mapbox-gl.css';
import 'react-map-gl-geocoder/dist/mapbox-gl-geocoder.css'

const mapboxToken = "pk.eyJ1IjoiYXZuaW1pdHRhbDEzIiwiYSI6ImNsYzJyODUxczF3djUzcW81d2VnMzA4c20ifQ.JP7LyAnaFa9GZID9b1R00A"

class Map extends Component {
    constructor() {
      super()
      this.state = {
        viewport: {
          width: '40vw',
          height: '40vh',
          latitude: 40.78343,
          longitude: -73.96625,
          zoom: 11
        },
        currMarker: null,
        markers: []
      }
      this.handleViewportChange = this.handleViewportChange.bind(this)
      this.handleGeocoderViewportChange = this.handleGeocoderViewportChange.bind(this)
      this.handleResult = this.handleResult.bind(this)
      this.addMarker = this.addMarker.bind(this)
    }
    mapRef = React.createRef()
  
    handleViewportChange(viewport) {
      this.setState(prevState => ({
        viewport: {...prevState.viewport, ...viewport}
      }))
    }
    handleGeocoderViewportChange(viewport) {
      const geocoderDefaultOverrides = {transitionDuration: 1000}
      return this.handleViewportChange({
        ...viewport,
        ...geocoderDefaultOverrides
      })
    }
    handleResult (result) {
      this.setState({
        currMarker: {
          name: result.result.place_name,
          latitude: result.result.center[1],
          longitude: result.result.center[0]
        }
      })
    }
    addMarker() {
      const {currMarker} = this.state
      this.setState(prevState => ({
        markers: [...prevState.markers, currMarker],
        currMarker: null
      }))
    }
    render() {
      const {viewport, markers} = this.state
      return (
        <ReactMapGl
          ref={this.mapRef}
          {...viewport}
          onViewportChange={viewport => this.setState({viewport})}
          mapboxApiAccessToken={mapboxToken}
          mapStyle="mapbox://styles/mapbox/streets-v10"
        >
          <button className="add-btn" onClick={this.addMarker}>Add</button>
          {markers.map((marker, idx) => {
            return (
              <Marker
                key={idx}
                latitude={marker.latitude}
                longitude={marker.longitude}
              >
                <img src="pin.png" alt="marker"/>
              </Marker>
            )
          })
          }
          <Geocoder
            onSelected={this.handleResult}
            mapRef={this.mapRef}
            placeholder="Search here!"
            onViewportChange={this.handleGeocoderViewportChange}
            onResult={this.handleResult}
            mapboxApiAccessToken={mapboxToken}
            countries="us"
            position="top-right"
          />
        </ReactMapGl>
      )
    }
  }
  
  export default Map;