
import * as React from 'react'
import {style,media} from 'typestyle'


//import {Group, Shape, Surface, Transform} from 'react-art'
/*var React = require('react')
var ReactART = React.ART*/
//import Circle from 'react-art/shapes/circle'
//var ARTSVGMode = require('art/modes/svg');
//var ARTCurrentMode = require('art/modes/current');
//import d3 from 'd3-scale'
//ARTCurrentMode.setCurrent(ARTSVGMode);



interface PointerProps {
  angle:number//方向角
}


export default class Pointer extends React.Component<PointerProps, any> {

  render() {
    const PointerBlockStyle = style({
        width:'160px',
        height:'140px',
        margin:'0px 0px -90px 0px',
        zoom:window.innerWidth>3800?3:1
      }
      )
      return (
        <div> </div>
          /*<svg>
    <use xlinkHref="/home/huhanjiang/实习/ocean-demo-frontend/src/assets/#heading_yaw" />
      </svg>*/
         
        
        /*<svg className={PointerBlockStyle} >
          <line x1="0" y1="50" x2="160" y2="50" stroke="white" strokeWidth="1"/>
          <line x1="80" y1="50" x2="80" y2="5" stroke="white" strokeWidth="1"/>
          <circle cx="80" cy="50" r="5"  fill="white"/>
          <line x1="80" y1="50" x2={`${80+45*Math.sin(this.props.angle*Math.PI/180) }`} y2={`${50-45*Math.cos(this.props.angle*Math.PI/180) }`} stroke="white" strokeWidth="3" />
      </svg>*/
      )
  }
}
