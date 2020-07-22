import * as React from 'react'
import * as Redux from 'redux'
import $ from 'jquery'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'


interface PlaneGaugeProps {
  angle:number
}

export default class PlaneGauge extends React.Component<PlaneGaugeProps, any> {

  div: HTMLDivElement

 

  render() {
    const wholeStyle = style(
      {
        height: '100%',
        width: '100%',
        margin: 'auto'
      }
    )
    //var attitude = $.flightIndicator('#attitude', 'attitude', {roll:50, pitch:-20, size:200, showBox : true});
    //attitude.setRoll(30)
    return (
      <div  className={wholeStyle}>
        <span id="attitude"></span> 
      </div>
    )
  }
}
