import * as React from 'react'
import * as Redux from 'redux'
import { style, media } from 'typestyle'
import { Slider } from 'antd'
import { changeMeterAction, MotorState } from '../../models/managestatus/managestatus'

type MotorProps = MotorState
interface SliderProps {
  dispatch: Redux.Dispatch<any>
}
export default class MotorSlider extends React.Component<SliderProps & MotorProps> {
  handleChange(value) {
    this.props.dispatch(changeMeterAction(value))
  }
  render() {
    const marks = {
      0.25: <div style={{fontSize:window.innerWidth>3800?'30px':''}}>0.25m</div>,
      0.5: <div  style={{fontSize:window.innerWidth>3800?'30px':''}}>0.5m</div>,
      1: <div  style={{fontSize:window.innerWidth>3800?'30px':''}}>1m</div>,
      1.5: <div style={{fontSize:window.innerWidth>3800?'30px':''}}>1.5m</div>,
      2: <div style={{fontSize:window.innerWidth>3800?'30px':''}}>2m</div>,
      2.5: <div style={{fontSize:window.innerWidth>3800?'30px':''}}>2.5m</div>,
      3: <div style={{fontSize:window.innerWidth>3800?'30px':''}}>3m</div>
    }
    const sliderStyle = style({
     
    },
    media({minWidth:3800}, 
      {
        margin:'50px 0'
      }
    ))
    function formatter(value) {
      return <div style={{fontSize:window.innerWidth>3800?'30px':''}}>{value}</div>;
    }
    return (
      <div className={sliderStyle}>
        <Slider min={0} max={3} marks={marks} step={null} tipFormatter={formatter} value={this.props.meter} defaultValue={0.25} onChange={(value) => { this.handleChange(value) }} />
      </div>
    )
  }
}