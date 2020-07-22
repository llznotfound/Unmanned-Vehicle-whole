import * as React from 'react'
import * as Redux from 'redux'
import { style, media } from 'typestyle'
import { Select } from 'antd'
import { changeMotorAction, MotorState } from '../../models/managestatus/managestatus'

type MotorProps = MotorState
interface SelectProps {
  dispatch: Redux.Dispatch<any>
}

export default class MotorSelect extends React.Component<SelectProps & MotorProps> {

  handleChange(value) {
    this.props.dispatch(changeMotorAction(value))
  }

  render() {
    const Option = Select.Option
    return (
      <div>
        <Select size='large' value={this.props.motor} style={{ position: 'absolute', top: window.innerWidth>3800?'350px':'', left: window.innerWidth>3800?'100px':'',width: window.innerWidth>3800?'300px':'100px',fontSize:window.innerWidth>3800?'30px':'', }} defaultValue='2' onChange={(value) => { this.handleChange(value) }} >
          <Option value='0'><div style={{fontSize:window.innerWidth>3800?'30px':''}}>收线</div></Option>
          <Option value='1'><div style={{fontSize:window.innerWidth>3800?'30px':''}}>放线</div></Option>
          <Option value='2'><div style={{fontSize:window.innerWidth>3800?'30px':''}}>停止</div></Option>
        </Select>
      </div>
    )
  }
}