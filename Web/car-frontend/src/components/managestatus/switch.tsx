import * as React from 'react'
import * as Redux from 'redux'
import { style, media } from 'typestyle'
import { Switch } from 'antd'
import { requestRelayChangeAction, RelayState } from '../../models/managestatus/managestatus'

type RelayProps = RelayState
interface SwitchProps {
  dispatch: Redux.Dispatch<any>
}

export default class RelaySwitch extends React.Component<SwitchProps & RelayProps> {
  //改变继电器开关状态
  handleChange() {
    this.props.dispatch(requestRelayChangeAction(this.props.num, this.props.relayState))
  }

  render() {
    return (
      <div>
        <Switch defaultChecked={false} checked={this.props.relayState[this.props.num - 1]} checkedChildren={`${this.props.num}号开`} unCheckedChildren={`${this.props.num}号关`} onChange={() => { this.handleChange() }} />
      </div>
    )
  }
}