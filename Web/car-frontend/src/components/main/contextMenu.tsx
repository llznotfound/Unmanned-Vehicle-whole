import * as React from 'react'
import * as Redux from 'redux'
import {updateWorkingPointAction} from '../../models/main/workingstatus'

declare let BMap

interface ContextMenuProps {
  map?: any
  dispatch: Redux.Dispatch<any>
}

//右键菜单添加关键路径点
export default class ContextMenu extends React.Component<ContextMenuProps, any> {
  menu: any

  render() {
    return null
  }

  componentDidMount() {
    console.log()
    this.menu = new BMap.ContextMenu();
    this.menu.addItem(new BMap.MenuItem('添加', (e) => {this.props.dispatch(updateWorkingPointAction(e))}));
    this.props.map.addContextMenu(this.menu);
  }

  componentWillUnmount() {
    this.props.map.removeContextMenu(this.menu)
  }
}
