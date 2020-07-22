import { createAction, Action } from 'redux-actions'
import * as Redux from 'redux'
import * as DVA from 'dva'
import { message } from 'antd'
import { PREFIX, TimeState, WorkingState } from './index'
import { IP } from '../main'

import * as Request from '../../utils/request'

const request: any = Request

// 改变倒计时的动作
export function changeTimeAction () {
  return createAction(`${PREFIX}/changeTime`)()
}

export function changeTime (state: TimeState) {
  let newState = { ...state }
  if (newState.times == 0) newState.times = 9
  else newState.times = newState.times - 1
  return newState
}

// 清除计时器
export function clearTimeAction () {
  return createAction(`${PREFIX}/clearTime`)()
}

export function clearTime (state: TimeState) {
  let newState = { ...state }
  newState.times = 10
  return newState
}

// 改变警告倒计时的动作
export function changeAlertTimeAction () {
  return createAction(`${PREFIX}/changeAlertTime`)()
}

export function changeAlertTime (state: WorkingState) {
  let newState = { ...state }
  if (newState.alertTime == 0) {
    newState.alertMode = false
  } else {
    newState.alertTime = newState.alertTime - 1
  }
  return newState
}

// 清除警告计时器
export function clearAlertTimeAction () {
  return createAction(`${PREFIX}/clearAlertTime`)()
}

export function clearAlertTime (state: WorkingState) {
  let newState = { ...state }
  newState.alertTime = 30
  return newState
}


// 向后端发送一键返航指令的动作
export function sendReturnAction () {
  return createAction(`${PREFIX}/sendReturn`)()
}

export function* sendReturn (action: Redux.Action, effects: DVA.EffectsCommandMap) {
  const response: { status: string } = yield(() => {
    return request({
      url: `http://${IP}/ocean/mode_return`,
    })
  })()
  if (response.status == 'ok') {
    message.success('发送一键返航指令成功', 3)
  }
}

// 切换连线模式与平扫模式的动作
export function toggleModeAction () {
  return createAction(`${PREFIX}/toggleMode`)()
}

export function toggleMode (state: WorkingState) {
  let newState = { ...state }
  newState.mode = !state.mode
  newState.boundPoint = []
  newState.workingPoint = [state.workingPoint[0]]
  return newState
}
