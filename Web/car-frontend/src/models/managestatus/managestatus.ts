import * as DVA from 'dva'
import { message } from 'antd'
import { createAction, Action } from 'redux-actions'
import { ManageState, PREFIX,ThemeState,NameState } from './index'
import { IP } from '../main'
declare const BMap

import * as Request from '../../utils/request'

const request: any = Request

export interface ResponseState {//返回信息
  status: string
}
export interface MotorState {// 电机信息
  meter: number,//最大行走距离
  motor: string//电机状态

}
export interface RelayState {// 继电器信息
  relayState: boolean[] //继电器开关状态
  num: number// 继电器编号
}

// 继电器反馈状态信息
interface RelayResponse {
  num: number
  status: number
}

/**
 * 更改继电器的动作
 */
export function requestRelayChangeAction(newItem: number, relay: Array<boolean>) {
  let payload: RelayState = { num: newItem, relayState: relay }
  return createAction<RelayState>(`${PREFIX}/sendRelayData`)(payload)
}

export function* sendRelayData(action: Action<RelayState>, effects: DVA.EffectsCommandMap) {
  let payload = { ...action.payload }
  const status = payload.relayState[payload.num - 1] ? '关闭' : '开启'
  var status1 = 0
  if(status == '关闭')status1 = 17
  else status1 = 0
  message.info(`正在${status}${payload.num}号瓶子，请稍候...`, 3)
  const response1: ResponseState = yield (() => {
    let params = `?id=${payload.num}&act=${payload.relayState[payload.num - 1] ? 17 :0}`
    return request({
      url: `http://${IP}/ocean/control_relay${params}`
    })
  })()
  const response2: ResponseState = yield (() => {
    let params = `?id=${8}&act=${status1}`
    return request({
      url: `http://${IP}/ocean/control_relay${params}`
    })
  })()
  if (response1.status == 'ok' && response2.status == 'ok') {
    message.success(`${payload.num}号瓶子${status}成功！`, 3)
    yield effects.put(onRelayResponseAction(payload.num - 1, 1))
  }
  else {
    message.error(`${payload.num}号瓶子${status}失败！`, 3)
    if (status == "开启") {
      yield effects.put(onRelayResponseAction(payload.num - 1, 2))
    } else {
      yield effects.put(changeRelayAction({num: payload.num - 1, status: 1}))
    }
  }
}

//更改继电器状态
export function changeRelayAction(state: RelayResponse) {
  return createAction<RelayResponse>(`${PREFIX}/changeRelay`)(state)
}

//更换主题颜色
export function updateThemeAction(payload:string){
  return createAction<string>(`${PREFIX}/updateTheme`)(payload)
}

export function updateTheme(state:ThemeState , action: Action<string>){
  let newState:ThemeState = {...state}
  newState.theme = action.payload==null?'false':action.payload
  return newState;
}

//更换列表显示内容
export function updateNameAction(payload:string){
  return createAction<string>(`${PREFIX}/updateName`)(payload)
}

export function updateName(state:NameState , action: Action<string>){
  let newState:NameState = {...state}
  newState.name = action.payload
  return newState
}

//生成随机数
export function updateDataAction(payload:any){
  return createAction<string>(`${PREFIX}/updateData`)(payload)
}

export function *updateData(state:ManageState , action: Action<any>){
  let newState:ManageState = {...state}
  newState.turbidity =  0+100 * Math.random()
  newState.conductivity =  0+ 700 * Math.random()
  newState.oxygen =  0+10 * Math.random()
  newState.temperature = 0+40 * Math.random()
  newState.ph = 0+10 * Math.random()
  return newState
}

export function changeRelay(state: ManageState, action: Action<RelayResponse>) {
  let newState = { ...state }
  const payload = action.payload
  newState.relayState[payload.num] = payload.status == 1
  newState.relayState[7] = payload.status == 1
  // console.log('up')
  // console.log(newState)
  // newState.relayState[7] = payload.status == 1
  newState.relayStatus[payload.num] = 0
  // newState.relayStatus[7] = 0
  newState.relayState = [... newState.relayState]
  return newState
}

//闪烁
export function blinkAction(newItem: number, relay: Array<boolean>) {
  let payload: RelayState = { num: newItem, relayState: relay }
  return createAction<RelayState>(`${PREFIX}/blink`)(payload)
}

export function blink(state: RelayState, action: Action<RelayState>) {
  let newState = { ...state }
  const itemNum = action.payload.num - 1
  let newArray = [ ...newState.relayState ]
  newArray[itemNum] = !newArray[itemNum]
  // newArray[7] = !newArray[7]
  newState.relayState = newArray
  return newState
}

// 继电器返回成功或者失败状态的响应动作
export function onRelayResponseAction(newItem: number, status: number) {
  const payload: RelayResponse = { num: newItem, status: status }
  return createAction<RelayResponse>(`${PREFIX}/onRelayResponse`)(payload)
}

export function onRelayResponse(state: ManageState, action: Action<RelayResponse>) {
  const newState = { ...state }
  const payload = action.payload
  newState.relayStatus[payload.num] = payload.status
  // newState.relayStatus[7] = payload.status;
  newState.relayStatus = [...newState.relayStatus]
  return newState
}

export function onBottleClickAction(item: number) {
  return createAction<number>(`${PREFIX}/bottleClicked`)(item)
}

export function bottleClicked(state: ManageState, action: Action<number>) {
  let newState = { ...state }
  const itemNum = action.payload - 1
  let newArray = [ ...newState.relayClicked ]
  newArray[itemNum] = !newArray[itemNum]
  newState.relayClicked = newArray
  return newState
}

// 关闭取水瓶右侧面版
export function closeRelayPanelAction(item: number) {
  return createAction<number>(`${PREFIX}/closeRelayPanel`)(item)
}

export function closeRelayPanel(state: ManageState, action: Action<number>) {
  let newState = { ...state }
  const itemNum = action.payload - 1
  newState.relayClicked[itemNum] = false
  newState.relayState[itemNum] = false
  newState.relayClicked = [...newState.relayClicked]
  return newState
}

/**
 * 更改电机的动作
 */
export function requestMotorChangeAction(payload1: string, payload2: number) {
  let payload: MotorState = { motor: payload1, meter: payload2 }
  return createAction<MotorState>(`${PREFIX}/sendMotorData`)(payload)
}

export function* sendMotorData(action: Action<MotorState>, effects: DVA.EffectsCommandMap) {
  message.info(`正在更改电机状态，请稍候...`, 3)
  const response: ResponseState = yield (() => {
    let params = `?meter=${action.payload.meter}&act=${action.payload.motor}`
    return request({
      url: `http://${IP}/ocean/control_motor${params}`
    })
  })()
  if (response.status == 'ok') {
    message.success("电机状态更改成功！", 3)
  } else {
    message.error("电机状态更改失败！", 3)
  }
}

//电机上锁
export function disarmAction(){
  return createAction(`${PREFIX}/disarm`)()
}

export function *disarm(state: any , action: Action<any>){
  message.info(`正在对电机进行上锁，请稍后...` , 3)
  const response: {status:String} = yield (() => {
    return request({
      url: `http://${IP}/ocean/disarm`
    })
  })()
  if(response.status == 'ok'){
    message.success("电机上锁成功！" , 3)
  } else {
    message.error("电机上锁失败!" , 3)
  }

}

//电机解锁
export function armAction(){
  return createAction(`${PREFIX}/arm`)()
}

export function *arm(state : any , action:Action<any>){
  message.info(`正在对电机进行解锁，请稍后...` , 3)
  const response: {status:String} = yield(() =>{
    return request({
      url: `http://${IP}/ocean/arm`
    })
  })()
  if(response.status == 'ok'){
    message.success("电机解锁成功!" , 3)
  } else {
    message.error("电机解锁失败!" , 3)
  }
}

//更改电机状态
export function changeMotorAction(payload: string) {
  return createAction<string>(`${PREFIX}/changeMotor`)(payload)
}
export function changeMotor(state: ManageState, action: Action<string>) {
  let newState = { ...state }
  const act = action.payload
  newState.motorAction = act
  return newState
}

//更改电机meter状态
export function changeMeterAction(payload: number) {
  return createAction<number>(`${PREFIX}/changeMeter`)(payload)
}
export function changeMeter(state: ManageState, action: Action<number>) {
  let newState = { ...state }
  const meter = action.payload
  newState.motorMeter = meter
  return newState
}

//更改采样容量
export function volumeChangeAction(value: number, num: number) {
  let payload = { value: value, num: num }
  return createAction<any>(`${PREFIX}/changeVolume`)(payload)
}
export function changeVolume(state: ManageState, action: Action<any>) {
  let newState = { ...state }
  const value = action.payload.value
  let newArray = [ ...newState.volume ]
  newArray[action.payload.num - 1] = value
  newState.volume = newArray
  return newState
}

//更改采样深度
export function depthChangeAction(value: number, num: number) {
  let payload = { value: value, num: num }
  return createAction<any>(`${PREFIX}/changeDepth`)(payload)
}
export function changeDepth(state: ManageState, action: Action<any>) {
  let newState = { ...state }
  const value = action.payload.value
  let newArray = [ ...newState.depth ]
  newArray[action.payload.num - 1] = value
  newState.depth = newArray
  return newState
}
function changeLocation(x: number, y: number) {
  return new Promise(resolve=>{
    let ggPoint = new BMap.Point(x, y)
    let convertor = new BMap.Convertor();
    let pointArr = [];
    pointArr.push(ggPoint);
    convertor.translate(pointArr, 1, 5, (data) => {
      if (data.status === 0) {
        resolve({lng:data.points[0].lng, lat:data.points[0].lat})
      }
    })
  })
}



