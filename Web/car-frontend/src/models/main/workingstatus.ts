import * as Redux from 'redux'
import * as DVA from 'dva'
import {Action, createAction} from 'redux-actions'
import {ChoseState, IP, PREFIX, RecordState, SubState, ThemeState,vehicle_No, WorkingState} from './index'
import * as coordtransform from 'coordtransform'
import {message} from 'antd'

import * as Request from '../../utils/request'

const request: any = Request
declare const BMap: any

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

//向后端请求剩余路径点
export function oneDataAction(){
  return createAction(`${PREFIX}/oneData`)()
}

export function* oneData(action:Redux.Action ,effects:DVA.EffectsCommandMap){
  const response:{status:string}=yield(()=> {
    return request ({
      url: `http://${IP}/ocean/remain_point`
    })
  })()
  if(response.status=='ok'){
    message.success("获取数据成功", 3)
  }
}

//向后端请求绕开的动作
export function avoidPointAction(){
  return createAction(`${PREFIX}/avoidPoint`)()
}

export function* avoidPoint(action:Redux.Action,effect:DVA.EffectsCommandMap){
  const response:{status:string}=yield(()=>{
    return request({
      url:`http://${IP}/ocean/bypass`
    })
  })()
  if(response.status=='ok'){
    message.success("指令下达成功")
  }
}

//向后端请求数据的动作
export function simuDataAction() {
  return createAction(`${PREFIX}/simuData`)()
}

export function* simuData(action: Redux.Action, effects: DVA.EffectsCommandMap) {
  const response: WorkingState = yield (() => {
    return request({
      url: `http://${IP}/ocean/all`
    })
  })()

  // const newPosition: { lng: number, lat: number } = yield(() => {
  //   return changeLocation(response.lng, response.lat)
  // })()
  // yield effects.put(updateDataAction({...response, lng:newPosition.lng, lat: newPosition.lat}))
  yield effects.put(updateDataAction({...response}))
}


function updateDataAction(payload: WorkingState) {
  return createAction<WorkingState>(`${PREFIX}/updateData`)(payload)
}

//更新节点状态
export function updateData(state: WorkingState, action: Action<WorkingState>) {
  let newState: WorkingState = {...state}
  newState.speed = action.payload.speed
  newState.angle = action.payload.angle
  newState.static = action.payload.static
  newState.communicate = action.payload.communicate
  newState.time = action.payload.time
  newState.battery = action.payload.battery
  newState.radius = action.payload.radius
  newState.voltage1 = action.payload.voltage1
  newState.voltage2 = action.payload.voltage2
  newState.lng = action.payload.lng
  newState.lat = action.payload.lat
  return newState
}

//更换主题颜色
export function updateThemeAction(payload: string) {
  return createAction<string>(`${PREFIX}/updateTheme`)(payload)
}

export function updateTheme(state: ThemeState, action: Action<string>) {
  let newState: ThemeState = {...state}
  newState.theme = action.payload == null ? 'false' : action.payload
  return newState;
}

//更改操控车辆
export function changeVehicle_NoAction(payload: string) {
  return createAction<string>(`${PREFIX}/changeVehicle_No`)(payload)
}

export function changeVehicle_No(state: vehicle_No, action: Action<string>) {
  let newState: vehicle_No = {...state}
  newState.vehicle_No = action.payload == null ? 'false' : action.payload
  return newState;
}

//更换地图栏目显示内容
export function updateKeys(payload: string) {
  return createAction<string>(`${PREFIX}/updateKey`)(payload)
}

export function updateKey(state: ChoseState, action: Action<string>) {
  let newState: ChoseState = {...state}
  newState.keysmap = action.payload
  return newState
}

//更改录像状态
export function updateRecordAction(payload: string) {
  return createAction<string>(`${PREFIX}/updateRecord`)(payload)
}

export function updateRecord(state: RecordState, action: Action<string>) {
  let newState: RecordState = {...state}
  newState.recordCam = action.payload
  return newState
}

//更新差分gps信息
export function updateSubAction(payload: SubState) {
  return createAction<SubState>(`${PREFIX}/updateSub`)(payload)
}

export function updateSub(state: SubState, action: Action<SubState>) {
  let newState: SubState = {...state}
  newState.gps_lat = action.payload.gps_lat
  newState.gps_heading = action.payload.gps_heading
  newState.gps_lng = action.payload.gps_lng
  return newState
}

//向后端请求获取csv文件的内容的动作
export function getWorkingPointsAction(payload) {
  return createAction<any>(`${PREFIX}/getWorkingPoints`)(payload)
}

export function* getWorkingPoints(action: Action<any>, effects: DVA.EffectsCommandMap) {
  const response: { status: string, lng?: number, lat?: number } = yield(() => {
    return request({
      url: `http://${IP}/ocean/get_routes?car=${action.payload.car}&file_name=${action.payload.file_name}`
    })
  })()
  if (response.status == 'ok') {
    yield effects.put(getWorkingPointAction(response))
    message.success("获取数据成功", 3)
  }
}

//向后端请求剩余任务点的动作
export function getMissionPointAction(payload){
  return createAction<any>(`${PREFIX}/getMissionPoint`)(payload)
}

export function* getMissionPoint(action:Action<any>,effects:DVA.EffectsCommandMap){
  const response:{num:number,status:string} = yield(()=>{
    return request({
      url:`http://${IP}/ocean/count_point`
    })
  })()
  if (response.status == 'ok') {
    yield effects.put(getMissionPointAction(response))
    message.success("获取数据成功", 3)
  }
}

//向后端请求获取所有文件的动作
export function getFileListAction(car) {
  return createAction<String>(`${PREFIX}/getFileList`)(car)
}

export function* getFileList(action: Action<String>, effects: DVA.EffectsCommandMap) {
  const response: any = yield (() => {
    return request({
      url: `http://${IP}/ocean/get_csv_files?car=${action.payload}`
    })
  })()
  if (response.status == 'ok') {
    const newResponse: any = yield (() => {
      return request({
        url: `http://${IP}/ocean/get_routes?car=${action.payload}&file_name=${response.file_list[0]}`
      })
    })()
    message.success("获取数据成功", 3)
    if (newResponse.status == 'ok') {
      yield effects.put(updateCarRouteAction({routes: newResponse.routes, file_list: response.file_list}))
      message.success("获取文件成功", 3)
    }
  }
}

// 开始采集命令
export function sendUploadPointAction() {
  return createAction(`${PREFIX}/sendUploadPoint`)()
}

export function* sendUploadPoint(action: Redux.Action, effects: DVA.EffectsCommandMap) {
  const response: { lng: number, lat: number, status: string } = yield(() => {
    return request({
      url: `http://${IP}/ocean/upload_point`
    })
  })()
  if (response.status === 'ok') {
    message.success('发送指令成功')
  }
  yield effects.put(updateWorkingPointAction(response))
}

// 结束采集命令
export function sendUploadPoint2Action() {
  return createAction(`${PREFIX}/sendUploadPoint2`)()
}

export function* sendUploadPoint2(action: Redux.Action, effects: DVA.EffectsCommandMap) {
  const response: { status: string } = yield(() => {
    return request({
      url: `http://${IP}/ocean/upload_point2`
    })
  })()
  if (response.status === 'ok') {
    message.success('发送指令成功')
  }
}

//手机端4个命令
export function sendStartAction(command: number) {
  return createAction<number>(`${PREFIX}/sendStart`)(command)
}

export function* sendStart(action: Action<number>, effects: DVA.EffectsCommandMap) {
  // message.info("命令正在下达 , 请稍候..." , 3)
  message.info("命令已下达，车辆开始启动！", 3)
  const response: { status: string } = yield(() => {
    return request({
      url: `http://${IP}/ocean/mode_frontend?mode=${action.payload}`
    })
  })()
  if (response.status == 'ok') {
    message.success("命令下达成功", 3)
  } else {
    message.error("命令下发失败", 3)
  }
}

//获取全部文件
export function updateCarRouteAction(payload: any) {
  return createAction<any>(`${PREFIX}/updateCarRoute`)(payload)
}

export function updateCarRoute(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.workingPoint = payload.routes
  newState.fileList = payload.file_list
  return newState
}

//获取全部文件
export function getAllFilesAction(payload: any) {
  return createAction<any>(`${PREFIX}/getAllFiles`)(payload)
}

export function getAllFiles(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.fileList = payload.file_list
  return newState
}

//获取全部工作路径点
export function getWorkingPointAction(payload: any) {
  return createAction<any>(`${PREFIX}/getWorkingPoint`)(payload)
}

export function getWorkingPoint(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.workingPoint = payload.routes
  return newState
}

//重置工作路径点
export function resetWorkingPointAction(payload: any) {
  return createAction<any>(`${PREFIX}/resetWorkingPoint`)(payload)
}

export function resetWorkingPoint(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.workingPoint = [{
    lng: payload.lng,
    lat: payload.lat,
    deviceLng: payload.lng,
    deviceLat: payload.lat,
    status: 'todo'
  }]
  return newState
}

//添加工作路径点
export function updateWorkingPointAction(payload: any) {
  return createAction<any>(`${PREFIX}/updateWorkingPoint`)(payload)
}

export function updateWorkingPoint(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.workingPoint = [...state.workingPoint, {
    lng: payload.lng,
    lat: payload.lat,
    deviceLng: payload.lng,
    deviceLat: payload.lat,
    status: 'todo'
  }]
  return newState
}

//删除工作路径点
export function deleteWorkingPointAction(payload: any) {
  return createAction<any>(`${PREFIX}/deleteWorkingPoint`)(payload)
}

export function deleteWorkingPoint(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  const payload = action.payload
  newState.workingPoint = newState.workingPoint.filter((point) => {
    return !(point.lat < payload.lat + 0.00005 && point.lat > payload.lat - 0.00005 &&
      point.lng < payload.lng + 0.00005 && point.lng > payload.lng - 0.00005)
  })
  return newState
}

//优化航线后进行路径点的替换
export function updateAllPointAction(payload: any) {
  return createAction<any>(`${PREFIX}/updateAllPoint`)(payload)
}

export function updateAllPoint(state: WorkingState, action: Action<any>) {
  let newState: WorkingState = {...state}
  newState.workingPoint = action.payload
  return newState
}

//GPS坐标转化为百度坐标
function changeLocation(x: number, y: number) {
  return new Promise(resolve => {
    let ggPoint = new BMap.Point(x, y)
    let convertor = new BMap.Convertor();
    let pointArr = [];
    pointArr.push(ggPoint);
    convertor.translate(pointArr, 1, 5, (data) => {
      if (data.status === 0) {
        resolve({lng: data.points[0].lng, lat: data.points[0].lat})
      }
    })
  })
}

export function sendRoutesAction(payload: { lng: number, lat: number, deviceLng?: number, deviceLat?: number, status: string }[]) {
  return createAction<{ lng: number, lat: number, deviceLng?: number, deviceLat?: number, status: string }[]>(`${PREFIX}/sendRoutes`)(payload)
}

export function* sendRoutes(action: Action<{ lng: number, lat: number, deviceLng?: number, deviceLat?: number, status: string }[]>, effects: DVA.EffectsCommandMap) {
  const GPSPosition = action.payload
  const response: { status: string } = yield(() => {
    let query = `?count=${GPSPosition.length}`
    GPSPosition.forEach((point, i) => {
      query += `&lng${i}=${point.deviceLng.toFixed(6)}&lat${i}=${point.deviceLat.toFixed(6)}`
    })
    return request({
      url: `http://${IP}/ocean/upload_point${query}`
    })
  })()
  if (response.status == 'ok') {
    message.success("工作路径设置成功", 3)
  } else {
    message.error("工作路径设置失败", 3)
  }
}

export function sendCpOpenFileAction(payload: any) {
  return createAction<any>(`${PREFIX}/sendCpOpenFile`)(payload)
}

export function* sendCpOpenFile(action: Action<any>, effects: DVA.EffectsCommandMap) {
  const fileName = action.payload
  const response: { status: string } = yield(() => {
    return request({
      url: `http://${IP}/ocean/cp_open_file?file_name=${fileName}`
    })
  })()
  if (response.status == 'ok') {
    message.success("文件复制成功", 3)
  } else {
    message.error("文件复制失败", 3)
  }
}

//百度坐标转化为GPS坐标(官方没有提供相应转换方式，采用github上的coordtransform库实现)
function changeLocationArray(points: { lng: number, lat: number }[]) {
  return points.map((point) => {
    const gcj02Coord = coordtransform.bd09togcj02(point.lng, point.lat); //百度经纬度坐标转国测局坐标
    const wgs84Coord = coordtransform.gcj02towgs84(gcj02Coord[0], gcj02Coord[1]); //国测局坐标转wgs84坐标
    return {lng: wgs84Coord[0], lat: wgs84Coord[1]}
  })
}

//计算总距离
export function findDistanceAction(points: Array<{ lng: number, lat: number, status: string }>) {
  return createAction<Array<{ lng: number, lat: number, status: string }>>(`${PREFIX}/findDistance`)(points)
}

export function findDistance(state: WorkingState, action: Action<Array<{ lng: number, lat: number, status: string }>>) {
  let newState = {...state}
  const points = action.payload
  let distance = [0]
  let pointArr = points.map((point) => {
    return new BMap.Point(point.lng, point.lat)
  })
  let map = new BMap.Map("map")
  for (let i = 0; i < pointArr.length - 1; i++) {
    distance.push(map.getDistance(pointArr[i], pointArr[i + 1]))
  }
  newState.totalDistance = distance
  return newState
}

//添加平扫关键点的动作
export function addBoundPointAction(payload: { lng: number, lat: number }) {
  return createAction<{ lng: number, lat: number }>(`${PREFIX}/addBoundPoint`)(payload)
}

export function addBoundPoint(state: WorkingState, action: Action<{ lng: number, lat: number }>) {
  let newState = {...state}
  const payload = action.payload
  newState.boundPoint = [...newState.boundPoint, {lng: payload.lng, lat: payload.lat}]
  return newState
}

//计算平扫路径的动作
export function calculateRouteAction() {
  return createAction(`${PREFIX}/calculateRoute`)()
}

export function calculateRoute(state: WorkingState) {
  let newState = {...state}
  const boundPoint = state.boundPoint
  boundPoint.sort((pointA, pointB) => { //对四个坐标点进行位置排序
    return pointA.lng < pointB.lng ? 1 : -1
  })
  const [pointNW, pointSW] = boundPoint.slice(0, 2).sort((pointA, pointB) => {
    return pointA.lat > pointB.lat ? 1 : -1
  })
  const [pointNE, pointSE] = boundPoint.slice(2).sort((pointA, pointB) => {
    return pointA.lat > pointB.lat ? 1 : -1
  })
  const num = 3 //每条边上的扫描点数
  const pointsNorth: { lng: number, lat: number }[] = []
  for (let i = 0; i <= num; i++) {
    const lat = (pointNW.lat - pointNE.lat) / num * i + pointNE.lat
    const lng = (pointNW.lng - pointNE.lng) / num * i + pointNE.lng
    pointsNorth.push({lng, lat})
  }
  const pointsSouth: { lng: number, lat: number }[] = []
  for (let j = 0; j <= num; j++) {
    const lat = (pointSW.lat - pointSE.lat) / num * j + pointSE.lat
    const lng = (pointSW.lng - pointSE.lng) / num * j + pointSE.lng
    pointsSouth.push({lng, lat})
  }
  for (let k = 0; k <= num; k++) {
    newState.workingPoint.push({lng: pointsNorth[k].lng, lat: pointsNorth[k].lat, status: 'todo'})
    newState.workingPoint.push({lng: pointsSouth[k].lng, lat: pointsSouth[k].lat, status: 'todo'})
  }
  setTimeout(() => {
    message.success("路径计算成功！", 3)
  }, 1000)
  return newState
}
