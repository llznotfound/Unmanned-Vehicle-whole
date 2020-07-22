import {
  addBoundPoint,
  calculateRoute,
  changeVehicle_No,
  deleteWorkingPoint,
  findDistance,
  getAllFiles,
  getFileList,
  getWorkingPoint,
  getWorkingPoints,
  resetWorkingPoint,
  sendCpOpenFile,
  sendRoutes,
  sendStart, sendUploadPoint, sendUploadPoint2,
  simuData,
  updateAllPoint,
  updateCarRoute,
  updateData,
  updateKey,
  updateRecord,
  updateSub,
  updateTheme,
  updateWorkingPoint,
  oneData,
  avoidPoint
} from './workingstatus'
import {changeAlertTime, clearAlertTime, sendReturn, toggleMode} from './changeTime'

export const PREFIX = 'planstatus'
export const IP = "localhost:8484"//改成rails运行的电脑的ip地址

export interface WorkingState {//船体工作状态信息
  fileList: any; // 所有文件
  speed: number//速度
  angle: number//航向角
  communicate: number//通信链路质量
  time: number//完成时间预估
  battery: number//电池
  radius: number//半径警告
  static: boolean // 是否静止
  lng: number //船体位置经度
  lat: number //船体位置纬度
  workingPoint: Array<{ lng: number, lat: number, deviceLng?: number, deviceLat?: number, status: string }> //工作路径点数组
  totalDistance: Array<number>//任务总距离
  alertMode: boolean //是否进入警告模式
  alertTime: number //警告模式30秒倒计时
  mode: boolean //true为连线模式，false为平扫模式
  boundPoint: Array<{ lng: number, lat: number }> //平扫模式下的四个边界点
  voltage1: number//左电压
  voltage2: number//右电压
  temperature: number  //温度
}

export interface TimeState {
  times: number //倒计时
}

export interface ManageState { //采样管理信息
  relayState: Array<boolean> //继电器开关状态
  motorMeter: number //电机最大行走距离
  motorAction: string //电机状态
  relayClicked: boolean[]//继电器被点击状态，弹出右侧框
  relayStatus: number[] //继电器数据传送状态，有变化时清除取水瓶闪烁效果，0：default，1：success，2：failure
  volume: number[] //采样容量
  depth: number[] //采样深度
  meter: number//最大行走距离
  mode1: boolean,//

}

export interface ThemeState {
  theme: string//主题
}

export interface PlansState {
  plans: string  //传感器数据是否在读取
}

export interface ManasState {
  mana: string //采样操作是否进行中
}

export interface ChoseState {
  keysmap: string  //地图栏目选中key值
}

export interface SubState {
  gps_lng: number  //差分经度
  gps_lat: number //差分纬度
  gps_heading: number
}

export interface UpdaRoadState {
  upda_road: string  //更新路径操作是否进行中
}

export interface RecordState {
  recordCam: string //是否录像中
}

export interface vehicle_No {
  vehicle_No: string
}

const initState: WorkingState & TimeState & ManageState & ThemeState & PlansState & ManasState & ChoseState & SubState & UpdaRoadState & RecordState & vehicle_No = {
  fileList: [],
  speed: 1,//速度
  angle: 78,//航向角
  communicate: 78,//通信链路质量
  time: 700,//完成时间预估
  battery: 100,//电池
  static: false, // 静止默认为false
  lng: 121.4494651085,  //经度
  lat: 31.0313365968, //纬度
  radius: 9.9,//半径警告,
  voltage1: 25.2, //左电压
  voltage2: 25.2, //右电压
  workingPoint: [],
  totalDistance: [0],
  alertMode: false,
  alertTime: 30,
  mode: true,
  boundPoint: [],
  times: 10,
  temperature: 28.6,
  motorMeter: 0.25,
  motorAction: '2',
  relayState: [false, false, false, false, false, false, false, false],
  relayClicked: [false, false, false, false, false, false, false, false],
  relayStatus: [0, 0, 0, 0, 0, 0, 0, 0],
  volume: [500, 500, 500, 500, 500, 500, 500, 500],
  depth: [50, 50, 50, 50, 50, 50, 50, 50],
  meter: 1,
  mode1: false,
  theme: 'false',
  mana: 'false',
  plans: 'true',
  keysmap: 'map',
  gps_lng: 121.535443069,
  gps_lat: 30.523782871,
  gps_heading: 220.62399292,
  upda_road: 'false',
  recordCam: 'false',
  vehicle_No: 'A'
}

export default {
  namespace: PREFIX,
  state: initState,
  effects: {
    simuData,
    sendReturn,
    sendStart,
    sendRoutes,
    sendCpOpenFile,
    getFileList,
    getWorkingPoints,
    sendUploadPoint,
    sendUploadPoint2,
    oneData,
    avoidPoint
  },
  reducers: {
    updateData,
    updateTheme,
    getWorkingPoint,
    getAllFiles,
    updateCarRoute,
    updateWorkingPoint,
    resetWorkingPoint,
    deleteWorkingPoint,
    findDistance,
    changeAlertTime,
    clearAlertTime,
    toggleMode,
    addBoundPoint,
    calculateRoute,
    updateKey,
    updateSub,
    updateRecord,
    updateAllPoint,
    changeVehicle_No,
  }
}
