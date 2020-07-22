import { sendRelayData, sendMotorData, changeRelay, changeMeter, changeMotor ,closeRelayPanel,
  blink, bottleClicked, changeVolume, changeDepth, onRelayResponse , arm , disarm,updateData} from './managestatus'
  import {updateTheme,updateName} from './managestatus'

export const PREFIX = 'managestatus'

export interface ManageState { //采样管理信息
  relayState: Array<boolean> //继电器开关状态
  motorMeter: number //电机最大行走距离
  motorAction: string //电机状态
  relayClicked: boolean[]//继电器被点击状态，弹出右侧框
  relayStatus: number[] //继电器数据传送状态，有变化时清除取水瓶闪烁效果，0：default，1：success，2：failure
  volume:number[] //采样容量
  depth:number[] //采样深度
  time:string//采样时间
  time1:number//
  lng: number //船体位置经度
  lat: number //船体位置纬度
  meter: number//最大行走距离
  mode1:boolean,//
  ph:number  //ph值
  oxygen:number  //溶氧量
  temperature:number //温度
  turbidity:number   //浊度
  conductivity:number  //电导率

}

export interface ThemeState {
  theme:string//主题 
}

export interface NameState {
  name:string //选中列表名称
}

const initState: ManageState& ThemeState & NameState = {
  motorMeter: 0.25,
  motorAction: '2',
  relayState: [false, false, false, false, false, false, false, false],
  relayClicked: [false, false, false, false, false, false, false, false],
  relayStatus: [0, 0, 0, 0, 0, 0, 0, 0],
  volume:[500, 500, 500, 500, 500, 500, 500, 500],
  depth:[50, 50, 50, 50, 50, 50, 50, 50],
  time:'2018.19:18',
  time1:100,
  lng: 121.445967,
  lat: 31.032097,
  meter:1,
  mode1:false,
  ph:7,
  oxygen:123,
  temperature:123,
  turbidity:123,
  conductivity:123,
  theme:'false',
  name:'任务点'
}

export default {
  namespace: PREFIX,
  state: initState,
  effects: {
    updateData:updateData,
    sendRelayData,
    sendMotorData,
    arm,
    disarm,
  },
  reducers: {
    closeRelayPanel,
    updateTheme:updateTheme,
    updateName:updateName,
    changeRelay,
    changeMeter,
    changeMotor,
    blink,
    bottleClicked,
    changeVolume,
    changeDepth,
    onRelayResponse,
    updateData: updateData
  }
}
