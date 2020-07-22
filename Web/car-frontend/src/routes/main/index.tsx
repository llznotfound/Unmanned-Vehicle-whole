import * as React from 'react'
import {connect} from 'dva'
import * as Redux from 'redux'

import {media, style} from 'typestyle'
import 'antd/dist/antd.css'
import {Button, Icon, Menu, message, Popover, Switch} from 'antd'
import SaveSpace from '../../components/main/savespace'
import MissionTabler from '../../components/main/table'
import * as coordtransform from 'coordtransform'
import {
  ChoseState,
  ManageState,
  ManasState,
  PlansState,
  PREFIX,
  RecordState,
  ThemeState,
  UpdaRoadState,
  vehicle_No,
  WorkingState
} from '../../models/main'
import {Map, Marker} from 'react-bmap'
import {
  addBoundPointAction,
  calculateRouteAction,
  changeVehicle_NoAction,
  findDistanceAction,
  sendStartAction,
  simuDataAction,
  updateAllPointAction,
  updateKeys,
  updateRecordAction,
  updateThemeAction,
  avoidPointAction
} from '../../models/main/workingstatus'
import ContextMenu from '../../components/main/contextMenu'
import WorkingPoint from '../../components/main/workingPoint'
import {
  changeAlertTimeAction,
  clearAlertTimeAction,
  sendReturnAction,
  toggleModeAction
} from '../../models/main/changeTime'

type MainProps = WorkingState
type ManageProps = ManageState
type ThemeProps = ThemeState
type PlansProps = PlansState
type ManasProps = ManasState
type ChoseProps = ChoseState
type UpdaRoadProps = UpdaRoadState
type RecordProps = RecordState
type VehicleNoProps=vehicle_No
interface MainDispatch {
  dispatch: Redux.Dispatch<any>
}
declare function require(path: string): any
declare let BMap
declare const BMAP_SATELLITE_MAP
declare const BMAP_HYBRID_MAP
declare const WebVideoCtrl
const batt = require('../../assets/Battery.png')
const boat = require('../../assets/car.png') //获取无人船的图标
const circle = require('../../assets/circle.png') //获取警告模式下黄色圆圈的图标
const initImage = require('../../assets/init.jpg') //获取警告模式下黄色圆圈的图标
var tnum = 1;//距离数组初始化
var data = [];//热力图数据数组
var pra = '启动'
var pra1= '直行'
var Automode= false //是否导航状态
var hantime = 0;
class Main extends React.Component<MainProps & MainDispatch & ManageProps & ThemeProps & PlansProps & ManasProps & ChoseProps & UpdaRoadProps & RecordProps &VehicleNoProps, any> {

  t:any ///存储setInterval的返回值用于清理
  t1:any//存储计时器
  t2:any //警告模式闪烁效果计时器
  timer:any
  t3: any[] = [null, null, null, null, null, null, null, null]//存放每个瓶子的时间
  th:any //存储主题
  tm:any[]=[0,0,0,0,0,0]  //存放6个瓶子采样的时间

  lngs:any //存储输入框中的经度
  lats:any //存储输入框中的纬度

  ips:any //存储输入框中的ip
  door:any //存储输入框中的端口
  userNames:any //存储输入框中的用户名
  passwords:any //存储输入框中的密码

  imageStyle = style({
    position: "fixed",
    zIndex: 100,
    left: "250px",
    top: "100px"
  })

  constructor(props) {
    super(props);
    this.state = {
      show: true,
      batts:100
    }
  }

  // manaInfo: Array<{lng:number, lat:number, time:string}> = [
  // {lng:0,lat:0,time:null},{lng:0,lat:0,time:null},
  // {lng:0,lat:0,time:null},{lng:0,lat:0,time:null},
  // {lng:0,lat:0,time:null},{lng:0,lat:0,time:null}]  //存放每个瓶子取样的时间和经纬度

  //count: number[] = [0, 0, 0, 0, 0, 0, 0, 0]//记录每个瓶子闪烁的次数
  render(){
    const wholeStyle = style ({
      // height:'909px',
      height:'100vh',
      // width:'1600px',
      backgroundColor:this.props.theme == "false"?'#424250':'#e6e6e6',
    })
    const vehicle_select = style ({ //车辆选择栏样式
        position:'absolute',
        top:'17px',
        left:'422px',
        width:'170px',
        height:'100px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'500px',
          height:'110px',
          left:'36px',
          top:'19px'
        }
      ))

    const vehicle_selectTextStyle = style ({  //车辆选择字体样式
        position:'absolute',
        top:'10px',
        left:'13px',
        width:'134.8px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'202px',
          height:'49px',
          left:'160px',
          top:'31px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))
    const vehicle_current = style ({  //当前车辆样式
        position:'absolute',
        top:'41px',
        left:'13px',
        width:'40px',
        height:'44px',
        fontFamily:'Impact',
        fontSize:'36px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'60px',
          height:'48px',
          left:'20px',
          top:'45px',
          fontSize:'30px'
        }
      ))
    const buttonText1 = style ({ //车辆选择按钮文字样式
        position:'absolute',
        top:'50px',
        left:'0px',
        width:'100px',
        height:'28px',
        fontFamily:"PingFangSC",
        fontSize:'20px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.1px',
        textAlign:'center',
        color:this.props.theme == "false"?'#fff':'#000'
      },
      media({minWidth:1700},
        {
          width:'120px',
          // height:'31px',
          left:'-25px',
          top:'13px',
          // fontWeight:900,
          fontSize:'25px'
        }
      ))
    const Button_vehicle_selectA = style ({ //车辆切换A按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'530px',
          top:'0px'
        }
      ))
    const Button_vehicle_selectB = style ({ //车辆切换B按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'610px',
          top:'0px'
        }
      ))
    const Button_vehicle_selectC = style ({ //车辆切换C按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'530px',
          top:'55px'
        }
      ))
    const Button_vehicle_selectD = style ({ //车辆切换D按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'610px',
          top:'55px'
        }
      ))

    const Button_vehicle_selectE = style ({ //车辆切换E按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'690px',
          top:'0px'
        }
      ))
    const Button_vehicle_selectF = style ({ //车辆切换F按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'770px',
          top:'0px'
        }
      ))
    const Button_vehicle_selectG = style ({ //车辆切换G按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'690px',
          top:'55px'
        }
      ))
    const Button_vehicle_selectH = style ({ //车辆切换H按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'70px',
          height:'50px',
          left:'770px',
          top:'55px'
        }
      ))

    const status = style ({ //任务状态栏目
        left:'24px',
        top:'17px',
        width:'380px',
        height:'159px',
        borderRadius:'3px',
        // backgroundColor:'#363636',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        position:'absolute'
      },
      media({minWidth:1700},
        {
          width:'850px',
          height:'140px',
          left:'36px',
          top:'155px'
        }
      ))

    const missionStatus = style ({  //任务状态栏目文字属性
        position:'absolute',
        left:'15px',
        top:'10px',
        width:'317px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'476px',
          height:'49px',
          left:'23px',
          top:'5px',
          fontSize:'25px',
          lineHeight:2.04,
          fontWeight:900
        }
      ))

    const lineBetween = style ({  //分割线
        position:'absolute',
        width:'380px',
        height:'1px',
        opacity:0.24,
        background:'#297fca',
        top:'46px'
      },
      media({minWidth:1700},
        {
          width:'850px',
          height:'1.1px',
          top:'51px'
        }
      ))

    const timeTextStyle = style ({ //预计剩余时间文字样式
        position:'absolute',
        top:'54px',
        left:'14px',
        width:'88px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'14px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.86,
        letterSpacing:'0.5px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'300px',
          height:'49px',
          left:'21px',
          top:'79px',
          fontSize:'25px',
          lineHeight:2.04,
          fontWeight:900
        }
      ))

    const timeStyle = style ({ //预计剩余时间样式
        position:'absolute',
        top:'83px',
        left:'17px',
        width:'86px',
        height:'51px',
        fontFamily:'Impact',
        fontSize:'42px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'129px',
          height:'56px',
          left:'26px',
          top:'91px',
          fontSize:'20px',
        }
      ))

    const distanceTextStyle = style ({  //已出发字体样式
        position:'absolute',
        top:'54px',
        left:'151px',
        width:'44px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'14px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.86,
        letterSpacing:'0.5px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'200px',
          height:'49px',
          left:'334px',
          top:'79px',
          fontSize:'25px',
          lineHeight:2.04
        }
      ))

    const distanceStyle = style ({  //出发数字样式
        position:'absolute',
        top:'83px',
        left:'150px',
        width:'78px',
        height:'51px',
        fontFamily:'Impact',
        fontSize:'42px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.8px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'117px',
          height:'56px',
          left:'225px',
          top:'91px',
          fontSize:'20px',
        }
      ))

    const distanceCompanyStyle = style ({ //出发单位样式
        position:'absolute',
        fontSize:'22px',
        letterSpacing:'4.6px',
        top:'17px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'20px'
        }
      ))

    const missionPointStyle = style ({  //任务点剩余样式
        position:'absolute',
        top:'54px',
        left:'261px',
        width:'73px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'14px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.86,
        letterSpacing:'0.5px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'240px',
          height:'49px',
          left:'602px',
          top:'79px',
          fontSize:'25px',
          lineHeight:2.04,
          fontWeight:900
        }
      ))

    const leftPointStyle = style ({ //剩余点数样式
        position:'absolute',
        top:'81px',
        left:'260px',
        width:'79px',
        height:'51px',
        fontFamily:'Impact',
        fontSize:'42px',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.8px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'119px',
          height:'56px',
          left:'390px',
          top:'89px',
          fontSize:'20px'
        }
      ))

    const totalPointStyle = style ({  //总点数样式
        position:'absolute',
        top:'17px',
        fontSize:'22px',
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'20px'
        }
      ))

    const speed = style ({  //速度栏目样式
        position:'absolute',
        top:'187px',
        left:'24px',
        width:'187px',
        height:'144px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'36px',
          top:'320px'
        }
      ))

    const speedTextStyle = style ({ //速度文字样式
        position:'absolute',
        top:'10px',
        left:'15px',
        width:'150px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'225px',
          height:'49px',
          left:'113px',
          top:'33px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))

    const speedLine = style ({  //速度栏分割线
        position:'absolute',
        top:'46px',
        width:'187px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'281px',
          height:'1.1px',
          top:'51px'
        }
      ))

    const speedNumStyle = style ({  //速度样式
        position:'absolute',
        top:'54px',
        left:'19px',
        width:'102px',
        height:'56px',
        fontFamily:'Impact',
        fontSize:'46px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.7px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'153px',
          height:'62px',
          left:'29px',
          top:'59px',
          fontSize:'25px'
        }
      ))

    const speedComStyle = style ({  //速度单位样式
        position:'absolute',
        top:'17px',
        fontSize:'26px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'15px'
        }
      ))

    const accelerate = style ({  //加速度栏目样式
        position:'absolute',
        top:'250px',
        left:'24px',
        width:'187px',
        height:'144px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'36px',
          top:'430px'
        }
      ))
    const accTextStyle = style ({ //加速度文字样式
        position:'absolute',
        top:'10px',
        left:'15px',
        width:'150px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'225px',
          height:'49px',
          left:'113px',
          top:'33px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))
    const accLine = style ({  //加速度栏分割线
        position:'absolute',
        top:'46px',
        width:'187px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'281px',
          height:'1.1px',
          top:'51px'
        }
      ))
    const accNumStyle = style ({  //加速度样式
        position:'absolute',
        top:'54px',
        left:'19px',
        width:'102px',
        height:'56px',
        fontFamily:'Impact',
        fontSize:'46px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.7px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'153px',
          height:'62px',
          left:'29px',
          top:'59px',
          fontSize:'25px'
        }
      ))

    const accComStyle = style ({  //加速度单位样式
        position:'absolute',
        top:'17px',
        fontSize:'26px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'15px'
        }
      ))
    const power = style ({  //功率栏目样式
        position:'absolute',
        top:'250px',
        left:'24px',
        width:'187px',
        height:'144px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'36px',
          top:'540px'
        }
      ))
    const powerTextStyle = style ({ //功率文字样式
        position:'absolute',
        top:'10px',
        left:'15px',
        width:'150px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'225px',
          height:'49px',
          left:'113px',
          top:'33px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))
    const powerLine = style ({  //功率栏分割线
        position:'absolute',
        top:'46px',
        width:'187px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'272px',
          height:'1.1px',
          top:'51px'
        }
      ))
    const powerNumStyle = style ({  //功率样式
        position:'absolute',
        top:'54px',
        left:'19px',
        width:'102px',
        height:'56px',
        fontFamily:'Impact',
        fontSize:'46px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.7px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'153px',
          height:'62px',
          left:'29px',
          top:'59px',
          fontSize:'25px'
        }
      ))

    const powerComStyle = style ({  //功率单位样式
        position:'absolute',
        top:'17px',
        fontSize:'26px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'15px'
        }
      ))

    const location = style ({ //当前位置栏样式
        position:'absolute',
        top:'340px',
        left:'424px',
        width:'170px',
        height:'100px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'503px',
          top:'430px'
        }
      ))

    const locationTextStyle = style ({  //当前位置字体样式
        position:'absolute',
        top:'10px',
        left:'13px',
        width:'134.8px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'202px',
          height:'49px',
          left:'80px',
          top:'25px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))

    const locationNumStyle = style ({  //当前位置坐标样式
        position:'absolute',
        top:'54px',
        left:'19px',
        width:'102px',
        height:'56px',
        fontFamily:'Impact',
        fontSize:'46px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.7px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'153px',
          height:'62px',
          left:'200px',
          top:'20px',
          fontSize:'25px'
        }
      ))

    const speedSliderStyle = style ({ //速度控件样式
      position:'absolute',
      top:'119px',
      left:'12px',
      width:'114px',
      height:'5px',
      borderRadius:'2.5px',
      color:'#2298ff'
    },media({minWidth:1700},
      {
        top:'131px',
        left:'29px',
        width:'210px',
        height:'6px'
      }
    ))

    const sport = style ({  //运动角度栏目样式
        position:'absolute',
        top:'187px',
        left:'222px',
        width:'181px',
        height:'144px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'503px',
          top:'320px'
        }
      ))

    const sportTextStyle = style ({ //运动角度文字样式
        position:'absolute',
        top:'10px',
        left:'15px',
        width:'144px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'216px',
          height:'49px',
          left:'113px',
          top:'33px',
          fontWeight:900,
          fontSize:'25px',
          lineHeight:1.79
        }
      ))

    const sportLine = style ({  //运动栏分割线
        position:'absolute',
        top:'46px',
        width:'181px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'272px',
          height:'1.1px',
          top:'51px'
        }
      ))

    const sportNumStyle = style ({  //运动角度样式
        position:'absolute',
        top:'57px',
        left:'17px',
        width:'54px',
        height:'56px',
        fontFamily:'Impact',
        fontSize:'46px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.7px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'81px',
          height:'62px',
          left:'20px',
          top:'63px',
          fontSize:'25px'
        }
      ))

    const sportPieStyle = style ({  //运动角度控件样式
      position:'absolute',
      top:'50px',
      left:'70px',
      width:'64px',
      height:'67px',
      // border:'solid 2px #808087'
    },media({minWidth:1700},
      {
        top:'55px',
        left:'160px',
        width:'96px',
        height:'74px',
      }
    ))

    const save = style ({ //存储空间栏样式
        position:'absolute',
        top:'17px',
        left:'422px',
        width:'170px',
        height:'100px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'255px',
          height:'110px',
          left:'633px',
          top:'19px'
        }
      ))

    const saveTextStyle = style ({  //存储空间字体样式
        position:'absolute',
        top:'10px',
        left:'13px',
        width:'134.8px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'202px',
          height:'29px',
          left:'20px',
          top:'11px',
          fontSize:'24px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))

    const saveStyle = style ({  //存储空间样式
        position:'absolute',
        top:'41px',
        left:'13px',
        width:'40px',
        height:'44px',
        fontFamily:'Impact',
        fontSize:'36px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'60px',
          height:'48px',
          left:'20px',
          top:'45px',
          fontSize:'54px'
        }
      ))

    const saveComStyle = style ({ //存储空间单位样式
        position:'absolute',
        top:'17px',
        fontSize:'16px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'24px'
        }
      ))

    const saveSpaceStyle = style ({ //存储空间右方圆盘样式
      position:'absolute',
      top:'10px',
      left:'60px',
      width:'52px',
      height:'52px'
    },media({minWidth:1700},
      {
        top:'15px',
        left:'121px',
        width:'73px',
        height:'57px'
      }
    ))

    const communicate = style ({ //通信质量栏样式
        position:'absolute',
        top:'126px',
        left:'422px',
        width:'170px',
        height:'100px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'255px',
          height:'110px',
          left:'633px',
          top:'155px'
        }
      ))

    const communicateTextStyle = style ({  //通信质量字体样式
        position:'absolute',
        top:'10px',
        left:'13px',
        width:'134.8px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'202px',
          height:'29px',
          left:'20px',
          top:'11px',
          fontWeight:900,
          fontSize:'24px',
          lineHeight:1.79
        }
      ))

    const communicateStyle = style ({  //通信质量样式
        position:'absolute',
        top:'43px',
        left:'13px',
        width:'40px',
        height:'44px',
        fontFamily:'Impact',
        fontSize:'36px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'60px',
          height:'48px',
          left:'20px',
          top:'47px',
          fontSize:'54px',
        }
      ))

    const communicateComStyle = style ({ //通信质量单位样式
        position:'absolute',
        top:'17px',
        fontSize:'16px'
      },
      media({minWidth:1700},
        {
          top:'27px',
          fontSize:'24px'
        }
      ))

    const signFirst = style ({ //通信质量第一根信号样式
      position:'absolute',
      left:'96px',
      top:'69.8px',
      width:'4.1px',
      height:'12.2px',
      backgroundColor:'#2298ff'
    },media({minWidth:1700},
      {
        left:'134px',
        top:'76px',
        width:'5.7px',
        height:'13.4px'
      }
    ))

    const signSecond = style ({ //通信质量第二根信号样式
      position:'absolute',
      left:'110.4px',
      top:'64.2px',
      width:'4.1px',
      height:'17.8px',
      backgroundColor:'#2298ff'
    },media({minWidth:1700},
      {
        left:'154.56px',
        top:'70.62px',
        width:'5.7px',
        height:'19.58px'
      }
    ))

    const signThird = style ({ //通信质量第三根信号样式
      position:'absolute',
      left:'124.9px',
      top:'52px',
      width:'4.1px',
      height:'29.2px',
      backgroundColor:'#2298ff'
    },media({minWidth:1700},
      {
        left:'174.86px',
        top:'57px',
        width:'5.7px',
        height:'32.12px'
      }
    ))

    const signForth = style ({ //通信质量第四根信号样式
      position:'absolute',
      left:'138px',
      top:'36.7px',
      width:'4.1px',
      height:'45.1px',
      backgroundColor:'#808087'
    },media({minWidth:1700},
      {
        left:'193.2px',
        top:'40.37px',
        width:'5.7px',
        height:'49.61px'
      }
    ))

    const signFifth = style ({ //通信质量第五根信号样式
      position:'absolute',
      left:'152.2px',
      top:'14px',
      width:'4.1px',
      height:'67.5px',
      backgroundColor:'#808087'
    },media({minWidth:1700},
      {
        left:'213.08px',
        top:'15.4px',
        width:'5.7px',
        height:'74.25px'
      }
    ))

    const battery = style ({ //电量栏样式
        position:'absolute',
        top:'255px',
        left:'424px',
        width:'170px',
        height:'100px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'381px',
          height:'100px',
          left:'503px',
          top:'540px'
        }
      ))

    const batteryTextStyle = style ({  //电量字体样式
        position:'absolute',
        top:'10px',
        left:'13px',
        width:'134.8px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'202px',
          height:'29px',
          left:'90px',
          top:'1px',
          fontSize:'25px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))

    const batteryStyle = style ({  //电量样式
        position:'absolute',
        top:'44px',
        left:'12px',
        width:'40px',
        height:'44px',
        fontFamily:'Impact',
        fontSize:'36px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'60px',
          height:'48px',
          left:'88px',
          top:'38px',
          fontSize:'54px'
        }
      ))

    const batteryComStyle = style ({ //电量单位样式
        position:'absolute',
        top:'12px',
        fontSize:'16px',
        left:'40px',
        width:'33px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'0.4px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'50px',
          height:'24px',
          top:'25px',
          left:'85px',
          fontWeight:900,
          fontSize:'24px'
        }
      ))

    const batteryPosStyle = style ({  //电池控件样式
      position:'absolute',
      top:'17.5px',
      left:'101.5px',
      width:'65px',
      height:'66px',
    },media({minWidth:1700},
      {
        top:'19.25px',
        left:'256.1px',
        width:'90px',
        height:'72.6px'
      }
    ))

    const batteryColor = style ({ //根据电量判定电池图标着色样式
      position:'absolute',
      top:'22.5px',
      left:'120.5px',
      width:'28px',
      height:'58.6px',
      backgroundColor:'#b61313',
      borderRadius:'3px',
      borderTop:'solid 3.1px #000000'
    },media({minWidth:1700},
      {
        top:'25.25px',
        left:'182px',
        width:'40px',
        height:'61.6px'
      }
    ))

    const manage = style ({ //采样管理栏
        position:'absolute',
        top:'344px',
        left:'26px',
        width:'568px',
        height:'185px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'852px',
          height:'244px',
          left:'39px',
          top:'378px'
        }
      ))

    const manageTextStyle = style ({  //采样管理文字样式
        position:"absolute",
        top:'10px',
        left:'15px',
        width:'481px',
        height:'26px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },
      media({minWidth:1700},
        {
          width:'722px',
          height:'29px',
          left:'23px',
          top:'11px',
          fontSize:'24px',
          fontWeight:900,
          lineHeight:1.79
        }
      ))

    const manageLine = style ({ //采样栏分割线
        position:'absolute',
        top:'46px',
        width:'568px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'852px',
          height:'1.1px',
          top:'51px'
        }
      ))

    const manageSwitch = style ({ //采样管理switch样式
      position:'absolute',
      top:'12px',
      left:'461px',
      width:'92px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'13px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'left',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3'
    },media({minWidth:1700},
      {
        top:'13.2px',
        left:'750px',
        // width:'128.8px',
        // height:'26.4px',
        fontSize:'18px',
      }
    ))



    const photo = style ({  //照片栏
        position:'absolute',
        top:'551px',
        left:'24px',
        width:'570px',
        height:'225px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'855px',
          height:'244px',
          left:'36px',
          top:'646px'
        }
      ))

    const photoLine = style ({  //照片栏分割线
        position:'absolute',
        top:'36px',
        width:'570px',
        height:'1px',
        opacity:0.24,
        backgroundColor:'#297fca'
      },
      media({minWidth:1700},
        {
          width:'855px',
          top:'40px'
        }
      ))

    const photoMenu = style ({  //照片栏导航栏
        position:'absolute',
        top:'0px',
        width:'570px',
        height:'28px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        borderRadius:'1px',
        borderBottom:'solid 1px #297fca',
        fontSize:'16px',
        fontFamily:'PingFangSC',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        color:this.props.theme == "false"?'#ffffff':'#000'
        // border:'solid 3px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'855px',
          height:'38px',
          fontSize:'22.4px',
          fontWeight:840
        }
      ))

    const backButton = style ({ //返航按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border: 'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'936px',
          top:'20px'
        }
      ))

    const buttonIcon = style ({ //按钮中图片样式
        position:'absolute',
        top:'13px',
        width:'100px',
        height:'30px',
        left:'0px',
        fontFamily:'simple-line-icons',
        fontSize:'30px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'normal',
        textAlign:'center',
        color:this.props.theme == "false"?'#fff':'#000'
      },
      media({minWidth:1700},
        {
          width:'120px',
          top:'14px',
          height:'33px',
        }
      ))

    const buttonText = style ({ //按钮文字样式
        position:'absolute',
        top:'50px',
        left:'0px',
        width:'100px',
        height:'28px',
        fontFamily:"PingFangSC",
        fontSize:'20px',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'1.1px',
        textAlign:'center',
        color:this.props.theme == "false"?'#fff':'#000'
      },
      media({minWidth:1700},
        {
          width:'120px',
          // height:'31px',
          // left:'0px',
          // top:'50px',
          // fontWeight:900,
          fontSize:'25px'
        }
      ))

    const Button1_0 = style ({ //启动按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'936px',
          top:'20px'
        }
      ))

    const Button1_1 = style ({ //启动按钮样式
        position:'absolute',
        top:'36px',
        left:'742px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1090px',
          top:'20px'
        }
      ))

    const Button1_2 = style ({ //返航按钮样式
        position:'absolute',
        top:'36px',
        left:'859px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1240px',
          top:'20px'
        }
      ))

    const Button1_3 = style ({  //停止按钮样式
        position:'absolute',
        top:'36px',
        left:'977px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1395px',
          top:'20px'
        }
      ))

    const Button1_4 = style ({ //更新路径按钮样式
        position:'absolute',
        top:'36px',
        left:'1095px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1550px',
          top:'20px'
        }
      ))

      const Button1_5 = style ({ //更新路径按钮样式
        position:'absolute',
        top:'36px',
        left:'1095px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1705px',
          top:'20px'
        }
      ))

    // const Button2_0 = style ({ //预留2_0按钮样式
    //     position:'absolute',
    //     top:'36px',
    //     left:'624px',
    //     width:'100px',
    //     height:'87px',
    //     borderRadius:'2px',
    //     backgroundColor:this.props.theme == "false"?'#363636':'#fff',
    //     border: 'solid 1px #2298ff'
    //   },
    //   media({minWidth:1700},
    //     {
    //       width:'120px',
    //       height:'96px',
    //       left:'936px',
    //       top:'130px'
    //     }
    //   ))
    const Button2_1 = style ({ //直走/停止按钮样式
        position:'absolute',
        top:'36px',
        left:'624px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'936px',
          top:'130px'
        }
      ))
    const Button2_2 = style ({ //后退按钮样式
        position:'absolute',
        top:'36px',
        left:'742px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1090px',
          top:'130px'
        }
      ))
    const Button2_3 = style ({ //左转按钮样式
        position:'absolute',
        top:'36px',
        left:'859px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1240px',
          top:'130px'
        }
      ))
    const Button2_4 = style ({  //右转按钮样式
        position:'absolute',
        top:'36px',
        left:'977px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1395px',
          top:'130px'
        }
      ))

    const Button2_5 = style ({ //预留2_5按钮样式
        position:'absolute',
        top:'36px',
        left:'1095px',
        width:'100px',
        height:'87px',
        borderRadius:'2px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        border:'solid 1px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'120px',
          height:'96px',
          left:'1550px',
          top:'130px'
        }
      ))

    const mapStyle = style ({ //地图栏样式
        position:'absolute',
        top:'143px',
        left:'623px',
        width:'570px',
        height:'360px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'770px',
          height:'750px',
          left:'935px',
          top:'127px'
        }
      ))

    const mapMenu = style ({  //地图栏导航栏
        position:'absolute',
        top:'0px',
        width:'570px',
        height:'28px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        borderRadius:'1px',
        borderBottom:'solid 1px #297fca',
        fontSize:'16px',
        fontFamily:'PingFangSC',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        color:this.props.theme == "false"?'#fff':'#000'
        // border:'solid 3px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'890px',
          height:'38px',
          fontSize:'22.4px',
          fontWeight:840
        }
      ))
    const mapMenu1 = style ({  //地图栏导航栏（for任务点列表）
        position:'absolute',
        top:'0px',
        width:'570px',
        height:'28px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        borderRadius:'1px',
        borderBottom:'solid 1px #297fca',
        fontSize:'16px',
        fontFamily:'PingFangSC',
        fontWeight:600,
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:1.63,
        letterSpacing:'0.6px',
        color:this.props.theme == "false"?'#fff':'#000'
        // border:'solid 3px #2298ff'
      },
      media({minWidth:1700},
        {
          width:'850px',
          height:'38px',
          fontSize:'25px',
          fontWeight:840
        }
      ))

    const mapBor = style ({ //地图栏展示内容样式
      position:"absolute",
      top:'58px',
      left:'12px',
      width:'546px',
      height:'293px',
      opacity:0.96,
      display:this.props.keysmap=="map"?'block':'none'
    },media({minWidth:1700},
      {
        top:'40.8px',
        left:'1px',
        width:'766.4px',
        height:'620.3px'
      }
    ))

    const camBor = style ({ //摄像栏展示内容样式
      position:"absolute",
      top:'58px',
      left:'12px',
      width:'546px',
      height:'293px',
      opacity:0.96,
      display:this.props.keysmap!="map"&&this.props.keysmap!="radar"?'block':'none'
    },media({minWidth:1700},
      {
        top:'40.8px',
        left:'1px',
        width:'766.4px',
        height:'356.3px'
      }
    ))

    const radarBor = style ({ //雷达展示内容样式
      position:"absolute",
      top:'58px',
      left:'12px',
      width:'546px',
      height:'293px',
      opacity:0.96,
      display:this.props.keysmap=="radar"?'block':'none'
    },media({minWidth:1700},
      {
        top:'40.8px',
        left:'1px',
        width:'766.4px',
        height:'356.3px'
      }
    ))

    const mapButton = style ({  //地图右上方添加航点按钮
      position:'absolute',
      top:'0px',
      left:'490px',
      width:'90px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'784.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'12px'
      }
    ))

    const translButton = style ({  //地图右上方优化航点按钮
      position:'absolute',
      top:'0px',
      left:'403px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'484.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const lineButton = style ({  //地图右上方优化直线航点按钮
      position:'absolute',
      top:'0px',
      left:'316px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'324.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const cleanButton = style ({  //地图右上方清除航线按钮
      position:'absolute',
      top:'0px',
      left:'223px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'404.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const camInitButton = style ({  //地图右上方初始化按钮
      position:'absolute',
      top:'0px',
      left:'296px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap != "map"&&this.props.keysmap != "radar"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'504.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))


    const camInButton = style ({  //地图右上方登录按钮
      position:'absolute',
      top:'0px',
      left:'393px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "photograph"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'594.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const camOutButton = style ({  //地图右上方登出按钮
      position:'absolute',
      top:'0px',
      left:'490px',
      width:'80px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "photograph"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'5px',
        left:'684.2px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const camGetButton = style ({  //地图右方预览按钮
      position:'absolute',
      top:'173px',
      left:'1200px',
      // width:'20px',
      height:'160px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      // borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap != "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'177px',
        left:'1707px',
        height:'178px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const camStopButton = style ({  //地图右方停止预览按钮
      position:'absolute',
      top:'345px',
      left:'1200px',
      // width:'20px',
      height:'160px',
      fontFamily:'PingFangSC',
      fontSize:'12px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'center',
      background:'#363636',
      // borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap != "map"?'block':'none',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      color:this.props.theme == "false"?'#fff':'#000'
    },media({minWidth:1700},
      {
        top:'443px',
        height:'178px',
        left:'1707px',
        // width:'78.8px',
        // height:'26.4px',
        fontSize:'14.2px'
      }
    ))

    const popstyle = (  //弹出框内容
      <div>
        经度:<input type = "text" ref={(lngs) => {this.lngs=lngs}} placeholder="请输入经度"/><br/>
        纬度:<input type = "text" ref={(lats) => {this.lats=lats}} placeholder="请输入纬度"/><br/>
        <a  onClick={()=>this.testInput()}>确认</a>
      </div>
    )

    const ipstyle = (  //弹出输入视频ip信息框内容
      <div>
        IP:<input type = "text" ref={(ips) => {this.ips=ips}} placeholder="请输入IP"/><br/>
        端口:<input type = "text" ref={(door) => {this.door=door}} placeholder="请输入纬度"/><br/>
        账号:<input type = "text" ref={(userName) => {this.userNames = userName}} placeholder = "请输入账号"/><br/>
        密码:<input type = "password" ref={(passwords) => {this.passwords = passwords}} placeholder = "请输入密码"/><br/>
        <a  onClick={()=>this.getCamInput()}>确认</a>
      </div>
    )

    const mapSwitch = style ({  //连线模式switch样式
      position:'absolute',
      top:'70px',
      left:'458px',
      width:'92px',
      height:'24px',
      fontFamily:'PingFangSC',
      fontSize:'13px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'normal',
      textAlign:'left',
      background:'#363636',
      borderRadius:'13.3px',
      border:'solid 1px #e3e3e3',
      zIndex:10,
      display:this.props.keysmap == "map"?'block':'none'
    },media({minWidth:1700},
      {
        top:'47px',
        left:'764.2px',
        // width:'128.8px',
        // height:'26.4px',
        fontSize:'18.2px'
      }
    ))

    const missionPoint = style ({ //任务点列表栏
        position:'absolute',
        top:'513px',
        left:'623px',
        width:'570px',
        height:'264px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff'
      },
      media({minWidth:1700},
        {
          width:'850px',
          height:'340px',
          //left:'935px',
          left:'36px',
          top:'654px',
          fontSize:'25px'
        }
      ))

    return (
      <div className = {wholeStyle} >

        {/* 车辆选择栏目 */}
        <div className={vehicle_select}>
          <div className={vehicle_selectTextStyle}>当前选择车辆 {this.props.vehicle_No}</div>
          {/* <div className={vehicle_current} >{this.props.vehicle_No}</div> */}
          <Button className={Button_vehicle_selectA} type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('A')))}>
            <div className={buttonText1}>A</div>
          </Button>
          <Button className={Button_vehicle_selectB }type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('B')))}>
            <div className={buttonText1}>B</div>
          </Button>
          <Button className={Button_vehicle_selectC}type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('C')))}>
            <div className={buttonText1}>C</div>
          </Button>
          <Button className={Button_vehicle_selectD}type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('D')))}>
            <div className={buttonText1}>D</div>
          </Button>
          <Button className={Button_vehicle_selectE} type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('E')))}>
            <div className={buttonText1}>E</div>
          </Button>
          <Button className={Button_vehicle_selectF }type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('F')))}>
            <div className={buttonText1}>F</div>
          </Button>
          <Button className={Button_vehicle_selectG}type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('G')))}>
            <div className={buttonText1}>G</div>
          </Button>
          <Button className={Button_vehicle_selectH}type = "primary" onClick = {(() => this.props.dispatch(changeVehicle_NoAction('H')))}>
            <div className={buttonText1}>H</div>
          </Button>

        </div>
        {/*任务栏 */}
        <div className = {status} >
          <div className = {missionStatus}>任务执行中</div>
          <div className = {lineBetween} />
          <div className = {timeTextStyle} >预计剩余时间 {this.getTime()}</div>
          {/* <div className = {timeStyle} >{this.getTime()}</div> */}
          <div className = {distanceTextStyle} >已出发 12km</div>
          {/* <div className = {distanceStyle} >12
            <span className = {distanceCompanyStyle} > km </span>
          </div> */}
          <div className = {missionPointStyle} > 任务点剩余 {this.getLessPoint()}/{this.getTotalPoint()}</div>
          {/* <div className = {leftPointStyle} > {this.getLessPoint()}
            <span className = {totalPointStyle} > /{this.getTotalPoint()}</span>
          </div> */}
        </div>
        {/*速度栏 最大速度3m/s*/}
        <div className = {speed}>
          <div className = {speedTextStyle} >速度 {isNaN(this.props.speed)?1.5:this.props.speed.toFixed(2)}m/s</div>
          {/* <div className = {speedLine} />
          <div className = {speedNumStyle} > {isNaN(this.props.speed)?1.5:this.props.speed.toFixed(2)}
            <span className = {speedComStyle} > m/s </span>
          </div> */}
          {/* <Slider className = {speedSliderStyle} defaultValue = {isNaN(this.props.speed)?30:(this.props.speed/3)*100} disabled = {false} /> */}
        </div>

        {/*瞬时加速度栏 */}
        <div className={accelerate}>
          <div className={accTextStyle}>加速度{this.props.vehicle_No=='A'?0:this.props.vehicle_No=='B'?10:this.props.vehicle_No=='C'?15:20}m/s²</div>
          {/* <div className={accLine}/>
          < div className={accNumStyle}>{this.props.vehicle_No=='A'?0:this.props.vehicle_No=='B'?10:this.props.vehicle_No=='C'?15:20}
            <span className={accComStyle}>m/s²</span>
          </div> */}
        </div>

        {/*瞬时功率栏 */}
        <div className={power}>
          <div className={powerTextStyle}>功率 12kw</div>
          {/* <div className={powerLine}/>
          < div className={powerNumStyle}>0.5
            <span className={powerComStyle}>kw</span>
          </div> */}
        </div>

        {/*当前位置栏 */}
        <div className={location}>
          <div className={locationTextStyle}>当前位置</div>
          <div className={locationNumStyle}>W{this.props.lng.toFixed(6)}<div>E{this.props.lat.toFixed(6)}</div>
          </div>
        </div>

        {/*运动角度栏 */}
        <div className = {sport} >
          <div className = {sportTextStyle} >北向角度 {isNaN(this.props.angle)?56:this.props.angle.toFixed(1)} </div>
          {/* <div className = {sportLine} /> */}
          {/* <div className = {sportNumStyle} > {isNaN(this.props.angle)?56:this.props.angle.toFixed(1)} </div> */}
          {/* <div className = {sportPieStyle} >
            <Angler  angle = {isNaN(this.props.angle)?56:this.props.angle}/>
          </div> */}
        </div>

        {/*存储空间栏 */}
        {/* <div className = {save} >
          <div className = {saveTextStyle} >存储空间</div>
          <div className = {saveStyle} > 25
            <span className = {saveComStyle} >%</span>
          </div>
          <div className = {saveSpaceStyle} >
            <SaveSpace />
          </div>
        </div> */}

        {/*通信质量栏 */}
        {/* <div className = {communicate} >
          <div className = {communicateTextStyle} >通信质量</div>
          <div className = {communicateStyle} > 75
            <span className = {communicateComStyle} > % </span>
          </div>
          <div className = {signFirst} />
          <div className = {signSecond} />
          <div className = {signThird} />
          <div className = {signForth} />
          <div className = {signFifth} />
        </div> */}

        {/* 电量栏 1块电池续航能力为1小时*/}
        <div className = {battery} >
          <div className = {batteryTextStyle} > 电量 </div>
          {/* <div className = {batteryStyle} > {this.getBatteryTime() == undefined?28:this.getBatteryTime()}
            <span className = {batteryComStyle} > 分钟 </span>
          </div> */}
          <div className={batteryStyle}> {this.state.batts.toFixed(1)}%</div>
          <div ><img className = {batteryPosStyle} src = {batt} />
            {this.renderBattery()}</div>
        </div>


        {/*采样管理栏 */}



        {/*照片栏*/}
        {/*<div className = {photo} >
          {/* <div className = {photoLine} /> */}
        {/* <Menu className = {photoMenu}  mode="horizontal" defaultSelectedKeys={['photo']} >
            <Menu.Item key="photo" >照片</Menu.Item>
            <Menu.Item key="record" >障碍物记录</Menu.Item>
          </Menu>
          </div>

        {/* 无人车启动按钮*/}
        {/*<Button className = {Button1_1} type = "primary" onClick = {(() => this.sendAction(0))} >
          <div className = {buttonIcon} ><Icon type="home" /></div>
          <div className = {buttonText} > 启动</div>
        </Button>
         无人车返航按钮
        <button className={Button1_2} type="primary">
          <div className={buttonIcon} ><Icon type="home" /></div>
          <div className = {buttonText} > 返航</div>
        </button>
         无人车停止按钮
        <button className={Button1_3} type="primary" onClick = {(() => Automode=false)}>
          <div className={buttonIcon} ><Icon type="home" /></div>
          <div className = {buttonText} > 停止</div>
        </button>
         更新路径按钮
        <Button className={Button1_4} type="primary" onClick={(() => this.sendAction(50))}>
          <div className={buttonIcon} ><Icon type="home" /></div>
          <div className = {buttonText} > 更新路径</div>
        </Button>*/}
        {/*预留2_1按钮*/}
        {/*<Button className = {Button1_1} type = "primary" onClick={(() => this.testDownload())} disabled={Automode?true:false} >
          <div className = {buttonIcon} ><Icon type="home" /></div>
          <div className = {buttonText} > BUT2_1</div>
        </Button>*/}

        {/*直行/停止按钮*/}
        <Button className = {Button1_0} type = "primary" onClick = {(() => this.sendAction(4))}>
          <div className = {buttonIcon} ><Icon type={pra == "停止"?'pause':'arrow-up'} /></div>
          <div className = {buttonText} >{pra1}</div>  {/*par1=直行*/}
        </Button>

        {/*后退按钮*/}
        <Button className = {Button1_1} type = "primary" onClick = {(() => this.sendAction(8))}>
          <div className = {buttonIcon} ><Icon type="arrow-down" /></div>
          <div className = {buttonText} >后退</div>  {/*par1=直行*/}
        </Button>

        {/*左转按钮*/}
        <Button className = {Button1_2} type = "primary" onClick = {(() => this.sendAction(20))}>
          <div className = {buttonIcon} ><Icon type="arrow-left" /></div>
          <div className = {buttonText} >左转</div>
        </Button>

        {/*右转按钮*/}
        <Button className = {Button1_3} type = "primary" onClick = {(() => this.sendAction(15))}>
          <div className = {buttonIcon} ><Icon type="arrow-right" /></div>
          <div className = {buttonText} >右转</div>
        </Button>

        {/*预留2_5按钮停的按钮*/}
        <Button className = {Button1_4} type = "primary" onClick = {(() => this.sendAction(10))}>
          <div className = {buttonIcon} ><Icon type='pause' /></div>
          <div className = {buttonText} >停</div>
        </Button>

        {/* 绕开按钮 */}
        <Button className = {Button1_5} type = "primary" onClick = {(() => this.props.dispatch(avoidPointAction()))}>
          <div className = {buttonIcon} ><Icon type='rollback' /></div>
          <div className = {buttonText} >绕开</div>
        </Button>

        {/*地图栏*/}
        <div className = {mapStyle} >
          <Menu className = {mapMenu}  mode="horizontal"  defaultSelectedKeys={['map']} onClick = {(e)=>this.testClick(e)} selectedKeys={[this.props.keysmap]}>
            <Menu.Item key="map" >地图视图</Menu.Item>
            <Menu.Item key="photograph" >摄像头视图</Menu.Item>
            {/* <Menu.Item key="radar" >雷达视图</Menu.Item> */}
            {/*{this.addMenu()}*/}
          </Menu>
          <Popover content = {popstyle} title="手动输入经纬度">
            <Button className = {mapButton} type = "primary" >添加路径点</Button>
          </Popover>
          {/* <Button className = {translButton} type = "primary" onClick = {(() => this.chanslationPoint())}>优化路线</Button> */}
          {/* <Button className = {cleanButton} type = "primary" onClick = {(() => this.cleanPoint())}>重置路线</Button> */}
          <Switch className = {mapSwitch} checkedChildren="连线模式" unCheckedChildren="平扫模式" checked={this.props.mode} onChange={(checked:boolean)=>{this.props.dispatch(toggleModeAction())}} />
          <div className = {mapBor}>
            <Map style={{height:'133%',
              width:'116%'}}
              //初始化地图中心点
                 center={this.props.workingPoint[0]?this.props.workingPoint[0]:(this.props.lng?{lng: this.props.lng, lat: this.props.lat}:{lng:121.4494651085, lat: 31.0313365968})}
                 //地图缩放等级
                 zoom={19} enableScrollWheelZoom
                 mapType={BMAP_HYBRID_MAP}
                 //地图点击事件
                 events={{click:(e)=>{this.handleMapClick(e)}}}>
                   {/* 渲染无人船位置 */}
              {this.renderBoat()}
              {/* 渲染工作路径点 */}
              {this.renderFlag()}
              {/* 渲染警告模式下的船体黄色圆圈 */}
              {this.renderYellowCircle()}
               {/* 渲染平扫模式下的四个点 */}
              {this.renderMarker()}
              {/* <MapvLayer data={this.getdata()} options={{
                size:13,
                gradient:{0.25: "rgb(0,0,255)", 0.55: "rgb(0,255,0)", 0.85: "yellow", 1.0: "rgb(255,0,0)"}, //0-3浅黄 ， 4-7深黄 ，8-12橘黄
                max:100,
                draw:'heatmap'
              }} /> */}
              {/* 鼠标右键点击事件 */}
              <ContextMenu dispatch={this.props.dispatch} />
            </Map>
          </div>
          <div className = {camBor} id = "divPlugin">camera field </div>
          <div className = {radarBor} id = "divPlugin1">Radar</div>
          <Button className = {camInitButton} type = "primary" onClick = {(() => this.testcamera())}>初始化</Button>
          <Button className = {camInButton} type = "primary" onClick = {(() => this.checkIncamera())}>登录</Button>
          <Button className = {camOutButton} type = "primary" onClick = {(() => this.getCamera())}>预览</Button>

        </div>

        <Button className = {camGetButton} type = "primary" onClick = {(() => this.catchCam())}><Icon type="camera"/><br/>拍<br/>照</Button>
        <Button className = {camStopButton} type = "primary" onClick = {(() => this.recordCam())}><Icon type="video-camera"/><br/>录<br/>像</Button>




        {/* {this.testcamera()} */}
        {/*任务点列表栏*/}
        <div className = {missionPoint} >
          <Menu className = {mapMenu1}  mode="horizontal" >
            <Menu.Item key="point" >任务点列表</Menu.Item>
          </Menu>
          <MissionTabler   workingPoint={this.props.workingPoint} lng={this.props.lng}
          lat={this.props.lat} theme={this.props.theme}/>
        </div>

      </div>
    )

  }



  //清除航线
  cleanPoint(){
    if(this.props.workingPoint.length == undefined || this.props.workingPoint.length < 2)return

    var resultPoint = []
    resultPoint.push(this.props.workingPoint[0])
    this.props.dispatch(updateAllPointAction(resultPoint))
  }

  //取中点让路径点更密集
  focuPoint(){
    var resultPoints = [];  //记录转换后的结果集
    if(this.props.workingPoint.length == undefined || this.props.workingPoint.length < 2){
      return ;
    }
    if(this.props.workingPoint.length > 12){
      return ;
    }
    for(var i = 0;i + 1 < this.props.workingPoint.length;i++){  //取两点之间的中间
      var middlePoint = this.codingK(this.props.workingPoint[i].lng , this.props.workingPoint[i].lat,
        this.props.workingPoint[i+1].lng , this.props.workingPoint[i+1].lat);

      resultPoints.push(this.props.workingPoint[i])
      resultPoints.push(middlePoint)
    }
    resultPoints[1].status = "doing"
    resultPoints[2].status = "todo"
    resultPoints.push(this.props.workingPoint[this.props.workingPoint.length-1])
    this.props.dispatch(updateAllPointAction(resultPoints))
  }

  //计算夹角
  chanslationPoint(){
    var resultPoints = []; //记录转换后的结果集
    if(this.props.workingPoint.length == undefined || this.props.workingPoint.length < 3){
      return ;
    }
    // let map = new BMap.Map("map")
    // let PointNow = new BMap.Point(nowLng , nowLat)  //目前经纬度
    // let PointTar = new BMap.Point(targetLng , targetLat)  //目标经纬度
    // var distance = map.getDistance(PointNow , PointTar) //初始化距离
    let map = new BMap.Map("map")
    // resultPoints.push(this.props.workingPoint[0]);
    var tofs = 'false'
    for(var i = 0;i + 2 < this.props.workingPoint.length;i++){  //通过判定三点构成的三角形的余弦是否小于0可知是否为钝角
      var pointA = new BMap.Point(this.props.workingPoint[i].lng , this.props.workingPoint[i].lat)
      var pointB = new BMap.Point(this.props.workingPoint[i + 1].lng , this.props.workingPoint[i + 1].lat)
      var pointC = new BMap.Point(this.props.workingPoint[i + 2].lng , this.props.workingPoint[i + 2].lat)
      var lineAB = map.getDistance(pointA , pointB)
      var lineBC = map.getDistance(pointB , pointC)
      var lineAC = map.getDistance(pointA , pointC)
      var cosB = (lineAB * lineAB + lineBC * lineBC - lineAC * lineAC) / (2 * lineAB * lineBC)
      // console.log('余弦值:' + cosB)
      if(cosB < 0){
        if(tofs != 'true'){
          resultPoints.push(this.props.workingPoint[i])
        }
        resultPoints.push(this.props.workingPoint[i+1])
        if(i + 2 == this.props.workingPoint.length - 1) { //如果刚好到了最后一个点
          resultPoints.push(this.props.workingPoint[i + 2])
        }
        tofs = 'false'
      } else {
        var pointB1 = this.codingK(this.props.workingPoint[i].lng , this.props.workingPoint[i].lat,
          this.props.workingPoint[i + 1].lng , this.props.workingPoint[i + 1].lat)
        var pointB2 = this.codingK(this.props.workingPoint[i + 1].lng , this.props.workingPoint[i + 1].lat,
          this.props.workingPoint[i + 2].lng , this.props.workingPoint[i + 2].lat)
        if(tofs != 'true'){
          resultPoints.push(this.props.workingPoint[i])
          resultPoints.push(pointB1)
        }

        resultPoints.push(pointB2)
        if(i + 2 == this.props.workingPoint.length - 1){
          resultPoints.push(this.props.workingPoint[i + 2])
        }
        tofs = 'true'
      }
    }
    resultPoints[1].status = "doing"
    for(var i = 0;i < resultPoints.length - 1;i++){
      if(resultPoints[i].lng == resultPoints[i + 1].lng){
        resultPoints.splice(i,1)
      }
    }
    this.props.dispatch(updateAllPointAction(resultPoints))
    // console.log('结果集:' + resultPoints)
    // var translPoints = this.changeLocationArray(resultPoints)
    // for(var i = 0 ;i < resultPoints.length;i++){
    //   console.log('结果集:resultPoint[' + i + '],lng:' + translPoints[i].lng + ',lat:' + translPoints[i].lat)
    // }

    // for(var i = 0 ;i < resultPoints.length;i++){
    //   console.log('结果集:resultPoint[' + i + '],lng:' + resultPoints[i].lng + ',lat:' + resultPoints[i].lat)
    // }

    // for(var i = 0 ;i < resultPoints.length;i++){
    //   console.log('删除后的结果集:resultPoint[' + i + '],lng:' + resultPoints[i].lng + ',lat:' + resultPoints[i].lat)
    // }
  }

  //计算直线斜率
  codingK(lng1:number, lat1:number , lng2:number, lat2:number){
    var k;
    var b;
    var results;
    var resultlng;
    var resultlat;
    if(lng2 != lng1){
      k = (lat2 - lat1) / (lng2 - lng1)
      b = lat1 - k * lng1
      resultlng = (lng1 + lng2)  * 0.5
      resultlat = k * resultlng + b
    } else {
      resultlng = lng1;
      resultlat = lat1
    }


    return {lng:resultlng , lat:resultlat, status:"todo"}
  }

  //百度坐标转化为GPS坐标(官方没有提供相应转换方式，采用github上的coordtransform库实现)
  changeLocationArray(points: {lng: number, lat: number}[]) {
    return points.map((point) => {
      const gcj02Coord = coordtransform.bd09togcj02(point.lng, point.lat); //百度经纬度坐标转国测局坐标
      const wgs84Coord = coordtransform.gcj02towgs84(gcj02Coord[0], gcj02Coord[1]); //国测局坐标转wgs84坐标
      return {lng: wgs84Coord[0], lat: wgs84Coord[1]}
    })
  }

  //出发报警
  getOneOneOne(){
    if(this.props.alertMode)hantime += 1  //如果是警告状态下每秒更新一次页面
    else hantime += 3 //如果是平常状态下3秒刷新一次页面
    var nowLng = this.props.lng;  //目前精度
    var nowLat = this.props.lat;  //目前纬度
    var nowPoint = new BMap.Point(nowLng,nowLat)

    if(this.props.workingPoint.length == 0 || this.props.workingPoint.length == 1 || this.props.workingPoint.length == 2)return 0;
    // console.log(this.props.workingPoint)
    var firstLng = this.props.workingPoint[tnum].lng;
    var firstLat = this.props.workingPoint[tnum].lat;
    var secondLng = this.props.workingPoint[tnum + 1].lng;
    var secondLat = this.props.workingPoint[tnum + 1].lat;
    // var middleLng = (firstLng + secondLng) / 2.0
    // var middleLat = (firstLat + secondLat) / 2.0
    // var middlePoint = new BMap.Point(middleLng , middleLat)
    // var circle = new BMap.circle(middlePoint , 30);
    // let map = new BMap.Map("map")
    const polyline = new BMap.Polyline([
      new BMap.Point(firstLng, firstLat),
      new BMap.Point(secondLng, secondLat),
    ], {strokeColor:'blue', strokeWeight:2, strokeOpacity:0.5, strokeStyle:'dashed'})
    if(hantime % 60 == 0){  //每隔一分钟判定一次是否在规划航线上
      // console.log(this.isPointOnPolyline(nowPoint , polyline))
      if(this.isPointOnPolyline(nowPoint , polyline) == false && this.props.alertMode == false){
        var status = true
        // console.log("test")
        // this.props.dispatch(updateAlertStationAction(status))
      } else{
        // console.log("test")
      }
    }

    // if(map.isPointInCircle(nowPoint , circle) == false){
    //   var status = false;
    //   this.props.dispatch(updateAlertStationAction(status))
    // }

  }

  /**
   * 判断点是否在折线上
   * @param {Point} point 点对象
   * @param {Polyline} polyline 折线对象
   * @returns {Boolean} 点在折线上返回true,否则返回false
   */
  isPointOnPolyline(point, polyline){

    //todo首先判断点是否在线的外包矩形内，如果在，则进一步判断，否则返回false

    //判断点是否在线段上，设点为Q，线段为P1P2 ，
    //判断点Q在该线段上的依据是：( Q - P1 ) × ( P2 - P1 ) = 0，且 Q 在以 P1，P2为对角顶点的矩形内
    var pts = polyline;
    for(var i = 0; i < pts.length - 2; i++){

      var curPt = {
        lng: pts[i],
        lat: pts[i + 1]
      }

      var nextPt = {
        lng: pts[i + 2],
        lat: pts[i + 3]
      }

      //首先判断point是否在curPt和nextPt之间，即：此判断该点是否在该线段的外包矩形内
      if (point.lng >= Math.min(curPt.lng, nextPt.lng) && point.lng <= Math.max(curPt.lng, nextPt.lng) &&
        point.lat >= Math.min(curPt.lat, nextPt.lat) && point.lat <= Math.max(curPt.lat, nextPt.lat)){
        //判断点是否在直线上公式
        var precision = (curPt.lng - point.lng) * (nextPt.lat - point.lat) -
          (nextPt.lng - point.lng) * (curPt.lat - point.lat);
        var diff = 5;
        diff = 5;
        if(precision < diff && precision > -diff){//实质判断是否接近30
          return true;
        }
      }
    }

    return false;
  }

  //更改地图栏目显示内容
  testClick(e){
    // console.log(e)
    console.log(e.key)
    // console.log(this.props.keysmap)
    if(e.key != this.props.keysmap){
      this.props.dispatch(updateKeys(e.key))
    }
  }

  //增加menu选项
  addMenu(){
    if(this.props.keysmap != 'map'){
      return <Menu.Item key = "camera">摄像头视图2</Menu.Item>
    }
  }

  //初始化摄像头环境
  testcamera(){
    // 检查插件是否已经安装过
    // console.log('test')
    // console.log(WebVideoCtrl)
    var iRet = WebVideoCtrl.I_CheckPluginInstall();
    if (-2 == iRet) {
      // alert("您的Chrome浏览器版本过高，不支持NPAPI插件！");
      message.info('您的Chrome浏览器版本过高,不支持NPAPI插件!',3)
      return;
    } else if (-1 == iRet) {
      // alert("您还未安装过插件，双击开发包目录里的WebComponentsKit.exe安装！");
      message.info('您还未安装过插件,双击开发包目录里的WebComponentsKit.exe安装！',3)
      return;
    }
    // console.log('test')
    // 初始化插件参数及插入插件
    WebVideoCtrl.I_InitPlugin('100%', '100%', {
      bWndFull: true,//是否支持单窗口双击全屏，默认支持 true:支持 false:不支持
      iWndowType: 1,
      cbSelWnd: function (xmlDoc){
        // console.log('pass')
        var g_iWindIndex = WebVideoCtrl.$(xmlDoc).find("SelectWnd").eq(0).text();
        // console.log(g_iWindIndex)
      }
    });
    WebVideoCtrl.I_InsertOBJECTPlugin("divPlugin");
    // console.log('iRet:'+iRet)
    // 检查插件是否最新
    if (-1 == WebVideoCtrl.I_CheckPluginVersion()) {
      // alert("检测到新的插件版本，双击开发包目录里的WebComponentsKit.exe升级！");
      message.info('检查到新的插件版本,双击开发包目录里的WebComponentsKit.exe升级！',3)
      return;
    }
  }

  //摄像头登录
  checkIncamera(){
    var szIP = this.props.keysmap=="camera"?'192.168.1.188':'192.168.1.188'
    var szPort = this.props.keysmap=="camera"?'88':'88'
    var szUsername = 'admin'
    var szPassword = 'bike2015'
    var self = this
    var iRet = WebVideoCtrl.I_Login(szIP , 1 , szPort , szUsername , szPassword , {
      success:function(xmlDoc){
        // console.log('登录成功')
        message.success('登录成功!',3)
        self.getChannel()
      },
      error:function(){
        // console.log('登录失败')
        message.error('登录失败!',3)
      }
    })

    if(-1 == iRet){
      // console.log('已经登录过')
      this.checkoutCamera()
    }
  }

  //退出摄像头登录
  checkoutCamera(){
    var szIP = this.props.keysmap=="camera"?'192.168.1.188':'192.168.1.108'

    var iRet = WebVideoCtrl.I_Logout(szIP)
    if(0 == iRet){
      // console.log('退出成功')
      message.success('退出成功!',3)
    } else {
      // console.log('退出失败')
      message.error('退出失败!',3)
    }

  }

  //获取通道
  getChannel(){
    var szIP = this.props.keysmap=="camera"?'192.168.1.188':'192.168.1.108'

    WebVideoCtrl.I_GetAnalogChannelInfo(szIP,{
      async:false,
      success:function(xmlDoc){
        // console.log('获取模拟通道成功')
        message.success('获取模拟通道成功!',3)
      },
      error:function(){
        // console.log('获取模拟通道失败')
        message.error('获取模拟通道失败!',3)
      }
    })
  }

  //预览摄像头
  getCamera(){
    var szIP = this.props.keysmap=="camera"?'192.168.1.188':'192.168.1.108'
    var oWndInfo = WebVideoCtrl.I_GetWindowStatus()
    // console.log(oWndInfo.length)
    if(oWndInfo.length != 0){ //说明已经在预览了
      this.stopCamera()
    } else {
      var iRet = WebVideoCtrl.I_StartRealPlay(szIP)

      if(0 == iRet){
        // console.log('开始预览成功')
        message.success('开始预览成功!',3)
      } else {
        // console.log('开始预览失败')
        message.error('开始预览失败!',3)
      }
    }

  }

  //停止预览
  stopCamera(){
    var oWndInfo = WebVideoCtrl.I_GetWindowStatus()
    if(oWndInfo != null){
      var iRet = WebVideoCtrl.I_Stop()
      if(0 == iRet){
        // console.log('停止预览成功')
        message.success('停止预览成功!',3)
      } else {
        // console.log('停止预览失败')
        message.error('停止预览失败!',3)
      }
    }
  }

  //抓图
  catchCam(){
    var oWndInfo = WebVideoCtrl.I_GetWindowStatus()
    if(oWndInfo.length != 0){ //说明在预览中
      var szPicName = 'joy_'+new Date().getTime()
      var iRet = WebVideoCtrl.I_CapturePic(szPicName)
      if(0 == iRet){
        // console.log('抓图成功!')
        message.success('拍照成功!',3)
        var xmlDoc = WebVideoCtrl.I_GetLocalCfg();
        alert('图片保存在:' + WebVideoCtrl.$(xmlDoc).find("CapturePath").eq(0).text())
      } else {
        // console.log('抓图失败!')
        message.error('拍照失败!',3)
      }
    }
  }

  //录像
  recordCam(){
    if(this.props.recordCam == 'false'){  //开始录像
      var oWndInfo = WebVideoCtrl.I_GetWindowStatus()
      if(oWndInfo.length != 0){
        var szFileName = 'joy_' + new Date().getTime()
        var iRet = WebVideoCtrl.I_StartRecord(szFileName)
        if(0 == iRet){
          // console.log('开始录像成功')
          message.success('正在录像中!',3)
          this.props.dispatch(updateRecordAction('true'))
        } else {
          // console.log('开始录像失败')
          message.error('开启录像失败!',3)
        }
      }
    } else {
      var oWndInfo = WebVideoCtrl.I_GetWindowStatus
      if(oWndInfo.length != 0){
        var iRet = WebVideoCtrl.I_StopRecord()
        if(0 == iRet){
          // console.log('停止录像成功')
          message.success('停止录像成功!',3)
          this.props.dispatch(updateRecordAction('false'))
          var xmlDoc = WebVideoCtrl.I_GetLocalCfg()
          // console.log(xmlDoc)
          alert('录像保存在:' + WebVideoCtrl.$(xmlDoc).find("RecordPath").eq(0).text())
        } else {
          // console.log('停止录像失败')
          message.error('停止录像失败!',3)
        }
      }
    }
  }

  //测试输入框
  testInput(){
    // console.log(this.lngs.value)
    // console.log(this.lats.value)
    var lng = this.lngs.value
    var lat = this.lats.value
    if(lng == "" || lng == undefined){
      alert("请输入经度")
      return ;
    }
    if(lat == "" || lat == undefined){
      alert("请输入纬度")
      return ;
    }
    var point = {
      'lng':lng,
      'lat':lat
    }

    // this.props.dispatch(updateTranAction(point))
    message.success("添加航点成功!",3)
    this.lngs.value = ""
    this.lats.value = ""

  }

  //输入摄像头登录信息
  getCamInput(){
    var ip = this.ips.value
    var door = this.door.value
    var userName = this.userNames.value
    var password = this.passwords.value
    if(ip == "" || ip == undefined){
      ip = "192.168.1.108"
    }
    if(door == "" || door == undefined){
      door = "86"
    }
    if(userName == "" || userName == undefined){
      userName = "admin"
    }
    if(password == "" || password == undefined){
      password = "camera2018"
    }
    // console.log("test:" + ip + ': ' + door)
    var self = this
    var iRet = WebVideoCtrl.I_Login(ip , 1 , door , userName  , password , {
      success:function(xmlDoc){
        // console.log('登录成功')
        message.success('登录成功!',3)
        // self.getChannel()
      },
      error:function(){
        // console.log('登录失败')
        message.error('登录失败!',3)
      }
    })

    if(-1 == iRet){
      // console.log('已经登录过')
      this.checkoutCamera()
    }


  }

  //出发采样动作
  // startBlinking(num: number) {
  //   if(this.props.plans == 'true'){
  //     message.info("正在读取传感器数据，请稍后再采样...",3)
  //     return ;
  //   }
  //   if(this.props.upda_road == 'true'){
  //     message.info("正在更新路径中，请稍后再采样...",3)
  //     return ;
  //   }
  //   if(this.props.mana == 'true'){
  //     message.info("上次采样动作还未完成，请稍后再操作...",3)
  //     return ;
  //   }
  //   this.props.dispatch(requestRelayChangeAction(num, this.props.relayState))
  //   if(this.props.relayState[num - 1] == false){  //记录采样时的经纬度和时间
  //     this.manaInfo[num-1].lat = this.props.lat
  //     this.manaInfo[num-1].lng = this.props.lng
  //     this.manaInfo[num-1].time = new Date().toLocaleTimeString();
  //   }
  // }


  //获取剩余的任务点
  getLessPoint(){
    var num = this.props.workingPoint.length
    var k=0;
    if(isNaN(num)){
      return 10;
    } else {
      if(this.props.workingPoint[0] == undefined)return;
      for(var i = 0 ;i < num;i++){

        if(this.props.workingPoint[i].status == "done")
          num--;
        else if(this.props.workingPoint[i].status == "doing")
          break;
      }
      return num;
    }
  }

  //获取总的任务点
  getTotalPoint(){
    var num = this.props.workingPoint.length
    if(isNaN(num)){
      return 20;
    }else {
      return num
    }
  }

  testDownload(){
    try{
      var elemIF = document.createElement("iframe");
      elemIF.src = "localhost:8484/ocean/download";
      elemIF.style.display = "none";
      document.body.appendChild(elemIF);
    }catch(e){

    }
  }
  //发送指令
  sendAction(param){
    switch(param){
      case 0:{
        pra = "启动"
        this.props.dispatch(sendStartAction(0))
        break;
      }
      case 4:{
        pra = "开始";
        this.props.dispatch(sendStartAction(4))
        break;
      }
      case 10:{
        pra = "停止";
        this.props.dispatch(sendStartAction(10));
        break;
      }
      case 11:{ //返航

        var backworkingPoint = []
        if(this.props.workingPoint.length < 2)break;
        if(this.props.mana == 'true'){
          message.info("当前正在进行采样操作，请稍后再试...",3)
          return ;
        }
        if(this.props.upda_road == 'true'){
          message.info("更新路径操作正在进行中，请稍后再试...",3)
          return ;
        }
        if(this.props.plans == 'true'){
          message.info("当前正在读取传感器数据，请稍后再试...",3)
          return ;
        }
        pra = "停止"
        var localPoint = {lng:this.props.lng , lat:this.props.lat, status:"doing"}
        backworkingPoint.push(localPoint)  //当前点作为起点
        backworkingPoint.push(this.props.workingPoint[0]) //起始点作为终点
        // this.props.dispatch(sendBackRoutesAction(backworkingPoint))
        // this.props.dispatch(sendStartAction(11));
        break;
      }
      case 15:{
        pra = "停止"
        this.props.dispatch(sendStartAction(15));
        break;
      }
      case 8:{
        pra = "停止"
        this.props.dispatch(sendStartAction(8));
        break;
      }
      case 20:{
        pra = "停止"
        this.props.dispatch(sendStartAction(20));
        break;
      }
      case 50:{
        if(this.props.mana == 'true'){
          message.info("当前正在进行采样操作，请稍后再试...",3)
          return ;
        }
        if(this.props.upda_road == 'true'){
          message.info("上次更新路径操作还在进行中，请稍后再试...",3)
          return ;
        }
        if(this.props.plans == 'true'){
          message.info("当前正在读取传感器数据，请稍后再试...",3)
          return ;
        }
        this.props.dispatch(sendStartAction(50))
        message.success("正在更新路径信息", 3)
        break;
      }
    }
  }

  //初始化任务执行时间s
  getTime(){
    var speed = this.props.speed  //初始化速度
    // var distance = this.props.totalDistance[tnum] // 初始化距离
    var time; //目标时间
    var nowLng = this.props.lng //目前经度
    var nowLat = this.props.lat //目前纬度
    var targetLng ; //下一个目标的经度
    var targetLat ;  //下一个目标的纬度
    // console.log('test')
    // console.log(this.props.workingPoint)
    if(this.props.workingPoint.length == 0 || this.props.workingPoint.length == 1)return 0;
    if(tnum > this.props.totalDistance.length)return 0;
    // var gpsPosition = this.changeLocationArray(this.props.workingPoint)
    if(this.props.workingPoint[0].status == "doing" && this.props.workingPoint.length > 1){
      this.props.workingPoint[0].status = "done"
      this.props.workingPoint[1].status = "doing"
    }
    if(tnum == this.props.totalDistance.length){  //说明到了最后一个目标点
      // console.log('test')
      targetLng = this.props.workingPoint[tnum].lng
      targetLat = this.props.workingPoint[tnum].lat
    } else {
      // console.log('test2')
      // console.log("距离数组长度:" + this.props.totalDistance.length)
      // console.log("经纬度数组数据:" + this.props.workingPoint)
      targetLng = this.props.workingPoint[tnum].lng==undefined?0:this.props.workingPoint[tnum].lng
      targetLat = this.props.workingPoint[tnum].lat==undefined?0:this.props.workingPoint[tnum].lat
    }
    let map = new BMap.Map("map")
    let PointNow = new BMap.Point(nowLng , nowLat)  //目前经纬度
    let PointTar = new BMap.Point(targetLng , targetLat)  //目标经纬度
    var distance = map.getDistance(PointNow , PointTar) //初始化距离
    // var distance2 = this.GetDistance(PointNow.lat , PointNow.lng , PointTar.lat , PointTar.lng)
    // console.log("距离:" + distance)
    if(distance <= 1){  //如果当前经纬度与目标经纬度距离小于1米，则认为已经通过
      // console.log(this.props.workingPoint)
      if(this.props.workingPoint[tnum] != undefined)
        this.props.workingPoint[tnum].status = "done" //修改过去的点为已完成状态
      // console.log(this.props.workingPoint[tnum].status)
      if(this.props.workingPoint[tnum+1] != undefined)
        this.props.workingPoint[tnum+1].status = "doing" //修改下一个点为正在进行
      tnum++;
    }
    if(speed != 0.00)time = distance / speed;  //目前单位为秒
    else time = 0
    // console.log("时间:" + time)
    return this.format(time);
  }

  /**
   * 将输入的秒数
   * @return {string} 转换后的01:23
   */
  format(ms) {
    var fmt;
    var minute = parseInt(ms, 10);
    var second = 0;

    if (minute <= 60) {
      fmt = minute < 10 ? `0${minute}` : minute;
    } else {
      second = Math.floor(minute / 60);
      // second = second < 10 ? `0${second}` : second;
      minute = Math.floor(minute % 60);
      // minute = minute < 10 ? `0${minute}` : minute;
      fmt = `${second}:${minute}`;
    }
    return fmt;
  }

  //获取电量
  getBatteryInfor(){
    var voltage = (this.props.voltage1 <= this.props.voltage2)?this.props.voltage1:this.props.voltage2
    var result;
    if(voltage > 24){
      result = 90 + (voltage - 24) * 10 / 1.2
    } else if(voltage <= 24 && voltage > 19.8){
      result = 20 + (voltage - 19.8) * 70 / 4.2
    } else {
      result = (voltage - 18) * 20 / 1.8
    }
    return result.toFixed(0)
  }

  //获取续航时间()
  // getBatteryTime(){
  //   var batts = this.getBatteryInfor();
  //   if(isNaN(batts))return 60
  //   var times = 60 * (batts / 100)
  //   return times.toFixed(0)
  // }

  //渲染无人船位置
  renderBoat() {
    if (this.props.lng && this.props.lat) {
      const myIcon = new BMap.Icon(boat, new BMap.Size(100, 100))
      myIcon.setImageSize(new BMap.Size(100, 100))
      // return <Marker position={{lng:this.props.lng, lat:this.props.lat}} icon={myIcon} offset={new BMap.Size(0,5)} enableMassClear={false}
      //                 events={{dblclick:(e)=>{this.props.dispatch(sendReturnAction())}}} />
      return <Marker position={{lng:this.props.lng, lat:this.props.lat}} icon={myIcon} offset={new BMap.Size(0,5)} enableMassClear={false}
      />
    } else {
      // const myIcon = new BMap.Icon(boat, new BMap.Size(100, 100))
      // myIcon.setImageSize(new BMap.Size(100, 100))
      // return <Marker position={{lng:121.4494651085, lat: 31.0313365968}} icon={myIcon} offset={new BMap.Size(0, 5)}
      // enableMassClear={false}/>
      return null
    }
  }

  // 渲染警告模式下的船体黄色圆圈
  renderYellowCircle() {
    const myIcon = new BMap.Icon(circle, new BMap.Size(32, 32))
    myIcon.setImageSize(new BMap.Size(32, 32))
    return this.props.alertTime % 2?<Marker position={{lng:this.props.lng, lat:this.props.lat}} icon={myIcon} offset={new BMap.Size(0,10)} enableMassClear={false} />:null
  }

  //渲染工作路径点
  renderFlag() {
    const workingPoint = this.props.workingPoint
    if (!workingPoint || workingPoint.length == 0) {
      return null
    }
    return <WorkingPoint workingPoint={this.props.workingPoint} dispatch={this.props.dispatch}/>
  }

  //渲染平扫模式下的四个点
  renderMarker() {
    const boundPoint = this.props.boundPoint
    if (!boundPoint || boundPoint.length == 0) {
      return null
    }
    return this.props.boundPoint.map((point)=>{
      return <Marker position={{lng:point.lng, lat:point.lat}} />
    })
  }

  //响应点击地图添加平扫关键路径点事件
  handleMapClick(e) {
    if (this.props.mode) { //连线模式下不显示处理点击事件
      return
    }
    if (this.props.boundPoint.length >= 4) { //最多添加四个平扫关键点
      return
    }
    this.props.dispatch(addBoundPointAction(e.point))
    switch (this.props.boundPoint.length) {
      case 1: message.success("标定第一个点成功，请添加第二个点", 3)
        break
      case 2: message.success("标定第二个点成功，请添加第三个点", 3)
        break
      case 3: message.success("标定第三个点成功，请添加第四个点", 3)
        break
      case 4: message.success("标定第四个点成功，开始计算平扫路径", 3)
        this.props.dispatch(calculateRouteAction())
        break
      default: return
    }
  }

  //初始化热力图参数
  //  getdata(){
  //   var randomCount=300;
  //   data = [];
  //   // console.log(data);
  //   // 构造数据 lng: 121.445967, lat:31.032097
  //   while (randomCount--) {
  //       // var cityCenter = Map.utilCityCenter.getCenterByCityName(citys[(Math.random() * citys.length)]);
  //       data.push({
  //           geometry: {
  //               type: 'Point',
  //               coordinates: [this.props.lng - 2 + Math.random() * 4, this.props.lat - 2 + Math.random() * 4 ]
  //           },
  //           count: 30 * Math.random()
  //       });
  //   }
  //   return data;
  // }

  renderBattery(){  //根据电量渲染电池组件

    const batteryColor = style ({ //根据电量判定电池图标着色样式
      position:'absolute',
      top:'22.5px',
      left:'119.5px',
      width:'29px',
      height:'58.6px',
      backgroundColor:'#b61313',
      borderRadius:'3px',
      borderTop:'solid 3.1px #000000'
    },media({minWidth:1700},
      {
        top:'26px',
        left:'281px',
        width:'41px',
        height:'61.6px'
      }
    ))

    var batte = this.state.batts.toFixed(1)
    if(isNaN(batte))batte = 100
    var colors;
    var heights = window.innerWidth > 1700?61.6:58.6
    var tops = window.innerWidth > 1700?26:22.5

    if(batte >= 80 && batte <=100){
      colors = 'green'
    }else if(batte >=50 && batte <80){
      colors = 'yellow'
    }else {
      colors = '#b61313'
    }

    var distance = (100 - batte) / 100  //获取应该缩小的倍率
    heights = heights - heights * distance
    tops = tops + tops * distance * 2.5

    return <div className = {batteryColor} style = {{backgroundColor:colors,height:heights,top:tops}} />

  }

  // renderBottle(num){
  //   const getLeft = () =>{
  //     switch(num){
  //       case 1 : return window.innerWidth > 1700?'77.8px':'54px';
  //       case 2 : return window.innerWidth > 1700?'209px':'147px';
  //       case 3 : return window.innerWidth > 1700?'340px':'240px';
  //       case 4 : return window.innerWidth > 1700?'468px':'332px';
  //       case 5 : return window.innerWidth > 1700?'599px':'425px';
  //       case 6 : return window.innerWidth > 1700?'729px':'518px';
  //     }
  //   }

  //   const bottleFull = style ({  //瓶子填充样式
  //     position:'absolute',
  //     top:'86px',
  //     left:'38px',
  //     width:'18px',
  //     height:'29px',
  //     backgroundColor:'#2298ff'
  //   },media({minWidth:1700},
  //     {
  //       top:'103px',
  //       left:'77.8px',
  //       width:'28px',
  //       height:'47px'
  //     }
  //   ))

  //   if(this.props.relayStatus[num-1] == 1){ //说明采样动作已经成功，启动填充模块
  //     var heights = window.innerWidth>1700?1:1;
  //     var tops=window.innerWidth>1700?151:0;
  //     if(this.tm[num - 1] < 30){
  //       var time = this.tm[num - 1];
  //       var diss = time / 30
  //       heights = 47 * diss
  //       tops = 151 - 48 * diss
  //       // console.log('time:'+time + ' height:'+heights+'tops:'+tops)
  //       this.tm[num-1] += 1
  //       return <div className = {bottleFull} style={{left:getLeft(),height:heights,top:tops}}/>
  //     } else {
  //       return <div className = {bottleFull} style={{left:getLeft()}}/>
  //     }

  //   }

  // }

  componentDidUpdate(prevProps: MainProps) {
    // if (this.props.workingPoint.length == 2 && prevProps.workingPoint.length == 1) {
    //   if (this.t1 == null) {
    //     this.t1 = setInterval(() => { //开启10秒倒计时，结束后自动更新路径
    //       if (this.props.time == 0) {
    //         this.props.dispatch(sendRoutesAction(this.props.workingPoint))
    //         this.props.dispatch(clearTimeAction())
    //         message.success("自动发送路径信息", 3)
    //         clearInterval(this.t1)
    //         this.t1 = null
    //       } else {
    //         this.props.dispatch(changeTimeAction())
    //       }
    //     }, 1000)
    //   }
    // }
    //自动获取移动端输入经纬度动作
    // if(this.props.mana == 'false'){
    //   this.props.dispatch(getPositionAction())
    // }
    // console.log(this.props.alertMode)
    if (this.props.alertMode) { //警报模式下开启30秒倒计时
      if (this.t2 == null) {
        this.t2 = setInterval(() => {
          this.props.dispatch(changeAlertTimeAction())
        }, 5000)
      }
    } else {
      if (this.props.alertTime == 0) { //警报时间到0时自动发送返航信息
        // hantime = 0;
        this.props.dispatch(sendReturnAction())
        this.props.dispatch(clearAlertTimeAction()) //复位30秒倒计时
      }
      if (this.t2) {
        clearInterval(this.t2)
        this.t2 = null;
      }
    }
  }

  componentWillUnmount() {
    if (this.t) {
      clearInterval(this.t)
    }
    if (this.t1) {
      clearInterval(this.t1)
    }
    if (this.t2) {
      clearInterval(this.t2)
    }
    if(this.th){
      clearInterval(this.th)
    }
  }

  componentDidMount() {    
    this.timer=setInterval(()=> {
      //console.log(123)
      var batts = this.state.batts;
      batts -= 0.3
      if (batts<=0) {
        batts= 0;
      }
      this.setState({
        batts:batts
      });
    }, 60000);
    //  this.props.dispatch(getWorkingPointsAction(this.props.vehicle_No))
    // this.t = setInterval(() => {
    // if(this.props.mana == 'true')return ;
    // else if(this.props.upda_road == 'true')return;
    // else {
    //   this.props.dispatch(simuDataAction())
    //   this.props.dispatch(findDistanceAction(this.props.workingPoint))
    //    this.props.dispatch(getPositionAction())
    //   this.props.dispatch(getGpsPositionAction())
    // }
    //  }, 3000)
    this.th = setInterval(() => {
      this.props.dispatch(updateThemeAction(sessionStorage.getItem('theme')))
    },100)
  }
}

function mapStateToProps(state: any): WorkingState {
  return state[PREFIX] as WorkingState
}

export = connect(mapStateToProps)(Main)
