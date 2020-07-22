import * as React from 'react'
import {connect} from 'dva'
import * as Redux from 'redux'
import {media, style} from 'typestyle'
import 'antd/dist/antd.css'
import {Button, Icon, Menu, message, Popover, Switch, Table, Upload,Input} from 'antd'
import SaveSpace from '../../components/main/savespace'
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
  WorkingState,
} from '../../models/main'
import {Map, Marker} from 'react-bmap'
import {
  addBoundPointAction,
  calculateRouteAction,
  changeVehicle_NoAction,
  findDistanceAction,
  getFileListAction,
  resetWorkingPointAction,
  sendCpOpenFileAction,
  sendRoutesAction,
  sendStartAction, sendUploadPoint2Action, sendUploadPointAction,
  simuDataAction,
  oneDataAction,
  updateAllPointAction,
  updateKeys,
  updateThemeAction,
  updateWorkingPointAction,
  getMissionPointAction,
} from '../../models/main/workingstatus'
import ContextMenu from '../../components/main/contextMenu'
import WorkingPoint from '../../components/main/workingPoint'
import {
  toggleModeAction
} from '../../models/main/changeTime'
import Column from 'antd/lib/table/Column'

type MainProps = WorkingState
type ManageProps = ManageState
type ThemeProps = ThemeState
type PlansProps = PlansState
type ManasProps = ManasState
type ChoseProps = ChoseState
type UpdaRoadProps = UpdaRoadState
type RecordProps = RecordState

interface MainDispatch {
  dispatch: Redux.Dispatch<any>
}

declare function require(path: string): any
declare const BMap
declare const BMapLib
declare const BMAP_HYBRID_MAP
const {Search} = Input
const batt = require('../../assets/Battery.png')
const boat = require('../../assets/car.png') //获取无人船的图标
const fly = require('../../assets/fly.png')
const circle = require('../../assets/circle.png') //获取警告模式下黄色圆圈的图标
const initImage = require('../../assets/init.jpg') //获取警告模式下黄色圆圈的图标
var tnum = 1;//距离数组初始化
var pra = '启动'

interface vehicle_No {
  vehicle_No: string
}

class Main extends React.Component<MainProps & MainDispatch & ManageProps & ThemeProps & PlansProps & ManasProps & ChoseProps & UpdaRoadProps & RecordProps & vehicle_No, any> {

  t: any ///存储setInterval的返回值用于清理
  t1: any//存储计时器
  t2: any //警告模式闪烁效果计时器
  th: any //存储主题
  timer:any

  columns = [
    {
      title: 'lat',
      dataIndex: 'lat',
      key: 'lat',
      height: 350,
    },
    {
      title: 'lng',
      dataIndex: 'lng',
      key: 'lng',
      height: 350,
    },
  ]

  columns1=[
    {
      title: '传感器',
      width: 100,
      dataIndex: 'name',
      key: 'name',
    },
    { title: 'A', dataIndex: 'address', key: '1' },
    { title: 'B', dataIndex: 'address', key: '2' },
    { title: 'C', dataIndex: 'address', key: '3' },
    { title: 'D', dataIndex: 'address', key: '4' },
    { title: 'E', dataIndex: 'address', key: '5' },
    { title: 'F', dataIndex: 'address', key: '6' },
    { title: 'G', dataIndex: 'address', key: '7' },
    { title: 'H', dataIndex: 'address', key: '8' },
  ]
  columns2=[
    { title: 'ID',
     dataIndex: 'address', 
     key: 'address' 
    },
    { title: 'level', 
    dataIndex: 'number', 
    key: 'number' 
    },
  ]
  columns3=[
    { title: '数据', width:100, dataIndex: 'name', key: 'name' },
    { title: '传感器A', dataIndex: 'address', key: '1' },
    { title: '传感器B', dataIndex: 'address', key: '2' },
    { title: '传感器C', dataIndex: 'address', key: '3' },
  ]
  openHeatmap: any
  closeHeatmap:any

  constructor(props, context) {
    super(props, context);
    this.state = {
      show: true,
      init: true,
      batts:100
    }
  }

  render() {
    const wholeStyle = style({
      height: '200vh',
      backgroundColor: this.props.theme == "false" ? '#424250' : '#e6e6e6',
    })

    const vehicle_select = style({ //车辆选择栏样式
        position: 'absolute',
        top: '17px',
        left: '422px',
        width: '170px',
        height: '100px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '500px',
          height: '110px',
          left: '36px',
          top: '19px'
        }
      ))

    const vehicle_selectTextStyle = style({  //车辆选择字体样式
        position: 'absolute',
        top: '10px',
        left: '13px',
        width: '134.8px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '222px',
          height: '49px',
          left: '160px',
          top: '31px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      )) 

    const vehicle_current = style({  //当前车辆样式
        position: 'absolute',
        top: '41px',
        left: '13px',
        width: '40px',
        height: '44px',
        fontFamily: 'Impact',
        fontSize: '36px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '60px',
          height: '48px',
          left: '80px',
          top: '65px',
          fontSize: '25px'
        }
      ))

    const buttonText1 = style({ //车辆选择按钮文字样式
        position: 'absolute',
        top: '50px',
        left: '0px',
        width: '100px',
        height: '28px',
        fontFamily: "PingFangSC",
        fontSize: '20px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.1px',
        textAlign: 'center',
        color: this.props.theme == "false" ? '#fff' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '120px',
          // height:'31px',
          left: '-25px',
          top: '13px',
          // fontWeight:900,
          fontSize: '25px'
        }
      ))

    const Button_vehicle_selectA = style({ //车辆切换A按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '530px',
          top: '0px'
        }
      ))

    const Button_vehicle_selectB = style({ //车辆切换B按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '610px',
          top: '0px'
        }
      ))

    const Button_vehicle_selectC = style({ //车辆切换C按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '530px',
          top: '55px'
        }
      ))

    const Button_vehicle_selectD = style({ //车辆切换D按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '610px',
          top: '55px'
        }
      ))

    const Button_vehicle_selectE = style({ //车辆切换E按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '690px',
          top: '0px'
        }
      ))

    const Button_vehicle_selectF = style({ //车辆切换F按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '690px',
          top: '55px'
        }
      ))

    const Button_vehicle_selectG = style({ //车辆切换G按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '770px',
          top: '0px'
        }
      ))

    const Button_vehicle_selectH = style({ //车辆切换H按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '70px',
          height: '50px',
          left: '770px',
          top: '55px'
        }
      ))

    const status = style({ //任务状态栏目
        left: '24px',
        top: '17px',
        width: '380px',
        height: '159px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        position: 'absolute'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '140px',
          left: '36px',
          top: '155px'
        }
      ))

    const missionStatus = style({  //任务状态栏目文字属性
        position: 'absolute',
        left: '15px',
        top: '10px',
        width: '317px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '476px',
          height: '49px',
          left: '23px',
          top: '5px',
          fontSize: '25px',
          lineHeight: 2.04,
          fontWeight: 900
        }
      ))

    const lineBetween = style({  //分割线
        position: 'absolute',
        width: '380px',
        height: '1px',
        opacity: 0.24,
        background: '#297fca',
        top: '46px'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '1.1px',
          top: '51px'
        }
      ))

    const timeTextStyle = style({ //预计剩余时间文字样式
        position: 'absolute',
        top: '54px',
        left: '14px',
        width: '88px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '14px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.86,
        letterSpacing: '0.5px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '300px',
          height: '49px',
          left: '21px',
          top: '79px',
          fontSize: '25px',
          lineHeight: 2.04,
          fontWeight: 900
        }
      ))

    const timeStyle = style({ //预计剩余时间样式
        position: 'absolute',
        top: '83px',
        left: '17px',
        width: '86px',
        height: '51px',
        fontFamily: 'Impact',
        fontSize: '42px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '129px',
          height: '56px',
          left: '26px',
          top: '91px',
          fontSize: '20px',
        }
      ))

    const distanceTextStyle = style({  //已出发字体样式
        position: 'absolute',
        top: '54px',
        left: '151px',
        width: '44px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '14px',
        fontWeight: 900,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.86,
        letterSpacing: '0.5px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '200px',
          height: '49px',
          left: '334px',
          top: '79px',
          fontSize: '25px',
          lineHeight: 2.04
        }
      ))

    const distanceStyle = style({  //出发数字样式
        position: 'absolute',
        top: '83px',
        left: '150px',
        width: '78px',
        height: '51px',
        fontFamily: 'Impact',
        fontSize: '42px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.8px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '117px',
          height: '56px',
          left: '225px',
          top: '91px',
          fontSize: '20px',
        }
      ))

    const distanceCompanyStyle = style({ //出发单位样式
        position: 'absolute',
        fontSize: '22px',
        letterSpacing: '4.6px',
        top: '17px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '20px'
        }
      ))

    const missionPointStyle = style({  //任务点剩余样式
        position: 'absolute',
        top: '54px',
        left: '261px',
        width: '73px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '14px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.86,
        letterSpacing: '0.5px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '240px',
          height: '49px',
          left: '602px',
          top: '79px',
          fontSize: '25px',
          lineHeight: 2.04,
          fontWeight: 900
        }
      ))

    const leftPointStyle = style({ //剩余点数样式
        position: 'absolute',
        top: '81px',
        left: '260px',
        width: '79px',
        height: '51px',
        fontFamily: 'Impact',
        fontSize: '42px',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.8px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '119px',
          height: '56px',
          left: '390px',
          top: '89px',
          fontSize: '20px'
        }
      ))

    const totalPointStyle = style({  //总点数样式
        position: 'absolute',
        top: '17px',
        fontSize: '22px',
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '20px'
        }
      ))

    const speed = style({  //速度栏目样式
        position: 'absolute',
        top: '187px',
        left: '24px',
        width: '187px',
        height: '144px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '36px',
          top: '320px'
        }
      ))

    const speedTextStyle = style({ //速度文字样式
        position: 'absolute',
        top: '10px',
        left: '15px',
        width: '150px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '225px',
          height: '49px',
          left: '113px',
          top: '33px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const speedLine = style({  //速度栏分割线
        position: 'absolute',
        top: '46px',
        width: '187px',
        height: '1px',
        opacity: 0.24,
        backgroundColor: '#297fca'
      },
      media({minWidth: 1700},
        {
          width: '281px',
          height: '1.1px',
          top: '51px'
        }
      ))

    const speedNumStyle = style({  //速度样式
        position: 'absolute',
        top: '54px',
        left: '19px',
        width: '102px',
        height: '56px',
        fontFamily: 'PingFangSC',
        fontSize: '46px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.7px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '153px',
          height: '62px',
          left: '29px',
          top: '59px',
          fontSize: '25px'
        }
      ))

    const speedComStyle = style({  //速度单位样式
        position: 'absolute',
        top: '17px',
        fontSize: '26px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '15px'
        }
      ))

    const accelerate = style({  //加速度栏目样式
        position: 'absolute',
        top: '250px',
        left: '24px',
        width: '187px',
        height: '144px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '36px',
          top: '430px'
        }
      ))

    const accTextStyle = style({ //加速度文字样式
        position: 'absolute',
        top: '10px',
        left: '15px',
        width: '150px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '225px',
          height: '49px',
          left: '113px',
          top: '25px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const accLine = style({  //加速度栏分割线
        position: 'absolute',
        top: '46px',
        width: '187px',
        height: '1px',
        opacity: 0.24,
        backgroundColor: '#297fca'
      },
      media({minWidth: 1700},
        {
          width: '281px',
          height: '1.1px',
          top: '51px'
        }
      ))

    const accNumStyle = style({  //加速度样式
        position: 'absolute',
        top: '54px',
        left: '19px',
        width: '102px',
        height: '56px',
        fontFamily: 'Impact',
        fontSize: '46px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.7px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '153px',
          height: '62px',
          left: '29px',
          top: '59px',
          fontSize: '25px'
        }
      ))

    const accComStyle = style({  //加速度单位样式
        position: 'absolute',
        top: '17px',
        fontSize: '26px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '15px'
        }
      ))

    const power = style({  //功率栏目样式
        position: 'absolute',
        top: '250px',
        left: '24px',
        width: '187px',
        height: '144px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '36px',
          top: '540px'
        }
      ))

    const powerTextStyle = style({ //功率文字样式
        position: 'absolute',
        top: '10px',
        left: '15px',
        width: '150px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '225px',
          height: '49px',
          left: '113px',
          top: '33px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const powerLine = style({  //功率栏分割线
        position: 'absolute',
        top: '46px',
        width: '187px',
        height: '1px',
        opacity: 0.24,
        backgroundColor: '#297fca'
      },
      media({minWidth: 1700},
        {
          width: '272px',
          height: '1.1px',
          top: '51px'
        }
      ))

    const powerNumStyle = style({  //功率样式
        position: 'absolute',
        top: '54px',
        left: '19px',
        width: '102px',
        height: '56px',
        fontFamily: 'Impact',
        fontSize: '46px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.7px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '153px',
          height: '62px',
          left: '29px',
          top: '59px',
          fontSize: '25px'
        }
      ))

    const powerComStyle = style({  //功率单位样式
        position: 'absolute',
        top: '17px',
        fontSize: '26px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '15px'
        }
      ))

    const location = style({ //当前位置栏样式
        position: 'absolute',
        top: '340px',
        left: '424px',
        width: '170px',
        height: '100px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '503px',
          top: '430px'
        }
      ))

    const locationTextStyle = style({  //当前位置字体样式
        position: 'absolute',
        top: '10px',
        left: '13px',
        width: '134.8px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '202px',
          height: '49px',
          left: '80px',
          top: '25px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const locationNumStyle = style({  //当前位置坐标样式
        position: 'absolute',
        top: '54px',
        left: '19px',
        width: '102px',
        height: '56px',
        fontFamily: 'Impact',
        fontSize: '46px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.7px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '153px',
          height: '62px',
          left: '200px',
          top: '20px',
          fontSize: '25px'
        }
      ))

    const sport = style({  //运动角度栏目样式
        position: 'absolute',
        top: '187px',
        left: '222px',
        width: '181px',
        height: '144px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '503px',
          top: '320px'
        }
      ))

    const sportTextStyle = style({ //运动角度文字样式
        position: 'absolute',
        top: '10px',
        left: '15px',
        width: '144px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '216px',
          height: '49px',
          left: '113px',
          top: '33px',
          fontWeight: 900,
          fontSize: '25px',
          lineHeight: 1.79
        }
      ))

    const sportLine = style({  //运动栏分割线
        position: 'absolute',
        top: '46px',
        width: '181px',
        height: '1px',
        opacity: 0.24,
        backgroundColor: '#297fca'
      },
      media({minWidth: 1700},
        {
          width: '272px',
          height: '1.1px',
          top: '51px'
        }
      ))

    const sportNumStyle = style({  //运动角度样式
        position: 'absolute',
        top: '57px',
        left: '17px',
        width: '54px',
        height: '56px',
        fontFamily: 'Impact',
        fontSize: '46px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.7px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '81px',
          height: '62px',
          left: '20px',
          top: '63px',
          fontSize: '25px'
        }
      ))

    const save = style({ //存储空间栏样式
        position: 'absolute',
        top: '17px',
        left: '422px',
        width: '170px',
        height: '100px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '255px',
          height: '110px',
          left: '633px',
          top: '19px'
        }
      ))

    const saveTextStyle = style({  //存储空间字体样式
        position: 'absolute',
        top: '10px',
        left: '13px',
        width: '134.8px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '202px',
          height: '29px',
          left: '20px',
          top: '11px',
          fontSize: '24px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const saveStyle = style({  //存储空间样式
        position: 'absolute',
        top: '41px',
        left: '13px',
        width: '40px',
        height: '44px',
        fontFamily: 'Impact',
        fontSize: '36px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '60px',
          height: '48px',
          left: '20px',
          top: '45px',
          fontSize: '54px'
        }
      ))

    const saveComStyle = style({ //存储空间单位样式
        position: 'absolute',
        top: '17px',
        fontSize: '16px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '24px'
        }
      ))

    const saveSpaceStyle = style({ //存储空间右方圆盘样式
      position: 'absolute',
      top: '10px',
      left: '60px',
      width: '52px',
      height: '52px'
    }, media({minWidth: 1700},
      {
        top: '15px',
        left: '121px',
        width: '73px',
        height: '57px'
      }
    ))

    const communicate = style({ //通信质量栏样式
        position: 'absolute',
        top: '126px',
        left: '422px',
        width: '170px',
        height: '100px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '255px',
          height: '110px',
          left: '633px',
          top: '155px'
        }
      ))

    const communicateTextStyle = style({  //通信质量字体样式
        position: 'absolute',
        top: '10px',
        left: '13px',
        width: '134.8px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '202px',
          height: '29px',
          left: '20px',
          top: '11px',
          fontWeight: 900,
          fontSize: '24px',
          lineHeight: 1.79
        }
      ))

    const communicateStyle = style({  //通信质量样式
        position: 'absolute',
        top: '43px',
        left: '13px',
        width: '40px',
        height: '44px',
        fontFamily: 'Impact',
        fontSize: '36px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '60px',
          height: '48px',
          left: '20px',
          top: '47px',
          fontSize: '54px',
        }
      ))

    const communicateComStyle = style({ //通信质量单位样式
        position: 'absolute',
        top: '17px',
        fontSize: '16px'
      },
      media({minWidth: 1700},
        {
          top: '27px',
          fontSize: '24px'
        }
      ))

    const signFirst = style({ //通信质量第一根信号样式
      position: 'absolute',
      left: '96px',
      top: '69.8px',
      width: '4.1px',
      height: '12.2px',
      backgroundColor: '#2298ff'
    }, media({minWidth: 1700},
      {
        left: '134px',
        top: '76px',
        width: '5.7px',
        height: '13.4px'
      }
    ))

    const signSecond = style({ //通信质量第二根信号样式
      position: 'absolute',
      left: '110.4px',
      top: '64.2px',
      width: '4.1px',
      height: '17.8px',
      backgroundColor: '#2298ff'
    }, media({minWidth: 1700},
      {
        left: '154.56px',
        top: '70.62px',
        width: '5.7px',
        height: '19.58px'
      }
    ))

    const signThird = style({ //通信质量第三根信号样式
      position: 'absolute',
      left: '124.9px',
      top: '52px',
      width: '4.1px',
      height: '29.2px',
      backgroundColor: '#2298ff'
    }, media({minWidth: 1700},
      {
        left: '174.86px',
        top: '57px',
        width: '5.7px',
        height: '32.12px'
      }
    ))

    const signForth = style({ //通信质量第四根信号样式
      position: 'absolute',
      left: '138px',
      top: '36.7px',
      width: '4.1px',
      height: '45.1px',
      backgroundColor: '#808087'
    }, media({minWidth: 1700},
      {
        left: '193.2px',
        top: '40.37px',
        width: '5.7px',
        height: '49.61px'
      }
    ))

    const signFifth = style({ //通信质量第五根信号样式
      position: 'absolute',
      left: '152.2px',
      top: '14px',
      width: '4.1px',
      height: '67.5px',
      backgroundColor: '#808087'
    }, media({minWidth: 1700},
      {
        left: '213.08px',
        top: '15.4px',
        width: '5.7px',
        height: '74.25px'
      }
    ))

    const battery = style({ //电量栏样式
        position: 'absolute',
        top: '255px',
        left: '424px',
        width: '170px',
        height: '100px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '381px',
          height: '100px',
          left: '503px',
          top: '540px'
        }
      ))

    const batteryTextStyle = style({  //电量字体样式
        position: 'absolute',
        top: '10px',
        left: '13px',
        width: '134.8px',
        height: '26px',
        fontFamily: 'PingFangSC',
        fontSize: '16px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '202px',
          height: '29px',
          left: '90px',
          top: '1px',
          fontSize: '25px',
          fontWeight: 900,
          lineHeight: 1.79
        }
      ))

    const batteryStyle = style({  //电量样式
        position: 'absolute',
        top: '44px',
        left: '12px',
        width: '40px',
        height: '44px',
        fontFamily: 'Impact',
        fontSize: '36px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '60px',
          height: '48px',
          left: '88px',
          top: '38px',
          fontSize: '54px'
        }
      ))

    const batteryComStyle = style({ //电量单位样式
        position: 'absolute',
        top: '12px',
        fontSize: '16px',
        left: '40px',
        width: '33px',
        height: '22px',
        fontFamily: 'PingFangSC',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '0.4px',
        textAlign: 'left',
        color: this.props.theme == "false" ? '#e3e3e3' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '50px',
          height: '24px',
          top: '25px',
          left: '85px',
          fontWeight: 900,
          fontSize: '24px'
        }
      ))

    const batteryPosStyle = style({  //电池控件样式
      position: 'absolute',
      top: '17.5px',
      left: '101.5px',
      width: '65px',
      height: '66px',
    }, media({minWidth: 1700},
      {
        top: '19.25px',
        left: '256.1px',
        width: '90px',
        height: '72.6px'
      }
    ))

    const buttonIcon = style({ //按钮中图片样式
        position: 'absolute',
        top: '13px',
        width: '100px',
        height: '30px',
        left: '0px',
        fontFamily: 'simple-line-icons',
        fontSize: '30px',
        fontWeight: 'normal',
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: 'normal',
        textAlign: 'center',
        color: this.props.theme == "false" ? '#fff' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '120px',
          top: '14px',
          height: '33px',
        }
      ))

    const buttonText = style({ //按钮文字样式
        position: 'absolute',
        top: '50px',
        left: '0px',
        width: '100px',
        height: '28px',
        fontFamily: "PingFangSC",
        fontSize: '20px',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 'normal',
        letterSpacing: '1.1px',
        textAlign: 'center',
        color: this.props.theme == "false" ? '#fff' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '120px',
          fontSize: '25px'
        }
      ))

    const Button1_0 = style({  //open按钮样式
        position: 'absolute',
        top: '36px',
        left: '624px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '940px',
          top: '20px'
        }
      ))

    const Button1_1 = style({ //启动按钮样式
        position: 'absolute',
        top: '36px',
        left: '742px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '1120px',
          top: '20px'
        }
      ))

    const Button1_2 = style({ //返航按钮样式
        position: 'absolute',
        top: '36px',
        left: '859px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '1300px',
          top: '20px'
        }
      ))

    const Button1_3 = style({  //停止按钮样式
        position: 'absolute',
        top: '36px',
        left: '977px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '1475px',
          top: '20px'
        }
      ))

    const Button1_4 = style({ //更新路径按钮样式
        position: 'absolute',
        top: '36px',
        left: '1095px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '1650px',
          top: '20px'
        }
      ))

      const Button1_5 = style({ //重新规划按钮样式
        position: 'absolute',
        top: '36px',
        left: '1095px',
        width: '100px',
        height: '87px',
        borderRadius: '2px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        border: 'solid 1px #2298ff',
      },
      media({minWidth: 1700},
        {
          width: '120px',
          height: '96px',
          left: '1835px',
          top: '20px'
        }
      ))

    const mapStyle = style({ //地图栏样式
        position: 'absolute',
        top: '143px',
        left: '623px',
        width: '570px',
        height: '365px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '845px',
          height: '750px',
          left: '935px',
          top: '127px'
        }
      ))

    const mapMenu = style({  //地图栏导航栏
        position: 'absolute',
        top: '0px',
        width: '570px',
        height: '28px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        borderRadius: '1px',
        borderBottom: 'solid 1px #297fca',
        fontSize: '16px',
        fontFamily: 'PingFangSC',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        color: this.props.theme == "false" ? '#fff' : '#000'
      },
      media({minWidth: 1700},
        {
          width: '1025px',
          height: '38px',
          fontSize: '22.4px',
          fontWeight: 840
        }
      ))

    const mapMenu1 = style({  //地图栏导航栏（for任务点列表）
        position: 'absolute',
        top: '0px',
        width: '570px',
        height: '28px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
        borderRadius: '1px',
        borderBottom: 'solid 1px #297fca',
        fontSize: '16px',
        fontFamily: 'PingFangSC',
        fontWeight: 600,
        fontStyle: 'normal',
        fontStretch: 'normal',
        lineHeight: 1.63,
        letterSpacing: '0.6px',
        color: this.props.theme == "false" ? '#fff' : '#000'
        // border:'solid 3px #2298ff'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '38px',
          fontSize: '22.4px',
          fontWeight: 840
        }
      ))

    const mapBor = style({ //地图栏展示内容样式
      position: "absolute",
      top: '58px',
      left: '12px',
      width: '546px',
      height: '293px',
      opacity: 0.96,
      display: this.props.keysmap == "map" ? 'block' : 'none'
    }, media({minWidth: 1700},
      {
        top: '40.8px',
        left: '1px',
        width: '766.4px',
        height: '620.3px'
      }
    ))

    const camBor = style({ //摄像栏展示内容样式
      position: "absolute",
      top: '58px',
      left: '12px',
      width: '546px',
      height: '293px',
      opacity: 0.96,
      display: this.props.keysmap != "map" && this.props.keysmap != "radar" ? 'block' : 'none'
    }, media({minWidth: 1700},
      {
        top: '40.8px',
        left: '1px',
        width: '766.4px',
        height: '356.3px'
      }
    ))

    const radarBor = style({ //雷达展示内容样式
      position: "absolute",
      top: '58px',
      left: '12px',
      width: '546px',
      height: '293px',
      opacity: 0.96,
      display: this.props.keysmap == "radar" ? 'block' : 'none'
    }, media({minWidth: 1700},
      {
        top: '40.8px',
        left: '1px',
        width: '766.4px',
        height: '356.3px'
      }
    ))

    const mapButton = style({  //地图右上方添加航点按钮
      position: 'absolute',
      top: '0px',
      left: '490px',
      width: '100px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '900.2px',
        fontSize: '12px'
      }
    ))

    const translButton = style({  //地图右上方优化航点按钮
      position: 'absolute',
      top: '0px',
      left: '403px',
      width: '100px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '644.2px',
        fontSize: '14.2px'
      }
    ))

    const lineButton = style({  //地图右上方优化直线航点按钮
      position: 'absolute',
      top: '0px',
      left: '316px',
      width: '100px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '374.2px',
        fontSize: '14.2px'
      }
    ))

    const cleanButton = style({  //地图右上方清除航线按钮
      position: 'absolute',
      top: '0px',
      left: '223px',
      width: '80px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '414.2px',
        fontSize: '14.2px'
      }
    ))

    const camInitButton = style({  //地图右上方初始化按钮
      position: 'absolute',
      top: '0px',
      left: '296px',
      width: '80px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap != "map" && this.props.keysmap != "radar" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '504.2px',
        fontSize: '14.2px'
      }
    ))

    const camInButton = style({  //地图右上方登录按钮
      position: 'absolute',
      top: '0px',
      left: '393px',
      width: '80px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "photograph" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '594.2px',
        fontSize: '14.2px'
      }
    ))

    const camOutButton = style({  //地图右上方登出按钮
      position: 'absolute',
      top: '0px',
      left: '490px',
      width: '80px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "photograph" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '5px',
        left: '684.2px',
        fontSize: '14.2px'
      }
    ))

    const camGetButton = style({  //地图右方预览按钮
      position: 'absolute',
      top: '173px',
      left: '1200px',
      height: '160px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap != "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '177px',
        left: '1707px',
        height: '178px',
        fontSize: '14.2px'
      }
    ))

    const camStopButton = style({  //地图右方停止预览按钮
      position: 'absolute',
      top: '345px',
      left: '1200px',
      height: '160px',
      fontFamily: 'PingFangSC',
      fontSize: '12px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'center',
      background: '#363636',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap != "map" ? 'block' : 'none',
      backgroundColor: this.props.theme == "false" ? '#363636' : '#fff',
      color: this.props.theme == "false" ? '#fff' : '#000'
    }, media({minWidth: 1700},
      {
        top: '443px',
        height: '178px',
        left: '1707px',
        fontSize: '14.2px'
      }
    ))

    const popstyle = (  //弹出添加路径点的框
      <div>
        经度:<input type="text" id="lng" defaultValue={this.props.lng.toString()}/><br/>
        纬度:<input type="text" id="lat" defaultValue={this.props.lat.toString()}/><br/>
      </div>
    )

    const mapSwitch = style({  //连线模式switch样式
      position: 'absolute',
      top: '70px',
      left: '458px',
      width: '100px',
      height: '24px',
      fontFamily: 'PingFangSC',
      fontSize: '13px',
      fontWeight: 'normal',
      fontStyle: 'normal',
      fontStretch: 'normal',
      lineHeight: 'normal',
      letterSpacing: 'normal',
      textAlign: 'left',
      background: '#363636',
      borderRadius: '13.3px',
      border: 'solid 1px #e3e3e3',
      zIndex: 10,
      display: this.props.keysmap == "map" ? 'block' : 'none'
    }, media({minWidth: 1700},
      {
        top: '47px',
        left: '875.2px',
        fontSize: '18.2px'
      }
    ))

    const missionPoint = style({ //任务点列表栏
        position: 'absolute',
        top: '513px',
        left: '623px',
        width: '570px',
        height: '264px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '340px',
          left: '36px',
          top: '1580px'
        }
      ))

      const dataInfo = style({ //任务点列表栏
        position: 'absolute',
        top: '513px',
        left: '623px',
        width: '570px',
        height: '264px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '300px',
          left: '36px',
          top: '654px'
        }
      ))

      const dataInfo1 = style({ //任务点列表栏
        position: 'absolute',
        top: '513px',
        left: '623px',
        width: '570px',
        height: '264px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '200px',
          left: '36px',
          top: '964px'
        }
      ))

      const dataInfo3 = style({ //任务点列表栏
        position: 'absolute',
        top: '513px',
        left: '623px',
        width: '570px',
        height: '264px',
        borderRadius: '3px',
        backgroundColor: this.props.theme == "false" ? '#363636' : '#fff'
      },
      media({minWidth: 1700},
        {
          width: '850px',
          height: '200px',
          left: '36px',
          top: '1174px'
        }
      ))

    const tableClass = style({  //连线模式switch样式
      background: '#363636',
      color: '#fff',
    })

    const imageStyle = style({
      position: "fixed",
      zIndex: 100,
      left: "250px",
      top: "100px"
    })

    return (
      <div className={wholeStyle}>
        {
          /*初始化欢迎图片*/
          this.state.show ? <img className={imageStyle} src={initImage}/> : ''
        }

        {/* 车辆选择栏目 */}
        <div className={vehicle_select}>
          <div className={vehicle_selectTextStyle}>当前选择车辆    {this.props.vehicle_No}</div>
          {/* <div className={vehicle_current}>{this.props.vehicle_No}</div> */}
          <Button className={Button_vehicle_selectA} type="primary" onClick={(() => this.handleCarSelect('A'))}>
            <div className={buttonText1}>A</div>
          </Button>
          <Button className={Button_vehicle_selectB} type="primary" onClick={(() => this.handleCarSelect('B'))}>
            <div className={buttonText1}>B</div>
          </Button>
          <Button className={Button_vehicle_selectC} type="primary" onClick={(() => this.handleCarSelect('C'))}>
            <div className={buttonText1}>C</div>
          </Button>
          <Button className={Button_vehicle_selectD} type="primary" onClick={(() => this.handleCarSelect('D'))}>
            <div className={buttonText1}>D</div>
          </Button>
          <Button className={Button_vehicle_selectE} type="primary" onClick={(() => this.handleCarSelect('E'))}>
            <div className={buttonText1}>E</div>
          </Button>
          <Button className={Button_vehicle_selectF} type="primary" onClick={(() => this.handleCarSelect('F'))}>
            <div className={buttonText1}>F</div>
          </Button>
          <Button className={Button_vehicle_selectG} type="primary" onClick={(() => this.handleCarSelect('G'))}>
            <div className={buttonText1}>G</div>
          </Button>
          <Button className={Button_vehicle_selectH} type="primary" onClick={(() => this.handleCarSelect('H'))}>
            <div className={buttonText1}>H</div>
          </Button>
        </div>

        {/*任务栏 */}
        <div className={status}>
          <div className={missionStatus}>任务执行中</div>
          <div className={lineBetween}/>
          <div className={timeTextStyle}>预计剩余时间 3:50</div>
          {/* <div className={timeTextStyle}>预计剩余时间 {this.getTime()}</div> */}
          {/* <div className={timeStyle}>{this.getTime()}</div> */}
          <div className={distanceTextStyle}>已出发 0km</div>
          {/* <div className={distanceStyle}>12
            <span className={distanceCompanyStyle}> km </span>
          </div> */}
          {/* <div className={missionPointStyle}> 任务点剩余    {this.getLessPoint()} /{this.getTotalPoint()}</div> */}
          <div className={missionPointStyle}> 任务点剩余    12/12</div>
          {/* <div className={leftPointStyle}> {this.getLessPoint()}
            <span className={totalPointStyle}> /{this.getTotalPoint()}</span>
          </div> */}
        </div>

        {/*速度栏 最大速度3m/s*/}
        <div className={speed}>
          <div className={speedTextStyle}>速度 {isNaN(this.props.speed) ? 12 : this.props.speed.toFixed(2)} m/s</div>
          {/* <div className={speedLine}/> */}
          {/* <div className={speedNumStyle}> {isNaN(this.props.speed) ? 12 : this.props.speed.toFixed(2)}
            <span className={speedComStyle}> m/s </span> 
      </div>*/}
        </div>

        {/*瞬时加速度栏 */}
        <div className={accelerate}>
          <div className={accTextStyle}>加速度 3m/s²</div>
          {/* <div className={accLine}/> */}
          {/* <div className={accNumStyle}>5
            <span className={accComStyle}>m/s²</span>
          </div> */}
        </div>

        {/*瞬时功率栏 */}
        <div className={power}>
          <div className={powerTextStyle}>功率 12kw</div>
          {/* <div className={powerLine}/>
          < div className={powerNumStyle}>1.2
            <span className={powerComStyle}>kw</span>
          </div> */}
        </div>

        {/*当前位置栏 */}
        <div className={location}>
          <div className={locationTextStyle}>当前位置</div>
          <div className={locationNumStyle}>N{this.props.lng.toFixed(6)}
            <div>E{this.props.lat.toFixed(6)}</div>
          </div>
        </div>

        {/*运动角度栏 */}
        <div className={sport}>
          <div className={sportTextStyle}>北向角度 {isNaN(this.props.angle) ? 56 : this.props.angle.toFixed(1)}</div>
          {/* <div className={sportLine}/> */}
          {/* <div className={sportNumStyle}> {isNaN(this.props.angle) ? 56 : this.props.angle.toFixed(1)} </div> */}
        </div>

        {/*存储空间栏 */}
        {/* <div className={save}>
          <div className={saveTextStyle}>存储空间</div>
          <div className={saveStyle}> 25
            <span className={saveComStyle}>%</span>
          </div>
          <div className={saveSpaceStyle}>
            <SaveSpace/>
          </div>
        </div> */}

        {/*通信质量栏 */}
        {/* <div className={communicate}>
          <div className={communicateTextStyle}>通信质量</div>
          <div className={communicateStyle}> 75
            <span className={communicateComStyle}> % </span>
          </div>
          <div className={signFirst}/>
          <div className={signSecond}/>
          <div className={signThird}/>
          <div className={signForth}/>
          <div className={signFifth}/>
        </div> */}

        {/*电量栏 1块电池续航能力为1小时*/}
        <div className={battery}>
          <div className={batteryTextStyle}> 电量</div>
          {/* <div className={batteryStyle}> {this.getBatteryTime() == undefined ? 28 : this.getBatteryTime()}
            <span className={batteryComStyle}> % </span>
          </div> */}
          <div className={batteryStyle}> {this.state.batts.toFixed(1)}%</div>
          <div><img className={batteryPosStyle} src={batt}/>
            {this.renderBattery()}</div>
        </div>

        {/* open按钮 */}
        <Upload {...this.uploadProp}>
          <Button className={Button1_0} type="primary">
            <div className={buttonIcon}><Icon type="home"/></div>
            <div className={buttonText}> 打开</div>
          </Button>
        </Upload>

        {/*启动按钮*/}
        <Button className={Button1_1} type="primary" onClick={(() => this.sendAction(0))}>
          <div className={buttonIcon}><Icon type="home"/></div>
          <div className={buttonText}> 启动</div>
        </Button>

        {/* 返航按钮 */}
        <Button className={Button1_2} type="primary" onClick={(() => this.props.dispatch(sendUploadPointAction()))}>
          <div className={buttonIcon}><Icon type="home"/></div>
          <div className={buttonText}> 采集</div>
        </Button>

        {/* 停止按钮 */}
        <Button className={Button1_3} type="primary" onClick={(() => this.props.dispatch(sendUploadPoint2Action()))}>
          <div className={buttonIcon}><Icon type='home'/></div>
          <div className={buttonText}>保存</div>
        </Button>

        {/* 更新路径按钮 */}
        <Button className={Button1_4} type="primary" onClick={(() => this.props.dispatch(sendRoutesAction(this.props.workingPoint)))}>
          <div className={buttonIcon}><Icon type="home"/></div>
          <div className={buttonText}> 更新</div>
        </Button>

          {/* 重新规划按钮 */}
        <Button className={Button1_5} type="primary" onClick={(() => this.props.dispatch(oneDataAction()))}>
          <div className={buttonIcon}><Icon type="home"/></div>
          <div className={buttonText}> 重新规划</div>
        </Button>


        {/*地图栏*/}
        <div className={mapStyle}>
          <Menu className={mapMenu} mode="horizontal" defaultSelectedKeys={['map']} onClick={(e) => this.testClick(e)} selectedKeys={[this.props.keysmap]}>
            <Menu.Item key="map">地图视图</Menu.Item>
            <Menu.Item key="photograph">摄像头视图</Menu.Item>
            {/* <Menu.Item key="radar">雷达视图</Menu.Item> */}
          </Menu>
          <Popover content={popstyle} title="调整经纬度">
            <Button className={mapButton} type="primary"
                    onClick={(() => this.addPoint(this.props.lat.toFixed(8), this.props.lng.toFixed(8)))}>
              添加路径点
            </Button>
          </Popover>
          {/* <Button className={lineButton} type="primary">密集路线</Button> */}
          <Switch className={mapSwitch} checkedChildren="连线模式" unCheckedChildren="平扫模式" checked={this.props.mode}
                  onChange={(checked: boolean) => {this.props.dispatch(toggleModeAction())}}/>
          <div className={mapBor}>
            {/* 实例化地图  */}
            <Map style={{height: '233%', width: '133.5%'}}
            // 初始化地图中心点  
                 center={this.props.workingPoint[0] ? this.props.workingPoint[0] : (this.props.lng ? {
                   lng: this.props.lng,
                   lat: this.props.lat
                 } : {lng:121.4494651085, lat: 31.0313365968})}
                 //设置地图缩放等级 
                 zoom={16} enableScrollWheelZoom 
                 mapType={BMAP_HYBRID_MAP}
                 //地图事件 
                 events={{
                   click: (e) => {
                     this.handleMapClick(e)
                   }
                 }}>
                  {/* 渲染无人船位置 */}
              {this.renderBoat()}
              {/* 渲染工作路径点 */}
              {this.renderFlag()}
               {/* 渲染警告模式下的船体黄色圆圈 */}
              {this.renderYellowCircle()}
              {/* 渲染平扫模式下的四个点 */}
              {this.renderMarker()}
              {/* 鼠标右键点击事件 */}
              <ContextMenu dispatch={this.props.dispatch}/>
            </Map>
          </div>

          <div className={camBor} id="divPlugin">camera field</div>
          <div className={radarBor} id="divPlugin1">Radar</div>
          <Button className={camInitButton} type="primary">初始化</Button>
          <Button className={camInButton} type="primary">登录</Button>
          <Button className={camOutButton} type="primary">预览</Button>
        </div>

        <Button className={camGetButton} type="primary"><Icon
          type="camera"/><br/>拍<br/>照</Button>
        <Button className={camStopButton} type="primary"><Icon
          type="video-camera"/><br/>录<br/>像</Button>

        <div className={dataInfo}>
          <Table columns={this.columns1} dataSource={this.data1} scroll={{x:1300}} />
        </div>

        <div className={dataInfo1}>
          <Table columns={this.columns2} dataSource={this.data2} scroll={{x:1000}} />
        </div>
        <div className={dataInfo3}>
          <Table columns={this.columns3} dataSource={this.props.workingPoint} scroll={{x:1300}} />
        </div>


        {/*任务点列表栏*/}
        <div className={missionPoint}>
          {/* <Menu className={mapMenu1} mode="horizontal">
            <Menu.Item key="point">任务点列表</Menu.Item>
          </Menu> */}
          <Table className={tableClass} dataSource={this.props.workingPoint} columns={this.columns} scroll={{y: 240}}/>
        </div>
      </div>
    )
  }
  data1=[
    {
      key: '1',
      name: '速度',
      address: '0.5',
    },
    {
      key:'2',
      name:'角度',
      address:'60'
    }
  ]
  data2=[
    {
      key:'1',
      address:'1234',
      number:'4'
    },
  ]
  data3=[
    {
      key:'1', 
    }
  ]

  //读取文件并将文件显示在任务点列表区域
  dataSource = []

  uploadProp = {
    name: 'file',
    accept: '.csv',
    action: '192.168.220.129:8484/ocean/download',
    beforeUpload: (file, filelist) => {
      let reader = new FileReader();
      reader.onload = (e) => {
        this.dataSource = (reader.result as any).split(/[\n]/).filter((row, index) => index !== 0 && row !== '').map((row, index) => {
          const arr = row.split(',')
          if (index === 0) {
            this.props.dispatch(resetWorkingPointAction({lng: arr[2], lat: arr[1]}))
          } else {
            this.props.dispatch(updateWorkingPointAction({lng: arr[2], lat: arr[1]}))
          }
          return {
            lat: arr[1],
            lng: arr[2]
          }
        })
        this.forceUpdate()
        this.props.dispatch(sendCpOpenFileAction(file.name))
      };
      reader.readAsText(file);
      // 阻止默认的文件上传功能
      return false
    },
  };


  handleCarSelect(car) {
    this.props.dispatch(changeVehicle_NoAction(car))
    this.props.dispatch(getFileListAction(car))
  }

  //清除航线
  cleanPoint() {
    if (this.props.workingPoint.length == undefined || this.props.workingPoint.length < 2) return
    const resultPoint = []
    resultPoint.push(this.props.workingPoint[0])
    this.props.dispatch(updateAllPointAction(resultPoint))
  }

  // 添加路径点按钮响应
  addPoint(lat, lng) {
    lat = parseFloat((document.getElementById('lat') as any).value)
    lng = parseFloat((document.getElementById('lng') as any).value)
    if (this.state.init) {
      this.setState({
        init: false
      })
      this.props.dispatch(resetWorkingPointAction({lng, lat}))
    } else {
      this.props.dispatch(updateWorkingPointAction({lng, lat}))
    }
  }

  //百度坐标转化为GPS坐标(官方没有提供相应转换方式，采用github上的coordtransform库实现)
  changeLocationArray(points: { lng: number, lat: number }[]) {
    return points.map((point) => {
      const gcj02Coord = coordtransform.bd09togcj02(point.lng, point.lat); //百度经纬度坐标转国测局坐标
      const wgs84Coord = coordtransform.gcj02towgs84(gcj02Coord[0], gcj02Coord[1]); //国测局坐标转wgs84坐标
      return {lng: wgs84Coord[0], lat: wgs84Coord[1]}
    })
  }

  //更改地图栏目显示内容
  testClick(e) {
    if (e.key != this.props.keysmap) {
      this.props.dispatch(updateKeys(e.key))
    }
  }

  //获取剩余的任务点
  getLessPoint() {
    var num = this.props.workingPoint.length
    var k = 0;
    if (isNaN(num)) {
      return 10;
    } else {
      if (this.props.workingPoint[0] == undefined) return;
      for (var i = 0; i < num; i++) {

        if (this.props.workingPoint[i].status == "done")
          num--;
        else if (this.props.workingPoint[i].status == "doing")
          break;
      }
      return num;
    }
  }

  //获取总的任务点
  getTotalPoint() {
    var num = this.props.workingPoint.length
    if (isNaN(num)) {
      return 20;
    } else {
      return num
    }
  }

  //发送指令
  sendAction(param) {
    switch (param) {
      case 0: {
        pra = "启动"
        this.props.dispatch(sendStartAction(0))
        break;
      }
      case 4: {
        pra = "开始";
        this.props.dispatch(sendStartAction(4))
        break;
      }
      case 10: {
        pra = "停止";
        this.props.dispatch(sendStartAction(10));
        break;
      }
      case 11: { //返航
        var backworkingPoint = []
        if (this.props.workingPoint.length < 2) break;
        if (this.props.mana == 'true') {
          message.info("当前正在进行采样操作，请稍后再试...", 3)
          return;
        }
        if (this.props.upda_road == 'true') {
          message.info("更新路径操作正在进行中，请稍后再试...", 3)
          return;
        }
        if (this.props.plans == 'true') {
          message.info("当前正在读取传感器数据，请稍后再试...", 3)
          return;
        }
        pra = "停止"
        var localPoint = {lng: this.props.lng, lat: this.props.lat, status: "doing"}
        backworkingPoint.push(localPoint)  //当前点作为起点
        backworkingPoint.push(this.props.workingPoint[0]) //起始点作为终点
        break;
      }
      case 15: {
        pra = "停止"
        this.props.dispatch(sendStartAction(15));
        break;
      }
      case 20: {
        pra = "停止"
        this.props.dispatch(sendStartAction(20));
        break;
      }
      case 50: {
        pra = "更新路径"
        this.props.dispatch(sendStartAction(50));
        message.info("正在更新路径信息", 3);
        break;
      }
    }
  }

  //初始化任务执行时间
  getTime() {
    var speed = this.props.speed  //初始化速度
    var time; //目标时间
    var nowLng = this.props.lng //目前经度
    var nowLat = this.props.lat //目前纬度
    var targetLng; //下一个目标的经度
    var targetLat;  //下一个目标的纬度
    if (this.props.workingPoint.length == 0 || this.props.workingPoint.length == 1) return 0;
    if (tnum > this.props.totalDistance.length) return 0;
    if (this.props.workingPoint[0].status == "doing" && this.props.workingPoint.length > 1) {
      this.props.workingPoint[0].status = "done"
      this.props.workingPoint[1].status = "doing"
    }
    if (tnum == this.props.totalDistance.length) {  //说明到了最后一个目标点
      targetLng = this.props.workingPoint[tnum].lng
      targetLat = this.props.workingPoint[tnum].lat
    } else {
      targetLng = this.props.workingPoint[tnum].lng == undefined ? 0 : this.props.workingPoint[tnum].lng
      targetLat = this.props.workingPoint[tnum].lat == undefined ? 0 : this.props.workingPoint[tnum].lat
    }
    let map = new BMap.Map("map")
    let PointNow = new BMap.Point(nowLng, nowLat)  //目前经纬度
    let PointTar = new BMap.Point(targetLng, targetLat)  //目标经纬度
    var distance = map.getDistance(PointNow, PointTar) //初始化距离
    if (distance <= 1) {  //如果当前经纬度与目标经纬度距离小于1米，则认为已经通过
      if (this.props.workingPoint[tnum] != undefined)
        this.props.workingPoint[tnum].status = "done" //修改过去的点为已完成状态
      if (this.props.workingPoint[tnum + 1] != undefined)
        this.props.workingPoint[tnum + 1].status = "doing" //修改下一个点为正在进行
      tnum++;
    }
    if (speed != 0.00) time = distance / speed;  //目前单位为秒
    else time = 0
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
      minute = Math.floor(minute % 60);
      fmt = `${second}:${minute}`;
    }
    return fmt;
  }

  //获取电量
  getBatteryInfor() {
    var voltage = (this.props.voltage1 <= this.props.voltage2) ? this.props.voltage1 : this.props.voltage2
    var result;
    if (voltage > 24) {
      result = 90 + (voltage - 24) * 10 / 1.2
    } else if (voltage <= 24 && voltage > 19.8) {
      result = 20 + (voltage - 19.8) * 70 / 4.2
    } else {
      result = (voltage - 18) * 20 / 1.8
    }
    return result.toFixed(0)
  }

  //获取续航时间()
  getBatteryTime() {
    //var batts = this.getBatteryInfor();
    //if (isNaN(batts)) return 100
    //var times = 100 * (batts / 100)
    //return times.toFixed(0)
  }

  //渲染无人船位置
  renderBoat() {
    if (this.props.lng && this.props.lat) {
      const myIcon = new BMap.Icon(boat, new BMap.Size(100, 100))
      myIcon.setImageSize(new BMap.Size(100, 100))
      return <Marker position={{lng: this.props.lng, lat: this.props.lat}} icon={myIcon} offset={new BMap.Size(0, 5)}
                     enableMassClear={false}/>
                     && 
              <Marker position={{lng: 113.84711204300291, lat: 22.681877811822183}} icon={myIcon} offset={new BMap.Size(0, 5)}
                     enableMassClear={false}/>
              
    } else {
      return null
    }
  }

  // 渲染警告模式下的船体黄色圆圈
  renderYellowCircle() {
    const myIcon = new BMap.Icon(circle, new BMap.Size(32, 32))
    myIcon.setImageSize(new BMap.Size(32, 32))
    return this.props.alertTime % 2 ?
      <Marker position={{lng: this.props.lng, lat: this.props.lat}} icon={myIcon} offset={new BMap.Size(0, 10)}
              enableMassClear={false}/> : null
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
    return this.props.boundPoint.map((point) => {
      return <Marker position={{lng: point.lng, lat: point.lat}}/>
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
      case 1:
        message.success("标定第一个点成功，请添加第二个点", 3)
        break
      case 2:
        message.success("标定第二个点成功，请添加第三个点", 3)
        break
      case 3:
        message.success("标定第三个点成功，请添加第四个点", 3)
        break
      case 4:
        message.success("标定第四个点成功，开始计算平扫路径", 3)
        this.props.dispatch(calculateRouteAction())
        break
      default:
        return
    }
  }

  renderBattery() {  //根据电量渲染电池组件
    const batteryColor = style({ //根据电量判定电池图标着色样式
      position: 'absolute',
      top: '22.5px',
      left: '119.5px',
      width: '29px',
      height: '58.6px',
      backgroundColor: '#b61313',
      borderRadius: '3px',
      borderTop: 'solid 3.1px #000000'
    }, media({minWidth: 1700},
      {
        top: '26px',
        left: '281px',
        width: '41px',
        height: '61.6px'
      }
    ))

    var batte = this.state.batts.toFixed(1)
    if (isNaN(batte)) batte = 100 
    var colors;
    var heights = window.innerWidth > 1700 ? 61.6 : 58.6
    var tops = window.innerWidth > 1700 ? 26 : 22.5

    if (batte >= 80 && batte <= 100) {
      colors = 'green'
    } else if (batte >= 50 && batte < 80) {
      colors = 'yellow'
    } else {
      colors = '#b61313'
    }

    var distance = (100 - batte) / 100  //获取应该缩小的倍率
    heights = heights - heights * distance
    tops = tops + tops * distance * 2.5

    return <div className={batteryColor} style={{backgroundColor: colors, height: heights, top: tops}}/>
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
    if (this.th) {
      clearInterval(this.th)
    }
  }

  componentDidMount() {
    this.timer= setInterval(()=>{
      var batts=this.state.batts
      batts -=0.3
      if(batts<=0){
        batts=0
        clearInterval(this.timer)
      }
      this.setState({
        batts:batts
      })
    },60000)

    setTimeout(() => {
      this.setState({
        show: false
      })
    }, 5000)
    this.props.dispatch(getFileListAction(this.props.vehicle_No))
    this.t = setInterval(() => {
      this.props.dispatch(simuDataAction())
      this.props.dispatch(findDistanceAction(this.props.workingPoint))
    }, 5000)
  }
  
}



function mapStateToProps(state: any): WorkingState {
  return state[PREFIX] as WorkingState
}

export = connect(mapStateToProps)(Main)
