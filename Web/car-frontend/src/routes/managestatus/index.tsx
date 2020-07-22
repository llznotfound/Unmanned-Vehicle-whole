import * as React from 'react'
import { connect } from 'dva'
import * as Redux from 'redux'
import { style, media } from 'typestyle'

import 'antd/dist/antd.css'

import InfoTable from '../../components/managestatus/table'
import {Button} from 'antd'
import {changeName} from '../../components/managestatus/table'

import { PREFIX, ManageState } from '../../models/managestatus/index'
import { ThemeState , NameState} from '../../models/managestatus/index'
import {  updateThemeAction,updateNameAction, updateDataAction, } from '../../models/managestatus/managestatus'
declare function require(path:string):any 

type ThemeProps = ThemeState
type NameProps = NameState
interface SliderProps {
  dispatch: Redux.Dispatch<any>
}
type MainProps = ManageState

interface MainDispatch {
  dispatch: Redux.Dispatch<any>
}



class Main extends React.Component<MainProps & MainDispatch &SliderProps & ThemeProps & NameProps, any> {
  
  th:any //存储主题

  render(){
    const wholeStyle = style ({     //背景  
      height:'100vh',
      // width:'1800px',
      backgroundColor:this.props.theme == "false"?'#424250':'#e6e6e6',
    })

    //任务点记录栏
    const taskRecord = style ({ //任务栏框架
      left:'24px',
      top:'25px',
      width:'170px',
      height:'124px',
      borderRadius:'3px',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      position:'absolute',
      
    },
    media({minWidth:1700},
      {
        left:'34px',
        top:'28px',
        width:'238px',
        height:'136px',
      }
    ))


    const taskTxt = style ({  //任务状态栏目文字属性
      position:'absolute',
      left:'0px',
      top:'10px',
      width:'170px',
      height:'26px',
      fontFamily:'PingFangSC',
      fontSize:'16px',
      fontWeight:600,
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:1.63,
      letterSpacing:'0.6px',
      textAlign:'center',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'11px',
        width:'238px',
        height:'29px',
        fontSize:'22px',
        fontWeight:840,
        lineHeight:1.79
      }
    ))

    const lineBetween = style ({  //分割线
      left:'0px', 
      position:'absolute',
      width:'170px',
      height:'1px',
      opacity:0.38,
      background:'#297fca',
      top:'46px'
    },media({minWidth:1700},
      {
        width:'238px',
        height:'1.1px',
        top:'51px'
      }
    ))

    const recordAmountTxt = style ({ //记录条数文字属性
      position:'absolute',
      top:'56px',
      left:'35px',
      width:'100px',
      height:'56px',
      fontFamily:'Impact',
      fontSize:'46px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'62px',
        left:'49px',
        width:'140px',
        height:'62px',
        fontSize:'64px',

      }
    )) 

    const recordAmountTxt1 = style ({ //记录条数文字属性1
      position:'absolute',
      left:'115px',
      top:'66px',
     
      fontFamily:'PingFangSC',
      fontSize:'26px',
      fontWeight:600,
      color:this.props.theme == "false"?'#e3e3e3':'#000',
      width:'100px',
      height:'56px',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
    },media({minWidth:1700},
      {
        left:'161px',
        fontSize:'36px',
        fontWeight:840,
        width:'140px',
        height:'62px',
        top:'76px',
      }
    )) 


    const sampleTxt = style ({  //采样栏目文字属性
      position:'absolute',
      left:'0px',
      top:'10px',
      width:'170px',
      height:'26px',
      fontFamily:'PingFangSC',
      fontSize:'16px',
      fontWeight:600,
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:1.63,
      letterSpacing:'0.6px',
      textAlign:'center',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'11px',
        width:'238px',
        height:'29px',
        fontSize:'22px',
        fontWeight:840,
        lineHeight:1.79
      }
    ))

    const lineBetween1 = style ({  //分割线
      left:'0px', 
      position:'absolute',
      width:'170px',
      height:'1px',
      opacity:0.38,
      background:'#297fca',
      top:'46px'
    },media({minWidth:1700},
      {
        width:'238px',
        height:'1.1px',
        top:'51px'
      }
    ))

    const sampleAmountTxt = style ({ //记录条数文字属性
      position:'absolute',
      top:'56px',
      left:'35px',
      width:'100px',
      height:'56px',
      fontFamily:'Impact',
      fontSize:'46px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'62px',
        left:'49px',
        width:'140px',
        height:'62px',
        fontSize:'64px'
      }
    )) 

    const sampleAmountTxt1 = style ({ //记录条数文字属性1
      position:'absolute',
      left:'115px',
      top:'66px',
     
      fontFamily:'PingFangSC',
      fontSize:'26px',
      fontWeight:600,
      color:this.props.theme == "false"?'#e3e3e3':'#000',
      width:'100px',
      height:'56px',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
    },media({minWidth:1700},
      {
        left:'161px',
        width:'140px',
        height:'62px',
        fontSize:'36px',
        top:'76px',
      }
    )) 

    //障碍点记录
    const barrierRecord = style ({ //障碍点栏框架
      left:'401px',
      top:'25px',
      width:'170px',
      height:'124px',
      borderRadius:'3px',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      position:'absolute',
      
    },media({minWidth:1700},
      {
        top:'28px',
        left:'294px',
        width:'238px',
        height:'136px',
      }
    ))

    const barrierTxt = style ({  //障碍点栏目文字属性
      position:'absolute',
      left:'0px',
      top:'10px',
      width:'170px',
      height:'26px',
      fontFamily:'PingFangSC',
      fontSize:'16px',
      fontWeight:600,
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:1.63,
      letterSpacing:'0.6px',
      textAlign:'center',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'11px',
        width:'238px',
        height:'29px',
        fontSize:'22px',
        fontWeight:840,
        lineHeight:1.79
      }
    ))

    const lineBetween2 = style ({  //分割线
      left:'0px', 
      position:'absolute',
      width:'170px',
      height:'1px',
      opacity:0.38,
      background:'#297fca',
      top:'46px'
    },media({minWidth:1700},
      {
        top:'51px',
        width:'238px',
        height:'1.1px',
      }
    ))

    const barrierAmountTxt = style ({ //记录条数文字属性
      position:'absolute',
      top:'56px',
      left:'35px',
      width:'100px',
      height:'56px',
      fontFamily:'Impact',
      fontSize:'46px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'62px',
        left:'49px',
        width:'140px',
        height:'62px',
        fontSize:'64px'
      }
    )) 

    const barrierAmountTxt1 = style ({ //记录条数文字属性1
      position:'absolute',
      left:'115px',
      top:'66px',
     
      fontFamily:'PingFangSC',
      fontSize:'26px',
      fontWeight:600,
      color:this.props.theme == "false"?'#e3e3e3':'#000',
      width:'100px',
      height:'56px',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
    },media({minWidth:1700},
      {
        left:'161px',
        width:'140px',
        height:'62px',
        fontSize:'36px',
        top:'76px',
      }
    )) 

    //工作历史记录
    const workHistoryRecord = style ({ //工作历史栏框架
      left:'586px',
      top:'25px',
      width:'170px',
      height:'124px',
      borderRadius:'3px',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      position:'absolute',
      
    },media({minWidth:1700},
      {
        top:'28px',
        left:'562px',
        width:'238px',
        height:'136px',
      }
    ))

    const workHistoryTxt = style ({  //工作历史栏目文字属性
      position:'absolute',
      left:'0px',
      top:'10px',
      width:'170px',
      height:'26px',
      fontFamily:'PingFangSC',
      fontSize:'16px',
      fontWeight:600,
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:1.63,
      letterSpacing:'0.6px',
      textAlign:'center',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'11px',
        width:'238px',
        height:'29px',
        fontSize:'22px',
        fontWeight:840,
        lineHeight:1.79
      }
    ))

    const lineBetween3 = style ({  //分割线
      left:'0px', 
      position:'absolute',
      width:'170px',
      height:'1px',
      opacity:0.38,
      background:'#297fca',
      top:'46px'
    },media({minWidth:1700},
      {
        top:'51px',
        width:'238px',
        height:'1.1px',
      }
    ))

    const workHistoryAmountTxt = style ({ //记录条数文字属性
      position:'absolute',
      top:'56px',
      left:'35px',
      width:'100px',
      height:'56px',
      fontFamily:'Impact',
      fontSize:'46px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'62px',
        left:'49px',
        width:'140px',
        height:'62px',
        fontSize:'64px'
      }
    )) 

    const workHistoryAmountTxt1 = style ({ //记录条数文字属性1
      position:'absolute',
      left:'115px',
      top:'66px',
     
      fontFamily:'PingFangSC',
      fontSize:'26px',
      fontWeight:600,
      color:this.props.theme == "false"?'#e3e3e3':'#000',
      width:'100px',
      height:'56px',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
    },media({minWidth:1700},
      {
        left:'161px',
        width:'140px',
        height:'62px',
        fontSize:'36px',
        top:'76px',
      }
    )) 

    //任务点0记录栏
    const taskRecord0 = style ({ //任务栏0框架
      left:'777px',
      top:'25px',
      width:'170px',
      height:'124px',
      borderRadius:'3px',
      backgroundColor:this.props.theme == "false"?'#363636':'#fff',
      position:'absolute',
     
    },media({minWidth:1700},
      {
        top:'28px',
        left:'850px',
        width:'238px',
        height:'136px',
      }
    ))

    const taskTxt0 = style ({  //任务状态栏0目文字属性
      position:'absolute',
      left:'0px',
      top:'10px',
      width:'170px',
      height:'26px',
      fontFamily:'PingFangSC',
      fontSize:'16px',
      fontWeight:600,
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:1.63,
      letterSpacing:'0.6px',
      textAlign:'center',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'11px',
        width:'238px',
        height:'29px',
        fontSize:'22px',
        fontWeight:840,
        lineHeight:1.79
      }
    ))

    const lineBetween0 = style ({  //分割线
      left:'0px', 
      position:'absolute',
      width:'170px',
      height:'1px',
      opacity:0.38,
      background:'#297fca',
      top:'46px'
    },media({minWidth:1700},
      {
        top:'51px',
        width:'238px',
        height:'1.1px',
      }
    ))

    const recordAmountTxt0 = style ({ //记录条数文字属性
      position:'absolute',
      top:'56px',
      left:'35px',
      width:'100px',
      height:'56px',
      fontFamily:'Impact',
      fontSize:'46px',
      fontWeight:'normal',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
      color:this.props.theme == "false"?'#e3e3e3':'#000'
    },media({minWidth:1700},
      {
        top:'62px',
        left:'49px',
        width:'140px',
        height:'62px',
        fontSize:'64px'
      }
    )) 

    const recordAmountTxt10 = style ({ //记录条数文字属性1
      position:'absolute',
      left:'115px',
      top:'66px',
     
      fontFamily:'PingFangSC',
      fontSize:'26px',
      fontWeight:600,
      color:this.props.theme == "false"?'#e3e3e3':'#000',
      width:'100px',
      height:'56px',
      fontStyle:'normal',
      fontStretch:'normal',
      lineHeight:'normal',
      letterSpacing:'2.1px',
      textAlign:'left',
    },media({minWidth:1700},
      {
        left:'161px',
        width:'140px',
        height:'62px',
        fontSize:'36px',
        top:'76px',
      }
    )) 

    return (
      <div className = {wholeStyle}>
       
        {/*任务点记录栏 */}
        <Button className = {taskRecord} type = "primary" onClick = {() => {this.changeList("任务点")}}>  
        <div className = {taskTxt}>任务点记录</div>
        <div className = {lineBetween}  />
        <div className = {recordAmountTxt}>30</div>  
        <div className = {recordAmountTxt1}>条</div>     
       </Button>


       {/*障碍点记录栏 */}
        <Button className = {barrierRecord} type = "primary" onClick = {() => {this.changeList("障碍点")}}>
        <div className = {barrierTxt}>障碍点记录</div>
        <div className = {lineBetween2} />
        <div className = {barrierAmountTxt}>30</div>  
        <div className = {barrierAmountTxt1}>条</div>
       </Button>

       {/*工作历史记录栏 */}
        <Button className = {workHistoryRecord} type = "primary" onClick = {() => {this.changeList("工作历史")}}>
        <div className = {workHistoryTxt}>工作历史记录</div>
        <div className = {lineBetween3} />
        <div className = {workHistoryAmountTxt}>30</div>  
        <div className = {workHistoryAmountTxt1}>条</div>
       </Button>

       {/*任务点0记录栏 */}
       <Button className = {taskRecord0} type = "primary" onClick = {() => {this.changeList("任务点")}}>  
        <div className = {taskTxt0}>任务点记录</div>
        <div className = {lineBetween0} />
        <div className = {recordAmountTxt0}>30</div>  
        <div className = {recordAmountTxt10}>条</div>     
       </Button>

        <InfoTable   ph = {this.props.ph}  
                    oxygen = {this.props.oxygen}
                    temperature = {this.props.temperature} 
                    turbidity = {this.props.turbidity}
                    conductivity = {this.props.conductivity} 
                    lng = {this.props.lng}
                    lat = {this.props.lat}
                    theme = {this.props.theme}/> 

      </div>
    )

  }

  changeList(name){
    if(name == this.props.name)return;
    changeName(name)
    this.props.dispatch(updateNameAction(name))
  }

  componentWillUnmount() {
    if(this.th){
      clearInterval(this.th)
    }
  }
  
  componentDidMount() {
    
    this.th = setInterval(() => {
      this.props.dispatch(updateThemeAction(sessionStorage.getItem('theme')))
    },100)
  }
    
}





function StateToProps(state: any): ManageState {
  return state[PREFIX] as ManageState
}

export = connect(StateToProps)(Main)
