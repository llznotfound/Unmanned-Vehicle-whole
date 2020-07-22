import * as React from 'react'
import {style , media } from 'typestyle'

import {Table} from 'antd'
// type PlanProps = PlanState
var num = 1;    //记录序号  

interface PlanProps {
    ph:number
    turbidity:number //浊度
    conductivity:number //电导率
    oxygen:number //溶氧
   
    temperature:number  //温度
    lng: number //船体位置经度
    lat: number //船体位置纬度
    theme:string
}

var showName = '任务点'
export function changeName(name){
  showName = name;
}

var dataTable = [[],[],[],[]] //对应4个选项

export default class MissionTable extends React.Component<PlanProps , any>{
    log: HTMLDivElement

    render() {
      const logStyle = style({    //给表格定框框
  
        left:'21px',
        top:'164px',
        width:'1168px',
        height:'686px',
        borderRadius:'3px',
        backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        position:'absolute',
      },media({minWidth:1700},
        {
          top:'180px',
          left:'29px',
          width:'1635px',
          height:'720px',
        }
      ))
 
      const flowStyle = style({
          overflowX: 'hidden',
          overflowY: 'auto',
          height: window.innerWidth>1700?'650px':'606px',
          width: window.innerWidth>1700?'1634px':'1365px',//为了使出现滑动条以后与表头对齐
          top:'239px',
          left:window.innerWidth>1700?'30px':'22px',
          position: 'absolute',
      })

      return (
      <div style={{height:'100%',width:'100%'}}>
          <table className={logStyle}>
              {this.getTr()}
          </table>
          <div className={flowStyle} ref={(log)=>{this.log=log}}>
          </div>
          {/* {this.updateLog()} */}
      </div>
      )
    }

    //获取表头
    getTr(){
      //时间标题         
      const timeTitle = style ({   
        position:'absolute',
        left:'82px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
       },media({minWidth:1700},
          {
            top:'15px',
            left:'180px',
            width:'440px',
            height:'24px',
            fontSize:'22px'
          }
      ))

      //坐标标题
      const corrdTitle = style ({
        position:'absolute',
        left:'220px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'left',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },media({minWidth:1700},
        {
          top:'15px',
          left:'1150px',
          width:'308px',
          height:'24px',
          fontSize:'22px'
        }
      ))
  

      //障碍点时间标题         
      const timeTitleTi = style ({   
        position:'absolute',
        left:'182px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
       },media({minWidth:1700},
          {
            top:'15px',
            left:'80px',
            width:'712.5px',
            height:'24px',
            fontSize:'22px'
          }
      ))

      //障碍点坐标标题
      const corrdTitleTi = style ({
        position:'absolute',
        left:'765px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },media({minWidth:1700},
        {
          top:'15px',
          left:'865.5px',
          width:'817.5px',
          height:'24px',
          fontSize:'22px'
        }
      ))

      //工作历史时间标题         
      const timeTitleHi = style ({   
        position:'absolute',
        left:'82px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
       },media({minWidth:1700},
          {
            top:'15px',
            left:'58px',
            width:'440px',
            height:'24px',
            fontSize:'22px'
          }
      ))

      //工作历史坐标标题
      const corrdTitleHi = style ({
        position:'absolute',
        left:'465px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },media({minWidth:1700},
        {
          top:'15px',
          left:'545px',
          width:'545px',
          height:'24px',
          fontSize:'22px'
        }
      ))
  
      //工作内容标题
      const phTitleHi = style ({
        position:'absolute',
        left:'862px',
        top:'20px',
        width:'220px',
        height:'22px',
        fontFamily:'PingFangSC',
        fontSize:'16px',
        fontWeight:'normal',
        fontStyle:'normal',
        fontStretch:'normal',
        lineHeight:'normal',
        letterSpacing:'-0.2px',
        textAlign:'center',
        color:this.props.theme == "false"?'#e3e3e3':'#000'
      },media({minWidth:1700},
        {
          top:'15px',
          left:'1245px',
          // width:'545px',
          height:'24px',
          fontSize:'22px'
        }
      ))
  
      switch(showName){
        case "任务点":return (
          <tr>
              <th className = {timeTitle} >   时间   </th>
              <th className = {corrdTitle}>  坐标   </th>
             
            </tr>
        )
    
        
        case "障碍点":return (
          <tr>
            <th className = {timeTitleTi} >   时间   </th>
            <th className = {corrdTitleTi}>  坐标   </th>
          </tr>
        )
        case "工作历史":return (
          <tr>
            <th className = {timeTitleHi} >   时间   </th>
            <th className = {corrdTitleHi}>  坐标   </th>
            <th className = {phTitleHi}>     工作内容    </th>
          </tr>
        )
      }
    }

    //更新log控件信息
    updateLog() {
      if (!this.log) {
      return
      }
      const getNum = () =>{
        switch(showName){
          case "任务点":return 0;
          case "采样":return 1;
          case "障碍点":return 2;
          case "工作历史":return 3;
        }
      }
      var num = getNum();
      this.log.innerHTML = `<table style = "border-collapse:collapse;
                                              width:${window.innerWidth>1700?'1635px':'1165px'};
                                              left:0px;
                                              position:'absolute';
                                              color:#e3e3e3;
                                              font-family:PingFangSC;
                                              fong-weight:normal;
                                              fontSize:16px;
                                              table-layout:fixed">
                            </table>`
      for(var i = 0 ; i < 30;i++){
        const temperature = (40 * Math.random()).toFixed(0)
        const time = new Date().toLocaleTimeString();
        const ph = (10 * Math.random()).toFixed(1)
        const conductivity = (700 * Math.random()).toFixed(0)
        const turbidity = (100 * Math.random()).toFixed(0)
        const oxygen = (10 * Math.random()).toFixed(0)
        const lng = 121.445967
        const lat = 31.032097
        
        var pre = { //任务点栏数据存储
          'time':time,
          'lng':lng,
          'lat':lat,
          'ph':ph,
          'oxygen':oxygen,
          'temperature':temperature,
          'turbidity':turbidity,
          'conductivity':conductivity
        }

        dataTable[0].push(pre)
        var mpre = {  //采样栏数据存储
          'time':time,
          'lng':lng,
          'lat':lat,
          'total':300,
          'times':30
        }
        
        dataTable[1].push(mpre)

        var gpre = {  //障碍点数据存储
          'time':time,
          'lng':lng,
          'lat':lat,
        }
        dataTable[2].push(gpre)

        var hpre = {
          'time':time,
          'lng':lng,
          'lat':lat,
          'contain':'对'+(1+5*Math.random()).toFixed(0)+'号瓶子进行取水'
        }
        dataTable[3].push(hpre)
      }
      

      for(var i = 0 ; i < 30;i++){
        const substring = this.log.innerHTML.substring(0, this.log.innerHTML.length-8)
        //更新表格数据
        if(num == 0){
          this.log.innerHTML = substring+`<tr style=" color:${this.props.theme == "false"?'#e3e3e3':'#000'};font-size:16px;${i%2!=0 ? 'background-color:rgba(237, 237, 237, 0.05)' : '' };height:60px  ">
                                
          <td align="center" style="padding-left:20px'">    ${dataTable[num][i].time}</td>
          <td align="center" >  经E${dataTable[num][i].lng}° <br/>纬W${this.props.lat}°   </td>
       
          </tr>
          </table>`
        } else if(num == 1){
          this.log.innerHTML = substring+`<tr style=" color:${this.props.theme == "false"?'#e3e3e3':'#000'};font-size:16px;${i%2!=0 ? 'background-color:rgba(237, 237, 237, 0.05)' : '' };height:60px  ">
                                
          <td align="center" style="padding-left:20px'">    ${dataTable[num][i].time}</td>
          <td align="center" >  经E${dataTable[num][i].lng}° <br/>纬W${dataTable[num][i].lat}°   </td>
          <td align="center" >   ${dataTable[num][i].total}              </td>
          <td align="center" >  ${dataTable[num][i].times} </td>
          </tr>
          </table>`
        } else if(num == 2){
          this.log.innerHTML = substring+`<tr style=" color:${this.props.theme == "false"?'#e3e3e3':'#000'};font-size:16px;${i%2!=0 ? 'background-color:rgba(237, 237, 237, 0.05)' : '' };height:60px  ">
                                
          <td align="center" style="padding-left:20px'">    ${dataTable[num][i].time}</td>
          <td align="center" >  经E${dataTable[num][i].lng}° <br/>纬W${dataTable[num][i].lat}°   </td>
          </tr>
          </table>`
        } else {
          this.log.innerHTML = substring+`<tr style=" color:${this.props.theme == "false"?'#e3e3e3':'#000'};font-size:16px;${i%2!=0 ? 'background-color:rgba(237, 237, 237, 0.05)' : '' };height:60px  ">
                                
          <td align="center" style="padding-left:20px'">    ${dataTable[num][i].time}</td>
          <td align="center" >  经E${dataTable[num][i].lng}° <br/>纬W${dataTable[num][i].lat}°   </td>
          <td align="center" >   ${dataTable[num][i].contain}              </td>
          </tr>
          </table>`
        }
        
        // this.log.scrollTop = this.log.scrollHeight
      }
      
    }

    componentDidUpdate(){
      this.updateLog()
    }
    componentDidMount() {
        this.updateLog()
        // setInterval(() => {
        // this.updateLog()
        // num++
        // }, 1000)
    }
}
