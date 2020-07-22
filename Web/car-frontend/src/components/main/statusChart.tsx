import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface StatusChartProps {
  speed: number//速度
  communicate:number//通信链路质量
  battery:number//电池
  radius:number//半径警告
  item:string //需要显示的信息项
}

//仿echarts雨量流量关系图组件
export default class StatusChart extends React.Component<StatusChartProps, any> {
  seriesData: Array<Array<number>> = [[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]]//系列数据的初始化
  getOption() {
    this.updateSeriesData();
    return {
      grid:
        {
          top:'40px', 
          left:'50px',
          width:window.innerWidth>3800?'78%':'93%',
          height:'80%',
          show: false
        },
      tooltip: {  
        trigger: 'axis',
        formatter: '{c0}',
        position: function (point, params, dom, rect, size) {
          return [point[0]+10,point[1]+10];
      },
        backgroundColor:'white',
        borderColor:'#00E5B9',
        borderWidth:'3px',
        textStyle: {
          fontSize: 12,
          color:'#00E5B9',
        }
      },
      color:['#00E5BB'],
      xAxis:
        {
          //show:false,
          boundaryGap: false,
          type: 'category',
          data: this.updateXAxis(),
          axisTick:{
            show:false
          },
          axisLabel:{
            show:false
          },
          axisLine:{
            lineStyle:{
              width:3,
              color:'#00E5BB'
            }
          }
        },  
      yAxis: this.getYAXis(),          
      series: this.getSeries()
    }
  }
  //更新系列的数据
  updateSeriesData(){
    this.seriesData[0].push(-this.props.communicate);
    this.seriesData[1].push(this.props.speed);
    this.seriesData[2].push(this.props.battery);
    this.seriesData[3].push(this.props.radius);
    this.seriesData[0].shift();
    this.seriesData[1].shift();
    this.seriesData[2].shift();
    this.seriesData[3].shift();
  }
  //更新时间轴
  updateXAxis(): any{
    let now = new Date();
    let res = [];
    let len = 10;
    while (len--) {
      res.unshift(now.toLocaleTimeString().replace('PM'||'AM','\nPM'||'\nAM'));//.replace(/^\D*/,'')
      now = new Date(now.valueOf() - 3000);
    }
    return res;
  }
  getYAXis():any{
    switch(this.props.item){
      case "communicate":
      return {
        name:'(-dBm)',
        nameTextStyle:{
          fontSize:12, 
          color: '#B1B1B1',      
        },
        nameGap:15,
        offset:30,
        type: 'value',
        max: 130,
        min: 50,
        interval: 20,
        axisLabel:{
          fontSize:12,            
          margin: -10,
          verticalAlign: 'center',
          color:'#B1B1B1',  
        },
        axisLine:{
          show:false
        },
        axisTick:{
          show: false
        }
      };
      case "speed":
      return {
        name:'(m/s)',
        nameTextStyle:{
          fontSize:12, 
          color: '#B1B1B1',      
        },
        nameGap:15,
        offset:30,
        type: 'value',
        max: 5,
        min: 0,
        interval: 1,
        axisLabel:{
          fontSize:12,            
          margin: -10,
          verticalAlign: 'center',
          color:'#B1B1B1',  
        },
        axisLine:{
          show:false
        },
        axisTick:{
          show: false
        }
      };
      case "battery":
      return {
        name:'(%)',
        nameTextStyle:{
          fontSize:12, 
          color: '#B1B1B1',      
        },
        nameGap:15,
        offset:30,
        type: 'value',
        max: 100,
        min: 0,
        interval: 20,
        axisLabel:{
          fontSize:12,            
          margin: -10,
          verticalAlign: 'center',
          color:'#B1B1B1',  
        },
        axisLine:{
          show:false
        },
        axisTick:{
          show: false
        }
      };
      case "radius":
      return {
        name:'(km)',
        nameTextStyle:{
          fontSize:12, 
          color: '#B1B1B1',      
        },
        nameGap:15,
        offset:30,
        type: 'value',
        max: 10,
        min: 0,
        interval: 2,
        axisLabel:{
          fontSize:12,            
          margin: -10,
          verticalAlign: 'center',
          color:'#B1B1B1',  
        },
        axisLine:{
          show:false
        },
        axisTick:{
          show: false
        }
      };
    }
  }
  getSeries():any{
    switch(this.props.item){
      case "communicate":
      return {
        name: '通信链路质量（-dBm）',
        type:'line',
        data: this.seriesData[0],
        symbol:'circle',
        symbolSize:12,
        showAllSymbol:true,
        areaStyle: {
          normal: {
            opacity:0.8,
            color: {
              type: 'linear',
              x: 0,
              y: 0,
              x2: 0,
              y2: 1,
              colorStops: [{
                  offset: 0.5, color: '#00F1D0' 
              }, {
                  offset: 1, color: '#8DFCED' 
              }],
              globalCoord: false 
          }
          }
        },
        lineStyle: {
          normal: {
            width: 3
          }
        }
      }
      case "speed":
      return{
        name: '速度（km/h）',
        type:'line',
        data: this.seriesData[1],
        symbol:'circle',
        symbolSize:12,
        showAllSymbol:true,
        areaStyle: {
          normal: {
            opacity:0.8,
            color: {
              type: 'linear',
              x: 0,
              y: 0,
              x2: 0,
              y2: 1,
              colorStops: [{
                  offset: 0.5, color: '#00F1D0' 
              }, {
                  offset: 1, color: '#8DFCED' 
              }],
              globalCoord: false 
          }
          }
        },
        lineStyle: {
          normal: {
            width: 3
          }
        }
      }
      case "battery":
      return{
        name: '电池（%）',
        type:'line',
        data: this.seriesData[2],
        symbol:'circle',
        symbolSize:12,
        showAllSymbol:true,
        areaStyle: {
          normal: {
            opacity:0.8,
            color: {
              type: 'linear',
              x: 0,
              y: 0,
              x2: 0,
              y2: 1,
              colorStops: [{
                  offset: 0.5, color: '#00F1D0' 
              }, {
                  offset: 1, color: '#8DFCED' 
              }],
              globalCoord: false 
          }
          }
        },
        lineStyle: {
          normal: {
            width: 3
          }
        }
      }
      case "radius":
      return{
        name: '半径警告（km）',
        type:'line',
        data: this.seriesData[3],
        symbol:'circle',
        symbolSize:12,
        showAllSymbol:true,
        areaStyle: {
          normal: {
            opacity:0.8,
            color: {
              type: 'linear',
              x: 0,
              y: 0,
              x2: 0,
              y2: 1,
              colorStops: [{
                  offset: 0.5, color: '#00F1D0' 
              }, {
                  offset: 1, color: '#8DFCED' 
              }],
              globalCoord: false 
          }
          }
        },
        lineStyle: {
          normal: {
            width: 3
          }
        },
      }
      }
    }
  
  render() {
    const wholeStyle = style(
      {
        height: '100%',
        width: '100%'
      }
    )
    return (
      <div className={wholeStyle}>
        <ReactEcharts option={this.getOption()} style={{height:'270px', width:'1000px'}}/>
      </div>
    )
  }
}
