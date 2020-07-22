import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface CommunicateProps {
  communicate:number //通信链路质量
}

export default class Communicate extends React.Component<CommunicateProps , any> {
  seriesDate : Array<number> = [0]
  getOption() {
    this.updateSeriesData();
    return {
      title: {
        text: '通信链路质量',
        // subtext: '数据来自西安兰特水电测控技术有限公司',
        x: 'center'
      },
      tooltip : {
        formatter: "{a} <br/>{b} : {c}"
    },
    toolbox: {
      show:false,
        feature: {
            restore: {},
            saveAsImage: {}
        }
    },
    series:this.getSeries()
    }
  }

  updateSeriesData(){
    this.seriesDate.push(this.props.communicate)
    this.seriesDate.shift();
  }

  getSeries(){
    return {
      name: '通信链路质量',
      type: 'gauge',
      detail: {formatter:'{value}'},
      data: [{value: this.props.communicate, name: 'dBm'}]
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
        <ReactEcharts option={this.getOption()} style={{height: '300px',width:'350px', marginLeft:'30px' }} />
      </div>
    )
  }
}
  
