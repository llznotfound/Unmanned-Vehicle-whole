import * as React from 'react'
import * as Redux from 'redux'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'


interface GaugeProps {
  min: number
  max: number
  value: number
  name: string
  unit:string
}

export default class Gauge extends React.Component<GaugeProps, any> {

  div: HTMLDivElement

  getOption() {
    return {
      series: [
        {
          radius: '85%',
          splitNumber: 5,
          min: this.props.min,
          max: this.props.max,
          type: 'gauge',
          title: {
            offsetCenter: [0, '45%'],
            textStyle: {
              color: 'white',
              fontSize: 14
            }
          },
          detail: {
            formatter:`{value}`,
            offsetCenter: [0, '90%'],
            textStyle: {
              color: 'white',
              fontSize: 18
            }
          },
          data: [{value: this.props.value, name: this.props.name+'\n'+this.props.unit}],
          axisTick: {
            lineStyle: {
              color: 'white',
              width: 2
            }
          },
          axisLabel: {
            show: true
          },
          axisLine: {
            lineStyle: {
              color: [[0.2, 'white'], [0.4, 'white'], [0.6, 'white'],[0.8, 'white'], [1, 'white']],
              width: 3
            }
          },
          splitLine: {
            length: 5,
            lineStyle: {
              color: 'white',
              width: 2.5
            }
          },
          pointer: {
            length: '75%',
            width: 3
          },
          itemStyle: {
            normal: {
              color: "white"
            }
          }
        }
      ]
    }
  }


  render() {
    const wholeStyle = style(
      {
        height: '100%',
        width: '100%',
        margin: 'auto'
      }
    )

    return (
      <div ref={(div)=>{this.div=div}} className={wholeStyle}>
        {}
        <ReactEcharts option={this.getOption()} style={{height: '130px',width:'130px'}} />
      </div>
    )
  }
}
