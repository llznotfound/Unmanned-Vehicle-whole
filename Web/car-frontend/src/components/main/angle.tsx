import * as React from 'react'
import * as Redux from 'redux'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

// import {changeItemAction} from '../../models/main/showItem'

interface angleProps {
    angle:number //航向角
    // dispatch: Redux.Dispatch<any>
}

export default class Angle extends React.Component<angleProps , any > {
  getOption() {
      return {

        series : [
            {
                name: '',
                type: 'pie',
                // radius : '55%',
                // center: ['50%', '60%'],
                data:[
                    {value:this.props.angle},
                    {value:360 - this.props.angle}
                ],
                avoidLabelOverlap: false,
                label: {
                    normal: {
                        show: false,
                        position: 'center'
                    },
                    emphasis: {
                        show: false,
                        textStyle: {
                            fontSize: '30',
                            fontWeight: 'bold'
                        }
                    }
                },
                itemStyle: {
                    // emphasis: {
                    //     shadowBlur: 10,
                    //     shadowOffsetX: 0,
                    //     shadowColor: 'rgba(0, 0, 0, 0.5)'
                    // },
                    normal: {
                      color:'#363636',
                      borderColor:'#979797'
                    }
                },
                startAngle:180
            }
        ]
        
      }
  }

  render() {
      const wholeStyle = style(
          {
              width:'100%',
              height:'100%'
          }
      )

    

      return (
          <div className = {wholeStyle}>
            <ReactEcharts option = {this.getOption()} style = {{height: '100px',width:'120px'}}/>
          </div>
      )
  }

}