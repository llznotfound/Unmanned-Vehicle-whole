import * as React from 'react'
import * as Redux from 'redux'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

// import {changeItemAction} from '../../models/main/showItem'

interface angleProps {
    
}

export default class SaveSpace extends React.Component<angleProps , any > {
  getOption() {
      return {

        series: [
            {
                name:'',
                type:'pie',
                radius: ['50%', '70%'],
                avoidLabelOverlap: false,
                // label: {
                //     normal: {
                //         show: false,
                //         position: 'center'
                //     },
                //     emphasis: {
                //         show: true,
                //         textStyle: {
                //             fontSize: '30',
                //             fontWeight: 'bold'
                //         }
                //     }
                // },
                labelLine: {
                    normal: {
                        show: false
                    }
                },
                data:[
                    {value:25},
                    {value:75}
                ],
                itemStyle:{
                    normal: {
                        color: function(params) {
                        　//首先定义一个数组
                          var colorList = [
                            '#289bff','#808087'
                          ];
                          var turnnumber = 0
                          turnnumber++;
                          return colorList[params.dataIndex % (12 + turnnumber)]
                        },
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