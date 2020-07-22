import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface speedProps {
    speed:number //速度
}


export default class Speed extends React.Component<speedProps , any> {

    getOption(){
        return {
            tooltip : {
                formatter: "{a} <br/>{b} : {c}m/s"
              },
        
              series: [
                {
                //   top:'-10px',
                //   radius: '70%',
                  axisLine: {  
                    lineStyle: {  
                        width: 5 // 这个是修改宽度的属性  
                    }  
                },  
                axisLabel: {
                  show: true,
                  textStyle: {
                      color: '#000',
                      fontSize: 20,
                  },
                  distance: -60,
              },
                  splitNumber: 5,
                  min: 0,
                  max: 5,
                  type: 'gauge',
                  title: {
                    offsetCenter: [0, '-60%'],
                    textStyle: {
                      color: '#0F0F0F',
                      fontSize: 20
                    }
                  },
                  detail: {
                    formatter:`{value}` + '(m/s)',
                    offsetCenter: [0, '50%'],
                    textStyle: {
                      color: 'black',
                      fontSize: 33
                    }
                  },
                  data: [{value: this.props.speed.toFixed(2) , name:'速度'}],
        
                 
                  
                    }
                  
                
              ]
        }
    }





    render(){
        const wholeStyle = style(
            {
                width:'100%',
                height:'100%'
            }
        )

        return (
            <div className = {wholeStyle}>
              <ReactEcharts option = {this.getOption()} style = {{width:'350px',height:'300px',marginLeft:'30px'}} />
            </div>
        )
    }

}