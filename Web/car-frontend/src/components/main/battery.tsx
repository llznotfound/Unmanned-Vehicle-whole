import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface batteryProps {
    battery:number //电池
}

export default class Battery extends React.Component<batteryProps , any> {
    seriesData : Array<number> = [0]
    getOption(){
        this.updateSeriesDate();
        return {
            title: {
                text: '电池',
                // subtext: '数据来自西安兰特水电测控技术有限公司',
                x: 'center',
                top:'30x'
              },
            tooltip : {
                formatter: "{a} <br/>{b} : {c}%"
            },
            toolbox: {
                show: false,
                feature: {
                    restore: {},
                    saveAsImage: {}
                }
            },
            series: this.getSeries()
        }
    }

    updateSeriesDate(){
        this.seriesData.push(this.props.battery)
        this.seriesData.shift();
    }

    getSeries(){
        return {
            name: '电池电量',
            type: 'gauge',
            detail: {formatter:'{value}%'},
            data: [{value: this.seriesData}]
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
            <div className={wholeStyle}>
              <ReactEcharts option = {this.getOption()} style={{width:'300px' , height:'350px', marginLeft:'30px'}} />
            </div>
        )
    }
}