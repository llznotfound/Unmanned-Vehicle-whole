import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface radiusProps {
    radius:number //半径预警
}

// var data = [];

// for (var i = 0; i <= 100; i++) {
//     var theta = i / 100 * 360;
//     var r = 5 * (1 + Math.sin(theta / 180 * Math.PI));
//     data.push([r, theta]);
// }


export default class Radius extends React.Component<radiusProps , any> {
    seriesData : Array<Array<number>> = [[0,0]]
    getOption(){
        this.updateSeriesData();
        return {
            title: {
                text: '工作半径:' + this.props.radius.toFixed(2) + 'km',
                x:'center'
            },
            // legend: {
            //     data: ['line'],
            //     x:'left'
            // },
            polar: {},
            tooltip: {
                trigger: 'axis',
                axisPointer: {
                    type: 'cross'
                },
                formatter:function(a){
                    // console.log(a)
                    return ('工作半径 : ' + a[0].data[0])
                },
            },
            angleAxis: {
                type: 'value',
                startAngle: 0,
                splitLine:{show: false},//去除网格线
                // splitArea : {show : true}//保留网格区域
                axisTick: {
                    show: false
                }
            },
            radiusAxis: {
                splitLine:{show: false},//去除网格线
                // splitArea : {show : true}//保留网格区域
                axisTick: {
                    show: false
                }
            },
            series: this.getSeries()
        }
    }

    updateSeriesData(){
        // var thema = this.props.radius / 100 * 360;
        // // var r = 5 * ( 1 + Math.sin(thema / 180 * Math.PI))
        // this.seriesData.push([this.props.radius , thema]);
        // this.seriesData.shift()
        // console.log(this.seriesData)
        // this.seriesData.shift();
        // this.seriesData.reverse();
        this.seriesData.splice(0,this.seriesData.length)
        for(var i = 0;i <= 100;i++){
            this.seriesData.push([this.props.radius ,i/100 * 360])
        }
    }

    getSeries(){
        return {
            coordinateSystem: 'polar',
            name: 'line',
            type: 'line',
            data: this.seriesData,
            symbol:'none'
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
            <div className = {wholeStyle} >
              <ReactEcharts option = {this.getOption()} style = {{height:'350px' , width:'300px' , marginLeft: '30px'}} />
            </div>
        )
    }

}