import * as React from 'react'
import {style,media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface timeProps {
    time:number //预估时间
}

export default class Time extends React.Component<timeProps , any >{ 
    serierData : Array<number> = [0]
    getOption(){
        this.updateSeriesData();
        return {
            title:{
                text:'预估时间\n'
                + this.serierData[0] + 'min',
                x:'center'
            },
            tooltip: { 
                // formatter: "{a}：{c}"
                // backgroundColor: '#fff',
                borderColor: '#f60',
                borderWidth: '1px',
                textStyle: {
                    color: '#333'
                },
                formatter: function(param) {
                    var time = Math.floor(param.value);
                    if(param.seriesIndex === 0){
                      return '<em style="color:' + param.color + ';">剩余小时:' + time + '</em>' 
                    }
                    if(param.seriesIndex === 1){
                      return '<em style="color:' + param.color + ';">剩余分钟:' + time + '</em>' 
                    }
                    if(param.seriesIndex === 2){
                      return '<em style="color:' + param.color + ';">剩余秒:' + time + '</em>' 
                    }
                    
                }
            },
            // backgroundColor: "rgba(0,0,0,0.1)",
            series: [
                this.getSeriesHours(),
                this.getSeriesMinutes()
            ]
        }
    }

    updateSeriesData(){
        this.serierData.push(this.props.time)
        this.serierData.shift();
    }

    getSeriesHours(){
        return {
            ///////////////////////////////////////////////大表盘时针
            name: '小时',
            type: 'gauge',
            radius: '90%', //仪表盘半径
            min: 0,
            max: 12,
            startAngle: 90,
            endAngle: -269.9999,
            splitNumber: 12,
            animation: 0,
            pointer: { //仪表盘指针
                length: '70%',
                width: 6
            },
            itemStyle: { //仪表盘指针样式
                normal: {
                    color: '#109A39',
                    shadowColor: 'rgba(0, 0, 0, 0.5)',
                    shadowBlur: 10,
                    shadowOffsetX: 2,
                    shadowOffsetY: 2
                }
            },
            axisLine: { //仪表盘轴线样式 
                show: 0,
                lineStyle: {
                    color: [
                        [1, '#337ab7']
                    ],
                    width: 10,
                    shadowColor: 'rgba(0, 0, 0, 0.8)',
                    shadowBlur: 12,
                    shadowOffsetX: 3,
                    shadowOffsetY: 3
                }
            },
            splitLine: { //分割线样式 
                length: 10,
                lineStyle: {
                    width: 2
                }
            },
            axisTick: { //仪表盘刻度样式
                show: true,
                splitNumber: 5, //分隔线之间分割的刻度数
                length: 5, //刻度线长
                lineStyle: {
                    color: ['#ffffff']
                }
            },
            axisLabel: {
                show: 0
            }, //刻度标签
            title: {
                show: 0
            }, //仪表盘标题
            detail: {
                show: 0
            }, //仪表盘显示数据
            data: [this.serierData[0] / 60]
        }
    }

    getSeriesMinutes(){
        return {
            ///////////////////////////////////////////////大表盘分针
            name: '分钟',
            type: 'gauge',
            radius: '90%', //仪表盘半径
            min: 0,
            max: 60,
            startAngle: 90,
            endAngle: -269.9999,
            splitNumber: 12,
            animation: 0,
            pointer: { //仪表盘指针
                length: '85%',
                width: 6
            },
            itemStyle: { //仪表盘指针样式
                normal: {
                    color: '#ca8622',
                    shadowColor: 'rgba(0, 0, 0, 0.5)',
                    shadowBlur: 10,
                    shadowOffsetX: 2,
                    shadowOffsetY: 2
                }
            },
            axisLine: { //仪表盘轴线样式 
                show: 0,
                lineStyle: {
                    width: 0
                }
            },
            splitLine: { //分割线样式 
                length: 10,
                lineStyle: {
                    width: 2
                }
            },
            axisTick: { //仪表盘刻度样式
                show: true,
                splitNumber: 5, //分隔线之间分割的刻度数
                length: 5, //刻度线长
                lineStyle: {
                    color: ['#ffffff']
                }
            },
            axisLabel: {
                show: 0
            }, //刻度标签
            title: {
                show: 0
            }, //仪表盘标题
            detail: {
                show: 0
            }, //仪表盘显示数据
            data: [this.serierData[0] % 60]
        }
    }

    getSeriesSeconds(){
        return {
            ///////////////////////////////////////////////大表盘秒针
            name: '秒',
            type: 'gauge',
            radius: '90%', //仪表盘半径
            min: 0,
            max: 60,
            startAngle: 90,
            endAngle: -269.9999,
            splitNumber: 12,
            animation: 0,
            pointer: { //仪表盘指针
                show: true,
                length: '95%',
                width: 4
            },
            itemStyle: { //仪表盘指针样式
                normal: {
                    color: '#00b0b0',
                    shadowColor: 'rgba(0, 0, 0, 0.8)',
                    shadowBlur: 10,
                    shadowOffsetX: 4,
                    shadowOffsetY: 4
                }
            },
            axisLine: { //仪表盘轴线样式 
                lineStyle: {
                    color: [
                        [1, '#337ab7']
                    ],
                    width: 10
                }
            },
            splitLine: { //分割线样式 
                length: 10,
                lineStyle: {
                    width: 2
                }
            },
            axisTick: { //仪表盘刻度样式
                show: 1,
                splitNumber: 5, //分隔线之间分割的刻度数
                length: 5, //刻度线长
                lineStyle: {
                    color: ['#fff']
                }
            },
            axisLabel: { //刻度标签
                show: 1,
                distance: 6, //标签与刻度线的距离
                textStyle: {
                    fontWeight: 'bold',
                    fontSize: 16
                },
                formatter: function(t) {
                    switch (t + '') {
                        case '0':
                            return '';
                        case '5':
                            return '1';
                        case '10':
                            return '2';
                        case '15':
                            return '3';
                        case '20':
                            return '4';
                        case '25':
                            return '5';
                        case '30':
                            return '6';
                        case '35':
                            return '7';
                        case '40':
                            return '8';
                        case '45':
                            return '9';
                        case '50':
                            return '10';
                        case '55':
                            return '11';
                        case '60':
                            return '12';
                    }
                }
            },
            title: {
                show: 0
            }, //仪表盘标题
            detail: { //仪表盘显示数据
                show: 0,
                formatter: '{value}',
                offsetCenter: [0, '60%']
            },
            data: [{}]
        }
    }

    render(){
        const wholeStyle = style(
            {
                width:'100%',
                height:'100%'
            }
        )

        const charStyle = style(
            {
                // height:'350px',
                width:'300px',
                fontSize:'100px',
                color:'blue'
            }
        )

        return (
            <div className = {wholeStyle} >
              {/* <div className = {charStyle} >{this.props.time}min</div> */}
              <ReactEcharts option = {this.getOption()} style = {{width:'300px' , height:'350px', marginLeft:'30px' }} />
            </div>
        )
    }
}