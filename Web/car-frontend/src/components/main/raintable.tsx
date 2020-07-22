import * as React from 'react'
import {style , media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'
import echarts from 'echarts'

interface rainProps {
    communicate:number //链路质量
    speed:number //速度
}

function getTime (timenum){
    var now = new Date();
    var res = [];
    var len = timenum;
    res.push(now.toLocaleTimeString());
    return res;
}

export default class RainTable extends React.Component<rainProps , any > {
    seriesData : Array<Array<number>> = [[0],[0],[0],[0]]
    timeNumber : Array<any> = [0]
    timenum = 0;
    getOption(){
        this.updateSeriesData()
        return {
            title: {
                // text: '通信链路质量 && 速度 关系图',
                // subtext: '数据来自西安兰特水电测控技术有限公司',
                // x: 'center'
            },
            tooltip: {
                trigger: 'axis',
                axisPointer: {
                    animation: false
                }
            },
            legend: {
                data:[{
                    name:'链路质量',
                    textStyle:{
                        fontSize : 20
                    }
                },{
                    name:'误码率',
                    textStyle:{
                        fontSize : 20
                    }
                },{
                    name:'速度',
                    textStyle:{
                        fontSize : 20
                    }
                },{
                    name:'加速度',
                    textStyle:{
                        fontSize : 20
                    }
                }],
                x: 'center',
            },
            toolbox: {
                show:false,
                feature: {
                    dataZoom: {
                        yAxisIndex: 'none'
                    },
                    restore: {},
                    saveAsImage: {}
                }
            },
            axisPointer: {
                link: {xAxisIndex: 'all'}
            },
            dataZoom: [
                {
                    show: true,
                    realtime: true,
                    // start: 0,
                    // end: 100,
                    xAxisIndex: [0, 1]
                },
                {
                    type: 'inside',
                    realtime: true,
                    // start: 30,
                    // end: 70,
                    xAxisIndex: [0, 1]
                }
            ],
            grid: [{
                left: 50,
                right: 50,
                height: '35%'
            }, {
                left: 50,
                right: 50,
                top: '55%',
                height: '35%'
            }],
            xAxis : [
                {
                    type : 'category',
                    boundaryGap : false,
                    axisLine: {onZero: true},
                    data: this.timeNumber
                },
                {
                    gridIndex: 1,
                    type : 'category',
                    boundaryGap : false,
                    axisLine: {onZero: true},
                    data: this.timeNumber,
                    position: 'top'
                }
            ],
            yAxis : [
                {
                    // name : '灵敏度(dBm)/速度(m/s)',
                    type : 'value',
                    max : 100
                },
                {
                    gridIndex: 1,
                    // name : '误码率(e-6)/加速度(m/s^2)',
                    type : 'value',
                    inverse: true
                }
            ],
            series : [
                this.getSeriesCommunicate(),
                this.getSeriesSpeed(),
                this.getSeriesOther2(),
               this.getSeriesOther()
            ]
                
            
        }
    }

    updateSeriesData(){
        var addspeed = (this.props.speed - this.seriesData[2][this.seriesData[2].length - 1])/ 1.0
        this.seriesData[0].push(this.props.communicate);
        this.seriesData[1].push(this.props.communicate);
        this.seriesData[2].push(this.props.speed);
        this.seriesData[3].push(addspeed);
        ++this.timenum;
        this.timeNumber.push(getTime(this.timenum))
    }

    getSeriesCommunicate(){
        return {
            name:'链路质量',
            type:'line',
            symbolSize: 8,
            hoverAnimation: false,
            data:this.seriesData[0],
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'black'
                },
                // width:8
            }
            // data:[
            //     45,65,74,53,88
            // ]
        }
    }

    getSeriesOther(){
        return {
            name:'误码率',
            type:'line',
            xAxisIndex: 1,
            yAxisIndex: 1,
            symbolSize: 8,
            hoverAnimation: false,
            // data: [
            //     2.4,3.2,2.8,1.9,5.2,1.7
            // ]
            data:this.seriesData[1],
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'blue'
                },
                // width:8
            }
        }
    }

    getSeriesSpeed(){
        return {
            name:'速度',
            type:'line',
            symbolSize: 8,
            hoverAnimation: false,
            data:this.seriesData[2],
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'red'
                },
                // width:8
            }
            // data:[
            //     2,4,3,5,3
            // ]
        }
    }

    getSeriesOther2(){
        return {
            name:'加速度',
            type:'line',
            xAxisIndex: 1,
            yAxisIndex: 1,
            symbolSize: 8,
            hoverAnimation: false,
            // data: [
            //     3.1,2.4,5.6,8.7,1.6
            // ]
            data:this.seriesData[3],
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'#006400'
                },
                // width:8
            }
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
              <ReactEcharts option = {this.getOption()} style = {{height:'500px' , width:'1130px'}} />
            </div>
        )
    }
}