import * as React from 'react'
import {style , media} from 'typestyle'

import ReactEcharts from 'echarts-for-react'

interface angleProps {
    angle:number //航向角
    battery:number //电池
    radius:number //半径警告
    time:number //预估时间
    accelerate:number
}

function getTime (timenum){
    var now = new Date();
    var res = [];
    var len = timenum;
    res.push(now.toLocaleTimeString());
    return res;
}

export default class AngleTable extends React.Component<angleProps , any > {
    seriesData : Array<Array<number>> = [[0],[0],[0],[0]]
    timeData : Array<any> = [0]
    timenumber = 0;
    getOption (){
        this.updateSeriesData();
        return {

            tooltip: {
                trigger: 'axis'
            },
            legend: {
                left: 'center',
                
                data: [ {
                    name:'航向角',
                    textStyle:{
                        fontSize : 20
                    }
                },{
                    name:'电池',
                    textStyle:{
                        fontSize : 20
                    }
                }, {
                    name:'工作半径',
                    textStyle:{
                        fontSize : 20
                    }
                }, {                
                    name:'预估时间',
                    textStyle:{
                        fontSize : 20
                    }
                }]
                
            },
            grid: {
                left: '3%',
                right: '4%',
                // bottom: '3%',
                containLabel: true
            },

            xAxis: {
                type: 'category',
                boundaryGap: false,
                data: this.timeData
            },
            yAxis: {
                type: 'value'
            },
            dataZoom: [
                {
                    type: 'inside',
                    // xAxisIndex: [0, 1],
                    // start: 0,
                    // end: 100,

                },
                {
                    show: true,
                    // xAxisIndex: [0, 1],
                    type: 'slider',
                    top: '90%',
                    // start: 20,
                    // end: 30
                }
            ],
            series: [
                this.getSeriesMA5(),
                this.getSeriesMA10(),
                this.getSeriesMA20(),
                this.getSeriesMA30(),
            ]
        }
    }

    

    updateSeriesData(){
        this.seriesData[0].push(this.props.angle);
        this.seriesData[1].push(this.props.battery);
        this.seriesData[2].push(this.props.radius);
        this.seriesData[3].push(this.props.time);
        ++this.timenumber
        this.timeData.push(getTime(this.timenumber))
        // this.seriesData.shift();
    }

    


    getSeriesMA5(){
        return {
            name: '航向角',
            type: 'line',
            data: this.seriesData[0],
            // symbolSize: 8,
            // data:[
            //     214.35,234.56,223.34,226.45,256.78
            // ],
            // smooth: true,
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'red'
                },
                
            }
        }
    }

    getSeriesMA10(){
        return {
            name: '电池',
            type: 'line',
            data: this.seriesData[1],
            // symbolSize: 8,
            // data:[
            //     100,95,87,75,54
            // ],
            // smooth: true,
            lineStyle: {
                normal: {
                    opacity: 0.5,
                    width:12,
                    // color:'black'
                },
            }
        }
    }

    getSeriesMA20(){
        return {
            name: '工作半径',
            type: 'line',
             data: this.seriesData[2],
            //  symbolSize: 8,
            // data:[5.4,5.8,4.3,3.7,1.9],
            // smooth: true,
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

    getSeriesMA30(){
        return {
            name: '预估时间',
            type: 'line',
            data: this.seriesData[3],
            // symbolSize: 8,
            // data:[162,154,135,124,110],
            // smooth: true,
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
                  <ReactEcharts option = {this.getOption()} style = {{height:'440px' , width: '1130px'}} />
                </div>
            )
        }
        
}
