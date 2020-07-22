import * as React from 'react'
import {style , media } from 'typestyle'
import * as coordtransform from 'coordtransform'

import {Table} from 'antd'
// type PlanProps = PlanState
var num = 1;    //记录序号  

interface PlanProps {
    ph:number
    turbidity:number //浊度
    conductivity:number //电导率
    oxygen:number //溶氧
    workingPoint: Array<{lng:number, lat:number, status:string}> //工作路径点数组
    temperature:number  //温度
    lng: number //船体位置经度
    lat: number //船体位置纬度
    theme: string   //主题
}

export default class MissionTable extends React.Component<PlanProps , any>{
    log: HTMLDivElement

    render() {
        const logStyle = style({    //给表格定框框
            position:'absolute',
            top:'36px',
            left:'0px',
            width:'570px',
            height:'227px',
            backgroundColor:this.props.theme == "false"?'#363636':'#fff',
        },
        media({minWidth:1700},
            {
             top:'39.6px',
             width:'850px',
             height:'263.4px'
            }
        ))

        const missionPoint = style ({   //表头任务点样式
            position:'absolute',
            top:'9px',
            width:'36px',
            height:'17px',
            fontFamily:'PingFangSC',
            fontSize:'12px',
            fontWeight:600,
            fontStyle:'normal',
            fontStretch:'normal',
            lineHeight:'normal',
            letterSpacing:'-0.2px',
            textAlign:'left',
            color:this.props.theme == "false"?'#e3e3e3':'#000',
        },media({minWidth:1700},
            {
                top:'9.9px',
                width:'50.4px',
                height:'18.7px',
                fontSize:'16.8px',
                // fontWeight:840
            }
        ))


        const flowStyle = style({
            overflowX: 'hidden',
            overflowY: 'auto',
            height: window.innerWidth>1700?'263.4px':'194px',
            width: window.innerWidth>1700?'758px':'570px',//为了使出现滑动条以后与表头对齐
            top:'72px',
            position: 'absolute',
        })

        return (
        <div style={{height:'100%',width:'100%'}}>
            <div className={logStyle}>
                <div className = {missionPoint} onClick={() => this.test1()} style={{left:window.innerWidth>1700?80:30}}>点位</div>
                <div className = {missionPoint} style={{left:window.innerWidth>1700?325.4:85}}>坐标</div>
                <div className = {missionPoint} style={{left:window.innerWidth>1700?525.8:145}}>状态</div>
                <div className = {missionPoint} style={{left:window.innerWidth>1700?640.4:201}}>时间</div>
            </div>
            <div className={flowStyle} ref={(log)=>{this.log=log}}>
            </div>
        </div>
        )
    }
    test1(){
        console.log('test')
    }

    //百度坐标转化为GPS坐标(官方没有提供相应转换方式，采用github上的coordtransform库实现)
    changeLocationArray(points: {lng: number, lat: number}[]) {
        return points.map((point) => {
        const gcj02Coord = coordtransform.bd09togcj02(point.lng, point.lat); //百度经纬度坐标转国测局坐标
        const wgs84Coord = coordtransform.gcj02towgs84(gcj02Coord[0], gcj02Coord[1]); //国测局坐标转wgs84坐标
        return {lng: wgs84Coord[0], lat: wgs84Coord[1]}
        })
    }

    //更新log控件信息
    updateLog() {
        if (!this.log) {
        return
        }
        if(this.props.workingPoint.length == 0){
            return 
        }
        // var nums = 0
        this.log.innerHTML = `<table style="border-collapse:collapse;width:${window.innerWidth>1700?'758px':'570px'};color:${this.props.theme == "false"?'#e3e3e3':'#000'};font-family:PingFangSC;fong-weight:840;lett-spacing:1.7px;table-layout:fixed">
                            </table>`
        // console.log(this.props.workingPoint)
        // if(num > this.props.workingPoint.length){
        //     return 
        // }

        var GPSposition = this.changeLocationArray(this.props.workingPoint)

        var workingPointGroup = [{
            lng:'121.23',
            lat:'31.2',
            status:'已离开'
        },{
            lng:'122.22',
            lat:'50.12',
            status:num>1?'已离开':'已完成'
        },{
            lng:'140.2',
            lat:'20.1',
            status:'前往中'
        }]
        // for(var i = 0; i < workingPointGroup.length;i++){
        for(var i = 0; i < GPSposition.length;i++){
            const id=i+1
            const statusC = this.props.workingPoint[i].status
            const status = ()=>{
                switch(statusC){
                    case "doing":return "前往中";
                    case "todo":return "已规划";
                    case "done":return "已离开";
                }
            }
            // const status = workingPointGroup[i].status
            //const temperature = this.props.temperature
            const time = new Date().toLocaleTimeString();
            //const ph = this.props.ph
            //const conductivity = this.props.conductivity
           //const turbidity = this.props.turbidity
            //const oxygen = this.props.oxygen
            // const lng = this.props.workingPoint[i].lng
            const lng = GPSposition[i].lng.toFixed(6)
            // const lng = workingPointGroup[i].lng
            // const lat = this.props.workingPoint[i].lat
            // const lat = workingPointGroup[i].lat
            const lat = GPSposition[i].lat.toFixed(6)

            const substring = this.log.innerHTML.substring(0, this.log.innerHTML.length-8)

            //更新表格数据
            this.log.innerHTML = substring+`<tr style="${id%2!=0?'background-color:rgba(237,237,237,0.05);':''}">
                                    <td align="center" >${id}</td>
                                    <td align="center" style="font-size:${window.innerWidth>1700?'18px':'8px'}">E${lng}°W${lat}°</td>
                                    <td align="right" style="">${status()}</td>
                                    <td align="center" style="">${time}</td>
                                    
                                </tr>
                                </table>`
        }
        
    }

    componentDidMount() {
        setInterval(() => {
        this.updateLog()
        num++
        }, 100)
    }
}