import * as React from 'react'
import {style , media } from 'typestyle'
import * as coordtransform from 'coordtransform'

import {Table} from 'antd'
import WorkingPoint from './workingPoint';
// type PlanProps = PlanState
var num = 1;    //记录序号  

interface tableProps {
    workingPoint: Array<{lng:number, lat:number, status:string}> //工作路径点数组
    lng: number //船体位置经度
    lat: number //船体位置纬度
    theme: string   //主题
}

const columns = [{
    title: '点位',
    dataIndex: 'point',
}, {
   title: '坐标',
    dataIndex: 'position',
   render: text => <a href="javascript:;">{text}</a>,
 }, {
   title: '状态',
    dataIndex: 'status',
},{
    title: '时间',
    dataIndex: 'time',
  }
   ];

 
  
export default class MissionTable extends React.Component<tableProps , any> {
 
    //百度坐标转化为GPS坐标(官方没有提供相应转换方式，采用github上的coordtransform库实现)
    changeLocationArray(points: {lng: number, lat: number}[]) {
      return points.map((point) => {
      const gcj02Coord = coordtransform.bd09togcj02(point.lng, point.lat); //百度经纬度坐标转国测局坐标
      const wgs84Coord = coordtransform.gcj02towgs84(gcj02Coord[0], gcj02Coord[1]); //国测局坐标转wgs84坐标
      return {lng: wgs84Coord[0], lat: wgs84Coord[1]}
      })
  }

  updateTable(){
    if (!this.context) {
      return
      }
      if(this.props.workingPoint.length == 0){
          return 
      }
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
            const time = new Date().toLocaleTimeString();
            const lng = GPSposition[i].lng.toFixed(6)
            const lat = GPSposition[i].lat.toFixed(6)

        
        }
        
        
  
  }
  render(){
    const wholeStyle = style ({
        width:'100%',
        height:'100%'
    })

    return (
        <div className = {wholeStyle}>
            水水水水
        </div>
    )
}

}
