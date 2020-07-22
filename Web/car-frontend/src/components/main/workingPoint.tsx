import * as React from 'react'
import * as Redux from 'redux'
import {style,media} from 'typestyle'
import {Marker} from 'react-bmap'
import {deleteWorkingPointAction} from '../../models/main/workingstatus'

declare let BMap
declare function require(path: string): any
const red = require('../../assets/red.gif') //获取红色路径点的图标
const yellow = require('../../assets/yellow.gif') //获取黄色路径点的图标
const green = require('../../assets/green.gif') //获取绿色路径点的图标

interface WorkingPointProps {
  workingPoint: Array<{lng:number, lat:number, status:string,}> //工作路径点数组
  map?: any
  dispatch: Redux.Dispatch<any>
}
var isFirstLoad = false;//是否是第一次加载，第一次加载不触发清除事件
var lineList = new Array();//记录要绘制的线
var arrowLineList = new Array();//记录绘制的箭头线
    
var arrowLineLengthRate = 15 / 10;//15是你初始时设置的地图显示范围，10是你初始时设置的箭头的长度，当地图放大缩小时保证箭头线长度一致
//显示添加的工作路径点的组件
export default class WorkingPoint extends React.Component<WorkingPointProps, any> {
  
  render() {
    return null
  }

  //根据状态选择路径点颜色
  chooseColor(status:string) {
    switch(status) {
      case 'done': return yellow
      case 'doing': return green
      case 'todo': return red
    }
  }

  //渲染工作路径点，响应右键删除功能
  renderFlag() {
    const workingPoint = this.props.workingPoint
    if (!workingPoint || workingPoint.length == 0) {
      return null
    }
    var num=0
    workingPoint.forEach((point)=>{
      const myIcon = new BMap.Icon(this.chooseColor(point.status), new BMap.Size(25, 25))
      myIcon.setImageSize(new BMap.Size(25, 25))
      const geo = new BMap.Point(point.lng, point.lat)
      const marker = new BMap.Marker(geo, {icon:myIcon})
      marker.setLabel(getNumberLabel(num++));
 
      function getNumberLabel(number) {
        var offsetSize = new BMap.Size(0, 0);
        // var labelStyle = style();
 
        if(number == 0){
          var label = new BMap.Label("起始点", {
            offset: offsetSize
        });
        } else {
          var label = new BMap.Label(number, {
            offset: offsetSize
        });
        }
 
        
        label.setStyle({
          color: "#fff",
          backgroundColor: "0.05",
          border: "0",
          fontSize:'20px',
          margin:'14px'
        });
        return label;
    }
      
      const markerMenu=new BMap.ContextMenu()
      markerMenu.addItem(new BMap.MenuItem('删除', (e) => {
        this.props.dispatch(deleteWorkingPointAction(e))
      }))
      
      this.props.map.addOverlay(marker)
      marker.addContextMenu(markerMenu)
    })
  }

  componentDidUpdate() {
    this.props.map.clearOverlays()
    this.renderFlag()
    this.renderPolyline()
  }

  componentWillUnmount() {
    this.props.map.clearOverlays()
  }

  //渲染工作点连线
  renderPolyline() { 
    const workingPoint = this.props.workingPoint
    if (!workingPoint || workingPoint.length < 2) {
      return null
    }
    const color = (status:string) => {
      switch(status) {
        case 'done': return 'yellow'
        case 'doing': return 'green'
        case 'todo': return 'red'
      }
    }
    

    workingPoint.reduce((prev, next)=>{
      const polyline = new BMap.Polyline([
        new BMap.Point(prev.lng, prev.lat),
        new BMap.Point(next.lng, next.lat),
      ], {strokeColor:'#0000ff', strokeWeight:1, strokeOpacity:1, strokeStyle:'solid'})
      this.props.map.addOverlay(polyline)

      lineList[lineList.length] = polyline;//记录要绘制的线
      arrowLineList[arrowLineList.length] = this.addArrow(polyline,10,Math.PI/7);//记录绘制的箭头线

      isFirstLoad = true;//第一次加载

      return next
    })
  } 

  /**
     * 在百度地图上给绘制的直线添加箭头
     * @param polyline 直线 var line = new BMap.Polyline([faydPoint,daohdPoint], {strokeColor:"blue", strokeWeight:3, strokeOpacity:0.5});
     * @param length 箭头线的长度 一般是10
     * @param angleValue 箭头与直线之间的角度 一般是Math.PI/7
     */
    addArrow(polyline,length,angleValue){ //绘制箭头的函数

      var linePoint=polyline.getPath();//线的坐标串
      var arrowCount=linePoint.length;
      for(var i =1;i<arrowCount;i++){ //在拐点处绘制箭头
          var pixelStart=this.props.map.pointToPixel(linePoint[i-1]);
          var pixelEnd=this.props.map.pointToPixel(linePoint[i]);
          var angle=angleValue;//箭头和主线的夹角
          var r=length; // r/Math.sin(angle)代表箭头长度
          var delta=0; //主线斜率，垂直时无斜率
          var param=0; //代码简洁考虑
          var pixelTemX,pixelTemY;//临时点坐标
          var pixelX,pixelY,pixelX1,pixelY1;//箭头两个点
          if(pixelEnd.x-pixelStart.x==0){ //斜率不存在是时
              pixelTemX=pixelEnd.x;
              if(pixelEnd.y>pixelStart.y)
              {
              pixelTemY=pixelEnd.y-r;
              }
              else
              {
              pixelTemY=pixelEnd.y+r;
              }    
              //已知直角三角形两个点坐标及其中一个角，求另外一个点坐标算法
              pixelX=pixelTemX-r*Math.tan(angle); 
              pixelX1=pixelTemX+r*Math.tan(angle);
              pixelY=pixelY1=pixelTemY;
          }
          else  //斜率存在时
          {
              delta=(pixelEnd.y-pixelStart.y)/(pixelEnd.x-pixelStart.x);
              param=Math.sqrt(delta*delta+1);

              if((pixelEnd.x-pixelStart.x)<0) //第二、三象限
              {
              pixelTemX=pixelEnd.x+ r/param;
              pixelTemY=pixelEnd.y+delta*r/param;
              }
              else//第一、四象限
              {
              pixelTemX=pixelEnd.x- r/param;
              pixelTemY=pixelEnd.y-delta*r/param;
              }
              //已知直角三角形两个点坐标及其中一个角，求另外一个点坐标算法
              pixelX=pixelTemX+ Math.tan(angle)*r*delta/param;
              pixelY=pixelTemY-Math.tan(angle)*r/param;

              pixelX1=pixelTemX- Math.tan(angle)*r*delta/param;
              pixelY1=pixelTemY+Math.tan(angle)*r/param;
          }

          var pointArrow=this.props.map.pixelToPoint(new BMap.Pixel(pixelX,pixelY));
          var pointArrow1=this.props.map.pixelToPoint(new BMap.Pixel(pixelX1,pixelY1));
          var Arrow = new BMap.Polyline([
              pointArrow,
           linePoint[i],
              pointArrow1
          ], {strokeColor:"black", strokeWeight:3, strokeOpacity:0.5});
          this.props.map.addOverlay(Arrow);
          return Arrow;
      }

      //地图加载完毕事件
      this.props.map.addEventListener("tilesloaded",function(){
        //alert("地图加载完毕");
        if(!isFirstLoad){
            //map.clearOverlays();//清除所有的覆盖物
            //清除上一次绘制的箭头线，不清除上一次的箭头线，当地图放大时箭头线也会跟着放大
            for(var i=0; i<arrowLineList.length; i++){
              this.props.map.removeOverlay(arrowLineList[i]);//清除制定的覆盖物，可以是直线、标注等
            }
            arrowLineList.length = 0;
            //重新绘制箭头线
            for(var i=0; i<lineList.length; i++){
                arrowLineList[arrowLineList.length] = this.addArrow(lineList[i],15 / arrowLineLengthRate,Math.PI / 7);//记录绘制的箭头线
            }
        }
        isFirstLoad = false;
      });

  }

}



  