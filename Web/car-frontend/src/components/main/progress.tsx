import * as React from 'react'
import {style,media} from 'typestyle'


interface LogProps {
  name: string //进度条名称
  value: number //进度条的值
  max: number //进度条最小值
  min: number //进度条最大值
  unit: string //进度条值对应的单位
}

//进度条控件
export default class MyProgress extends React.Component<LogProps, any> {

  render() {
    const wholeStyle = style({
      width: '100%'
    })

    const nameStyle = style({
      float: 'left',
      color: '#00A18B',
      margin:'-2px 0px',
      fontSize:'12px',
      textIndent:'6px'
    },
    media({minWidth:3800}, 
      {
        margin:'-7px 0px',
        fontSize:'30px',
        textIndent:'20px'
      }
    ))

    const progressStyle = style({
      float: 'left',
      height: '14px',
      width: '100%',
      background: 'white',
      borderRadius: '7px',
      borderColor:'#00F2C4',
      borderStyle:'solid',
      borderWidth:'1px',
      margin:'-2px 0px',
    },
    media({minWidth:3800}, 
      {
        height: '30px',
        borderRadius: '10px'
      }
    ))

    const widthStyle = style({
      float: 'left',
      height: '12px',
      width: `${(this.props.value-this.props.min)*100/(this.props.max-this.props.min)}%`,
      borderRadius: '6px', 
    },
    media({minWidth:3800}, 
      {
        height: '30px',
        borderRadius: '10px'
      }
    ))

    const valueStyle = style({
      float: 'right',
      color: 'white',
      fontSize:'12px',
      margin:'-2.5px 8px 0px 0px'
    }, 
    media({minWidth:3800},  
      { 
        fontSize:'30px', 
        margin:'-10px 8px 0px 0px' 
      } 
    )) 

    if (this.props.name == "半径警告") {
      return (
        <div className={wholeStyle}>
          <div className={nameStyle}>{this.props.name}</div>
          <br />
          <div className={progressStyle}>
            <div className={widthStyle} style={{background:'linear-gradient(to right,#00F2C4,#FFFF33)'}}>
              <div className={valueStyle}>{this.props.value+this.props.unit}</div>
            </div>
          </div>
        </div>
      )
    } else {
      return (
        <div className={wholeStyle}>
          <div className={nameStyle}>{this.props.name}</div>
          <br />
          <div className={progressStyle}>
            <div className={widthStyle} style={{background:'linear-gradient(to right,#00F2C4,#9AEEA5)'}}> 
              <div className={valueStyle}>{this.props.value+this.props.unit}</div>
            </div>
          </div>
        </div>
      )
    }
    
  }
}
