// pages/map/map.js
const app = getApp();
// 引用百度地图微信小程序JSAPI模块
const BMap = require('../../libs/bmap-wx.min.js');
var bmap;
var util = require('../../utils/util.js')
Page({

  /**
   * 页面的初始数据
   */
  data: {
    result: [],
    types: ["将GCJ-02(火星坐标)转为百度坐标"],
    longitude: '', //经度
    latitude: '', //纬度
    address: '',
    scale: 13, //地图的扩大倍数
    markers: [{ //标记点用于在地图上显示标记的位置
      id: 1,
      latitude: '',
      longitude: '',
      iconPath: '../../public/image/location.png',
      width: 1,
      height: 1,
    }],
  },
  /**
   * 生命周期函数--监听页面加载
   */
  onLoad: function(options) {
    var array = [];
    /**
     * 将GCJ-02(火星坐标)转为百度坐标
     */
    var result2 = util.transformFromGCJToBaidu(22.53329, 113.83308);
    console.log("result2 = ", result2)
    array.push(result2);
    this.setData({
      result: array
    })
    const that = this;

    // 实例化百度地图API核心类
    bmap = new BMap.BMapWX({
      ak: app.globalData.ak
    })

    //获取当前位置经纬度
    app.getLocation(function(location) {
      console.log(location);
      var str = 'markers[0].longitude',
        str2 = 'markers[0].latitude';
      that.setData({
        longitude: location.longitude,
        latitude: location.latitude,
        [str]: location.longitude,
        [str2]: location.latitude,
      })
    })
  },

  getLngLat: function() {
    var that = this;
    this.mapCtx = wx.createMapContext("myMap");
    var latitude, longitude;
    this.mapCtx.getCenterLocation({
      success: function(res) {
        latitude = res.latitude;
        longitude = res.longitude;
        var str = 'markers[0].longitude',
          str2 = 'markers[0].latitude';
        var array = [];
        /**
         * 将GCJ-02(火星坐标)转为百度坐标
         */
        var result2 = util.transformFromGCJToBaidu(res.longitude, res.latitude);
        //console.log("result2 = ", result2)
        array.push(result2);
        that.setData({
          longitude: res.longitude,
          latitude: res.latitude,
          [str]: res.longitude,
          [str2]: res.latitude,
          result: array,
        })
      }
    })

    //平移marker，修改坐标位置 
    // this.mapCtx.translateMarker({
    //   markerId: 1,
    //   autoRotate: true,
    //   duration: 1000,
    //   destination: {
    //     latitude: latitude,
    //     longitude: longitude,
    //   },
    //   animationEnd() {
    //     console.log('animation end')
    //   }
    // })
  },

  //地图位置发生变化
  regionchange(e) {
    // 地图发生变化的时候，获取中间点，也就是用户选择的位置
    if (e.type == 'end' && (e.causedBy == 'scale' || e.causedBy == 'drag')) {
      //this.getLngLat();
    }
  },
  markertap(e) {
    console.log(e.markerId)
    //this.regionchange(e)
    //this.getLngLat();
    console.log(e);
  },
  controltap(e) {
    //console.log(e.controlId)
    var that = this;
    console.log("scale===" + this.data.scale)
    if (e.controlId === 1) {
      that.setData({
        scale: ++this.data.scale
      })
    } else {
      that.setData({
        scale: --this.data.scale
      })
    }
  },
  click: function() {
    this.getLngLat()
  },

  // 发起regeocoding逆地址解析 -- 从经纬度转换为地址信息
  // regeocoding() {
  //   const that = this;
  //   bmap.regeocoding({
  //     location: that.data.latitude + ',' + that.data.longitude,
  //     success: function (res) {
  //       that.setData({
  //         address: res.wxMarkerData[0].address
  //       })
  //     },
  //     fail: function (res) {
  //       that.tipsModal('请开启位置服务权限并重试!')
  //     },

  //   });
  // },
  //提示
  // tipsModal: function (msg) {
  //   wx.showModal({
  //     title: '提示',
  //     content: msg,
  //     showCancel: false,
  //     confirmColor: '#2FB385'
  //   })
  // },
  downloadFile: function () {
    wx.downloadFile({
      url: 'http://yqxspj.natappfree.cc/ocean/download',
      success(res) {
        console.log(res)
        // 只要服务器有响应数据，就会把响应内容写入文件并进入 success 回调，业务需要自行判断是否下载到了想要的内容
        if (res.statusCode === 200) {
          wx.saveFile({
            tempFilePath: res.tempFilePath,
            success: function (res) {
              console.log(res)
              var savedFilePath = res.savedFilePath
              console.log("文件已下载到：" + savedFilePath)
              wx.getSavedFileList({
                success: function (res) {
                  console.log(res)
                }
              })
              wx.openDocument({
                filePath: savedFilePath,
                success: function (res) {
                  console.log('打开文档成功')
                }
              })
            }
          })
          // wx.playVoice({
          //   filePath: res.tempFilePath
          // })
        }
      },
      fail: function (res) {
        console.log(res)
      }
    })
  },

  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady: function() {

  },

  /**
   * 生命周期函数--监听页面显示
   */
  onShow: function() {

  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide: function() {

  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload: function() {

  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh: function() {

  },

  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom: function() {

  },

  /**
   * 用户点击右上角分享
   */
  onShareAppMessage: function() {

  }
})