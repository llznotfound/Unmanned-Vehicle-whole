/// <reference path="../types/mockjs.d.ts" />

import {XMLHttpRequest} from 'xmlhttprequest'
import Mock from 'mockjs'

const wirelessPayloads = () => {
  return Mock.mock({
      data: {
        'payloads|4': ['@float(120, 200, 2, 2)']
      }
    }
  )
}

export default {
  [`GET /api/v1/warehouse/wireless/payloads`](req, res) {
    let payloads = wirelessPayloads()

    let newRequest = new XMLHttpRequest()
    newRequest.open('GET', 'http://demo.iot.sjtudoit.cn/api/v2/fog/alives', false)
    newRequest.send(null)
    if (200 == newRequest.status) {
      const responseText: string = newRequest.responseText
      const data: Array<Boolean> = JSON.parse(responseText).data
      const num = payloads.data.payloads.length < data.length ? payloads.data.payloads.length : data.length
      let aliveNum = 0
      for (let index in data.slice(0, num)) {
        if (data[index]) {
          aliveNum++
        }
      }
      if (0 == aliveNum) {
        for (let index in payloads.data.payloads) {
          payloads.data.payloads[index] = 0
        }
      } else {
        for (let ii = 0; ii < num; ++ii) {
          if (data[ii]) {
            payloads.data.payloads[ii] *= num / aliveNum
          } else {
            payloads.data.payloads[ii] = 0
          }
        }
        for (let ii = num; ii < payloads.data.length; ++ii) {
          payloads.data.payloads[ii] = 0
        }
      }
    }
    res.json(payloads)
  },
  [`GET /api/v1/warehouse/wireless/payloads/random`](req, res) {
    res.json(wirelessPayloads())
  }
}
