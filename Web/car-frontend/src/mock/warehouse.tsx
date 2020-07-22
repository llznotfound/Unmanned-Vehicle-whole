/// <reference path="../types/mockjs.d.ts" />

import {XMLHttpRequest} from 'xmlhttprequest'
import Mock from 'mockjs'

// const cpus = () => {
//   return Mock.mock({
//       data: {
//         'numbers|256': 1,
//         'payload|1': '@float(20, 40, 2, 2)'
//       }
//     }
//   )
// }
//
// const memory = () => {
//   return Mock.mock({
//       data: {
//         'used': '@float(280, 300, 2, 2)',
//         'total': 1024
//       }
//     }
//   )
// }
//
// const database = () => {
//   return Mock.mock({
//       'data|5': [{
//         'read': '@float(200, 800, 2, 2)',
//         'write': '@float(200, 300, 2, 2)'
//       }]
//     }
//   )
// }

const status = () => {
  return Mock.mock({
      data: {
        cpus: {
          'numbers|256': 1,
          'payload|1': '@float(33, 40, 2, 2)'
        },
        memory: {
          'used': '@float(280, 300, 2, 2)',
          'total': 1024
        },
        'database|5': [{
          'read':
            '@float(200, 800, 2, 2)',
          'write':
            '@float(200, 300, 2, 2)'
        }]
      }
    }
  )
}

export default {
  // [`GET /api/v1/warehouse/status/cpus`](req, res) {
  //   res.json(cpus())
  // },
  // [`GET /api/v1/warehouse/status/memory`](req, res) {
  //   res.json(memory())
  // },
  // [`GET /api/v1/warehouse/status/database`](req, res) {
  //   res.json(database())
  // }
  [`GET /api/v1/warehouse/status/random`](req, res) {
    res.json(status())
  },
  [`GET /api/v1/warehouse/status`](req, res) {
    let payloads = status()

    let newRequest = new XMLHttpRequest()
    newRequest.open('GET', 'http://demo.iot.sjtudoit.cn/api/v2/warehouse/alives', false)
    newRequest.send(null)
    if (200 == newRequest.status) {
      const responseText: string = newRequest.responseText
      const data: Array<Boolean> = JSON.parse(responseText).data
      const num = payloads.data.database.length < data.length ? payloads.data.database.length : data.length
      let aliveNum = 0
      for (let index in data.slice(0, num)) {
        if (data[index]) {
          aliveNum++
        }
      }
      if (0 == num) {
        payloads.data.cpus.payload = 0
        payloads.data.memory.used = 0
        for (let item of payloads.data.database) {
          item.read = 0
          item.write = 0
        }
      } else {
        payloads.data.cpus.payload *= aliveNum / num
        payloads.data.memory.used *= aliveNum / num
        for (let ii = 0; ii < num; ++ii) {
          if (!data[ii]) {
            payloads.data.database[ii].read = 0
            payloads.data.database[ii].write = 0
          }
        }
        for (let ii = num; ii < payloads.data.length; ++ii) {
          payloads.data.database[ii].read = 0
          payloads.data.database[ii].write = 0
        }
      }
    }
    res.json(payloads)
  }
}
