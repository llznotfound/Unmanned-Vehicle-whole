const APIV1 = '/api/v1'
const APIV2 = '/api/v2'

module.exports = {
  name: '无人船控制总界面',
  prefix: '物联网综合演示',
  footerText: '无人船控制总界面',
  logo: '/logo.png',
  iconFontCSS: '/iconfont.css',
  iconFontJS: '/iconfont.js',
  YQL: ['http://www.zuimeitianqi.com'],
  CORS: ['http://localhost:7000'],
  // 修改起始页,直接跳过登录页面
  // openPages: ['/login'],
  openPages: ['/api/v1/user?name=admin&passport=admin'],
  apiPrefix: '/api/v1',
  api: {
    userLogin: `${APIV1}/user/login`,
    userLogout: `${APIV1}/user/logout`,
    userInfo: `${APIV1}/userInfo`,
    users: `${APIV1}/users`,
    posts: `${APIV1}/posts`,
    user: `${APIV1}/user/:id`,
    dashboard: `${APIV1}/dashboard`,
    v1test: `${APIV1}/test`,
    v2test: `${APIV2}/test`,
  },
}
