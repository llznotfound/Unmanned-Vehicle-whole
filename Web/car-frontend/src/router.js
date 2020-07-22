import React from 'react'
import PropTypes from 'prop-types'
import { Router } from 'dva/router'
import App from './routes/app'

const registerModel = (app, model) => {
  if (!(app._models.filter(m => m.namespace === model.namespace).length === 1)) {
    app.model(model)
  }
}

const Routers = function ({ history, app }) {
  const routes = [
    {
      path: '/',
      component: App,
      getIndexRoute (nextState, cb) {
        require.ensure([], require => {
          registerModel(app, require('./models/main/index').default)
          cb(null, { component: require('./routes/main_2/') })
        }, 'working-status')
      },
      childRoutes: [
        {
          path: 'main_2',
          getComponent (nextState, cb) {
            require.ensure([], require => {
              registerModel(app, require('./models/main/index').default)
              cb(null, require('./routes/main_2/'))
            }, 'working-status')
          },
        },
        {
          path: 'main',
          getComponent (nextState, cb) {
            require.ensure([], require => {
              registerModel(app, require('./models/main/index').default)
              cb(null, require('./routes/main/'))
            }, 'main')
          },
        },
        {
          path: 'manageStatus',
          getComponent (nextState, cb) {
            require.ensure([], require => {
              registerModel(app, require('./models/managestatus').default)
              cb(null, require('./routes/managestatus/'))
            }, 'manageStatus')
          },
        },
        {
          path: '*',
          getComponent (nextState, cb) {
            require.ensure([], require => {
              cb(null, require('./routes/error/'))
            }, 'error')
          },
        },
      ],
    },
  ]

  return <Router history={history} routes={routes} />
}

Routers.propTypes = {
  history: PropTypes.object,
  app: PropTypes.object,
}

export default Routers
