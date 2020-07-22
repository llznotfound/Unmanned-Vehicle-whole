import React from 'react'
import PropTypes from 'prop-types'
import { Icon, Switch } from 'antd'
import styles from './Layout.less'
import { config } from '../../utils'
import Menus from './Menu'

const Sider = ({ siderFold, darkTheme, location, changeTheme, navOpenKeys, changeOpenKeys, menu }) => {
  const menusProps = {
    menu,
    siderFold,
    darkTheme,
    location,
    navOpenKeys,
    changeOpenKeys,
  }
  return (
    <div>
      <div className={styles.logo} style={{ backgroundColor: darkTheme ? '#363636' : '#fff' }}>
        <img alt={'logo'} src={config.logo} />
        {/* {siderFold ? '' : <div>无人船控制总界面</div>} */}
      </div>
      <Menus {...menusProps} />
      {!siderFold ? <div className={styles.switchtheme} onClick={(() => changeThemes())}>
        {/* <Switch onChange={changeTheme} defaultChecked={darkTheme} checkedChildren="Dark" unCheckedChildren="Light" style={{display:'none'}}/> */}
        <Switch onChange={changeTheme} defaultChecked={darkTheme} checkedChildren="Dark" unCheckedChildren="Light" />
      </div> : ''}
    </div>
  )

  function changeThemes () {
    sessionStorage.setItem('theme', darkTheme)
  }
}

Sider.propTypes = {
  menu: PropTypes.array,
  siderFold: PropTypes.bool,
  darkTheme: PropTypes.bool,
  location: PropTypes.object,
  changeTheme: PropTypes.func,
  navOpenKeys: PropTypes.array,
  changeOpenKeys: PropTypes.func,
}

export default Sider
