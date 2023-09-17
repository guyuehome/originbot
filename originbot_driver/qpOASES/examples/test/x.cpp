
PIO
static uint16_t _DI_PINS[GPIO_PORT_IDX_NUM][GPIO_PORT_PINS_NUM] = {
    // GPIOA
    {0},
    // GPIOB
    //                              副臂上位     正臂上位     安全门      全自动       开模完      关模完
    {MPG_AXIS2_PIN, MPG_AXIS1_PIN, DI_X07_PIN, DI_X00_PIN, DI_X42_PIN, DI_X48_PIN, DI_X46_PIN, DI_X45_PIN},
    // GPIOC
    //                              中板检测     预留        开放侧安全  开放侧下降许可
    {MPG_AXIS4_PIN, MPG_AXIS3_PIN, DI_X35_PIN, DI_X52_PIN, DI_X55_PIN, DI_X56_PIN},
    // GPIOD
    // 不良品     顶针前进限  中子1前进限  中子1后退限  顶针后退限   开模到中间   中子2前进限  中子2后退限
    {DI_X44_PIN, DI_X50_PIN, DI_X54_PIN, DI_X57_PIN, DI_X53_PIN, DI_X60_PIN, DI_X58_PIN, DI_X59_PIN},
    // GPIOE
    //                                                                                    开放位置信号
    {MPG_AXIS0_PIN, MPG_X100_PIN, MPG_X1_PIN, MPG_X10_PIN, MPG_BUTTON_PIN, MPG_ESTOP_PIN, DI_X18_PIN},
};

static uint16_t _DI_PINS[GPIO_PORT_IDX_NUM][GPIO_PORT_PINS_NUM] = {
    // GPIOA
    {0},
    // GPIOB
    //                              副臂上位     正臂上位     安全门      全自动       开模完      关模完
    {MPG_AXIS2_PIN, MPG_AXIS1_PIN, DI_X07_PIN, DI_X00_PIN, DI_X42_PIN, DI_X48_PIN, DI_X46_PIN, DI_X45_PIN},
    // GPIOC
    //                              中板检测     预留        开放侧安全  开放侧下降许可
    {MPG_AXIS4_PIN, MPG_AXIS3_PIN, DI_X35_PIN, DI_X52_PIN, DI_X55_PIN, DI_X56_PIN},
    // GPIOD
    // 不良品     顶针前进限  中子1前进限  中子1后退限  顶针后退限   开模到中间   中子2前进限  中子2后退限
    {DI_X44_PIN, DI_X50_PIN, DI_X54_PIN, DI_X57_PIN, DI_X53_PIN, DI_X60_PIN, DI_X58_PIN, DI_X59_PIN},
    // GPIOE
    //                                                                                    开放位置信号
    {MPG_AXIS0_PIN, MPG_X100_PIN, MPG_X1_PIN, MPG_X10_PIN, MPG_BUTTON_PIN, MPG_ESTOP_PIN, DI_X18_PIN},
};

TIO
static uint16_t _DI_PINS[GPIO_PORT_IDX_NUM][GPIO_PORT_PINS_NUM] = {
    // GPIOA
    // 第七轴极限 第六轴极限   第七轴原点   第六轴原点   吸2检测-主臂
    {DI_X37_PIN, DI_X38_PIN, DI_X36_PIN, DI_X34_PIN, DI_X24_PIN},
    // GPIOB
    // 抱1检测-主臂 侧姿信号-主臂 正臂防撞  正臂上位    正臂上下极限 正臂上下原点-MZ原点 预留    预留
    {DI_X19_PIN, DI_X16_PIN, DI_X33_PIN, DI_X00_PIN, DI_X02_PIN, DI_X01_PIN, DI_X14_PIN, DI_X08_PIN},
    // GPIOC
    // 抱2检测-主臂 回正信号-主臂 置物侧区域 走行轴原点-Y原点 正臂引拔原点-MX原点 预留
    {DI_X20_PIN, DI_X15_PIN, DI_X18_PIN, DI_X12_PIN, DI_X03_PIN, DI_X05_PIN},
    // GPIOD
    // 预留   副臂上下原点-SZ原点 预留-SX原点 副臂上位  吸1检测-主臂 副夹检测-副臂 取物侧区域-Y方向 预留
    {DI_X17_PIN, DI_X06_PIN, DI_X09_PIN, DI_X07_PIN, DI_X23_PIN, DI_X29_PIN, DI_X04_PIN, DI_X13_PIN},
    // GPIOE
    // 空气压力检测 抱3检测    吸4检测      抱4检测      吸3检测    旋转动作复位   预留      旋转动作到位    预留
    {DI_X26_PIN, DI_X21_PIN, DI_X27_PIN, DI_X22_PIN, DI_X25_PIN, DI_X10_PIN, DI_X31_PIN, DI_X11_PIN, DI_X32_PIN},
};

uint16_t _DO_PINS[GPIO_PORT_IDX_NUM][GPIO_PORT_PINS_NUM] = {
    // GPIOA
    // 预留         预留       旋转动作
    {DO_Y16_PIN, DO_Y17_PIN, DO_Y47_PIN},
    // GPIOB
    // 抱2-主臂   副夹-副臂    三色灯(红)   蜂鸣器      预留
    {DO_Y03_PIN, DO_Y10_PIN, DO_Y41_PIN, DO_Y40_PIN, DO_Y18_PIN, DO_BK3_PIN},
    // GPIOC
    // 预留         预留         预留       预留         吸3        旋转复位      吸4       三色灯(绿)   自动打油      预留
    {DO_Y29_PIN, DO_Y06_PIN, DO_Y28_PIN, DO_Y25_PIN, DO_Y14_PIN, DO_Y45_PIN, DO_Y15_PIN, DO_Y43_PIN, DO_Y27_PIN, DO_Y46_PIN},
    // GPIOD
    // 抱3          抱4         喷雾     侧姿动作-主臂 回正动作-主臂 吸1-主臂   吸2-主臂      抱1-主臂
    {DO_Y04_PIN, DO_Y05_PIN, DO_Y26_PIN, DO_Y01_PIN, DO_Y00_PIN, DO_Y08_PIN, DO_Y09_PIN, DO_Y02_PIN},
    // GPIOE
    // 三色灯(黄)
    {DO_Y42_PIN, DO_BK5_PIN, DO_BK7_PIN, DO_BK6_PIN, DO_BK1_PIN, DO_BK2_PIN, DO_BK4_PIN},
};