/**
 * 使用PCA9685芯片的Micro:bit扩展板控制MeArm机械臂
 */
//% weight=80 color=#00A3E0
namespace mearm {
  const PCA9685_ADD = 0x40;
  const MODE1 = 0x00;
  const LED0_ON_L = 0x06;
  const LED0_ON_H = 0x07;
  const LED0_OFF_L = 0x08;
  const LED0_OFF_H = 0x09;

  const PRESCALE = 0xFE;

  let initialized = false;

  export enum MearmServo {
    //% block="基座舵机"
    Base = 0,
    //% block="右边舵机"
    Right = 1,
    //% block="左边舵机"
    Left = 2,
    //% block="机械爪舵机"
    Grip = 3
  }
  export enum Direction {        
    //% block="逆时针"
    counterclockwise = 0,
    //% block="顺时针"
    clockwise = 1,
  
  }
  
  export enum ServoPin {        
    S1 = 0,
    S2,
    S3,
    S4,
    S5,
    S6,
    S7,
    S8
  }

  function i2cwrite(addr: number, reg: number, value: number) {
    let buf = pins.createBuffer(2)
    buf[0] = reg
    buf[1] = value
    pins.i2cWriteBuffer(addr, buf)
  }

  function i2ccmd(addr: number, value: number) {
      let buf = pins.createBuffer(1)
      buf[0] = value
      pins.i2cWriteBuffer(addr, buf)
  }

  function i2cread(addr: number, reg: number) {
      pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
      let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
      return val;
  }

  function initPCA9685(): void {
      i2cwrite(PCA9685_ADD, MODE1, 0x00);
      setFreq(50);
      initialized = true;
  }

  function setFreq(freq: number): void {
    let prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
    let oldmode = i2cread(PCA9685_ADD, MODE1);
    let newmode = (oldmode & 0x7F) | 0x10; 
    i2cwrite(PCA9685_ADD, MODE1, newmode);
    i2cwrite(PCA9685_ADD, PRESCALE, prescale);
    i2cwrite(PCA9685_ADD, MODE1, oldmode);
    control.waitMicros(5000);
    i2cwrite(PCA9685_ADD, MODE1, oldmode | 0xa1);
  }

  function setPwm(channel: number, on: number, off: number): void {
      if (channel < 0 || channel > 15)
          return;
      if (!initialized) {
          initPCA9685();
      }
      let buf = pins.createBuffer(5);
      buf[0] = LED0_ON_L + 4 * channel;
      buf[1] = on & 0xff;
      buf[2] = (on >> 8) & 0xff;
      buf[3] = off & 0xff;
      buf[4] = (off >> 8) & 0xff;
      pins.i2cWriteBuffer(PCA9685_ADD, buf);
  }

  let servos = [
    {minAngle: 0,   maxAngle: 179, currentAngle: 90, centerAngle: 90, servo: ServoPin.S1,  direction: Direction.clockwise},
    {minAngle: 0,   maxAngle: 135, currentAngle: 90, centerAngle: 90, servo: ServoPin.S2,  direction: Direction.clockwise},
    {minAngle: 30,  maxAngle: 160, currentAngle: 90, centerAngle: 90, servo: ServoPin.S3,  direction: Direction.clockwise},
    {minAngle: 0,   maxAngle: 89,  currentAngle: 90, centerAngle: 90, servo: ServoPin.S4,  direction: Direction.clockwise}
  ];

  function setServoAngle(servo: MearmServo, angle: number){
    let _servo = servos[servo];
    if(angle < _servo.minAngle){
      angle = _servo.minAngle;
    }else if(angle > _servo.maxAngle){
      angle = _servo.maxAngle;
    }
    if(angle !== _servo.currentAngle){
      _servo.currentAngle = angle;
      moveServo(_servo.servo,angle);
    }
  }

  function resetServoAngle(servo: MearmServo){
    let _servo = servos[servo];
    _servo.currentAngle = _servo.centerAngle;
    moveServo(_servo.servo,_servo.centerAngle);
  }

  function moveServoToMax(servo: MearmServo){
    let _servo = servos[servo];
    _servo.currentAngle = _servo.maxAngle;
    moveServo(_servo.servo,_servo.maxAngle);
  }

  function moveServoToMin(servo: MearmServo){
    let _servo = servos[servo];
    _servo.currentAngle = _servo.minAngle;
    moveServo(_servo.servo,_servo.minAngle);
  }

  function moveServo(num: ServoPin, value: number) {
    // 50hz: 20,000 us
    let us = (value * 1800 / 180 + 600); // 0.6 ~ 2.4
    let pwm = us * 4096 / 20000;
    setPwm(num, 0, pwm);

  }

  /**
   * 配置舵机
   */
  //% weight=90
  //% blockGap=10
  //% minAgl.min=0 minAgl.max=180 maxAgl.min=0 maxAgl.max=180 defaultAngle.min=0 defaultAngle.max=180
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20  
  //% blockId=config_servo block="配置|%servo=MearmServo|使用引脚|%pin=ServoPin|最小角度|%min|最大角度|%max|初始角度|%defaultAngle|方向|%dir=Direction|"
  export function configServo(servo: MearmServo,pin: ServoPin, minAgl: number, maxAgl: number,defaultAngle: number,dir: Direction){
    let _servo = servos[servo];
    _servo.servo=pin;
    _servo.currentAngle=defaultAngle;
    _servo.direction=dir;
    _servo.maxAngle=minAgl;
    _servo.minAngle=maxAgl;
    resetServoAngle(servo);
  }
  
  /**
   * 转动指定舵机到一个绝对角度
   */
  //% weight=90
  //% blockGap=10
  //% angle.min=0 angle.max=180
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20  
  //% blockId=move_to block="转动|%servo=MearmServo|到|%angle|度位置"
  export function moveToAngle(servo: MearmServo, angle: number){
    setServoAngle(servo, angle);
  }

  /**
   * 让指定舵机从当前位置转动一个角度
   */
  //% weight=80
  //% blockGap=10
  //% angle.min=0 angle.max=180
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20  
  //% blockId=move_by block="控制|%servo=MearmServo|转动|%angle|度"
  export function moveByAngle(servo: MearmServo, angle: number){
    setServoAngle(servo, servos[servo].currentAngle + angle);
  }

  /**
   * 让指定舵机回到中间位置
   */
  //% weight=70
  //% blockId=move_to_centre block="控制|%servo=MearmServo|回到中间位置"
  export function moveToCentre(servo: MearmServo){
    resetServoAngle(servo);
  }
  
  /**
   * 打开机械爪
   */
  //% weight=50
  //% blockId=open_grip block="打开机械爪"
  export function openGrip(){
    moveServoToMax(MearmServo.Grip);
  }

  /**
   * 关闭机械爪
   */
  //% weight=40
  //% blockId=close_grip block="关闭机械爪"
  export function closeGrip(){
    moveServoToMin(MearmServo.Grip);
  }

}
