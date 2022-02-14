/**
 * 使用PCA9685芯片的Micro:bit扩展板控制MeArm机械臂
 */
//% weight=80 color=#00A3E0
namespace mearm {

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

    //几何尺寸，L1、L2和L3的数值参考集合定义,mearm的源项目是以下数值
    const L1 = 80;
    const L2 = 80;
    const L3 = 68;

  const PI = 3.14159265359;
  const PCA9685_ADD = 0x40;
  const MODE1 = 0x00;
  const LED0_ON_L = 0x06;
  const LED0_ON_H = 0x07;
  const LED0_OFF_L = 0x08;
  const LED0_OFF_H = 0x09;
  const PRESCALE = 0xFE;
  let initialized = false;
    let currentGesture = { "x": 0, "y": 0,"z": 0,"r":0,"theta":0 };

  function i2cwrite(addr: number, reg: number, value: number) {
    let buf = pins.createBuffer(2)
    buf[0] = reg
    buf[1] = value
    pins.i2cWriteBuffer(addr, buf)
  }

    function cart2polar(a: number, b: number)
    {
        let rad=0.0;
        let theta=0.0;
        rad =Math.sqrt(a * a + b * b);
        if (rad != 0){
            let c = a / rad;
            let s = b / rad;

            if (s > 1){
                s = 1;
            } 
            if (c > 1){
                c = 1;
            }
            if (s < -1){
                s = -1;
            } 
            if (c < -1){
                c = -1;
            } 
            theta = Math.acos(c);
            if (s < 0){
                theta *= -1;
            } 
        }
        return { "rad": rad,"theta":theta};

    }
    function polarToCartesian(theta:number, r:number){
        currentGesture.r=r;
        currentGesture.theta=theta;
        let x = r * Math.sin(theta);
        let y = r * Math.cos(theta);
        return {"x":x,"y":y};
    }
    // Get angle from a triangle using cosine rule
    function cosangle(opp:number, adj1:number, adj2:number)
    {
        // Cosine :
        // C^2 = A^2 + B^2 - 2*A*B*cos(angle_AB)
        // cos(angle_AB) = (A^2 + B^2 - C^2)/(2*A*B)
        // C is opposite
        // A, B are adjacent
        let theta=0.0;
        let den = 2 * adj1 * adj2;
        if (den == 0){
            return { "success": false, "theta": theta};
        } else{
            let c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den;
            if (c > 1 || c < -1){
                return { "success": false, "theta": theta };
            } else{
                theta = Math.acos(c);
                return { "success": true, "theta": theta };
            }
        }
    }

    function iksolve(x:number, y:number, z:number)
    {
        let topDownView=cart2polar(y, x);
        let r=topDownView.rad;
        let th0=topDownView.theta;
        r -= L3;
        let inArmPlane=cart2polar(r, z);
        let R=inArmPlane.rad;
        let ang_P=inArmPlane.rad;

        let armInnerAngle1 = cosangle(L2, L1, R);
        let armInnerAngle2 = cosangle(R, L1, L2);
        if(armInnerAngle1.success==false || armInnerAngle2.success==false){
            return { success: false, "a0": 0, "a1": 0, "a2": 0};
        }else{
            let B=armInnerAngle1.theta;
            let C=armInnerAngle2.theta;
            let a0 = th0;
            let a1 = ang_P + B;
            let a2 = C + a1 - PI;
            return { success: true, "a0": a0, "a1": a1, "a2": a2 };
        }
    }

    function getX(){
        return currentGesture.x;
    }

    function getY(){
        return currentGesture.y;
    }

    function getZ(){
        return currentGesture.z
    }
    function getR(){
        return currentGesture.r;
    }

    function getTheta(){
        return currentGesture.theta;
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
    {minAngle: 0,   maxAngle: 180,  currentAngle: 90, centerAngle: 90, servo: ServoPin.S4,  direction: Direction.clockwise}
  ];

    //使用反向动力学的方式让爪子移动到指定位置 
    function goDirectlyTo(x: number, y: number, z: number) {
        let solveResult = iksolve(x, y, z);
        if(solveResult.success){
            moveToAngle(MearmServo.Base, solveResult.a0);
            moveToAngle(MearmServo.Right, solveResult.a1);
            moveToAngle(MearmServo.Left, solveResult.a2);
            currentGesture.x=x;
            currentGesture.y=y;
            currentGesture.z=z;
        }
    }

    //让爪子平滑移动到指定位置 
    function gotoPoint(x: number, y: number, z: number) {
        //起始点 - 当前点
        let x0 = currentGesture.x;
        let y0 = currentGesture.y;
        let z0 = currentGesture.z;
        let dist = Math.sqrt((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y) + (z0 - z) * (z0 - z));
        let step = 10;
        for (let i = 0; i < dist; i += step) {
            goDirectlyTo(x0 + (x - x0) * i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist);
            basic.pause(50);
        }
        goDirectlyTo(x, y, z);
        basic.pause(50);
    }

    function gotoPointCylinder(theta: number, r: number, z: number){
        let v=polarToCartesian(theta, r);        
        gotoPoint(v.x, v.y, z);
    }

    function goDirectlyToCylinder(theta: number, r: number,z: number){
        let v=polarToCartesian(theta, r);
        goDirectlyTo(v.x, v.y, z);
    }

    //检查机械爪是否可以到达指定点
    function isReachable(x: number, y: number, z: number) {
        return (iksolve(x, y, z).success);
    }

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
  //% blockId=config_servo block="配置 %servo=MearmServo 使用引脚 %pin=ServoPin|最小角度 %minAgl 最大角度 %maxAgl|初始角度 %defaultAngle 方向 %dir=Direction"
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
  //% blockId=move_by block="控制|%servo=MearmServo|转动|%angle|度"
  export function moveByAngle(servo: MearmServo, angle: number){
    setServoAngle(servo, servos[servo].currentAngle + angle);
  }

  /**
   * 让所有舵机转到初始位置
   */
  //% weight=80
  //% blockGap=10
  //% blockId=reset_all block="所有舵机转初始位置"
  export function resetAllServos(){
    moveToCentre(MearmServo.Base);
    moveToCentre(MearmServo.Right);
    moveToCentre(MearmServo.Left);
    moveToCentre(MearmServo.Grip);
  }
  
    /**
     * 执行舵机测试动作序列
     */
    //% weight=10
    //% blockGap=10
    //% blockId=test_sequences block="执行自动测试"
    export function testSequences(repeat:Number) {
        resetAllServos();
        basic.pause(2000);
        for(let r=0;r<repeat;r++){
            moveToAngle(MearmServo.Base,30);
            basic.pause(500);
            moveToAngle(MearmServo.Right, 30);
            basic.pause(500);
            moveToAngle(MearmServo.Left, 30);
            basic.pause(500);
            moveToAngle(MearmServo.Grip, 30);
            basic.pause(2000);
            moveToAngle(MearmServo.Base, 150);
            basic.pause(500);
            moveToAngle(MearmServo.Right, 150);
            basic.pause(500);
            moveToAngle(MearmServo.Left, 150);
            basic.pause(500);
            moveToAngle(MearmServo.Grip, 150);
            basic.pause(2000);
            resetAllServos();
            basic.pause(1000);
        }
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
    /**
     * 使用反向动力学控制机械爪移动
     */
    //% weight=30
    //% blockGap=10
    //% blockId=move_grip_to block="机械爪移动到位置:x|%x|y|%y|z|%y|"
    export function moveGripTo(x: number, y: number, z: number) {
        gotoPoint(x,y,z);
    }

}
