开源机械臂MeArm的MakeCode扩展 
============================

本扩展可以使用PCA9685芯片的Micro:bit扩展板控制MeArm机械臂

## API 说明

* **moveToAngle** - 转动指定舵机到一个绝对值角度

```
  mearm.moveToAngle(MearmServo.Base, 90);
```

* **moveByAngle** - 让指定舵机转动一个角度 (角度可以是正数或负数，代表不同的方向)

```
  mearm.moveByAngle(MearmServo.Base, -5);
```

* **openGrip** - 打开机械爪

```
  mearm.openGrip();
```

* **closeGrip** - 合拢机械爪

```
  mearm.closeGrip();
```

### 舵机的定义

 * **MearmServo.Base** - 基座底部舵机
 * **MearmServo.Right** - 基座上右边的舵机
 * **MearmServo.Left** - 基座上左边的舵机
 * **MearmServo.Grip** - 机械爪的舵机

## License

MIT

## Supported targets

 * for PXT/microbit

```package
mearm-microbit=github:skyerider/pxt-generic-mearm
```
